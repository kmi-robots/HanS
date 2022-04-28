# Code to prep size background knowledge given a set of classes as input
# Relies on measures found from ShapeNet, Web-scraped or hardcoded, all to be provided as raw csv files

import json
import os
import re
from functools import reduce
import operator
import itertools
from sklearn.neighbors import LocalOutlierFactor
import numpy as np
import statistics
import difflib

import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams.update({'font.size': 8})

from utils import get_csv_data

#Target qualitative bins X = object surface area, Y= object thickness
Xlabels = ['XS','small','medium','large','XL']
Ylabels = ['flat','thin','thick','bulky']
all_bins = []
for al in Xlabels:
    for dl in Ylabels:
        all_bins.append(al+"-"+dl)
body_bins = ['large-thin','large-thick','large-bulky','XL-thin','XL-thick',"XL-bulky"]

def extract_size_kb(classdict, args):

    size_catalog = {}

    print("Retrieving raw size measurements from sources")
    shp_gen = get_csv_data(args.size_src)
    hardcoded_gen = get_csv_data(args.hardcoded_size_src)

    tgt_classes = [cl.replace("_", " ") for cl in list(classdict.values())]

    # populate with ShapeNet data first
    size_catalog = dict_from_csv(shp_gen, tgt_classes, size_catalog)
    # add web-scraped measures
    scraped_files = [os.path.join(args.scraped_size_src, fname) for fname in os.listdir(args.scraped_size_src) if
                     fname.endswith('csv')]
    size_catalog = integrate_scraped(size_catalog, scraped_files)
    remainder = [c for c in tgt_classes if c not in size_catalog.keys()]

    hcsv_gen, ar_gen = itertools.tee(hardcoded_gen)
    size_catalog = add_hardcoded(size_catalog, remainder, hcsv_gen, tolerance=args.sizetol)
    size_catalog = add_ARflat_hardcoded(size_catalog, ar_gen)

    #Add manual annotations for flat/non-flat
    try:  # load flat/no-flat from existing json, if any is found
        with open(args.flat_src, 'r') as fin:
            backup = json.load(fin)
            for k in size_catalog:
                size_catalog[k]['is_flat'] = backup[k]['is_flat']
    except:
        pass

    print("Sorting objects by relative size")
    #LOF outlier removal
    size_catalog = remove_outliers(size_catalog)

    #Automated object sorting through histograms
    sorted_res = object_sorting(size_catalog)
    size_catalog, aTs, dTs = bin_creation(size_catalog, sorted_res)

    # Validate autom annotation with flat/no-flat annotations collected manually
    size_catalog = valid_adjustments(size_catalog)

    # Print edge params, that can be then used to setup the reasoning params
    print("The logarithmic area thresholds used for bin creation were")
    print(("Config 1 %s") % str(aTs[0]))
    print(("Config 2 %s") % str(aTs[1]))
    print(("Config 3 %s") % str(aTs[2]))
    print("The logarithmic depth thresholds used for bin creation were")
    print(("Config 1 %s") % str(dTs[0]))  # print(("Config 1 %s") % (str(dthresh_config1)+str(dTs[0][1:]))) ###
    print(("Config 2 %s") % str(dTs[1]))
    print(("Config 3 %s") % str(dTs[2]))

    with open(args.sizekb_path, 'w') as fout: #save KB locally as json
        json.dump(size_catalog, fout)

    return size_catalog


def bin_creation(obj_dict,input_sorts,n_areabins=len(Xlabels),n_depthbins=len(Ylabels), C=3):
    """
    Create object groups/quadrants based on area surface v depth sorting
    Splits up groups automatically to keep histogram bins equidistributed
    Expects dims of a N x M (e.g.,5x4) grid to be given, depending on how many bins one wants to create
    """
    area_thresholds, depth_thresholds = [],[]
    fig, axes = plt.subplots(nrows=1,ncols=3,figsize=(7,3))#, constrained_layout=True)
    for config in range(C): # 3 configurations
        areas, depths = input_sorts[config], input_sorts[config+3]
        areas_ar, depths_ar = np.asarray(list(zip(*areas))[1]), np.asarray(list(zip(*depths))[1])
        areas_ar, depths_ar = np.log(areas_ar),np.log(depths_ar)
        H, xedges, yedges = np.histogram2d(areas_ar,depths_ar,bins=[n_areabins,n_depthbins])
        H = H.T # Let each row list bins with common y range.

        # Plot 2D histogram
        ax = axes[config]
        #im = ax.imshow(H, interpolation='nearest', origin='low',extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]])
        #fig.colorbar(im, ax=ax, fraction=0.046, pad=0.05)
        #ax.set_aspect('equal')
        ax.set_yticklabels([])
        ax.set_xticklabels([])
        if config ==1: xttl,yttl ='Mean area (d1*d2)', 'Mean thickness (d3)'
        elif config ==2: xttl,yttl ='Mean area (d1*d3)', 'Mean thickness (d2)'
        else: xttl,yttl ='Mean area (d2*d3)', 'Mean thickness (d1)'
        ax.set_xlabel(xttl)
        ax.set_ylabel(yttl)
        X, Y = np.meshgrid(xedges, yedges)
        pcm = ax.pcolormesh(X,Y,H)
        fig.colorbar(pcm, ax=ax, fraction=0.046, pad=0.05)
        #cb.set_label('counts in bin')
        #set aspect based on axis limits to obtained three scaled subplots
        asp = np.diff(ax.get_xlim())[0] / np.diff(ax.get_ylim())[0]
        ax.set_aspect(asp)

        #Keep track of thresholds used for bin creation
        xedges_log,yedges_log = xedges.tolist().copy(),yedges.tolist().copy()
        area_thresholds.append(xedges_log[1:len(xedges)-1])
        depth_thresholds.append(yedges_log[1:len(yedges)-1])
        #Area bin memberships
        xindices = np.digitize(areas_ar,xedges)
        yindices = np.digitize(depths_ar,yedges)
        for j,(obj1, areav) in enumerate(areas):
            idepth = [i for i,(o,v) in enumerate(depths) if o == obj1][0]
            areabin,depthbin = xindices[j],yindices[idepth]
            #adjust exceeding values
            if areabin > n_areabins: areabin = n_areabins
            if depthbin > n_depthbins: depthbin = n_depthbins
            #print("%s is %s and %s" % (obj1,Xlabels[areabin-1],Ylabels[depthbin-1]))
            #update KB
            try:
                obj_dict[obj1]["has_size"].append('-'.join((Xlabels[areabin-1],Ylabels[depthbin-1])))
            except KeyError:
                obj_dict[obj1]["has_size"] = []
                obj_dict[obj1]["has_size"].append('-'.join((Xlabels[areabin - 1], Ylabels[depthbin - 1])))

    plt.tight_layout()
    plt.show()

    #After all configurations are considered, remove area-depth combination duplicates
    for k in obj_dict.keys():
        size_list = obj_dict[k]["has_size"]
        obj_dict[k]["has_size"] = list(set(size_list))

    return obj_dict,area_thresholds,depth_thresholds

def valid_adjustments(obj_dict):
    """
    Validate the automatically-generated bins with manually collected
    knowledge of flat/non-flat objects
    """
    for k in obj_dict.keys():
        flat_only = False
        if k =='box' or k=='power cord':
            # extreme cases, all bin combinations are possible
            obj_dict[k]['has_size']= all_bins
            continue
        elif k =='person':
            #not enough measures to model people sizes, understimates
            obj_dict[k]['has_size'] = body_bins
            continue
        if str(True) in str(obj_dict[k]["is_flat"]) and not str(False) in str(obj_dict[k]["is_flat"]):
            # if object marked as striclty flat, validate autom generated thinness
            bins_ = [tuple(t.split('-')) for t in obj_dict[k]['has_size']]
            nbins = list(set([ a+'-'+'flat' for a,d in bins_]))
            obj_dict[k]['has_size'] = nbins
            flat_only = True
        elif str(True) in str(obj_dict[k]["is_flat"]) and str(False) in str(obj_dict[k]["is_flat"])\
            and "flat" not in str(obj_dict[k]["has_size"]):
            # can be flat or not, but it was not annotated as flat automatically
            nbins = list(set([s.split("-")[0] for s in obj_dict[k]['has_size']]))
            obj_dict[k]['has_size'].extend([s+"-flat" for s in nbins])
        elif not str(True) in str(obj_dict[k]["is_flat"]) and str(False) in str(obj_dict[k]["is_flat"])\
            and "flat" in str(obj_dict[k]["has_size"]):
            #conversely, object is strictly non-flat, but was marked as flat automatically
            # replace "flat" cases with "thin" instead
            nbins = []
            for s in obj_dict[k]['has_size']:
                if "flat" in s:
                    nbins.append(s.split("-")[0]+"-thin")
                else: nbins.append(s)
            obj_dict[k]['has_size'] = list(set(nbins))

        #all cases: fill gaps on X axis (e.g., object marked as both medium and XL but not large
        as_ = list(set([c.split('-')[0] for c in obj_dict[k]['has_size']]))
        if len(as_)>1:
            ar_indices = [Xlabels.index(a) for a in as_]
            mina,maxa = min(ar_indices),max(ar_indices)
            all_indices = list(range(mina,maxa+1))
            missing_indices = [ind for ind in all_indices if ind not in ar_indices]
            if len(missing_indices)> 0:
                tgt_thick = [c.split('-')[1] for c in obj_dict[k]['has_size'] if c.split('-')[0]==Xlabels[mina]][0] #thickness of lowest area bin
                obj_dict[k]['has_size'].extend([Xlabels[ind]+'-'+tgt_thick for ind in all_indices])

        if not flat_only:
            # fill gaps in between flat and max thinness at a certain area value
            as_ = list(set([c.split('-')[0] for c in obj_dict[k]['has_size']]))
            for a in as_:
                same_clus = [c.split('-')[1] for c in obj_dict[k]['has_size'] if c.split('-')[0]==a]
                if len(same_clus)> 1: # there is at least another bin of same qual area
                    #check if there are gaps to fill in terms of thickness
                    thick_indices = [Ylabels.index(t) for t in same_clus]
                    mint, maxt = min(thick_indices), max(thick_indices)
                    obj_dict[k]['has_size'].extend([a+'-'+Ylabels[k] for k in range(mint+1,maxt)])
            nb = obj_dict[k]['has_size'] #finally, remove dups if any
            obj_dict[k]['has_size'] = list(set(nb))

    return obj_dict


def object_sorting(KB):
    areas1, areas2, areas3 = [], [], []
    depths1, depths2, depths3 = [], [], []
    keyword = 'dims_cm'
    for k in KB.keys():
        measurements = KB[k][keyword]  # array of measurements
        if str(True) in str(KB[k]["is_flat"]) and not str(False) in str(KB[k]["is_flat"]):
            # if object marked as striclty flat, we can assume depth is the minimum
            # i.e., only one configuration
            all_depths, all_areas = [], []
            for dims in measurements:
                # avoid near-zero values
                for j, d in enumerate(dims):
                    if d < 1.:  # in cm
                        dims[j] = 1.
                sdims = [float(d / 100) for d in dims]  # convert cm to meters,i.e., as in arc case
                all_depths.append(min(sdims))
                sdims.remove(min(sdims))
                all_areas.append(np.prod(sdims))
            # add mean to class-wise aggregated list
            depths1.append((k, statistics.mean(all_depths)))
            areas1.append((k, statistics.mean(all_areas)))

        else:  # we cannot univoquely map any of the dimensions to w,h,d
            # exploring 3 configurations as in the ARC case
            d1s, d2s, d3s, a1s, a2s, a3s = [], [], [], [], [], []
            for dims in measurements:
                # avoid near-zero values
                for j, d in enumerate(dims):
                    if d < 1.:  # in cm
                        dims[j] = 1.
                d1, d2, d3 = dims
                d1 = float(d1 / 100)
                d2 = float(d2 / 100)
                d3 = float(d3 / 100)
                d1s.append(d1)  # d1 is the depth
                d2s.append(d2)  # d2 is the depth, etc.
                d3s.append(d3)
                a1s.append(d2 * d3)
                a2s.append(d1 * d3)
                a3s.append(d1 * d2)

            depths1.append((k, statistics.mean(d1s)))
            depths2.append((k, statistics.mean(d2s)))
            depths3.append((k, statistics.mean(d3s)))
            areas1.append((k, statistics.mean(a1s)))
            areas2.append((k, statistics.mean(a2s)))
            areas3.append((k, statistics.mean(a3s)))

    # Sort (ascending)
    areas1 = list(sorted(areas1, key=lambda x: x[1]))
    areas2 = list(sorted(areas2, key=lambda x: x[1]))
    areas3 = list(sorted(areas3, key=lambda x: x[1]))
    depths1 = list(sorted(depths1, key=lambda x: x[1]))
    depths2 = list(sorted(depths2, key=lambda x: x[1]))
    depths3 = list(sorted(depths3, key=lambda x: x[1]))

    # OPTIONAL: create output txt to visually inspect sorted results
    with open('./data/obj_sorting.txt', 'w') as fout:
        fout.write("Sorted by surface area - ascending (config1)\n")
        fout.write('\n'.join('{} {}'.format(x[0], x[1]) for x in areas1))
        fout.write("========== \n")
        fout.write("Sorted by surface area - ascending (config2)\n")
        fout.write('\n'.join('{} {}'.format(x[0], x[1]) for x in areas2))
        fout.write("========== \n")
        fout.write("Sorted by surface area - ascending (config3)\n")
        fout.write('\n'.join('{} {}'.format(x[0], x[1]) for x in areas3))
        fout.write("========== \n")
        fout.write("Sorted by depth value - ascending (config1)\n")
        fout.write('\n'.join('{} {}'.format(x[0], x[1]) for x in depths1))
        fout.write("========== \n")
        fout.write("Sorted by depth value - ascending (config2)\n")
        fout.write('\n'.join('{} {}'.format(x[0], x[1]) for x in depths2))
        fout.write("========== \n")
        fout.write("Sorted by depth value - ascending (config3)\n")
        fout.write('\n'.join('{} {}'.format(x[0], x[1]) for x in depths3))
        fout.write("========== \n")

    return [areas1, areas2, areas3, depths1, depths2, depths3]


def remove_outliers(obj_dict):
    clf = LocalOutlierFactor(n_neighbors=2,metric='euclidean',n_jobs=-1)
    for k in obj_dict.keys():
        measurements = obj_dict[k]["dims_cm"]  # array of measurements
        try:
            is_inlier = clf.fit_predict(measurements)
            outls = [dims for i,dims in enumerate(measurements) if is_inlier[i]==-1]
            cleaned_measures = [dims for i,dims in enumerate(measurements) if is_inlier[i]==1]
            obj_dict[k]["dims_cm"] = cleaned_measures
        except ValueError:
            #too few examples for outlier removal based on NN
            continue #skip
    return obj_dict



def add_ARflat_hardcoded(obj_dict, ref_csv):
    for i,row in enumerate(ref_csv):
        if i ==0: continue
        obj_name = row[0].replace("_", " ")
        prop = row[9]
        rates = prop.split('/')
        if len(rates) > 1:
            obj_dict[obj_name]['aspect_ratio'] = rates
        else:
            obj_dict[obj_name]['aspect_ratio'] = rates[0]
        flat = row[10]
        if "-" in flat:obj_dict[obj_name]['is_flat'] = [True, False]
        elif '0' in flat:obj_dict[obj_name]['is_flat'] = False
        elif '1' in flat: obj_dict[obj_name]['is_flat'] = True
    return obj_dict

def add_hardcoded(obj_dict, bespoke_list, ref_csv, tolerance= 0.05): #5% of obj dim
    """
    # Add hardcoded entries
    # overwrites ShapeNet if class present in both (more accurate info)
    returns: object dictionary with the addition of hardcoded values
    """
    for i,row in enumerate(ref_csv):
        if i ==0: continue
        obj_name = row[0].replace("_", " ")

        if row[1] != '':
            dims_min = [float(v) for v in row[1:4]]
            dims_max = [float(v) for v in row[4:7]]
        elif row[1]=='' and row[4] !='':
            dims = [float(v) for v in row[4:7]]
            dims_min = [(d - tolerance * d) for d in dims]  # min-max range of dims
            dims_max = [(d + tolerance * d) for d in dims]
        if obj_name in bespoke_list: # update dictionary only for give set of objects
            try:
                obj_dict[obj_name]['dims_cm'] = [dims_min, dims_max]
            except KeyError:
                obj_dict[obj_name]={}
                obj_dict[obj_name]['dims_cm'] = [dims_min, dims_max]
            vol_min, vol_max = reduce(operator.mul, dims_min, 1), reduce(operator.mul, dims_max, 1)
            obj_dict[obj_name]['volume_cm3'] = [vol_min, vol_max]
            obj_dict[obj_name]['volume_m3'] = [float(vol_min / 10 ** 6), float(vol_max / 10 ** 6)]

    return obj_dict



def dict_from_csv(csv_gen, classes, base):

    """Extract dimensions from the shapenet raw csv data"""
    for h,row in enumerate(csv_gen):
        if h==0: continue #skip header
        obj_name = row[3]
        super_class = row[1]
        # find all class names matching row keywords
        tgts = []
        for cat in classes:
            if cat == obj_name or cat in obj_name: #exact or partial match
                if (not "piano" in obj_name) \
                    and (not "lamppost" in obj_name) \
                    and (not (cat == 'chair' and "armchair" in obj_name)):
                    tgts.append((cat,obj_name))
            #elif cat in super_class.lower():
            #    tgts.append((cat, super_class.lower()))
            elif (cat == 'sofa' and "armchair" in obj_name) \
                or (cat == 'cupboard' and "cabinet" in obj_name) \
                or (cat == 'big screen' and "tv" in obj_name) \
                or (cat == 'plant vase' and "vase" in obj_name) \
                or (cat == 'rubbish bin' and ("trash" in obj_name or "waste" in obj_name) ) \
                or (cat == 'headphones' and "earphone" in obj_name) \
                or (cat == 'desk' and "table" in obj_name):
                tgts.append((cat, obj_name))
            elif (cat == 'sofa' and "couch" in super_class.lower()) \
                or (cat =='whiteboard' and super_class=='Whiteboard')\
                or (cat == 'wallpaper' and "WallArt" in super_class) \
                or ('food' in cat and "FoodItem" in super_class):
                tgts.append((cat, super_class.lower()))
        if len(tgts)>0:
            if len(tgts)==1: #only one match available
                kw = tgts[0][0]
            else: #more than one match available
                #prefer exact match first
                exacts = [match for cl,match in tgts if cl==match]
                if len(exacts)>0:
                    kw = exacts[0]
                else:
                    # partial match
                    # assume it is a compound word - pick last token
                    #print("Compound word in %s" % str(tgts))
                    kw = tgts[0][0].split(' ')[-1]

            cat = difflib.get_close_matches(kw, classes)[0] #closest match to keyword kw among target classes
            try:
                base[cat]['dims_cm'].append([float(dim) for dim in row[7].split('\,')])
            except: #first time object is added to the dictionary
                base[cat] = {}
                base[cat]['dims_cm'] = []
                base[cat]['volume_cm3'] = []
                base[cat]['volume_m3'] = []
                base[cat]['dims_cm'].append([float(dim) for dim in row[7].split('\,')])

            vol = reduce(operator.mul, [float(dim) for dim in row[7].split('\,')], 1)
            base[cat]['volume_cm3'].append(vol)
            base[cat]['volume_m3'].append(float(vol / 10 ** 6))


    return base


def integrate_scraped(obj_dict, path_to_csvs,blacklist=[]):

    for csvp in path_to_csvs:
        scrap_gen = get_csv_data(csvp)
        for i,row in enumerate(scrap_gen):
            if i == 0: continue #skip header
            obj_name = row[0].replace('_', ' ')
            base = row[1].replace(' ','')
            unit = base[-2:]
            if unit == 'cm' or unit =='mm':
                dimensions = base[:-2]
                if 'x' in dimensions:
                    num_dims = len([d for d in dimensions.split('x')])
                    # if it contains any other letters, remove them
                    dim_list = [float(''.join([c if c.isdigit() else '' for c in d])) \
                                    if re.search('[a-zA-Z]', d) is not None else float(d) \
                                for d in dimensions.split('x')]

                elif ',' in dimensions:
                    num_dims = len([d for d in dimensions.split(',')])
                    dim_list = [float(''.join([c if c.isdigit() else '' for c in d])) \
                                    if re.search('[a-zA-Z]', d) is not None else float(d) \
                                for d in dimensions.split(',')]

                if num_dims < 3: #here happens for signs of paper with non-significant depth
                    dim_list.append(0.25)#add depth without compromising volume too much
                if unit=='mm': #convert to cm first
                    dim_list= [float(d/ 10) for d in dim_list]
                if obj_name not in blacklist: #in remainder_list and obj_name not in blacklist:
                    # overwrites SHP with Amazon if executed after the shapenet extraction
                    try:
                        obj_dict[obj_name]['dims_cm'].append(dim_list)
                    except:
                        obj_dict[obj_name] = {}
                        obj_dict[obj_name]['dims_cm'] = []
                        obj_dict[obj_name]['volume_cm3'] = []
                        obj_dict[obj_name]['volume_m3'] = []
                        obj_dict[obj_name]['dims_cm'].append(dim_list)

                    vol = reduce(operator.mul, dim_list, 1)
                    obj_dict[obj_name]['volume_cm3'].append(vol)
                    obj_dict[obj_name]['volume_m3'].append(float(vol / 10 ** 6))

    return obj_dict
