"""Extraction of commonsense facts for graph completion from the Quasimodo resource"""

from pattern.text.en import singularize
import json

from utils import get_csv_data

tgt_objects = ['book', 'fire door','safety sign', 'fire extinguisher', 'chair', 'trash can', 'kettle',
               'electric heater', 'desk', 'power socket', 'plug', 'person', 'radiator']

tgt_objects_single = [s.split(' ') for s in tgt_objects]


tgt_rel =['be made of','be made from', 'cause', 'need','has_property', 'has_effect',
          'be important in', 'be useful in', 'get']

tgt_properties= ['ignition','flammable', 'combustible', 'hazard', 'dangerous', 'harmful', 'unsafe']

#Add materials from shapenet (path under args.material_src)

def extract_csk(args):

    commonsense_facts = {k: {} for k in tgt_objects}

    """Extract materials from ShapeNet"""
    mat_gen = get_csv_data(args.material_src, delim=",")
    # skip header
    shpheader = next(mat_gen)
    print("ShapeNet header:")
    print(shpheader)

    entities_tocheck = []
    for row in mat_gen:
        kw = row[0].lower()
        match_idx = [n for n, subl in enumerate(tgt_objects_single) if kw in subl]
        if len(match_idx) > 0:
            tgt_k = tgt_objects[match_idx[0]]# matching string will be the considered object

            if 'madeof' not in commonsense_facts[tgt_k].keys():
                commonsense_facts[tgt_k]['madeof']=[]

            # print((row[1].lower(), str(float(row[2])*100)+'%'))
            commonsense_facts[tgt_k]['madeof'].append((row[1], str(float(row[2])*100)+'%')) # append material and ratio of material
            # in quasimodo the tuples are property (can be also a material) and score instead
            # Add shapenet material also to list of entities to be checked against quasimodo later
            # e.g., if paper is found check if it is flammable
            entities_tocheck.append(row[1].lower())

    """Extract commonsense facts from Quasimodo"""
    tsv_gen = get_csv_data(args.quasi_src, delim="\t")
    header = next(tsv_gen)
    print("Quasimodo header:")
    print(header)

    incomplete_rows = 0

    for row in tsv_gen:
        try:
            sub, pred, obj_, modality, is_negative, score, sentence_score, typicality, saliency = row
            sub = singularize(sub)  # singularize for linking entities
            # obj_ = obj_.split()
            score = float(score)
            typicality = float(typicality)
            saliency = float(saliency)
        except ValueError:
            print("skip noisy row")
            print(row)
            incomplete_rows += 1
            continue

        # look for matches to graph nodes
        if sub in tgt_objects and pred in tgt_rel and score >= args.quasi_t:  # and len(obj_)==1:
            # obj_ = obj_[0]
            if pred == 'has_property' and obj_ not in tgt_properties:
                continue

            if pred in ['be made of','be made from']:
                pred = 'madeof' # so it is aggregated with materials added previously from Shapenet
            # print(' '.join([sub, pred, obj_, str(score), str(typicality), str(saliency)]))
            obj_ = singularize(obj_)
            entities_tocheck.append(obj_)
            if pred not in commonsense_facts[sub].keys():
                commonsense_facts[sub][pred] = []
            commonsense_facts[sub][pred].append((obj_, score))

    entities_tocheck = list(set(entities_tocheck))  # remove duplicates before level 2 query
    print("No of incomplete rows %i" % incomplete_rows)
    print("Second iter to check for entities to expand")

    tsv_gen = get_csv_data(args.quasi_src, delim="\t")
    header = next(tsv_gen)
    for row in tsv_gen:
        try:
            sub, pred, obj_, modality, is_negative, score, sentence_score, typicality, saliency = row
            sub = singularize(sub)
            # obj_ = obj_.split()
            score = float(score)
            # typicality = float(typicality)
            # saliency = float(saliency)
        except ValueError:
            print("skip noisy row")
            continue

        # expand with concepts linked to graph entities (level 1)
        if sub in entities_tocheck and pred in tgt_rel and score >= args.quasi_t:  # and len(obj_)==1:
            # obj_ = obj_[0]
            if pred == 'has_property' and obj_ not in tgt_properties:
                continue
            # print(' '.join([sub, pred, obj_, str(score), str(typicality), str(saliency)]))

            if pred in ['be made of','be made from']:
                pred = 'madeof'

            if sub not in commonsense_facts.keys():
                commonsense_facts[sub] = {}

            if pred not in commonsense_facts[sub].keys():
                commonsense_facts[sub][pred] = []
            commonsense_facts[sub][pred].append((obj_, score))

    print("Dictionary of extracted commonsense facts created!")
    with open(args.quasikb_path, 'w') as fp:
        json.dump(commonsense_facts, fp)
    print("and saved locally")

    return commonsense_facts


"""ConceptNet"""
"""import requests

for objclass in tgt_objects:
    relation = "/r/MadeOf"
    obj = requests.get('http://api.conceptnet.io/c/en/'+objclass).json()

    for edge in obj['edges']:
        print(edge)
        \"""extract made of relations
        if edge['rel']['@id'] == relation:
            print(objclass + relation)
            print(edge['end']['@id'])\"""
"""
"""DoQ - Elazar et al., 2019

format per entry: 
['obj', 'head', 'dim', 'mean', 'perc_5', 'perc_25', 'median', 'perc_75', 'perc_95', 'std'] where perc_5 means 5th percentile of measurements


Units:
- Speed is meter per hour
- Mass is grams
- Duration is seconds
- Temperature is Kelvin (for some reason.. ;)
- Volume is cubes
- Length is meters
- Currency is dollars
"""
"""
import csv
from collections import Counter
filename = './data/DoQ_raw_nouns_noun_obj_quantization_dist10.csv'

prop_dict =Counter()
with open(filename) as file:

    csvreader = csv.reader(file,delimiter='\t')
    rows = []
    for row in csvreader:
        prop = row[2]
        if prop not in prop_dict.keys():
            prop_dict[prop] = 1
        else:
            prop_dict[prop] += 1

"""
""" TransOMCS """

"""
filename = './data/TransOMCS_full.txt'

print("Reading TransOMCS..it may take long")

obj_dict = {}

with open(filename) as file:

    for line in file:

        line = line.rstrip()
        sub, rel, obj, score = line.split('\t')

        if sub =='book' and rel=='MadeOf' and obj=='paper':
            print(line)
            continue

        if (sub=='flammable'):
            if not sub in obj_dict.keys():
                obj_dict[sub] = {}
            try:
                obj_dict[sub][rel].append((obj,score))
            except KeyError:
                obj_dict[sub][rel] = []
                obj_dict[sub][rel].append((obj, score))

print("TransOMCS loaded as dictionary!")
"""

