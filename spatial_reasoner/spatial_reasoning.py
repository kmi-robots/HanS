"""Methods used for spatial reasoning on a Postgresql db"""
from collections import OrderedDict
import networkx as nx
from utils import plot_graph
import itertools
import statistics

from params import get_parser
from pycoral.utils.dataset import read_label_file
from DL.ranking_aggregation import merge_scored_ranks

parser = get_parser()
args_dict, unknown = parser.parse_known_args()
target_classes = read_label_file(args_dict.classes)
class_map = dict((v, k) for k, v in target_classes.items())  # swap keys with indices


def build_QSR_graph(conn, cur, anchors, args):

    session = (conn,cur)
    QSRs = nx.MultiDiGraph()

    ids_ord = order_by_volume((conn,cur), list(anchors.keys()))
    QSRs.add_nodes_from(ids_ord.keys()) # one node per anchor
    nx.set_node_attributes(QSRs, "", "obj_label")  # init field for DL prediction for later use
    nx.set_node_attributes(QSRs, 0, "obj_volume")  # init field for DL prediction for later use

    for (o_id, o_volume) in ids_ord.items():

        QSRs.nodes[o_id]["obj_volume"] = o_volume # keep track of obj volume in graph, used later for rule check
        figure_objs = find_neighbours(session, o_id, ids_ord, dis=args.dis)
        if len(figure_objs) > 0:
            QSRs = extract_QSR(session, o_id, figure_objs, QSRs, int_perc=args.int_perc)  # relations between objects
            # plot_graph(QSRs)

        #Changed to extract_surface_QSR (no precalc) + new walls
        QSRs = extract_surface_QSR(session, o_id, QSRs, fht=args.fht, wht=args.wht)  # relations with walls and floor

    # after all reference objects have been examined
    # derive special cases of ON
    QSRs = infer_special_ON(QSRs)

    # add relations with map areas, e.g., wast area, fire escape area
    # Note: added at the end so containment in abstract regions do not interfere with physical checks
    areas_of_interest = retrieve_AOI((conn, cur))
    for o_id in ids_ord.keys():
        QSRs = extract_specialarea_QSR(session, o_id, areas_of_interest, QSRs)

    # QSRs = remove_redundant_qsrs(QSRs) # used to evaluate the QSR to make sure they were not duplicated but for VG best to keep synonyms e.g., both LeftOf and beside

    #to visualize extracted QSR graph
    #plot_graph(QSRs)

    return QSRs


def spatial_validate(node_id, DLrank, sizerank, QSRs, KB, taxonomy, meta='waterfall'):

    """If meta=waterfall, validates sizerank
    Both reasoners applied in cascade.
    If meta=parallel, checks DLrank and sizerank separately and combines both by majority voting
    Assumes in this case that inputs are ranked by descending confidence and uses typicality scores,
    to be changed to atypicality in the case inputs are ranked by ascending distance
    ML predictions are used as spatial labels, i.e., no ground truth, as this code is intended for live use
    Only ML predictions above confidence threshold dlconf are considered
    """
    dlclasses, dlscores = zip(*DLrank)

    if target_classes[dlclasses[0]] =='person': #if top pred is a person, skip spatial validation
        print("Recognised as person, cannot determine typical locations")
        return sizerank

    if meta =='waterfall':
        input_rankings = [sizerank] # only size-validated DL ranking (rankings modified on cascade)
    elif meta == 'parallel':
        input_rankings = [DLrank, sizerank] # DL and size-validated ranking are input in parallel

    out_ranks = []
    for inputr in input_rankings:
        spacerank = inputr.copy() # starting by size validated ranking
        spacecls, spacescores = zip(*spacerank)
        spacecls, spacescores = list(spacecls), list(spacescores)
        for i, (cid, score) in enumerate(inputr):
            pred_label = target_classes[cid]
            wn_syn = taxonomy[pred_label] #wordnet synsets associated with label

            fig_qsrs = [(pred_label, QSRs.nodes[ref]["obj_label"], r['QSR'])
                        for f, ref, r in QSRs.out_edges(node_id, data=True) if ref in class_map.keys()
                        and dlscores[dlclasses.index(class_map[QSRs.nodes[ref]["obj_label"]])] >= args_dict.dlconf]  # rels where obj is figure
            ref_qsrs = [(QSRs.nodes[f]["obj_label"], pred_label, r['QSR'])
                        for f, ref, r in QSRs.in_edges(node_id, data=True) if f in class_map.keys()
                        and dlscores[dlclasses.index(class_map[QSRs.nodes[ref]["obj_label"]])] >= args_dict.dlconf]

            # Retrieve wall and floor QSRs, only in figure/reference form - e.g., 'object onTopOf
            surface_qsrs = [(pred_label, ref, r['QSR']) for f, ref, r \
                            in QSRs.out_edges(node_id, data=True) if ref in ['wall', 'floor']]  # only those in fig/ref form
            fig_qsrs.extend(surface_qsrs)  # merge into list of fig/ref relations

            if not wn_syn or pred_label=='person' or (len(ref_qsrs)==0 and len(fig_qsrs)==0):
                # objects that do not have a mapping to VG through WN (foosball table and pigeon holes)
                # we skip people as they are mobile and can be anywhere, space is not discriminating
                # OR there are no QSRs to consider
                spacescores[i] += 0. #set total score to 0. typicality, so the DL score does not change

            #Find scores for relations where obj is figure
            spatial_scores = modify_with_typicalities(wn_syn, fig_qsrs,taxonomy, KB, relform='figure')
            # Similarly, for QSRs where predicted obj is reference, i.e., object
            spatial_scores = modify_with_typicalities(wn_syn, ref_qsrs, taxonomy, KB, relform='reference', all_spatial_scores=spatial_scores)

            # Average across all QSRs
            if len(spatial_scores)>0:
                avg_spatial_score = statistics.mean(spatial_scores)
                spacescores[i] += avg_spatial_score # add up to ML score

        # Normalise combined scores cross-class so they are between 0 and 1
        min_, max_ = min(spacescores), max(spacescores)
        if max_ - min_ != 0:  # avoid division by zero
            spacescores_norm = [(x - min_) / (max_ - min_) for x in spacescores]
        spacerank_mod = list(zip(spacecls, spacescores_norm))
        # re-sort by descending confidence scores
        spacerank_mod = sorted(spacerank_mod, key=lambda x: (x[1], x[1]), reverse=True)
        out_ranks.append(spacerank_mod)

    if meta =='waterfall':
        return out_ranks[0] # only one output
    elif meta == 'parallel':
        #merge two output rankings by avg score, if score not null,
        # i.e., we reuse the same strategy as when aggregating across observations

        spatialclasses =[]
        out_ranks_valid = []
        for outr_ in out_ranks: #keep only classes of non-zero score after combining DL and spatial typicalities
            spatialclasses.extend([cnum for cnum, jconf in outr_ if jconf > 0.])
            out_ranks_valid.append([(cnum, jconf) for cnum, jconf in outr_ if jconf > 0.])
        """if len(spatialclasses) == 0: #should not happen in this case because at least DL has a score 
            #there were no useful scores through spatial reasoning
            print("Keep size ranking")
            return sizerank"""

        mrank = merge_scored_ranks(out_ranks_valid)

        return mrank


def modify_with_typicalities(in_syns, qsrlist, classtax, spatialdict, relform='figure', all_spatial_scores = []):

    relset = list(set([r for _, _, r in qsrlist]))  # distinct figure relations present
    static_syn = in_syns
    for fig, ref, r in qsrlist:  # for each QSR where obj is figure, i.e., subject
        if relform == 'figure':
            tgt_syn = ref
        elif relform == 'reference':
            tgt_syn = fig

        if tgt_syn == 'wall':
            tgt_syn = ['wall.n.01']  # cases where reference is wall or floor
        elif tgt_syn == 'floor':
            tgt_syn = ['floor.n.01']
        else:
            tgt_syn = classtax[ref]

        if tgt_syn == '' or static_syn=='':  # obj is e.g., foosball table or pigeon holes (absent from background KB)
            all_spatial_scores.append(0.)  # add up 0. as if not found to not alter ML ranking and skip
            continue

        if r == 'touches':
            if len(relset) == 1:  # touches is the only rel
                all_spatial_scores.append(0.)  # add up 0. as if not found to not alter ML ranking and skip
                continue
            else:
                continue  # there are other types, just skip this one as not relevant for VG
        elif r == 'beside':
            continue  # beside already checked through L/R rel
        elif r == 'leansOn' or r == 'affixedOn':
            r = 'against'  # mapping on VG predicate

        if relform == 'figure':
            all_spatial_scores = compute_all_scores(spatialdict, all_spatial_scores, static_syn, tgt_syn, r)
        elif relform == 'reference': # swap sub-obj order in method call
            all_spatial_scores = compute_all_scores(spatialdict, all_spatial_scores, tgt_syn, static_syn, r)

    return all_spatial_scores

def compute_all_scores(spacedict, all_scores, sub_syn, obj_syn, r):

    if len(sub_syn) > 1 or len(obj_syn) > 1:
        if len(sub_syn) > 1 and len(obj_syn) > 1:  # try all sub,obj ordered combos
            # print(list(itertools.product(sub_syn, obj_syn)))
            typscores = [compute_typicality_score(spacedict, sub_s, obj_s, r) \
                         for sub_s, obj_s in list(itertools.product(sub_syn, obj_syn))]
        elif len(sub_syn) == 1 and len(obj_syn) > 1:
            sub_syn = sub_syn[0]
            typscores = [compute_typicality_score(spacedict, sub_syn, osyn, r) for osyn in obj_syn]
        elif len(sub_syn) > 1 and len(obj_syn) == 1:
            obj_syn = obj_syn[0]
            typscores = [compute_typicality_score(spacedict, subs, obj_syn, r) for subs in sub_syn]

        typscores = [s for s in typscores if s != 0.]  # keep only synset that of no-null typicality
        # in order of taxonomy (from preferred synset to least preferred)
        if len(typscores) == 0:
            typscore = 0.
        else:
            typscore = typscores[0]  # first one in the order

    else:
        sub_syn, obj_syn = sub_syn[0], obj_syn[0]
        typscore = compute_typicality_score(spacedict, sub_syn, obj_syn, r)
    all_scores.append(typscore)  # Typicality score comparable with DL confidence scores )
                                # Change to complement to 1 (i.e., atypicality) if DL uses distance scores
    return all_scores

def compute_typicality_score(VGstats, sub_syn, obj_syn, rel, use_beside=False, use_near=False):
    # no of times the two appeared in that relation in VG
    try:
        nom = float(VGstats['predicates'][rel]['relations'][str((str(sub_syn), str(obj_syn)))])
    except KeyError:  # if any hit is found
        if rel != 'above':
            if rel == 'leftOn' or rel == 'rightOf':  # check also hits for (more generic) beside predicate
                try:
                    nom = float(VGstats['predicates']['beside']['relations'][
                                    str((str(sub_syn), str(obj_syn)))])
                    use_beside = True
                except KeyError:
                    return 0.
            else:  # check also hits for (more generic) near predicate
                try:
                    nom = float(
                        VGstats['predicates']['near']['relations'][str((str(sub_syn), str(obj_syn)))])
                    use_near = True
                except KeyError:
                    return 0.
        else:
            return 0.
    # no of times sub_syn was subject of r relation in VG
    try:
        if use_near:
            denom1 = float(VGstats['subjects'][sub_syn]['near'])
        elif use_beside:
            denom1 = float(VGstats['subjects'][sub_syn]['beside'])
        else:
            denom1 = float(VGstats['subjects'][sub_syn][rel])
    except KeyError:  # if any hit is found
        return 0.
    # no of times obj_syn was object of r relation in VG
    try:
        if use_near:
            denom2 = float(VGstats['objects'][obj_syn]['near'])
        elif use_beside:
            denom2 = float(VGstats['objects'][obj_syn]['beside'])
        else:
            denom2 = float(VGstats['objects'][obj_syn][rel])
    except KeyError:  # if any hit is found
        return 0.
    return nom / (denom1 + denom2 - nom)


def remove_redundant_qsrs(qgraph):

    QSRs_fil = qgraph.copy() # remove redundant spatial rels
    for (n1, n2, k, d) in qgraph.edges(data=True,keys=True):
        rel_name = d['QSR']
        print(str(n1)+' '+rel_name+' '+str(n2))
        rem =False
        if rel_name =='touches' or rel_name=='beside' or rel_name=='near':
            QSRs_fil.remove_edge(n1,n2,k)
            rem=True
        if n1 =='wall' or n1=='floor' and not rem:
            QSRs_fil.remove_edge(n1,n2, k) #and not already removed above
            # rem=True

    return QSRs_fil

def order_by_volume(session, akeys):
    #order anchors from larger volume to smaller volume
    tmp_conn, tmp_cur = session
    tmp_cur.execute('SELECT anchor_key, ST_Volume(bbox) as v FROM anchors '\
                    'WHERE anchor_key in %s '\
                    'ORDER BY v DESC', (tuple(akeys),)) #use string pattern matching to check only first part of ID
    # & order objects by volume, descending, to identify reference objects
    res = tmp_cur.fetchall()
    return OrderedDict(res) #preserve ordering in dict #{k[0]: {} for k in res}  # init with obj ids


def find_neighbours(session, ref_id, ordered_objs,dis=2):
    """dis = distance threshold to find neighbours, defaults to 2 units in the SRID of spatial DB"""
    #Find nearby objects which are also smaller in the ordering
    i = list(ordered_objs.keys()).index(ref_id)
    candidates = list(ordered_objs.keys())[i+1:] #candidate figure objects, i.e., smaller
    # Which ones are nearby?
    tmp_conn, tmp_cur = session
    tmp_cur.execute('SELECT anchor_key FROM anchors'\
                    ' WHERE ST_3DDWithin(bbox, '\
                    '(SELECT bbox FROM anchors '\
                    'WHERE anchor_key = %s), %s) '\
                    'AND anchor_key != %s', (ref_id,str(dis),ref_id))

    nearby = [t[0] for t in tmp_cur.fetchall()]
    return [id_ for id_ in nearby if id_ in candidates] # return only the ones which are both nearby and smaller than


def extract_QSR(session, ref_id, figure_objs, qsr_graph, int_perc=0.05):
    """
    Expects qsrs to be collected as nx.MultiDGraph
    D is the space granularity as defined in the paper"""
    tmp_conn, tmp_cur = session
    for figure_id in figure_objs:
        isAbove = False
        touches = False
        if not qsr_graph.has_node(figure_id): #add new node if not already there
            qsr_graph.add_node(figure_id)
        #Use postGIS for deriving truth values of base operators
        # tmp_conn, tmp_cur = connect_DB(us,dbname)
        try:
            tmp_cur.execute('SELECT ST_3DDWithin(fig.bbox,reff.bbox, 0)'
                            ' from anchors as reff, anchors as fig'
                            ' WHERE reff.anchor_key = %s'\
                        ' AND fig.anchor_key = %s', (ref_id,figure_id))
            res = tmp_cur.fetchone()
            if res[0] is True:
                qsr_graph.add_edge(figure_id, ref_id, QSR='touches')
                touches = True
        except Exception as e:
            print("Problem")
            print(str(e))
            print(str(ref_id) + " " + str(figure_id))
        try:

            #Intersection tests are run by checking volume of intersection against thresholds instead of using ST_3DIntersects
            # ST_3DIntersects causes degenerate cases of 1 pixel overlap, changing to thresholds
            tmp_cur.execute(
                'SELECT ST_Volume(ST_3DIntersection(fig.bbox,reff.tophsproj)), '
                'ST_Volume(ST_3DIntersection(fig.bbox,reff.bottomhsproj)), ST_Volume(fig.bbox)'
                ' from anchors as reff, anchors as fig'
                ' WHERE reff.anchor_key = %s' \
                ' AND fig.anchor_key = %s', (ref_id, figure_id))
            res = tmp_cur.fetchone()
            figvol = res[2]
            if res[0] > figvol*int_perc: #intersection is at list x% of volume of figure obj
                qsr_graph.add_edge(figure_id, ref_id, QSR='above')
                isAbove = True
            if res[1] >figvol*int_perc: qsr_graph.add_edge(figure_id, ref_id, QSR='below')
        except:
            print("Problem")
            print(str(e))
            print(str(ref_id) + " " + str(figure_id))
            return qsr_graph

        try:
            tmp_cur.execute('SELECT ST_Volume(ST_3DIntersection(fig.bbox,reff.lefthsproj)),ST_Volume(ST_3DIntersection(fig.bbox,reff.righthsproj)),'
                            'ST_Volume(fig.bbox)'
                            ' from anchors as reff, anchors as fig'
                            ' WHERE reff.anchor_key = %s' \
                            ' AND fig.anchor_key = %s', (ref_id, figure_id))
            res = tmp_cur.fetchone()
            figvol = res[2]
            if res[0] >figvol*int_perc:
                qsr_graph.add_edge(figure_id, ref_id, QSR='leftOf')
                qsr_graph.add_edge(figure_id, ref_id, QSR='beside')

            if res[1]>figvol*int_perc:
                qsr_graph.add_edge(figure_id, ref_id, QSR='rightOf')
                qsr_graph.add_edge(figure_id, ref_id, QSR='beside')

        except Exception as e:
            print("Problem")
            print(str(e))
            print(str(ref_id) + " " + str(figure_id))
            return qsr_graph
        try:
            tmp_cur.execute('SELECT ST_Volume(ST_3DIntersection(fig.bbox,reff.fronthsproj)), ST_Volume(ST_3DIntersection(fig.bbox,reff.backhsproj)),'
                            'ST_Volume(fig.bbox)'
                            ' from anchors as reff, anchors as fig'
                            ' WHERE reff.anchor_key = %s' \
                            ' AND fig.anchor_key = %s', (ref_id, figure_id))
            res = tmp_cur.fetchone()
            figvol = res[2]
            if res[0] >figvol*int_perc:
                qsr_graph.add_edge(figure_id, ref_id, QSR='inFrontOf')

            if res[1] >figvol*int_perc:
                qsr_graph.add_edge(figure_id, ref_id, QSR='behind')

        except:
            print("Problem")
            print(str(e))
            print(str(ref_id) + " " + str(figure_id))
            return qsr_graph

        #Infer more complex QSR by combining the base ones (e.g., touch and above --> on top of)
        if isAbove is True and touches is True: qsr_graph.add_edge(figure_id, ref_id, QSR='onTopOf')
        # to test the other cases of ON (affixedOn and leanOn) we need all base QSRs with other objects gathered first

        if not qsr_graph.has_edge(figure_id, ref_id): # If not special rel, by default/definition, they are neighbours
            qsr_graph.add_edge(figure_id, ref_id, QSR='near')

    return qsr_graph

def extract_surface_QSR(session, obj_id, qsr_graph, fht=0.15, wht=0.259):#fht=0.15, wht=0.2):
    """Extract QSRs through PostGIS
    between current object and surfaces marked as wall/floor
    fht: threshold to find objects that are at floor height, i.e., min Z coordinate = 0
    wht: for near wall surfaces - e.g., by default 20 cm
    motivation for threshold: granularity of map/GUI for wall annotation requires tolerance
    + we consider walls that are modelled 2D surfaces without depth in the GIS db, i.e., wht higher value than fht
    """
    tmp_conn, tmp_cur = session
    # Use postGIS for deriving truth values of base operators
    # min Z coordinate of bounding box is used to decide whether the object touches the floor
    # assumption of mobile ground robot: operates in contact with the floor with standard upward-pointing Z axis
    tmp_cur.execute("""SELECT ST_Zmin(a.bbox)
                    FROM anchors as a
                    WHERE a.anchor_key = %s
                    """,(obj_id,))
    # Unpack results and infer QSR predicates
    res = tmp_cur.fetchone()[0]
    if res <= fht:
        qsr_graph.add_edge(obj_id, 'floor', QSR='touches')
        qsr_graph.add_edge(obj_id, 'floor', QSR='onTopOf')
        #also add relations in opposite direction #this is only to infer special ON cases later
        qsr_graph.add_edge('floor', obj_id, QSR='touches')
        qsr_graph.add_edge('floor', obj_id, QSR='below')

    tmp_cur.execute("""SELECT ST_3DDistance(a.bbox,w.surface)
                       FROM anchors as a, sw_walls as w
                       WHERE a.anchor_key = %s
                        """, (obj_id,))
    res = [r[0] for r in tmp_cur.fetchall()] #[0]
    #find the min distance to walls
    ws = min(res)
    if ws <= wht:
        qsr_graph.add_edge(obj_id, 'wall', QSR='touches')
        #qsr_graph.add_edge('wall', obj_id, QSR='touches') #also add relation in opposite direction
    return qsr_graph

def retrieve_AOI(session):

    tmp_conn, tmp_cur = session
    tmp_cur.execute("""SELECT areakey, area_type
                            FROM sw_areas as ar""")

    return [(r[0], r[1]) for r in tmp_cur.fetchall()]

def extract_specialarea_QSR(session, obj_id, aoi_list, qsr_graph, dthresh=0.5):

    """
    Add to the graph the relations between object anchors and Areas of Interest
    (i.e., fire escape routes, designated waste areas, and fire call points)
    dthresh is the distance threshold to consider the proximity to a fire call point, in meters
    """

    tmp_conn, tmp_cur = session

    for area_key, area_type in aoi_list: #for each area of interest

        if area_type in ['fire_escape_area', 'waste_area']:
            # for fire escape areas and waste_areas: check containment
            # Areas are 2D polygons so we can use ST_Contain between anchor centroid and areas,
            # this operator will automatically project the 3D centroid on the 2D area to perform the containment check

            tmp_cur.execute("""SELECT ST_Contains(ar.area, a.location_3d)
                                       FROM anchors as a, sw_areas as ar
                                       WHERE a.anchor_key = %s
                                       and ar.areakey =%s
                                        """, (obj_id,area_key))

            is_in = tmp_cur.fetchone()[0]
            if is_in: #centroid of object within area of interest
                qsr_graph.add_edge(obj_id, area_type, QSR='in')

            # If robot is on fire escape route, add node to graph, if not added before
            if not qsr_graph.has_node('fire_escape_area') and area_type == 'fire_escape_area':
                tmp_cur.execute("""SELECT ST_Contains(ar.area, a.robot_position)
                                           FROM anchors as a, sw_areas as ar
                                           WHERE a.anchor_key = %s
                                           and ar.areakey =%s
                                           """, (obj_id, area_key))
                robot_is_in = tmp_cur.fetchone()[0]
                if robot_is_in: qsr_graph.add_node('fire_escape_area')

        else:

            # for fire call points:
            # if distance between obj centroid and point less than threshold say they touch
            tmp_cur.execute("""SELECT ST_DWithin(ar.area, a.location_3d, %s)
                                                   FROM anchors as a, sw_areas as ar
                                                   WHERE a.anchor_key = %s
                                                   and ar.areakey =%s
                                                    """, (dthresh,obj_id, area_key))

            is_within = tmp_cur.fetchone()[0]
            if is_within:  # centroid of object within area of interest
                qsr_graph.add_edge(obj_id, area_type, QSR='touches')

    return qsr_graph

def infer_special_ON(local_graph):
    """Iterates only over QSRs in current image
    but propagates new QSRs found to global graph
    expects label mapping in human-readable form as input"""
    for node1 in local_graph.nodes():

        t = [(f,ref,r) for f,ref,r in local_graph.out_edges(node1, data=True) if r['QSR'] =='touches']
        is_t = [(f,ref,r) for f,ref,r in local_graph.in_edges(node1, data=True) if r['QSR'] =='touches']
        is_a = [f for f,_,r in local_graph.in_edges(node1, data=True) if r['QSR']=='below'] #edges where obj1 is reference and figure objects are below it
        t_rs = list(set([ref for _,ref,_ in t]))
        if len(t)==0 and len(is_t)==0: continue #skip
        elif len(t)==1 and len(is_t)==0 or (len(t_rs) ==1 and t_rs=='wall'): #exactly one touch relation or touching only walls
            #where obj1 is fig, obj2 is ref
            node2 = t[0][1]
            l = [k for k in local_graph.get_edge_data(node1,node2) if local_graph.get_edge_data(node1,node2,k)['QSR']=='above']
            if len(l)==0: # obj1 is not above obj2
                local_graph.add_edge(node1, node2, QSR='affixedOn') #then obj1 is affixed on obj2
        elif len(t) ==0 and len(is_t)==1: continue # skip, will be added later when obj is found as fig in for loop
        else: # touches/is touched by more than one object
            #consider those where obj1 is figure
            nodes2 = [tr[1] for tr in t]
            for node2 in nodes2:
                others_below = [n for n in is_a if n!=node2 and n in nodes2]
                l = [k for k in local_graph.get_edge_data(node1, node2) if
                     local_graph.get_edge_data(node1, node2, k)['QSR'] == 'above']
                lb = [k for k in local_graph.get_edge_data(node1, node2) if
                     local_graph.get_edge_data(node1, node2, k)['QSR'] == 'below']
                if len(l)==0 and len(lb)==0 and len(others_below)>0: #and there is at least an o3 different from o2 which is below o1
                    local_graph.add_edge(node1, node2, QSR='leansOn') # then o1 leans on o2
    return local_graph