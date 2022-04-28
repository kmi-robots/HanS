"""Methods used for spatial reasoning on a Postgresql db"""
from collections import OrderedDict
import networkx as nx
from utils import plot_graph

def build_QSR_graph(conn, cur, anchors, args):

    session = (conn,cur)
    QSRs = nx.MultiDiGraph()
    ids_ord = order_by_volume((conn,cur), list(anchors.keys()))
    QSRs.add_nodes_from(ids_ord.keys()) # one node per anchor

    for i, o_id in enumerate(ids_ord.keys()):
        figure_objs = find_neighbours(session, o_id, ids_ord, T=args.T)
        if len(figure_objs) > 0:
            QSRs = extract_QSR(session, o_id, figure_objs, QSRs, int_perc=args.int_perc)  # relations between objects
            # plot_graph(QSRs)

        #Changed to extract_surface_QSR (no precalc) + new walls
        QSRs = extract_surface_QSR(session, o_id, QSRs, fht=args.fht, wht=args.wht)  # relations with walls and floor
    # after all reference objects have been examined
    # derive special cases of ON
    QSRs = infer_special_ON(QSRs)

    # QSRs = remove_redundant_qsrs(QSRs) # used to evaluate the QSR to make sure they were not duplicated but for VG best to keep synonyms e.g., both LeftOf and beside

    #to visualize extracted QSR graph
    # plot_graph(QSRs)

    return QSRs

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


def find_neighbours(session, ref_id, ordered_objs,T=2):
    """T = distance threshold to find neighbours, defaults to 2 units in the SRID of spatial DB"""
    #Find nearby objects which are also smaller in the ordering
    i = list(ordered_objs.keys()).index(ref_id)
    candidates = list(ordered_objs.keys())[i+1:] #candidate figure objects, i.e., smaller
    # Which ones are nearby?
    tmp_conn, tmp_cur = session
    tmp_cur.execute('SELECT anchor_key FROM anchors'\
                    ' WHERE ST_3DDWithin(bbox, '\
                    '(SELECT bbox FROM anchors '\
                    'WHERE anchor_key = %s), %s) '\
                    'AND anchor_key != %s', (ref_id,str(T),ref_id))

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
            if res[0] >figvol*int_perc: qsr_graph.add_edge(figure_id, ref_id, QSR='inFrontOf')

            if res[1] >figvol*int_perc: qsr_graph.add_edge(figure_id, ref_id, QSR='behind')

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