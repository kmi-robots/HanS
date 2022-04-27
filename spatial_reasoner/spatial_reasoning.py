"""Methods used for spatial reasoning on a Postgresql db"""
from collections import OrderedDict

def spatial_reason(conn, cur, anchors):

    return


def order_by_volume(session, akey, vthresh= 10.):
    #now we also filter objects with volume above 10 m3 (clear outliers)
    tmp_conn, tmp_cur = session
    tmp_cur.execute('SELECT anchor_key, ST_Volume(bbox) as v FROM anchors '\
                    'WHERE anchor_key %s AND ST_Volume(bbox) <= %s '\
                    'ORDER BY v DESC', (akey,vthresh)) #use string pattern matching to check only first part of ID
    # & order objects by volume, descending, to identify reference objects
    res = tmp_cur.fetchall()
    return OrderedDict(res) #preserve ordering in dict #{k[0]: {} for k in res}  # init with obj ids


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
            tmp_cur.execute('SELECT ST_3DDWithin(fig.object_polyhedral_surface,reff.object_polyhedral_surface, 0)'
                            ' from semantic_map as reff, semantic_map as fig'
                            ' WHERE reff.object_id = %s'\
                        ' AND fig.object_id = %s', (ref_id,figure_id))
            res = tmp_cur.fetchone()
            if res[0] is True:
                qsr_graph.add_edge(figure_id, ref_id, QSR='touches')
                touches = True
        except Exception as e:
            print(str(e))
            print("Query too large, server problem raised")
            print(ref_id + " " + figure_id)
            return qsr_graph
        try:
            """tmp_cur.execute('SELECT ST_3DIntersects(fig.bbox,reff.tophsproj),ST_3DIntersects(fig.bbox,reff.bottomhsproj)'
                            ' from semantic_map as reff, semantic_map as fig'
                            ' WHERE reff.object_id = %s' \
                            ' AND fig.object_id = %s', (ref_id, figure_id))"""
            #causes degenerate cases of 1 pixel overlap, changing to thresholds
            tmp_cur.execute(
                'SELECT ST_Volume(ST_3DIntersection(fig.bbox,reff.tophsproj)),ST_Volume(ST_3DIntersection(fig.bbox,reff.bottomhsproj)), ST_Volume(fig.bbox)'
                ' from semantic_map as reff, semantic_map as fig'
                ' WHERE reff.object_id = %s' \
                ' AND fig.object_id = %s', (ref_id, figure_id))
            res = tmp_cur.fetchone()
            figvol = res[2]
            if res[0] > figvol*int_perc: #intersection is at list x% of volume of figure obj
                qsr_graph.add_edge(figure_id, ref_id, QSR='above')
                isAbove = True
            if res[1] >figvol*int_perc: qsr_graph.add_edge(figure_id, ref_id, QSR='below')
        except:
            print("Query too large, server problem raised")
            print(ref_id + " " + figure_id)
            return qsr_graph

        try:
            tmp_cur.execute('SELECT ST_Volume(ST_3DIntersection(fig.bbox,reff.lefthsproj)),ST_Volume(ST_3DIntersection(fig.bbox,reff.righthsproj)),'
                            'ST_Volume(fig.bbox)'
                            ' from semantic_map as reff, semantic_map as fig'
                            ' WHERE reff.object_id = %s' \
                            ' AND fig.object_id = %s', (ref_id, figure_id))
            res = tmp_cur.fetchone()
            figvol = res[2]
            if res[0] >figvol*int_perc:
                qsr_graph.add_edge(figure_id, ref_id, QSR='leftOf')
                qsr_graph.add_edge(figure_id, ref_id, QSR='beside')

            if res[1]>figvol*int_perc:
                qsr_graph.add_edge(figure_id, ref_id, QSR='rightOf')
                qsr_graph.add_edge(figure_id, ref_id, QSR='beside')

        except:
            print("Query too large, server problem raised")
            print(ref_id + " " + figure_id)
            return qsr_graph
        try:
            tmp_cur.execute('SELECT ST_Volume(ST_3DIntersection(fig.bbox,reff.fronthsproj)), ST_Volume(ST_3DIntersection(fig.bbox,reff.backhsproj)),'
                            'ST_Volume(fig.bbox)'
                            ' from semantic_map as reff, semantic_map as fig'
                            ' WHERE reff.object_id = %s' \
                            ' AND fig.object_id = %s', (ref_id, figure_id))
            res = tmp_cur.fetchone()
            figvol = res[2]
            if res[0] >figvol*int_perc: qsr_graph.add_edge(figure_id, ref_id, QSR='inFrontOf')

            if res[1] >figvol*int_perc: qsr_graph.add_edge(figure_id, ref_id, QSR='behind')

        except:
            print("Query too large, server problem raised")
            print(ref_id + " " + figure_id)
            return qsr_graph

        #Infer more complex QSR by combining the base ones (e.g., touch and above --> on top of)
        if isAbove is True and touches is True: qsr_graph.add_edge(figure_id, ref_id, QSR='onTopOf')
        # to test the other cases of ON (affixedOn and leanOn) we need all base QSRs with other objects gathered first

        if not qsr_graph.has_edge(figure_id, ref_id): # If not special rel, by default/definition, they are neighbours
            qsr_graph.add_edge(figure_id, ref_id, QSR='near')

    return qsr_graph

def extract_surface_QSR(session, obj_id, qsr_graph, fht=0.15, wht=0.2):
    """Extract QSRs through PostGIS
    between current object and surfaces marked as wall/floor
    fht: threshold to find objects that are at floor height, i.e., min Z coordinate = 0
    wht: for near wall surfaces - e.g., by default 20 cm
    motivation for threshold: granularity of map/GUI for wall annotation requires tolerance
    + account that walls are modelled 2D surfaces without depth in the GIS db, i.e., needs higher value than fht
    Expects a table of precomputed values formatted as per ./data/space_prep.py
    """
    tmp_conn, tmp_cur = session
    # Use postGIS for deriving truth values of base operators
    tmp_cur.execute("""SELECT o_zmin
                    FROM objects_precalc
                    WHERE object_id = %s
                    """,(obj_id,))
    # Unpack results and infer QSR predicates
    res = tmp_cur.fetchone()[0]
    if res <= fht:
        qsr_graph.add_edge(obj_id, 'floor', QSR='touches')
        qsr_graph.add_edge(obj_id, 'floor', QSR='onTopOf')
        #also add relations in opposite direction #this is only to infer special ON cases later
        qsr_graph.add_edge('floor', obj_id, QSR='touches')
        qsr_graph.add_edge('floor', obj_id, QSR='below')

    tmp_cur.execute("""SELECT ow_distance
                       FROM objects_precalc
                       WHERE object_id = %s
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