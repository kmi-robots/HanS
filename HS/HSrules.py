import networkx as nx
from itertools import product

def is_there_an_edge(scene_graph, node1, node2, nmap, edge_type=None, except_node=None):
    """
    All rules are modelled generically as queries on an input graph.
    Is there an edge of type edge_type between node1 and node2?
    :param scene_graph: input graph from the observed scene
    :param node1: named of object class or property as subject of relation
    :param node2: named of object class or property as object of relation
    :param nmap: dictionary that maps class names to anchor ids in the map, if node not directly labelled with class
    :param edge_type: the type of relation we are looking for
    :param except_no: class for which the rule does not apply, associated with cases where node2==any
    :return: boolean
    """
    logic_res = False

    neighbour_nodes =[nb for nb in scene_graph.neighbors(node1)] #neighbours are only out edges from node 1
    neighbour_nodes.extend([n_ for n_,_ in scene_graph.in_edges(node1)])

    if node2 == 'any':
        tgt_neighbour_nodes = [nb for nb in neighbour_nodes if nb != except_node]# any object except value of except_node counts
    else:
        tgt_neighbour_nodes = [node2]

    if node2 in nmap.keys():
        tgt_neighbour_nodes.extend(nmap[node2]) # add anchor ids for node2 in case class not directly labelled on node

    for tnn in tgt_neighbour_nodes:
        edge_data = scene_graph.get_edge_data(node1, tnn) # returns None if there is no edge
        if edge_data is None:
            edge_data = scene_graph.get_edge_data(tnn, node1) #try swapping the two (directed graph)
        if edge_data is not None:
            for vdict in edge_data.values():  # e.g. vdict = {'QSR': 'affixedOn'}
                vit = list(vdict.items())[0]
                vk, vv = vit
                if vk == edge_type or vv == edge_type: # is there an edge of type edge_type?
                    logic_res = True
    return logic_res

def is_there_property_edge(scene_graph, propkey, maxhops=3):
    # Handle multi-hop edge check for commonsense properties in knowledge-completed graph, e.g., in the ignition, flammable case
    matching_nodes = []
    Gu = scene_graph.to_undirected() # make multidigraph unidrected otherwise path search not supported

    for nid in scene_graph.nodes():
        if nid not in matching_nodes:# skip visited nodes from next loop
            path = nx.shortest_path(Gu, source=nid, target=propkey) #e.g., shortest path between book and flammable
            if len(path)>0 and len(path)<=maxhops+1: #maximum number of hops allowed to consider a path valid
                #if path is found
                hasqsr = False
                for j in range(len(path)-1):
                    n1 = path[j]
                    n2 = path[j+1]
                    edge_attr = scene_graph.get_edge_data(n1, n2) #check on original directed graph
                    if edge_attr is None: #consider direction of relation this time, path does not consider it
                        edge_attr = scene_graph.get_edge_data(n2, n1) #swap n1 and n2

                    qsr_list = [v for v in edge_attr.values() if 'QSR' in v.keys()]
                    if len(qsr_list)>0:
                        hasqsr=True
                        break # we skip paths that contain QSR relations (being near an object that has that property does not count)
                if not hasqsr:
                    matching_nodes.extend([p for p in path if p!=propkey])

    matching_nodes = list(set(matching_nodes)) #remove dups
    return matching_nodes

def is_small(graph, obj_node, volume_thresh=0.002):
    # volume threshold in cube meters, e.g., defaults to 2 litres
    # if object is small and on floor then trip hazard
    return graph[obj_node]["obj_volume"] <= volume_thresh

def call_rule_check(G, node1prefix, node1, rule_body, nodemapping):

    # single rule call
    n2 = rule_body['node2']
    n2prefix, n2key = n2.split(':')
    rel = rule_body['edge']
    negated = rule_body['negation']

    # Optional keys: rule exception and max no of edges allowed between objects
    if "except" in rule_body.keys():
        n3 = rule_body['except'].split(':')[1]  # class that is an exception to the rule
    else:
        n3 = None

    check_flag = False

    if node1prefix== "property": #separate behaviour to check properties, walk on graph paths

        # is there a node with that property?
        n1list = is_there_property_edge(G, node1)
        if n2prefix == "property":
            n2list = is_there_property_edge(G, n2key)
        else:
            n2list = [n2key]
        # is there the target qsr between elements from these two paths? e.g., does heater touch book
        pairs = list(product(n1list, n2list))
        for n1, n2 in pairs:
            check_flag = is_there_an_edge(G, n1, n2, nodemapping, edge_type=rel)
            if check_flag: break  # stop when you find one

    else: # objects or areas
        check_flag = is_there_an_edge(G, node1, n2key, nodemapping, edge_type=rel, except_node=n3)

    if "is_small" in rule_body.keys():
        check_flag = check_flag and is_small(G, node1)

    if negated:
        check_flag = not check_flag

    return check_flag



def check_rules(mod_graph, rules, node_map):

    """
    In our representation, rules can be defined between nodes of type area, object or property
    """

    for rname, rnode in rules.items():
        class_to_check = rnode["node1"]

        classprefix, classkey = class_to_check.split(':')# remove node prefix from name

        if classkey not in node_map.keys() and not mod_graph.has_node(classkey):
            print("No node with property %s in graph" % classkey)
            print("skipping rule")
            continue

        if classkey in node_map.keys():
            anchor_list = [(classprefix, ck) for ck in node_map[classkey]]  # map class name to node id(s)
        else:
            anchor_list = [(classprefix,classkey)]

        print(rnode["description"])  # rule applies, print out rule under check
        rule_template = rnode["rule"]

        if "maxno" in rule_template.keys():
            mn = rule_template['maxno']
        else:
            mn = float('inf')

        cnt = 0
        for n1pref, n1 in anchor_list:  # for each node of target class

            if "node2" in rule_template:  # single rule call
                check = call_rule_check(mod_graph, n1pref, n1, rule_template, node_map)

            else:
                # composite rule with logic operators
                if "or" in rule_template.keys():
                    rts = rule_template['or']
                    check = any(
                        [call_rule_check(mod_graph, n1pref, n1, rt, node_map) for rt in
                         rts])  # equivalent of OR on python lists

                elif "and" in rule_template.keys():
                    rts = rule_template['and']
                    check = all(
                        [call_rule_check(mod_graph, n1pref, n1, rt, node_map) for rt in
                         rts])  # equivalent of AND on python lists
                else:
                    print("Unknown operand type.. skipping rule")
                    continue

            if check and mn < float('inf'):
                cnt += 1  # maxno constraint in rule, keep track of number of matches
                check = check and cnt < mn  # is number of matching edges within maxno?, e.g., for the multiplug rule

            if check:
                print("Rule check passed for anchor %s of class %s" % (str(n1), classkey))

            else:
                print("[Warning!] rule check not passed for anchor %s of class %s" % (str(n1), classkey))
                if cnt >= mn:
                    print("Max number of allowed relations reached")  # added for further explanation



