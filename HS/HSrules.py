

def is_there_an_edge(scene_graph, node1, node2, nmap, edge_type=None, except_node=None, maxno=float('inf')):
    """
    All rules are modelled generically as queries on an input graph.
    Is there an edge of type edge_type between node1 and node2?
    :param scene_graph: input graph from the observed scene
    :param node1: named of object class or property as subject of relation
    :param node2: named of object class or property as object of relation
    :param nmap: dictionary that maps class names to anchor ids in the map, if node not directly labelled with class
    :param edge_type: the type of relation we are looking for
    :param except_no: class for which the rule does not apply, associated with cases where node2==any
    :param maxno: maximum number of edges allowed between the objects (e.g., how many plugs?)
    :return: boolean
    """
    #TODO handle multi-hop edge check , e.g., in the ignition, flammable case

    logic_res = False

    neighbour_nodes =[nb for nb in scene_graph.neighbors(node1)]
    if node2 == 'any':
        tgt_neighbour_nodes = [nb for nb in neighbour_nodes if nb != except_node]# any object except value of except_node counts

    elif edge_type=='any': #assumption: there are no rules where both node2 and edge_type equal any
        # any type of edge counts, do not need to check relation type
        if node2 in neighbour_nodes:
            logic_res= True
        return logic_res
    else:
        tgt_neighbour_nodes = [node2]

    if node2 in nmap.keys():
        tgt_neighbour_nodes.extend(nmap[node2]) # add anchor ids for node2 in case class not directly labelled on node

    for tnn in tgt_neighbour_nodes:
        edge_data = scene_graph.get_edge_data(node1, tnn) # returns None if there is no edge
        if edge_data is not None:
            for vdict in edge_data.values():  # e.g. vdict = {'QSR': 'affixedOn'}
                vit = list(vdict.items())[0]
                vk, vv = vit
                if vk == edge_type or vv == edge_type: # is there an edge of type edge_type?
                    logic_res = True
                    break # as soon as you find one match, break

    return logic_res

def is_small(graph, obj_node, volume_thresh=0.002):
    # volume threshold in cube meters, e.g., defaults to 2 litres
    # if object is small and on floor then trip hazard
    return graph[obj_node]["obj_volume"] <= volume_thresh

def call_rule_check(G, node1, rule_body, nodemapping):

    # single rule call
    n2 = rule_body['node2']
    rel = rule_body['edge']
    negated = rule_body['negation']

    # Optional keys: rule exception and max no of edges allowed between objects
    if "except" in rule_body.keys():
        n3 = rule_body['except']  # class that is an exception to the rule
    else:
        n3 = None

    if "maxno" in rule_body.keys():
        mn = rule_body['maxno']  # class that is an exception to the rule
    else:
        mn = float('inf')

    check_flag = is_there_an_edge(G, node1, n2, nodemapping, edge_type=rel, except_node=n3, maxno=mn)

    if "is_small" in rule_body.keys():
        check_flag = check_flag and is_small(G, node1)

    if negated:
        check_flag = not check_flag

    return check_flag


def check_rules(mod_graph, rules, node_map):

    for rname, rnode in rules.items():
        class_to_check = rnode["node1"]

        #TODO handle checks on map areas based on current robot position
        if "area" in class_to_check:
            continue
        elif class_to_check in ["wall", "floor"]:
            continue #TODO Floor or wall specific checks
        else: #all other classes of objects
            if class_to_check not in node_map.keys():
                print("No objects of type %s detected" % class_to_check)
                print("Skipping to next rule")
                continue

            anchor_list = node_map[class_to_check]
            print(rnode["description"]) # rule applies, print out rule under check
            rule_template = rnode["rule"]

            for n1 in anchor_list: # for each node of target class
                if "node2" in rule_template: # single rule call
                    check = call_rule_check(mod_graph, n1, rule_template, node_map)

                else:
                    # composite rule with logic operators
                    if "or" in rule_template.keys():
                        rts = rule_template['or']
                        check = any([call_rule_check(mod_graph, n1, rt, node_map) for rt in rts]) # equivalent of OR on python lists

                    elif "and" in rule_template.keys():
                        rts = rule_template['and']
                        check = all([call_rule_check(mod_graph, n1, rt, node_map) for rt in rts])  # equivalent of AND on python lists
                    else:
                        print("Unknown operand type.. skipping rule")
                        continue

                if check:
                    print("Rule check passed for anchor %s of class %s" %(str(n1),(class_to_check)))
                else:
                    print("[Warning!] rule check not passed for anchor %s of class %s" % (str(n1), (class_to_check)))


