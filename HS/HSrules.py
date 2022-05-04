
def is_there_an_edge(scene_graph, node1, node2, edge_type=None):
    """
    All rules are modelled generically as queries on an input graph.
    Is there an edge of type edge_type between node1 and node2?
    :param scene_graph: input graph from the observed scene
    :param node1: named of object class or property as subject of relation
    :param node2: named of object class or property as object of relation
    :param edge_type: the type of relation we are looking for
    :return: boolean
    """

    logic_res = False

    return logic_res

def is_small():
    # if object is small and on floor then trip hazard
    return False

"""def is_overloaded():

    return False

def is_in():
    return False

def is_near():
    return False

def is_combustible():
    return False

def is_ignition_source():
    return False

def is_accesible():
    return False
"""


def check_rules(mod_graph):

    #Are portable fire extinguishers either securely wall mounted or on a supplied stand?
    #TODO is there a node fire extinguisher ?
    # if no, skip rule as not relevant
    #otherwise show on screen
    print("Are portable fire extinguishers either securely wall mounted or on a supplied stand?")
    check = is_there_an_edge(mod_graph, 'fire_extinguisher', 'wall', edge_type='affixedOn') \
                or is_there_an_edge(mod_graph, 'fire_extinguisher', 'stand', edge_type='leansOn')
    if check:
        print("Passed check!")
    else:
        print("Did not pass check!")
    pass