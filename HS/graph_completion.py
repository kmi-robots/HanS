#TODO add mapping from classes extracted in quasimodo and target classes
# e.g., fire door mapped to door

def complete_graph(graph,property_dict):

    Gmod = graph.copy()
    for nodeid in graph.nodes():
        if "obj_label" in graph.nodes[nodeid].keys():
            cname = graph.nodes[nodeid]["obj_label"]
        else: continue # skip walls, floors and areas of interest,
                       # only objects are enriched with commonsense properties

        cname = cname.replace('_', ' ')
        commonsense_rels = property_dict[cname]

        for rel_, vs in commonsense_rels.items():
            for prop, KBscore in vs:
                if not Gmod.has_node(prop):
                    Gmod.add_node(prop)
                Gmod.add_edge(nodeid, prop, rel=rel_, weight=KBscore)

    return Gmod