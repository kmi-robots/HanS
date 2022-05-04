

def complete_graph(graph,property_dict):

    Gmod = graph.copy()
    for nodeid in graph.nodes():
        if nodeid not in ['wall','floor']:
            cname = graph.nodes[nodeid]["obj_label"]
        else: continue

        cname = cname.replace('_', ' ')
        commonsense_rels = property_dict[cname]

        for rel_, vs in commonsense_rels.items():
            for prop, KBscore in vs:
                if not Gmod.has_node(prop):
                    Gmod.add_node(prop)
                Gmod.add_edge(nodeid, prop, rel=rel_, weight=KBscore)

    return Gmod