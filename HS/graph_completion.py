"""
Completing a scene graph with commonsense knowledge retrieved externally
"""

taxonomy_to_quasi = {
    "fire door": "door",
    "fire extinguisher sign":"safety sign",
    "fire alarm assembly sign":"safety sign",
    "emergency exit sign":"safety sign",
    "power_cord": "plug",
    "rubbish bin": "trash can"
}

def complete_graph(graph,property_dict):

    Gmod = graph.copy()
    for nodeid in graph.nodes():
        if "obj_label" in graph.nodes[nodeid].keys():
            cname = graph.nodes[nodeid]["obj_label"]
        else: continue # skip walls, floors and areas of interest,
                       # only objects are enriched with commonsense properties

        cname = cname.replace('_', ' ')
        if cname not in property_dict.keys() and cname in taxonomy_to_quasi.keys():
            # find equivalent keyword used in quasimodo
            cname = taxonomy_to_quasi[cname]
        elif cname not in property_dict.keys() and cname not in taxonomy_to_quasi.keys():
            continue # unknown object class based on current KB

        commonsense_rels = property_dict[cname]
        for rel_, vs in commonsense_rels.items():
            for prop, KBscore in vs:

                if isinstance(KBscore, str):
                    #convert material percentages from str to float
                    KBscore = float(KBscore[:-1])

                if not Gmod.has_node(prop):
                    Gmod.add_node(prop)
                Gmod.add_edge(nodeid, prop, rel=rel_, weight=KBscore)

    return Gmod