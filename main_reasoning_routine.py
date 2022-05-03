import sys
import json
from pycoral.utils.dataset import read_label_file

from params import get_parser
from postgresql.io import *
from postgresql.basic_queries import retrieve_new_anchor_measurements
from postgresql.spatial_queries import populate_with_boxes
from postgresql.size_queries import populate_with_sizes

from spatial_reasoner.spatial_reasoning import build_QSR_graph,spatial_validate

from KB.size_kb import extract_size_kb
from KB.spatial_kb import extract_spatial_kb
from KB.commonsense_rels import extract_csk
from KB.wordnet_linking import map_to_synsets

from DL.ranking_aggregation import merge_scored_ranks
from size_reasoner.size_reasoning import size_validate

def main():

    # Load parameters and maintain as dictionary
    parser = get_parser()
    args_dict, unknown = parser.parse_known_args()
    target_classes = read_label_file(args_dict.classes) #dictionary with format classid: classname

    connection, cursor = connect_db(args_dict.dbuser, args_dict.dbname) # connect to db

    print("class to WN synset mapping")
    # dictionary of format classname: array of synsets
    if args_dict.extract_synsets:
        target_synsets = map_to_synsets(list(target_classes.values()), args_dict)
    else:
        with open(args_dict.syn_path) as fp: #load pre-extracted from local
            target_synsets = json.load(fp)
            print("Loaded size KB from local")

    print("Retrieving background KBs")
    if args_dict.extract_sizekb: #extract from scratch
        print("Extracting size knowledge from scratch ... it will take long")
        sizeKB = extract_size_kb(target_classes, args_dict)
    else:
        with open(args_dict.sizekb_path) as fp: #load pre-extracted from local
            sizeKB = json.load(fp)
            print("Loaded size KB from local")

    if args_dict.extract_spatialkb: #extract from scratch
        print("Extracting spatial knowledge from scratch ... it will take long")
        spatialKB = extract_spatial_kb(connection, cursor, args_dict)
    else:
        with open(args_dict.spatialkb_path) as fp: #load pre-extracted from local
            spatialKB = json.load(fp)
            print("Loaded spatial KB from local")

    if args_dict.extract_quasi: #extract from scratch
        print("Extracting commonsense facts from Quasimodo... it will take long")
        quasiKB = extract_csk(args_dict)
    else:
        with open(args_dict.quasikb_path) as fp: #load pre-extracted from local
            quasiKB = json.load(fp)
            print("Loaded relevant quasimodo facts from local")

    # retrieve new object anchors to be examined and all DL predictions related to each anchor
    #i.e., either a newly added anchor or a former anchor for which a new measurement was recorded
    anchor_dict = retrieve_new_anchor_measurements(connection, cursor)
    populate_with_boxes(connection,cursor,sf=args_dict.sf) # compute size and spatial bboxes of union chull
    populate_with_sizes(connection,cursor) # estimate anchor sizes (based on bbox)
    print("Spatial DB completed with anchor bounding boxes and sizes")

    print("Extracting observed QSR between objects")
    qsr_graph = build_QSR_graph(connection, cursor, anchor_dict, args_dict) # extract QSR

    print("Aggregating DL predictions across observations")
    aggr_ranks = []
    for a_id, attr in anchor_dict.items():
        #Aggregate DL rankings on same anchor
        aggr_DL_rank = merge_scored_ranks(attr['DL_predictions'])

        # TODO select which anchors in anchor_dict need correction (based on aggr confidence)
        # annotate qsr graph with top1 DL pred
        topclass = target_classes[aggr_DL_rank[0][0]]
        qsr_graph.nodes[a_id]["DL_label"] = topclass
        aggr_ranks.append((a_id, aggr_DL_rank))

    print("Reasoning on object anchors")
    for a_id, aggr_DL_rank in aggr_ranks: #TODO only loop through those that need correction
        # Validate merged ranking based on size KB
        sizev_rank = size_validate(aggr_DL_rank, cursor, a_id, sizeKB)
        # Validate ranking based on spatial KB
        spatial_outrank = spatial_validate(a_id, aggr_DL_rank,sizev_rank, qsr_graph, spatialKB, target_synsets, meta=args_dict.meta)
        read_input = [(target_classes[cid], score) for cid, score in spatial_outrank]
        print("Ranking post meta-reasoning")
        print(read_input)

    # TODO scene assessment part
    # expand QSR graph based on Quasimodo concepts ./data/commonsense_extracted.json
    # check rules
    # once done with reasoning, mark all object anchors as complete
    disconnect_DB(connection,cursor) #close database connection
    # evaluate results
    pass


if __name__ == "__main__":
    main()
    sys.exit(0)