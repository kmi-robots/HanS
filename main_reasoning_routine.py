import sys
from params import get_parser
from postgresql.io import *
from postgresql.basic_queries import retrieve_new_anchor_measurements
from postgresql.spatial_queries import populate_with_boxes
from postgresql.size_queries import populate_with_sizes
from spatial_reasoner.spatial_reasoning import spatial_reason

def main():

    # Load parameters and maintain as dictionary
    parser = get_parser()
    args_dict, unknown = parser.parse_known_args()
    connection, cursor = connect_db(args_dict.dbuser, args_dict.dbname) # connect to db

    # retrieve new object anchors to be examined and all DL predictions related to each anchor
    #i.e., either a newly added anchor or a former anchor for which a new measurement was recorded
    anchor_dict = retrieve_new_anchor_measurements(connection, cursor)
    populate_with_boxes(connection,cursor,sf=args_dict.sf)# compute size and spatial bboxes of union chull
    populate_with_sizes(connection,cursor) #estimate anchor sizes (based on bbox)
    print("Spatial DB completed with anchor bounding boxes and sizes")

    qsr_graph = spatial_reason(connection,cursor,anchor_dict) # spatial reasoner
    # TODO

    # aggregate DL with/without reasoning + meta-reasoning options

    # TODO (not here) code to prep size and spatial background knowledge given a set of classes as input
    # TODO (not here) add code for wall annotation given a map

    # expand QSR graph based on Quasimodo concepts ./data/commonsense_extracted.json
    # check rules
    # once done with reasoning, mark all object anchors as complete
    # evaluate results
    pass


if __name__ == "__main__":
    main()
    sys.exit(0)