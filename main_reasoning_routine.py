import sys
from params import get_parser
from postgresql.io import *
from postgresql.basic_queries import retrieve_new_anchor_measurements
def main():

    # Load parameters and maintain as dictionary
    parser = get_parser()
    args_dict, unknown = parser.parse_known_args()
    connection, cursor = connect_db(args_dict.dbuser, args_dict.dbname) # connect to db

    measurement_anchor_list = retrieve_new_anchor_measurements(connection, cursor)
    # TODO

    # retrieve all DL predictions related to anchor
    # compute size and spatial bboxes of union chull
    # compute QSR between anchors
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