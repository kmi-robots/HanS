import sys
import json
from pycoral.utils.dataset import read_label_file

from params import get_parser
from postgresql.io import *
from postgresql.basic_queries import retrieve_new_anchor_measurements
from postgresql.spatial_queries import populate_with_boxes
from postgresql.size_queries import populate_with_sizes

from spatial_reasoner.spatial_reasoning import spatial_reason

from KB.size_kb import extract_size_kb
from KB.spatial_kb import extract_spatial_kb
from KB.commonsense_rels import extract_csk
from KB.wordnet_linking import map_to_synsets

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

    #TODO aggregate DL rankings on same anchor
    #TODO size reasoning based on background size KB

    qsr_graph = spatial_reason(connection,cursor,anchor_dict,args_dict) # spatial reasoner
    # TODO validate spatial relations based on background KB
    # aggregate DL with/without reasoning + meta-reasoning options
    # expand QSR graph based on Quasimodo concepts ./data/commonsense_extracted.json
    # check rules
    # once done with reasoning, mark all object anchors as complete
    disconnect_DB(connection,cursor) #close database connection
    # evaluate results
    pass


if __name__ == "__main__":
    main()
    sys.exit(0)