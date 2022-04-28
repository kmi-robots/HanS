"""
Overview of all architecture parameters
- can be passed as command line arguments
"""

import argparse
import os


def get_parser():
    parser = argparse.ArgumentParser()

    # Deep Learning params
    parser.add_argument('--model_path', default='./data/ssd_mobilenet_v1_coco_quant_postprocess_edgetpu.tflite', help='Path to trained DL model')
    parser.add_argument('--classif_path', default='./data/kmi_imprinted_model.tflite', help='Path to classifier to run separately from detection.')
    parser.add_argument('--classes', default='./data/kmi_imprinted_model.txt', help='Path to txt listing object classes')
    parser.add_argument('--segm_model', default='./data/frozen_inference_graph_mask.pb', help='Path to segmentation model for OpenCV')
    parser.add_argument('--segm_config', default='./data/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt', help='Path to segmentation model for OpenCV')
    parser.add_argument('--segm_size', default=(300, 300), help='Value to resize input images for bbox detection')
    parser.add_argument('--segm_mean', default=(104, 117, 123), help='Mean values to normalise input images for bbox detection')
    parser.add_argument('--conft', default=0.05, help='Confidence threshold for detected bounding boxes')

    # ROS & Robot-specific params
    parser.add_argument('--rgb_topic', default='/camera/rgb/image_rect_color', help='RGB camera topic name')
    parser.add_argument('--pcl_topic', default='/camera/depth_registered/points', help='PointCloud topic name')
    parser.add_argument('--chull_topic', default='/polyhedron', help='Output topic for detected convex hulls')
    parser.add_argument('--intr_path', default='./data/camera_intrinsics.txt', help='Path to camera intrinsics txt file')
    parser.add_argument('--map_frame', default='map', help='Frame of reference of the map')
    parser.add_argument('--robot_frame', default='base_link', help='Frame of reference of the robot')

    # Postgresql/PostGIS params
    parser.add_argument('--dbuser', default=os.environ['USER'], help='Username of postgresql database')
    parser.add_argument('--dbname', default='gis_database', help='Postgresql database name')

    # PCL Outlier removal
    parser.add_argument('--vx', default=5, help='Number of points per voxel kept during downsampling')
    parser.add_argument('--std_r', default=2.0, help='Standard deviation threshold for statistical outlier removal')
    parser.add_argument('--eps', default=0.05, help='DBSCAN eps value')
    parser.add_argument('--minp', default=10, help='DBSCAN min no of points')

    #spatial reasoner
    parser.add_argument('--extract_spatialkb', type=str2bool, nargs='?', const=False, default=False,
                        help='If True, also extracts the background spatial knowledge from VG. '
                             'Otherwise, pre-extracted rels from VG are assumed to be available ')
    parser.add_argument('--spatialkb_path', default='./data/VG_spatial_stats.json',
                        help='Path to background spatial knowledge')
    parser.add_argument('--vg_src)', default='./data/relationships.json', help='Path to source Visual Genome relationships')
    parser.add_argument('--sf', default=1.2, help='Scaling factor to extrude object halfspaces')
    parser.add_argument('--dis', default=2, help='Distance threshold to find neighbours, defaults to 2 units in the SRID of spatial DB')
    parser.add_argument('--fht', default=0.15, help='Threshold to find objects that are at floor height, i.e., min Z coordinate = 0')
    parser.add_argument('--wht', default=0.259, help='Threshold to decide if object touches wall surfaces - e.g., by default 20 cm')
    parser.add_argument('--int_perc', default=0.05, help='Ratio of volume of figure object used for intersect spatial tests')

    #size reasoner
    parser.add_argument('--extract_sizekb', type=str2bool, nargs='?', const=False, default=False,
                        help='If True, also extracts the background size knowledge. '
                             'Otherwise, pre-extracted rels are assumed to be available ')
    parser.add_argument('--sizekb_path', default='./data/lab_obj_catalogue_autom_valid.json', help='Path to background size knowledge')
    parser.add_argument('--size_src', default='./data/shapenet_metadata_full.csv', help='Path to source size data')
    parser.add_argument('--hardcoded_size_src', default='./data/lab_obj_catalogue_manual.csv', help='Path to hardcoded size measures')
    parser.add_argument('--scraped_size_src', default='./data/scraped_objs', help='Path to Web-scraped size measures')
    parser.add_argument('--flat_src', default='./data/lab_obj_catalogue_autom_valid.json',
                        help='Manual annotations of flat/non-flat property, path to JSON file')
    parser.add_argument('--sizetol', default=0.05, help='Tolerance when considering min and max size'
                                                        'from hardcoded dimensions. Defaults to 5% of input dimension.')
    parser.add_argument('--Lambda', default=0.0, type=float, help='Set of cutoff thresholds for quantising the thickness')
    parser.add_argument('--T', default=0.0, type=float, help='Set of cutoff thresholds for quantising the area')
    parser.add_argument('--w0', default=1.4, type=float, help='Ratio for quantising the Aspect Ratio')

    #Graph completion
    parser.add_argument('--extract_quasi', type=str2bool, nargs='?', const=False, default=False,
                        help='If True, also extracts background knowledge from Quasimodo. '
                             'Otherwise, pre-extracted kb assumed to be available ')
    parser.add_argument('--quasikb_path', default='./data/commonsense_extracted.json', help='Path to background cs knowledge')
    parser.add_argument('--quasi_src', default='./data/quasimodo43.tsv', help='Path to source quasimodo data')
    parser.add_argument('--quasi_t', default=0.64, help='Threshold for filtering facts by confidence')
    parser.add_argument('--material_src', default='./data/shapenet_materials.csv', help='Path to source material data')

    #Wordnet terms
    parser.add_argument('--extract_synsets', type=str2bool, nargs='?', const=False, default=False,
                        help='If True, also extracts synsets for target classes from WordNet. '
                             'Otherwise, the mapping linked in syn_path is assumed as available ')
    parser.add_argument('--syn_path', default='./data/class_to_synset.json', help='Path to class name / synset mapping')

    #Map of the environment
    parser.add_argument("--map_img", default='./map/south_wing.pgm', help="Path to the pgm image of the map")
    parser.add_argument("--map_cfg", default='./map/south_wing.yaml', help="Path to yaml file with map configuration")
    parser.add_argument("--wall_height", default=4, type=int, help="Height for extruding walls, in meters. Defaults to 4.")

    # Data viz
    parser.add_argument('--bbox_color', default=(255, 0, 0), help='Color of detected bboxes')
    parser.add_argument('--bbox_thick', default=2, help='Border thickness of detected bboxes')
    return parser


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else: raise argparse.ArgumentTypeError('Boolean value expected.')

