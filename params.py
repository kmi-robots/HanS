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
    parser.add_argument('--segm_model', default='./data/frozen_inference_graph_V2.pb', help='Path to segmentation model for OpenCV')
    parser.add_argument('--segm_config', default='./data/ssd_mobilenet_v2_coco_2018_03_29.pbtxt.txt', help='Path to segmentation model for OpenCV')
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

    # Data viz
    parser.add_argument('--bbox_color', default=(255, 0, 0), help='Color of detected bboxes')
    parser.add_argument('--bbox_thick', default=2, help='Border thickness of detected bboxes')

    return parser


