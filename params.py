"""
Overview of all architecture parameters
- can be passed as command line arguments
"""

import argparse


def get_parser():
    parser = argparse.ArgumentParser()

    parser.add_argument('--mode', default='test', choices=['train', 'test'], help='Run in train or test mode')

    #Deep Learning params
    parser.add_argument('--model', default='ssd_mobilenet1',
                        choices=['ssd_mobilenet1', 'ssd_fpn_mobilenet1', 'ssd_mobilenet2', 'ssdlite_mobiledet'
                                 'efficientdet0', 'efficientdet1', 'efficientdet2', 'efficientdet3', 'efficientdet3x'], help='TPU model used to detect objects')

    parser.add_argument('--model_path', default='./data/ssd_mobilenet_v1_coco_quant_postprocess_edgetpu.tflite', help='Path to trained DL model')
    parser.add_argument('--impr_path', default=None, help='Path to classifier trained with weight imprinting.')
    parser.add_argument('--classes', default='./data/coco_labels.txt',help='Path to txt listing object classes')
    parser.add_argument('--conft', default=0.05, help='Confidence threshold for detected bounding boxes')

    #ROS & Robot-specific params
    parser.add_argument('--rgb_topic', default='/camera/rgb/image_rect_color', help='RGB camera topic name')
    parser.add_argument('--pcl_topic', default='/camera/depth_registered/points', help='PointCloud topic name')
    parser.add_argument('--chull_topic', default='/polyhedron', help='Output topic for detected convex hulls')
    parser.add_argument('--intr_path', default='./data/camera_intrinsics.txt', help='Path to camera intrinsics txt file')

    #PCL Outlier removal
    parser.add_argument('--dist', default=0.2, help='Threshold to filter pcl points by distance')
    parser.add_argument('--eps', default=0.05, help='DBSCAN eps value')
    parser.add_argument('--minp', default=10, help='DBSCAN min no of points')

    #Data viz
    parser.add_argument('--bbox_color', default=(255,0,0), help='Color of detected bboxes')
    parser.add_argument('--bbox_thick', default=2, help='Border thickness of detected bboxes')

    return parser


