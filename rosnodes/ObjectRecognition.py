"""
Interfaces with ROS2 to read RGB and Depth (RGBD) messages and detects objects through TPU
can be read live from the robot sensors or from bag played in the background
"""

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import message_filters

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime
from scipy.spatial.transform import Rotation as R

from utils import load_camera_intrinsics_txt
from utils import derive_object_cloud
from utils import pcl_remove_outliers
from utils import derive_convex_hull

from DL.models import ObjRecEngine
from boundingbox_msg.msg import ConvexHull


class ObjectRecognition(Node):

    def __init__(self, cliargs):

        super().__init__('ObjectRecognition')
        self.camintr = load_camera_intrinsics_txt(cliargs.intr_path)
        self.params = cliargs

        self.bridge = CvBridge()
        self.obj_engine = ObjRecEngine(cliargs)

        self.id_prefix = datetime.today().strftime('%Y%m%d%H%M')
        self.id_counter = 0

        self.rgb_sub = message_filters.Subscriber(self, Image, cliargs.rgb_topic)
        self.pcl_sub = message_filters.Subscriber(self, PointCloud2, cliargs.pcl_topic)
        self.pub_rgb = self.create_publisher(Image, cliargs.rgb_topic + '/bboxes', 10)

        # sync rgb and pcl messages
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.pcl_sub], queue_size=1, slop=0.1)

        # A QoSProfile or a history depth to apply to the publisher
        self.pub_hull = self.create_publisher(ConvexHull, cliargs.chull_topic, qos_profile=10)

        # one callback for all
        self.ts.registerCallback(self.callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def callback(self, img_msg: Image, pcl_msg: PointCloud2):

        try:
            pc_trans = self.tf_buffer.lookup_transform(self.params.map_frame, pcl_msg.header.frame_id,
                                                       pcl_msg.header.stamp)
            robot_pose = self.tf_buffer.lookup_transform(self.params.map_frame, self.params.robot_frame,
                                                         pcl_msg.header.stamp)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform map to {pcl_msg.header.frame_id}: {ex}')
            return

        o3d_transform = np.identity(4)
        o3d_transform[0:3, 0:3] = R.from_quat([pc_trans.transform.rotation.x,
                                               pc_trans.transform.rotation.y,
                                               pc_trans.transform.rotation.z,
                                               pc_trans.transform.rotation.w]).as_matrix()
        o3d_transform[0, 3] = pc_trans.transform.translation.x
        o3d_transform[1, 3] = pc_trans.transform.translation.y
        o3d_transform[2, 3] = pc_trans.transform.translation.z

        cv2_im = self.bridge.imgmsg_to_cv2(img_msg)
        cv2_im_rgb_big = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)

        det_results = self.obj_engine.detect_objects(cv2_im_rgb_big)
        cv2_labeled_img = self.obj_engine.visualize_bboxes(cv2_im_rgb_big, det_results)

        rgb_msg = self.bridge.cv2_to_imgmsg(cv2_labeled_img, encoding='bgr8')
        self.pub_rgb.publish(rgb_msg)

        # Visualize img annotated with bboxes
        # cv2.imshow('win', cv2_labeled_img)
        # cv2.waitKey(5000)
        # cv2.destroyAllWindows()

        for obj_ in det_results:
            x1, y1, x2, y2 = int(obj_.bbox.xmin), int(obj_.bbox.ymin), int(obj_.bbox.xmax), int(obj_.bbox.ymax)
            obj_roi = cv2_im_rgb_big.copy()
            obj_roi = obj_roi[y1:y2, x1:x2, :]  # crop image to bbox

            # cv2.imshow('win', obj_roi)
            # cv2.waitKey(2000)
            # cv2.destroyAllWindows()

            segmented_pcl = derive_object_cloud([x1, y1, x2, y2], pcl_msg)
            filtered_pcl = pcl_remove_outliers(segmented_pcl, o3d_transform, self.params)
            print(f'Filtered cloud of size: {len(filtered_pcl.points)}')
            chull_id = self.id_prefix + '_' + str(self.id_counter)
            chull_msg = derive_convex_hull(filtered_pcl, obj_.pred_ranking, pcl_msg, chull_id)
            chull_msg.robot_pose.header.stamp = chull_msg.header
            chull_msg.robot_pose.pose.position = robot_pose.transform.translation
            chull_msg.robot_pose.pose.orientation = robot_pose.transform.rotation
            self.id_counter += 1

            self.pub_hull.publish(chull_msg)
