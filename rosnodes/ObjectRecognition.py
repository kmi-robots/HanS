"""
Interfaces with ROS2 to read RGB and Depth (RGBD) messages and detects objects through TPU
can be read live from the robot sensors or from bag played in the background
"""

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import message_filters

from cv_bridge import CvBridge
import cv2

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

        self.rgb_sub = message_filters.Subscriber(self, Image, cliargs.rgb_topic)
        self.pcl_sub = message_filters.Subscriber(self, PointCloud2, cliargs.pcl_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.pcl_sub], queue_size=1, slop=0.1) # sync rgb and pcl messages

        self.pub_hull = self.create_publisher(ConvexHull, cliargs.chull_topic, qos_profile=10) #A QoSProfile or a history depth to apply to the publisher

        # one callback for all
        self.ts.registerCallback(self.callback)


    def callback(self, img_msg, pcl_msg):

        cv2_im = self.bridge.imgmsg_to_cv2(img_msg)
        cv2_im_rgb_big = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)

        det_results = self.obj_engine.detect_objects(cv2_im_rgb_big)
        cv2_labeled_img = self.obj_engine.visualize_bboxes(cv2_im_rgb_big, det_results)

        #Visualize img annotated with bboxes
        cv2.imshow('win', cv2_labeled_img)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()

        for obj_ in det_results:
            x1,y1,x2,y2 = int(obj_.bbox.xmin), int(obj_.bbox.ymin), int(obj_.bbox.xmax), int(obj_.bbox.ymax)
            obj_roi = cv2_im_rgb_big.copy()
            obj_roi = obj_roi[y1:y2, x1:x2, :]  # crop image to bbox

            cv2.imshow('win', obj_roi)
            cv2.waitKey(2000)
            cv2.destroyAllWindows()

            segmented_pcl = derive_object_cloud([x1,y1,x2,y2], pcl_msg)
            filtered_pcl = pcl_remove_outliers(segmented_pcl,self.params)
            print(f'Filtered cloud of size: {len(filtered_pcl.points)}')
            chull_msg = derive_convex_hull(filtered_pcl,obj_.pred_ranking,pcl_msg)

            self.pub_hull.publish(chull_msg)






