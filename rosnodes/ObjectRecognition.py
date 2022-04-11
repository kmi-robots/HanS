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
from utils import remove_outliers
from utils import derive_convex_hull

from DL.models import ObjRecEngine
# from boundingbox_msg.msg import ConvexHull

class ObjectRecognition(Node):

    def __init__(self, cliargs):

        super().__init__('ObjectRecognition')
        self.camintr = load_camera_intrinsics_txt(cliargs.intr_path)
        self.params = cliargs

        self.bridge = CvBridge()
        self.obj_engine = ObjRecEngine(cliargs)

        self.rgb_sub = message_filters.Subscriber(self, Image,cliargs.rgb_topic)
        self.pcl_sub = message_filters.Subscriber(self, PointCloud2,cliargs.pcl_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.pcl_sub],queue_size=1, slop=0.1) # sync rgb and pcl messages

        # self.pub_hull = self.create_publisher(ConvexHull, cliargs.chull_topic)

        # one callback for all
        self.ts.registerCallback(self.callback)


    def callback(self, img_msg, pcl_msg):

        cv2_im = self.bridge.imgmsg_to_cv2(img_msg)
        cv2_im_rgb_big = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        cv2_im_rgb = cv2.resize(cv2_im_rgb_big, self.obj_engine.input_size)


        objs = self.obj_engine.detect(cv2_im_rgb.tobytes())
        bboxes, dets = self.obj_engine.get_predictions(objs, cv2_im.shape)
        cv2_labeled_img = self.obj_engine.visualize_bboxes(cv2_im_rgb_big, bboxes,dets)

        #Visualize img annotated with bboxes
        cv2.imshow('win', cv2_labeled_img)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

        for coords, preds in zip(bboxes,dets):

            segmented_pcl = derive_object_cloud(coords, pcl_msg)
            filtered_pcl = remove_outliers(segmented_pcl,self.params)
            print(f'Filtered cloud of size: {len(filtered_pcl.points)}')
            chull_msg = derive_convex_hull(filtered_pcl,preds,pcl_msg)

            # self.pub_hull.publish(chull_msg)






