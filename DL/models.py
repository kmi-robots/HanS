"""
Based on object detection model zoo for Edge TPU by pycoral
https://coral.ai/models/object-detection/

Models available at Apr 11th, 2022.
Testing with:
- SSD MobileNet V1
- SSD/FPN MobileNet V1
- SSD MobileNet V2
- SSDLite MobileDet
- EfficientDet-Lite0
- EfficientDet-Lite1
- EfficientDet-Lite2
- EfficientDet-Lite3
- EfficientDet-Lite3x

All pre-trained on COCO.
Additional approach: use the above only for bounding boxes
And weight imprinted classifier to classify the bounding boxes
"""

from pycoral.utils.edgetpu import make_interpreter, run_inference
from pycoral.utils.dataset import read_label_file
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.adapters.classify import get_classes


import cv2
import numpy as np

class ObjRecEngine():

    def __init__(self,params):

        #BBox detection via openCV
        self.segm_model = cv2.dnn.readNet(model=params.segm_model,
                                          config=params.segm_config,
                                          framework='TensorFlow')
        self.segm_size = params.segm_size
        self.segm_mean =  params.segm_mean
        self.threshold = params.conft

        #Object classification on Edge TPU
        self.interpreter = make_interpreter(params.model_path)
        self.interpreter.allocate_tensors()
        self.labels = read_label_file(params.classes)
        self.input_size = input_size(self.interpreter)

        self.color = params.bbox_color
        self.thickness = params.bbox_thick

    def detect(self,rgb_img):

        height, width, channels = rgb_img.shape

        blob = cv2.dnn.blobFromImage(image=rgb_img, size=self.segm_size, mean=self.segm_mean)
        self.segm_model.setInput(blob)

        output = self.segm_model.forward()

        bbox_list = []
        for detection in output[0, 0, :, :]:  # loop over each of the detection

            confidence = detection[2]  # extract the confidence of the detection
            if confidence > self.threshold:
                # get the bounding box coordinates & normalise to image width and height
                bounding_box = detection[3:7] * np.array([width, height, width, height])
                (b_x, b_y, b_x2, b_y2) = bounding_box.astype("int")
                # avoid overflowing bboxes
                if b_x <= 0:  b_x = 1
                if b_y <= 0: b_y = 1
                if b_x2 >= width: b_x2 = int(width - 1)
                if b_y2 >= height: b_y2 = int(height - 1)

                bbox_list.append([b_x, b_y, b_x2, b_y2])

        return bbox_list

    def classify_bbox(self, cropped_img):

        resized_roi = cv2.resize(cropped_img, self.input_size)
        run_inference(self.interpreter, resized_roi.tobytes())
        all_class_ranking = get_classes(self.interpreter)
        all_class_ranking = [(self.labels.get(cl.id, cl.id), cl.score) for cl in all_class_ranking]

        return all_class_ranking


    def visualize_bboxes(self,cvimg, bbox_list):

        for bbox_ in bbox_list:

            cvimg = cv2.rectangle(cvimg, tuple(bbox_[:2]), tuple(bbox_[2:4]), self.color, self.thickness)
            #cv2.putText(cvimg, oclass+'\t'+str(conf_score), (bbox_[0], bbox_[1] - 10),
            #            cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.9, color=self.color, thickness=self.thickness)

        return cvimg
