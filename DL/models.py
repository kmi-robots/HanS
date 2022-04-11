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

import cv2

class ObjRecEngine():

    def __init__(self,params):

        if params.impr_path is None:

            #run object recognition with same model
            self.interpreter = make_interpreter(params.model_path)
            self.interpreter.allocate_tensors()
            self.labels = read_label_file(params.classes)
            self.input_size = input_size(self.interpreter)
            self.threshold = params.conft

        else:
            #TODO run detection with bbox models and classify with imprinted model
            pass

        self.color = params.bbox_color
        self.thickness = params.bbox_thick

    def detect(self,bin_img):

        run_inference(self.interpreter, bin_img)

        return get_objects(self.interpreter, score_threshold=self.threshold)

    def get_predictions(self, objs_, shape):

        height, width, channels = shape
        # Adjust coordinates to image pre resizing
        scale_x, scale_y = width / self.input_size[0], height / self.input_size[1]

        bbox_list = []
        detections = []

        for obj in objs_:

            bbox = obj.bbox.scale(scale_x, scale_y)
            bbox_list.append([int(bbox.xmin), int(bbox.ymin), int(bbox.xmax), int(bbox.ymax)])
            detections.append((obj.id, obj.score))

        return bbox_list, detections

    def visualize_bboxes(self,cvimg, bbox_list, detection_list):

        for bbox_,pred in zip(bbox_list,detection_list):

            cvimg = cv2.rectangle(cvimg, tuple(bbox_[:2]), tuple(bbox_[2:4]), self.color, self.thickness)
            obj_class = self.labels.get(pred[0], pred[0])
            conf_score = pred[1]
            cv2.putText(cvimg, obj_class+'\t'+str(conf_score), (bbox_[0], bbox_[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.9, color=self.color, thickness=self.thickness)

        return cvimg
