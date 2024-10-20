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
from pycoral.adapters.classify import get_classes
from pycoral.adapters.detect import BBox
from pycoral.adapters import common

import cv2
import numpy as np
import collections
import random

DetectionWithMask = collections.namedtuple('DetectionWithMask',
                                           ['pred_ranking',
                                            'bbox',
                                            'mask'])
DetObject = collections.namedtuple('DetectionObject',
                                   ['pred_ranking',
                                    'bbox'])


class ObjRecEngine:

    def __init__(self, params):

        # Object classification on Edge TPU
        if params.classif_path is None:
            self.interpreter = make_interpreter(params.model_path)
            self.segm_model = None
        else:
            # BBox detection via openCV
            self.segm_model = cv2.dnn.readNet(model=params.segm_model,
                                              config=params.segm_config,
                                              framework='TensorFlow')
            self.interpreter = make_interpreter(params.classif_path)

        self.segm_size = params.segm_size
        self.segm_mean = params.segm_mean
        self.threshold = params.conft

        self.interpreter.allocate_tensors()
        self.labels = read_label_file(params.classes)
        self.input_size = input_size(self.interpreter)

        self.color = params.bbox_color
        self.thickness = params.bbox_thick

    def detect_objects(self, rgb_img):

        height, width, channels = rgb_img.shape
        scalex, scaley = width / self.input_size[0], height / self.input_size[1]

        """Apply saliency detection to focus only on subportions of the image"""

        """saliency = cv2.saliency.StaticSaliencyFineGrained_create()
        (success, saliencyMap) = saliency.computeSaliency(rgb_img)
        saliencyMapforbin = (saliencyMap * 255).astype("uint8")

        threshMap = cv2.threshold(saliencyMapforbin, 0, 255,
                                  cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

        rgb_img_masked = cv2.bitwise_and(rgb_img, rgb_img, mask=threshMap)
        """
        """cv2.imshow("Thresh", rgb_img)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()"""


        if self.segm_model is None: #detection+classification done together on TPU, e.g., ssd or yolo-like model
            resized_img = cv2.resize(rgb_img, self.input_size)
            run_inference(self.interpreter, resized_img.tobytes())
            # obj_list = get_objects(self.interpreter, score_threshold=self.threshold)
            obj_list_pre = self.get_objects_extended(self.interpreter, score_threshold=self.threshold)
            # scale bbox again from 300x300 to full image size
            obj_list = [DetObject(
                pred_ranking=obj_.pred_ranking,
                bbox=obj_.bbox.scale(scalex, scaley)) for obj_ in obj_list_pre]

        else:

            # masked image after saliency used for segmentation, but bounding box cropped on original image pre saliency
            # (we found it tends to perform better for bbox classification accuracy)

            # Detect bboxes with openCV first then classify single boxes with imprinted classifier
            blob = cv2.dnn.blobFromImage(image=rgb_img, crop=False)
            self.segm_model.setInput(blob)
            (output, masks) = self.segm_model.forward(["detection_out_final", "detection_masks"])

            obj_list = []
            for i, detection in enumerate(output[0, 0, :, :]):  # loop over each of the detection
                confidence = detection[2]  # extract the confidence of the detection
                class_id = int(detection[1])
                if confidence > self.threshold:
                    # get the bounding box coordinates & normalise to image width and height
                    bounding_box = detection[3:7] * np.array([width, height, width, height])
                    (b_x, b_y, b_x2, b_y2) = bounding_box.astype("int")
                    mask = masks[i, class_id]

                    # avoid overflowing bboxes
                    if b_x <= 0:
                        b_x = 1
                    if b_y <= 0:
                        b_y = 1
                    if b_x2 >= width:
                        b_x2 = int(width - 1)
                    if b_y2 >= height:
                        b_y2 = int(height - 1)

                    box_w = b_x2 - b_x
                    box_h = b_y2 - b_y

                    mask = cv2.resize(mask, (box_w, box_h), interpolation=cv2.INTER_NEAREST)
                    mask = (mask > 0.5)

                    obj_roi = rgb_img.copy()
                    obj_roi = obj_roi[b_y:b_y2, b_x:b_x2, :]

                    resized_roi = cv2.resize(obj_roi, self.input_size)
                    run_inference(self.interpreter, resized_roi.tobytes())
                    all_class_ranking = get_classes(self.interpreter)
                    all_class_ranking = [(cl.id, cl.score) for cl in all_class_ranking]
                    obj_list.append(DetectionWithMask(
                        pred_ranking=all_class_ranking,
                        bbox=BBox(xmin=b_x, ymin=b_y, xmax=b_x2, ymax=b_y2).map(int),
                        mask=mask))

        return obj_list

    def visualize_bboxes(self, cvimg, res_list):

        for o_ in res_list:
            bbox_ = [int(o_.bbox.xmin), int(o_.bbox.ymin), int(o_.bbox.xmax), int(o_.bbox.ymax)]
            clone = cvimg.copy()
            roi = clone[bbox_[1]:bbox_[3],bbox_[0]:bbox_[2]]
            cvimg = cv2.rectangle(cvimg, tuple(bbox_[:2]), tuple(bbox_[2:4]), self.color, self.thickness)

            roi = roi[o_.mask]
            blended = ((0.4 * np.array(self.color)) + (0.6 * roi)).astype("uint8")
            cvimg[bbox_[1]:bbox_[3],bbox_[0]:bbox_[2]][o_.mask] = blended

            topclass = self.labels[o_.pred_ranking[0][0]]
            conf_score = o_.pred_ranking[0][1]
            cv2.putText(cvimg, str(topclass) + '  ' + str(conf_score), (bbox_[0], bbox_[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.9, color=self.color, thickness=self.thickness)

        return cvimg

    def get_objects_extended(self, interpreter,
                             score_threshold=-float('inf'),
                             image_scale=(1.0, 1.0)):
        """Slightly modified the pycoral.adapters.detect get_objects method to name the top1 as pred_ranking,
        although all pycoral detection models only allow to extract 1 (top-1) prediction per bounding box


        Gets results from a detection model as a list of detected objects.
        Args:
          interpreter: The ``tf.lite.Interpreter`` to query for results.
          score_threshold (float): The score threshold for results. All returned
            results have a score greater-than-or-equal-to this value.
          image_scale (float, float): Scaling factor to apply to the bounding boxes as
            (x-scale-factor, y-scale-factor), where each factor is from 0 to 1.0.
        Returns:
          A list of :obj:`Object` objects, which each contains the detected object's
          id, score, and bounding box as :obj:`BBox`.
        """
        # If a model has signature, we use the signature output tensor names to parse
        # the results. Otherwise, we parse the results based on some assumption of the
        # output tensor order and size.
        # pylint: disable=protected-access

        signature_list = interpreter._get_full_signature_list()
        # pylint: enable=protected-access

        if signature_list:
            if len(signature_list) > 1:
                raise ValueError('Only support model with one signature.')
            signature = signature_list[next(iter(signature_list))]
            count = int(interpreter.tensor(signature['outputs']['output_0'])()[0])
            scores = interpreter.tensor(signature['outputs']['output_1'])()[0]
            class_ids = interpreter.tensor(signature['outputs']['output_2'])()[0]
            boxes = interpreter.tensor(signature['outputs']['output_3'])()[0]
        elif common.output_tensor(interpreter, 3).size == 1:  # then out no. 3 is the count, i.e., tot no of boxes
            boxes = common.output_tensor(interpreter, 0)[0]
            class_ids = common.output_tensor(interpreter, 1)[0]
            scores = common.output_tensor(interpreter, 2)[0]
            count = int(common.output_tensor(interpreter, 3)[0])
        else:
            scores = common.output_tensor(interpreter, 0)[0]  # no. 2 is the count and 3 is the list of category ids
            boxes = common.output_tensor(interpreter, 1)[0]
            count = (int)(common.output_tensor(interpreter, 2)[0])
            class_ids = common.output_tensor(interpreter, 3)[0]

        # output pre-division by box
        print(interpreter.get_output_details())
        width, height = common.input_size(interpreter)
        image_scale_x, image_scale_y = image_scale
        sx, sy = width / image_scale_x, height / image_scale_y

        def make(i):
            ymin, xmin, ymax, xmax = boxes[i]
            return DetObject(
                pred_ranking=[(self.labels.get(class_ids[i], class_ids[i]), scores[i])],
                bbox=BBox(xmin=xmin, ymin=ymin, xmax=xmax,
                          ymax=ymax).scale(sx, sy).map(int))

        return [make(i) for i in range(count) if scores[i] >= score_threshold]
