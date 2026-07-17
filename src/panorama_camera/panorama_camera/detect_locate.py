import sys
import os
import cv2
import numpy as np

_BUILD_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../../../../..", "extend", "trt_inference", "build", "lib.linux-x86_64-cpython-312")
)

sys.path.insert(0, _BUILD_DIR)

import Yolo11DetTrt

from panorama_camera.yolo_onnx import YOLOv11ONNX, COCO_NAMES


class Yolo11OnnxDetector:
    """YOLOv11 ONNX detector specialised for panorama images.

    The panorama is split into 4 overlapping 480×480 sub-images so that
    each sub-image matches the model’s native input size without letterbox
    waste.  Detections from all sub-images are merged with a second-pass
    global NMS to eliminate duplicates that span overlapping boundaries.
    """

    def __init__(self, model_path, conf_thres=0.35, iou_thres=0.45,
                 global_iou_thres=0.35, logger=None):
        self._det = YOLOv11ONNX(model_path, conf_thres=conf_thres,
                                iou_thres=iou_thres, logger=logger)
        self.global_iou_thres = global_iou_thres
        self.logger = logger

    def detect_panorama(self, panorama):
        """Run detection on a panorama image.

        The panorama is split into 4 overlapping 480×480 sub-images.
        Returns a list of detection dicts with bbox in panorama coordinates.
        """
        if panorama is None:
            return []

        h, w = panorama.shape[:2]
        sub_size = 480
        all_detections = []

        if w <= sub_size:
            # Panorama fits in a single sub-image
            dets, _ = self._det.detect(panorama)
            return dets

        # Split into 4 evenly-spaced overlapping sub-images
        stride_x = (w - sub_size) // 3

        for i in range(4):
            x_start = i * stride_x
            x_end = x_start + sub_size

            # Clamp last sub-image to panorama bounds
            if x_end > w:
                x_start = w - sub_size
                x_end = w

            sub_img = panorama[0:sub_size, x_start:x_end]
            sub_dets, _ = self._det.detect(sub_img)

            # Offset bounding boxes to panorama coordinates
            for det in sub_dets:
                det['bbox'][0] += x_start
                det['bbox'][2] += x_start

            all_detections.extend(sub_dets)

        return self._global_nms(all_detections)

    def _global_nms(self, detections):
        """Apply global NMS on detections collected from all sub-images.

        Objects that span across overlapping sub-image boundaries will be
        detected multiple times.  A second-pass NMS over the panorama-space
        coordinates merges these duplicates into a single detection.
        """
        if len(detections) == 0:
            return []

        # cv2.dnn.NMSBoxes expects [x, y, w, h] format
        boxes_xywh = [
            [b[0], b[1], b[2] - b[0], b[3] - b[1]]
            for b in (det['bbox'] for det in detections)
        ]
        scores = [det['score'] for det in detections]

        indices = cv2.dnn.NMSBoxes(boxes_xywh, scores,
                                   self._det.conf_thres, self.global_iou_thres)

        if len(indices) == 0:
            return []

        return [detections[i] for i in indices.flatten()]


class Yolo11TrtDetector:
    """YOLOv11 TensorRT detector specialised for panorama images.

    The panorama is split into 4 overlapping 480×480 sub-images so that
    each sub-image is processed by the TRT engine efficiently.  Detections
    from all sub-images are merged with a second-pass global NMS to eliminate
    duplicates that span overlapping boundaries.
    CUDA-accelerated inference is used throughout.
    """

    def __init__(self, engine_path, conf_thres=0.35, iou_thres=0.45,
                 global_iou_thres=0.35, num_classes=81, logger=None):
        self._det = Yolo11DetTrt.YOLO11TRT(
            engine_path, conf_thresh=conf_thres, iou_thresh=iou_thres,
            num_classes=num_classes
        )
        self.global_iou_thres = global_iou_thres
        self.logger = logger
        self.conf_thres = conf_thres
        self.class_names = COCO_NAMES

    def _class_name(self, cls_id):
        """Look up class name from COCO names list."""
        return self.class_names[cls_id] if cls_id < len(self.class_names) else str(cls_id)

    def detect_panorama(self, panorama):
        """Run detection on a panorama image using TRT CUDA acceleration.

        The panorama is split into 4 overlapping 480×480 sub-images.
        Returns a list of detection dicts with bbox in panorama coordinates.
        """
        if panorama is None:
            return []

        h, w = panorama.shape[:2]
        sub_size = 480
        all_detections = []

        if w <= sub_size:
            # Panorama fits in a single sub-image
            dets = self._det.detect_cuda(panorama)
            return self._convert_detections(dets)

        # Split into 4 evenly-spaced overlapping sub-images
        stride_x = (w - sub_size) // 3

        for i in range(4):
            x_start = i * stride_x
            x_end = x_start + sub_size

            # Clamp last sub-image to panorama bounds
            if x_end > w:
                x_start = w - sub_size
                x_end = w

            sub_img = panorama[0:sub_size, x_start:x_end]
            sub_dets = self._det.detect_cuda(sub_img)

            # Convert TRT detection format and offset to panorama coordinates
            for det in sub_dets:
                cls_id = det['class_id']
                all_detections.append({
                    'bbox': [int(det['x1']) + x_start, int(det['y1']),
                             int(det['x2']) + x_start, int(det['y2'])],
                    'score': det['score'],
                    'class_id': cls_id,
                    'class_name': self._class_name(cls_id),
                })

        return self._global_nms(all_detections)

    def _convert_detections(self, dets):
        """Convert TRT detections to standard format with 'bbox' and 'class_name' keys."""
        return [
            {
                'bbox': [int(d['x1']), int(d['y1']), int(d['x2']), int(d['y2'])],
                'score': d['score'],
                'class_id': d['class_id'],
                'class_name': self._class_name(d['class_id']),
            }
            for d in dets
        ]

    def _global_nms(self, detections):
        """Apply global NMS on detections collected from all sub-images.

        Objects that span across overlapping sub-image boundaries will be
        detected multiple times.  A second-pass NMS over the panorama-space
        coordinates merges these duplicates into a single detection.
        """
        if len(detections) == 0:
            return []

        # cv2.dnn.NMSBoxes expects [x, y, w, h] format
        boxes_xywh = [
            [b[0], b[1], b[2] - b[0], b[3] - b[1]]
            for b in (det['bbox'] for det in detections)
        ]
        scores = [det['score'] for det in detections]

        indices = cv2.dnn.NMSBoxes(boxes_xywh, scores,
                                   self.conf_thres, self.global_iou_thres)

        if len(indices) == 0:
            return []

        return [detections[i] for i in indices.flatten()]


if __name__ == "__main__":
    pass
