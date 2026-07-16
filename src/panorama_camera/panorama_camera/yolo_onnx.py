import cv2
import numpy as np
import sys
import logging

# for path in sys.path:
#     print(path, flush=True)
import onnxruntime as ort
import time

# Default COCO class names used by YOLOv11
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush", "checkboard"
]


class YOLOv11ONNX:
    def __init__(self, model_path, conf_thres=0.25, iou_thres=0.45, input_size=(480, 480),
                 logger=None, class_names=None):
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.input_size = input_size
        self.logger = logger
        self.class_names = class_names if class_names is not None else COCO_NAMES

        # 1. 检查当前 ONNX Runtime 的版本
        self._log(f"ONNX Runtime 版本: {ort.__version__}")

        # 2. 检查当前可用的执行提供器（如果包含 CUDAExecutionProvider，说明 GPU 配置成功）
        self._log(f"可用的执行提供器: {ort.get_available_providers()}")

        # 3. 检查底层识别到的设备
        self._log(f"识别到的设备: {ort.get_device()}")
        # Load ONNX model
        self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])  # 或 CUDAExecutionProvider
        # self.session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])  # 或 CUDAExecutionProvider
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def _log(self, msg):
        """统一日志输出：优先使用 ROS2 logger，否则用 print"""
        if self.logger is not None:
            self.logger.info(msg)
        else:
            print(msg, flush=True)

    def preprocess(self, img):
        """
        Preprocess image: resize with letterbox and normalize to [0,1]
        """
        h, w = img.shape[:2]
        input_h, input_w = self.input_size

        # Letterbox resize
        scale = min(input_w / w, input_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        resized_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Padding to input_size
        dw = input_w - new_w
        dh = input_h - new_h
        top, bottom = dh // 2, dh - dh // 2
        left, right = dw // 2, dw - dw // 2
        padded_img = cv2.copyMakeBorder(resized_img, top, bottom, left, right,
                                        cv2.BORDER_CONSTANT, value=(114, 114, 114))

        # HWC to CHW, BGR to RGB, normalize to [0,1]
        input_tensor = cv2.cvtColor(padded_img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        input_tensor = np.transpose(input_tensor, (2, 0, 1))  # CHW
        input_tensor = np.expand_dims(input_tensor, axis=0)   # BCHW

        return input_tensor, scale, (left, top)

    def postprocess(self, outputs, scale, pad, original_shape):
        """
        Postprocess output from ONNX model.
        YOLOv11 ONNX output shape: [1, 84, 8400] -> 84 = 4 (box) + 80 (classes)
        """
        outputs = np.squeeze(outputs).T  # [8400, 84]

        # Filter by confidence
        max_conf = np.max(outputs[:, 4:], axis=1)
        valid_idx = max_conf >= self.conf_thres
        outputs = outputs[valid_idx]

        if outputs.shape[0] == 0:
            return []

        # Get boxes and scores
        boxes = outputs[:, :4]
        scores = np.max(outputs[:, 4:], axis=1)
        class_ids = np.argmax(outputs[:, 4:], axis=1)

        # Convert YOLO format (cx, cy, w, h) to (x1, y1, x2, y2)
        boxes[:, 0] -= boxes[:, 2] / 2  # x1 = cx - w/2
        boxes[:, 1] -= boxes[:, 3] / 2  # y1 = cy - h/2
        boxes[:, 2] += boxes[:, 0]      # x2 = x1 + w
        boxes[:, 3] += boxes[:, 1]      # y2 = y1 + h

        # Scale back to original image
        pad_w, pad_h = pad
        boxes -= np.array([pad_w, pad_h, pad_w, pad_h])
        boxes /= scale

        # Clip boxes to image bounds
        h, w = original_shape[:2]
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, w)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, h)

        # NMS expects [x, y, w, h] format; convert from [x1, y1, x2, y2]
        boxes_xywh = np.column_stack((
            boxes[:, 0],
            boxes[:, 1],
            boxes[:, 2] - boxes[:, 0],
            boxes[:, 3] - boxes[:, 1]
        ))
        indices = cv2.dnn.NMSBoxes(
            boxes_xywh.tolist(),
            scores.tolist(),
            self.conf_thres,
            self.iou_thres
        )

        detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                x1, y1, x2, y2 = boxes[i].astype(int)
                conf = float(scores[i])
                cls = int(class_ids[i])
                cls_name = self.class_names[cls] if cls < len(self.class_names) else str(cls)
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'score': conf,
                    'class_id': cls,
                    'class_name': cls_name,
                })

        return detections

    def detect(self, img):

        original_shape = img.shape
        input_tensor, scale, pad = self.preprocess(img)

        # Inference
        outputs = self.session.run([self.output_name], {self.input_name: input_tensor})[0]

        # Postprocess
        detections = self.postprocess(outputs, scale, pad, original_shape)
        return detections, img
