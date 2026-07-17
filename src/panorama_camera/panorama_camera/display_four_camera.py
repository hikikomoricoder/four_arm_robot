import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
import numpy as np
import time

from panorama_camera.four_camera_concat import FourCameraStitcher
from panorama_camera.detect_locate import Yolo11OnnxDetector, Yolo11TrtDetector

class DisplayFourCamera(Node):
    def __init__(self):
        super().__init__('display_four_camera')
        self.bridge = CvBridge()

        self.camera_topics = [
            '/camera_1/image_raw',
            '/camera_2/image_raw',
            '/camera_3/image_raw',
            '/camera_4/image_raw',
        ]

        self.images = [None] * 4
        self.global_iou_thres = 0.35

        self.subs = []
        for i, topic in enumerate(self.camera_topics):
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, idx=i: self.image_callback(msg, idx),
                10
            )
            self.subs.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')

        # Timer to refresh display at ~15 fps
        self.timer = self.create_timer(0.066, self.display_cameras)

        # Stitcher: computes the panorama geometry once and reuses it
        self.stitcher = FourCameraStitcher()
        self.recompute_service = self.create_service(
            Trigger, 'recompute_stitch', self.handle_recompute_stitch)

        # Optimisation: avoid redundant stitching when no new frame arrived
        self._images_updated = False
        self._last_panorama = None
        self._last_annotated_pano = None
        self._updated_indices = [False] * 4  # all 4 cameras must refresh before stitch
        
        self.if_det = True
        self.if_seg = False

        if self.if_det:
            # model_path = os.path.join(
            #     os.path.dirname(os.path.abspath(__file__)),
            #     "../../../../../../model_weights/gazebo_room_coco.onnx"
            # )
            # self.detector = Yolo11OnnxDetector(model_path, conf_thres=0.35, iou_thres=0.45,
            #                             global_iou_thres=self.global_iou_thres,
            #                             logger=self.get_logger())
            model_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "../../../../../../model_weights/gazebo_room_coco.engine"
            )
            self.detector = Yolo11TrtDetector(model_path, conf_thres=0.35, iou_thres=0.45,
                                        global_iou_thres=self.global_iou_thres,
                                        logger=self.get_logger())

    def image_callback(self, msg, index):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.images[index] = cv_img
            self._updated_indices[index] = True
            # Only trigger stitch when all 4 cameras have sent a new frame
            if all(self._updated_indices):
                self._images_updated = True
                self._updated_indices = [False] * 4
        except Exception as e:
            self.get_logger().error(f'Failed to convert image from camera {index + 1}: {e}')

    def display_cameras(self):
        display_imgs = []
        # for i in range(4):
        #     if self.images[i] is not None:
        #         img = self.images[i].copy()
        #         h, w = img.shape[:2]
        #         cv2.putText(img, f'Camera {i + 1}', (10, 30),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        #         display_imgs.append(img)
        #     else:
        #         # Show placeholder while waiting for first image
        #         blank = np.zeros((480, 640, 3), dtype=np.uint8)
        #         cv2.putText(blank, f'Camera {i + 1} - Waiting...', (10, 30),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        #         display_imgs.append(blank)

        # # Arrange 4 cameras in a 2x2 grid
        # top = np.hstack((display_imgs[0], display_imgs[1]))
        # bottom = np.hstack((display_imgs[2], display_imgs[3]))
        # grid = np.vstack((top, bottom))

        # cv2.imshow('Four Camera View', grid)

        # Compute panorama only when all 4 cameras have new data (~1 Hz)
        if self._images_updated:
            self._images_updated = False
            t1 = time.time()
            panorama = self.stitcher.stitch(self.images)
            self.get_logger().info('stitch time: {:.2f} ms'.format((time.time() - t1) * 1000))
            
            if panorama is not None:
                self._last_panorama = panorama

                # Detection and annotation also run only on fresh panorama
                max_w, max_h = 1440, 480
                t1 = time.time()
                show_pano = cv2.resize(panorama, (max_w, max_h))

                if self.if_det:
                    detections = self.detector.detect_panorama(show_pano)

                    self.get_logger().info(f'Detections: {detections}')
                    for det in detections:
                        x1, y1, x2, y2 = det['bbox']
                        cv2.rectangle(show_pano, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"{det['class_name']} {det['score']:.2f}"
                        cv2.putText(
                            show_pano, label,
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                        )

                self._last_annotated_pano = show_pano
                self.get_logger().info('det time: {:.2f} ms'.format((time.time() - t1) * 1000))

            # Display the latest result (refreshes window at ~15 Hz even when no new data)
            if self._last_annotated_pano is not None:
                cv2.imshow('Panorama', self._last_annotated_pano)
            else:
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                status = self.stitcher.get_status()
                cv2.putText(placeholder, f'Panorama - {status}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow('Panorama', placeholder)

            key = cv2.waitKey(1)
            if key != -1 and (key & 0xFF) == ord('r'):
                self.get_logger().info(
                    'Keyboard recompute requested; stitch geometry will be recalculated.')
                self.stitcher.request_recompute()

    def handle_recompute_stitch(self, request, response):
        """ROS service callback to request a recomputation of the stitch geometry."""
        self.stitcher.request_recompute()
        response.success = True
        response.message = 'Stitch geometry recomputation requested.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DisplayFourCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
