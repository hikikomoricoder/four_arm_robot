import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
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

        # --- TF-based camera azimuth monitoring (1 Hz) ---
        # Each camera's optical-axis azimuth is read from TF; when any
        # camera's azimuth drifts more than tf_change_thresh_deg from the
        # last recorded value, the stitch geometry (and with it the 10°
        # interval boundary table) is recomputed on the next frame.
        self.reference_frame = 'base_footprint'
        self.camera_frames = [f'camera_optical_link_{i}' for i in range(1, 5)]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_change_thresh_deg = 5.0
        self._recorded_azimuths = None                # absolute, in base frame
        self.axis_angles = [0.0, 90.0, 180.0, 270.0]  # relative to cam0 axis
        self.tf_timer = self.create_timer(1.0, self.check_camera_tf)

        # 10° interval visualisation on the displayed panorama:
        # semi-transparent colour bands blended with cv2.addWeighted
        self.draw_intervals = True
        self.interval_band_alpha = 0.25
        self._last_intervals = None

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
            self.detector = Yolo11TrtDetector(model_path, conf_thres=0.32, iou_thres=0.45,
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

                # 10° azimuth interval boundaries (cached inside the
                # stitcher; recomputed only when the geometry or the
                # TF-derived axis angles change beyond threshold)
                intervals = self.stitcher.get_interval_boundaries(self.axis_angles)
                if intervals is not None and intervals is not self._last_intervals:
                    self.get_logger().info(
                        'Interval boundaries (deg→X): ' + ', '.join(
                            f'{th:.0f}:{hits[0][1]:.0f}' if hits else f'{th:.0f}:--'
                            for th, hits in intervals))
                    self._last_intervals = intervals

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

                # Overlay the 10° intervals as semi-transparent colour
                # bands (cv2.addWeighted): each band spans two consecutive
                # primary boundary X positions; the duplicated wrap-around
                # strip at the opposite panorama edge is tinted orange.
                if self.draw_intervals and self._last_intervals:
                    scale_x = max_w / panorama.shape[1]
                    pano_w = panorama.shape[1]
                    band_colors = [  # 9 BGR colours → cycle repeats every 90°
                        (255, 128, 0), (0, 200, 255), (128, 255, 0),
                        (255, 0, 255), (0, 128, 255), (255, 255, 0),
                        (0, 255, 128), (255, 0, 128), (128, 128, 255),
                    ]
                    overlay = show_pano.copy()

                    # Main bands between consecutive primary boundaries
                    # (min/max: primary X may ascend or descend with theta)
                    for k in range(len(self._last_intervals) - 1):
                        hits0 = self._last_intervals[k][1]
                        hits1 = self._last_intervals[k + 1][1]
                        if not hits0 or not hits1:
                            continue
                        x0 = int(round(hits0[0][1] * scale_x))
                        x1 = int(round(hits1[0][1] * scale_x))
                        xa, xb = min(x0, x1), max(x0, x1)
                        xa, xb = max(xa, 0), min(xb, max_w)
                        if xb <= xa:
                            continue
                        cv2.rectangle(overlay, (xa, 0), (xb, max_h - 1),
                                      band_colors[k % len(band_colors)], -1)

                    # Duplicated wrap-band strips: second hits landing on
                    # the opposite panorama edge (|dX| > half the canvas)
                    dup_x = [
                        hits[1][1]
                        if hits and len(hits) > 1 and
                           abs(hits[1][1] - hits[0][1]) > pano_w * 0.5
                        else None
                        for _, hits in self._last_intervals
                    ]
                    for k in range(len(dup_x) - 1):
                        if dup_x[k] is None or dup_x[k + 1] is None:
                            continue
                        x0 = int(round(dup_x[k] * scale_x))
                        x1 = int(round(dup_x[k + 1] * scale_x))
                        xa, xb = min(x0, x1), max(x0, x1)
                        xa, xb = max(xa, 0), min(xb, max_w)
                        if xb <= xa:
                            continue
                        cv2.rectangle(overlay, (xa, 0), (xb, max_h - 1),
                                      (0, 165, 255), -1)

                    show_pano = cv2.addWeighted(
                        overlay, self.interval_band_alpha,
                        show_pano, 1.0 - self.interval_band_alpha, 0)

                    # Camera region-start labels on top of the blended
                    # bands: 4 labels at the midpoints between adjacent
                    # camera axes (nominal -45 / 45 / 135 / 225 deg)
                    starts = self.stitcher.get_camera_region_starts(
                        self.axis_angles)
                    if starts:
                        for ang, X in starts:
                            if X is None:
                                continue
                            sx = int(round(X * scale_x))
                            if 0 <= sx < max_w:
                                cv2.putText(show_pano, f'{ang:.0f}',
                                            (sx + 2, 15),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                            (255, 255, 255), 1)

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

    @staticmethod
    def _quat_to_rot(q):
        """Convert a geometry_msgs Quaternion to a 3x3 rotation matrix."""
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def _camera_azimuths(self):
        """Azimuth (deg) of each camera's optical axis projected onto the
        reference frame's XY plane (optical axis = +Z of the optical frame)."""
        azimuths = []
        for frame in self.camera_frames:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame, frame, rclpy.time.Time())
            rot = self._quat_to_rot(tf.transform.rotation)
            axis = rot[:, 2]
            azimuths.append(math.degrees(math.atan2(axis[1], axis[0])))
        return azimuths

    def check_camera_tf(self):
        """1 Hz poll: request a stitch recompute when any camera azimuth
        drifts more than ``tf_change_thresh_deg`` from the recorded value."""
        try:
            azimuths = self._camera_azimuths()
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().warn(f'Camera TF lookup failed: {e}',
                                   throttle_duration_sec=5.0)
            return

        if self._recorded_azimuths is None:
            self._recorded_azimuths = azimuths
            self.axis_angles = [(a - azimuths[0]) % 360.0 for a in azimuths]
            self.get_logger().info(
                'Initial camera azimuths (deg): '
                + ', '.join(f'{a:.1f}' for a in azimuths))
            return

        deltas = [abs((a - r + 180.0) % 360.0 - 180.0)
                  for a, r in zip(azimuths, self._recorded_azimuths)]
        if max(deltas) > self.tf_change_thresh_deg:
            self.get_logger().info(
                'Camera azimuth drift ('
                + ', '.join(f'{d:.1f}' for d in deltas)
                + f') > {self.tf_change_thresh_deg:.0f} deg; '
                  'requesting stitch recompute')
            self._recorded_azimuths = azimuths
            self.axis_angles = [(a - azimuths[0]) % 360.0 for a in azimuths]
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
