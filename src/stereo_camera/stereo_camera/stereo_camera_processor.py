"""
stereo_camera_processor.py

Stereo ranging node for the four-arm robot project.

Pipeline (all launched from my_robot_gazebo.launch.xml):
    Gazebo camera_5/camera_6  --(ros_gz_bridge)-->  /_gz/camera_5|6/camera_info
    camera_info_corrector     -->  /camera_5|6/camera_info  (Tx = -fx * baseline)
    stereo_image_proc disparity_node  -->  /disparity
    THIS node:
        * time-syncs the left image (/camera_5/image_raw) with /disparity
        * runs YOLO11 TensorRT detection on the left image (single frame)
        * estimates the metric distance of every detection from the disparity
        * publishes the annotated left frame on /stereo_camera/detect_estimate

Detection + ranging are gated by the ``stereo_estimate`` parameter (default
False, toggled from the robot panel).  The TensorRT engine itself is loaded
once at startup regardless, so enabling the switch later incurs no load
delay.

The detector reuses panorama_camera.detect_locate.Yolo11TrtDetector which
wraps the CUDA-accelerated Yolo11DetTrt extension.  For stereo ranging we
detect on a single camera view, so ``detect_single`` is used (no panorama
sub-image splitting).
"""

import os

import cv2
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError

from panorama_camera.detect_locate import Yolo11TrtDetector
from stereo_camera.stereo_distance_estimator import StereoDistanceEstimator

# Default YOLO11 TensorRT engine (shared with the panorama detector).
# The module depth differs between source (src/...) and installed
# (install/.../site-packages/...) layouts, so instead of a fixed relative
# path we walk up the directory tree until model_weights/<engine> is found.
def _find_default_engine(filename="gazebo_room_coco.engine"):
    cur = os.path.dirname(os.path.abspath(__file__))
    while True:
        candidate = os.path.join(cur, "model_weights", filename)
        if os.path.exists(candidate):
            return candidate
        parent = os.path.dirname(cur)
        if parent == cur:  # reached filesystem root
            return os.path.join("model_weights", filename)
        cur = parent


_DEFAULT_ENGINE = _find_default_engine()


class StereoCameraProcessor(Node):
    """Detects objects on the left stereo image and ranges them via disparity."""

    def __init__(self):
        super().__init__("stereo_camera_processor")

        # --- Parameters -------------------------------------------------
        self.declare_parameter("model_path", _DEFAULT_ENGINE)
        self.declare_parameter("conf_thres", 0.32)
        self.declare_parameter("iou_thres", 0.45)
        # camera_5/camera_6 geometry:
        #   fx = 381.3 px (horizontal_fov = 1.3962634 rad, width = 640)
        #   baseline = 0.08 m (camera_5 y = +0.04, camera_6 y = -0.04)
        self.declare_parameter("focal_length", 381.3)
        self.declare_parameter("baseline", 0.08)
        self.declare_parameter("left_image_topic", "/camera_5/image_raw")
        self.declare_parameter("disparity_topic", "/disparity")
        # Gates detection + ranging (default off, toggled from the robot
        # panel).  The TensorRT engine still loads below, so enabling the
        # switch later incurs no load delay.
        self.declare_parameter("stereo_estimate", False)

        model_path = self.get_parameter("model_path").value
        conf_thres = self.get_parameter("conf_thres").value
        iou_thres = self.get_parameter("iou_thres").value
        focal_length = self.get_parameter("focal_length").value
        baseline = self.get_parameter("baseline").value
        left_topic = self.get_parameter("left_image_topic").value
        disp_topic = self.get_parameter("disparity_topic").value

        # --- Detector (TensorRT / CUDA) ---------------------------------
        self.detector = Yolo11TrtDetector(
            model_path, conf_thres=conf_thres, iou_thres=iou_thres,
            logger=self.get_logger(),
        )

        # --- Distance estimator -----------------------------------------
        self.distance_estimator = StereoDistanceEstimator(
            focal_length=focal_length,
            baseline=baseline,
        )

        self.bridge_left = CvBridge()
        self.frame_counter = 0

        # --- Callback groups --------------------------------------------
        # ReentrantCallbackGroup for the two message_filters subscribers so
        # they can run concurrently.
        self.sync_callback_group = ReentrantCallbackGroup()

        # --- Time sync: left image + disparity --------------------------
        # ApproximateTimeSynchronizer only fires when the two frames have
        # matching timestamps, avoiding stale-disparity distance jumps.
        self.left_filter = message_filters.Subscriber(
            self, Image, left_topic,
            callback_group=self.sync_callback_group,
        )
        self.disp_filter = message_filters.Subscriber(
            self, DisparityImage, disp_topic,
            callback_group=self.sync_callback_group,
        )
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_filter, self.disp_filter],
            queue_size=10,
            slop=0.2,
        )
        self.time_sync.registerCallback(self.synchronized_callback)

        # --- Result publisher (replaces the former cv2.imshow GUI) -------
        # Annotated left frame, published every synced frame so remote
        # displays (e.g. the robot panel) can subscribe and render it.
        self.estimate_pub = self.create_publisher(
            Image, "/stereo_camera/detect_estimate", 10,
        )

        self.get_logger().info(
            "StereoCameraProcessor started:\n"
            f"  left     : {left_topic}\n"
            f"  disparity: {disp_topic}\n"
            f"  engine   : {model_path}\n"
            f"  publish  : /stereo_camera/detect_estimate\n"
            f"  fx={focal_length:.1f} px, baseline={baseline:.3f} m"
        )

    # ------------------------------------------------------------------
    # Core callback: left image + disparity timestamp-aligned
    # ------------------------------------------------------------------
    def synchronized_callback(self, img_msg: Image, disparity_msg: DisparityImage):
        try:
            cv_image = self.bridge_left.imgmsg_to_cv2(
                img_msg, desired_encoding="bgr8"
            )
            self.frame_counter += 1

            # Detection + ranging run only while stereo_estimate is on
            # (default off, toggled from the robot panel).  The engine is
            # already loaded at startup, so there is no load delay here.
            if self.get_parameter("stereo_estimate").value:
                # Update the estimator with the disparity aligned to this frame.
                self.distance_estimator.update_disparity(disparity_msg)

                # Single-frame TensorRT detection on the left image.
                detections = self.detector.detect_single(cv_image)

                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    distance = self.distance_estimator.estimate_distance(
                        (x1, y1, x2, y2)
                    )

                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{det['class_name']} {det['score']:.2f}"
                    if distance is not None:
                        label += f" {distance:.2f}m"
                    else:
                        label += " dist:N/A"
                    cv2.putText(
                        cv_image, label, (x1, max(y1 - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                    )

                if detections:
                    self.get_logger().info(
                        "Detections: " + ", ".join(
                            f"{d['class_name']} "
                            f"{self.distance_estimator.estimate_distance(tuple(d['bbox'])) or float('nan'):.2f}m"
                            for d in detections
                        )
                    )

            # Publish the annotated frame regardless of stereo_estimate so
            # remote displays always have a live feed; bounding boxes and
            # distances only appear while the switch is enabled.
            out_msg = self.bridge_left.cv2_to_imgmsg(cv_image, encoding="bgr8")
            out_msg.header = img_msg.header
            self.estimate_pub.publish(out_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error in synchronized_callback: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in synchronized_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraProcessor()

    # The sync callback group needs 2 concurrent threads (left + disparity
    # filter subscribers).
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
