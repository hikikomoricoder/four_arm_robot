"""
camera_info_corrector.py

Subscribes to a CameraInfo topic (typically bridged from Gazebo, which
publishes with Tx=0 for all cameras), replaces the calibration fields with
values loaded from a YAML calibration file, and republishes on the
standard topic — preserving the original header (timestamp + frame_id) so
that stereo_image_proc time-synchronization works correctly.

Usage in a launch file (XML):

    <node pkg="stereo_camera" exec="camera_info_corrector"
          name="camera_info_corrector_left" output="screen">
        <param name="calibration_file"
               value="$(find-pkg-share my_robot_bringup)/config/left_camera.yaml" />
        <param name="input_topic"  value="/_gz/camera_5/camera_info" />
        <param name="output_topic" value="/camera_5/camera_info" />
    </node>
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml


class CameraInfoCorrector(Node):
    """Relay node that replaces CameraInfo calibration with values from a YAML file.

    Gazebo simulates each camera as monocular and therefore publishes a
    CameraInfo where the projection matrix has Tx = 0.  For stereo
    algorithms (stereo_image_proc) the right-camera projection matrix must
    contain Tx = -fx * baseline.  This node corrects that on-the-fly while
    keeping the original message timestamp so that approximate-time
    synchronisers in the image pipeline continue to work.
    """

    def __init__(self):
        super().__init__('camera_info_corrector')

        self.declare_parameter('calibration_file', '')
        self.declare_parameter('input_topic', '/_gz/camera/camera_info')
        self.declare_parameter('output_topic', '/camera/camera_info')

        calib_path = self.get_parameter('calibration_file').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        if not calib_path:
            raise RuntimeError(
                'CameraInfoCorrector: parameter "calibration_file" is required'
            )

        # Load calibration from YAML (ROS camera_calibration format)
        with open(calib_path, 'r') as fh:
            data = yaml.safe_load(fh)

        self._width = int(data['image_width'])
        self._height = int(data['image_height'])
        self._distortion_model = data['distortion_model']
        self._k = list(data['camera_matrix']['data'])
        self._d = list(data['distortion_coefficients']['data'])
        self._r = list(data['rectification_matrix']['data'])
        self._p = list(data['projection_matrix']['data'])

        self._pub = self.create_publisher(CameraInfo, output_topic, 10)
        self._sub = self.create_subscription(
            CameraInfo, input_topic, self._callback, 10
        )

        self.get_logger().info(
            f'CameraInfoCorrector ready:\n'
            f'  input  : {input_topic}\n'
            f'  output : {output_topic}\n'
            f'  calib  : {calib_path}\n'
            f'  P[3]   : {self._p[3]:.4f}  (Tx)'
        )

    def _callback(self, msg: CameraInfo) -> None:
        # Keep header intact (timestamp from Gazebo + original frame_id)
        msg.width = self._width
        msg.height = self._height
        msg.distortion_model = self._distortion_model
        msg.k = self._k
        msg.d = self._d
        msg.r = self._r
        msg.p = self._p
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
