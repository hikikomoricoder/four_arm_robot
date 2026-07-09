import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError  # 用于转换
import cv2  # OpenCV
from functools import partial
import sys


class CameraProcessor(Node):
    def __init__(self):
        super().__init__("camera_processor")
        self.get_logger().info("import path")
        for path in sys.path:
            self.get_logger().warn("import path "+path)

        # 创建 CvBridge 实例
        self.bridge_list = [CvBridge(), CvBridge()]
        self.image_counter = [0, 0]
        self.image_list = [None, None]

        # 订阅摄像头图像话题
        # 注意：话题名称需要与 Gazebo 中发布的完全一致
        # 可以用 `ros2 topic list` 命令查看
        self.subscription_1 = self.create_subscription(
            Image,
            "/camera_5/image_raw",  # <-- 确保这是正确的topic名称
            partial(self.image_callback, camera_id=1),
            10,
        )
        self.subscription_2 = self.create_subscription(
            Image,
            "/camera_6/image_raw",  # <-- 确保这是正确的topic名称
            partial(self.image_callback, camera_id=2),
            10,
        )
        self.subscription_1  # 防止被垃圾回收
        self.subscription_2  # 防止被垃圾回收

        self.get_logger().info("Camera processor node has been started.")

    def image_callback(self, msg: Image, camera_id):
        try:
            # 将 ROS 2 Image 消息转换为 OpenCV 图像
            # "bgr8" 表示输出为 BGR 格式的 8 位图像
            cv_image = self.bridge_list[camera_id - 1].imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
            self.image_counter[camera_id-1]+=1
            self.image_list[camera_id-1]=cv_image

            # --- 在这里使用 OpenCV 处理图像 ---
            # 示例：添加文本和显示
            cv2.putText(
                cv_image,
                "frame_num "+str(self.image_counter[camera_id-1]),
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            # # 显示图像 (确保有 GUI 环境，或在 Docker 中配置 X11)
            cv2.imshow("Camera View " + str(camera_id), cv_image)
            cv2.waitKey(1)  # 必须调用，否则窗口不会刷新

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"General Error in callback: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
