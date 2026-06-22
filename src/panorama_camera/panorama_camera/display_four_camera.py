import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


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

    def image_callback(self, msg, index):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.images[index] = cv_img
        except Exception as e:
            self.get_logger().error(f'Failed to convert image from camera {index + 1}: {e}')

    def display_cameras(self):
        display_imgs = []
        for i in range(4):
            if self.images[i] is not None:
                img = self.images[i].copy()
                h, w = img.shape[:2]
                cv2.putText(img, f'Camera {i + 1}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                display_imgs.append(img)
            else:
                # Show placeholder while waiting for first image
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, f'Camera {i + 1} - Waiting...', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                display_imgs.append(blank)

        # Arrange 4 cameras in a 2x2 grid
        top = np.hstack((display_imgs[0], display_imgs[1]))
        bottom = np.hstack((display_imgs[2], display_imgs[3]))
        grid = np.vstack((top, bottom))

        cv2.imshow('Four Camera View', grid)
        cv2.waitKey(1)


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
