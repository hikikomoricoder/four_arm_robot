"""Panorama image display block with ROS2 subscription."""

import threading
import tkinter as tk

import numpy as np
from PIL import Image as PILImage, ImageTk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PanoramaBlock(tk.Frame):
    """Displays the stitched panorama from /panorama/annotated at 1/2 size.

    Visibility is gated by RobotStateBlock.is_panorama_visible() —
    only shown when both *Show Panorama* is checked and
    *panorama_concat* is enabled.
    """

    def __init__(self, parent, bg, fg, border, robot_state=None):
        """Build the block frame with a ROS2 image subscriber.

        Args:
            parent: parent tk widget.
            bg, fg, border: color palette.
            robot_state: RobotStateBlock reference for visibility gating.
        """
        super().__init__(parent, bg=bg, bd=0,
                         highlightbackground=border,
                         highlightcolor=border,
                         highlightthickness=1)
        self._bg = bg
        self._fg = fg
        self._robot_state = robot_state
        self._bridge = CvBridge()
        self._photo = None

        # Image label (hidden until first frame)
        self._image_label = tk.Label(self, bg=bg)
        self._image_label.place(relx=0.5, rely=0.5, anchor='center')

        # Fallback status label
        self._status_label = tk.Label(
            self, text='panorama', bg=bg, fg=fg)
        self._status_label.place(relx=0.5, rely=0.5, anchor='center')

        # Start ROS2 subscriber in a background daemon thread
        self._ros_ready = threading.Event()
        self._ros_thread = threading.Thread(
            target=self._ros_spin, daemon=True)
        self._ros_thread.start()

    # ------------------------------------------------------------------
    # ROS2 subscriber (background thread)
    # ------------------------------------------------------------------
    def _ros_spin(self):
        """Create a ROS2 node and subscribe to /panorama/annotated."""
        if not rclpy.ok():
            rclpy.init()
        self._ros_node = Node('panorama_display')
        self._ros_node.create_subscription(
            Image, '/panorama/annotated', self._on_panorama_msg, 10)
        self._ros_ready.set()
        rclpy.spin(self._ros_node)

    def _on_panorama_msg(self, msg):
        """ROS callback (background thread): schedule UI update on main."""
        # Check visibility conditions (must happen on main thread via after)
        self.after(0, self._handle_frame, msg)

    # ------------------------------------------------------------------
    # Main-thread handlers
    # ------------------------------------------------------------------
    def _handle_frame(self, msg):
        """Process an incoming frame on the main thread."""
        if self._robot_state is not None:
            if not self._robot_state.is_panorama_visible():
                self._clear()
                return

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        self._update_image(cv_image)

    def _update_image(self, cv_image):
        """Resize to 1/2 and display as a tkinter PhotoImage."""
        h, w = cv_image.shape[:2]
        new_w, new_h = w // 2, h // 2

        # BGR → RGB via numpy slice (avoids extra cv2 dependency in UI)
        rgb = cv_image[..., ::-1]
        pil_img = PILImage.fromarray(rgb)
        pil_img = pil_img.resize((new_w, new_h), PILImage.LANCZOS)

        self._photo = ImageTk.PhotoImage(pil_img)
        self._image_label.config(image=self._photo)
        self._image_label.place(x=0, y=0, width=new_w, height=new_h)
        self._status_label.place_forget()

    def _clear(self):
        """Hide the panorama image and show the status label."""
        self._image_label.place_forget()
        self._status_label.place(relx=0.5, rely=0.5, anchor='center')
        self._photo = None

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy(self):
        """Shut down the ROS2 node before destroying the widget."""
        if hasattr(self, '_ros_node') and self._ros_node is not None:
            self._ros_node.destroy_node()
        super().destroy()
