"""Camera image display block with ROS2 subscription.

Subscribes to a selectable /camera_N/image_raw topic (or, for the
'stereo' selection, /stereo_camera/detect_estimate), downscales the
image to 1/2 resolution, and displays it at a user-configurable
refresh rate controlled by RobotStateBlock.
"""

import threading
import tkinter as tk

import numpy as np
from PIL import Image as PILImage, ImageTk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraBlock(tk.Frame):
    """Displays a single camera feed at 1/2 resolution.

    Camera selection, visibility (Show Camera), and refresh rate
    are read from the RobotStateBlock on every display tick.  The
    'stereo' selection is gated by the stereo_estimate module switch.
    """

    def __init__(self, parent, bg, fg, border, robot_state=None):
        """Build the block frame and start the ROS2 background thread.

        Args:
            parent: parent tk widget.
            bg, fg, border: color palette.
            robot_state: RobotStateBlock instance for camera config.
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
        self._latest_msg = None
        self._sub = None
        self._current_camera = None
        self._display_timer_id = None
        self._running = False

        # Image label (hidden until first frame)
        self._image_label = tk.Label(self, bg=bg)
        self._image_label.place(relx=0.5, rely=0.5, anchor='center')

        # Fallback status label
        self._status_label = tk.Label(
            self, text='camera', bg=bg, fg=fg)
        self._status_label.place(relx=0.5, rely=0.5, anchor='center')

        # ROS2 subscriber node (lazy: subscription created on first start)
        self._ros_node = None
        self._ros_ready = threading.Event()
        self._ros_thread = threading.Thread(
            target=self._ros_spin, daemon=True)
        self._ros_thread.start()

    # ------------------------------------------------------------------
    # ROS2 subscriber (background thread)
    # ------------------------------------------------------------------
    def _ros_spin(self):
        """Create a ROS2 node and spin."""
        if not rclpy.ok():
            rclpy.init()
        self._ros_node = Node('camera_display')
        self._ros_ready.set()
        # Dedicated executor: rclpy.spin() would share the process-global
        # SingleThreadedExecutor with the panorama thread, and concurrent
        # spin_once() calls on it raise "generator already executing".
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._ros_node)
        self._executor.spin()

    def _ensure_subscription(self, camera_name):
        """Create or switch the ROS2 subscription for the given camera.

        Args:
            camera_name: e.g. 'camera1' → subscribes to /camera_1/image_raw;
                         'stereo' → subscribes to /stereo_camera/detect_estimate.
        """
        if self._sub is not None and self._current_camera == camera_name:
            return  # already subscribed — nothing to do

        # Destroy previous subscription
        if self._sub is not None:
            self._ros_node.destroy_subscription(self._sub)
            self._sub = None
            self._latest_msg = None

        # Map camera1 → /camera_1/image_raw; 'stereo' is the annotated
        # detection + ranging view published by stereo_camera_processor.
        if camera_name == 'stereo':
            topic = '/stereo_camera/detect_estimate'
        else:
            idx = camera_name.replace('camera', '')
            topic = f'/camera_{idx}/image_raw'
        self._ros_node.get_logger().info(f'CameraBlock subscribing to {topic}')
        self._sub = self._ros_node.create_subscription(
            Image, topic, self._on_camera_msg, 10)
        self._current_camera = camera_name

    def _on_camera_msg(self, msg):
        """ROS callback (background thread): store the latest frame."""
        self._latest_msg = msg

    # ------------------------------------------------------------------
    # Public API – called by RobotStateBlock
    # ------------------------------------------------------------------
    def start_display(self):
        """Start the periodic display loop. Safe to call repeatedly."""
        if self._running:
            return
        if not self._ros_ready.is_set():
            # ROS node not yet ready — retry shortly
            self.after(100, self.start_display)
            return
        self._running = True
        self._schedule_next_frame()

    def stop_display(self):
        """Stop the display loop and clear the image."""
        self._running = False
        if self._display_timer_id is not None:
            self.after_cancel(self._display_timer_id)
            self._display_timer_id = None
        # Destroy ROS subscription to stop network traffic
        if self._sub is not None:
            self._ros_node.destroy_subscription(self._sub)
            self._sub = None
            self._current_camera = None
        self._clear()

    # ------------------------------------------------------------------
    # Main-thread display loop
    # ------------------------------------------------------------------
    def _schedule_next_frame(self):
        """Display the latest frame and schedule the next tick."""
        if not self._running:
            return
        self._display_timer_id = None

        # Visibility re-check (may have been toggled off externally)
        if (self._robot_state is None
                or not self._robot_state._camera_var.get()):
            self._running = False
            self._clear()
            return

        # Ensure ROS subscription matches the selected camera
        selected = self._robot_state._camera_choice.get()
        if self._ros_node is not None:
            self._ensure_subscription(selected)

        # Display the latest received frame.  The stereo view only shows a
        # picture while the stereo_estimate module switch is enabled.
        stereo_enabled = self._robot_state._module_vars['stereo_estimate'].get()
        if self._latest_msg is not None and (selected != 'stereo' or stereo_enabled):
            self._handle_frame()
        elif selected == 'stereo' and not stereo_enabled:
            self._clear()

        # Parse refresh rate and schedule the next tick
        try:
            refresh_hz = float(self._robot_state._refresh_var.get())
        except ValueError:
            refresh_hz = 10.0
        period_ms = max(16, int(1000.0 / max(refresh_hz, 0.1)))
        self._display_timer_id = self.after(
            period_ms, self._schedule_next_frame)

    def _handle_frame(self):
        """Convert the latest ROS image message and display it."""
        msg = self._latest_msg
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        self._update_image(cv_image)

    def _update_image(self, cv_image):
        """Downscale to 1/2 and display as a tkinter PhotoImage."""
        h, w = cv_image.shape[:2]
        new_w, new_h = max(1, w // 2), max(1, h // 2)

        # BGR (OpenCV) → RGB (Pillow)
        rgb = cv_image[..., ::-1]
        pil_img = PILImage.fromarray(rgb)
        pil_img = pil_img.resize((new_w, new_h), PILImage.LANCZOS)

        self._photo = ImageTk.PhotoImage(pil_img)
        self._image_label.config(image=self._photo)
        self._image_label.place(x=0, y=0, width=new_w, height=new_h)
        self._status_label.place_forget()

    def _clear(self):
        """Hide the camera image and show the status label."""
        self._image_label.place_forget()
        self._status_label.place(relx=0.5, rely=0.5, anchor='center')
        self._photo = None
        self._latest_msg = None

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy(self):
        """Shut down the ROS2 node and timers before destroying."""
        self._running = False
        if self._display_timer_id is not None:
            self.after_cancel(self._display_timer_id)
            self._display_timer_id = None
        if self._sub is not None:
            self._ros_node.destroy_subscription(self._sub)
            self._sub = None
        if hasattr(self, '_ros_node') and self._ros_node is not None:
            self._ros_node.destroy_node()
        super().destroy()