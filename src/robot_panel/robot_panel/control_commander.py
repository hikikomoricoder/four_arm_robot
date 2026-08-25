"""Control commander block with basic, interact, and semantic sections."""

import ctypes
import subprocess
import threading
import tkinter as tk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Known block dimensions from panel.py BLOCK_LAYOUT
BLOCK_W = 720
BLOCK_H = 480
TOP_H = int(BLOCK_H * 0.6)   # 288
BOT_H = BLOCK_H - TOP_H       # 192
HALF_W = BLOCK_W // 2          # 360

# Layout constants
PAD_X = 8
PAD_Y = 4
BTN_H = 26
LBL_H = 18
SEC_H = 20
ROW_GAP = 6

# ----------------------------------------------------------------------
# X11 keyboard-focus helper.
# The panel window is borderless (overrideredirect); Tk's focus_force()
# only updates Tk-internal state in that setup and never sets the real X
# input focus, so typing into an Entry silently does nothing.  Calling
# XSetInputFocus through libX11 directly does work, so entries use it on
# click (verified on GNOME/mutter X11).  Falls back to focus_force() on
# non-X11 platforms.
_libx11 = None
_x11_display = None


def _x_force_focus(widget):
    """Give widget the real X11 keyboard focus (falls back to Tk)."""
    global _libx11, _x11_display
    widget.focus_force()  # keep Tk's own bookkeeping consistent
    try:
        if _libx11 is None:
            lib = ctypes.CDLL('libX11.so.6')
            lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
            lib.XOpenDisplay.restype = ctypes.c_void_p
            lib.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                                           ctypes.c_int, ctypes.c_ulong]
            lib.XSetInputFocus.restype = ctypes.c_int
            lib.XFlush.argtypes = [ctypes.c_void_p]
            lib.XFlush.restype = ctypes.c_int
            _x11_display = lib.XOpenDisplay(None)
            _libx11 = lib if _x11_display else False
        if not _libx11:
            return
        _libx11.XSetInputFocus(_x11_display, widget.winfo_id(),
                               2, 0)  # RevertToParent, CurrentTime
        _libx11.XFlush(_x11_display)
    except Exception:
        pass


class ControlCommanderBlock(tk.Frame):
    """Three-section frame: basic_commander (top-left),
    robot_interact (top-right), semantic_commander (bottom).
    """

    def __init__(self, parent, bg, fg, border):
        """Build the block frame with three sub-sections.

        Args:
            parent: parent tk widget.
            bg, fg, border: color palette.
        """
        super().__init__(parent, bg=bg, bd=0,
                         highlightbackground=border,
                         highlightcolor=border,
                         highlightthickness=1)
        self._bg = bg
        self._fg = fg
        self._border = border

        # --- Top-left: basic_commander ---
        self._basic_frame = tk.Frame(
            self, bg=bg, bd=0,
            highlightbackground=border,
            highlightcolor=border,
            highlightthickness=1)
        self._basic_frame.place(x=0, y=0, width=HALF_W, height=TOP_H)
        self._build_basic_commander(self._basic_frame)

        # --- Top-right: robot_interact ---
        self._interact_frame = tk.Frame(
            self, bg=bg, bd=0,
            highlightbackground=border,
            highlightcolor=border,
            highlightthickness=1)
        self._interact_frame.place(x=HALF_W, y=0, width=HALF_W, height=TOP_H)
        self._build_robot_interact(self._interact_frame)

        # --- Bottom: semantic_commander ---
        self._semantic_frame = tk.Frame(
            self, bg=bg, bd=0,
            highlightbackground=border,
            highlightcolor=border,
            highlightthickness=1)
        self._semantic_frame.place(x=0, y=TOP_H, width=BLOCK_W, height=BOT_H)
        self._build_semantic_commander(self._semantic_frame)

        # ROS2 interface for the robot_interact broadcast buttons
        self._ros_node = None
        self._ros_ready = threading.Event()
        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _section_label(self, parent, text):
        """Create a bold section title label (caller must place)."""
        return tk.Label(
            parent, text=text, bg=self._bg, fg=self._fg,
            font=('TkDefaultFont', 10, 'bold'), anchor='w')

    def _group_label(self, parent, text, x, y):
        """Create a group sub-heading label."""
        lbl = tk.Label(
            parent, text=text, bg=self._bg, fg=self._fg,
            font=('TkDefaultFont', 9), anchor='w')
        lbl.place(x=x, y=y)
        return lbl

    def _make_btn(self, parent, text, x, y, w):
        """Create a flat styled button (command attached by caller)."""
        btn = tk.Button(
            parent, text=text, relief='flat', bd=0,
            bg=self._bg, fg=self._fg,
            activebackground=self._bg, activeforeground=self._fg)
        btn.place(x=x, y=y, width=w, height=BTN_H)
        return btn

    def _make_entry(self, parent, x, y, w, default=''):
        """Create a tk Entry with dark styling.

        The panel window is borderless (overrideredirect), where Tk's
        focus_force() never sets the real X input focus; the click
        binding sets it directly via libX11 (see _x_force_focus) so
        typing into the entry works.
        """
        entry = tk.Entry(
            parent, bg='#1e1e1e', fg=self._fg,
            insertbackground=self._fg,
            relief='flat', bd=1,
            highlightbackground=self._border,
            highlightcolor=self._border,
            highlightthickness=1)
        entry.place(x=x, y=y, width=w, height=BTN_H)
        if default:
            entry.insert(0, default)
        entry.bind('<Button-1>', lambda e: _x_force_focus(e.widget),
                   add='+')
        return entry

    # ------------------------------------------------------------------
    # basic_commander (top-left)
    # ------------------------------------------------------------------
    def _build_basic_commander(self, parent):
        """Build the basic_commander section with mode buttons
        grouped by veer / wheel / compound, plus speed & duration inputs.
        The speed entry doubles as the turn-angle (deg) input for the
        wheel 'turn_angle' mode.
        """
        bw = HALF_W  # 360
        y = PAD_Y

        # Section title
        title = self._section_label(parent, 'basic_commander')
        title.place(relx=0.5, y=y, anchor='n')
        y += SEC_H + ROW_GAP

        # -- Veer group --
        self._group_label(parent, 'Veer:', PAD_X, y)
        y += LBL_H + 2

        veer_modes = ['home', 'forward', 'turn', 'lift']
        n = len(veer_modes)
        usable = bw - PAD_X * 2
        btn_w = (usable - (n - 1) * ROW_GAP) // n
        for i, mode in enumerate(veer_modes):
            bx = PAD_X + i * (btn_w + ROW_GAP)
            btn = self._make_btn(parent, mode, bx, y, btn_w)
            btn.config(command=lambda m=mode: self._on_veer(m))
        y += BTN_H + ROW_GAP

        # -- Wheel group --
        self._group_label(parent, 'Wheel:', PAD_X, y)
        y += LBL_H + 2

        # (button label, wheel_commander_test mode)
        wheel_modes = [('forward', 'forward'),
                       ('turn_right', 'turn_right_speed'),
                       ('turn_left', 'turn_left_speed'),
                       ('turn_angle', 'turn_angle')]
        n = len(wheel_modes)
        btn_w = (usable - (n - 1) * ROW_GAP) // n
        for i, (label, mode) in enumerate(wheel_modes):
            bx = PAD_X + i * (btn_w + ROW_GAP)
            btn = self._make_btn(parent, label, bx, y, btn_w)
            if mode == 'turn_angle':
                btn.config(command=self._on_wheel_turn_angle)
            else:
                btn.config(command=lambda m=mode: self._on_wheel(m))
        y += BTN_H + ROW_GAP

        # -- Compound group --
        self._group_label(parent, 'Compound:', PAD_X, y)
        y += LBL_H + 2

        # (button label, compound_commander_test mode)
        compound_modes = [('home', 'home'),
                          ('low', 'low'),
                          ('high', 'high'),
                          ('rhom_1', 'rhombus_1'),
                          ('rhom_2', 'rhombus_2')]
        n = len(compound_modes)
        btn_w = (usable - (n - 1) * ROW_GAP) // n
        for i, (label, mode) in enumerate(compound_modes):
            bx = PAD_X + i * (btn_w + ROW_GAP)
            btn = self._make_btn(parent, label, bx, y, btn_w)
            btn.config(command=lambda m=mode: self._on_compound(m))
        y += BTN_H + ROW_GAP + 4

        # -- Speed & Duration inputs --
        field_y = y
        tk.Label(parent, text='speed:', bg=self._bg, fg=self._fg,
                 font=('TkDefaultFont', 9), anchor='e').place(
            x=PAD_X, y=field_y, width=48, height=BTN_H)
        self._speed_entry = self._make_entry(
            parent, PAD_X + 52, field_y, 72, default='0.1')

        tk.Label(parent, text='m/s', bg=self._bg, fg=self._fg,
                 font=('TkDefaultFont', 8), anchor='w').place(
            x=PAD_X + 52 + 74, y=field_y, width=24, height=BTN_H)

        tk.Label(parent, text='duration:', bg=self._bg, fg=self._fg,
                 font=('TkDefaultFont', 9), anchor='e').place(
            x=PAD_X + 160, y=field_y, width=56, height=BTN_H)
        self._duration_entry = self._make_entry(
            parent, PAD_X + 220, field_y, 72, default='3.0')

        tk.Label(parent, text='s', bg=self._bg, fg=self._fg,
                 font=('TkDefaultFont', 8), anchor='w').place(
            x=PAD_X + 220 + 74, y=field_y, width=16, height=BTN_H)

    # ------------------------------------------------------------------
    # robot_interact (top-right)
    # ------------------------------------------------------------------
    def _build_robot_interact(self, parent):
        """Build the robot_interact section with two broadcast buttons."""
        iw = HALF_W  # 360
        y = PAD_Y

        title = self._section_label(parent, 'robot_interact')
        title.place(relx=0.5, y=y, anchor='n')
        y += SEC_H + 20

        # Buttons centered, wider
        btn_w = iw - PAD_X * 6
        btn_x = PAD_X * 3

        pano_btn = self._make_btn(parent, 'panorama_info_broadcast',
                                  btn_x, y, btn_w)
        pano_btn.config(command=self._on_panorama_broadcast)
        y += BTN_H + 16

        self._make_btn(parent, 'stereo_distance_broadcast',
                       btn_x, y, btn_w)

    # ------------------------------------------------------------------
    # ROS2 interface (background thread)
    # ------------------------------------------------------------------
    def _ros_spin(self):
        """Create the ROS2 node: TTS publisher + azimuth description client."""
        if not rclpy.ok():
            rclpy.init()
        self._ros_node = Node('control_commander')
        self._tts_pub = self._ros_node.create_publisher(String, '/tts/say', 10)
        self._desc_client = self._ros_node.create_client(
            Trigger, 'get_azimuth_description')
        self._ros_ready.set()
        # Dedicated executor per spinning thread (see PanoramaBlock).
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._ros_node)
        self._executor.spin()

    # ------------------------------------------------------------------
    # Button handlers (main thread)
    # ------------------------------------------------------------------
    def _on_wheel_turn_angle(self):
        """Run wheel_commander_test turn_angle; the angle (deg) is read
        from the speed entry, which is reused as the turn-angle input.
        Runs in a background thread so the UI stays responsive."""
        raw = self._speed_entry.get().strip()
        try:
            angle = float(raw)
        except ValueError:
            if self._ros_ready.is_set():
                self._ros_node.get_logger().warn(
                    f'turn_angle: invalid angle {raw!r} in speed entry')
            return

        def run():
            try:
                proc = subprocess.run(
                    ['ros2', 'run', 'robot_commander',
                     'wheel_commander_test', 'turn_angle', str(angle)],
                    capture_output=True, text=True)
                if self._ros_ready.is_set() and proc.returncode != 0:
                    self._ros_node.get_logger().warn(
                        f'turn_angle {angle} deg failed '
                        f'(exit {proc.returncode}): {proc.stderr.strip()[:200]}')
            except Exception as e:
                if self._ros_ready.is_set():
                    self._ros_node.get_logger().warn(
                        f'turn_angle command failed: {e}')

        threading.Thread(target=run, daemon=True).start()

    def _read_float(self, entry, field):
        """Parse a float from an entry; warn and return None if invalid."""
        raw = entry.get().strip()
        try:
            return float(raw)
        except ValueError:
            if self._ros_ready.is_set():
                self._ros_node.get_logger().warn(
                    f'{field}: invalid value {raw!r}')
            return None

    def _run_commander(self, cmd, tag):
        """Run a ros2 commander executable in a background thread so the
        UI stays responsive; log non-zero exits via the ROS logger."""
        def run():
            try:
                proc = subprocess.run(cmd, capture_output=True, text=True)
                if self._ros_ready.is_set() and proc.returncode != 0:
                    self._ros_node.get_logger().warn(
                        f'{tag} failed (exit {proc.returncode}): '
                        f'{proc.stderr.strip()[:200]}')
            except Exception as e:
                if self._ros_ready.is_set():
                    self._ros_node.get_logger().warn(
                        f'{tag} command failed: {e}')
        threading.Thread(target=run, daemon=True).start()

    def _on_veer(self, mode):
        """Run veer_commander_test <mode> <duration>."""
        duration = self._read_float(
            self._duration_entry, f'veer {mode} duration')
        if duration is None:
            return
        self._run_commander(
            ['ros2', 'run', 'robot_commander', 'veer_commander_test',
             mode, str(duration)],
            f'veer {mode}')

    def _on_wheel(self, mode):
        """Run wheel_commander_test <mode> <speed> <duration>."""
        speed = self._read_float(self._speed_entry, f'wheel {mode} speed')
        duration = self._read_float(
            self._duration_entry, f'wheel {mode} duration')
        if speed is None or duration is None:
            return
        self._run_commander(
            ['ros2', 'run', 'robot_commander', 'wheel_commander_test',
             mode, str(speed), str(duration)],
            f'wheel {mode}')

    def _on_compound(self, mode):
        """Run compound_commander_test <mode> <duration>."""
        duration = self._read_float(
            self._duration_entry, f'compound {mode} duration')
        if duration is None:
            return
        self._run_commander(
            ['ros2', 'run', 'robot_commander', 'compound_commander_test',
             mode, str(duration)],
            f'compound {mode}')

    def _on_panorama_broadcast(self):
        """Speak the latest panorama azimuth description once via TTS."""
        if not self._ros_ready.is_set():
            return
        if not self._desc_client.service_is_ready():
            self._ros_node.get_logger().warn(
                'get_azimuth_description unavailable — '
                'is display_four_camera running?')
            return
        future = self._desc_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_desc_response)

    def _on_desc_response(self, future):
        """Service done callback (executor thread): forward text to TTS."""
        try:
            resp = future.result()
        except Exception as e:
            self._ros_node.get_logger().warn(
                f'Azimuth description request failed: {e}')
            return
        if not resp.success or not resp.message:
            self._ros_node.get_logger().info(
                f'Nothing to broadcast ({resp.message})')
            return
        self._tts_pub.publish(String(data=resp.message))
        self._ros_node.get_logger().info(f'Broadcasting: {resp.message}')

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy(self):
        """Shut down the ROS2 node before destroying the widget."""
        if self._ros_node is not None:
            self._ros_node.destroy_node()
        super().destroy()

    # ------------------------------------------------------------------
    # semantic_commander (bottom)
    # ------------------------------------------------------------------
    def _build_semantic_commander(self, parent):
        """Build the semantic_commander section (empty placeholder)."""
        y = PAD_Y

        title = self._section_label(parent, 'semantic_commander')
        title.place(relx=0.5, y=y, anchor='n')