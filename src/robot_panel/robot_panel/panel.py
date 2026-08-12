"""Borderless robot status and control panel (layout skeleton only)."""

import tkinter as tk

import rclpy

from .camera import CameraBlock
from .control_commander import ControlCommanderBlock
from .drawer import Drawer
from .panorama import PanoramaBlock
from .robot_state import RobotStateBlock

WINDOW_W = 1070
WINDOW_H = 750
PAD = 10
GAP = 10

COLOR_BG = '#2b2b2b'
COLOR_FG = '#ffffff'
COLOR_BORDER = '#ffffff'

# (name, x, y, width, height, block_class)
BLOCK_LAYOUT = (
    ('robot_state',       PAD,           PAD,                 320, 480, RobotStateBlock),
    ('control_commander', PAD + 320 + GAP, PAD,               720, 480, ControlCommanderBlock),
    ('panorama',          PAD + 320 + GAP, PAD + 480 + GAP,   720, 240, PanoramaBlock),
    ('camera',            PAD,           PAD + 480 + GAP,     320, 240, CameraBlock),
)


class RobotPanel(tk.Tk):
    """Main window with four content blocks and a right drawer."""

    def __init__(self):
        """Build the borderless window and place all layout blocks."""
        super().__init__()
        self.title('robot_panel')
        self.overrideredirect(True)
        self.configure(bg=COLOR_BG)
        self.geometry(f'{WINDOW_W}x{WINDOW_H}')
        self._center_on_screen()

        # Init ROS2 before creating subscriber-bearing widgets
        if not rclpy.ok():
            rclpy.init()

        self.blocks = {}

        # Create robot_state first so panorama can reference it
        robot_state = RobotStateBlock(self, COLOR_BG, COLOR_FG, COLOR_BORDER)
        robot_state.place(x=PAD, y=PAD, width=320, height=480)
        self.blocks['robot_state'] = robot_state

        # Remaining blocks
        control = ControlCommanderBlock(self, COLOR_BG, COLOR_FG, COLOR_BORDER)
        control.place(x=PAD + 320 + GAP, y=PAD, width=720, height=480)
        self.blocks['control_commander'] = control

        panorama = PanoramaBlock(self, COLOR_BG, COLOR_FG, COLOR_BORDER,
                                 robot_state=robot_state)
        panorama.place(x=PAD + 320 + GAP, y=PAD + 480 + GAP,
                       width=720, height=240)
        self.blocks['panorama'] = panorama

        camera = CameraBlock(self, COLOR_BG, COLOR_FG, COLOR_BORDER)
        camera.place(x=PAD, y=PAD + 480 + GAP, width=320, height=240)
        self.blocks['camera'] = camera

        self.drawer = Drawer(self, WINDOW_W, WINDOW_H,
                             COLOR_BG, COLOR_FG, COLOR_BORDER)

        self.bind_all('<Alt-F4>', lambda _event: self.close())
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.after(100, self.focus_force)

        self._move_offset = None
        self.bind('<Control-ButtonPress-1>', self._start_move)
        self.bind('<Control-B1-Motion>', self._on_move)
        self.bind('<Control-ButtonRelease-1>', self._end_move)

    def _center_on_screen(self):
        self.update_idletasks()
        x = (self.winfo_screenwidth() - WINDOW_W) // 2
        y = (self.winfo_screenheight() - WINDOW_H) // 2
        self.geometry(f'+{x}+{y}')

    def _start_move(self, event):
        self._move_offset = (event.x_root - self.winfo_x(),
                             event.y_root - self.winfo_y())

    def _on_move(self, event):
        if self._move_offset is None:
            return
        ox, oy = self._move_offset
        self.geometry(f'+{event.x_root - ox}+{event.y_root - oy}')

    def _end_move(self, _event):
        self._move_offset = None

    def toggle_drawer(self):
        """Expand or collapse the right drawer."""
        self.drawer.toggle()

    def set_drawer_page(self, page):
        """Switch drawer content by situation (placeholder, TBD)."""
        self.drawer.set_page(page)

    def close(self):
        """Close the window (Alt+F4 or WM close)."""
        self.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    """Entry point for the robot_panel console script."""
    RobotPanel().mainloop()


if __name__ == '__main__':
    main()