"""Borderless robot status and control panel (layout skeleton only)."""

import tkinter as tk

WINDOW_W = 1070
WINDOW_H = 750
PAD = 10
GAP = 10
DRAWER_W = 180
HANDLE_W = 24

COLOR_BG = '#2b2b2b'
COLOR_FG = '#ffffff'
COLOR_BORDER = '#ffffff'

# (name, x, y, width, height)
BLOCK_LAYOUT = (
    ('robot_state', PAD, PAD, 320, 480),
    ('control_commander', PAD + 320 + GAP, PAD, 720, 480),
    ('panorama', PAD + 320 + GAP, PAD + 480 + GAP, 720, 240),
    ('camera', PAD, PAD + 480 + GAP, 320, 240),
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

        self.blocks = {}
        for name, x, y, w, h in BLOCK_LAYOUT:
            self.blocks[name] = self._make_block(name, x, y, w, h)

        self._drawer_expanded = False
        self._build_drawer()

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

    def _make_block(self, name, x, y, w, h):
        frame = tk.Frame(self, bg=COLOR_BG, bd=0,
                         highlightbackground=COLOR_BORDER,
                         highlightcolor=COLOR_BORDER,
                         highlightthickness=1)
        frame.place(x=x, y=y, width=w, height=h)
        tk.Label(frame, text=name, bg=COLOR_BG,
                 fg=COLOR_FG).place(relx=0.5, rely=0.5, anchor='center')
        return frame

    def _build_drawer(self):
        self.drawer = tk.Frame(self, bg=COLOR_BG, bd=0,
                               highlightbackground=COLOR_BORDER,
                               highlightcolor=COLOR_BORDER,
                               highlightthickness=1)
        self._toggle_btn = tk.Button(
            self.drawer, text='<', relief='flat', bd=0,
            bg=COLOR_BG, fg=COLOR_FG,
            activebackground=COLOR_BG, activeforeground=COLOR_FG,
            command=self.toggle_drawer)
        self._toggle_btn.place(x=2, y=4, width=20, height=28)
        self._drawer_content = tk.Frame(self.drawer, bg=COLOR_BG)
        tk.Label(self._drawer_content, bg=COLOR_BG, fg=COLOR_FG,
                 text='drawer\n(content varies by state)').pack(expand=True)
        self._place_drawer()

    def _place_drawer(self):
        width = DRAWER_W if self._drawer_expanded else HANDLE_W
        self.drawer.place(x=WINDOW_W - width, y=0,
                          width=width, height=WINDOW_H)
        if self._drawer_expanded:
            self._drawer_content.place(x=1, y=36,
                                       width=DRAWER_W - 2,
                                       height=WINDOW_H - 37)
        else:
            self._drawer_content.place_forget()
        self.drawer.lift()

    def toggle_drawer(self):
        """Expand or collapse the right drawer."""
        self._drawer_expanded = not self._drawer_expanded
        self._toggle_btn.configure(
            text='>' if self._drawer_expanded else '<')
        self._place_drawer()

    def set_drawer_page(self, page):
        """Switch drawer content by situation (placeholder, TBD)."""

    def close(self):
        """Close the window (Alt+F4 or WM close)."""
        self.destroy()


def main():
    """Entry point for the robot_panel console script."""
    RobotPanel().mainloop()


if __name__ == '__main__':
    main()
