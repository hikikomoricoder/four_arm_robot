"""Control commander block with basic, interact, and semantic sections."""

import tkinter as tk

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
        """Create a flat styled button (placeholder, no command)."""
        btn = tk.Button(
            parent, text=text, relief='flat', bd=0,
            bg=self._bg, fg=self._fg,
            activebackground=self._bg, activeforeground=self._fg)
        btn.place(x=x, y=y, width=w, height=BTN_H)
        return btn

    def _make_entry(self, parent, x, y, w, default=''):
        """Create a tk Entry with dark styling."""
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
        return entry

    # ------------------------------------------------------------------
    # basic_commander (top-left)
    # ------------------------------------------------------------------
    def _build_basic_commander(self, parent):
        """Build the basic_commander section with mode buttons
        grouped by veer / wheel / compound, plus speed & duration inputs.
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
            self._make_btn(parent, mode, bx, y, btn_w)
        y += BTN_H + ROW_GAP

        # -- Wheel group --
        self._group_label(parent, 'Wheel:', PAD_X, y)
        y += LBL_H + 2

        wheel_modes = ['forward', 'turn_right', 'turn_left']
        n = len(wheel_modes)
        btn_w = (usable - (n - 1) * ROW_GAP) // n
        for i, mode in enumerate(wheel_modes):
            bx = PAD_X + i * (btn_w + ROW_GAP)
            self._make_btn(parent, mode, bx, y, btn_w)
        y += BTN_H + ROW_GAP

        # -- Compound group --
        self._group_label(parent, 'Compound:', PAD_X, y)
        y += LBL_H + 2

        compound_modes = ['home', 'low', 'high', 'rhom_1', 'rhom_2']
        n = len(compound_modes)
        btn_w = (usable - (n - 1) * ROW_GAP) // n
        for i, mode in enumerate(compound_modes):
            bx = PAD_X + i * (btn_w + ROW_GAP)
            self._make_btn(parent, mode, bx, y, btn_w)
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

        self._make_btn(parent, 'panorama_info_broadcast',
                       btn_x, y, btn_w)
        y += BTN_H + 16

        self._make_btn(parent, 'stereo_distance_broadcast',
                       btn_x, y, btn_w)

    # ------------------------------------------------------------------
    # semantic_commander (bottom)
    # ------------------------------------------------------------------
    def _build_semantic_commander(self, parent):
        """Build the semantic_commander section (empty placeholder)."""
        y = PAD_Y

        title = self._section_label(parent, 'semantic_commander')
        title.place(relx=0.5, y=y, anchor='n')