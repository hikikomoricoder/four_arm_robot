"""Robot state display block with module control, drawer content, and display settings."""

import tkinter as tk
from tkinter import ttk


class RobotStateBlock(tk.Frame):
    """Three-section control panel for module toggles, drawer content
    selection, and display settings.
    """

    def __init__(self, parent, bg, fg, border):
        """Build the three-section control block.

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

        # ------------------------------------------------------------------
        # Section 1: Module Control
        # ------------------------------------------------------------------
        self._sec1_label = tk.Label(
            self, text='Module Control', bg=bg, fg=fg,
            font=('TkDefaultFont', 10, 'bold'))
        self._sec1_label.place(x=8, y=8)

        self._module_vars = {}
        self._module_switches = {}
        modules = ['panorama_concat', 'panorama_detect', 'stereo_estimate']
        for i, name in enumerate(modules):
            var = tk.BooleanVar(value=False)
            self._module_vars[name] = var
            cb = tk.Checkbutton(
                self, text=name, variable=var,
                bg=bg, fg=fg, selectcolor=bg,
                activebackground=bg, activeforeground=fg,
                anchor='w')
            cb.place(x=16, y=36 + i * 28, width=200, height=22)
            self._module_switches[name] = cb

        # ------------------------------------------------------------------
        # Section 2: Drawer Content
        # ------------------------------------------------------------------
        sec2_y = 36 + 3 * 28 + 16
        self._sec2_label = tk.Label(
            self, text='Drawer Content', bg=bg, fg=fg,
            font=('TkDefaultFont', 10, 'bold'))
        self._sec2_label.place(x=8, y=sec2_y)

        self._drawer_var = tk.StringVar(value='')
        drawer_items = ['joint_state', 'robot_state']
        self._drawer_radios = {}
        for i, name in enumerate(drawer_items):
            rb = tk.Radiobutton(
                self, text=name, variable=self._drawer_var,
                value=name,
                bg=bg, fg=fg, selectcolor=bg,
                activebackground=bg, activeforeground=fg,
                anchor='w')
            rb.place(x=16, y=sec2_y + 28 + i * 28, width=200, height=22)
            self._drawer_radios[name] = rb

        # None / clear selection button
        self._none_btn = tk.Button(
            self, text='Show None', relief='flat', bd=0,
            bg=bg, fg=fg,
            activebackground=bg, activeforeground=fg,
            command=self._clear_drawer)
        self._none_btn.place(x=16, y=sec2_y + 28 + 2 * 28, width=100, height=22)

        # ------------------------------------------------------------------
        # Section 3: Display Settings
        # ------------------------------------------------------------------
        sec3_y = sec2_y + 28 + 3 * 28 + 16
        self._sec3_label = tk.Label(
            self, text='Display Settings', bg=bg, fg=fg,
            font=('TkDefaultFont', 10, 'bold'))
        self._sec3_label.place(x=8, y=sec3_y)

        # Panorama display toggle
        self._panorama_var = tk.BooleanVar(value=False)
        self._panorama_cb = tk.Checkbutton(
            self, text='Show Panorama', variable=self._panorama_var,
            bg=bg, fg=fg, selectcolor=bg,
            activebackground=bg, activeforeground=fg,
            anchor='w',
            command=self._on_panorama_toggle)
        self._panorama_cb.place(x=16, y=sec3_y + 28, width=160, height=22)

        # Camera display toggle
        self._camera_var = tk.BooleanVar(value=False)
        self._camera_cb = tk.Checkbutton(
            self, text='Show Camera', variable=self._camera_var,
            bg=bg, fg=fg, selectcolor=bg,
            activebackground=bg, activeforeground=fg,
            anchor='w',
            command=self._on_camera_toggle)
        self._camera_cb.place(x=16, y=sec3_y + 28 + 28, width=160, height=22)

        # Camera selector
        camera_label = tk.Label(
            self, text='Camera:', bg=bg, fg=fg, anchor='w')
        camera_label.place(x=32, y=sec3_y + 28 + 28 + 28, width=60, height=22)

        self._camera_choice = tk.StringVar(value='camera1')
        camera_options = [f'camera{i}' for i in range(1, 7)]
        self._camera_combo = ttk.Combobox(
            self, textvariable=self._camera_choice,
            values=camera_options, state='readonly', width=10)
        self._camera_combo.place(x=96, y=sec3_y + 28 + 28 + 28, width=100, height=22)

        # Refresh rate
        refresh_label = tk.Label(
            self, text='Refresh (Hz):', bg=bg, fg=fg, anchor='w')
        refresh_label.place(x=16, y=sec3_y + 28 + 28 + 28 + 36, width=100, height=22)

        self._refresh_var = tk.StringVar(value='30')
        refresh_options = ['10', '15', '20', '30', '60']
        self._refresh_combo = ttk.Combobox(
            self, textvariable=self._refresh_var,
            values=refresh_options, state='readonly', width=6)
        self._refresh_combo.place(x=120, y=sec3_y + 28 + 28 + 28 + 36, width=64, height=22)

        # Status hint
        self._hint_label = tk.Label(
            self, text='', bg=bg, fg='#888888', anchor='w',
            font=('TkDefaultFont', 8))
        self._hint_label.place(x=8, y=464, width=304, height=14)

        # Apply initial state
        self._on_panorama_toggle()
        self._on_camera_toggle()

    # ------------------------------------------------------------------
    # Callbacks (placeholder – logic will be wired later)
    # ------------------------------------------------------------------
    def _clear_drawer(self):
        """Clear drawer content selection (show nothing)."""
        self._drawer_var.set('')

    def _on_panorama_toggle(self):
        """Handle panorama display toggle; block if panorama_concat is off."""
        if self._panorama_var.get() and not self._module_vars['panorama_concat'].get():
            self._panorama_var.set(False)
            self._hint_label.config(text='Panorama requires panorama_concat enabled')
        else:
            self._hint_label.config(text='')

    def _on_camera_toggle(self):
        """Handle camera display toggle."""
        # placeholder for future logic
        pass