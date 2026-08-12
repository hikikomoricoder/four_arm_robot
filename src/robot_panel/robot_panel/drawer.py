"""Right-side drawer panel with expand/collapse toggle."""

import tkinter as tk

DRAWER_W = 180
HANDLE_W = 24


class Drawer(tk.Frame):
    """Collapsible drawer anchored to the right edge of the window."""

    def __init__(self, parent, window_w, window_h, bg, fg, border):
        """Build the drawer frame, toggle button, and content placeholder.

        Args:
            parent: parent tk widget.
            window_w, window_h: main window dimensions for edge placement.
            bg, fg, border: color palette.
        """
        super().__init__(parent, bg=bg, bd=0,
                         highlightbackground=border,
                         highlightcolor=border,
                         highlightthickness=1)
        self._window_w = window_w
        self._window_h = window_h
        self._bg = bg
        self._fg = fg
        self._expanded = False

        self._toggle_btn = tk.Button(
            self, text='<', relief='flat', bd=0,
            bg=bg, fg=fg,
            activebackground=bg, activeforeground=fg,
            command=self.toggle)
        self._toggle_btn.place(x=2, y=4, width=20, height=28)

        self._content = tk.Frame(self, bg=bg)
        tk.Label(self._content, bg=bg, fg=fg,
                 text='drawer\n(content varies by state)').pack(expand=True)

        self._place()

    # ------------------------------------------------------------------
    def _place(self):
        """Position the drawer according to current expand state."""
        width = DRAWER_W if self._expanded else HANDLE_W
        self.place(x=self._window_w - width, y=0,
                   width=width, height=self._window_h)
        if self._expanded:
            self._content.place(x=1, y=36,
                                width=DRAWER_W - 2,
                                height=self._window_h - 37)
        else:
            self._content.place_forget()
        self.lift()

    # ------------------------------------------------------------------
    def toggle(self):
        """Expand or collapse the drawer."""
        self._expanded = not self._expanded
        self._toggle_btn.configure(
            text='>' if self._expanded else '<')
        self._place()

    # ------------------------------------------------------------------
    def set_page(self, page):
        """Switch drawer content by situation (placeholder, TBD)."""