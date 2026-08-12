"""Panorama image display block (placeholder)."""

import tkinter as tk


class PanoramaBlock(tk.Frame):
    """Placeholder frame for stitched panorama image."""

    def __init__(self, parent, bg, fg, border):
        """Build the block frame with a centered label.

        Args:
            parent: parent tk widget.
            bg, fg, border: color palette.
        """
        super().__init__(parent, bg=bg, bd=0,
                         highlightbackground=border,
                         highlightcolor=border,
                         highlightthickness=1)
        tk.Label(self, text='panorama', bg=bg, fg=fg).place(
            relx=0.5, rely=0.5, anchor='center')