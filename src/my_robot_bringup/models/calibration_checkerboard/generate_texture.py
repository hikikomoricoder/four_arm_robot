import cv2
import numpy as np
import os

# Standard calibration checkerboard: 9x6 inner corners -> 10x7 squares
squares_x = 10
squares_y = 7
pixels_per_square = 64

img_w = squares_x * pixels_per_square
img_h = squares_y * pixels_per_square

# Create white image
img = np.full((img_h, img_w, 3), 255, dtype=np.uint8)

# Draw black squares in checkerboard pattern
for row in range(squares_y):
    for col in range(squares_x):
        if (row + col) % 2 == 1:
            y_start = row * pixels_per_square
            y_end = (row + 1) * pixels_per_square
            x_start = col * pixels_per_square
            x_end = (col + 1) * pixels_per_square
            img[y_start:y_end, x_start:x_end] = (0, 0, 0)

# Add thin white border around the board
border_px = 4
img = cv2.copyMakeBorder(img, border_px, border_px, border_px, border_px, cv2.BORDER_CONSTANT, value=(255, 255, 255))

# Relative to this script location
script_dir = os.path.dirname(os.path.abspath(__file__))
output_path = os.path.join(script_dir, 'materials', 'textures', 'checkerboard.png')
cv2.imwrite(output_path, img)
print(f'Checkerboard texture saved to: {output_path}')
print(f'Image size: {img.shape[1]} x {img.shape[0]}')
