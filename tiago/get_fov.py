import numpy as np

# Prepare
w, h = 1920, 1200
fx, fy = 1744.9, 1745.47

# Go
fov_x = np.rad2deg(2 * np.arctan2(w, 2 * fx))
fov_y = np.rad2deg(2 * np.arctan2(h, 2 * fy))

print("Field of View (degrees):")
print(f"  {fov_x = :.1f}\N{DEGREE SIGN}")
print(f"  {fov_y = :.1f}\N{DEGREE SIGN}")
