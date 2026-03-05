import numpy as np

# Rotation from tool frame to camera frame
# 180° rotation around Z axis (determined through validation)
R_tool_cam = np.array([
    [-1.0,  0.0, 0.0],
    [0.0,  -1.0, 0.0],
    [0.0,   0.0, 1.0],
])

# Translation from tool flange to camera lens (in meters)
# Official Kinova Gen3 specs for COLOR sensor:
#   (dx, dy, dz) = (0 mm, 56.39 mm, -3.05 mm)
# Source: Kinova Gen3 Vision Module documentation
# 
# Additional Y correction: computed Y is 9.5cm too negative
# Due to 180° Z rotation, tool X affects base Y
t_tool_cam = np.array([
    0.095,     # X: adjust to correct Base Y error (positive direction)
    0.05639,   # Y: 56.39mm forward (official spec)
    -0.00305,  # Z: 3.05mm below (official spec)
])

# ============================================================================
# ORIGINAL CALIBRATION (from tools/solve_tool_cam.py)
# Kept for reference - had ~10cm error on Y axis
# ============================================================================
# R_tool_cam_original = np.array([
#     [-1.0,  0.0, 0.0],
#     [0.0,  -1.0, 0.0],
#     [0.0,   0.0, 1.0],
# ])
# t_tool_cam_original = np.array([
#     0.0,    # X: camera centered
#     0.006,  # Y: camera 0.6cm forward of flange
#     0.0,    # Z: no vertical offset
# ])
# ============================================================================