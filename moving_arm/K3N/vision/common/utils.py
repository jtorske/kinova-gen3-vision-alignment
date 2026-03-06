import cv2
import numpy as np
from pupil_apriltags import Detector

from .webcam_config import camera_params, tag_size, tag_family

# =====================
# AprilTag Utilities
# =====================
def create_detector():
    return Detector(families=tag_family)

def detect_tags(detector, gray_image):
    """
    Runs AprilTag detection with pose estimation on a grayscale image.
    Returns the raw 'results' from pupil_apriltags.
    """
    return detector.detect(
        gray_image,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=tag_size,
    )

# =====================
# Transformation Utilities
# =====================
def euler_xyz_to_R(rx, ry, rz):
    cx, cy, cz = np.cos([rx, ry, rz])
    sx, sy, sz = np.sin([rx, ry, rz])

    Rx = np.array([[1, 0, 0],
                   [0, cx, -sx],
                   [0, sx,  cx]])

    Ry = np.array([[ cy, 0, sy],
                   [  0, 1, 0],
                   [-sy, 0, cy]])

    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [ 0,  0, 1]])

    return Rz @ Ry @ Rx

def rotation_to_euler_xyz(R):
    """
    Convert a 3x3 rotation matrix to XYZ Euler angles (radians).
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        yaw = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        roll = np.arctan2(R[1, 0], R[0, 0])
    else:
        yaw = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        roll = 0.0
    return yaw, pitch, roll

def compose_base_tag(R_base_tool, t_base_tool, R_tool_cam, t_tool_cam, R_cam_tag, t_cam_tag):
    """
    Compose Base→Tag from Base→Tool, Tool→Cam, Cam→Tag
    """
    R_base_cam = R_base_tool @ R_tool_cam
    t_base_cam = R_base_tool @ t_tool_cam + t_base_tool
    R_base_tag = R_base_cam @ R_cam_tag
    t_base_tag = R_base_cam @ t_cam_tag + t_base_cam
    return R_base_tag, t_base_tag

def rotation_error_deg(R_ref, R_live):
    R_err = R_ref.T @ R_live
    tr = np.trace(R_err)
    cos_theta = max(min((tr - 1.0) / 2.0, 1.0), -1.0)
    return float(np.degrees(np.arccos(cos_theta)))
