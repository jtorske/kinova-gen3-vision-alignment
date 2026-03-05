# robot/apriltag_viewer.py
#
# Live AprilTag viewer for the Kinova Gen3 robotic arm.
#
# This module provides real-time AprilTag detection and pose estimation:
#   - Reads the Kinova Gen3 RTSP camera stream
#   - Detects AprilTags using calibrated camera intrinsics
#   - Computes Cam→Tag pose (tag position relative to camera)
#   - Computes Base→Tag pose (tag position relative to robot base)
#   - Outputs coordinates suitable for arm motion planning
#
# Coordinate Frames:
#   - Camera Frame: Origin at camera lens, Z forward, X right, Y down
#   - Base Frame: Origin at robot base, follows Kinova convention
#   - Tag Frame: Origin at tag center
#
# Output for Motion Planning Team:
#   - Base→Tag (x, y, z): Position of tag in robot base frame (meters)
#   - This is the target position for end-effector motion planning

import cv2
import numpy as np
import argparse
from collections import deque
from datetime import datetime

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from robot.device_connection import DeviceConnection

from common.utils import create_detector, rotation_to_euler_xyz, euler_xyz_to_R, compose_base_tag, rotation_error_deg
from common.webcam_config import (
    rtsp_url,
    tag_size,
    camera_params,
)
from common.tool_cam_config import R_tool_cam, t_tool_cam


# ================================================================
# CALIBRATION PARAMETERS
# ================================================================

# Camera intrinsics from calibration
fx_calib, fy_calib, cx_calib, cy_calib = camera_params

camera_params_tag = (fx_calib, fy_calib, cx_calib, cy_calib)

def print_config():
    """Print configuration for debugging."""
    print("=" * 60)
    print("KINOVA APRILTAG VIEWER - CONFIGURATION")
    print("=" * 60)
    print(f"[Camera Intrinsics (calibrated)]")
    print(f"  fx={fx_calib:.2f}, fy={fy_calib:.2f}, cx={cx_calib:.2f}, cy={cy_calib:.2f}")
    print(f"[Camera Intrinsics (for AprilTag)]")
    print(f"  fx={fx_calib:.2f}, fy={fy_calib:.2f}, cx={cx_calib:.2f}, cy={cy_calib:.2f}")
    print(f"[Tag Size]: {tag_size:.4f} m ({tag_size*100:.1f} cm)")
    print(f"[Tool→Cam Translation (m)]: {t_tool_cam}")
    print(f"[Tool→Cam Rotation]:\n{R_tool_cam}")
    print("=" * 60)



# ================================================================
# GET CURRENT BASE→TOOL POSE FROM ROBOT
# ================================================================
def get_robot_pose(base_client: BaseClient):
    """
    Query the robot for current end-effector (tool) pose in base frame.
    
    Returns:
        R_base_tool: 3x3 rotation matrix (Base→Tool)
        t_base_tool: 3-element translation vector in meters (Base→Tool)
    """
    pose = base_client.GetMeasuredCartesianPose()

    # Position in meters
    x = float(pose.x)
    y = float(pose.y)
    z = float(pose.z)

    # Orientation as Euler angles (Kinova uses degrees)
    rx = np.deg2rad(pose.theta_x)
    ry = np.deg2rad(pose.theta_y)
    rz = np.deg2rad(pose.theta_z)

    R = euler_xyz_to_R(rx, ry, rz)
    t = np.array([x, y, z], dtype=float)

    return R, t


# ================================================================
# TAG COORDINATE OUTPUT (for motion planning team)
# ================================================================
class TagCoordinates:
    """
    Container for AprilTag pose data to pass to motion planning.
    """
    def __init__(self):
        self.tag_id = None
        self.timestamp = None
        
        # Camera frame coordinates
        self.cam_x = None
        self.cam_y = None
        self.cam_z = None
        self.cam_distance = None
        
        # Base frame coordinates (USE THESE FOR ARM MOTION)
        self.base_x = None
        self.base_y = None
        self.base_z = None
        self.base_distance = None
        
        # Rotation (Euler angles in degrees)
        self.base_roll = None
        self.base_pitch = None
        self.base_yaw = None
        
        # Rotation matrix (for precise orientation control)
        self.R_base_tag = None
        
        # Valid flag
        self.valid = False
    
    def to_dict(self):
        """Export as dictionary for JSON serialization or inter-process communication."""
        return {
            "tag_id": self.tag_id,
            "timestamp": self.timestamp,
            "valid": self.valid,
            "camera_frame": {
                "x_m": self.cam_x,
                "y_m": self.cam_y,
                "z_m": self.cam_z,
                "distance_m": self.cam_distance,
            },
            "base_frame": {
                "x_m": self.base_x,
                "y_m": self.base_y,
                "z_m": self.base_z,
                "distance_m": self.base_distance,
                "roll_deg": self.base_roll,
                "pitch_deg": self.base_pitch,
                "yaw_deg": self.base_yaw,
            }
        }
    
    def __str__(self):
        if not self.valid:
            return "TagCoordinates(invalid - no tag detected)"
        return (
            f"TagCoordinates(id={self.tag_id}, "
            f"base=[{self.base_x:.3f}, {self.base_y:.3f}, {self.base_z:.3f}]m, "
            f"dist={self.base_distance:.3f}m)"
        )


def compute_tag_coordinates(
    tag_id: int,
    R_cam_tag: np.ndarray,
    t_cam_tag: np.ndarray,
    R_base_tool: np.ndarray,
    t_base_tool: np.ndarray,
) -> TagCoordinates:
    """
    Compute full tag coordinates in both camera and base frames.
    
    Args:
        tag_id: AprilTag ID
        R_cam_tag: Rotation matrix from camera to tag
        t_cam_tag: Translation vector from camera to tag (meters)
        R_base_tool: Current robot base→tool rotation
        t_base_tool: Current robot base→tool translation (meters)
    
    Returns:
        TagCoordinates object with all pose data
    """
    coords = TagCoordinates()
    coords.tag_id = tag_id
    coords.timestamp = datetime.now().isoformat()
    
    # Camera frame
    coords.cam_x = float(t_cam_tag[0])
    coords.cam_y = float(t_cam_tag[1])
    coords.cam_z = float(t_cam_tag[2])
    coords.cam_distance = float(np.linalg.norm(t_cam_tag))
    
    # Compute Base→Tag transformation
    # Chain: Base→Tool→Cam→Tag
    R_base_tag, t_base_tag = compose_base_tag(
        R_base_tool, t_base_tool,
        R_tool_cam, t_tool_cam,
        R_cam_tag, t_cam_tag
    )
    
    # Base frame position
    coords.base_x = float(t_base_tag[0])
    coords.base_y = float(t_base_tag[1])
    coords.base_z = float(t_base_tag[2])
    coords.base_distance = float(np.linalg.norm(t_base_tag))
    
    # Base frame orientation
    coords.R_base_tag = R_base_tag
    yaw, pitch, roll = rotation_to_euler_xyz(R_base_tag)
    coords.base_roll = float(np.degrees(roll))
    coords.base_pitch = float(np.degrees(pitch))
    coords.base_yaw = float(np.degrees(yaw))
    
    coords.valid = True
    return coords


# ================================================================
# ARG PARSER
# ================================================================
def parse_args():
    p = argparse.ArgumentParser(
        description="AprilTag detection and pose estimation for Kinova Gen3 arm"
    )
    p.add_argument(
        "--ip",
        type=str,
        required=True,
        help="Robot IP address (required)",
    )
    p.add_argument(
        "-u", "--username",
        type=str,
        required=True,
        help="Robot username (required)"
    )
    p.add_argument(
        "-p", "--password",
        type=str,
        required=True,
        help="Robot password (required)"
    )
    p.add_argument(
        "--log",
        action="store_true",
        help="Log tag coordinates to console each frame"
    )
    p.add_argument(
        "--log-file",
        type=str,
        default=None,
        help="Log tag coordinates to CSV file"
    )
    p.add_argument(
        "--gt-x", type=float, default=None,
        help="Ground-truth Base→Tag X position [m] for accuracy validation"
    )
    p.add_argument(
        "--gt-y", type=float, default=None,
        help="Ground-truth Base→Tag Y position [m] for accuracy validation"
    )
    p.add_argument(
        "--gt-z", type=float, default=None,
        help="Ground-truth Base→Tag Z position [m] for accuracy validation"
    )
    return p.parse_args()


# ================================================================
# VISUALIZATION HELPERS
# ================================================================
def draw_tag_axes(frame, R, t, camera_params, axis_length):
    """Draw 3D coordinate axes on the tag. Red=X, Green=Y, Blue=Z"""
    fx, fy, cx, cy = camera_params
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    
    axis_len = axis_length * 0.5
    axes_3d = np.array([
        [0, 0, 0],
        [axis_len, 0, 0],
        [0, axis_len, 0],
        [0, 0, -axis_len],
    ], dtype=np.float32).T
    
    axes_cam = R @ axes_3d + t.reshape(3, 1)
    axes_2d = K @ axes_cam
    axes_2d = axes_2d[:2, :] / axes_2d[2, :]
    axes_2d = axes_2d.T.astype(int)
    
    origin = tuple(axes_2d[0])
    cv2.line(frame, origin, tuple(axes_2d[1]), (0, 0, 255), 2)  # X = Red
    cv2.line(frame, origin, tuple(axes_2d[2]), (0, 255, 0), 2)  # Y = Green
    cv2.line(frame, origin, tuple(axes_2d[3]), (255, 0, 0), 2)  # Z = Blue


# ================================================================
# MAIN VIEWER
# ================================================================
def main():
    args = parse_args()
    
    # Print configuration
    print_config()

    # Ground truth for validation (optional)
    gt_vec = None
    if args.gt_x is not None and args.gt_y is not None and args.gt_z is not None:
        gt_vec = np.array([args.gt_x, args.gt_y, args.gt_z], dtype=float)
        print(f"[Ground Truth] Base→Tag: x={gt_vec[0]:.3f}, y={gt_vec[1]:.3f}, z={gt_vec[2]:.3f} m")

    # CSV logging setup
    csv_file = None
    if args.log_file:
        csv_file = open(args.log_file, 'w')
        csv_file.write("timestamp,tag_id,cam_x,cam_y,cam_z,cam_dist,base_x,base_y,base_z,base_dist,roll,pitch,yaw\n")
        print(f"[Logging] Writing to: {args.log_file}")

    # ---------- Connect to robot ----------
    class ConnArgs:
        def __init__(self, ip, username, password):
            self.ip = ip
            self.username = username
            self.password = password

    conn_args = ConnArgs(args.ip, args.username, args.password)

    try:
        with DeviceConnection.create_tcp_connection(conn_args) as router:
            base_client = BaseClient(router)
            print("[✓] Connected to Kinova Gen3 robot")
            
            # Test robot connection
            R_test, t_test = get_robot_pose(base_client)
            print(f"[✓] Current tool position: x={t_test[0]:.3f}, y={t_test[1]:.3f}, z={t_test[2]:.3f} m")

            # ---------- Open RTSP camera stream ----------
            url = rtsp_url(args.ip)
            print(f"[INFO] Opening camera stream: {url}")
            cap = cv2.VideoCapture(url)

            if not cap.isOpened():
                print(f"[ERROR] Could not open RTSP stream: {url}")
                return

            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"[INFO] Stream resolution: {w} × {h}")

            detector = create_detector()

            # Smoothing buffers for stable display
            pos_history = deque(maxlen=5)
            
            print("\n" + "=" * 60)
            print("APRILTAG VIEWER RUNNING")
            print("=" * 60)
            print("Controls:")
            print("  'q' - Quit")
            print("  's' - Print current tag coordinates to console")
            print("=" * 60 + "\n")

            last_coords = TagCoordinates()

            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[!] Failed to read frame")
                    break

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Detect AprilTags
                results = detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=camera_params_tag,
                    tag_size=tag_size,
                )

                # Get current robot pose
                R_base_tool, t_base_tool = get_robot_pose(base_client)

                coords = TagCoordinates()
                pos_err = None

                for r in results:
                    # Draw tag outline
                    pts = np.array([r.corners], dtype=np.int32)
                    cv2.polylines(frame, pts, True, (0, 255, 0), 3)
                    
                    # Draw center
                    center = r.center.astype(int)
                    cv2.circle(frame, tuple(center), 6, (0, 0, 255), -1)

                    # Tag ID label
                    ptA = r.corners[0]
                    cv2.putText(
                        frame,
                        f"ID: {r.tag_id}",
                        (int(ptA[0]), int(ptA[1]) - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 0, 0),
                        2,
                    )

                    # Extract pose
                    t_cam_tag = r.pose_t[:, 0]
                    R_cam_tag = r.pose_R

                    # Compute full coordinates
                    coords = compute_tag_coordinates(
                        r.tag_id,
                        R_cam_tag,
                        t_cam_tag,
                        R_base_tool,
                        t_base_tool
                    )
                    
                    # Smoothing
                    pos_history.append([coords.base_x, coords.base_y, coords.base_z])
                    if len(pos_history) >= 3:
                        avg_pos = np.mean(pos_history, axis=0)
                        coords.base_x = float(avg_pos[0])
                        coords.base_y = float(avg_pos[1])
                        coords.base_z = float(avg_pos[2])
                        coords.base_distance = float(np.linalg.norm(avg_pos))

                    last_coords = coords

                    # Draw coordinate axes on tag
                    draw_tag_axes(frame, R_cam_tag, t_cam_tag, camera_params_tag, tag_size)

                    # Compute error vs ground truth
                    if gt_vec is not None:
                        base_pos = np.array([coords.base_x, coords.base_y, coords.base_z])
                        pos_err = base_pos - gt_vec

                    # Console logging
                    if args.log:
                        print(coords)

                    # CSV logging
                    if csv_file:
                        csv_file.write(
                            f"{coords.timestamp},{coords.tag_id},"
                            f"{coords.cam_x:.4f},{coords.cam_y:.4f},{coords.cam_z:.4f},{coords.cam_distance:.4f},"
                            f"{coords.base_x:.4f},{coords.base_y:.4f},{coords.base_z:.4f},{coords.base_distance:.4f},"
                            f"{coords.base_roll:.2f},{coords.base_pitch:.2f},{coords.base_yaw:.2f}\n"
                        )
                        csv_file.flush()

                    break  # Process only first tag

                # ---------------- HUD ----------------
                hud_h = 220 if gt_vec is not None else 180
                cv2.rectangle(frame, (5, 5), (580, hud_h), (0, 0, 0), -1)
                cv2.rectangle(frame, (5, 5), (580, hud_h), (100, 100, 100), 2)
                y0 = 28
                dy = 26

                if coords.valid:
                    # Camera frame position
                    cv2.putText(
                        frame,
                        f"CAM->TAG: x={coords.cam_x:+.3f}  y={coords.cam_y:+.3f}  z={coords.cam_z:+.3f} m",
                        (15, y0),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )
                    y0 += dy

                    cv2.putText(
                        frame,
                        f"Distance from camera: {coords.cam_distance:.3f} m",
                        (15, y0),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        1,
                    )
                    y0 += dy + 5

                    # Base frame position (MAIN OUTPUT)
                    cv2.putText(
                        frame,
                        f"BASE->TAG: x={coords.base_x:+.3f}  y={coords.base_y:+.3f}  z={coords.base_z:+.3f} m",
                        (15, y0),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 200, 255),
                        2,
                    )
                    y0 += dy

                    cv2.putText(
                        frame,
                        f"Distance from base: {coords.base_distance:.3f} m",
                        (15, y0),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 200, 255),
                        1,
                    )
                    y0 += dy

                    # Orientation
                    cv2.putText(
                        frame,
                        f"Orientation: R={coords.base_roll:.1f} P={coords.base_pitch:.1f} Y={coords.base_yaw:.1f} deg",
                        (15, y0),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (200, 200, 200),
                        1,
                    )
                    y0 += dy

                    # Ground truth comparison
                    if gt_vec is not None and pos_err is not None:
                        err_norm = np.linalg.norm(pos_err) * 100  # cm
                        cv2.putText(
                            frame,
                            f"Error vs GT: dx={pos_err[0]*100:+.1f} dy={pos_err[1]*100:+.1f} dz={pos_err[2]*100:+.1f} cm  |err|={err_norm:.1f} cm",
                            (15, y0),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 0),
                            1,
                        )

                else:
                    cv2.putText(
                        frame,
                        "No AprilTag detected",
                        (15, y0),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                    )
                    y0 += dy
                    cv2.putText(
                        frame,
                        "Point camera at an AprilTag (tag36h11)",
                        (15, y0),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (150, 150, 150),
                        1,
                    )

                cv2.imshow("Kinova AprilTag Viewer", frame)
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    # Print current coordinates
                    print("\n" + "=" * 50)
                    print("CURRENT TAG COORDINATES")
                    print("=" * 50)
                    if last_coords.valid:
                        print(f"Tag ID: {last_coords.tag_id}")
                        print(f"\nCamera Frame:")
                        print(f"  x = {last_coords.cam_x:+.4f} m")
                        print(f"  y = {last_coords.cam_y:+.4f} m")
                        print(f"  z = {last_coords.cam_z:+.4f} m")
                        print(f"  distance = {last_coords.cam_distance:.4f} m")
                        print(f"\nBase Frame (USE FOR ARM MOTION):")
                        print(f"  x = {last_coords.base_x:+.4f} m")
                        print(f"  y = {last_coords.base_y:+.4f} m")
                        print(f"  z = {last_coords.base_z:+.4f} m")
                        print(f"  distance = {last_coords.base_distance:.4f} m")
                        print(f"\nOrientation:")
                        print(f"  roll  = {last_coords.base_roll:+.2f} deg")
                        print(f"  pitch = {last_coords.base_pitch:+.2f} deg")
                        print(f"  yaw   = {last_coords.base_yaw:+.2f} deg")
                    else:
                        print("No tag currently detected")
                    print("=" * 50 + "\n")

            cap.release()
            cv2.destroyAllWindows()
            
            if csv_file:
                csv_file.close()
                
            print("[*] Viewer closed.")

    except Exception as e:
        print(f"[ERROR] {e}")
        raise


# ================================================================
# PUBLIC API - For importing into other modules
# ================================================================
def get_tag_coordinates(base_client: BaseClient, detector, frame) -> TagCoordinates:
    """
    Single-frame AprilTag detection for use by other modules.
    
    Args:
        base_client: Connected Kinova BaseClient
        detector: AprilTag detector instance
        frame: BGR image from camera
    
    Returns:
        TagCoordinates object (check .valid before using)
    
    Example:
        from robot.apriltag_viewer import get_tag_coordinates, create_detector
        
        coords = get_tag_coordinates(base_client, detector, frame)
        if coords.valid:
            target_x = coords.base_x
            target_y = coords.base_y
            target_z = coords.base_z
            # Send to motion planner...
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    results = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params_tag,
        tag_size=tag_size,
    )
    
    if not results:
        return TagCoordinates()
    
    r = results[0]
    R_base_tool, t_base_tool = get_robot_pose(base_client)
    
    return compute_tag_coordinates(
        r.tag_id,
        r.pose_R,
        r.pose_t[:, 0],
        R_base_tool,
        t_base_tool
    )


if __name__ == "__main__":
    main()
