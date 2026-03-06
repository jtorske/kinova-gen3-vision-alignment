# robot/offline_apriltag_viewer.py
#
# Offline AprilTag viewer for use outside the lab (no robotic arm needed).
#
# Mirrors the UI/structure of robot/apriltag_viewer.py, but operates only in
# the camera frame:
#   - Uses any standard webcam (no RTSP / no Kinova connection required)
#   - Detects AprilTags using provided/estimated camera intrinsics
#   - Computes Cam→Tag pose (position + orientation)
#   - Optional CSV logging + optional ground-truth error in camera frame
#
# Coordinate Frames:
#   - Camera Frame: Origin at camera lens, Z forward, X right, Y down
#   - Tag Frame: Origin at tag center

import cv2
import numpy as np
import argparse
from collections import deque
from datetime import datetime

from common.utils import rotation_to_euler_xyz


# ================================================================
# DEFAULTS (webcam-friendly)
# ================================================================
DEFAULT_TAG_SIZE = 0.05        # meters (5 cm)
DEFAULT_TAG_FAMILY = "tag36h11"


def estimate_focal_from_resolution(w: int, h: int) -> float:
    """
    Rough focal estimate assuming ~65° horizontal FOV typical for webcams.
    fx ≈ width / (2*tan(HFOV/2)) ≈ width * 0.78
    """
    return max(w, h) * 0.78


def print_config(cam_params, tag_size, tag_family, camera_index, w, h, log_file, gt_vec):
    fx, fy, cx, cy = cam_params
    print("=" * 60)
    print("OFFLINE APRILTAG VIEWER - CONFIGURATION")
    print("=" * 60)
    print(f"[Camera Index]: {camera_index}")
    print(f"[Resolution]: {w} × {h}")
    print(f"[Camera Intrinsics (used)]")
    print(f"  fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
    print(f"[Tag Family]: {tag_family}")
    print(f"[Tag Size]: {tag_size:.4f} m ({tag_size*100:.1f} cm)")
    if gt_vec is not None:
        print(f"[Ground Truth] Cam→Tag: x={gt_vec[0]:.3f}, y={gt_vec[1]:.3f}, z={gt_vec[2]:.3f} m")
    if log_file:
        print(f"[Logging] Writing to: {log_file}")
    print("=" * 60)


# ================================================================
# ARG PARSER (style matches apriltag_viewer.py)
# ================================================================
def parse_args():
    p = argparse.ArgumentParser(
        description="Offline AprilTag detection and pose estimation (webcam, no robot)"
    )
    p.add_argument("--camera", type=int, default=0, help="Webcam index (default: 0)")
    p.add_argument("--width", type=int, default=1280, help="Capture width (default: 1280)")
    p.add_argument("--height", type=int, default=720, help="Capture height (default: 720)")
    p.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE, help="AprilTag size [m]")
    p.add_argument("--tag-family", type=str, default=DEFAULT_TAG_FAMILY, help="AprilTag family (e.g., tag36h11)")

    # Optional intrinsics override
    p.add_argument("--fx", type=float, default=None, help="Focal length X [px] (optional)")
    p.add_argument("--fy", type=float, default=None, help="Focal length Y [px] (optional)")
    p.add_argument("--cx", type=float, default=None, help="Principal point X [px] (optional)")
    p.add_argument("--cy", type=float, default=None, help="Principal point Y [px] (optional)")

    # Logging + ground truth, same “shape” as online viewer
    p.add_argument("--log", action="store_true", help="Log tag coordinates to console each frame")
    p.add_argument("--log-file", type=str, default=None, help="Log tag coordinates to CSV file")
    p.add_argument("--gt-x", type=float, default=None, help="Ground-truth Cam→Tag X [m] (optional)")
    p.add_argument("--gt-y", type=float, default=None, help="Ground-truth Cam→Tag Y [m] (optional)")
    p.add_argument("--gt-z", type=float, default=None, help="Ground-truth Cam→Tag Z [m] (optional)")
    return p.parse_args()


# ================================================================
# TAG COORDINATE OUTPUT (camera frame only)
# ================================================================
class TagCoordinates:
    def __init__(self):
        self.tag_id = None
        self.timestamp = None

        # Camera frame coordinates
        self.cam_x = None
        self.cam_y = None
        self.cam_z = None
        self.cam_distance = None

        # Orientation (Euler in degrees)
        self.cam_roll = None
        self.cam_pitch = None
        self.cam_yaw = None

        self.valid = False

    def __str__(self):
        if not self.valid:
            return "TagCoordinates(invalid - no tag detected)"
        return (
            f"TagCoordinates(id={self.tag_id}, "
            f"cam=[{self.cam_x:.3f}, {self.cam_y:.3f}, {self.cam_z:.3f}]m, "
            f"dist={self.cam_distance:.3f}m)"
        )


def compute_cam_tag_coordinates(tag_id: int, R_cam_tag: np.ndarray, t_cam_tag: np.ndarray) -> TagCoordinates:
    coords = TagCoordinates()
    coords.tag_id = tag_id
    coords.timestamp = datetime.now().isoformat()

    coords.cam_x = float(t_cam_tag[0])
    coords.cam_y = float(t_cam_tag[1])
    coords.cam_z = float(t_cam_tag[2])
    coords.cam_distance = float(np.linalg.norm(t_cam_tag))

    yaw, pitch, roll = rotation_to_euler_xyz(R_cam_tag)
    coords.cam_roll = float(np.degrees(roll))
    coords.cam_pitch = float(np.degrees(pitch))
    coords.cam_yaw = float(np.degrees(yaw))

    coords.valid = True
    return coords


# ================================================================
# VISUALIZATION HELPERS (same naming as online viewer)
# ================================================================
def draw_tag_axes(frame, R, t, camera_params, axis_length):
    """Draw 3D coordinate axes on the tag. Red=X, Green=Y, Blue=Z"""
    fx, fy, cx, cy = camera_params
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    axis_len = axis_length * 0.5
    axes_3d = np.array(
        [
            [0, 0, 0],
            [axis_len, 0, 0],
            [0, axis_len, 0],
            [0, 0, -axis_len],
        ],
        dtype=np.float32,
    ).T

    axes_cam = R @ axes_3d + t.reshape(3, 1)
    axes_2d = K @ axes_cam
    axes_2d = axes_2d[:2, :] / axes_2d[2, :]
    axes_2d = axes_2d.T.astype(int)

    origin = tuple(axes_2d[0])
    cv2.line(frame, origin, tuple(axes_2d[1]), (0, 0, 255), 2)   # X
    cv2.line(frame, origin, tuple(axes_2d[2]), (0, 255, 0), 2)   # Y
    cv2.line(frame, origin, tuple(axes_2d[3]), (255, 0, 0), 2)   # Z


# ================================================================
# MAIN VIEWER (mirrors online viewer control flow)
# ================================================================
def main():
    args = parse_args()

    # ---------- Open webcam ----------
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera index {args.camera}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Intrinsics
    cx = args.cx if args.cx is not None else w / 2.0
    cy = args.cy if args.cy is not None else h / 2.0
    if args.fx is None or args.fy is None:
        est_f = estimate_focal_from_resolution(w, h)
    fx = args.fx if args.fx is not None else est_f
    fy = args.fy if args.fy is not None else est_f
    camera_params = (fx, fy, cx, cy)

    tag_size = args.tag_size
    tag_family = args.tag_family

    # Ground truth (camera frame)
    gt_vec = None
    if args.gt_x is not None and args.gt_y is not None and args.gt_z is not None:
        gt_vec = np.array([args.gt_x, args.gt_y, args.gt_z], dtype=float)

    # CSV logging
    csv_file = None
    if args.log_file:
        csv_file = open(args.log_file, "w")
        csv_file.write("timestamp,tag_id,cam_x,cam_y,cam_z,cam_dist,roll,pitch,yaw,err_dx,err_dy,err_dz,err_norm_cm\n")

    print_config(camera_params, tag_size, tag_family, args.camera, w, h, args.log_file, gt_vec)

    # Detector
    from pupil_apriltags import Detector
    detector = Detector(families=tag_family)

    # Smoothing buffers (match online viewer approach: position deque)
    pos_history = deque(maxlen=5)

    print("\n" + "=" * 60)
    print("OFFLINE APRILTAG VIEWER RUNNING")
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

        results = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size,
        )

        coords = TagCoordinates()
        pos_err = None

        for r in results:
            # Draw tag outline (match thickness/style)
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

            coords = compute_cam_tag_coordinates(r.tag_id, R_cam_tag, t_cam_tag)

            # Smoothing (position)
            pos_history.append([coords.cam_x, coords.cam_y, coords.cam_z])
            if len(pos_history) >= 3:
                avg_pos = np.mean(pos_history, axis=0)
                coords.cam_x = float(avg_pos[0])
                coords.cam_y = float(avg_pos[1])
                coords.cam_z = float(avg_pos[2])
                coords.cam_distance = float(np.linalg.norm(avg_pos))

            last_coords = coords

            # Draw coordinate axes on tag
            draw_tag_axes(frame, R_cam_tag, t_cam_tag, camera_params, tag_size)

            # Error vs ground truth (camera frame)
            if gt_vec is not None:
                cam_pos = np.array([coords.cam_x, coords.cam_y, coords.cam_z])
                pos_err = cam_pos - gt_vec

            # Console logging
            if args.log:
                print(coords)

            # CSV logging
            if csv_file:
                err_dx = err_dy = err_dz = err_norm_cm = ""
                if pos_err is not None:
                    err_dx = f"{pos_err[0]:.4f}"
                    err_dy = f"{pos_err[1]:.4f}"
                    err_dz = f"{pos_err[2]:.4f}"
                    err_norm_cm = f"{np.linalg.norm(pos_err) * 100:.2f}"
                csv_file.write(
                    f"{coords.timestamp},{coords.tag_id},"
                    f"{coords.cam_x:.4f},{coords.cam_y:.4f},{coords.cam_z:.4f},{coords.cam_distance:.4f},"
                    f"{coords.cam_roll:.2f},{coords.cam_pitch:.2f},{coords.cam_yaw:.2f},"
                    f"{err_dx},{err_dy},{err_dz},{err_norm_cm}\n"
                )
                csv_file.flush()

            break  # first tag only (matches online viewer)

        # ---------------- HUD (match online viewer style/spacing) ----------------
        hud_h = 220 if gt_vec is not None else 180
        cv2.rectangle(frame, (5, 5), (580, hud_h), (0, 0, 0), -1)
        cv2.rectangle(frame, (5, 5), (580, hud_h), (100, 100, 100), 2)
        y0 = 28
        dy = 26

        if coords.valid:
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

            cv2.putText(
                frame,
                f"Orientation: R={coords.cam_roll:.1f} P={coords.cam_pitch:.1f} Y={coords.cam_yaw:.1f} deg",
                (15, y0),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (200, 200, 200),
                1,
            )
            y0 += dy

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
                f"Point camera at an AprilTag ({tag_family})",
                (15, y0),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (150, 150, 150),
                1,
            )

        cv2.imshow("Offline AprilTag Viewer", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("s"):
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
                print(f"\nOrientation:")
                print(f"  roll  = {last_coords.cam_roll:+.2f} deg")
                print(f"  pitch = {last_coords.cam_pitch:+.2f} deg")
                print(f"  yaw   = {last_coords.cam_yaw:+.2f} deg")
            else:
                print("No tag currently detected")
            print("=" * 50 + "\n")

    cap.release()
    cv2.destroyAllWindows()

    if csv_file:
        csv_file.close()

    print("[*] Viewer closed.")


if __name__ == "__main__":
    main()
