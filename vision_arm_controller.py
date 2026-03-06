import sys
import os
import cv2
import argparse
import numpy as np
import time


_THIS_DIR = os.path.dirname(os.path.abspath(__file__))

sys.path.insert(0, os.path.join(_THIS_DIR, 'AprilTagMVP'))
sys.path.insert(0, os.path.join(_THIS_DIR, 'moving_arm', 'K3N'))
sys.path.insert(0, os.path.join(_THIS_DIR, 'computer_vision'))

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from vision.robot.device_connection import DeviceConnection  # type: ignore
from vision.robot.apriltag_viewer import (  # type: ignore
    get_robot_pose,
    compute_tag_coordinates,
    draw_tag_axes,
    camera_params_tag,
    TagCoordinates,
)
from common.utils import create_detector  # type: ignore
from common.webcam_config import rtsp_url, tag_size  # type: ignore
from common.tool_cam_config import R_tool_cam  # type: ignore
from movement.auto_move import AutonomousMovement  # type: ignore

Z_OFFSET = 0.25
MOVE_SPEED = 0.05
ROTATION_SPEED = 0.15
MAX_REACH = 0.85
TAG_LOST_TIMEOUT = 3.0
ALIGN_THRESHOLD_DEG = 0.5
CENTER_THRESHOLD_M = 0.02
CENTER_CORRECTION_GAIN = 0.3
BACKUP_DISTANCE = 0.10
EDGE_MARGIN_PERCENT = 0.25
EDGE_CORRECTION_GAIN = 0.5


FINAL_APPROACH_OFFSET = 0.0

PHASE_IDLE = 0
PHASE_ALIGN_ROLL = 1
PHASE_ALIGN_PITCH = 2
PHASE_ALIGN_YAW = 3
PHASE_TRACKING = 4
PHASE_FINAL_ROLL = 5
PHASE_FINAL_PITCH = 6
PHASE_FINAL_YAW = 7
PHASE_FINAL_MOVE = 8
PHASE_COMPLETE = 9

PHASE_NAMES = {
    PHASE_IDLE: "IDLE",
    PHASE_ALIGN_ROLL: "ALIGN_ROLL",
    PHASE_ALIGN_PITCH: "ALIGN_PITCH",
    PHASE_ALIGN_YAW: "ALIGN_YAW",
    PHASE_TRACKING: "TRACKING",
    PHASE_FINAL_ROLL: "FINAL_ROLL",
    PHASE_FINAL_PITCH: "FINAL_PITCH",
    PHASE_FINAL_YAW: "FINAL_YAW",
    PHASE_FINAL_MOVE: "FINAL_MOVE",
    PHASE_COMPLETE: "COMPLETE"
}


def compute_alignment_error(current_pose, coords):
    target_yaw = coords.base_yaw
    target_pitch = coords.base_pitch
    target_roll = coords.base_roll
    
    current_roll = current_pose.theta_z
    current_pitch = current_pose.theta_y
    current_yaw = current_pose.theta_x
    
    def angle_diff(a, b):
        diff = a - b
        while diff > 180: diff -= 360
        while diff < -180: diff += 360
        if abs(abs(diff) - 180) < ALIGN_THRESHOLD_DEG:
            return 0
        return diff
    
    def normalize_angle(a):
        while a > 180: a -= 360
        while a < -180: a += 360
        return a
    
    target_yaw = 180
    err_yaw = target_yaw - current_yaw
    while err_yaw > 180: err_yaw -= 360
    while err_yaw < -180: err_yaw += 360
    
    target_pitch = 180
    err_pitch = target_pitch - current_pitch
    while err_pitch > 180: err_pitch -= 360
    while err_pitch < -180: err_pitch += 360
    
    target_roll_normal = target_roll
    target_roll_flipped = normalize_angle(target_roll + 180)
    
    err_roll_normal = angle_diff(target_roll_normal, current_roll)
    err_roll_flipped = angle_diff(target_roll_flipped, current_roll)
    
    if abs(err_roll_flipped) < abs(err_roll_normal):
        err_roll = err_roll_flipped
        target_roll = target_roll_flipped
    else:
        err_roll = err_roll_normal
    
    return err_roll, err_pitch, err_yaw, target_roll, target_pitch, target_yaw


def main():
    parser = argparse.ArgumentParser(description="Vision-guided arm controller")
    parser.add_argument("--ip", type=str, required=True, help="Robot IP address")
    parser.add_argument("-u", "--username", type=str, required=True, help="Robot username")
    parser.add_argument("-p", "--password", type=str, required=True, help="Robot password")
    args = parser.parse_args()

    class ConnArgs:
        def __init__(self, ip, username, password):
            self.ip = ip
            self.username = username
            self.password = password

    conn_args = ConnArgs(args.ip, args.username, args.password)

    with DeviceConnection.create_tcp_connection(conn_args) as router:
        base_client = BaseClient(router)
        movement = AutonomousMovement(base_client)
        detector = create_detector()
        
        print("[OK] Connected to robot")

        cap = cv2.VideoCapture(rtsp_url(args.ip))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if not cap.isOpened():
            print("[ERROR] Could not open camera")
            return

        print("\n" + "=" * 50)
        print("VISION ARM CONTROLLER (Relative Movement)")
 

        last_cam_tag = None
        last_coords = TagCoordinates()
        tracking_active = True
        tracking_phase = PHASE_ALIGN_ROLL
        last_command_time = 0
        tag_lost_time = None
        COMMAND_INTERVAL = 0.5
        
        last_err_roll = []
        last_err_pitch = []
        last_err_yaw = []
        locked_orientation = None
        has_backed_up = False
        tag_near_edge = False

        while True:
            for _ in range(3):
                cap.grab()
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params_tag,
                tag_size=tag_size,
            )

            R_base_tool, t_base_tool = get_robot_pose(base_client)
            coords = TagCoordinates()

            for r in results:
                pts = np.array([r.corners], dtype=np.int32)
                cv2.polylines(frame, pts, True, (0, 255, 0), 3)
                center = r.center.astype(int)
                cv2.circle(frame, tuple(center), 6, (0, 0, 255), -1)

                t_cam_tag = r.pose_t[:, 0]
                last_cam_tag = t_cam_tag.copy()
                
                R_cam_tag = r.pose_R
                coords = compute_tag_coordinates(
                    r.tag_id, R_cam_tag, t_cam_tag, R_base_tool, t_base_tool
                )
                last_coords = coords

                draw_tag_axes(frame, R_cam_tag, t_cam_tag, camera_params_tag, tag_size)
                break

            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2
            
            tag_near_edge = False
            edge_margin_x = int(w * EDGE_MARGIN_PERCENT)
            edge_margin_y = int(h * EDGE_MARGIN_PERCENT)
            if coords.valid:
                tag_cx, tag_cy = int(results[0].center[0]), int(results[0].center[1])
                if tag_cx < edge_margin_x or tag_cx > w - edge_margin_x or tag_cy < edge_margin_y or tag_cy > h - edge_margin_y:
                    tag_near_edge = True
                    cv2.rectangle(frame, (edge_margin_x, edge_margin_y), (w - edge_margin_x, h - edge_margin_y), (0, 0, 255), 2)
                else:
                    cv2.rectangle(frame, (edge_margin_x, edge_margin_y), (w - edge_margin_x, h - edge_margin_y), (0, 255, 0), 1)
            
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1)

            cv2.rectangle(frame, (5, 5), (520, 310), (0, 0, 0), -1)
            y = 28

            if tracking_active:
                if tag_lost_time is not None:
                    lost_duration = time.time() - tag_lost_time
                    cv2.putText(frame, f">>> TAG LOST {lost_duration:.1f}s - searching... <<<", (15, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
                elif tag_near_edge:
                    cv2.putText(frame, ">>> TAG NEAR EDGE - Centering... <<<", (15, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                elif tracking_phase in (PHASE_ALIGN_ROLL, PHASE_ALIGN_PITCH, PHASE_ALIGN_YAW):
                    phase_name = {PHASE_ALIGN_ROLL: "ROLL", PHASE_ALIGN_PITCH: "PITCH", PHASE_ALIGN_YAW: "YAW"}[tracking_phase]
                    cv2.putText(frame, f">>> ALIGNING {phase_name} - Press 'e' for E-STOP <<<", (15, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 165, 0), 2)
                else:
                    cv2.putText(frame, ">>> TRACKING tag - Press 'e' for E-STOP <<<", (15, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                y += 28

            if coords.valid:
                cv2.putText(frame, f"Tag {coords.tag_id} detected", (15, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y += 28
                cv2.putText(frame, f"CAM->TAG: x={coords.cam_x:+.3f} y={coords.cam_y:+.3f} z={coords.cam_z:+.3f} m",
                           (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                y += 24
                cv2.putText(frame, f"Distance from camera: {coords.cam_distance:.3f} m",
                           (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                y += 28
                cv2.putText(frame, f"Current tool pos: x={t_base_tool[0]:.3f} y={t_base_tool[1]:.3f} z={t_base_tool[2]:.3f}",
                           (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                y += 24
                current_pose = base_client.GetMeasuredCartesianPose()
                cv2.putText(frame, f"TOOL: Roll={current_pose.theta_z:.1f} Pitch={current_pose.theta_y:.1f} Yaw={current_pose.theta_x:.1f}",
                           (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 100), 1)
                y += 22
                cv2.putText(frame, f"TAG:  Roll={coords.base_roll:.1f} Pitch={coords.base_pitch:.1f} Yaw={coords.base_yaw:.1f}",
                           (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 200, 255), 1)
                y += 22
                
                def wrap_error(a, b):
                    diff = a - b
                    while diff > 180: diff -= 360
                    while diff < -180: diff += 360
                    if abs(abs(diff) - 180) < ALIGN_THRESHOLD_DEG:
                        return 0
                    return diff
                
                err_yaw = wrap_error(coords.base_yaw, current_pose.theta_x)
                err_pitch = wrap_error(coords.base_pitch, current_pose.theta_y)
                err_roll = wrap_error(coords.base_roll, current_pose.theta_z)
                
                def get_color(err):
                    if abs(err) < ALIGN_THRESHOLD_DEG:
                        return (0, 255, 0)
                    return (255, 100, 255)
                
                col_roll = get_color(err_roll)
                col_pitch = get_color(err_pitch)
                col_yaw = get_color(err_yaw)
                
                cv2.putText(frame, "Errors:", (15, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                cv2.putText(frame, f"Roll={err_roll:+.1f}", (80, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, col_roll, 1)
                cv2.putText(frame, f"Pitch={err_pitch:+.1f}", (180, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, col_pitch, 1)
                cv2.putText(frame, f"Yaw={err_yaw:+.1f}", (295, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, col_yaw, 1)
                y += 26
                
                centered_x = abs(coords.cam_x) < CENTER_THRESHOLD_M
                centered_y = abs(coords.cam_y) < CENTER_THRESHOLD_M
                center_color = (0, 255, 0) if (centered_x and centered_y) else (255, 100, 255)
                cv2.putText(frame, f"Center offset: X={coords.cam_x*100:+.1f}cm Y={coords.cam_y*100:+.1f}cm", (15, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, center_color, 1)
                y += 26
                
                
            else:
                cv2.putText(frame, "No tag detected", (15, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("Vision Arm Controller", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            if tracking_active and not coords.valid:
                if tag_lost_time is None:
                    tag_lost_time = time.time()
                    print("[WARN] Tag lost - stopping arm, searching...")
                    try:
                        base_client.Stop()
                    except:
                        pass
                elif not has_backed_up and time.time() - tag_lost_time > 1.0:
                    print(f"[RECOVER] Tag still lost - backing up {BACKUP_DISTANCE*100:.0f}cm to recapture...")
                    has_backed_up = True
                    R_base_tool, t_base_tool = get_robot_pose(base_client)
                    current_pose = base_client.GetMeasuredCartesianPose()
                    R_base_cam = R_base_tool @ R_tool_cam
                    backup_offset_cam = np.array([0.0, 0.0, -BACKUP_DISTANCE])
                    backup_offset_base = R_base_cam @ backup_offset_cam
                    backup_pos = [
                        t_base_tool[0] + backup_offset_base[0],
                        t_base_tool[1] + backup_offset_base[1],
                        t_base_tool[2] + backup_offset_base[2]
                    ]
                    backup_orientation = [current_pose.theta_x, current_pose.theta_y, current_pose.theta_z]
                    movement.cartesian_action_movement(backup_pos, backup_orientation, MOVE_SPEED, blocking=True)
                    tag_lost_time = time.time()
                elif time.time() - tag_lost_time > TAG_LOST_TIMEOUT:
                    print(f"[WARN] Tag lost for {TAG_LOST_TIMEOUT}s - giving up")
                    tracking_active = False
                    tracking_phase = PHASE_IDLE
                    tag_lost_time = None
                    locked_orientation = None
                    has_backed_up = False
            elif coords.valid:
                if tag_lost_time is not None:
                    print("[OK] Tag reacquired")
                tag_lost_time = None
            
            if tracking_active and coords.valid and last_cam_tag is not None:
                if tag_near_edge:
                    R_base_cam = R_base_tool @ R_tool_cam
                    correction_gain = EDGE_CORRECTION_GAIN
                    center_offset_cam = np.array([
                        coords.cam_x * correction_gain,
                        coords.cam_y * correction_gain,
                        0.0
                    ])
                    center_offset_base = R_base_cam @ center_offset_cam
                    position = [
                        t_base_tool[0] + center_offset_base[0],
                        t_base_tool[1] + center_offset_base[1],
                        t_base_tool[2] + center_offset_base[2]
                    ]
                    orientation = [current_pose.theta_x, current_pose.theta_y, current_pose.theta_z]  
                    print("[EDGE RECOVERY] Tag outside box, only centering x/y")
                    movement.cartesian_action_movement(position, orientation, MOVE_SPEED, blocking=False)
                    continue  


                if tracking_phase == PHASE_TRACKING and coords.cam_z < Z_OFFSET:
                    print(f"[PHASE: {PHASE_NAMES[PHASE_TRACKING]}] Arrived at tag (distance: {coords.cam_z:.3f}m)")
                    print(f"[PHASE: {PHASE_NAMES[PHASE_FINAL_ROLL]}] Starting final alignment...")
                    tracking_phase = PHASE_FINAL_ROLL
                    locked_orientation = None
                    last_err_roll.clear()
                    last_err_pitch.clear()
                    last_err_yaw.clear()
                    continue
                
                if tracking_phase == PHASE_COMPLETE:
                    continue
                
                current_time = time.time()
                if not tag_near_edge and current_time - last_command_time < COMMAND_INTERVAL:
                    continue
                last_command_time = current_time
                
                R_base_tool, t_base_tool = get_robot_pose(base_client)
                current_pose = base_client.GetMeasuredCartesianPose()
                
                err_roll, err_pitch, err_yaw, target_roll, target_pitch, target_yaw = compute_alignment_error(current_pose, coords)
                
                if tracking_phase in (PHASE_ALIGN_ROLL, PHASE_ALIGN_PITCH, PHASE_ALIGN_YAW,
                                      PHASE_FINAL_ROLL, PHASE_FINAL_PITCH, PHASE_FINAL_YAW):
                    R_base_cam = R_base_tool @ R_tool_cam
                    center_offset_cam = np.array([0.0, 0.0, 0.0])
                    
                    if tag_near_edge:
                        correction_gain = EDGE_CORRECTION_GAIN
                        print(f"[WARN] Tag near edge! Urgent centering correction (cam_x={coords.cam_x:.3f}, cam_y={coords.cam_y:.3f})")
                    else:
                        correction_gain = CENTER_CORRECTION_GAIN
                    
                    if abs(coords.cam_x) > CENTER_THRESHOLD_M or abs(coords.cam_y) > CENTER_THRESHOLD_M or tag_near_edge:
                        center_offset_cam = np.array([
                            coords.cam_x * correction_gain,
                            coords.cam_y * correction_gain,
                            0.0
                        ])
                    center_offset_base = R_base_cam @ center_offset_cam
                    position = [
                        t_base_tool[0] + center_offset_base[0],
                        t_base_tool[1] + center_offset_base[1],
                        t_base_tool[2] + center_offset_base[2]
                    ]
                    
                    adj_yaw = current_pose.theta_x
                    adj_pitch = current_pose.theta_y
                    adj_roll = current_pose.theta_z
                    
                    last_err_roll.append(err_roll)
                    last_err_pitch.append(err_pitch)
                    last_err_yaw.append(err_yaw)
                    if len(last_err_roll) > 6: last_err_roll.pop(0)
                    if len(last_err_pitch) > 6: last_err_pitch.pop(0)
                    if len(last_err_yaw) > 6: last_err_yaw.pop(0)
                    
                    def is_oscillating(err_history):
                        if len(err_history) < 4:
                            return False
                        for i in range(1, len(err_history)):
                            if abs(err_history[i] - err_history[i-1]) > 100:
                                return True
                        signs = [1 if e > 0 else -1 for e in err_history]
                        flips = sum(1 for i in range(1, len(signs)) if signs[i] != signs[i-1])
                        return flips >= 3
                    
                    def is_aligned_or_done(err, err_history):
                        if abs(err) < ALIGN_THRESHOLD_DEG:
                            return True, "aligned"
                        if is_oscillating(err_history):
                            min_err = min(abs(e) for e in err_history)
                            return True, f"oscillating (best was {min_err:.1f}°)"
                        return False, None
                    
                    if tracking_phase == PHASE_ALIGN_ROLL:
                        aligned, reason = is_aligned_or_done(err_roll, last_err_roll)
                        if aligned:
                            print(f"[PHASE: {PHASE_NAMES[PHASE_ALIGN_ROLL]}] Roll {reason} (err={err_roll:.1f}°)")
                            print(f"[PHASE: {PHASE_NAMES[PHASE_ALIGN_PITCH]}] Starting pitch alignment...")
                            tracking_phase = PHASE_ALIGN_PITCH
                            last_err_roll.clear()
                            continue
                        adj_roll = target_roll
                        print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] roll_err={err_roll:.1f}° -> target_roll={target_roll:.1f}°")
                    
                    elif tracking_phase == PHASE_ALIGN_PITCH:
                        aligned, reason = is_aligned_or_done(err_pitch, last_err_pitch)
                        if aligned:
                            print(f"[PHASE: {PHASE_NAMES[PHASE_ALIGN_PITCH]}] Pitch {reason} (err={err_pitch:.1f}°)")
                            print(f"[PHASE: {PHASE_NAMES[PHASE_ALIGN_YAW]}] Starting yaw alignment...")
                            tracking_phase = PHASE_ALIGN_YAW
                            last_err_pitch.clear()
                            continue
                        adj_pitch = target_pitch
                        print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] pitch_err={err_pitch:.1f}° -> target_pitch={target_pitch:.1f}°")
                    
                    elif tracking_phase == PHASE_ALIGN_YAW:
                        aligned, reason = is_aligned_or_done(err_yaw, last_err_yaw)
                        if aligned:
                            print(f"[PHASE: {PHASE_NAMES[PHASE_ALIGN_YAW]}] Yaw {reason} (err={err_yaw:.1f}°)")
                            locked_orientation = [current_pose.theta_x, current_pose.theta_y, current_pose.theta_z]
                            print(f"[PHASE: {PHASE_NAMES[PHASE_TRACKING]}] Orientation locked, starting approach...")
                            tracking_phase = PHASE_TRACKING
                            last_err_yaw.clear()
                            continue
                        adj_yaw = current_pose.theta_x - err_yaw
                        print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] yaw_err={err_yaw:.1f}° -> adj_yaw={adj_yaw:.1f}°")
                    
                    elif tracking_phase == PHASE_FINAL_ROLL:
                        aligned, reason = is_aligned_or_done(err_roll, last_err_roll)
                        if aligned:
                            print(f"[PHASE: {PHASE_NAMES[PHASE_FINAL_ROLL]}] Roll {reason} (err={err_roll:.1f}°)")
                            print(f"[PHASE: {PHASE_NAMES[PHASE_FINAL_PITCH]}] Final pitch alignment...")
                            tracking_phase = PHASE_FINAL_PITCH
                            last_err_roll.clear()
                            continue
                        adj_roll = current_pose.theta_z + err_roll * 0.5
                        print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] roll_err={err_roll:.1f}° -> adj_roll={adj_roll:.1f}°")
                    
                    elif tracking_phase == PHASE_FINAL_PITCH:
                        aligned, reason = is_aligned_or_done(err_pitch, last_err_pitch)
                        if aligned:
                            print(f"[PHASE: {PHASE_NAMES[PHASE_FINAL_PITCH]}] Pitch {reason} (err={err_pitch:.1f}°)")
                            print(f"[PHASE: {PHASE_NAMES[PHASE_FINAL_YAW]}] Final yaw alignment...")
                            tracking_phase = PHASE_FINAL_YAW
                            last_err_pitch.clear()
                            continue
                        adj_pitch = current_pose.theta_y + err_pitch * 0.5
                        print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] pitch_err={err_pitch:.1f}° -> adj_pitch={adj_pitch:.1f}°")
                    
                    elif tracking_phase == PHASE_FINAL_YAW:
                        yaw_near_180 = abs(abs(err_yaw) - 180) < 5.0
                        if yaw_near_180 or abs(err_yaw) < ALIGN_THRESHOLD_DEG:
                            yaw_stable = True
                            all_stable = (abs(err_roll) < ALIGN_THRESHOLD_DEG and 
                                         abs(err_pitch) < ALIGN_THRESHOLD_DEG)
                            if all_stable:
                                print(f"[PHASE: {PHASE_NAMES[PHASE_FINAL_YAW]}] All angles stabilized (roll={err_roll:.1f}° pitch={err_pitch:.1f}° yaw={err_yaw:.1f}°)")
                                print(f"[PHASE: {PHASE_NAMES[PHASE_FINAL_MOVE]}] Starting final 5cm approach...")
                                tracking_phase = PHASE_FINAL_MOVE
                            else:
                                print(f"[PHASE: {PHASE_NAMES[PHASE_FINAL_YAW]}] Yaw OK, but not stable yet (roll={err_roll:.1f}° pitch={err_pitch:.1f}°) - restarting final alignment")
                                tracking_phase = PHASE_FINAL_ROLL
                                last_err_roll.clear()
                                last_err_pitch.clear()
                                last_err_yaw.clear()
                            continue
                        adj_yaw = current_pose.theta_x - err_yaw * 0.5
                        print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] yaw_err={err_yaw:.1f}° -> adj_yaw={adj_yaw:.1f}°")
                    
                    orientation = [adj_yaw, adj_pitch, adj_roll]
                    success = movement.cartesian_action_movement(
                        position, orientation, ROTATION_SPEED, blocking=False
                    )
                    
                    if not success:
                        print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] Alignment command failed, stopping")
                        tracking_active = False
                        tracking_phase = PHASE_IDLE
                        locked_orientation = None
                    continue
                
                if tracking_phase == PHASE_TRACKING:
                    if locked_orientation is None:
                        locked_orientation = [current_pose.theta_x, current_pose.theta_y, current_pose.theta_z]
                    
                    orientation = locked_orientation
                    
                    R_base_cam = R_base_tool @ R_tool_cam
                    
                    approach_distance = coords.cam_z - Z_OFFSET
                    
                    combined_offset_cam = np.array([
                        coords.cam_x,
                        coords.cam_y,
                        approach_distance
                    ])
                    
                    combined_offset_base = R_base_cam @ combined_offset_cam
                    
                    target_pos_x = t_base_tool[0] + combined_offset_base[0]
                    target_pos_y = t_base_tool[1] + combined_offset_base[1]
                    target_pos_z = t_base_tool[2] + combined_offset_base[2]
                    
                    horiz_dist = np.sqrt(target_pos_x**2 + target_pos_y**2)
                    if horiz_dist > MAX_REACH:
                        scale = MAX_REACH / horiz_dist
                        target_pos_x = target_pos_x * scale
                        target_pos_y = target_pos_y * scale
                    
                    print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] dist={coords.cam_z:.3f}m -> target=({target_pos_x:.3f}, {target_pos_y:.3f}, {target_pos_z:.3f})")
                    
                    position = [target_pos_x, target_pos_y, target_pos_z]
                    success = movement.cartesian_action_movement(
                        position, orientation, MOVE_SPEED, blocking=False
                    )
                    
                    if not success:
                        print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] Movement command failed, stopping")
                        tracking_active = False
                        tracking_phase = PHASE_IDLE
                        locked_orientation = None
                
                if tracking_phase == PHASE_FINAL_MOVE:
                    R_base_cam = R_base_tool @ R_tool_cam
                    
                    final_offset_cam = np.array([
                        FINAL_APPROACH_OFFSET,
                        FINAL_APPROACH_OFFSET,
                        0.0
                    ])
                    
                    final_offset_base = R_base_cam @ final_offset_cam
                    
                    target_pos_x = t_base_tool[0] + final_offset_base[0]
                    target_pos_y = t_base_tool[1] + final_offset_base[1]
                    target_pos_z = t_base_tool[2] + final_offset_base[2]
                    
                    orientation = [current_pose.theta_x, current_pose.theta_y, current_pose.theta_z]
                    position = [target_pos_x, target_pos_y, target_pos_z]
                    
                    print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] Moving 5cm in X and Y -> ({target_pos_x:.3f}, {target_pos_y:.3f}, {target_pos_z:.3f})")
                    
                    success = movement.cartesian_action_movement(
                        position, orientation, MOVE_SPEED, blocking=True
                    )
                    
                    if success:
                        print(f"[PHASE: COMPLETE] Sequence finished!")
                    else:
                        print(f"[PHASE: {PHASE_NAMES[tracking_phase]}] Final move failed")
                    
                    tracking_phase = PHASE_COMPLETE
                    tracking_active = False

        cap.release()
        cv2.destroyAllWindows()
        print("[OK] Done")


if __name__ == "__main__":
    main()
