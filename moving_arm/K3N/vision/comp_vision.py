import time
import numpy as np
import cv2
from logging import getLogger

from .common.utils import create_detector
from .robot.apriltag_viewer import get_tag_coordinates, TagCoordinates

from ..commander.location import Location
from ..commander.vectors import Vectors
from ..movement.action_frame import ActionFrame

logger = getLogger("ComputerVisionModule")

DEFAULT_TRANSLATION_SPEED = 0.1
Z_OFFSET_FROM_TAG = 0.15

class ComputerVisionModule:

    def __init__(self, base_client, camera_capture, movement):
        self.base = base_client
        self.cap = camera_capture
        self.movement = movement

        self.detector = create_detector()
        self.last_coords = None

    def detectTag(self) -> TagCoordinates:
        """
        Scans AprilTag and returns its coordinates relative to the robot base. 
        This is a single scan and may be noisy.
        """
        logger.info("Detecting tag...")

        ret, frame = self.cap.read()
        if not ret:
            logger.error("Failed to capture frame")
            return TagCoordinates()

        coords = get_tag_coordinates(self.base, self.detector, frame)
        self.last_coords = coords

        return coords

    def detectTagAveraged(self, samples: int = 5, delay: float = 0.1):
        """
        Do multiple scans and average the results to improve accuracy and reduce noise. 
        Returns None if no valid detections.
        """

        logger.info("Detecting tag with averaging")

        positions = []
        orientations = []

        for _ in range(samples):

            coords = self.detectTag()

            if coords.valid:

                positions.append([
                    coords.base_x,
                    coords.base_y,
                    coords.base_z
                ])

                orientations.append([
                    coords.base_roll,
                    coords.base_pitch,
                    coords.base_yaw
                ])

            time.sleep(delay)

        if len(positions) == 0:
            logger.warning("No valid detections")
            return None

        avg_pos = np.mean(positions, axis=0)
        avg_orient = np.mean(orientations, axis=0)

        location = Location(
            float(avg_pos[0]),
            float(avg_pos[1]),
            float(avg_pos[2])
        )

        vector = Vectors(
            float(avg_orient[0]),
            float(avg_orient[1]),
            float(avg_orient[2])
        )

        return location, vector

    def moveToTag(self, z_offset: float = Z_OFFSET_FROM_TAG):
        """
        Will be scrapped in favor or using auto_move based
        on location extracted instead of doing it in a function within
        vision module, but for now this is a simple wrapper that moves the robot to the tag location.
        It applies a Z offset to avoid collisions with the table.
        """

        ## TODO: delete once commander and state machine is done
        logger.info("Moving to tag")

        result = self.detectTagAveraged()

        if result is None:
            logger.error("No tag detected")
            return False

        location, vector = result

        # apply safety Z offset
        target_location = Location(
            location.x,
            location.y,
            location.z + z_offset
        )

        # end-effector orientation
        target_vector = Vectors(
            180.0,
            0.0,
            vector.zTheta
        )

        logger.info(
            "Target: (%.3f, %.3f, %.3f)",
            target_location.x,
            target_location.y,
            target_location.z
        )

        return self.movement.cartesian_action_movement(
            target_location,
            target_vector,
            DEFAULT_TRANSLATION_SPEED
        )

    def scanAprilTag(self, location: Location):
        """
        Scan the AprilTag at the given location. Used for when the system knows
        where the AprilTag should be. Returns the information encoded in the tag.

        ActionFrame: Location, Vectors of info extracted from the april tag
        """

        logger.info("Scanning AprilTag at %s", location)

        result = self.detectTagAveraged()

        if result is None:
            logger.warning("Tag not detected during scan")
            return None

        location, vector = result

        return ActionFrame(location, vector)
    
    def findAprilTag(self,
                 yaw_step: float = 15.0,
                 max_yaw_attempts: int = 6,
                 x_step: float = 0.1,
                 max_x_locations: int = 2,
                 z_step: float = 0.1,
                 z_levels: int = 4):
        """
        Search for an AprilTag by sweeping across X and Z positions
        and performing a yaw scan at each position.

        Returns:
            (Location, Vectors) if tag found
            None otherwise
        """

        logger.info("Starting AprilTag search routine")

        pose = self.base.GetMeasuredCartesianPose()

        base_location = Location(pose.x, pose.y, pose.z)
        base_vector = Vectors(pose.theta_x, pose.theta_y, pose.theta_z)

        # Create X offsets (center → left → right pattern)
        x_offsets = [0]
        for i in range(1, max_x_locations + 1):
            x_offsets.append(-i * x_step)
            x_offsets.append(i * x_step)

        # Create Z levels (centered → up → down)
        z_offsets = [0]
        for i in range(1, z_levels + 1):
            z_offsets.append(i * z_step)

        for z_offset in z_offsets:

            logger.info("Scanning at Z offset %.3f", z_offset)

            for x_offset in x_offsets:

                logger.info("Moving to search position X %.3f Z %.3f", x_offset, z_offset)

                search_location = Location(
                    base_location.x + x_offset,
                    base_location.y,
                    base_location.z + z_offset
                )

                # Move robot to this search position
                self.movement.cartesian_action_movement(
                    search_location,
                    base_vector,
                    DEFAULT_TRANSLATION_SPEED
                )

                time.sleep(1)

                # Perform yaw scan
                for i in range(max_yaw_attempts):

                    result = self.detectTagAveraged()

                    if result is not None:
                        logger.info("AprilTag detected!")
                        return result

                    new_vector = Vectors(
                        base_vector.xTheta,
                        base_vector.yTheta,
                        base_vector.zTheta + yaw_step * (i + 1)
                    )

                    self.movement.cartesian_action_movement(
                        search_location,
                        new_vector,
                        DEFAULT_TRANSLATION_SPEED
                    )

                    time.sleep(0.5)

                # Reset yaw before next search position
                logger.info("Resetting yaw")

                self.movement.cartesian_action_movement(
                    search_location,
                    base_vector,
                    DEFAULT_TRANSLATION_SPEED
                )

                time.sleep(0.5)

        logger.error("AprilTag not found after full search")
        return None
    
    def findAprilTagLookingDown(self,
                 yaw_step=10,
                 yaw_attempts=6,
                 x_step=0.05,
                 x_locations=2,
                 z_step=0.05,
                 z_levels=1):
        
        DOWN_CAMERA_JOINTS = [
            18.19185,
            33.89965,
            126.84781,
            263.79263,
            207.30588,
            63.46091,
            91.60575
        ]

        logger.info("Starting AprilTag search")

        # Move to known camera-down pose
        self.movement.angular_action_movement(DOWN_CAMERA_JOINTS)
        time.sleep(2)

        pose = self.base.GetMeasuredCartesianPose()

        base_location = Location(pose.x, pose.y, pose.z)
        base_vector = Vectors(pose.theta_x, pose.theta_y, pose.theta_z)

        # X offsets
        x_offsets = [0]
        for i in range(1, x_locations + 1):
            x_offsets.append(i * x_step)
            x_offsets.append(-i * x_step)

        # Z offsets
        z_offsets = [0]
        for i in range(1, z_levels + 1):
            z_offsets.append(i * z_step)
            z_offsets.append(-i * z_step)

        for z_offset in z_offsets:

            for x_offset in x_offsets:

                search_location = Location(
                    base_location.x + x_offset,
                    base_location.y,
                    base_location.z + z_offset
                )

                logger.info("Moving search position X %.3f Z %.3f", x_offset, z_offset)

                self.movement.cartesian_action_movement(
                    search_location,
                    base_vector,
                    DEFAULT_TRANSLATION_SPEED
                )

                time.sleep(1)

                # Get current joint angles
                joints = list(self.base.GetMeasuredJointAngles().joint_angles)

                base_joint = joints[0]

                # Sweep joint 0
                for i in range(yaw_attempts):

                    joints[0] = base_joint + yaw_step * (i + 1)

                    self.movement.joint_action_movement(joints)

                    time.sleep(0.6)

                    result = self.detectTagAveraged()

                    if result is not None:
                        logger.info("AprilTag found")
                        return result

                # Reset joint
                joints[0] = base_joint
                self.movement.joint_action_movement(joints)

        logger.error("AprilTag not found")
        return None

        
    def saveCurrentPose(self):
        """
        Save the current pose of the robot.
        Useful for recording tool caddy position.

        Returns:
            ActionFrame: Location and Vectors of the current pose, with action=0
        """

        logger.info("Saving pose")

        pose = self.base.GetMeasuredCartesianPose()

        location = Location(
            pose.x,
            pose.y,
            pose.z
        )

        vector = Vectors(
            pose.theta_x,
            pose.theta_y,
            pose.theta_z
        )

        return ActionFrame(location, vector)

    def calibrateCamera(self):
        """
        Placeholder for camera calibration routine. This should be run 
        at the start of the program to ensure accurate detections.
        """
        logger.info("Calibrating camera...")