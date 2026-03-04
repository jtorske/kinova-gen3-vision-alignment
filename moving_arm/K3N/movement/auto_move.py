from ..commander.location import Location
from ..commander.vectors import Vectors
from logging import getLogger
from .action_frame import ActionFrame
from ..utilities import read_csv

import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

ACTION_TIMEOUT_DURATION = 20
ACTION_CARTESIAN = 6
ACTION_ANGULAR = 7
ACTION_GRIPPER = 33

logger = getLogger("Movement")
class AutonomousMovement:

    def __init__(self, base: BaseClient):
        self.base = base
        self.storedLocations = {}  # Dictionary[Location]

    def runSequence(self, csvPath: str) -> str:
        logger.info(f"Running sequence from: {csvPath}")
        angles = []
        positions = []
        orientations = []
        gripper_positions = []
        translation_speeds = []
        action_sequence = []

        try:
            read_csv(
                csvPath,
                angles,
                positions,
                orientations,
                gripper_positions,
                translation_speeds,
                action_sequence
            )
        except Exception as e:
            logger.error(f"Failed to read CSV: {e}")
            return "CSV_ERROR"
        
        if not action_sequence:
            logger.warning("No actions found in CSV")
            return "NO_ACTIONS"
        
        logger.info(f"Executing {len(action_sequence)} actions from CSV")

        results = []
        for index, action_type in enumerate(action_sequence, start=1):
            logger.info(f"Executing action {index}/{len(action_sequence)}")
            # Build ActionFrame based on action type
            # Angular Movement
            if action_type == ACTION_ANGULAR:
                joint_angles = angles.pop(0)

                frame = ActionFrame(
                    location=Location(*joint_angles),
                    vector=None,
                    action=action_type,
                    translation_speed=None
                )
                result = self.moveArm(frame)
            # Cartesian Movement
            elif action_type == ACTION_CARTESIAN:
                position = positions.pop(0)
                orientation = orientations.pop(0)

                frame = ActionFrame(
                    location=Location(*position),
                    vector=Vectors(*orientation),
                    action=action_type,
                    translation_speed=translation_speeds.pop(0)
                )

                result = self.moveArm(frame)
            # Gripper Activation - not implemented
            elif action_type == ACTION_GRIPPER:
                logger.info("Gripper action received - skipping in this implementation")
                result = "OK"
            else:
                logger.warning(f"Unknown action type {action_type}")
                result = "UNKNOWN_ACTION"
            # Add result to list
            results.append(result)
            
        logger.info("Sequence completed successfully")
        return results
    
    def moveArm(self, frame: ActionFrame) -> str:
        logger.info("Moving arm using ActionFrame")
        # Will use functions defined below to perform the movement
        # e.g., self.cartesian_action_movement(...) or self.angular_action_movement(...)
        # depending on frame contents, ie: action type
        action_type = frame.action

        if action_type == ACTION_ANGULAR:
            logger.info("Executing angular movement")

            # Expecting frame.location to carry joint angles as x, y, z, ...
            # If you later expand joints, this is the only line that changes
            joint_angles = [
                frame.location.x,
                frame.location.y,
                frame.location.z
            ]

            success = AutonomousMovement.angular_action_movement(
                self.base,
                joint_angles
            )

            if not success:
                logger.error("Angular movement failed")
                return "FAILED"
            
            logger.info("Angular movement completed successfully, moved to" \
            " joint angles: " + str(joint_angles))

        elif action_type == ACTION_CARTESIAN:
            logger.info("Executing cartesian movement")
            position = [
                frame.location.x,
                frame.location.y,
                frame.location.z
            ]

            orientation = [
                frame.vector.xTheta,
                frame.vector.yTheta,
                frame.vector.zTheta
            ]

            # Use a conservative default translation speed
            translation_speed = frame.translation_speed

            success = AutonomousMovement.cartesian_action_movement(
                self.base,
                position,
                orientation,
                translation_speed
            )

            if not success:
                logger.error("Cartesian movement failed")
                return "FAILED"
            
            logger.info("Cartesian movement completed successfully, " \
            "moved to position: " + str(position) + " and orientation: " + str(orientation))


        elif action_type == ACTION_GRIPPER:  # Gripper
            logger.info("Gripper action received - skipping in this implementation")

        else:
            logger.warning(f"Unknown action type {action_type}")
        
        return "OK"
    
    # Create closure to set an event after an END or an ABORT
    def check_for_end_or_abort(self, e):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """
        def check(notification, e = e):
            print("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check

    def angular_action_movement(self, angles):
    
        print("Starting angular action movement ...")
        action = Base_pb2.Action()
        action.name = "Example angular action movement"
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()

        # Place arm straight up
        for joint_id in range(actuator_count.count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = angles[joint_id]

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )
        
        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(ACTION_TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)
        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished
    
    def cartesian_action_movement(self, position, orientation, velocity):
    
        action = Base_pb2.Action()
        action.name = "Example Cartesian action movement"
        action.application_data = ""

        #feedback = base_cyclic.RefreshFeedback()

        cartesian_pose = action.reach_pose.target_pose
        #speed
        speed=action.reach_pose.constraint.speed
        speed.translation=velocity
        #
        cartesian_pose.x = position[0]         # (meters)
        cartesian_pose.y = position[1]    # (meters)
        cartesian_pose.z = position[2]    # (meters)
        cartesian_pose.theta_x = orientation[0] # (degrees)
        cartesian_pose.theta_y = orientation[1] # (degrees)
        cartesian_pose.theta_z = orientation[2] # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(ACTION_TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)
        if finished:
            print("Cartesian movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def move_to_home_position(self):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        # Move arm to ready position
        print("Moving the arm to home position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle

        if action_handle == None:
            print("Can't reach home position. Exiting")
            return False

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)
        finished = e.wait(ACTION_TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Home position reached")
        else:
            print("Timeout on action notification wait")
        return finished
    
    # Execute Command Placeholder that could be implemented for something specific to end effector
    def executeCommand(self) -> str:
        logger.info("Executing command...")
        return "OK"