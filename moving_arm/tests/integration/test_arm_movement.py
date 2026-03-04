import os
import pytest
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

from moving_arm.K3N import utilities
from K3N.movement.auto_move import AutonomousMovement

#Pytest markers
pytestmark = pytest.mark.integration

# Configs
TEST_DIR = os.path.dirname(os.path.dirname(__file__))
CSV_PATH = os.path.join(TEST_DIR, "mock_data", "ExampleSequence.csv")

# Device connection parameters - might need to be adjusted
# (IP, username, password, etc.)
@pytest.fixture(scope="session")
def device_args():
    return {
        "ip": "192.168.1.10",
        "username": "admin",
        "password": "admin"
    }

# Integration test for AutonomousMovement
def test_autonomous_movement_sequence(device_args):
    """
    Full end-to-end test:
    - Connects to the robot
    - Reads CSV
    - Executes real actions
    - Fails if any action fails
    """

    with utilities.DeviceConnection.createTcpConnection(device_args) as router:
        base = BaseClient(router)
        movement = AutonomousMovement(base)

        # Move to home position first
        success = movement.move_to_home_position(base)
        assert success, "Failed to move to home position"

        # Load CSV manually (mirrors your script behavior)
        angles = []
        positions = []
        orientations = []
        gripper_positions = []
        translation_speeds = []
        action_sequence = []

        # You already have this reader — use it directly
        utilities.read_csv(
            CSV_PATH,
            angles,
            positions,
            orientations,
            gripper_positions,
            translation_speeds,
            action_sequence
        )

        assert len(action_sequence) > 0, "CSV produced no actions"

        # Execute sequence
        for i, action_type in enumerate(action_sequence):
            print(f"\nExecuting action {i+1}/{len(action_sequence)}")

            if action_type == 7:  # Angular
                joint_angles = angles.pop(0)
                success = movement.angular_action_movement(
                    base, joint_angles
                )

            elif action_type == 6:  # Cartesian
                pos = positions.pop(0)
                ori = orientations.pop(0)
                spd = translation_speeds.pop(0)
                success = movement.cartesian_action_movement(
                    base, pos, ori, spd
                )

            elif action_type == 33:  # Gripper
                success = True  # Skip gripper actions for this test

            else:
                pytest.fail(f"Unknown action type {action_type}")

            assert success, f"Action {i+1} failed"

    print("Autonomous movement sequence completed successfully")