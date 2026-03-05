import os
import pytest
import time
from typing import Optional

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

from K3N import utilities
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
        "username": "SSE_Student",
        "password": "KinovaG3"
    }

def _best_effort_shutdown(router, base: Optional[BaseClient]):
    """
    Never raise from cleanup. This should *only* try to leave the device/connection
    in a sane state and close the comms channel.
    """
    # Try to stop any motion/action (depends on what your API exposes)
    if base is not None:
        try:
            # Most Kortex setups allow cancel/stop semantics; adjust to what you use.
            base.Stop()
        except Exception:
            pass

        try:
            base.ClearFaults()
        except Exception:
            pass

    # Close router/transport if your connection object supports it
    if router is not None:
        try:
            router.close()
        except Exception:
            pass

        # Some implementations have transport underneath
        try:
            if hasattr(router, "transport") and router.transport is not None:
                router.transport.close()
        except Exception:
            pass

# Integration test for AutonomousMovement
def test_autonomous_movement_sequence(device_args):
    """
    Full end-to-end test:
    - Connects to the robot
    - Reads CSV
    - Executes real actions
    - Fails if any action fails
    """
    router = None
    base = None
    movement = None
    conn = None
    try:
        # NOTE: if createTcpConnection already returns a context manager, we can still use it,
        # but we ALSO keep refs for finally-cleanup.
        conn = utilities.DeviceConnection.createTcpConnection(
            device_args["ip"],
            device_args["username"],
            device_args["password"]
        )
        router = conn.__enter__()  # manually enter so finally always runs cleanup
        base = BaseClient(router)
        movement = AutonomousMovement(base)

        # Move to home position first
        success = movement.move_to_home_position()
        assert success, "Failed to move to home position"

        # Load CSV
        angles = []
        positions = []
        orientations = []
        gripper_positions = []
        translation_speeds = []
        action_sequence = []

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
                success = movement.angular_action_movement(joint_angles)

            elif action_type == 6:  # Cartesian
                pos = positions.pop(0)
                ori = orientations.pop(0)
                spd = translation_speeds.pop(0)
                success = movement.cartesian_action_movement(pos, ori, spd)

            elif action_type == 33:  # Gripper
                success = True  # Skip gripper actions for this test

            else:
                pytest.fail(f"Unknown action type {action_type}")

            assert success, f"Action {i+1} failed"

        print("Autonomous movement sequence completed successfully")

    finally:
        # Always run cleanup, even on assert/pytest.fail/exception
        _best_effort_shutdown(router, base)

        # If we manually entered the connection, also manually exit it
        # (this lets the DeviceConnection do its own cleanup too).
        try:
            if router is not None:
                # conn exists only if we got past createTcpConnection()
                conn.__exit__(None, None, None)
        except Exception:
            pass