import cv2
import time
import pytest

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from K3N import utilities
from K3N.vision.comp_vision import ComputerVisionModule
from K3N.movement.auto_move import AutonomousMovement

#Pytest markers
pytestmark = pytest.mark.integration

# Device connection parameters - might need to be adjusted
# (IP, username, password, etc.)
@pytest.fixture(scope="session")
def device_args():
    return {
        "ip": "192.168.1.10",
        "username": "SSE_Student",
        "password": "KinovaG3"
    }

def test_find_april_tag_integration():
    """
    Integration test:
    - connects to robot
    - opens camera
    - runs findAprilTag()
    - verifies a tag can be detected
    """

    conn = utilities.DeviceConnection.createTcpConnection(
            device_args["ip"],
            device_args["username"],
            device_args["password"]
        )
    router = conn.__enter__()  # manually enter so finally always runs cleanup

    base = BaseClient(router)

    movement = AutonomousMovement(base)

    # open camera stream
    cap = cv2.VideoCapture(0)

    assert cap.isOpened(), "Camera failed to open"

    vision = ComputerVisionModule(base, cap, movement)

    start_time = time.time()
    timeout = 120   # seconds

    result = None

    while result is None and time.time() - start_time < timeout:
        result = vision.findAprilTag()

    cap.release()

    assert result is not None, "AprilTag was not detected within timeout"

    location, vector = result

    print("Detected location:", location.x, location.y, location.z)
    print("Detected orientation:", vector.xTheta, vector.yTheta, vector.zTheta)

    assert isinstance(location.x, float)
    assert isinstance(vector.zTheta, float)