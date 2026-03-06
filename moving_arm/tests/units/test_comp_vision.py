import pytest
import numpy as np

from unittest.mock import MagicMock

from K3N.vision.comp_vision import ComputerVisionModule
from K3N.commander.location import Location
from K3N.commander.vectors import Vectors
from K3N.movement.action_frame import ActionFrame

# Marker for pytest
pytestmark = pytest.mark.units

class FakeTag:
    def __init__(self, valid=True):
        self.valid = valid
        self.base_x = 0.5
        self.base_y = -0.2
        self.base_z = 0.1

        self.base_roll = 1.0
        self.base_pitch = 2.0
        self.base_yaw = 90.0


@pytest.fixture
def mock_camera():
    cam = MagicMock()
    cam.read.return_value = (True, np.zeros((480, 640, 3)))
    return cam


@pytest.fixture
def mock_base():
    base = MagicMock()

    pose = MagicMock()
    pose.x = 0.3
    pose.y = 0.2
    pose.z = 0.1
    pose.theta_x = 180
    pose.theta_y = 0
    pose.theta_z = 90

    base.GetMeasuredCartesianPose.return_value = pose
    return base


@pytest.fixture
def mock_movement():
    movement = MagicMock()
    movement.cartesian_action_movement.return_value = True
    return movement


@pytest.fixture
def cv_module(mock_base, mock_camera, mock_movement):
    return ComputerVisionModule(mock_base, mock_camera, mock_movement)

def test_detect_tag(monkeypatch, cv_module):

    def fake_detection(base, detector, frame):
        return FakeTag()

    monkeypatch.setattr(
        "K3N.vision.robot.apriltag_viewer.get_tag_coordinates",
        fake_detection
    )

    result = cv_module.detectTag()

    assert result.valid
    assert result.base_x == 0.5

def test_detect_tag_averaged(monkeypatch, cv_module):

    def fake_detection(base, detector, frame):
        return FakeTag()

    monkeypatch.setattr(
        "K3N.vision.robot.apriltag_viewer.get_tag_coordinates",
        fake_detection
    )

    location, vector = cv_module.detectTagAveraged(samples=3, delay=0)

    assert isinstance(location, Location)
    assert isinstance(vector, Vectors)

    assert location.x == pytest.approx(0.5)
    assert location.y == pytest.approx(-0.2)
    assert location.z == pytest.approx(0.1)

    assert vector.zTheta == pytest.approx(90.0)

def test_move_to_tag(monkeypatch, cv_module, mock_movement):

    def fake_detection(base, detector, frame):
        return FakeTag()

    monkeypatch.setattr(
        "K3N.vision.robot.apriltag_viewer.get_tag_coordinates",
        fake_detection
    )

    result = cv_module.moveToTag()

    assert result is True
    mock_movement.cartesian_action_movement.assert_called_once()

def test_scan_apriltag(monkeypatch, cv_module):

    def fake_detection(base, detector, frame):
        return FakeTag()

    monkeypatch.setattr(
        "K3N.vision.robot.apriltag_viewer.get_tag_coordinates",
        fake_detection
    )

    location = Location(0, 0, 0)

    result = cv_module.scanAprilTag(location)

    assert isinstance(result, ActionFrame)
    assert isinstance(result.location, Location)
    assert isinstance(result.vector, Vectors)

def test_save_current_pose(cv_module):

    result = cv_module.saveCurrentPose()

    assert isinstance(result, ActionFrame)
    assert result.location.x == pytest.approx(0.3)
    assert result.vector.zTheta == pytest.approx(90)