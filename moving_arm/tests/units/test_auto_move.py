import pytest
import logging
from unittest.mock import MagicMock, patch

from K3N.movement.auto_move import AutonomousMovement, ACTION_CARTESIAN, ACTION_ANGULAR, ACTION_GRIPPER
from K3N.commander.location import Location
from K3N.commander.vectors import Vectors
from K3N.movement.action_frame import ActionFrame

# Marker for pytest
pytestmark = pytest.mark.units

# Fixtures
@pytest.fixture
def mock_base():
    return MagicMock()

@pytest.fixture
def arm(mock_base):
    return AutonomousMovement(base=mock_base)

@patch("K3N.movement.auto_move.read_csv")
def test_run_sequence_csv_error(mock_read_csv, arm, caplog):
    # Simulate CSV read raising exception
    with caplog.at_level(logging.INFO, logger="Movement"):
        mock_read_csv.side_effect = Exception("CSV read failed")
        result = arm.runSequence("fake_path.csv")
        assert caplog.records[-1].levelname == "ERROR"
        assert "Failed to read CSV:" in caplog.text
        assert result == "CSV_ERROR"

@patch("K3N.movement.auto_move.read_csv")
def test_run_sequence_no_actions(mock_read_csv, arm, caplog):
    # Simulate empty CSV (no actions)
    with caplog.at_level(logging.INFO, logger="Movement"):
        mock_read_csv.side_effect = lambda *args, **kwargs: None
        result = arm.runSequence("fake_path.csv")
        assert "Running sequence from: fake_path.csv" in caplog.text
        assert caplog.records[-1].levelname == "WARNING"
        assert "No actions found in CSV" in caplog.text
        assert result == "NO_ACTIONS"

@patch.object(AutonomousMovement, 'moveArm')
@patch("K3N.movement.auto_move.read_csv")
def test_run_sequence_actions(mock_read_csv, mock_moveArm, arm, caplog):
    # Prepare dummy CSV data
    with caplog.at_level(logging.INFO, logger="Movement"):
        def fake_read_csv(path, angles, positions, orientations, gripper_positions, translation_speeds, action_sequence):
            angles.append([0.0, 0.1, 0.2])
            positions.append([0.5, 0.5, 0.5])
            orientations.append([0.0, 90.0, 0.0])
            translation_speeds.append(0.1)
            action_sequence.extend([ACTION_ANGULAR, ACTION_CARTESIAN, ACTION_GRIPPER])
        mock_read_csv.side_effect = fake_read_csv
        # Simulate moveArm always succeeding
        mock_moveArm.side_effect = lambda frame: "OK"

        results = arm.runSequence("dummy.csv")
        assert "Running sequence from: dummy.csv" in caplog.text
        assert "Executing 3 actions from CSV" in caplog.text
        assert results == ["OK", "OK", "OK"]
        assert mock_moveArm.call_count == 2  # Gripper does not call moveArm

def test_moveArm_angular(arm, caplog):
    # Patch the angular movement to always succeed
    with caplog.at_level(logging.INFO, logger="Movement"):
        with patch.object(AutonomousMovement, 'angular_action_movement', return_value=True):
            frame = ActionFrame(location=Location(1,2,3), vector=None, action=ACTION_ANGULAR, translation_speed=None)
            result = arm.moveArm(frame)
            assert "Executing angular movement" in caplog.text
            assert "Angular movement completed successfully, moved to" \
            " joint angles: " + str([1, 2, 3]) in caplog.text
            assert result == "OK"

def test_moveArm_cartesian(arm, caplog):
    # Patch cartesian movement to always succeed
    with caplog.at_level(logging.INFO, logger="Movement"):
        with patch.object(AutonomousMovement, 'cartesian_action_movement', return_value=True):
            frame = ActionFrame(location=Location(1,2,3), vector=Vectors(10,20,30), action=ACTION_CARTESIAN, translation_speed=0.5)
            result = arm.moveArm(frame)
            assert "Executing cartesian movement" in caplog.text
            assert "Cartesian movement completed successfully, " \
            "moved to position: " + str([1, 2, 3]) + " and orientation: " + str([10, 20, 30]) in caplog.text
            assert result == "OK"

def test_moveArm_unknown_action(arm, caplog):
    with caplog.at_level(logging.INFO, logger="Movement"):
        frame = ActionFrame(location=None, vector=None, action=999, translation_speed=None)
        result = arm.moveArm(frame)
        assert caplog.records[-1].levelname == "WARNING"
        assert "Unknown action type 999" in caplog.text
        assert result == "OK"