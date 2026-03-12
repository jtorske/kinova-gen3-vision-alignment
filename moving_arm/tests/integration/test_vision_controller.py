import pytest
import cv2
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from K3N import utilities
from K3N.vision.comp_vision import ComputerVisionModule
from K3N.movement.auto_move import AutonomousMovement
from K3N.vision.vision_arm_controller import (
    parse_args, make_conn_args, create_components, print_startup, open_camera,
    grab_latest_frame, detect_tag, process_detections, update_edge_status,  
    draw_overlay, show_frame_and_get_key, handle_tag_loss, handle_tracking_logic, initialize_state
)

#Pytest markers
pytestmark = pytest.mark.single

@pytest.fixture
def args():
    class Args:
        ip = "192.168.1.10"
        username = "admin"
        password = "admin"
    return Args()

@pytest.fixture
def conn_args(args):
    return make_conn_args(args)

def test_vision_arm_controller_integration(args, conn_args):
    """
    Integration test that mimics vision_arm_conntroller's main() but runs for a limited number
    of iterations and guarantees cleanup using a finally block.
    """

    cap = None
    router_ctx = None

    try:
        router_ctx = utilities.DeviceConnection.createTcpConnection(
            conn_args.ip, conn_args.username, conn_args.password
        )
        router = router_ctx.__enter__()

        base_client = BaseClient(router)
        movement, detector = create_components(base_client)

        cap = open_camera(args.ip)
        if not cap.isOpened():
            pytest.fail("Camera failed to open")

        print_startup()
        vision = ComputerVisionModule(base_client, cap, movement)

        start_time = time.time()
        timeout = 120   # seconds
        result = None
        while result is None and time.time() - start_time < timeout:
            result = vision.findAprilTag()

        assert result is not None, "AprilTag was not detected within timeout"

        state = initialize_state()
        max_iterations = 300  # prevent infinite loop during testing
        iteration = 0

        while iteration < max_iterations:
            iteration += 1

            ret, frame = grab_latest_frame(vision.cap)
            if not ret:
                continue

            results = detect_tag(vision.detector, frame)

            coords, R_base_tool, t_base_tool = process_detections(
                frame, results, vision.base, state
            )

            update_edge_status(frame, coords, results, state)

            draw_overlay(frame, coords, t_base_tool, vision.base, state)

            key = show_frame_and_get_key(frame)
            if key == ord("q"):
                break

            handle_tag_loss(vision.base, vision.movement, coords, state)

            handle_tracking_logic(vision.base, vision.movement, coords, state)

        # basic sanity checks for integration test
        assert vision.cap is not None
        assert vision.detector is not None
        assert vision.movement is not None
        assert state is not None

    finally:
        if cap is not None:
            cap.release()

        cv2.destroyAllWindows()

        if router_ctx is not None:
            router_ctx.__exit__(None, None, None)

        print("[TEST CLEANUP] Resources released")