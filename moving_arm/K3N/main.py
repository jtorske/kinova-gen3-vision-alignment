import cv2
import time

from config import Config
from commander.commander import CommanderGUI
from comslogic.coms_logic import ComsLogic2K3N
from moving_arm.K3N.comslogic.bridge_connection import UARTBridgeConfig, UARTBridge
from moving_arm.tests.integration.test_scan_and_move import device_args
from vision.comp_vision import ComputerVisionModule

from vision.vision_arm_controller import (
    useVision
)
from utilities import utilities
from movement.auto_move import AutonomousMovement
from logging import Logging

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from enum import Enum

class States(Enum):
    HOME = 0
    ENGAGE = 1
    CONFIRM_LATCH = 2
    IDLE_LATCH = 3
    DISENGAGE = 4
    INOPERABLE = 5
class ToolLatchController:

    def __init__(self, uart, movement):

        self.uart = uart
        self.movement = movement
        self.state = States.HOME
        
    def home_state(self):

        print("STATE: HOME")

        # request tool pickup
        self.uart.send("ENGAGE")

        self.state = States.ENGAGE

    def engage_state(self):

        print("STATE: ENGAGE")

        # move robot into pickup pose
        # (placeholder – integrate movement module)
        print("Moving robot to pickup position")
        time.sleep(2)

        # notify firmware arm is positioned
        self.uart.send("ENG_PLACE")

        self.state = States.CONFIRM_LATCH

    def confirm_latch_state(self):

        print("STATE: CONFIRM_LATCH")

        while True:

            msg = self.uart.read()

            if msg:

                print("UART:", msg)

                if "Latch Confirmed" in msg:
                    self.state = States.IDLE_LATCH
                    return

                if "Latch Failed" in msg:
                    self.state = States.INOPERABLE
                    return

            time.sleep(0.1)

    def idle_state(self):

        print("STATE: IDLE_LATCH")

        # example: wait then drop tool
        time.sleep(5)

        self.uart.send("DISENGAGE")

        self.state = States.DISENGAGE

    def disengage_state(self):

        print("STATE: DISENGAGE")

        # move robot to drop pose
        print("Moving robot to drop position")

        time.sleep(2)

        self.uart.send("DIS_PLACE")

        while True:

            msg = self.uart.read()

            if msg:

                print("UART:", msg)

                if "Unlatch Confirmed" in msg:
                    self.state = States.HOME
                    return

                if "unlatch Failed" in msg:
                    self.state = States.INOPERABLE
                    return

            time.sleep(0.1)

    def inoperable_state(self):

        print("STATE: INOPERABLE")

        while True:

            msg = self.uart.read()

            if msg:
                print("UART:", msg)

            time.sleep(0.5)

    def run(self):

        while True:

            if self.state == States.HOME:
                self.home_state()

            elif self.state == States.ENGAGE:
                self.engage_state()

            elif self.state == States.CONFIRM_LATCH:
                self.confirm_latch_state()

            elif self.state == States.IDLE_LATCH:
                self.idle_state()

            elif self.state == States.DISENGAGE:
                self.disengage_state()

            elif self.state == States.INOPERABLE:
                self.inoperable_state()

def main():
    config = Config()
    config.load()
    commander = CommanderGUI()
    Logging.logInfo("Commander initialized")

    coms = ComsLogic2K3N()
    Logging.logInfo("Communication initialized")

    conn = utilities.DeviceConnection.createTcpConnection(
            config["arm_ip"],
            config["arm_username"],
            config["arm_password"]
        )
    
    router = conn.__enter__()  # manually enter so finally always runs cleanup
    
    # Setting up arm movement and connection
    base = BaseClient(router)
    movement = AutonomousMovement(base)

    # open camera stream
    iterations = 0
    while cap is None or not cap.isOpened() and iterations < 10:
        try:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("Camera failed to open, retrying...")
                time.sleep(2)
        except Exception as e:
            print(f"Error opening camera: {e}")
            time.sleep(2)
        iterations += 1
    if cap is None or not cap.isOpened():
        print("Failed to open camera after multiple attempts, exiting.")
        return
    vision = ComputerVisionModule(base, cap, movement)
    
    # Find AprilTag and move to pickup pose
    start_time = time.time()
    timeout = 120   # seconds
    found_tag = None
    while found_tag is None and time.time() - start_time < timeout:
        found_tag = vision.findAprilTag()
    if found_tag is None:
        print("AprilTag was not detected within timeout, exiting.")
        return
    
    lined_up = useVision(vision)
    if not lined_up:
        print("Failed to line up with AprilTag, exiting.")
        return

    uart = UARTBridge(router, config["ip"])

    controller = ToolLatchController(uart, movement)

    controller.run()


if __name__ == "__main__":
    main()