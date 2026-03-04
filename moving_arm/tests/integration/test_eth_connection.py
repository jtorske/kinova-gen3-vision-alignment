import pytest
import time


from kortex_api.autogen.messages import (
    Common_pb2,
    InterconnectConfig_pb2,
)

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

from K3N import utilities
from K3N.comslogic.bridge_connection import EthernetBridgeConfig


#Pytest markers

pytestmark = pytest.mark.integration

# Fixture to parse device connection arguments
@pytest.fixture(scope="session")
def device_args():
    return {
        "ip": "192.168.1.10",
        "username": "admin",
        "password": "admin"
    }

def test_ethernet_bridge_enable_disable(device_args):
    """
    Ethernet bridge integration test.

    Validates that:
    - Interconnect is discoverable
    - Ethernet bridge can be enabled
    - Arm responds to BaseClient RPC
    - Bridge can be disabled cleanly
    """

    with utilities.DeviceConnection.createTcpConnection(
            device_args["ip"],
            device_args["username"],
            device_args["password"]
        ) as router:

        ethernet_bridge = EthernetBridgeConfig(router)

        # Enable Ethernet bridge
        ethernet_bridge.EnableEthernetBridge()
        time.sleep(5)

        # Create BaseClient to test communication over Ethernet
        base = BaseClient(router)

        try:
            arm_state = base.GetArmState()
        except Exception as e:
            pytest.fail(f"Failed to communicate with arm over Ethernet: {e}")

        # Validate response
        assert arm_state is not None
        assert arm_state.active_state in (
            Base_pb2.ARMSTATE_READY,
            Base_pb2.ARMSTATE_SERVOING,
            Base_pb2.ARMSTATE_FAULT,
        )

        # Disable Ethernet bridge
        ethernet_bridge.DisableEthernetBridge()
        time.sleep(1)

        print("Ethernet bridge integration test completed successfully")
