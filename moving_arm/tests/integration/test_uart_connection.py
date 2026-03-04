import os
import sys
import time
import socket
import select
import pytest

from kortex_api.autogen.messages import (
    Base_pb2,
    Common_pb2,
    InterconnectConfig_pb2,
)

from K3N import utilities
from K3N.comslogic.bridge_connection import UARTBridgeConfig

#pytest markers
pytestmark = pytest.mark.integration

# Device connection parameters - might need to be adjusted
# (IP, username, password, etc.)
@pytest.fixture(scope="session")
def device_args():
    return {
        "ip": "192.168.1.10",
        "username": "admin",
        "password": "admin"
    }

def test_uart_bridge_send_and_receive(device_args):
    """
    End-to-end UART bridge integration test using UARTBridgeConfig.

    This test:
    - Configures UART on interconnect
    - Enables TCP↔UART bridge
    - Opens TCP socket
    - Sends data
    - Reads echoed data
    - Cleans up bridge and UART config
    """

    with utilities.DeviceConnection.createTcpConnection(device_args) as router:

        uart = UARTBridgeConfig(router, device_args.ip)

        # Enable UART on interconnect

        uart.Configure(
            InterconnectConfig_pb2.UART_PORT_EXPANSION,
            enabled=True,
            speed=Common_pb2.UART_SPEED_115200,
            word_length=Common_pb2.UART_WORD_LENGTH_8,
            stop_bits=Common_pb2.UART_STOP_BITS_1,
            parity=Common_pb2.UART_PARITY_NONE,
        )

        time.sleep(1)

        # Enable Bridge

        bridge_result = uart.EnableBridge(Base_pb2.BRIDGE_TYPE_UART)
        assert bridge_result.status == Base_pb2.BRIDGE_STATUS_OK, \
            "Failed to enable UART bridge"

        bridge_id = bridge_result.bridge_id

        ## Retrieve bridge configuration
        bridge_config = uart.base.GetBridgeConfig(bridge_id)
        base_port = bridge_config.port_config.out_port
        interconnect_port = bridge_config.port_config.target_port

        print(
            f"UART bridge #{bridge_id.bridge_id} created between "
            f"Interconnect device {uart.interconnect_device_id} "
            f"(port {interconnect_port}) and base port {base_port}"
        )

        ## Open TCP socket to base port

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((device_args.ip, base_port))
        client_socket.setblocking(False)

        ## Send Test Data
        ## Will want to split this into a new test that tests sending and receiving separately
        ## For now, we just want to ensure data can be sent and received

        test_message = b"UART integration test payload\n"
        client_socket.send(test_message)

        ## Read in Response
        received = b""
        timeout = 10
        start = time.time()

        while time.time() - start < timeout:
            ready, _, _ = select.select([client_socket], [], [], 1)
            if ready:
                data = client_socket.recv(1)
                if data:
                    received += data

        client_socket.close()

        print("Received UART data:")
        print(received.decode("utf-8", errors="ignore"))

        assert len(received) > 0, "No data received from UART"

        # Cleanup
        uart.DisableBridge(bridge_id)

        uart.Configure(
            InterconnectConfig_pb2.UART_PORT_EXPANSION,
            enabled=False,
            speed=Common_pb2.UART_SPEED_115200,
            word_length=Common_pb2.UART_WORD_LENGTH_8,
            stop_bits=Common_pb2.UART_STOP_BITS_1,
            parity=Common_pb2.UART_PARITY_NONE,
        )

        print("UART bridge integration test completed successfully")