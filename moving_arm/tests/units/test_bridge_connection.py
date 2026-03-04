import pytest
import logging
from unittest.mock import MagicMock, patch

# Import your module here
from K3N.comslogic.bridge_connection import (
    createNewBridge, EthernetBridgeConfig, UARTBridgeConfig
)

from kortex_api.autogen.messages import Common_pb2

# Marker for pytest
pytestmark = pytest.mark.units

# Fixtures
@pytest.fixture
def mock_router():
    return MagicMock()

# Tests
@patch("K3N.comslogic.bridge_connection.utilities.DeviceConnection")
def test_createNewBridge_ethernet(mock_device_connection, mock_router):
    mock_device_connection.createTcpConnection.return_value.__enter__.return_value = mock_router
    mock_eth_bridge = MagicMock()
    with patch("K3N.comslogic.bridge_connection.EthernetBridgeConfig", return_value=mock_eth_bridge) as mock_bridge_class:
        createNewBridge("ETHERNET", "eth0", "192.168.1.10")
        mock_device_connection.createTcpConnection.assert_called_with("eth0", "192.168.1.10")
        mock_bridge_class.assert_called_with(mock_router)
        mock_eth_bridge.EnableEthernetBridge.assert_called_once()

@patch("K3N.comslogic.bridge_connection.utilities.DeviceConnection")
def test_createNewBridge_uart(mock_device_connection, mock_router):
    mock_device_connection.createUARTConnection.return_value.__enter__.return_value = mock_router
    mock_uart_bridge = MagicMock()
    with patch("K3N.comslogic.bridge_connection.UARTBridgeConfig", return_value=mock_uart_bridge) as mock_bridge_class:
        createNewBridge("UART", "ttyUSB0", "192.168.1.11")
        mock_device_connection.createUARTConnection.assert_called_with("ttyUSB0", "192.168.1.11")
        mock_bridge_class.assert_called_with(mock_router)
        mock_uart_bridge.EnableUARTBridge.assert_called_once()

def test_EthernetBridgeConfig_get_device_id_found(mock_router):
    with patch("K3N.comslogic.bridge_connection.DeviceManagerClient") as mock_dm, \
         patch("K3N.comslogic.bridge_connection.InterconnectConfigClient"):
        devices = MagicMock()
        device = MagicMock()
        device.device_type = Common_pb2.INTERCONNECT
        device.device_identifier = 123
        devices.device_handle = [device]
        mock_dm.return_value.ReadAllDevices.return_value = devices
        eth_bridge = EthernetBridgeConfig(mock_router)
        device_id = eth_bridge.GetDeviceIdFromDevType(Common_pb2.INTERCONNECT, 0)
        assert device_id == 123

def test_EthernetBridgeConfig_enable_disable(mock_router):
    with patch("K3N.comslogic.bridge_connection.InterconnectConfigClient") as mock_ic, \
         patch("K3N.comslogic.bridge_connection.DeviceManagerClient") as mock_dm:
        devices = MagicMock()
        device = MagicMock()
        device.device_type = Common_pb2.INTERCONNECT
        device.device_identifier = 1
        devices.device_handle = [device]
        mock_dm.return_value.ReadAllDevices.return_value = devices
        eth_bridge = EthernetBridgeConfig(mock_router)
        eth_bridge.EnableEthernetBridge()
        mock_ic.return_value.SetEthernetConfiguration.assert_called()
        eth_bridge.DisableEthernetBridge()
        assert mock_ic.return_value.SetEthernetConfiguration.call_count == 2

def test_UARTBridgeConfig_configure_enable_disable(mock_router):
    mock_base_client = MagicMock()
    with patch("K3N.comslogic.bridge_connection.BaseClient", return_value=mock_base_client), \
         patch("K3N.comslogic.bridge_connection.DeviceManagerClient") as mock_dm, \
         patch("K3N.comslogic.bridge_connection.InterconnectConfigClient") as mock_ic:
        devices = MagicMock()
        device = MagicMock()
        device.device_type = Common_pb2.INTERCONNECT
        device.device_identifier = 5
        devices.device_handle = [device]
        mock_dm.return_value.ReadAllDevices.return_value = devices

        uart_bridge = UARTBridgeConfig(mock_router, "192.168.1.11")
        uart_bridge.Configure(port_id=1, enabled=True, speed=9600, word_length=8, stop_bits=1, parity=0)
        mock_ic.return_value.SetUARTConfiguration.assert_called()

        uart_bridge.EnableBridge(bridge_type=1, target=2, output=3)
        mock_base_client.EnableBridge.assert_called()

        uart_bridge.DisableBridge(bridge_id=1)
        mock_base_client.DisableBridge.assert_called()

