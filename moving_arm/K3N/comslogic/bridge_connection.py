
import sys
import os
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.client_stubs.InterconnectConfigClientRpc import InterconnectConfigClient

from kortex_api.autogen.messages import Session_pb2, Base_pb2, Common_pb2, InterconnectConfig_pb2, DeviceManager_pb2

import utilities

def createNewBridge(type, interface_name, device_ip):
    if type == "ETHERNET":
        with utilities.DeviceConnection.createTcpConnection(interface_name, device_ip) as router:
            ethernet_bridge = EthernetBridgeConfig(router)
            ethernet_bridge.EnableEthernetBridge()
    if type == "UART":
        with utilities.DeviceConnection.createUARTConnection(interface_name, device_ip) as router:
            uart_bridge = UARTBridgeConfig(router)
            uart_bridge.EnableUARTBridge()

class EthernetBridgeConfig:
    def __init__(self, router):
        # Create required services        
        self.interconnect_config = InterconnectConfigClient(router)
        self.device_manager = DeviceManagerClient(router)
        
        self.interconnect_device_id = self.GetDeviceIdFromDevType(Common_pb2.INTERCONNECT, 0)
        if (self.interconnect_device_id is None):
            print ("Could not find the Interconnect in the device list, exiting...")
            sys.exit(0)

    def GetDeviceIdFromDevType(self, device_type, device_index = 0):
        devices = self.device_manager.ReadAllDevices()

        current_index = 0
        for device in devices.device_handle:
            if device.device_type == device_type:
                if current_index == device_index:
                    print ("Found the Interconnect on device identifier {}".format(device.device_identifier))
                    return device.device_identifier
                current_index += 1
        return None

    def EnableEthernetBridge(self):

        # Configure the Interconnect to enable the bridge
        ethernet_configuration = InterconnectConfig_pb2.EthernetConfiguration()
        ethernet_configuration.device = InterconnectConfig_pb2.ETHERNET_DEVICE_EXPANSION
        ethernet_configuration.enabled = True
        ethernet_configuration.speed = InterconnectConfig_pb2.ETHERNET_SPEED_100M
        ethernet_configuration.duplex = InterconnectConfig_pb2.ETHERNET_DUPLEX_FULL
        try:
            self.interconnect_config.SetEthernetConfiguration(ethernet_configuration, self.interconnect_device_id)
        except Exception as e:
            print ("An unexpected error occured : {}".format(e))
    
    def DisableEthernetBridge(self):
        # Disable the bridge
        ethernet_configuration = InterconnectConfig_pb2.EthernetConfiguration()
        ethernet_configuration.device = InterconnectConfig_pb2.ETHERNET_DEVICE_EXPANSION
        ethernet_configuration.enabled = False
        try:
            self.interconnect_config.SetEthernetConfiguration(ethernet_configuration, self.interconnect_device_id)
        except Exception as e:
            print ("An unexpected error occured : {}".format(e))

class UARTBridgeConfig:

        def __init__(self, router, ip_address):

            self.router = router
            self.base_ip_address = ip_address

            # Create services
            self.base = BaseClient(self.router)
            self.device_manager =  DeviceManagerClient(self.router)
            self.interconnect_config = InterconnectConfigClient(self.router)

            self.interconnect_device_id = self.GetDeviceIdFromDevType(Common_pb2.INTERCONNECT, 0)
            if (self.interconnect_device_id is None):
                print ("Could not find the Interconnect in the device list, exiting...")
                sys.exit(0)

        def GetDeviceIdFromDevType(self, device_type, device_index = 0):
            devices = self.device_manager.ReadAllDevices()

            current_index = 0
            for device in devices.device_handle:
                if device.device_type == device_type:
                    if current_index == device_index:
                        print ("Found the Interconnect on device identifier {}".format(device.device_identifier))
                        return device.device_identifier
                    current_index += 1
            return None

        def Configure(self, port_id, enabled, speed, word_length, stop_bits, parity):
            '''
            Enable and configure UART on interconnect. This will open a TCP port on the interconnect. This
            port allows bridging TCP socket to UART.
            '''
            uart_config              = Common_pb2.UARTConfiguration()
            uart_config.port_id      = port_id 
            uart_config.enabled      = enabled # Setting enabled to true opens the TCP port dedicated to UART bridging. Setting this
                                            # field to false disables designated uart and closes the TCP port.
            uart_config.speed        = speed 
            uart_config.word_length  = word_length
            uart_config.stop_bits    = stop_bits
            uart_config.parity       = parity

            self.interconnect_config.SetUARTConfiguration(uart_config, deviceId=self.interconnect_device_id)

        def EnableBridge(self, bridge_type, target = 0, output = 0):
            
            # Create bridge configuration
            bridge_config = Base_pb2.BridgeConfig()
            bridge_config.device_identifier = self.interconnect_device_id
            bridge_config.bridgetype = bridge_type

            # If either target or ouput port has valid port value, add port config to bridge configuration
            if target or output:
                bridge_config.port_config.target_port = 0
                bridge_config.port_config.out_port = 0
                if target:
                    bridge_config.port_config.target_port = target
                if output:
                    bridge_config.port_config.out_port = output

            # Send the configuration and return the result
            bridge_result = self.base.EnableBridge(bridge_config)
            return bridge_result

        def DisableBridge(self, bridge_id):
            return self.base.DisableBridge(bridge_id)