import argparse
import csv

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

class DeviceConnection:
    
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(device_ip, username, password): 
        """
        returns RouterClient required to create services and send requests to device or sub-devices,
        """

        return DeviceConnection(device_ip, port=DeviceConnection.TCP_PORT, credentials=(username, password))

    @staticmethod
    def createUARTConnection(device_ip, username, password): 
        """        
        returns RouterClient that allows to create services and send requests to a device or its sub-devices @ 1khz.
        """

        return DeviceConnection(device_ip, port=DeviceConnection.UDP_PORT, credentials=(username, password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    # Called when entering 'with' statement
    def __enter__(self):
        
        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000   # (milliseconds)
            session_info.connection_inactivity_timeout = 2000 # (milliseconds)

            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):
    
        if self.sessionManager != None:

            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000 
            
            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()

def read_csv(filename, angle, position, orientation, gripper_position, translation_speed, action_id):
    """
    Reads a Kinova Gen3 action CSV file and extracts required information.

    Parameters
    ----------
    filename : str
        Path to the CSV file
    angle : list (2D)
        Joint angles (angular motion)
    position : list (2D)
        Cartesian positions (x,y,z for pose)
    orientation : list (2D)
        Cartesian orientation (thetaX, thetaY, thetaZ)
    gripper_position : list
        Finger positions
    translation_speed : list
        Pose translation speed constraints
    action_sequence : list
        Action ID (6=pose, 7=angular, 33=finger)
    """
    with open(filename, newline='', mode='r') as file:
        reader = csv.reader(file)
        for lines in reader:
            #Assuming the csv columns are in the order of angle1, angle2, angle3, angle4, angle5, angle6, 
            # posX, posY, posZ, oriX, oriY, oriZ, gripPos, speed, action

            for row in reader:
                if not row:  # skip empty rows
                    continue

                action_type = row[4].strip()  # 5th column = action ID
                action_id.append(int(action_type))

                #THESE ARE GOOD
                if action_type == "6":  # Cartesian Pose
                    pos = [float(row[27]), float(row[28]), float(row[29])]   # X,Y,Z (cols 28-30)
                    ori = [float(row[30]), float(row[31]), float(row[32])]   # θX,θY,θZ (cols 31-33)
                    spd = float(row[33]) if row[33] else 0.0                 # Speed (col 34)

                    position.append(pos)
                    orientation.append(ori)
                    translation_speed.append(spd)

                #
                elif action_type == "7":  # Angular Motion
                    # Joint angles in columns 6-20 (0-based: 5-19, step by 2 as there identifiers are in between)
                    joints = [float(row[i]) for i in range(5, 19, 2)]
                    angle.append(joints)

                elif action_type == "33":  # Finger Position
                    grip = float(row[24]) if row[24] else 0.0  # col 25
                    gripper_position.append(grip)

    return angle, position, orientation, gripper_position, translation_speed, action_id