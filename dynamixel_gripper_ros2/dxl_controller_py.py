#!/usr/bin/env python
# license removed for brevity
# This scripts runs a node that opens and closes the dyanmixel gripper via ros topics
# Created by Poh Yong Keat 2018

import rclpy
from dynamixel_sdk import *
from dynamixel_sdk.port_handler import *
from dynamixel_sdk.packet_handler import *
from rclpy.node import Node
from dynamixel_gripper_ros2_msgs.msg import GripState
from dynamixel_gripper_ros2_msgs.msg import LoadState
from std_msgs.msg import String, Float64, Int32

# Control table address for Dynamixel AX
ADDR_AX_TORQUE_ENABLE       = 24
ADDR_AX_GOAL_POSITION       = 30
ADDR_AX_VELOCITY            = 32
ADDR_AX_PRESENT_POSITION    = 36
ADDR_AX_PRESENT_LOAD        = 40
ADDR_AX_PRESENT_TEMP        = 43

# Protocol version
PROTOCOL_VERSION1           = 1.0               #  Protocol version is used to communicate with Dynamixel

# Default setting
DXLleft_ID                  = 1                 # Left dynamixel id 1, right dynamixel id 2
DXLright_ID                 = 2
BAUDRATE                    = 1000000
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXLleft_OPEN_POSITION_VALUE = int(2.2355 * (1023/5.2333))           # Dynamixel will rotate between these values
DXLleft_CLOSE_POSITION_VALUE = int(3.1717 * (1023/5.2333))
DXLright_OPEN_POSITION_VALUE = int(3.8009 * (1023/5.2333))           # Dynamixel will rotate between these values
DXLright_CLOSE_POSITION_VALUE = int(2.7164 * (1023/5.2333))
DXL_MOVING_STATUS_THRESHOLD = 5                 # Allowable tolerance between (dynamixel) goal and current position
DXL_MOVING_SPEED            = 20                # Moving speed set to slow
LEFTLOAD_THRESHOLD = 0.3                        # Adjust grip load thresholds accordingly to prevent overloading
RIGHTLOAD_THRESHOLD = 0.3
STEP_RELEASE_ANGLE = 1                          # Rate of step release to reduce load magnitude

global packetHandler
global portHandler
global node

# initialise dynamixel connection & enable torque
def dxl_init():
    global packetHandler
    global portHandler

    import os

    if os.name == 'nt':
        import msvcrt
        def getch():
            return msvcrt.getch().decode()
    else:
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        def getch():
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION1)

    # Open port
    if portHandler.openPort():
        node.get_logger().info("Succeeded to open the port")
    else:
        node.get_logger().error("Failed to open the port")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        node.get_logger().info("Succeeded to change the baudrate")
    else:
        node.get_logger().error("Failed to change the baudrate")
        getch()
        quit()

    # Enable left Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXLleft_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        node.get_logger().info("Dynamixel#%d has been successfully connected" % DXLleft_ID)

    # Enable right Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXLright_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        node.get_logger().info("Dynamixel#%d has been successfully connected" % DXLright_ID)

    # Set dynamixel joint speed
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLleft_ID, ADDR_AX_VELOCITY, DXL_MOVING_SPEED)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLright_ID, ADDR_AX_VELOCITY, DXL_MOVING_SPEED)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

# Open dynamixel gripper
def dxl_open():
    global packetHandler
    global portHandler

    # Write Dynamixel gripper open position to left and right motors
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLleft_ID, ADDR_AX_GOAL_POSITION, DXLleft_OPEN_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLright_ID, ADDR_AX_GOAL_POSITION, DXLright_OPEN_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

def dxl_close():
    global packetHandler
    global portHandler

    # Write Dynamixel gripper closed position to left and right motors
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLleft_ID, ADDR_AX_GOAL_POSITION, DXLleft_CLOSE_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLright_ID, ADDR_AX_GOAL_POSITION, DXLright_CLOSE_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

# Get current joint position of both servos
def dxl_present_pos():
    global packetHandler
    global portHandler

    # Read Dynamixel gripper present position
    dxlleft_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXLleft_ID, ADDR_AX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

    dxlright_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXLright_ID, ADDR_AX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

    return [dxlleft_present_position,dxlright_present_position]

# Get current temperature of both servos
def dxl_present_temp():
    global packetHandler
    global portHandler

    # Read Dynamixel gripper present temperature
    dxlleft_present_temp, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXLleft_ID, ADDR_AX_PRESENT_TEMP)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

    dxlright_present_temp, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXLright_ID, ADDR_AX_PRESENT_TEMP)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

    return [dxlleft_present_temp,dxlright_present_temp]

# Checks if dynamixel goal position has been met
def dxl_reached(goal,present_pos):
    if(goal == 'close'):
        goal_pos = [DXLleft_CLOSE_POSITION_VALUE,DXLright_CLOSE_POSITION_VALUE]
    elif(goal == 'open'):
        goal_pos = [DXLleft_OPEN_POSITION_VALUE,DXLright_OPEN_POSITION_VALUE]
    else:
        return 0

    # Checks if current position is within given threshold from goal position
    for i in range(2):
        if (abs(goal_pos[i] - present_pos[i]) > DXL_MOVING_STATUS_THRESHOLD):
            return 0

    return 1

# Execute stepwise increment/decrement of joint positions based on current load, to reduce servo load
def dxl_step_release(gripState):
    global node
    global packetHandler
    global portHandler

    new_left_pos = 0
    new_right_pos = 0

    [left_load,right_load] = dxl_get_load()
    [left_pos,right_pos] = dxl_present_pos()

    if(gripState == 1):
        # gradual but slow release of grip angles to maintain grip torque (using step release)
        if(left_load > LEFTLOAD_THRESHOLD or right_load > RIGHTLOAD_THRESHOLD):
            new_left_pos = left_pos-STEP_RELEASE_ANGLE

            new_right_pos = right_pos+STEP_RELEASE_ANGLE

            # Write Dynamixel gripper step release
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLleft_ID, ADDR_AX_GOAL_POSITION, new_left_pos)
            if dxl_comm_result != COMM_SUCCESS:
                node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLright_ID, ADDR_AX_GOAL_POSITION, new_right_pos)
            if dxl_comm_result != COMM_SUCCESS:
                node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

            node.get_logger().info("High load detected in closed pos, releasing joint angles.")


    elif(gripState == 0):
        # gradual but slow release of grip angles to maintain grip torque (using step release)
        if(left_load > LEFTLOAD_THRESHOLD):
            new_left_pos = left_pos+STEP_RELEASE_ANGLE

            # Write Dynamixel gripper step release
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLleft_ID, ADDR_AX_GOAL_POSITION, new_left_pos)
            if dxl_comm_result != COMM_SUCCESS:
                node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

            node.get_logger().info("High load detected in open pos [left], releasing joint angles.")

        # Write Dynamixel gripper step release
        if(right_load > RIGHTLOAD_THRESHOLD):
            new_right_pos = right_pos-STEP_RELEASE_ANGLE

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXLright_ID, ADDR_AX_GOAL_POSITION, new_right_pos)
            if dxl_comm_result != COMM_SUCCESS:
                node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

            node.get_logger().info("High load detected in open pos [right], releasing joint angles.")

# Get torque load from each servo
def dxl_get_load():
    global packetHandler
    global portHandler

    # Read Dynamixel gripper load
    dxlleft_load, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXLleft_ID, ADDR_AX_PRESENT_LOAD)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

    dxlright_load, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXLright_ID, ADDR_AX_PRESENT_LOAD)
    if dxl_comm_result != COMM_SUCCESS:
        node.get_logger().info("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        node.get_logger().error("%s" % packetHandler.getRxPacketError(dxl_error))

    # Normalise load values
    dxlleft_load = (dxlleft_load & int('1111111111', 2)) / 1024.0
    dxlright_load = (dxlright_load & int('1111111111', 2)) / 1024.0

    return [dxlleft_load,dxlright_load]

# Gripper Node class (publishes to /gripper/state & /gripper/load, subscribes to /gripper/command)
class dxl_controller(Node):

    def __init__(self):
        super().__init__('dxl_controller')
        self.gripState = 0
        timer_period = 1.0
        self.pubLoad = self.create_publisher(LoadState, '/gripper/load')
        self.pubState = self.create_publisher(GripState, '/gripper/state')
        self.subCmd = self.create_subscription(Int32,'/gripper/command',self.command_callback)
        self.tmr = self.create_timer(timer_period, self.timer_callback)

        dxl_open()

    def timer_callback(self):

        # Report servo load
        load = LoadState()
        [load_left,load_right] = dxl_get_load()
        load.load_left = float(load_left)
        load.load_right = float(load_right)
        load.avg_load = (load_left+load_right)/2
        self.pubLoad.publish(load)

        # Report open/close state, position, temperature & load of motors
        state = GripState()
        state.gripper_state = self.gripState
        state.avg_temp = sum(dxl_present_temp())/2
        state.avg_load = sum(dxl_get_load())/2
        [state.left_pos,state.right_pos] = [float(dxl_present_pos()[0]),float(dxl_present_pos()[1])]
        self.pubState.publish(state)

        dxl_step_release(self.gripState) # Check to ensure gripper is not overloaded at any point in time

    # Respond to /gripper/command topic
    def command_callback(self,msg):
        gripCommand = msg.data
        state = GripState();
        dxl_pos = dxl_present_pos()

        # Close gripper
        if(gripCommand == 1):
            dxl_close()
            node.get_logger().info('Closing gripper.')
            state.gripper_state = 1
            self.gripState = 1

        # Open gripper
        elif(gripCommand == 0):
            dxl_open()
            node.get_logger().info('Opening gripper.)
            state.gripper_state = 0
            self.gripState = 0

        else:
            node.get_logger().info('Invalid gripper command: {}'.format(gripCommand))

        # update new gripper state
        state.avg_temp = sum(dxl_present_temp())/2
        state.avg_load = sum(dxl_get_load())/2
        [state.left_pos,state.right_pos] = [float(dxl_present_pos()[0]),float(dxl_present_pos()[1])]
        self.pubState.publish(state)

def main(args=None):
    global node

    # initialise dynamixel connection
    dxl_init()

    # initialise new gripper node
    rclpy.init(args=args)
    node = dxl_controller()

    # Spin continuously to publish gripper state
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
