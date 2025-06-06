#/usr/bin/env python3
from calendar import c
from px4_msgs.msg._vehicle_control_mode import VehicleControlMode
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleStatus
)
from std_msgs.msg import Bool

class SimpleMission(Node):
    """
    A simple ROS 2 node for PX4 offboard control mission.
    Arms a quadcopter, takes off to 10m, flys forward 10m, and lands.

    Based on Jaeyoung Lim's offboard controll example: https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/px4_offboard/offboard_control.py
    """

    def __init__(self):
        super().__init__('simple_mission_node')

        self.get_logger().info("Starting Simple Mission Node")

        # Quality of Service (QoS) settings for the subscribers and publishers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos)
        
        # Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        # creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # 50 Hz control loop
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.offboard_mode = False
        self.flight_check = False
        self.arm_message = True
        self.failsafe = False

    def vehicle_status_callback(self, msg: VehicleStatus):
        nav_state_names = {
            VehicleStatus.NAVIGATION_STATE_MANUAL: "MANUAL",
            VehicleStatus.NAVIGATION_STATE_ALTCTL: "ALTCTL",
            VehicleStatus.NAVIGATION_STATE_POSCTL: "POSCTL",
            VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: "AUTO_MISSION",
            VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: "AUTO_LOITER",
            VehicleStatus.NAVIGATION_STATE_AUTO_RTL: "AUTO_RTL",
            VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: "AUTO_TAKEOFF",
            VehicleStatus.NAVIGATION_STATE_AUTO_LAND: "AUTO_LAND",
            VehicleStatus.NAVIGATION_STATE_OFFBOARD: "OFFBOARD",
            VehicleStatus.NAVIGATION_STATE_STAB: "STAB",
            VehicleStatus.NAVIGATION_STATE_ACRO: "ACRO",
            VehicleStatus.NAVIGATION_STATE_DESCEND: "DESCEND",
            VehicleStatus.NAVIGATION_STATE_TERMINATION: "TERMINATION"
        }
        # TODO: handle NED->ENU transformation (North-East-Down to East-North-Up) See https://mavlink.io/en/messages/common.html#MAV_FRAME
        self.get_logger().info(f"NAV_STATE: {nav_state_names[msg.nav_state]}, CURRENT_STATE: {self.current_state}, ARMING_STATE: {msg.arming_state}, FLIGHT_CHECK: {msg.pre_flight_checks_pass}, FAILSAFE: {msg.failsafe}, ARM_MESSAGE: {self.arm_message}")
        
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass

    # Wrapper function to publish a vehicle command
    # Docs for VehicleCommand: https://docs.px4.io/main/en/msg_docs/VehicleCommand.html
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7       # altitude value in takeoff command
        msg.command = command     # command ID
        msg.target_system = 1     # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1     # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.publisher_vehicle_command.publish(msg)

    # Arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

     # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=10.0) # param7 is the altitude in meters
        self.get_logger().info("Takeoff command send")

    # Switches to offboard mode
    def state_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # 6.0 is the offboard mode
        self.offboard_mode = True  

    # callback function that arms, takes off, and switches to offboard mode
    # implements a finite state machine
    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if(self.flight_check and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flight_check) or self.arming_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()

            case "LANDING":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Landing, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Landing, Idle")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                
        if(self.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.get_logger().info(f"State changed from {self.last_state} to {self.current_state}")
            self.last_state = self.current_state

    def cmdloop_callback(self):
        if(self.offboard_mode == True):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)

            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.position = [0.0, 10.0, -5.0]
            trajectory_msg.yaw = float('nan')  # No yaw control

            self.publisher_trajectory.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)

    node = SimpleMission()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()