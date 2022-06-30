#!/usr/bin/env python3

from tkinter import SEL
from turtle import position
import rclpy
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from rcl_interfaces.msg import ParameterType, Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Image
from std_msgs.msg import String
from rclpy.node import Node
from rclpy import qos
from action_py.setPosition_action_client import SetPositionActionClient
from action_py.setPosition_action_server import SetPositionActionServer
import numpy as np
import sys
from cv_bridge import CvBridge 

from mavbase2.action import SetPosition 

DEBUG = False

class MAV2(Node):

    def __init__(self):
        super().__init__('mav')

        ############ Attributes #################

        self.rate = self.create_rate(60)
        self.desired_state = ""
        self.drone_state = State()
        self.goal_pose = PoseStamped()
        self.drone_pose = PoseStamped()
        self.pose_target = PositionTarget()
        self.goal_vel = TwistStamped()
        self.drone_state = State()
        self.battery = BatteryState()
        self.global_pose = NavSatFix()
        self.gps_target = GeoPoseStamped()
        self.cam = Image()
        self.bridge_object = CvBridge()
        self.camera_topic = ""

        ############# Services ##################

        self.set_mode_srv = self.create_client(SetMode, '/mavros/set_mode')

        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.param_set_srv = self.create_client(SetParameters,'/mavros/param/set_parameters')
        
        ############## Action Server ##################
        self.setPosition_action_server = SetPositionActionServer()
        
        ############# Action Clients ###################
        self._setposition_action_client = SetPositionActionClient()

        ############### Publishers ##############

        self.local_position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 20)
        self.velocity_pub = self.create_publisher(TwistStamped,  '/mavros/setpoint_velocity/cmd_vel', 5)
        self.target_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 5)
        self.global_position_pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 20)

        ########## Subscribers ##################

        self.local_atual = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_callback, qos.qos_profile_sensor_data)
        self.state_sub =  self.create_subscription(State, '/mavros/state', self.state_callback, qos.qos_profile_sensor_data)
        self.battery_sub =  self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos.qos_profile_sensor_data)
        self.global_position_sub =  self.create_subscription(NavSatFix,'/mavros/global_position/global' , self.global_callback, qos.qos_profile_sensor_data)
        self.cam_sub = self.create_subscription(Image, self.camera_topic, self.cam_callback, qos.qos_profile_sensor_data)


        service_timeout = 15
        while not self.set_mode_srv.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().info('Set mode service not available, waiting again...')
        while not self.arm_srv.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().info('Arm service not available, waiting again...')
        while not self.param_set_srv.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().info('Set param service not available, waiting again...')

        self.arm_req = CommandBool.Request()
        self.set_mode_req = SetMode.Request()
        self.param_set_req = SetParameters.Request()
        self.get_logger().info("Services are up")

        while self.drone_state.mode == "":
            rclpy.spin_once(self)
        self.get_logger().info("Subscribers are up")
        #self.future = self.NOMEDOSRV.call_async(self.req)

    ########## Callback functions ###########
    
    def state_callback(self, state_data):
        self.drone_state = state_data

    def battery_callback (self, battery_data):
        self.battery = battery_data
    
    def local_callback(self, data):
        self.drone_pose = data
    
    def global_callback(self, global_data):
        self.global_pose = global_data    

    def cam_callback(self, cam_data):
        self.cam = self.bridge_object.imgmsg_to_cv2(cam_data,"bgr8")
    
    ###Set mode: PX4 mode - string, timeout (seconds) - int
    def set_mode(self, mode):
        #self.get_logger().info("setting FCU mode: {0}".format(mode))
        self.set_mode_req.base_mode = 0
        self.set_mode_req.custom_mode = mode
        future = self.set_mode_srv.call_async(self.set_mode_req)  # 0 is custom mode
        while future.result() == None:
            rclpy.spin_once(self)
        if not future.result().mode_sent:
            raise Exception("failed to send mode command")
            
    def set_param(self, param_value):
        self.param_set_req.parameters = [param_value]
        self.param_set_srv.call_async(self.param_set_req)

    def takeoff(self, height, speed=1.5, safety_on=True):
        height = float(height)

        alt_param_value = Parameter(name= 'MIS_TAKEOFF_ALT', value=ParameterValue(double_value=height, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(alt_param_value)
        
        speed = float(speed)
        speed_param_value = Parameter(name= 'MPC_TKO_SPEED', value=ParameterValue(double_value=speed, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(speed_param_value)
        rclpy.spin_once(self)
        if self.drone_state.armed:
            self.get_logger().error("Drone already armed! Takeoff cancelled")
            return
        self.__arm()
        self.set_mode("AUTO.TAKEOFF")
        
        #safety measures: locks program while taking off
        if safety_on:
            while self.drone_state.mode != "AUTO.TAKEOFF":
                rclpy.spin_once(self)
            while self.drone_state.mode == "AUTO.TAKEOFF":
                rclpy.spin_once(self)
    
    ####### Goal Position and Velocity #########
    def set_position(self, x, y, z):
        while self.drone_state.mode != "OFFBOARD":
            self.goal_pose.pose.position.x = float(x)
            self.goal_pose.pose.position.y = float(y)
            self.goal_pose.pose.position.z = float(z)
            self.local_position_pub.publish(self.goal_pose)
            self.set_mode("OFFBOARD")

        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)
        self.local_position_pub.publish(self.goal_pose)
    
    def go_to_local(self, goal_x, goal_y, goal_z, TOL=0.2):
        self.get_logger().info("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + ")")
        current_x = self.drone_pose.pose.position.x
        current_y = self.drone_pose.pose.position.y
        current_z = self.drone_pose.pose.position.z
        while(np.sqrt((goal_x - current_x  )**2 + (goal_y - current_y)**2 + (goal_z - current_z)**2)) > TOL:
            rclpy.spin_once(self)
            current_x = self.drone_pose.pose.position.x
            current_y = self.drone_pose.pose.position.y
            current_z = self.drone_pose.pose.position.z
            self.set_position(goal_x, goal_y, goal_z)
        self.get_logger().info("Arrived at local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + ")")

    def set_vel(self, x, y, z, roll = 0, pitch = 0, yaw = 0):
        while self.drone_state.mode != "OFFBOARD":
            self.goal_vel.twist.linear.x = float(x)
            self.goal_vel.twist.linear.y = float(y)
            self.goal_vel.twist.linear.z = float(z)

            self.goal_vel.twist.angular.x = float(roll)
            self.goal_vel.twist.angular.y = float(pitch)
            self.goal_vel.twist.angular.z = float(yaw)
            self.velocity_pub.publish(self.goal_vel)  
            self.set_mode("OFFBOARD")

        self.goal_vel.twist.linear.x = float(x)
        self.goal_vel.twist.linear.y = float(y)
        self.goal_vel.twist.linear.z = float(z)

        self.goal_vel.twist.angular.x = float(roll)
        self.goal_vel.twist.angular.y = float(pitch)
        self.goal_vel.twist.angular.z = float(yaw)
        self.velocity_pub.publish(self.goal_vel)    


    def land(self, auto_disarm=True, speed=0.7):
        if auto_disarm:
            auto_disarm_param_value= Parameter(name= 'COM_DISARM_LAND', value=ParameterValue(double_value= 2.0, type=ParameterType.PARAMETER_DOUBLE))
        else:
            auto_disarm_param_value= Parameter(name= 'COM_DISARM_LAND', value=ParameterValue(double_value= -1.0, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(auto_disarm_param_value)

        speed = float(speed)
        land_speed_param_value= Parameter(name= 'MPC_LAND_SPEED', value=ParameterValue(double_value= speed, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(land_speed_param_value)

        self.set_mode('AUTO.LAND')

    ########## Arm #######

    def __arm(self):
        self.get_logger().warn('ARMING MAV')
        self.arm_req.value = True
        self.arm_srv.call_async(self.arm_req)
        

    ########## Disarm #######
    def __disarm(self):
        self.get_logger().warn('DISARMING MAV')
        self.arm_req.value = False
        self.arm_srv.call_async(self.arm_req) #substituir isso por um metodo mais seguro depois (comentei pq o subscriber de pose n ta funcionando)
        #if self.drone_pose.pose.position.z < TOL:
        #    for i in range(3):
                #self.arm_srv.call_async(self.arm_req)
        #        if DEBUG:
        #            self.get_logger().info('Drone height' + str(self.drone_pose.pose.position.z))
        #
        #else:
        #    self.get_logger().warn('Altitude too high for disarming!')
        #    self.land()
        #    self.arm_srv.call_async(self.arm_req)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    mav = MAV2()
    mav.takeoff(5)
    mav.go_to_local(-10,5,2,0.1)



   
   

    