#from asyncio.windows_events import NULL
from tkinter import SEL
from turtle import position
import rclpy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from rcl_interfaces.msg import ParameterType, Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy import qos
import numpy as np
import math
import time
import sys

from mavbase2.action import Takeoff


TOL = 0.3
DEBUG = False
#mavros_local_position_pub: '/mavros/setpoint_position/local'
#mavros_velocity_pub : '/mavros/setpoint_velocity/cmd_vel'
#mavros_local_atual :  '/mavros/local_position/pose'
#mavros_state_sub :    '/mavros/state'
#mavros_arm :          '/mavros/cmd/arming'
#mavros_set_mode :     '/mavros/set_mode'
#mavros_battery_sub :  '/mavros/battery'
#extended_state_sub:   '/mavros/extended_state'
#mavros_global_position_sub: '/mavros/global_position/global'
#mavros_pose_target_sub:  '/mavros/setpoint_raw/local'
#mavros_set_global_pub: '/mavros/setpoint_position/global'

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

        ############# Services ##################

        self.set_mode_srv = self.create_client(SetMode, '/mavros/set_mode')

        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.param_set_srv = self.create_client(SetParameters,'/mavros/param/set_parameters')
        

        '''
        ############## Action Servers ##################
        self._action_server = ActionServer(
            self,
            setPosition,
            'setposition',
            self.setposition_server_callback)

        ############# Action Clients ###################
        self._seetposition_action_client = ActionClient(self, setPosition, 'setposition')
        '''

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
        print("Subscribers are up")
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

    '''
    def setposition_server_callback(self, height):
        self.get_logger().info("...")

        feedback_msg = setPosition.Feedback()
        velocity = 1 # velocidade media

        self.set_mode("OFFBOARD", 2)

        if not self.drone_state.armed:
            self.get_logger().warn("ARMING DRONE")
            fb = self.arm(True)
            while not fb.success:
                if DEBUG:
                    self.get_logger().warn("ARMING DRONE {}".format(fb))
                fb = self.arm(True)
                self.rate.sleep()
            self.get_logger().info("DRONE ARMED")
        else:
            self.get_logger().info("DRONE ALREADY ARMED")

        self.rate.sleep()
        self.get_logger().info('Executing takeoff methods...')

        p = self.drone_pose.pose.position.z
        time = 0
        #while abs(self.drone_pose.pose.position.z - height) >= TOL and not rclpy.is_shutdown():
        while abs(self.takeoff_feedback - height) >= TOL and not rclpy.is_shutdown():
            time += 1/60.0 #sec - init_time

            if p < height:
                p = ((-2 * (velocity**3) * (time**3)) / height**2) + ((3*(time**2) * (velocity**2))/height)
                self.drone_pose.pose.position.z = p
            else:
                self.drone_pose.pose.position.z = height

        actualHeight = self.drone_pose.pose.position.z

        for i in range(1, height.request.height_request):
            feedback_msg.partial_height = actualHeight
            self.get_logger().info('setPosition feedback: {0}'.format(feedback_msg.partial_height))
            height.publish_feedback(feedback_msg)
            time.sleep(1)

        height.succeed()

        result = setPosition.Result()

        result.height_result = feedback_msg.partial_height

        return result


    # Takeoff server call response
    def setposition_response_callback(self, future):
        takeoff_handle = future.result()
        if not takeoff_handle.accepted:
            self.get_logger().info("Set position command DENIED")
            return

        self.get_logger().info("Set position command ACCEPTED")

        self._get_result_future = takeoff_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_takeoff_result_callback)

    # Takeoff height after takeoff method completion
    def get_setposition_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Takeoff result: {0}".format(result.height_result))
        rclpy.shutdown()

    # Takeoff height durind ascension
    def setposition_feedback_callback(self, feedback_msg):
        self._setposition_feedback = feedback_msg.feedback
        self.get_logger().info("Received set position feedback: {0}".format(self._setposition_feedback.partial_height))

    '''
    
    
    ###Set mode: PX4 mode - string, timeout (seconds) - int
    def set_mode(self, mode, timeout):
        self.get_logger().info("setting FCU mode: {0}".format(mode))
        self.desired_state = mode
        old_mode = self.drone_state.mode
        loop_freq = 1  # Hz
        loop_rate = self.create_rate(loop_freq)
        mode_set = False
        for i in range(timeout * loop_freq):
            if self.drone_state.mode == mode:
                mode_set = True
                break
            else:
                try:
                    self.set_mode_req.base_mode = 0
                    self.set_mode_req.custom_mode = mode
                    future = self.set_mode_srv.call_async(self.set_mode_req)  # 0 is custom mode
                    #if not future.result().mode_sent:
                    #    self.get_logger().info("failed to send mode command")
                except Exception as e:
                    self.get_logger().info(e)
            #try:
            #    loop_rate.sleep()
            #except Exception as e:
            #    self.get_logger().info(e)
    
    def set_param(self, param_value):
        self.param_set_req.parameters = [param_value]
        self.param_set_srv.call_async(self.param_set_req)

    '''
    def __setPosition (self, x, y, z):
       height_goal_msg = Takeoff.Goal()
       height_goal_msg.height_request = float(height)

       self._setposition_action_client.wait_for_server()

       self._send_setposition_future = self._takeoff_action_client.send_goal_async(height_goal_msg, feedback_callback=self.takeoff_feedback_callback)
       self._send_setposition_future.add_done_callback(self.setposition_response_callback)
       return self._send_setposition_future

    def setPosition(self, x, y, z):
       future = self.__takeoff(height)
       rclpy.spin_until_future_complete(self, future)

    '''
    def takeoff(self, height, speed=1.5, safety_on=True):
        height = float(height)

        alt_param_value = Parameter(name= 'MIS_TAKEOFF_ALT', value=ParameterValue(double_value=height, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(alt_param_value)
        
        speed = float(speed)
        speed_param_value = Parameter(name= 'MPC_TKO_SPEED', value=ParameterValue(double_value=speed, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(speed_param_value)

        self.__arm()
        self.set_mode("AUTO.TAKEOFF", 2)
        
        #safety measures: locks program while taking off
        if safety_on:
            while self.drone_state.mode != "AUTO.TAKEOFF":
                rclpy.spin_once(self)
            while self.drone_state.mode == "AUTO.TAKEOFF":
                rclpy.spin_once(self)

    """
    ####### Goal Position and Velocity #########
    def set_position(self, x, y, z):
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z
        self.local_position_pub.publish(self.goal_pose)
        self.rate.sleep()
        
    def set_vel(self, x, y, z, roll = 0, pitch = 0, yaw = 0):
        self.goal_vel.twist.linear.x = x 
        self.goal_vel.twist.linear.y = y
        self.goal_vel.twist.linear.z = z 

        self.goal_vel.twist.angular.x = roll
        self.goal_vel.twist.angular.y = pitch
        self.goal_vel.twist.angular.z = yaw
        self.velocity_pub.publish(self.goal_vel)    
    """


    def land(self, auto_disarm=True, speed=0.7):
        if auto_disarm:
            auto_disarm_param_value= Parameter(name= 'COM_DISARM_LAND', value=ParameterValue(double_value= 2.0, type=ParameterType.PARAMETER_DOUBLE))
        else:
            auto_disarm_param_value= Parameter(name= 'COM_DISARM_LAND', value=ParameterValue(double_value= -1.0, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(auto_disarm_param_value)

        speed = float(speed)
        land_speed_param_value= Parameter(name= 'MPC_LAND_SPEED', value=ParameterValue(double_value= speed, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(land_speed_param_value)

        self.set_mode('AUTO.LAND',2)

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
    mav.land()
    #rclpy.spin(mav)