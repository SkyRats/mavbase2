import rclpy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix
#from std_msgs.msg import String
from rclpy.node import Node
import numpy as np
import math
import time



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
        self.pose_target = PositionTarget()
        self.goal_vel = TwistStamped()
        self.drone_state = State()
        self.battery = BatteryState()
        self.global_pose = NavSatFix()
        self.gps_target = GeoPoseStamped()
        
        ############# Services ##################
        
        self.set_mode_srv = self.create_client('/mavros/set_mode', SetMode)
        
        self.arm = self.create_client('/mavros/cmd/arming', CommandBool)
         
        
        ############## Actions ##################

        ############### Publishers ##############
        
        self.local_position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', queue_size = 20)
        self.velocity_pub = self.create_publisher(TwistStamped,  '/mavros/setpoint_velocity/cmd_vel', queue_size=5)
        self.target_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', queue_size=5)
        self.global_position_pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', queue_size= 20)
        
        ########## Subscribers ##################

        self.local_atual = self.create_subscriber(PoseStamped, '/mavros/setpoint_position/local', self.local_callback)
        self.state_sub =  self.create_subscriber(State, '/mavros/state', self.state_callback, queue_size=10) 
        self.battery_sub =  self.create_subscriber(BatteryState, '/mavros/battery', self.battery_callback)
        self.global_position_sub =  self.create_subscriber(NavSatFix,'/mavros/global_position/global' , self.global_callback)
        self.extended_state_sub = self.create_subscriber(ExtendedState,'/mavros/extended_state', self.extended_state_callback, queue_size=2)

        ########## Callback functions ###########


        service_timeout = 15
        while not self.set_mode_srv.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().info('Set mode service not available, waiting again...')
        while not self.arm.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().info('Arm service not available, waiting again...')
            
        self.arm_req = 'mavros/cmd/arming'.Request()
        self.set_mode_req = 'mavros/set_mode'.Request()
        self.get_logger().info("Services are up")
        #self.future = self.NOMEDOSRV.call_async(self.req)


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
                    result = self.set_mode_srv.call_async(self.set_mode_req)  # 0 is custom mode
                    if not result.mode_sent:
                        self.get_logger().info("failed to send mode command")
                except Exception as e:
                    self.get_logger().info(e)
            try:
                loop_rate.sleep()
            except Exception as e:
                self.get_logger().info(e)
        
    def land(self):
        # IMPLEMENTAR
        print()
        
        
    ########## Disarm #######
    def _disarm(self):
        self.get_logger().warn('DISARM MAV')
        self.arm_req = False
        if self.drone_pose.pose.position.z < TOL:
            for i in range(3):
                self.arm.call_async(self.arm_req)
                if DEBUG:
                    self.get_logger().info('Drone height' + str(self.drone_pose.pose.position.z))

        else:
            self.get_logger().warn('Altitude too high for disarming!')
            self.land()
            self.arm.call_async(self.arm_req)
        


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    mav = MAV2("jorge")
    mav.takeoff(3)
    mav.RTL()