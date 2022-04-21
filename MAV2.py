import rclpy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
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
        self.rate = self.create_rate(60)
        self.desired_state = ""
        
        ############# Services ##################
        self.set_mode_srv = self.create_client('/mavros/set_mode', SetMode)
        
        self.arm = self.create_client('/mavros/cmd/arming', CommandBool)
         
        
        ############## Actions ##################

        ############### Publishers ##############

        ########## Subscribers ##################


        service_timeout = 15
        while not self.cli.wait_for_service(service_timeout):
            self.get_logger().info('service not available, waiting again...')
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
                    result = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not result.mode_sent:
                        self.get_logger().info("failed to send mode command")
                except SystemError as e:
                    print('exception in server:', file=sys.stderr)
                    self.get_logger().info(e)
                except rospy.ServiceException as e:
                    self.get_logger().info(e)

            try:
                loop_rate.sleep()
            except rospy.ROSException as e:
                self.get.logger().info(e)
        
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
    mav = MAV("jorge")
    mav.takeoff(3)
    mav.RTL()