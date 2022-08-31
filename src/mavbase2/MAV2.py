#!/usr/bin/env python3

from ast import Pass
from cmath import nan
from tkinter import SEL
from turtle import position
import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSet
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Image
import numpy as np
<<<<<<< HEAD
import math
import sys
from cv_bridge import CvBridge 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
=======
>>>>>>> 14576a08cb8cf75657c640ee13db6354eb4b1a58

from cv_bridge import CvBridge 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

DEBUG = False

class MAV2():

    def __init__(self):
        ############ Attributes #################

        self.rate = rospy.Rate(60)
        self.drone_state = State()
        self.goal_pose = PoseStamped()
        self.drone_pose = PoseStamped()
        self.goal_vel = TwistStamped()
        self.drone_state = State()
        self.drone_extended_state = ExtendedState()
        self.battery = BatteryState()
        self.global_pose = NavSatFix()
        self.cam = Image()
        self.bridge_object = CvBridge()
        self.camera_topic = "/camera/image_raw"

        ############# Services ##################

        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.param_set_srv = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.takeoff_srv = rospy.ServiceProxy('/mavros_msgs/CommandTOL', CommandTOL)
   

    
        ############### Publishers ##############
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 20)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',  TwistStamped, queue_size=5)
        self.target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=5)

        ########## Subscribers ##################
        self.local_atual = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback, queue_size=10) 
        self.battery_sub = rospy.Subscriber('/mavros/battery', BatteryState, self.battery_callback)
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_callback, queue_size=2)        
        self.cam_sub = rospy.Subscriber(self.camera_topic, Image, self.cam_callback, queue_size=2)        

        service_timeout = 15
        try:
            rospy.wait_for_message('/mavros/extended_state', ExtendedState)
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr("failed to connect to services")

        rospy.logerr("Services are up")
        

        while self.drone_state.mode == "":
            pass
        rospy.logerr("Subscribers are up")
       
    ########## Callback functions ###########
    
    def state_callback(self, state_data):
        self.drone_state = state_data
        
    
    def extended_state_callback(self, es_data):
        self.drone_extended_state = es_data
        #Values for referente self.drone_extended_state.landed_state
        #uint8 LANDED_STATE_UNDEFINED = 0
        #uint8 LANDED_STATE_ON_GROUND = 1
        #uint8 LANDED_STATE_IN_AIR = 2
        #uint8 LANDED_STATE_TAKEOFF = 3
        #uint8 LANDED_STATE_LANDING = 4

    def battery_callback (self, battery_data):
        self.battery = battery_data
    
    def local_callback(self, data):
        self.drone_pose = data

    def cam_callback(self, cam_data):
        self.cam = self.bridge_object.imgmsg_to_cv2(cam_data,"bgr8")
    
    
    ########## Battery verification ##########
    def verify_battery(self):
        percentage = self.battery.percentage
        rospy.logerr("Waiting for battery topic...")
        while percentage == 0 and self.battery.voltage == 0:
            pass
        rospy.logerr("Battery percentage is: " + str(percentage))
        rospy.logerr("Battery voltage is: " + str(self.battery.voltage))


    ###Set mode: PX4 mode - string, timeout (seconds) - int
    def set_mode(self, mode):
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        service_timeout = 15
        rospy.wait_for_service('mavros/set_mode', service_timeout)
        while (self.drone_state.mode != mode ):        
            self.set_mode_srv(0, mode)
            
    def set_param(self, param_name, param_value):
        service_timeout = 15
        rospy.wait_for_service('/mavros/param/set', service_timeout)
        a = ParamValue()
        a.real = param_value        
        self.param_set_srv(param_name, a)

    '''def takeoff(self, height):
 
        self.arm()
        
        rospy.loginfo("Takeoff")

        self.go_to_local(0,0,height)



    '''
    def takeoff(self, height, speed=1.5, safety_on=True):
    
        name_alt= 'MIS_TAKEOFF_ALT'
        self.set_param(name_alt, float(height))
        
        name_vel_z = 'MPC_Z_VEL_ALL'
        self.set_param(name_vel_z, -3.0)

        name_speed = 'MPC_TKO_SPEED'
        self.set_param(name_speed, speed)
        

        #if self.drone_extended_state.landed_state != 1:
        #    rospy.logerr("Drone is not grounded! Takeoff cancelled")
        #    return
        self.arm()
        
        self.set_mode("AUTO.TAKEOFF")
        
        if safety_on:
            while self.drone_state.mode != "AUTO.TAKEOFF":
                pass
            while self.drone_state.mode == "AUTO.TAKEOFF":
                pass
        rospy.loginfo("Takeoff completed!")




    def hold(self, hold_time): # hold time in seconds
        x_init = self.drone_pose.pose.position.x
        y_init = self.drone_pose.pose.position.y
        z_init = self.drone_pose.pose.position.z
        now = rospy.get_rostime()
        rospy.loginfo("Goind on hold for " + str(hold_time) + " s")
        while not rospy.get_rostime() - now > rospy.Duration(secs=hold_time):
            self.set_position(x_init, y_init, z_init)
    
    ####### Goal Position and Velocity #########
    def set_position(self, x, y, z, yaw = None):
        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)
        if yaw == None:
            self.goal_pose.pose.orientation = self.drone_pose.pose.orientation
        else:
            [self.goal_pose.pose.orientation.x, 
            self.goal_pose.pose.orientation.y, 
            self.goal_pose.pose.orientation.z,
            self.goal_pose.pose.orientation.w] = quaternion_from_euler(0,0,yaw) #roll,pitch,yaw
            #print("X: " + str(self.goal_pose)))

        while self.drone_state.mode != "OFFBOARD":
            self.local_position_pub.publish(self.goal_pose)
            self.set_mode("OFFBOARD")
        self.local_position_pub.publish(self.goal_pose)

    
    def go_to_local(self, goal_x, goal_y, goal_z, yaw = None, TOL=0.2):
        rospy.loginfo("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))
        current_x = self.drone_pose.pose.position.x
        current_y = self.drone_pose.pose.position.y
        current_z = self.drone_pose.pose.position.z
        [_,_,current_yaw] = euler_from_quaternion([self.drone_pose.pose.orientation.x,self.drone_pose.pose.orientation.y,self.drone_pose.pose.orientation.z,self.drone_pose.pose.orientation.w])
        if yaw == None:
            yaw = current_yaw
        
        if yaw*current_yaw >= 0:
            yaw_diff = yaw - current_yaw
        else:
            yaw_diff = yaw + current_yaw
        while not rospy.is_shutdown() and (np.sqrt((goal_x - current_x  )**2 + (goal_y - current_y)**2 + (goal_z - current_z)**2) + (yaw_diff)**2) > TOL:
            current_x = self.drone_pose.pose.position.x
            current_y = self.drone_pose.pose.position.y
            current_z = self.drone_pose.pose.position.z
            [_,_,current_yaw] = euler_from_quaternion([self.drone_pose.pose.orientation.x,self.drone_pose.pose.orientation.y,self.drone_pose.pose.orientation.z,self.drone_pose.pose.orientation.w])
            self.set_position(goal_x, goal_y, goal_z, yaw)
            if yaw*current_yaw >= 0:
                yaw_diff = yaw - current_yaw
            else:
                yaw_diff = yaw + current_yaw
            if yaw_diff > 3:
                print("deu merda familia")
        rospy.loginfo("Arrived at requested position")

    
    def change_auto_speed(self, vel_xy = None, vel_z = None):
        if vel_xy != None:
            name_vel_xy = 'MPC_XY_VEL_MAX'
            self.set_param(name_vel_xy, vel_xy)
            rospy.loginfo("Horizontal velocity parameter set to " +str(vel_xy))
        else:
            name_xy_max = 'MPC_XY_VEL_MAX'
            self.set_param(name_xy_max,12.0)
            rospy.loginfo("Horizontal velocity parameter set to default")
        if vel_z != None:
            name_vel_z = 'MPC_Z_VEL_ALL'
            self.set_param(name_vel_z, vel_z)
            rospy.loginfo("Vertical velocity parameter set to " +str(vel_z))
        else:
            name_z_max = 'MPC_Z_VEL_ALL'
            self.set_param(name_z_max, -3.0)
            rospy.loginfo("Vertical velocity parameter set to default")

    def set_vel(self, x, y, z, yaw = 0):
        while self.drone_state.mode != "OFFBOARD":
            self.goal_vel.twist.linear.x = float(x)
            self.goal_vel.twist.linear.y = float(y)
            self.goal_vel.twist.linear.z = float(z)

            self.goal_vel.twist.angular.z = float(yaw)
            self.velocity_pub.publish(self.goal_vel)  
            self.set_mode("OFFBOARD")

        self.goal_vel.twist.linear.x = float(x)
        self.goal_vel.twist.linear.y = float(y)
        self.goal_vel.twist.linear.z = float(z)

        self.goal_vel.twist.angular.z = float(yaw)
        self.velocity_pub.publish(self.goal_vel)    


    def land(self, auto_disarm=True, speed=0.7, safety_on=True):
        rospy.loginfo("Landing...")
        name_vel_z = 'MPC_Z_VEL_ALL'
        self.set_param(name_vel_z, -3.0)

        name_speed = 'MPC_LAND_SPEED'
        self.set_param(name_speed, speed)

        self.set_mode('AUTO.LAND')

        if safety_on:
            while self.drone_extended_state.landed_state != 4:
                self.rate.sleep()
            while self.drone_extended_state.landed_state == 4:
                self.rate.sleep()
        rospy.loginfo("Landing completed!")

        if auto_disarm:
            self.disarm()
        
    ########## Arm #######

    def arm(self):
        service_timeout = 15
        rospy.loginfo('ARMING MAV') 
        rospy.wait_for_service('mavros/cmd/arming', service_timeout)
        self.arm_srv(True)
        while not self.drone_state.armed:
            self.arm_srv(True)
        rospy.loginfo('Drone is armed')
        

    ########## Disarm #######
    def disarm(self):
        service_timeout = 15
        rospy.loginfo('DISARMING MAV')
        rospy.wait_for_service('mavros/cmd/arming', service_timeout)
        self.arm_srv(False)
        while not self.drone_state.armed:
            self.arm_srv(False)
        rospy.loginfo('Drone is disarmed')

    def takeoff_teste(self, height):
        velocity = 1 # velocidade media

        for i in range(100):
            self.set_position(self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, 0)
            self.rate.sleep()

        self.set_mode("OFFBOARD", 2)

        if not self.drone_state.armed:
            rospy.logwarn("ARMING DRONE")
            fb = self.arm(True)
            while not fb.success:
                if DEBUG:
                    rospy.logwarn("ARMING DRONE {}".format(fb))
                fb = self.arm(True)
                self.rate.sleep()
            rospy.loginfo("DRONE ARMED")
        else:
            rospy.loginfo("DRONE ALREADY ARMED")
        self.rate.sleep()
        rospy.logwarn("EXECUTING TAKEOFF METHODS")
        p = self.drone_pose.pose.position.z
        time=0
        while abs(self.drone_pose.pose.position.z - height) >= TOL and not rospy.is_shutdown():
            time += 1/60.0#sec - init_time
            if DEBUG:
                rospy.logwarn('TAKING OFF AT ' + str(velocity) + ' m/s')   
            
            if p < height:
                p = ((-2 * (velocity**3) * (time**3)) / height**2) + ((3*(time**2) * (velocity**2))/height)
                self.go_to_local(self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, p)

            else:
                self.go_to_local(self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, height)

            #rospy.loginfo('Position: (' + str(self.drone_pose.pose.position.x) + ', ' + str(self.drone_pose.pose.position.y) + ', '+ str(self.drone_pose.pose.position.z) + ')')

        self.rate.sleep()
        self.go_to_local(self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, height)
        rospy.loginfo("TAKEOFF FINISHED")
        
        return "done"

if __name__ == '__main__':
    rospy.init_node('mavbase2')
    mav = MAV2()
    #mav.set_mode("AUTO.LAND")
    #mav.set_param("MIS_TAKEOFF_ALT", 2)
<<<<<<< HEAD

    mav.takeoff_teste(0.5)
    mav.hold(10)
    #mav.land()
=======
    #mav.change_auto_speed(vel_xy = 2, vel_z = 1.5)
    mav.takeoff(2)
    #mav.go_to_local(0,0,3, yaw = -1.5708)
    #mav.go_to_local(5,0,3)
    #mav.go_to_local(0,0,3, yaw = 0)
    #mav.go_to_local(5,0,3)
    mav.go_to_local(0,0,3, yaw = 1.5708)
    mav.go_to_local(5,0,3)
    mav.go_to_local(0,0,3, yaw = 3.14)
    mav.go_to_local(5,0,3)
    mav.go_to_local(0,0,3, yaw = -3.14)
    mav.go_to_local(5,0,3)


    mav.land()
>>>>>>> 14576a08cb8cf75657c640ee13db6354eb4b1a58

  

    
