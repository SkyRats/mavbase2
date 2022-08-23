#!/usr/bin/env python3

from ast import Pass
from tkinter import SEL
from turtle import position
import rospy
from mavros_msgs.srv import SetMode, CommandBool, WaypointPull, WaypointSetCurrent, ParamSet
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Image
import numpy as np
import math
import sys
from cv_bridge import CvBridge 
from tf_transformations import quaternion_from_euler, euler_from_quaternion


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

        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.param_set_srv = rospy.ServiceProxy('/mavros/param/set', ParamSet)
     
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
    
    def local_callback(self, local):
        self.drone_pose.pose.position.x = local.pose.position.x
        self.drone_pose.pose.position.y = local.pose.position.y
        self.drone_pose.pose.position.z = local.pose.position.z

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
        #self.get_logger().info("setting FCU mode: {0}".format(mode))
        service_timeout = 15
        rospy.wait_for_service('mavros/set_mode', service_timeout)
  
        result = self.set_mode_srv(0, mode)
        if not result.mode_sent:
            rospy.logerr("failed to send mode command")
            
    def set_param(self, param_name, param_value):
        service_timeout = 15
        rospy.wait_for_service('/mavros/param/set', service_timeout)
        a = ParamValue()
        a.real = param_value        
        self.param_set_srv(param_name, a)

    def takeoff(self, height, speed=1.5, safety_on=True):
    
        name_alt= 'MIS_TAKEOFF_ALT'
        self.set_param(name_alt, height)
        
        name_vel_z = 'MPC_Z_VEL_ALL'
        self.set_param(name_vel_z, -3)

        name_speed = 'MPC_TKO_SPEED'
        self.set_param(name_speed, speed)
        

        if self.drone_extended_state.landed_state != 1 and self.drone_extended_state.landed_state != 0:
            print(self.drone_extended_state.landed_state)
            rospy.logerr("Drone is not grounded! Takeoff cancelled")
            return
        self.arm()
        self.set_mode("AUTO.TAKEOFF")
        
        #safety measures: locks program while taking off
        if safety_on:
            while self.drone_extended_state.landed_state != 3:
                pass
            while self.drone_extended_state.landed_state == 3:
                pass
        self.get_logger().info("Takeoff completed!")

    def hold(self, hold_time): # hold time in seconds
        init = now = self.get_clock().now()
        now = self.get_clock().now()
        time = rclpy.duration.Duration(seconds=hold_time, nanoseconds=0)
        self.get_logger().info("Goind on hold for " + str(hold_time) + " s")
        while (now - init).nanoseconds < hold_time*10**9:
            self.set_position(self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, self.drone_pose.pose.position.z)
            now = self.get_clock().now()
    
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

        while self.drone_state.mode != "OFFBOARD":
            self.local_position_pub.publish(self.goal_pose)
            self.set_mode("OFFBOARD")
        self.local_position_pub.publish(self.goal_pose)

    
    def go_to_local(self, goal_x, goal_y, goal_z, yaw = None, vel_xy = None, vel_z = None, TOL=0.2):
        self.get_logger().info("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))
        current_x = self.drone_pose.pose.position.x
        current_y = self.drone_pose.pose.position.y
        current_z = self.drone_pose.pose.position.z
        [_,_,current_yaw] = euler_from_quaternion([self.drone_pose.pose.orientation.x,self.drone_pose.pose.orientation.y,self.drone_pose.pose.orientation.z,self.drone_pose.pose.orientation.w])
        if yaw == None:
            yaw = current_yaw
        self.change_auto_speed(vel_xy, vel_z)
        while(np.sqrt((goal_x - current_x  )**2 + (goal_y - current_y)**2 + (goal_z - current_z)**2) + (current_yaw - yaw)**2) > TOL:
            rclpy.spin_once(self)
            current_x = self.drone_pose.pose.position.x
            current_y = self.drone_pose.pose.position.y
            current_z = self.drone_pose.pose.position.z
            [_,_,current_yaw] = euler_from_quaternion([self.drone_pose.pose.orientation.x,self.drone_pose.pose.orientation.y,self.drone_pose.pose.orientation.z,self.drone_pose.pose.orientation.w])
            self.set_position(goal_x, goal_y, goal_z, yaw)
        self.get_logger().info("Arrived at requested position")

    def go_to_global(self, lat, lon, alt,yaw=0, vel_xy = None, vel_z = None, GLOBAL_TOL = 0.2):
        self.get_logger().info("Going to latitude " + str(lat) + ", longitude " + str(lon) + " and altitude: " + str(alt))
        self.gps_target.pose.position.latitude = lat
        self.gps_target.pose.position.longitude = lon
        self.gps_target.pose.position.altitude = alt
        time_stamp = Clock().now()
        self.gps_target.header.stamp = time_stamp.to_msg()
        goal = [lat, lon]
        actual_global_pose = [self.global_pose.latitude, self.global_pose.longitude]
        dist = self.global_dist(goal, actual_global_pose)
        self.get_logger().info("Distance: " + str(dist))
        if dist > 550:
            self.get_logger().warn("Distance too far! Trajectory cancelled...")
            return
        self.change_auto_speed(vel_xy, vel_z)
        while self.drone_state.mode != "OFFBOARD":
                self.global_position_pub.publish(self.gps_target)
                self.set_mode("OFFBOARD")
        while dist >= GLOBAL_TOL:
            self.global_position_pub.publish(self.gps_target)
            actual_global_pose = [self.global_pose.latitude, self.global_pose.longitude]
            dist = self.global_dist(goal, actual_global_pose)
            rclpy.spin_once(self)
        self.get_logger().info("Arrived at latitude: " + str(self.global_pose.latitude)+ " longitude: " + str(self.global_pose.longitude))

    def change_auto_speed(self, vel_xy = None, vel_z = None):
        if vel_xy != None:
            vel_xy_param_value = Parameter(name= 'MPC_XY_VEL_MAX', value=ParameterValue(double_value=float(vel_xy), type=ParameterType.PARAMETER_DOUBLE))
            self.set_param(vel_xy_param_value)
            self.get_logger().info("Horizontal velocity parameter set to " +str(vel_xy))
        else:
            vel_xy_param_value = Parameter(name= 'MPC_XY_VEL_MAX', value=ParameterValue(double_value=12.0, type=ParameterType.PARAMETER_DOUBLE))
            self.set_param(vel_xy_param_value)
            self.get_logger().info("Horizontal velocity parameter set to default")
        if vel_z != None:
           vel_z_param_value = Parameter(name= 'MPC_Z_VEL_ALL', value=ParameterValue(double_value=float(vel_z), type=ParameterType.PARAMETER_DOUBLE))
           self.set_param(vel_z_param_value)
           self.get_logger().info("Vertical velocity parameter set to " +str(vel_z))
        else:
           vel_z_param_value = Parameter(name= 'MPC_Z_VEL_ALL', value=ParameterValue(double_value=-3.0, type=ParameterType.PARAMETER_DOUBLE))
           self.set_param(vel_z_param_value)
           self.get_logger().info("Vertical velocity parameter set to default")
        rclpy.spin_once(self)

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
        self.get_logger().info("Landing...")
        rclpy.spin_once(self)

        vel_z_general_param_value = Parameter(name= 'MPC_Z_VEL_ALL', value=ParameterValue(double_value=-3.0, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(vel_z_general_param_value)
        land_speed_param_value= Parameter(name= 'MPC_LAND_SPEED', value=ParameterValue(double_value= float(speed), type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(land_speed_param_value)

        self.set_mode('AUTO.LAND')

        if safety_on:
            while self.drone_extended_state.landed_state != 4:
                rclpy.spin_once(self)
            while self.drone_extended_state.landed_state == 4:
                rclpy.spin_once(self)
        self.get_logger().info("Landing completed!")

        if auto_disarm:
            self.disarm()
        
    ########## Arm #######

    def arm(self):
        self.get_logger().warn('ARMING MAV')
        self.arm_req.value = True
        while not self.arm_srv.wait_for_service(timeout_sec=5):
            self.get_logger().info('Arm/Disarm service not available, waiting again...')
        self.arm_srv.call_async(self.arm_req)
        self.get_logger().warn('Drone is armed')
        

    ########## Disarm #######
    def disarm(self):
        self.get_logger().warn('DISARMING MAV')
        self.arm_req.value = False
        while not self.arm_srv.wait_for_service(timeout_sec=5):
            self.get_logger().info('Arm/Disarm service not available, waiting again...')
        self.arm_srv.call_async(self.arm_req)
        self.get_logger().warn('Drone is disarmed')
  

    ############ Mission Functions ############
    def mission_start(self):
        self.arm()
        self.get_logger().info('STARTING MISSION MODE')
        self.set_mode("AUTO.MISSION")
    
    def mission_pause(self):
        self.get_logger().info('PAUSING MISSION MODE')
        self.set_mode("AUTO.LOITER")

    def mission_get_waypoints_list(self):
        while not self.waypoint_pull_srv.wait_for_service(timeout_sec=5):
            self.get_logger().info('Waypoint pull service not available, waiting again...')
        future = self.waypoint_pull_srv.call_async(self.waypoint_pull_req)
        while future.result() == None:
            rclpy.spin_once(self)
        return self.received_waypoints.waypoints
    
    def mission_get_current_waypoint(self):
        while not self.waypoint_pull_srv.wait_for_service(timeout_sec=5):
            self.get_logger().info('Waypoint pull service not available, waiting again...')
        future = self.waypoint_pull_srv.call_async(self.waypoint_pull_req)
        while future.result() == None:
            rclpy.spin_once(self)
        return self.received_waypoints.current_seq
    
    def mission_set_current_waypoint(self,wp_number):
        self.waypoint_set_req.wp_seq = wp_number
        while not self.waypoint_set_srv.wait_for_service(timeout_sec=5):
            self.get_logger().info('Waypoint set service not available, waiting again...')
        future = self.waypoint_set_srv.call_async(self.waypoint_set_req)
        while future.result() == None:
            rclpy.spin_once(self)
        return future.result().success
    
    def mission_infinite_loop(self): #WARNING THIS IS AN INFINITE LOOP, SHOULD BE USED FOR DEBUG ONLY!
        self.get_logger().error('INFINITE MISSION MODE, KILL THE PROGRAM TO EXIT')
        self.mission_start()
        while rclpy.ok():
            if self.mission_get_current_waypoint() == len(self.mission_get_waypoints_list())-1:
                print("setting waypoint 0")
                self.mission_set_current_waypoint(0)
                
    ############ Additional functions #############
    def geoid_height(self, lat, lon):
        return self._egm96.height(lat, lon)

    def global_dist(self, coord1 , coord2):  # distance from 2 gps coordinates using Haversine function
        #coord1 = [lat, long]
        lat1,lon1=coord1
        lat2,lon2=coord2

        R=6371000                               # radius of Earth in meters
        phi_1=math.radians(lat1)
        phi_2=math.radians(lat2)

        delta_phi=math.radians(lat2-lat1)
        delta_lambda=math.radians(lon2-lon1)

        a=math.sin(delta_phi/2.0)**2+\
        math.cos(phi_1)*math.cos(phi_2)*\
        math.sin(delta_lambda/2.0)**2
        c=2*math.atan2(math.sqrt(a),math.sqrt(1-a))
        
        return R*c                             # output distance in meters
    
    ############# Centralization Functions #############
    def camera_pid(self, delta_x, delta_y, delta_area):
        self.TARGET = (int(self.cam.shape[0]/2), int(self.cam.shape[1]/2))
        self.TOL = 0.0140
        self.PID = 1/2000
        self.PID_area = 1/500000

        # Centralization PID
        vel_x = delta_y * self.PID
        vel_y = delta_x * self.PID
        vel_z = delta_area * self.PID_area

        # Set PID tolerances
        if abs(vel_x) < self.TOL:
            vel_x = 0.0
        if abs(vel_y) < self.TOL:
            vel_y = 0.0
        if abs(vel_z) < self.TOL:
            vel_z = 0.0
        
        # Set drone instant velocity
        self.set_vel(vel_x, vel_y, vel_z)
        self.get_logger().info(f"Set_vel -> x: {vel_x} y: {vel_y} z: {vel_z}")

if __name__ == '__main__':
    rospy.init_node('mavbase2')
    mav = MAV2()
    #mav.set_mode("AUTO.LAND")
    #mav.set_param("MIS_TAKEOFF_ALT", 2)
    mav.takeoff(2)


    
