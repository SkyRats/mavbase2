#!/usr/bin/env python3

from tkinter import SEL
from turtle import position
import rclpy
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool, WaypointPull, WaypointSetCurrent
from mavros_msgs.msg import State, ExtendedState, PositionTarget, WaypointList, Altitude
from rcl_interfaces.msg import ParameterType, Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Image
from std_msgs.msg import String
from rclpy.node import Node
from rclpy import qos
from rclpy.clock import Clock
from action_py.setPosition_action_client import SetPositionActionClient
from action_py.setPosition_action_server import SetPositionActionServer
import numpy as np
import math
import sys
import cv2
from cv_bridge import CvBridge 
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from pygeodesy.geoids import GeoidPGM
from mavbase2.action import SetPosition 
import csv
import time

DEBUG = False

class MAV2(Node):

    def __init__(self):
        super().__init__('mav')

        ############ Attributes #################

        self.rate = self.create_rate(60)
        self.desired_state = ""
        self.drone_state = State()
        self.pose_target = PositionTarget()
        self.goal_pose = PoseStamped()
        self.drone_pose = PoseStamped()
        self.drone_vel = TwistStamped()
        self.goal_vel = TwistStamped()
        self.drone_state = State()
        self.drone_extended_state = ExtendedState()
        self.battery = BatteryState()
        self.global_pose = NavSatFix()
        self.gps_target = GeoPoseStamped()
        self.cam = Image()
        self.bridge_object = CvBridge()
        self.camera_topic = "/camera/image_raw"
        self._egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)
        self.received_waypoints = []
        self.init_time = time.time()
        self.altitude = Altitude()

        ############# Services ##################

        self.set_mode_srv = self.create_client(SetMode, '/mavros/set_mode')

        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.param_set_srv = self.create_client(SetParameters,'/mavros/param/set_parameters')

        self.waypoint_pull_srv = self.create_client(WaypointPull,"/mavros/mission/pull")

        self.waypoint_set_srv = self.create_client(WaypointSetCurrent,"/mavros/mission/set_current")
        
        ############## Action Server ##################
        self.setPosition_action_server = SetPositionActionServer()
        
        ############# Action Clients ###################
        self._setposition_action_client = SetPositionActionClient()

        ############### Publishers ##############

        self.local_position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 20)
        self.pose_target_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 20)
        self.velocity_pub = self.create_publisher(TwistStamped,  '/mavros/setpoint_velocity/cmd_vel', 5)
        self.target_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 5)
        self.global_position_pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 20)

        ########## Subscribers ##################

        self.local_atual = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_callback, qos.qos_profile_sensor_data)
        self.vel_atual = self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.vel_callback, qos.qos_profile_sensor_data)
        self.state_sub =  self.create_subscription(State, '/mavros/state', self.state_callback, qos.qos_profile_sensor_data)
        self.extended_state_sub =  self.create_subscription(ExtendedState, '/mavros/extended_state', self.extended_state_callback, qos.qos_profile_sensor_data)
        self.battery_sub =  self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos.qos_profile_sensor_data)
        self.global_position_sub =  self.create_subscription(NavSatFix,'/mavros/global_position/global' , self.global_callback, qos.qos_profile_sensor_data)
        self.cam_sub = self.create_subscription(Image, self.camera_topic, self.cam_callback, qos.qos_profile_sensor_data)
        self.waypoints_sub =self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.waypoints_callback, qos.qos_profile_sensor_data)
        self.altitude_sub = self.create_subscription(Altitude, '/mavros/altitude', self.altitude_callback, qos.qos_profile_sensor_data)

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
        self.waypoint_pull_req = WaypointPull.Request()
        self.waypoint_set_req = WaypointSetCurrent.Request()
        self.get_logger().info("Services are up")
        

        while self.drone_state.mode == "":
            rclpy.spin_once(self)
        self.get_logger().info("Subscribers are up")
        #self.future = self.NOMEDOSRV.call_async(self.req)

    ########## Callback functions ###########
    
    def state_callback(self, state_data):
        self.drone_state = state_data
    
    def extended_state_callback(self, extended_state_data):
        self.drone_extended_state = extended_state_data
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

    def vel_callback(self, data):
        self.drone_vel = data
    
    def global_callback(self, global_data):
        self.global_pose = global_data
        self.global_altitude = self.global_pose.altitude - self.geoid_height(self.global_pose.latitude, self.global_pose.longitude)    

    def cam_callback(self, cam_data):
        self.cam = self.bridge_object.imgmsg_to_cv2(cam_data,"bgr8")
    
    def waypoints_callback(self, waypoints_data):
        self.received_waypoints = waypoints_data

    def altitude_callback(self, alt_data):
        self.altitude = alt_data.local
    
    ########## Battery verification ##########
    def verify_battery(self):
        percentage = self.battery.percentage
        self.get_logger().info("Waiting for battery topic...")
        while percentage == 0 and self.battery.voltage == 0:
            rclpy.spin_once(self)
        self.get_logger().info("Battery percentage is: " + str(percentage))
        self.get_logger().info("Battery voltage is: " + str(self.battery.voltage))


    ###Set mode: PX4 mode - string, timeout (seconds) - int
    def set_mode(self, mode):
        #self.get_logger().info("setting FCU mode: {0}".format(mode))
        self.set_mode_req.base_mode = 0
        self.set_mode_req.custom_mode = mode
        while not self.set_mode_srv.wait_for_service(timeout_sec=5):
            self.get_logger().info('Set mode service not available, waiting again...')
        future = self.set_mode_srv.call_async(self.set_mode_req)  # 0 is custom mode
        while future.result() == None:
            rclpy.spin_once(self)
        if not future.result().mode_sent:
            raise Exception("failed to send mode command")
            
    def set_param(self, param_value):
        self.param_set_req.parameters = [param_value]
        while not self.param_set_srv.wait_for_service(timeout_sec=5):
            self.get_logger().info('Set param service not available, waiting again...')
        self.param_set_srv.call_async(self.param_set_req)
        rclpy.spin_once(self)

    def takeoff(self, height, speed=1.5, safety_on=True):
        self.get_logger().info("Taking off...")

        alt_param_value = Parameter(name= 'MIS_TAKEOFF_ALT', value=ParameterValue(double_value=float(height), type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(alt_param_value)
        
        vel_z_param_value = Parameter(name= 'MPC_Z_VEL_ALL', value=ParameterValue(double_value=-3.0, type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(vel_z_param_value)
        speed_param_value = Parameter(name= 'MPC_TKO_SPEED', value=ParameterValue(double_value=float(speed), type=ParameterType.PARAMETER_DOUBLE))
        self.set_param(speed_param_value)
        
        rclpy.spin_once(self)
        if self.drone_extended_state.landed_state != 1:
            print(self.drone_extended_state.landed_state)
            self.get_logger().error("Drone is not grounded! Takeoff cancelled")
            return
        self.arm()
        self.set_mode("AUTO.TAKEOFF")
        
        #safety measures: locks program while taking off
        if safety_on:
            while self.drone_extended_state.landed_state != 3:
                rclpy.spin_once(self)
            while self.drone_extended_state.landed_state == 3:
                rclpy.spin_once(self)
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
            rclpy.spin_once(self)
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


    def set_position_target(self, x, y, z, yaw = None, coordinate_frame=PositionTarget.FRAME_LOCAL_NED, type_mask=0b0000101111111000):
        # https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK

        self.pose_target.type_mask = type_mask
        self.pose_target.coordinate_frame = coordinate_frame

        self.pose_target.position.x = float(x)
        self.pose_target.position.y = float(y)
        self.pose_target.position.z = float(z)

        if yaw == None:
            rclpy.spin_once(self)
            [current_roll, current_pitch, current_yaw] = euler_from_quaternion([self.drone_pose.pose.orientation.x,self.drone_pose.pose.orientation.y,self.drone_pose.pose.orientation.z,self.drone_pose.pose.orientation.w])
            self.pose_target.yaw = float(current_yaw)
        else:
            self.pose_target.yaw = float(yaw)

        while self.drone_state.mode != "OFFBOARD":
            self.local_position_pub.publish(self.pose_target)
            self.set_mode("OFFBOARD")
        self.pose_target_pub.publish(self.pose_target)

    def go_to_local(self, goal_x, goal_y, goal_z, yaw = None, coordinate_frame=PositionTarget.FRAME_LOCAL_NED, type_mask=0b0000101111111000, TOL=0.2):
        self.get_logger().info("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))
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
        while(np.sqrt((goal_x - current_x  )**2 + (goal_y - current_y)**2 + (goal_z - current_z)**2) + (yaw - current_yaw)**2) > TOL:
            rclpy.spin_once(self)
            current_x = self.drone_pose.pose.position.x
            current_y = self.drone_pose.pose.position.y
            current_z = self.drone_pose.pose.position.z
            [_,_,current_yaw] = euler_from_quaternion([self.drone_pose.pose.orientation.x,self.drone_pose.pose.orientation.y,self.drone_pose.pose.orientation.z,self.drone_pose.pose.orientation.w])
            self.set_position(goal_x, goal_y, goal_z, yaw, coordinate_frame, type_mask)
        self.get_logger().info("Arrived at requested position")

    def go_to_global(self, lat, lon, alt,yaw=0,  GLOBAL_TOL = 0.2):
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
        
    def change_altitude(self, goal_alt, ALT_TOL=0.2):
        self.get_logger().info("Setting altitude to " + str(goal_alt) + " m")
        while abs(goal_alt - self.altitude) > ALT_TOL:
            
            self.goal_pose.pose.position.x = self.drone_pose.pose.position.x
            self.goal_pose.pose.position.y = self.drone_pose.pose.position.y
            self.goal_pose.pose.position.z = float(goal_alt)
            self.local_position_pub.publish(self.goal_pose)
            rclpy.spin_once(self)
            self.get_logger().info(str(abs(goal_alt - self.altitude)))
            while self.drone_state.mode != "OFFBOARD":
                self.local_position_pub.publish(self.goal_pose)
                self.set_mode("OFFBOARD")
        
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

    def write_csv_log(self):
        now = time.time() - self.init_time
        
        log = {'Time in seconds' : now,
        'Latitude' : self.global_pose.latitude,
        'Longitude' : self.global_pose.longitude,
        'Altitude' : self.global_altitude
        }
        file = open('MissionLog.csv', 'a', newline ='')
        with file:   
            header = ['Time in seconds', 'Latitude', 'Longitude', 'Altitude']
            writer = csv.DictWriter(file, fieldnames = header)
            writer.writerow(log)
    
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
    rclpy.init(args=sys.argv)
    mav = MAV2()
    mav.set_mode("OFFBOARD")
    mav.mission_set_current_waypoint(0)
    mav.mission_start()
    while mav.mission_get_current_waypoint() < len(mav.mission_get_waypoints_list()) - 1:
        mav.get_logger().info(str(mav.mission_get_current_waypoint()))
        mav.get_logger().info(str(len(mav.mission_get_waypoints_list())-1))
        pass
    mav.mission_pause()
    mav.change_altitude(9)
    time.sleep(1)
    mav.change_altitude(6)
    mav.hold(1)
    mav.change_altitude(10)
    mav.mission_set_current_waypoint(0)
    mav.mission_start()
    


    
