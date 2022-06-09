import math
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from mavbase2.action import SetPosition
TOL = 0.3

class SetPositionActionServer(Node):

    def __init__(self):
        super().__init__('setPosition_action_server')
        self._action_server = ActionServer(self, SetPosition, 'setPosition', self.execute_callback)

    
    # Need to get the initial position from MAV2 #
    def execute_callback(self, goal_handle, drone_position, speed):
        self.get_logger().info('Executing positioning..')
        
        feedback_msg = SetPosition.Feedback()
        feedback_msg.partial_position = drone_position
        

        angle = math.atan2((goal_handle.request.position_resquest[1]- drone_position[1]), (goal_handle.request.position_resquest[0]- drone_position[0]))
        
        while (math.sqrt((goal_handle.request.position_resquest[1]- drone_position[1])**2 + (goal_handle.request.position_resquest[0]- drone_position[0])**2) >= TOL):
           drone_position[1] += speed*math.cos(angle); 
           drone_position[0] += speed*math.sin(angle); 
           feedback_msg.partial_position = drone_position
           self.get_logger().info('Feedback: '.format(feedback_msg.partial_position))
           goal_handle.publish_feedback(feedback_msg)
           time.sleep(1)

        goal_handle.succeed()
        
        result = SetPosition.Result()
        result.position_result = feedback_msg.partial_position
        return result


def main(args=None):
    rclpy.init(args=args)

    setPositionaction_server = SetPositionActionServer()

    rclpy.spin(setPositionaction_server)


if __name__ == '__main__':
    main()