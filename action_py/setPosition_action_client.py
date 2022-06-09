import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mavbase2.action import SetPosition


class SetPositionActionClient(Node):

    def __init__(self):
        super().__init__('setPositiont_action_client')
        self._action_client = ActionClient(self, SetPosition, 'setPosition')

    def send_goal(self, position_request):
        print("chegou no send_goal")
        goal_msg = SetPosition.Goal()
        goal_msg.position_request = position_request

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        print("chegouu no response")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        print("passou aqui")
        
    def get_result_callback(self, future):
        print("chegou no result")
        
        
        result = future.result().result
        while future.result() == None:
            rclpy.spin_once(self)

        self.get_logger().info('Result: '.format(result.position_result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        print("mandando feeback")
        feedback = feedback_msg.feedback
        self.get_logger().info('Received Feedback: {0}'.format(feedback.partial_position))

#receive the initial postion of the drone 
def main(args=None):
    rclpy.init(args=args)

    action_client = SetPositionActionClient()

    future = action_client.send_goal(10, 10, 3)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()