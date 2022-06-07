import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mavbase2.action import SetPosition


class SetPositionActionClient(Node):

    def __init__(self):
        super().__init__('setPositiont_action_client')
        self._action_client = ActionClient(self, SetPosition, 'setPositiont')

    def send_goal(self, position_request):
        goal_msg = SetPosition.Goal()
        goal_msg.position_request = position_request

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '.format(result.position_result))
        rclpy.shutdown()


#receive the initial postion of the drone 
def main(args=None):
    rclpy.init(args=args)

    action_client = SetPositionActionClient()

    future = action_client.send_goal(10, 10, 3)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()