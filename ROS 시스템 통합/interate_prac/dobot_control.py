from dobot_msgs.action import PointToPoint
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32  # Integer 값을 구독하기 위해 추가
import time

class PTP_MOVE(Node):

    def __init__(self):
        super().__init__('dobot_PTP_client')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self.subscription = self.create_subscription(
            Int32,
            'target_location',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # 위치 정의
        self.locations = {
            1: [200.0, -200.0, 100.0, 0.0],
            2: [200.0, 0.0, 100.0, 0.0],
            3: [200.0, 200.0, 100.0, 0.0]
        }

    def cancel_done(self, future): 
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.current_pose))

    def send_goal(self, target, mode):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = target
        goal_msg.motion_type = mode
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def listener_callback(self, msg):
        target_number = msg.data
        if target_number in self.locations:
            self.get_logger().info(f'Received target number: {target_number}')
            target_pose = self.locations[target_number]
            self.send_goal(target=target_pose, mode=1)
        else:
            self.get_logger().info(f'Invalid target number received: {target_number}')

def main(args=None):
    rclpy.init(args=args)

    action_client = PTP_MOVE()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
