from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

class PTP_MOVE(Node):

    def __init__(self):
        super().__init__('dobot_PTP_client')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self.suction_client = self.create_client(SuctionCupControl, '/dobot_suction_cup_service')
        while not self.suction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SuctionCupControl.Request()

    def turn_suction(self, on_off):
        self.req.enable_suction = on_off
        future = self.suction_client.call_async(self.req)
        future.add_done_callback(self.callback)
    
    def callback(self, future):
        response = future.result()
        self.get_logger().info(f"start Suction {response}")

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

        # Start a 0.5 second timer
        # self._timer = self.create_timer(0.5, self.timer_callback)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.current_pose))

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

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


def main(args=None):
    rclpy.init(args=args)

    action_client = PTP_MOVE()
    action_client.turn_suction(False)
    num = ''
    while num != '0':

        num = input("Enter the number:")
        if num == '1': 
            print(1)
            action_client.send_goal(target = [155.0, -136.0, 18.0, 0.0], mode = 1)
            time.sleep(2)
            action_client.turn_suction(True)
        elif num == '2':
            print(2)
            action_client.send_goal(target = [200.0, 0.0, 18.0, 0.0], mode = 1)
            time.sleep(2)
        elif num == '3':
            print(3)
            action_client.send_goal(target = [158.0, 123.0, 18.0, 0.0], mode = 1)
            time.sleep(2)
            action_client.turn_suction(False)
        else:
            print("Enter the right number")
        # action_client.send_goal(target = [200.0, 0.0, 100.0, 0.0], mode = 1)
        # time.sleep(2)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
