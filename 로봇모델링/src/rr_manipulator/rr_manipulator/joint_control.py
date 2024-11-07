import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2']
        self.joint_state.position = [0.0, 0.0]  # 초기 조인트 값

    def timer_callback(self):
        # 각 조인트에 대한 각도를 계산하는 알고리즘 (예시: 사인 함수로 움직임)
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position[0] = math.sin(self.get_clock().now().seconds_nanoseconds()[0] * 0.5)  # joint1
        self.joint_state.position[1] = math.sin(self.get_clock().now().seconds_nanoseconds()[0] * 0.3)  # joint2
        self.publisher_.publish(self.joint_state)

    def ptp_move(self, target_positions):
        """각 조인트를 목표 위치로 이동시키는 함수"""
        current_positions = list(self.joint_state.position)
        step_size = 0.01  # 이동 단계
        for i in range(len(current_positions)):
            if current_positions[i] < target_positions[i]:
                current_positions[i] += step_size
            elif current_positions[i] > target_positions[i]:
                current_positions[i] -= step_size

        self.joint_state.position = current_positions
        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
