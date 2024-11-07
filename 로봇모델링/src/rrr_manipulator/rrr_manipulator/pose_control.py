import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PoseControlNode(Node):
    def __init__(self):
        super().__init__('pose_control_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2', 'joint3']
        self.pose_list = [
            [30, 60, 90],  # Pose 1
            [20, 80, 50],  # Pose 2
            [0, 100, 10]   # Pose 3
        ]
        self.current_pose_index = 0
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        pose = self.pose_list[self.current_pose_index]
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [angle * 3.14159 / 180 for angle in pose]  # 각도를 라디안으로 변환
        self.publisher_.publish(self.joint_state)

        # 다음 포즈로 이동
        self.current_pose_index = (self.current_pose_index + 1) % len(self.pose_list)

def main(args=None):
    rclpy.init(args=args)
    node = PoseControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
