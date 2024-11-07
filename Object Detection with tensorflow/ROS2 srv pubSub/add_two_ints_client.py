import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for AddTwoInts service...')
        self.get_logger().info('AddTwoInts Client is ready.')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info(f'Result: {self.future.result().sum}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()

    # Get integers from command-line arguments
    if len(sys.argv) != 3:
        node.get_logger().error('Usage: ros2 run problem6 add_two_ints_client <int a> <int b>')
        return

    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
        node.send_request(a, b)
    except ValueError:
        node.get_logger().error('Invalid input. Please enter integers only.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
