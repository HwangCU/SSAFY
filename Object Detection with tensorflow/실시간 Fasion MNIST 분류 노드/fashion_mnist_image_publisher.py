import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import tensorflow as tf

class FashionMNISTImagePublisher(Node):
    def __init__(self):
        super().__init__('fashion_mnist_image_publisher')
        self.publisher = self.create_publisher(Image, 'fashion_mnist_image', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()

        # Load Fashion MNIST dataset
        (_, _), (test_images, _) = tf.keras.datasets.fashion_mnist.load_data()
        self.test_images = test_images
        self.index = 0
        self.get_logger().info('Fashion MNIST Image Publisher Node is ready.')

    def timer_callback(self):
        # Get the next image and prepare it
        image = self.test_images[self.index]
        self.index = (self.index + 1) % len(self.test_images)  # Loop over images
        image_msg = self.bridge.cv2_to_imgmsg(image.astype(np.uint8), encoding='mono8')

        # Publish the image
        self.publisher.publish(image_msg)
        self.get_logger().info('Published an image.')

def main(args=None):
    rclpy.init(args=args)
    node = FashionMNISTImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
