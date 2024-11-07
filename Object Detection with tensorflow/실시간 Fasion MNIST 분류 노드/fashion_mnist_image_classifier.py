import rclpy
from rclpy.node import Node
import tensorflow as tf
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 클래스 이름
class_names = ["T-shirt/top", "Trouser", "Pullover", "Dress", "Coat",
               "Sandal", "Shirt", "Sneaker", "Bag", "Ankle boot"]

class FashionMNISTImageClassifier(Node):
    def __init__(self):
        super().__init__('fashion_mnist_image_classifier')
        self.model = tf.keras.models.load_model("fashion_mnist_model.h5")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'fashion_mnist_image', self.image_callback, 10)
        self.publisher = self.create_publisher(String, 'fashion_mnist_classification', 10)
        self.get_logger().info('Fashion MNIST Image Classifier Node is ready.')

    def image_callback(self, msg):
        # Convert ROS Image message to numpy array
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        image_array = np.array(cv_image, dtype=np.float32) / 255.0  # Normalize
        image_array = np.expand_dims(image_array, axis=0)  # Add batch dimension

        # Perform prediction
        predictions = self.model.predict(image_array)
        class_index = np.argmax(predictions[0])
        class_name = class_names[class_index]

        # Publish classification result
        result_msg = String()
        result_msg.data = f"Class: {class_name}"
        self.publisher.publish(result_msg)
        self.get_logger().info(f"Published result: {class_name}")

def main(args=None):
    rclpy.init(args=args)
    node = FashionMNISTImageClassifier()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
