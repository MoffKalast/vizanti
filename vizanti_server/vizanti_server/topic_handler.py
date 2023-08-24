
from tf2_msgs.msg import TFMessage
import threading
import rclpy
from rclpy.node import Node

class TopicHandler(Node):
    def __init__(self):
        super().__init__("vizanti_topic_handler")

        self.updated = False
        self.lock = threading.Lock()
        self.transforms = {}

        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.tf_pub = self.create_publisher(TFMessage, '/vizanti/tf_consolidated', 10)

        self.get_logger().info("TF handler ready.")

    def publish(self):
        if not self.updated:
            return

        msg = TFMessage()
        with self.lock:
            msg.transforms = list(self.transforms.values())
            self.updated = False

        self.tf_pub.publish(msg)

    def tf_callback(self, msg):
        with self.lock:
            for transform in msg.transforms:
                self.transforms[transform.child_frame_id] = transform
            self.updated = True

def spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    odr = TopicHandler()

    spinner = threading.Thread(target=spin, args=(odr,))
    spinner.start()

    rate = odr.create_rate(30)

    while rclpy.ok():
        odr.publish()
        rate.sleep()

    odr.destroy_node()
    rclpy.shutdown()
    spinner.join()