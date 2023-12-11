
import threading
import rclpy
import time

from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.duration import Duration

class TopicHandler(Node):
    def __init__(self):
        super().__init__("vizanti_topic_handler")

        self.updated = False
        self.lock = threading.Lock()
        self.transforms = {}

        self.transform_timeout = {}
        self.timeout_prescaler = 0 # once per second

        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.tf_pub = self.create_publisher(
            TFMessage, 
            '/vizanti/tf_consolidated',
            QoSProfile(
                depth=5,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                lifespan=Duration(seconds=0.066) #66ms
            )
        )

        self.get_logger().info("TF handler ready.")

    def clear_old_tfs(self):
        with self.lock:
            for key in list(self.transforms.keys()):
                if time.time() - self.transform_timeout[key] > 10.0:
                    parent = self.transforms[key].header.frame_id
                    del self.transforms[key]
                    del self.transform_timeout[key]
                    self.updated = True
                    self.get_logger().warn("Deleted old TF link: "+str(parent)+" -> "+str(key))

    def publish(self):
        # once per 5 seconds
        self.timeout_prescaler = (self.timeout_prescaler + 1) % 150
        if self.timeout_prescaler == 0:
            self.clear_old_tfs()

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
                self.transform_timeout[transform.child_frame_id] = time.time()
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