#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty as EmptySrv, Trigger, SetBool
from std_msgs.msg import Bool, Empty as EmptyMsg
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class TestServiceNode(Node):
    def __init__(self):
        super().__init__('test_button_service_node')

        # For visualization
        self.empty_state = False
        self.trigger_state = False
        self.setbool_state = False
        self.empty_topic_state = False

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # Ensures latched-like behavior

        self.emptysrv_pub = self.create_publisher(Bool, '/empty_srv_triggered', qos_profile)
        self.emptymsg_pub = self.create_publisher(Bool, '/empty_msg_triggered', qos_profile)
        self.trigger_pub = self.create_publisher(Bool, '/trigger_triggered', qos_profile)
        self.setbool_pub = self.create_publisher(Bool, '/setbool_triggered', qos_profile)

        self.emptysrv_pub.publish(Bool(data=self.empty_state))
        self.emptymsg_pub.publish(Bool(data=self.empty_state))
        self.trigger_pub.publish(Bool(data=self.trigger_state))
        self.setbool_pub.publish(Bool(data=self.setbool_state))

        self.create_service(EmptySrv, '/test_emptysrv', self.handle_empty)
        self.create_service(Trigger, '/test_trigger', self.handle_trigger)
        self.create_service(SetBool, '/test_setbool', self.handle_setbool)

        self.create_subscription(EmptyMsg, '/test_emptymsg', self.handle_empty_topic, 10)

        self.get_logger().info('Test service node ready.')

    def handle_empty(self, request, response):
        self.empty_state = not self.empty_state
        self.emptysrv_pub.publish(Bool(data=self.empty_state))
        self.get_logger().info(f'Empty service triggered, state: {self.empty_state}')
        return response

    def handle_trigger(self, request, response):
        self.trigger_state = not self.trigger_state
        self.trigger_pub.publish(Bool(data=self.trigger_state))
        self.get_logger().info(f'Trigger service triggered, state: {self.trigger_state}')
        response.success = True
        response.message = 'Trigger service called'
        return response

    def handle_setbool(self, request, response):
        self.setbool_state = request.data
        self.setbool_pub.publish(Bool(data=self.setbool_state))
        self.get_logger().info(f'SetBool service triggered, state: {self.setbool_state}')
        response.success = True
        response.message = 'SetBool service called'
        return response

    def handle_empty_topic(self, msg):
        self.empty_topic_state = not self.empty_topic_state
        self.emptymsg_pub.publish(Bool(data=self.empty_topic_state))
        self.get_logger().info(f'Empty topic message received, state: {self.empty_topic_state}')



def main(args=None):
    rclpy.init(args=args)
    node = TestServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
