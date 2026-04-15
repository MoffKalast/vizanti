#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool

class TestServiceNode:
    def __init__(self):
        rospy.init_node('test_service_node')

        # Initialize toggle states
        self.empty_state = False
        self.trigger_state = False
        self.setbool_state = False

        # Publishers with latch=True
        self.empty_pub = rospy.Publisher('/empty_triggered', Bool, latch=True, queue_size=1)
        self.trigger_pub = rospy.Publisher('/trigger_triggered', Bool, latch=True, queue_size=1)
        self.setbool_pub = rospy.Publisher('/setbool_triggered', Bool, latch=True, queue_size=1)

        # Initial publish so topics have a latched message
        self.empty_pub.publish(self.empty_state)
        self.trigger_pub.publish(self.trigger_state)
        self.setbool_pub.publish(self.setbool_state)

        # Service servers
        self.empty_srv = rospy.Service('/test_empty', Empty, self.handle_empty)
        self.trigger_srv = rospy.Service('/test_trigger', Trigger, self.handle_trigger)
        self.setbool_srv = rospy.Service('/test_setbool', SetBool, self.handle_setbool)

        rospy.loginfo("Test service node ready.")
        rospy.spin()

    def handle_empty(self, req):
        self.empty_state = not self.empty_state
        self.empty_pub.publish(self.empty_state)
        rospy.loginfo("Empty service triggered, state: %s", self.empty_state)
        return EmptyResponse()

    def handle_trigger(self, req):
        self.trigger_state = not self.trigger_state
        self.trigger_pub.publish(self.trigger_state)
        rospy.loginfo("Trigger service triggered, state: %s", self.trigger_state)
        return TriggerResponse(success=True, message="Trigger service called")

    def handle_setbool(self, req):
        self.setbool_state = req.data
        self.setbool_pub.publish(self.setbool_state)
        rospy.loginfo("SetBool service triggered, state: %s", self.setbool_state)
        return SetBoolResponse(success=True, message="SetBool service called")

if __name__ == '__main__':
    try:
        TestServiceNode()
    except rospy.ROSInterruptException:
        pass
 
