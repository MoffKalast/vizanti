#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from outdooros.srv import GetNodeParameters, GetNodeParametersResponse
import subprocess

def get_dynamic_reconfigure_nodes_cb(req):
	list_nodes_output = subprocess.check_output(["rosrun", "dynamic_reconfigure", "dynparam", "list"]).decode('utf-8')
	nodes_list = list_nodes_output.strip().split("\n")

	response = TriggerResponse()
	response.success = True
	response.message = "\n".join(nodes_list)

	return response

def get_node_parameters_cb(req):
	node_params = subprocess.check_output(["rosrun", "dynamic_reconfigure", "dynparam", "get", req.node]).decode('utf-8')

	response = GetNodeParametersResponse()
	response.parameters = node_params

	return response

rospy.init_node('outdooros_service_handler', anonymous=True)
get_nodes_service = rospy.Service('outdooros/get_dynamic_reconfigure_nodes', Trigger, get_dynamic_reconfigure_nodes_cb)
get_node_parameters_service = rospy.Service('outdooros/get_node_parameters', GetNodeParameters, get_node_parameters_cb)

print("Service handler ready.")
rospy.spin()