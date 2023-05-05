#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from outdooros.srv import GetNodeParameters, GetNodeParametersResponse
from outdooros.srv import LoadMap, LoadMapResponse, SaveMap, SaveMapResponse
from nav_msgs.srv import GetMap
import subprocess
import select
import os
import fcntl

def load_map(req):
    file_path = os.path.expanduser(req.file_path)
    topic = req.topic
    try:
        process = subprocess.Popen(["rosrun", "map_server", "map_server", file_path, "map:=" + topic, "__name:=outdooros_map_server"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        flags = fcntl.fcntl(process.stdout, fcntl.F_GETFL)
        fcntl.fcntl(process.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        # Wait for it to either fail or not
        rospy.sleep(1)

        # Check if the process is still running
        if process.poll() is not None:
            # Process terminated, read the error output
            error_output = process.stdout.read().decode('utf-8')
            return LoadMapResponse(success=False, message="Map server failed to load the map: " + error_output)

        return LoadMapResponse(success=True, message="Map loaded successfully")
    except Exception as e:
        return LoadMapResponse(success=False, message=str(e))

def save_map(req):
	file_path = os.path.expanduser(req.file_path)
	topic = req.topic
	try:
		process = subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", file_path, "map:=" + topic, "__name:=outdooros_map_saver"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		flags = fcntl.fcntl(process.stdout, fcntl.F_GETFL)
		fcntl.fcntl(process.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)

		while True:
			# Check if the process is still running
			if process.poll() is not None:
				break

			while True:
				try:
					line = process.stdout.readline()
					if not line:
						break
					
					if b"[ERROR]" in line:
						process.terminate()
						return SaveMapResponse(success=False, message="Map saver failed to save the map: " + line.decode('utf-8'))
				except IOError:
					break

			# Sleep for a short period of time to avoid excessive CPU usage
			rospy.sleep(0.2)

		return SaveMapResponse(success=True, message="Map saved successfully")
	except Exception as e:
		return SaveMapResponse(success=False, message=str(e))

def get_dynamic_reconfigure_nodes(req):
	list_nodes_output = subprocess.check_output(["rosrun", "dynamic_reconfigure", "dynparam", "list"]).decode('utf-8')
	nodes_list = list_nodes_output.strip().split("\n")

	response = TriggerResponse()
	response.success = True
	response.message = "\n".join(nodes_list)

	return response

def get_node_parameters(req):
	node_params = subprocess.check_output(["rosrun", "dynamic_reconfigure", "dynparam", "get", req.node]).decode('utf-8')

	response = GetNodeParametersResponse()
	response.parameters = node_params

	return response

rospy.init_node('outdooros_service_handler', anonymous=True)
get_nodes_service = rospy.Service('outdooros/get_dynamic_reconfigure_nodes', Trigger, get_dynamic_reconfigure_nodes)
get_node_parameters_service = rospy.Service('outdooros/get_node_parameters', GetNodeParameters, get_node_parameters)
load_map_service = rospy.Service('outdooros/load_map', LoadMap, load_map)
save_map_service = rospy.Service('outdooros/save_map', SaveMap, save_map)

print("Service handler ready.")
rospy.spin()