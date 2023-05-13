#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from outdooros.srv import GetNodeParameters, GetNodeParametersResponse, LoadMap, LoadMapResponse, SaveMap, SaveMapResponse, RecordRosbag, RecordRosbagResponse
from nav_msgs.srv import GetMap
import subprocess
import os
import fcntl

class ServiceHandler:

	def __init__(self):
		rospy.init_node('outdooros_service_handler')

		self.get_nodes_service = rospy.Service('outdooros/get_dynamic_reconfigure_nodes', Trigger,self. get_dynamic_reconfigure_nodes)
		self.get_node_parameters_service = rospy.Service('outdooros/get_node_parameters', GetNodeParameters, self.get_node_parameters)
		self.load_map_service = rospy.Service('outdooros/load_map', LoadMap, self.load_map)
		self.save_map_service = rospy.Service('outdooros/save_map', SaveMap, self.save_map)
		self.record_setup_service = rospy.Service('outdooros/bag/setup', RecordRosbag, self.recording_setup)
		self.record_status_service = rospy.Service('outdooros/bag/status', Trigger, self.recording_status)

		self.proc = None

		rospy.loginfo("Service handler ready.")

	def load_map(self, req):
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

	def save_map(self, req):
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

	def get_dynamic_reconfigure_nodes(self, req):
		list_nodes_output = subprocess.check_output(["rosrun", "dynamic_reconfigure", "dynparam", "list"]).decode('utf-8')
		nodes_list = list_nodes_output.strip().split("\n")

		response = TriggerResponse()
		response.success = True
		response.message = "\n".join(nodes_list)

		return response

	def get_node_parameters(self, req):
		node_params = subprocess.check_output(["rosrun", "dynamic_reconfigure", "dynparam", "get", req.node]).decode('utf-8')

		response = GetNodeParametersResponse()
		response.parameters = node_params

		return response

	def recording_status(self, msg):
		response = TriggerResponse()
		response.success = self.proc is not None

		if response.success:
			response.message = "Bag recording in progress..."
		else:
			response.message = "Bag recorder idle."

		return response


	def recording_setup(self, req):

		response = RecordRosbagResponse()

		if req.start:
			if self.proc is not None:
				response.success = False
				response.message = "Already recording, please stop the current recording first."
			else:
				command = ['rosbag', 'record', '-O']

				# Expand and add the path to the command
				expanded_path = os.path.expanduser(req.path)
				command.append(expanded_path)

				# Add the topics to the command
				for topic in req.topics:
					command.append(topic)

				# Use subprocess to start rosbag record in a new process
				self.proc = subprocess.Popen(command)

				response.success = True
				response.message = "Started recording rosbag with PID " + str(self.proc.pid)
		else:
			if self.proc is None:
				response.success = False
				response.message = "No recording to stop."
			else:
				self.proc.terminate()
				self.proc = None

				response.success = True
				response.message = "Stopped recording rosbag."

		return response


handler = ServiceHandler()
rospy.spin()