#!/usr/bin/env python3

import subprocess
import os
import fcntl
import sys
import rospy
import json
import dynamic_reconfigure

from dynamic_reconfigure.client import Client
from std_srvs.srv import Trigger, TriggerResponse
from vizanti.srv import GetNodeParameters, GetNodeParametersResponse
from vizanti.srv import LoadMap, LoadMapResponse, SaveMap, SaveMapResponse
from vizanti.srv import RecordRosbag, RecordRosbagResponse
from vizanti.srv import ManageNode, ListPackages, ListExecutables, ListExecutablesResponse, ListPackagesResponse

class ServiceHandler:

	def __init__(self):
		rospy.init_node('vizanti_service_handler')

		self.proc = None
		self.packages = self.get_packages()

		self.get_nodes_service = rospy.Service('vizanti/get_dynamic_reconfigure_nodes', Trigger,self. get_dynamic_reconfigure_nodes)
		self.get_node_parameters_service = rospy.Service('vizanti/get_node_parameters', GetNodeParameters, self.get_node_parameters)
		self.get_node_parameter_info_service = rospy.Service('vizanti/get_node_parameter_info', GetNodeParameters, self.get_node_parameter_info)

		self.load_map_service = rospy.Service('vizanti/load_map', LoadMap, self.load_map)
		self.save_map_service = rospy.Service('vizanti/save_map', SaveMap, self.save_map)

		self.record_setup_service = rospy.Service('vizanti/bag/setup', RecordRosbag, self.recording_setup)
		self.record_status_service = rospy.Service('vizanti/bag/status', Trigger, self.recording_status)

		self.node_kill_service = rospy.Service('vizanti/node/kill', ManageNode, self.node_kill)
		self.node_start_service = rospy.Service('vizanti/node/start', ManageNode, self.node_start)
		self.node_info_service = rospy.Service('vizanti/node/info', ManageNode, self.node_info)
		self.node_info_service = rospy.Service('vizanti/roswtf', Trigger, self.roswtf)

		self.list_packages_service = rospy.Service('vizanti/list_packages', ListPackages ,self.list_packages_callback)
		self.list_executables_service = rospy.Service('vizanti/list_executables', ListExecutables ,self.list_executables_callback)

		rospy.loginfo("Service handler ready.")

	def get_packages(self):
		# Use rospack to get list of packages
		cmd = ["rospack", "list"]
		process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
		output, error = process.communicate()
		# Process output
		lines = output.decode('utf-8').split('\n')
		packages = {line.split()[0]: line.split()[1] for line in lines if line}
		return packages

	def list_packages_callback(self, req):
		return ListPackagesResponse(self.packages.keys())
	
	def list_executables_callback(self, req):
		if req.package not in self.packages:
			rospy.logerr("Package not found: " + req.package)
			return ListExecutablesResponse([])

		path = self.packages[req.package]
		# Use find to get list of executables
		cmd_exec = ["find", path, "-type", "f", "!", "-name", "*.*", "-executable"]
		cmd_python_and_launch = ["find", path, "-type", "f", 
								"(",
								"-iname", "*.py", "-executable",
								"-o",
								"-iname", "*.launch",
								")"]
								
		process_exec = subprocess.Popen(cmd_exec, stdout=subprocess.PIPE)
		process_python_and_launch = subprocess.Popen(cmd_python_and_launch, stdout=subprocess.PIPE)
		
		output_exec, error_exec = process_exec.communicate()
		output_python_and_launch, error_python_and_launch = process_python_and_launch.communicate()
		
		# Process output
		lines_exec = output_exec.decode('utf-8').split('\n')
		lines_python_and_launch = output_python_and_launch.decode('utf-8').split('\n')
		
		executables = [line.split("/")[-1] for line in lines_exec if line]
		python_and_launch_files = [line.split("/")[-1] for line in lines_python_and_launch if line]
		
		return ListExecutablesResponse(executables + python_and_launch_files)

	def node_kill(self, req):
		try:
			subprocess.call(['rosnode', 'kill', req.node])
			return {'success': True, 'message': f'Killed node {req.node}'}
		except Exception as e:
			return {'success': False, 'message': str(e)}

	def node_start(self, req):
		try:
			args = req.node.split(" ")

			# Open /dev/null
			devnull = open(os.devnull, 'w')

			# Set up the process to ignore the SIGTERM signal
			def preexec():
				os.setpgrp()
				sys.stdin = open(os.devnull, 'r')
				sys.stdout = open(os.devnull, 'w')
				sys.stderr = open(os.devnull, 'w')

			subprocess.Popen(args, stdout=devnull, stderr=devnull, preexec_fn=preexec)
			
			return {'success': True, 'message': f'Started node {req.node}'}
		except Exception as e:
			return {'success': False, 'message': str(e)}
		
	def node_info(self, req):
		try:
			rosinfo = subprocess.check_output(["rosnode", "info", req.node]).decode('utf-8')
			rosinfo = rosinfo.replace("--------------------------------------------------------------------------------","")
			return {'success': True, 'message': rosinfo}
		except Exception as e:
			return {'success': False, 'message': str(e)}
		
	def roswtf(self, req):
		try:
			rosinfo = subprocess.check_output(["roswtf"]).decode('utf-8')
			return {'success': True, 'message': rosinfo}
		except Exception as e:
			return {'success': False, 'message': str(e)}
		

	def load_map(self, req):
		file_path = os.path.expanduser(req.file_path)
		topic = req.topic
		try:
			process = subprocess.Popen(["rosrun", "map_server", "map_server", file_path, "map:=" + topic, "__name:=vizanti_map_server"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
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
			process = subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", file_path, "map:=" + topic, "__name:=vizanti_map_saver"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
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
		nodes_list = dynamic_reconfigure.find_reconfigure_services()
		response = TriggerResponse()
		response.success = True
		response.message = json.dumps(nodes_list)
		return response

	def get_node_parameter_info(self, req):
		response = GetNodeParametersResponse()

		try:
			client = Client(req.node, timeout=2.0)
			param_desc = client.get_parameter_descriptions(timeout=2.0)

			param_info = {}
			for p in param_desc:
				name = p["name"]
				param_info[name] = {}
				param_info[name]["type"] = p["type"]
				param_info[name]["min"] = p["min"]
				param_info[name]["max"] = p["max"]
				param_info[name]["description"] = p["description"]

				#is it an enum?
				if p["edit_method"] != "":
					parsed = json.loads(p["edit_method"].replace("\'","\""))
					enum_options = []
					for e in parsed["enum"]:
						vals = {}
						vals["value"] =  e["value"]
						vals["description"] =  e["description"]
						enum_options.append(vals)

					param_info[name]["enum"] = enum_options

			response.parameters = json.dumps(param_info)
		except Exception as e:
			rospy.logerr(f"Failed to fetch parameter info from {req.node}: {e}")
			response.parameters = "[]"

		return response

	def get_node_parameters(self, req):
		response = GetNodeParametersResponse()

		try:
			client = Client(req.node, timeout=2.0)
			config = client.get_configuration(timeout=2.0)

			param_list = []
			for name, value in config.items():
				# Filter out internal dynamic_reconfigure keys
				if name.startswith('__'):
					continue

				value_type = type(value).__name__
				if value_type == "Config":
					continue
				
				param_list.append([name, value])

			response.parameters = json.dumps(param_list)
		except Exception as e:
			rospy.logerr(f"Failed to fetch parameters from {req.node}: {e}")
			response.parameters = "[]"

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
				response.message = "Started recording rosbag with PID " + str(self.proc.pid) +" to path "+ str(expanded_path)
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