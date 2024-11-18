#!/usr/bin/env python3

import subprocess
import os
import fcntl
import sys
import rclpy
import json
import time

from rclpy.node import Node

from rclpy.executors import  MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from ros2lifecycle.api import get_node_names
from ros2pkg.api import get_package_names, get_prefix_path
from rqt_reconfigure_param_api import create_param_client
from rclpy.parameter import Parameter

from std_srvs.srv import Trigger
from vizanti_msgs.srv import GetNodeParameters, SetNodeParameter
from vizanti_msgs.srv import LoadMap, SaveMap
from vizanti_msgs.srv import RecordRosbag
from vizanti_msgs.srv import ManageNode, ListPackages, ListExecutables, ListLifecycles

from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State

class ServiceHandler(Node):
    def __init__(self, group):
        super().__init__("vizanti_service_handler")

        self.proc = None
        self.packages = sorted(list(get_package_names()))

        self.get_node_parameters_service = self.create_service(GetNodeParameters, 'vizanti/get_node_parameters', self.get_node_parameters, callback_group=group)
        self.set_node_parameter_service = self.create_service(SetNodeParameter, 'vizanti/set_node_parameter', self.set_node_parameter, callback_group=group)

        self.load_map_service = self.create_service(LoadMap, 'vizanti/load_map', self.load_map, callback_group=group)
        self.save_map_service = self.create_service(SaveMap, 'vizanti/save_map', self.save_map, callback_group=group)

        self.record_setup_service = self.create_service(RecordRosbag, 'vizanti/bag/setup', self.recording_setup, callback_group=group)

        self.kill_service = self.create_service(ManageNode, 'vizanti/node/kill', self.node_kill, callback_group=group)
        self.start_service = self.create_service(ManageNode, 'vizanti/node/start', self.node_start, callback_group=group)
        self.info_service = self.create_service(ManageNode, 'vizanti/node/info', self.node_info, callback_group=group)
        self.wtf_service = self.create_service(Trigger, 'vizanti/roswtf', self.roswtf, callback_group=group)
        self.record_status_service = self.create_service(Trigger, 'vizanti/bag/status', self.recording_status, callback_group=group)
        self.list_lifecycle_service = self.create_service(ListLifecycles, 'vizanti/list_lifecycle_nodes', self.list_lifecycle_nodes_status, callback_group=group)

        self.list_packages_service = self.create_service(ListPackages, 'vizanti/list_packages', self.list_packages_callback, callback_group=group)
        self.list_executables_service = self.create_service(ListExecutables, 'vizanti/list_executables', self.list_executables_callback, callback_group=group)

        self.get_logger().info("Service handler ready.")

    def list_lifecycle_nodes_status(self, req, res):
        node_names = get_node_names(node=self, include_hidden_nodes=True)
        fullnames = [n.full_name for n in node_names]
        state_ids = [0] * len(fullnames)

        for i in range(len(fullnames)):
            try:
                client = self.create_client(GetState,f'{fullnames[i]}/get_state')
                response = client.call(GetState.Request())
                state_ids[i] = response.current_state.id
            except:
                pass

        res.nodes = fullnames
        res.states = state_ids
        return res

    def list_packages_callback(self, req, res):
        res.packages = self.packages
        return res

    def get_filenames(self, file_paths):
        file_names = []
        for file_path in file_paths:
            base_name = os.path.basename(file_path)
            if base_name.endswith(tuple([".py",".launch",".yaml"])) or "." not in base_name:
                file_names.append(base_name)
        return file_names

    def list_executables_callback(self, req, res):

        if req.package not in self.packages:
            self.get_logger().error("Package not found: " + req.package)
            res.executables = []
            return res
        
        path = get_prefix_path(req.package)

        #TODO add path to apt installed packages, I'm not sure where exactly those executables are yet
        #self.get_logger().info(f"libpath: {libpath}")

        cmd_exec = ["find", path+"/share/"+req.package] # get list of executables
        cmd_exec = cmd_exec + ["-type", "f", "-o", "-type", "l"] # files or symlinks

        cmd_launch = ["find", path+"/lib/"+req.package]
        cmd_launch = cmd_launch + ["-type", "f", "-o", "-type", "l"]
        process_exec = subprocess.Popen(cmd_exec, stdout=subprocess.PIPE)
        process_launch = subprocess.Popen(cmd_launch, stdout=subprocess.PIPE)

        output_exec, _ = process_exec.communicate()
        output_launch, _ = process_launch.communicate()

        # Process output
        lines_exec = self.get_filenames(output_exec.decode('utf-8').split('\n'))
        lines_launch = self.get_filenames(output_launch.decode('utf-8').split('\n'))

        self.get_logger().info(f"lines_exec: {lines_exec}")
        self.get_logger().info(f"output_python_and_launch: {lines_launch}")

        executables = [line.split("/")[-1] for line in lines_exec if line]
        launch_files = [line.split("/")[-1] for line in lines_launch if line]

        res.executables = executables + launch_files
        return res

    def node_kill(self, req, res):
        try:
            #ros 2 doesn't let you kill nodes in a legit way, so we have to be extra janky lol
            #this seems to also have the weird side effect that it takes a year for ros2 node list to show the change
            self.get_logger().info("Attempting to kill node "+str(req.node))
            subprocess.call("ps aux | grep '"+req.node+"' | awk '{print $2}' | xargs kill -9", shell=True)
            res.success = True
            res.message = f'Killed node {req.node}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def node_start(self, req, res):
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

            self.get_logger().info("Starting node "+str(req.node))

            res.success = True
            res.message = f'Started node {req.node}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def node_info(self, req, res):
        try:
            rosinfo = subprocess.check_output(["ros2", "node", "info", req.node]).decode('utf-8')
            rosinfo = rosinfo.replace("--------------------------------------------------------------------------------", "")
            res.success = True
            res.message = rosinfo
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def roswtf(self, req, res):
        try:
            self.get_logger().info("Compiling doctor report...")
            rosinfo = subprocess.check_output(["ros2", "doctor", "--report"]).decode('utf-8')
            res.success = True
            res.message = rosinfo
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def load_map(self, req, res):
        file_path = os.path.expanduser(req.file_path)
        topic = req.topic
        try:
            process = subprocess.Popen(["ros2", "run", "nav2_map_server", "map_server", file_path, "map:=" + topic, "__name:=vizanti_map_server"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            flags = fcntl.fcntl(process.stdout, fcntl.F_GETFL)
            fcntl.fcntl(process.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)

            # Wait for it to either fail or not
            time.sleep(1)

            # Check if the process is still running
            if process.poll() is not None:
                # Process terminated, read the error output
                error_output = process.stdout.read().decode('utf-8')
                res.success = False
                res.message = "Map server failed to load the map: " + error_output
            else:
                res.success = True
                res.message = "Map loaded successfully"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def save_map(self, req, res):
        file_path = os.path.expanduser(req.file_path)
        topic = req.topic
        try:
            process = subprocess.Popen(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", file_path, "--ros-args", "map:=" + topic], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
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
                            res.success = False
                            res.message = "Map saver failed to save the map: " + line.decode('utf-8')
                            return res
                    except IOError:
                        break

                # Sleep for a short period of time to avoid excessive CPU usage
                time.sleep(0.2)

            res.success = True
            res.message = "Map saved successfully"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def get_node_parameters(self, req, res):
        try:
            param_client = create_param_client(self, req.node)
            param_names = param_client.list_parameters()
            descriptors = param_client.describe_parameters(param_names)
            #Parameter.Type Enum
            #  <Type.NOT_SET: 0>,
            #  <Type.BOOL: 1>,
            #  <Type.INTEGER: 2>,
            #  <Type.DOUBLE: 3>,
            #  <Type.STRING: 4>,
            #  <Type.BYTE_ARRAY: 5>,
            #  <Type.BOOL_ARRAY: 6>,
            #  <Type.INTEGER_ARRAY: 7>,
            #  <Type.DOUBLE_ARRAY: 8>,
            #  <Type.STRING_ARRAY: 9>

            parameters = param_client.get_parameters(param_names)
            param_list = []
            for param, descriptor in zip(parameters, descriptors):
                if descriptor.type > 0 and descriptor.type < 5: #TODO add support for the rest if anyone actually uses them
                    param_list.append([param.name, param.value, descriptor.type])
            res.parameters = json.dumps(param_list)            
        except Exception as e:
            res.parameters = "[]"
            print(f"Failed to fetch parameters from node: {e}")
        return res
    
    def set_node_parameter(self, req, res):
        try:
            param_client = create_param_client(self, req.node)
            descriptors = param_client.describe_parameters([req.param])
            param_type = Parameter.Type(descriptors[0].type)

            value = req.value
            if param_type == Parameter.Type.BOOL:
                value = bool(value)
            elif param_type == Parameter.Type.INTEGER:
                value = int(value)
            elif param_type == Parameter.Type.DOUBLE:
                value = float(value)
            """elif param_type == Parameter.Type.BYTE_ARRAY:
                value = int(value)
            elif param_type == Parameter.Type.BOOL_ARRAY:
                value = int(value)
            elif param_type == Parameter.Type.DOUBLE_ARRAY:
                value = int(value)
            elif param_type == Parameter.Type.STRING_ARRAY:
                value = int(value)"""

            parameter = Parameter(name=req.param, type_=param_type, value=value)
            param_client.set_parameters([parameter])
            res.status = "Ok."
        except Exception as e:
            res.status = "Error, could not set param."
        return res

    def recording_status(self, req, res):
        response = Trigger.Response()
        response.success = self.proc is not None

        if response.success:
            response.message = "Bag recording in progress..."
        else:
            response.message = "Bag recorder idle."

        return response

    def recording_setup(self, req, res):
        response = RecordRosbag.Response()

        if req.start:
            if self.proc is not None:
                response.success = False
                response.message = "Already recording, please stop the current recording first."
            else:
                command = ['ros2', 'bag', 'record', '-o']

                # Expand and add the path to the command
                expanded_path = os.path.expanduser(req.path)
                command.append(expanded_path)

                # Add the topics to the command
                for topic in req.topics:
                    command.append(topic)

                # Use subprocess to start rosbag record in a new process
                self.proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                response.success = True
                response.message = "Recording started."

                self.get_logger().info("Recording ros2 bag to "+str(expanded_path))
        else:
            if self.proc is not None:
                # Terminate the rosbag record process
                self.proc.terminate()
                self.proc.wait()
                self.proc = None
                response.success = True
                response.message = "Recording stopped."

                self.get_logger().info("Recording stopped.")
            else:
                response.success = False
                response.message = "No active recording found."
                self.get_logger().info("No active recording found.")

        return response

def main(args=None):
    rclpy.init(args=args)

    service_handler = ServiceHandler(group=ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(num_threads=20)
    executor.add_node(service_handler)

    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

    service_handler.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
