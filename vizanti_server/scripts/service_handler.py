#!/usr/bin/env python3

import subprocess
import os
import fcntl
import sys
import rclpy
import yaml
import json
import time

from rclpy.node import Node

from rclpy.executors import  MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from vizanti_msgs.srv import GetNodeParameters, SetNodeParameter
from vizanti_msgs.srv import LoadMap, SaveMap
from vizanti_msgs.srv import RecordRosbag
from vizanti_msgs.srv import ManageNode, ListPackages, ListExecutables

class ServiceHandler(Node):
    def __init__(self, group):
        super().__init__("vizanti_topic_handler")

        self.proc = None
        self.packages = self.get_packages()

        self.get_node_parameters_service = self.create_service(GetNodeParameters, 'vizanti/get_node_parameters', self.get_node_parameters, callback_group=group)
        self.set_node_parameter_service = self.create_service(SetNodeParameter, 'vizanti/set_node_parameter', self.set_node_parameter, callback_group=group)

        self.load_map_service = self.create_service(LoadMap, 'vizanti/load_map', self.load_map, callback_group=group)
        self.save_map_service = self.create_service(SaveMap, 'vizanti/save_map', self.save_map, callback_group=group)

        self.record_setup_service = self.create_service(RecordRosbag, 'vizanti/bag/setup', self.recording_setup, callback_group=group)


        self.kill_service = self.create_service(ManageNode, 'vizanti/node/kill', self.node_kill, callback_group=group)
        self.start_service = self.create_service(ManageNode, 'vizanti/node/start', self.node_start, callback_group=group)
        self.info_service = self.create_service(ManageNode, 'vizanti/node/info', self.node_info, callback_group=group)
        self.info_service = self.create_service(Trigger, 'vizanti/roswtf', self.roswtf, callback_group=group)
        self.record_status_service = self.create_service(Trigger, 'vizanti/bag/status', self.recording_status, callback_group=group)

        self.list_packages_service = self.create_service(ListPackages, 'vizanti/list_packages', self.list_packages_callback, callback_group=group)
        self.list_executables_service = self.create_service(ListExecutables, 'vizanti/list_executables', self.list_executables_callback, callback_group=group)

        self.get_logger().info("Service handler ready.")
    
    def get_packages(self):
        # Use aros2pkg to get list of packages
        process = subprocess.Popen(['ros2', 'pkg', 'list'], stdout=subprocess.PIPE)
        output, error = process.communicate()
        # Process output
        lines = output.decode('utf-8').split('\n')
        return lines

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
        
        process = subprocess.Popen(['ros2', 'pkg', 'prefix', req.package], stdout=subprocess.PIPE)
        output, error = process.communicate()
        path = output.decode('utf-8').strip()

        self.get_logger().info(f"Executables Package: {req.package}")
        self.get_logger().info(f"Path: {path}")

        libpath = path+"/share/"+req.package
        launchpath = path+"/lib/"+req.package
        #TODO add path to apt installed packages, I'm not sure where exactly those executables are yet
        self.get_logger().info(f"libpath: {libpath}")

        cmd_exec = ["find", libpath] # get list of executables
        cmd_exec = cmd_exec + ["-type", "f", "-o", "-type", "l"] # files or symlinks

        cmd_launch = ["find", launchpath]
        cmd_launch = cmd_launch + ["-type", "f", "-o", "-type", "l"]
        
        self.get_logger().info(f"cmd_exec: {cmd_exec}")
        self.get_logger().info(f"cmd_python_and_launch: {cmd_launch}")

        process_exec = subprocess.Popen(cmd_exec, stdout=subprocess.PIPE)
        process_launch = subprocess.Popen(cmd_launch, stdout=subprocess.PIPE)

        output_exec, error_exec = process_exec.communicate()
        output_launch, error_launch = process_launch.communicate()

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
            proc = subprocess.Popen("ros2 param dump "+req.node, stdout=subprocess.PIPE, stderr=subprocess. STDOUT, shell=True)
            node_params = proc.communicate(timeout=5)[0].decode('utf-8')

            if node_params.startswith("failed"):
                raise Exception

            parsed = yaml.safe_load(node_params)
            res.parameters = json.dumps(parsed[list(parsed.keys())[0]])
        except:
            res.parameters = "{}"
        return res
    
    def set_node_parameter(self, req, res):
        try:
            proc = subprocess.Popen("ros2 param set "+req.node+" "+req.param+" "+req.value, stdout=subprocess.PIPE, stderr=subprocess. STDOUT, shell=True)
            node_params = proc.communicate(timeout=5)[0].decode('utf-8')

            if node_params.startswith("failed"):
                raise Exception

            res.status = "Ok."
        except:
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
