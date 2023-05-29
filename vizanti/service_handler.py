import subprocess
import os
import fcntl
import sys
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from std_srvs.srv import Trigger
from vizanti_interfaces.srv import GetNodeParameters
from vizanti_interfaces.srv import LoadMap, SaveMap
from vizanti_interfaces.srv import RecordRosbag
from vizanti_interfaces.srv import ManageNode, ListPackages, ListExecutables

class ServiceHandler(Node):
    def __init__(self):
        super().__init__("vizanti_topic_handler")

        self.proc = None
        self.packages = self.get_packages()

        self.get_nodes_service = self.create_service(Trigger, 'vizanti/get_dynamic_reconfigure_nodes', self.get_dynamic_reconfigure_nodes)
        self.get_node_parameters_service = self.create_service(GetNodeParameters, 'vizanti/get_node_parameters', self.get_node_parameters)

        self.load_map_service = self.create_service(LoadMap, 'vizanti/load_map', self.load_map)
        self.save_map_service = self.create_service(SaveMap, 'vizanti/save_map', self.save_map)

        self.record_setup_service = self.create_service(RecordRosbag, 'vizanti/bag/setup', self.recording_setup)
        self.record_status_service = self.create_service(Trigger, 'vizanti/bag/status', self.recording_status)

        self_kill_service = self.create_service(ManageNode, 'vizanti/node/kill', self.node_kill)
        self_start_service = self.create_service(ManageNode, 'vizanti/node/start', self.node_start)
        self_info_service = self.create_service(ManageNode, 'vizanti/node/info', self.node_info)
        self_info_service = self.create_service(Trigger, 'vizanti/roswtf', self.roswtf)

        self.list_packages_service = self.create_service(ListPackages, 'vizanti/list_packages', self.list_packages_callback)
        self.list_executables_service = self.create_service(ListExecutables, 'vizanti/list_executables', self.list_executables_callback)

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

    def list_executables_callback(self, req, res):
        if req.package not in self.packages:
            self.get_logger().error("Package not found: " + req.package)
            res.executables = []
            return res

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

        res.executables = executables + python_and_launch_files
        return res

    def node_kill(self, req, res):
        try:
            subprocess.call(['ros2', 'node', 'kill', req.node])
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
            rosinfo = subprocess.check_output(["ros2", "doctor"]).decode('utf-8')
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
            rclpy.sleep(1)

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
            process = subprocess.Popen(["ros2", "run", "nav2_map_server", "map_saver", "-f", file_path, "map:=" + topic, "__name:=vizanti_map_saver"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
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
                rclpy.sleep(0.2)

            res.success = True
            res.message = "Map saved successfully"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def get_dynamic_reconfigure_nodes(self, req, res):
        process = subprocess.Popen(["ros2", "run", "rqt_reconfigure", "rqt_reconfigure"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        flags = fcntl.fcntl(process.stdout, fcntl.F_GETFL)
        fcntl.fcntl(process.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        # Wait for it to either fail or not
        rclpy.sleep(1)

        # Check if the process is still running
        if process.poll() is not None:
            # Process terminated, read the error output
            error_output = process.stdout.read().decode('utf-8')
            res.success = False
            res.message = "Failed to get dynamic reconfigure nodes: " + error_output
        else:
            res.success = True
            res.message = "Got dynamic reconfigure nodes successfully"
        return res

    def get_node_parameters(self, req, res):
        node_params = subprocess.check_output(["ros2", "param", "list", req.node]).decode('utf-8')

        res.parameters = node_params
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
        else:
            if self.proc is not None:
                # Terminate the rosbag record process
                self.proc.terminate()
                self.proc.wait()
                self.proc = None
                response.success = True
                response.message = "Recording stopped."
            else:
                response.success = False
                response.message = "No active recording found."

        return response

def main(args=None):
    rclpy.init(args=args)
    service_handler = ServiceHandler()
    rclpy.spin(service_handler)
    service_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
