#!/usr/bin/env python3

import os
import threading
import logging
import json
import rclpy

from flask import Flask, render_template, send_from_directory, make_response
from werkzeug.serving import make_server, WSGIRequestHandler

from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from pathlib import Path

node = None
param_base_url = ""
param_port = 5000
param_port_rosbridge = 5001
param_compression = "none"
param_default_widget_config = ""

def get_public_dir():
	p = Path(__file__).resolve()
	path = p.parents[1] / 'public'
	if path.exists():
		return path #for --symlink-install
	return get_package_share_directory('vizanti_server')+ '/public/'

app = Flask(__name__, static_folder=get_public_dir(), template_folder=get_public_dir())

def get_file(path):
	with open(param_default_widget_config, 'r') as f:
		file_content = f.read()
		js_module = f"const content = {json.dumps(file_content)};\nexport default content;"
		response = make_response(js_module)
		response.headers['Content-Type'] = 'application/javascript'
		return response

def get_files(path, valid_extensions):
	templates_dir = os.path.join(app.static_folder, path)
	file_list = []

	for root, dirs, files in os.walk(templates_dir):
		for file in files:
			if os.path.splitext(file)[1] in valid_extensions:
				file_path = os.path.join(root, file)
				with open(file_path, 'r') as f:
					file_content = f.read()
				file_list.append({'path': os.path.relpath(file_path, templates_dir), 'content': file_content})

	js_module = f"const files = {json.dumps(file_list)};\nexport default files;"

	#fetch workaround hackery for webkit support on HTTP
	response = make_response(js_module)
	response.headers['Content-Type'] = 'application/javascript'
	return response

def get_paths(path, valid_extensions):
	templates_dir = os.path.join(app.static_folder, path)
	file_list = []

	for root, dirs, files in os.walk(templates_dir):
		for file in files:
			if os.path.splitext(file)[1] in valid_extensions:
				file_list.append(os.path.relpath(os.path.join(root, file), templates_dir))

	js_module = f"const paths = {json.dumps(file_list)};\nexport default paths;"

	response = make_response(js_module)
	response.headers['Content-Type'] = 'application/javascript'
	return response

def index():
	return render_template('index.html', base_url=param_base_url)

def list_template_files():
	return get_files("templates", ['.html', '.js', '.css'])

def list_robot_model_files():
	return get_paths("assets/robot_model", ['.png'])

def get_default_widget_config():
	return get_file(param_default_widget_config)

def list_ros_launch_params():
	params = {
		"port": param_port,
		"port_rosbridge": param_port_rosbridge,
		"compression": param_compression
	}
	js_module = f"const params = {json.dumps(params)};\nexport default params;"
	response = make_response(js_module)
	response.headers['Content-Type'] = 'application/javascript'
	return response

def serve_static(path):
	return send_from_directory(app.static_folder, path)

class RequestHandler(WSGIRequestHandler):
	def log(self, type, message, *args):
		self.server.log(type, message, *args)

class ServerThread(threading.Thread):
	def __init__(self, app, host='0.0.0.0', port=5000):
		threading.Thread.__init__(self)

		self.log = logging.getLogger('werkzeug')
		self.log.setLevel(logging.INFO)
		handler = logging.StreamHandler()
		handler.setFormatter(logging.Formatter(
			'%(asctime)s %(levelname)s: %(message)s '
			'[in %(pathname)s:%(lineno)d]'
		))
		self.log.addHandler(handler)

		self.srv = make_server(host, port, app, request_handler=RequestHandler)
		self.ctx = app.app_context()
		self.ctx.push()

	def run(self):
		self.srv.serve_forever()

	def shutdown(self):
		self.srv.shutdown()

def main(args=None):
	global node, param_base_url, param_port, param_port_rosbridge, param_compression, param_default_widget_config

	rclpy.init(args=args)
	node = rclpy.create_node('vizanti_flask_node')

	node.declare_parameter('host', '0.0.0.0')
	node.declare_parameter('port', param_port)
	node.declare_parameter('port_rosbridge', param_port_rosbridge)
	node.declare_parameter('flask_debug', True)
	node.declare_parameter('base_url', param_base_url)
	node.declare_parameter('compression', param_compression)
	node.declare_parameter('default_widget_config',param_default_widget_config)

	param_host = node.get_parameter('host').value
	param_port = node.get_parameter('port').value
	param_port_rosbridge = node.get_parameter('port_rosbridge').value
	param_base_url = node.get_parameter('base_url').value
	param_compression = node.get_parameter('compression').value
	param_default_widget_config = node.get_parameter('default_widget_config').value

	if param_default_widget_config != "":
		param_default_widget_config = os.path.expanduser(param_default_widget_config)
	else:
		param_default_widget_config = os.path.join(app.static_folder, "assets/default_layout.json")

	node.get_logger().info(f"Default widget config set to {param_default_widget_config}")

	app.debug = node.get_parameter('flask_debug').value
	app.add_url_rule(param_base_url + '/', 'index', index)
	app.add_url_rule(param_base_url + '/templates/files', 'list_template_files', list_template_files)
	app.add_url_rule(param_base_url + '/assets/robot_model/paths', 'list_robot_model_files', list_robot_model_files)
	app.add_url_rule(param_base_url + '/ros_launch_params', 'ros_launch_params', list_ros_launch_params)
	app.add_url_rule(param_base_url + '/default_widget_config', 'get_default_widget_config', get_default_widget_config)
	app.add_url_rule(param_base_url + '/<path:path>', 'serve_static', serve_static)

	server = ServerThread(app, param_host, param_port)
	server.start()

	node.get_logger().info(f"Flask server running at http://{param_host}:{param_port}{param_base_url}")
	node.get_logger().info(f"Public directory set as {get_public_dir()}")

	rclpy.spin(node)

	server.shutdown()
	server.join()
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
