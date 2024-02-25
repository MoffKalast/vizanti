#!/usr/bin/env python3
import os
import rospy
from rospkg import RosPack
import threading
import logging
import json

from flask import Flask, render_template, send_from_directory, make_response
from werkzeug.serving import make_server, WSGIRequestHandler

rospy.init_node('vizanti_flask_node')

param_host = rospy.get_param('~host', '0.0.0.0')
param_port = rospy.get_param('~port', 5000)
param_port_rosbridge = rospy.get_param('~port_rosbridge', 5001)
param_base_url = rospy.get_param('~base_url', '')

public_dir = RosPack().get_path('vizanti') + '/public/'

app = Flask(__name__, static_folder=public_dir, template_folder=public_dir)
app.debug = rospy.get_param('~flask_debug', True)

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

	js_module = f"const files = {json.dumps(file_list)};\n\nexport default files;"

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

	js_module = f"const paths = {json.dumps(file_list)};\n\nexport default paths;"

	response = make_response(js_module)
	response.headers['Content-Type'] = 'application/javascript'
	return response

@app.route(param_base_url + '/')
def index():
	return render_template('index.html', base_url=param_base_url)

@app.route(param_base_url + '/templates/files')
def list_template_files():
	return get_files("templates", ['.html', '.js', '.css'])

@app.route(param_base_url + '/assets/robot_model/paths')
def list_robot_model_files():
	return get_paths("assets/robot_model", ['.png'])

@app.route(param_base_url + '/ros_launch_params')
def list_ros_launch_params():
    params = {
        "port": param_port,
        "port_rosbridge": param_port_rosbridge
    }
    js_module = f"const params = {json.dumps(params)};\n\nexport default params;"
    response = make_response(js_module)
    response.headers['Content-Type'] = 'application/javascript'
    return response

@app.route(param_base_url + '/<path:path>')
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


server = ServerThread(app, param_host, param_port)
server.start()

rospy.on_shutdown(server.shutdown)
rospy.spin()