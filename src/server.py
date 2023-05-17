#!/usr/bin/env python3
import os
import subprocess
import rospy
import threading
import logging

from flask import Flask, render_template, send_from_directory, jsonify
from werkzeug.serving import make_server, WSGIRequestHandler

app = Flask(__name__, static_folder='../public', template_folder='../public')

def get_files(path, valid_extensions):
	templates_dir = os.path.join(app.static_folder, path)
	output = subprocess.check_output(["ls", "-R", templates_dir]).decode("utf-8")
	all_lines = output.strip().split("\n")
	file_list = []

	current_path = templates_dir
	for line in all_lines:
		if line.endswith(':'):
			current_path = os.path.relpath(line[:-1], templates_dir)
		else:
			file_extension = os.path.splitext(line)[1]
			if (file_extension in valid_extensions) and (line.count('_') == 1):
				file_list.append(os.path.join(current_path, line))

	return jsonify(file_list)

@app.route('/')
def index():
	return render_template('index.html')

@app.route('/templates/files')
def list_template_files():
	return get_files("templates", ['.html', '.js', '.css'])

@app.route('/assets/robot_model/files')
def list_robot_model_files():
	return get_files("assets/robot_model", ['.png'])

@app.route('/<path:path>')
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

rospy.init_node('outdooros_flask_node')

param_host = rospy.get_param('~host', '0.0.0.0')
param_port = rospy.get_param('~port', 5000)

app.debug = rospy.get_param('~flask_debug', True)

server = ServerThread(app, param_host, param_port)
server.start()

rospy.on_shutdown(server.shutdown)
rospy.spin()