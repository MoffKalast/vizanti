#!/usr/bin/env python3
import os
import rospy
from rospkg import RosPack
import threading
import logging
import json
import gzip
import hashlib

from flask import Flask, render_template, send_from_directory, make_response, request
from waitress.server import create_server

GZIP_CACHE_MAX_ENTRIES = 256
COMPRESSIBLE_MIMETYPES = {'application/javascript', 'text/javascript', 'text/css', 'text/html', 'application/json', 'image/svg+xml', 'text/plain'}
gzip_cache = {} 

rospy.init_node('vizanti_flask_node')

param_host = rospy.get_param('~host', '0.0.0.0')
param_port = rospy.get_param('~port', 5000)
param_port_rosbridge = rospy.get_param('~port_rosbridge', 5001)
param_base_url = rospy.get_param('~base_url', '')
param_default_widget_config = rospy.get_param('~default_widget_config', '')

public_dir = RosPack().get_path('vizanti') + '/public/'

app = Flask(__name__, static_folder=public_dir, template_folder=public_dir)
app.debug = rospy.get_param('~flask_debug', False)

if param_default_widget_config != "":
	param_default_widget_config = os.path.expanduser(param_default_widget_config)
else:
	param_default_widget_config = os.path.join(app.static_folder, "assets/default_layout.json")

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
	templates_dir = os.path.join(app.static_folder, "assets/robot_model")
	categorized_files = {
		'ground': [],
		'air': [],
		'sea': [],
		'misc': []
	}
	
	for root, dirs, files in os.walk(templates_dir):
		for file in files:
			if os.path.splitext(file)[1] == '.png':
				rel_path = os.path.relpath(root, templates_dir)
				category = rel_path if rel_path in categorized_files else 'misc'
				if category == '.':  # files in root directory
					category = 'misc'
				categorized_files[category].append(file)
	
	js_module = f"const categorizedPaths = {json.dumps(categorized_files)};\nexport default categorizedPaths;"
	response = make_response(js_module)
	response.headers['Content-Type'] = 'application/javascript'
	return response

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

@app.route(param_base_url + '/default_widget_config')
def get_default_widget_config():
	return get_file(param_default_widget_config)

@app.route(param_base_url + '/<path:path>')
def serve_static(path):
	return send_from_directory(app.static_folder, path)

@app.after_request
def add_caching_and_compression(response):
	if response.status_code != 200:
		return response

	# images and other binary assets are safe to cache for a day,
	# text content stays no-cache so the browser revalidates via ETag and gets cheap 304s
	if response.mimetype in COMPRESSIBLE_MIMETYPES:
		response.headers['Cache-Control'] = 'no-cache'
	else:
		response.headers['Cache-Control'] = 'public, max-age=86400'
		return response

	response.direct_passthrough = False
	data = response.get_data()

	content_hash = hashlib.md5(data).hexdigest()
	accepts_gzip = 'gzip' in request.headers.get('Accept-Encoding', '').lower()

	if accepts_gzip and len(data) > 512:
		compressed = gzip_cache.get(content_hash)
		if compressed is None:
			compressed = gzip.compress(data, 6)
			if len(gzip_cache) >= GZIP_CACHE_MAX_ENTRIES:
				gzip_cache.clear()
			gzip_cache[content_hash] = compressed

		if len(compressed) < len(data):
			response.set_data(compressed)
			response.headers['Content-Encoding'] = 'gzip'
			response.headers['Vary'] = 'Accept-Encoding'
			response.set_etag(content_hash + '-gz')
		else:
			response.set_etag(content_hash)
	else:
		response.set_etag(content_hash)

	# turns the response into a 304 if the client's If-None-Match matches
	return response.make_conditional(request)

class ServerThread(threading.Thread):
	
	def __init__(self, app, host='0.0.0.0', port=5000):
		threading.Thread.__init__(self)
		self.daemon = True
		
		self.log = logging.getLogger('waitress')
		self.log.setLevel(logging.INFO)
		handler = logging.StreamHandler()
		handler.setFormatter(logging.Formatter(
			'[%(levelname)s] [%(asctime)s] [waitress]: %(message)s '
		))
		self.log.addHandler(handler)
		
		self.app = app
		self.host = host
		self.port = port
		self.ctx = app.app_context()
		self.ctx.push()
		
		self._server = None
		self._stop_event = threading.Event()
		
	def run(self):
		self._server = create_server(self.app, host=self.host, port=self.port)
		try:
			self._server.run()
		except KeyboardInterrupt:
			self.shutdown()
		
	def shutdown(self):
		if self._server:
			self._server.close()  # This triggers waitress to stop accepting new connections
			self._stop_event.set()  # Signal that we're stopping
			rospy.loginfo("Waitress server shutting down...")

server = ServerThread(app, param_host, param_port)
server.start()

rospy.on_shutdown(server.shutdown)
rospy.spin()