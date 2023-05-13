#!/usr/bin/env python3
import os
import subprocess
import rospy

from flask import Flask, render_template, send_from_directory, jsonify

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

rospy.init_node('outdooros_server')
app.run(debug=True, host='0.0.0.0')