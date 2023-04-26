#!/usr/bin/env python3
import os
import subprocess

from flask import Flask, render_template, send_from_directory, jsonify

app = Flask(__name__, static_folder='../public', template_folder='../public')

@app.route('/')
def index():
	return render_template('index.html')

@app.route('/templates/files')
def list_template_files():
    templates_dir = os.path.join(app.static_folder, "templates")
    output = subprocess.check_output(["ls", templates_dir]).decode("utf-8")
    all_files = output.strip().split("\n")

    valid_extensions = ['.html', '.js', '.css']
    file_list = [file for file in all_files if
                 (os.path.splitext(file)[1] in valid_extensions) and
                 (file.count('_') == 1)]

    return jsonify(file_list)

@app.route('/<path:path>')
def serve_static(path):
	return send_from_directory(app.static_folder, path)

app.run(debug=True, host='0.0.0.0')