#!/usr/bin/env python3

from flask import Flask, render_template, send_from_directory

app = Flask(__name__, static_folder='../public', template_folder='../public')

@app.route('/')
def index():
	return render_template('index.html')



@app.route('/<path:path>')
def serve_static(path):
	return send_from_directory(app.static_folder, path)

app.run(debug=True, host='0.0.0.0')