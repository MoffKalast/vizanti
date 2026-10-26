#!/usr/bin/env python3

import os
import sys

sys.path.insert(0, os.path.join(os.environ.get("ROS_HOME", os.path.expanduser("~/.ros")), "vizanti", "pylib"))

import re
import time
import asyncio
import logging
import threading
import fractions

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image, CompressedImage
from vizanti.srv import ListImageStreams, ListImageStreamsResponse, StartImageStream, StartImageStreamResponse

try:
	import av
	import aiortc
	from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCRtpSender, MediaStreamTrack
	from aiortc.codecs.h264 import H264Encoder
	WEBRTC_ERROR = "" if hasattr(H264Encoder, "pack") else "aiortc " + aiortc.__version__ + " is too old, version 1.4 or newer is required"
except ImportError:
	MediaStreamTrack = object
	WEBRTC_ERROR = "requires aiortc and PyAV"

WEBRTC_AVAILABLE = WEBRTC_ERROR == ""

STREAM_NAMESPACE = "/vizanti/image_streams"
IMAGE_TYPE = "sensor_msgs/Image"
COMPRESSED_TYPE = "sensor_msgs/CompressedImage"
IMAGE_ENCODINGS = ("bgr8", "rgb8", "mono8")

IDLE_GRACE = 5.0
CONNECT_TIMEOUT = 15.0
KEYFRAME_INTERVAL = 2.0
MAX_QUEUED_PACKETS = 30
TIME_BASE = fractions.Fraction(1, 90000)

def image_to_array(msg):
	if isinstance(msg, CompressedImage):
		array = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
		if array is None:
			raise ValueError("Failed to decode compressed image with format: " + msg.format)
		return array

	if msg.encoding not in IMAGE_ENCODINGS:
		raise ValueError("Unsupported image encoding: " + msg.encoding)

	channels = 1 if msg.encoding == "mono8" else 3
	array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]

	if channels == 1:
		return array

	array = array.reshape(msg.height, msg.width, 3)

	if msg.encoding == "rgb8":
		return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)

	return array

def create_encoder(width, height, crf):
	codec = av.CodecContext.create("libx264", "w")
	codec.width = width
	codec.height = height
	codec.pix_fmt = "yuv420p"
	codec.framerate = fractions.Fraction(30, 1)
	codec.time_base = TIME_BASE
	codec.options = {"preset": "ultrafast", "tune": "zerolatency", "forced-idr": "1", "profile": "baseline", "crf": str(crf)}
	return codec

class RospyLogHandler(logging.Handler):

	def emit(self, record):
		message = record.name + ": " + record.getMessage()

		if record.levelno >= logging.WARNING:
			rospy.logwarn(message)
		else:
			rospy.loginfo(message)

class JpegStream:

	def __init__(self, topic, interval, quality):
		self.topic = topic
		self.output_topic = STREAM_NAMESPACE + topic
		self.interval = interval
		self.quality = quality
		self.last_encode = 0.0
		self.requested = time.monotonic()

		self.publisher = rospy.Publisher(self.output_topic, CompressedImage, queue_size=1)
		self.subscriber = rospy.Subscriber(topic, Image, self.callback, queue_size=1, buff_size=2**24)

	def request(self, interval):
		self.interval = min(self.interval, interval)
		self.requested = time.monotonic()

	def idle(self):
		return self.publisher.get_num_connections() == 0 and time.monotonic() - self.requested > IDLE_GRACE

	def callback(self, msg):
		now = time.monotonic()
		if now - self.last_encode < self.interval:
			return

		self.last_encode = now

		try:
			array = image_to_array(msg)
		except ValueError as e:
			rospy.logwarn_throttle(10, self.topic + ": " + str(e))
			return

		success, data = cv2.imencode(".jpg", array, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
		if not success:
			rospy.logwarn_throttle(10, self.topic + ": JPEG encoding failed.")
			return

		output = CompressedImage()
		output.header = msg.header
		output.format = msg.encoding + "; jpeg compressed " + ("mono8" if array.ndim == 2 else "bgr8")
		output.data = data.tobytes()
		self.publisher.publish(output)

	def close(self):
		self.subscriber.unregister()
		self.publisher.unregister()

class PacketTrack(MediaStreamTrack):

	kind = "video"

	def __init__(self, interval):
		super().__init__()
		self.interval = interval
		self.queue = asyncio.Queue()
		self.waiting_for_keyframe = True

	def push(self, packet):
		if self.queue.qsize() > MAX_QUEUED_PACKETS:
			while not self.queue.empty():
				self.queue.get_nowait()
			self.waiting_for_keyframe = True

		if self.waiting_for_keyframe:
			if not packet.is_keyframe:
				return False
			self.waiting_for_keyframe = False

		self.queue.put_nowait(packet)
		return True

	async def recv(self):
		return await self.queue.get()

class WebrtcStream:

	def __init__(self, topic, message_type, crf, loop):
		self.topic = topic
		self.interval = 0.0
		self.crf = crf
		self.loop = loop
		self.tracks = set()

		self.codec = None
		self.start_time = time.monotonic()
		self.last_encode = 0.0
		self.last_keyframe = 0.0
		self.force_keyframe = True

		message_class = Image if message_type == IMAGE_TYPE else CompressedImage
		self.subscriber = rospy.Subscriber(topic, message_class, self.callback, queue_size=1, buff_size=2**24)

	def add_track(self, track):
		self.tracks.add(track)
		self.interval = min(t.interval for t in self.tracks)

	def remove_track(self, track):
		self.tracks.discard(track)

		if self.tracks:
			self.interval = min(t.interval for t in self.tracks)

	def callback(self, msg):
		now = time.monotonic()
		if now - self.last_encode < self.interval:
			return

		self.last_encode = now

		try:
			array = image_to_array(msg)
		except ValueError as e:
			rospy.logwarn_throttle(10, self.topic + ": " + str(e))
			return

		height = array.shape[0] & ~1
		width = array.shape[1] & ~1

		frame = av.VideoFrame.from_ndarray(np.ascontiguousarray(array[:height, :width]), format="gray" if array.ndim == 2 else "bgr24")
		frame.pts = int((now - self.start_time) * 90000)
		frame.time_base = TIME_BASE

		if self.codec is None or self.codec.width != width or self.codec.height != height:
			self.codec = create_encoder(width, height, self.crf)
			self.force_keyframe = True

		if self.force_keyframe or now - self.last_keyframe > KEYFRAME_INTERVAL:
			frame.pict_type = av.video.frame.PictureType.I
			self.force_keyframe = False
			self.last_keyframe = now

		for packet in self.codec.encode(frame):
			self.loop.call_soon_threadsafe(self.dispatch, packet)

	def dispatch(self, packet):
		for track in self.tracks:
			if not track.push(packet):
				self.force_keyframe = True

	def close(self):
		self.subscriber.unregister()

class ImageStreamer:

	def __init__(self):
		rospy.init_node('vizanti_image_streamer')

		self.jpeg_quality = rospy.get_param('~jpeg_quality', 80)
		self.webrtc_crf = rospy.get_param('~webrtc_crf', 26)
		self.debug = rospy.get_param('~debug', False)
		self.peer_count = 0

		handler = RospyLogHandler()
		for name in ("aiortc", "aioice"):
			logger = logging.getLogger(name)
			logger.setLevel(logging.INFO if self.debug else logging.WARNING)
			logger.addHandler(handler)

		self.lock = threading.Lock()
		self.jpeg_streams = {}
		self.webrtc_streams = {}

		if WEBRTC_AVAILABLE:
			self.loop = asyncio.new_event_loop()
			threading.Thread(target=self.loop.run_forever, daemon=True).start()

		self.list_service = rospy.Service(STREAM_NAMESPACE + '/list', ListImageStreams, self.list_streams)
		self.start_service = rospy.Service(STREAM_NAMESPACE + '/start', StartImageStream, self.start_stream)
		self.cleanup_timer = rospy.Timer(rospy.Duration(1.0), self.cleanup)

		if WEBRTC_AVAILABLE:
			rospy.loginfo("Image streamer ready, WebRTC streaming available (aiortc " + aiortc.__version__ + ", PyAV " + av.__version__ + ").")
		else:
			rospy.loginfo("Image streamer ready, WebRTC streaming disabled (" + WEBRTC_ERROR + ").")

	def log_debug(self, message):
		if self.debug:
			rospy.loginfo(message)

	def log_clients(self, stream):
		self.log_debug(stream.topic + ": " + str(len(stream.tracks)) + " WebRTC client(s), encoding every " + str(stream.interval) + " s")

	def published_image_topics(self):
		return {topic: message_type for topic, message_type in rospy.get_published_topics() if message_type in (IMAGE_TYPE, COMPRESSED_TYPE) and not topic.startswith("/vizanti/")}

	def list_streams(self, req):
		response = ListImageStreamsResponse()

		for topic, message_type in sorted(self.published_image_topics().items()):
			options = [("mjpeg", "passthrough" if message_type == COMPRESSED_TYPE else "transcode")]

			if WEBRTC_AVAILABLE:
				options.append(("webrtc", "transcode"))

			for mode, pipeline in options:
				response.topics.append(topic)
				response.types.append(message_type)
				response.modes.append(mode)
				response.pipelines.append(pipeline)

		return response

	def start_stream(self, req):
		interval = max(req.throttle, 0) / 1000.0
		message_type = self.published_image_topics().get(req.topic)

		if req.mode == "mjpeg":
			if message_type != IMAGE_TYPE:
				return StartImageStreamResponse(success=True, message="", stream_topic=req.topic, sdp="")

			return StartImageStreamResponse(success=True, message="", stream_topic=self.start_jpeg(req.topic, interval), sdp="")

		if req.mode != "webrtc":
			return StartImageStreamResponse(success=False, message="Unknown stream mode: " + req.mode, stream_topic="", sdp="")

		if not WEBRTC_AVAILABLE:
			return StartImageStreamResponse(success=False, message="WebRTC streaming is unavailable on the robot: " + WEBRTC_ERROR, stream_topic="", sdp="")

		if message_type is None:
			return StartImageStreamResponse(success=False, message="Topic is not published: " + req.topic, stream_topic="", sdp="")

		if "h264/" not in req.sdp.lower():
			return StartImageStreamResponse(success=False, message="This browser does not support H.264 video over WebRTC.", stream_topic="", sdp="")

		future = asyncio.run_coroutine_threadsafe(self.create_peer(req.topic, message_type, interval, req.sdp), self.loop)

		try:
			sdp = future.result(timeout=10)
		except Exception as e:
			return StartImageStreamResponse(success=False, message="WebRTC negotiation failed: " + str(e), stream_topic="", sdp="")

		return StartImageStreamResponse(success=True, message="", stream_topic="", sdp=sdp)

	def start_jpeg(self, topic, interval):
		with self.lock:
			stream = self.jpeg_streams.get(topic)

			if stream is None:
				stream = JpegStream(topic, interval, self.jpeg_quality)
				self.jpeg_streams[topic] = stream
			else:
				stream.request(interval)

			return stream.output_topic

	def cleanup(self, event):
		with self.lock:
			for topic, stream in list(self.jpeg_streams.items()):
				if stream.idle():
					stream.close()
					del self.jpeg_streams[topic]

	async def create_peer(self, topic, message_type, interval, sdp):
		stream = self.webrtc_streams.get(topic)

		if stream is None:
			stream = WebrtcStream(topic, message_type, self.webrtc_crf, self.loop)
			self.webrtc_streams[topic] = stream

		self.peer_count += 1
		label = topic + " client " + str(self.peer_count)
		self.log_debug(label + ": offer received, throttle " + str(interval) + " s")

		track = PacketTrack(interval)
		stream.add_track(track)
		self.log_clients(stream)
		peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))

		@peer.on("connectionstatechange")
		async def on_connection_state_change():
			self.log_debug(label + ": connection " + peer.connectionState)

			if peer.connectionState in ("failed", "closed"):
				await self.remove_peer(peer, stream, track)

		@peer.on("iceconnectionstatechange")
		async def on_ice_connection_state_change():
			self.log_debug(label + ": ICE " + peer.iceConnectionState)

		try:
			sender = peer.addTrack(track)
			transceiver = next(t for t in peer.getTransceivers() if t.sender == sender)
			transceiver.setCodecPreferences([codec for codec in RTCRtpSender.getCapabilities("video").codecs if codec.mimeType == "video/H264"])

			await peer.setRemoteDescription(RTCSessionDescription(sdp=re.sub(r"a=end-of-candidates\r?\n", "", sdp), type="offer"))
			await peer.setLocalDescription(await peer.createAnswer())
		except Exception as e:
			self.log_debug(label + ": negotiation failed: " + str(e))
			await self.remove_peer(peer, stream, track)
			raise

		self.log_debug(label + ": answer sent")
		asyncio.ensure_future(self.expire_peer(peer, stream, track, label))
		return peer.localDescription.sdp

	async def expire_peer(self, peer, stream, track, label):
		await asyncio.sleep(CONNECT_TIMEOUT)

		if peer.connectionState in ("new", "connecting"):
			self.log_debug(label + ": not connected after " + str(CONNECT_TIMEOUT) + " s, closing")
			await self.remove_peer(peer, stream, track)

	async def remove_peer(self, peer, stream, track):
		if track in stream.tracks:
			stream.remove_track(track)
			self.log_clients(stream)

		await peer.close()

		if not stream.tracks and self.webrtc_streams.get(stream.topic) is stream:
			stream.close()
			del self.webrtc_streams[stream.topic]
			self.log_debug(stream.topic + ": no WebRTC clients left, stream closed")

ImageStreamer()
rospy.spin()
