#!/usr/bin/env python3

import os
import sys

sys.path.insert(0, os.path.join(os.environ.get("ROS_HOME", os.path.expanduser("~/.ros")), "vizanti", "pylib"))

import re
import time
import array
import asyncio
import logging
import threading
import fractions

import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image, CompressedImage
from vizanti_msgs.srv import ListImageStreams, StartImageStream

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
IMAGE_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_TYPE = "sensor_msgs/msg/CompressedImage"
MIN_INTERVAL = 1.0 / 30
IDLE_GRACE = 5.0
CONNECT_TIMEOUT = 15.0
KEYFRAME_INTERVAL = 2.0
MAX_QUEUED_PACKETS = 30
TIME_BASE = fractions.Fraction(1, 90000)
DEPTH_RANGE = 10.0

COLOR_ENCODINGS = {
    "bgr8": (np.uint8, 3, None),
    "8uc3": (np.uint8, 3, None),
    "rgb8": (np.uint8, 3, cv2.COLOR_RGB2BGR),
    "bgra8": (np.uint8, 4, cv2.COLOR_BGRA2BGR),
    "8uc4": (np.uint8, 4, cv2.COLOR_BGRA2BGR),
    "rgba8": (np.uint8, 4, cv2.COLOR_RGBA2BGR),
    "mono8": (np.uint8, 1, None),
    "8uc1": (np.uint8, 1, None),
    "yuv422": (np.uint8, 2, cv2.COLOR_YUV2BGR_UYVY),
    "uyvy": (np.uint8, 2, cv2.COLOR_YUV2BGR_UYVY),
    "yuv422_yuy2": (np.uint8, 2, cv2.COLOR_YUV2BGR_YUYV),
    "yuyv": (np.uint8, 2, cv2.COLOR_YUV2BGR_YUYV),
    "bayer_rggb8": (np.uint8, 1, cv2.COLOR_BayerBG2BGR),
    "bayer_bggr8": (np.uint8, 1, cv2.COLOR_BayerRG2BGR),
    "bayer_gbrg8": (np.uint8, 1, cv2.COLOR_BayerGR2BGR),
    "bayer_grbg8": (np.uint8, 1, cv2.COLOR_BayerGB2BGR),
    "bgr16": (np.uint16, 3, None),
    "rgb16": (np.uint16, 3, cv2.COLOR_RGB2BGR),
    "bgra16": (np.uint16, 4, cv2.COLOR_BGRA2BGR),
    "rgba16": (np.uint16, 4, cv2.COLOR_RGBA2BGR),
    "bayer_rggb16": (np.uint16, 1, cv2.COLOR_BayerBG2BGR),
    "bayer_bggr16": (np.uint16, 1, cv2.COLOR_BayerRG2BGR),
    "bayer_gbrg16": (np.uint16, 1, cv2.COLOR_BayerGR2BGR),
    "bayer_grbg16": (np.uint16, 1, cv2.COLOR_BayerGB2BGR)
}

DEPTH_ENCODINGS = {
    "16uc1": (np.uint16, 0.001),
    "32fc1": (np.float32, 1.0)
}

def pixel_array(msg, dtype, channels, rows):
    dtype = np.dtype(dtype)
    source_dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
    image = np.ndarray((rows, msg.width, channels), source_dtype, msg.data, strides=(msg.step, channels * dtype.itemsize, dtype.itemsize)).astype(dtype, copy=False)
    return image[:, :, 0] if channels == 1 else image

def depth_to_mono(depth, scale):
    return cv2.convertScaleAbs(depth, alpha=scale * 255.0 / DEPTH_RANGE)

def compressed_depth_to_array(msg):
    data = bytes(msg.data)
    start = data.find(b"\x89PNG")

    if start < 12:
        raise ValueError("Unsupported compressedDepth format, only PNG is supported: " + msg.format)

    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8, offset=start), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("Failed to decode compressedDepth image with format: " + msg.format)

    if not msg.format.lower().startswith("32fc1"):
        return depth_to_mono(image, 0.001)

    quant_a, quant_b = np.frombuffer(data, dtype="<f4", count=2, offset=start - 8)
    depth = np.zeros(image.shape, dtype=np.float32)
    valid = image > 0
    depth[valid] = quant_a / (image[valid] - quant_b)
    return depth_to_mono(depth, 1.0)

def image_to_array(msg):
    if isinstance(msg, CompressedImage):
        if "compresseddepth" in msg.format.lower():
            return compressed_depth_to_array(msg)

        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Failed to decode compressed image with format: " + msg.format)
        return image

    encoding = msg.encoding.lower()

    if encoding in DEPTH_ENCODINGS:
        dtype, scale = DEPTH_ENCODINGS[encoding]
        return depth_to_mono(pixel_array(msg, dtype, 1, msg.height), scale)

    if encoding == "mono16":
        image = pixel_array(msg, np.uint16, 1, msg.height)
        low, high = cv2.minMaxLoc(image)[:2]
        return cv2.convertScaleAbs(image, alpha=255.0 / max(high - low, 1), beta=-low * 255.0 / max(high - low, 1))

    if encoding == "nv21":
        return cv2.cvtColor(pixel_array(msg, np.uint8, 1, msg.height * 3 // 2), cv2.COLOR_YUV2BGR_NV21)

    if encoding not in COLOR_ENCODINGS:
        raise ValueError("Unsupported image encoding: " + msg.encoding)

    dtype, channels, conversion = COLOR_ENCODINGS[encoding]
    image = pixel_array(msg, dtype, channels, msg.height)

    if dtype == np.uint16:
        image = cv2.convertScaleAbs(image, alpha=1.0 / 256)

    return image if conversion is None else cv2.cvtColor(image, conversion)

def transcoded(topic, message_type):
    return message_type == IMAGE_TYPE or topic.endswith("/compressedDepth")

def source_qos(node, topic):
    best_effort = any(info.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for info in node.get_publishers_info_by_topic(topic))
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE)

def create_encoder(width, height, crf):
    codec = av.CodecContext.create("libx264", "w")
    codec.width = width
    codec.height = height
    codec.pix_fmt = "yuv420p"
    codec.framerate = fractions.Fraction(30, 1)
    codec.time_base = TIME_BASE
    codec.options = {"preset": "ultrafast", "tune": "zerolatency", "forced-idr": "1", "profile": "baseline", "crf": str(crf)}
    return codec

class RateLimiter:

    def __init__(self):
        self.tokens = 2.0
        self.last = time.monotonic()

    def allow(self, interval):
        now = time.monotonic()
        self.tokens = min(2.0, self.tokens + (now - self.last) / interval)
        self.last = now

        if self.tokens < 1.0:
            return False

        self.tokens -= 1.0
        return True

class RclpyLogHandler(logging.Handler):

    def __init__(self, logger):
        super().__init__()
        self.logger = logger

    def emit(self, record):
        message = record.name + ": " + record.getMessage()

        if record.levelno >= logging.WARNING:
            self.logger.warning(message)
        else:
            self.logger.info(message)

class JpegStream:

    def __init__(self, node, topic, message_type, interval, quality):
        self.node = node
        self.topic = topic
        self.message_class = Image if message_type == IMAGE_TYPE else CompressedImage
        self.output_topic = STREAM_NAMESPACE + topic
        self.interval = interval
        self.quality = quality
        self.limiter = RateLimiter()
        self.requested = time.monotonic()
        self.lock = threading.Lock()
        self.closed = False

        self.publisher = node.create_publisher(CompressedImage, self.output_topic, 1)
        self.subscription = node.create_subscription(self.message_class, topic, self.callback, source_qos(node, topic), callback_group=MutuallyExclusiveCallbackGroup(), raw=True)

    def request(self, interval):
        self.interval = min(self.interval, interval)
        self.requested = time.monotonic()

    def idle(self):
        return self.publisher.get_subscription_count() == 0 and time.monotonic() - self.requested > IDLE_GRACE

    def callback(self, data):
        if not self.limiter.allow(self.interval):
            return

        msg = deserialize_message(data, self.message_class)

        try:
            image = image_to_array(msg)
        except Exception as e:
            self.node.get_logger().warning(self.topic + ": " + str(e), throttle_duration_sec=10)
            return

        success, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not success:
            self.node.get_logger().warning(self.topic + ": JPEG encoding failed.", throttle_duration_sec=10)
            return

        output = CompressedImage()
        output.header = msg.header
        encoding = msg.encoding if self.message_class == Image else msg.format.split(";")[0].strip()
        output.format = encoding + "; jpeg compressed " + ("mono8" if image.ndim == 2 else "bgr8")
        output.data = array.array("B", encoded.tobytes())

        with self.lock:
            if not self.closed:
                self.publisher.publish(output)

    def close(self):
        with self.lock:
            self.closed = True
            self.node.destroy_subscription(self.subscription)
            self.node.destroy_publisher(self.publisher)

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

    def __init__(self, node, topic, message_type, crf, loop):
        self.node = node
        self.topic = topic
        self.interval = MIN_INTERVAL
        self.crf = crf
        self.loop = loop
        self.tracks = set()

        self.codec = None
        self.limiter = RateLimiter()
        self.start_time = time.monotonic()
        self.last_keyframe = 0.0
        self.force_keyframe = True

        self.message_class = Image if message_type == IMAGE_TYPE else CompressedImage
        self.subscription = node.create_subscription(self.message_class, topic, self.callback, source_qos(node, topic), callback_group=MutuallyExclusiveCallbackGroup(), raw=True)

    def add_track(self, track):
        self.tracks.add(track)
        self.interval = min(t.interval for t in self.tracks)

    def remove_track(self, track):
        self.tracks.discard(track)

        if self.tracks:
            self.interval = min(t.interval for t in self.tracks)

    def callback(self, data):
        if not self.limiter.allow(self.interval):
            return

        now = time.monotonic()

        try:
            array = image_to_array(deserialize_message(data, self.message_class))
        except Exception as e:
            self.node.get_logger().warning(self.topic + ": " + str(e), throttle_duration_sec=10)
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
        self.node.destroy_subscription(self.subscription)

class ImageStreamer(Node):

    def __init__(self):
        super().__init__("vizanti_image_streamer")

        self.jpeg_quality = self.declare_parameter("jpeg_quality", 80).value
        self.webrtc_crf = self.declare_parameter("webrtc_crf", 26).value
        self.debug = self.declare_parameter("debug", False).value
        self.peer_count = 0

        handler = RclpyLogHandler(self.get_logger())
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

        group = ReentrantCallbackGroup()
        self.list_service = self.create_service(ListImageStreams, STREAM_NAMESPACE + "/list", self.list_streams, callback_group=group)
        self.start_service = self.create_service(StartImageStream, STREAM_NAMESPACE + "/start", self.start_stream, callback_group=group)
        self.cleanup_timer = self.create_timer(1.0, self.cleanup, callback_group=group)

        if WEBRTC_AVAILABLE:
            self.get_logger().info("Image streamer ready, WebRTC streaming available (aiortc " + aiortc.__version__ + ", PyAV " + av.__version__ + ").")
        else:
            self.get_logger().info("Image streamer ready, WebRTC streaming disabled (" + WEBRTC_ERROR + ").")

    def log_debug(self, message):
        if self.debug:
            self.get_logger().info(message)

    def log_clients(self, stream):
        self.log_debug(stream.topic + ": " + str(len(stream.tracks)) + " WebRTC client(s), encoding every " + str(stream.interval) + " s")

    def published_image_topics(self):
        topics = {}

        for topic, types in self.get_topic_names_and_types():
            if topic.startswith("/vizanti/"):
                continue

            message_type = next((t for t in types if t in (IMAGE_TYPE, COMPRESSED_TYPE)), None)
            if message_type is not None and self.count_publishers(topic) > 0:
                topics[topic] = message_type

        return topics

    def list_streams(self, req, res):
        for topic, message_type in sorted(self.published_image_topics().items()):
            options = [("mjpeg", "transcode" if transcoded(topic, message_type) else "passthrough")]

            if WEBRTC_AVAILABLE:
                options.append(("webrtc", "transcode"))

            for mode, pipeline in options:
                res.topics.append(topic)
                res.types.append(message_type)
                res.modes.append(mode)
                res.pipelines.append(pipeline)

        return res

    def start_stream(self, req, res):
        interval = max(req.throttle / 1000.0, MIN_INTERVAL)
        message_type = self.published_image_topics().get(req.topic)

        if req.mode == "mjpeg":
            res.success = True

            if transcoded(req.topic, message_type):
                res.stream_topic = self.start_jpeg(req.topic, message_type, interval)
            else:
                res.stream_topic = req.topic

            return res

        if req.mode != "webrtc":
            res.message = "Unknown stream mode: " + req.mode
            return res

        if not WEBRTC_AVAILABLE:
            res.message = "WebRTC streaming is unavailable on the robot: " + WEBRTC_ERROR
            return res

        if message_type is None:
            res.message = "Topic is not published: " + req.topic
            return res

        if "h264/" not in req.sdp.lower():
            res.message = "This browser does not support H.264 video over WebRTC."
            return res

        future = asyncio.run_coroutine_threadsafe(self.create_peer(req.topic, message_type, interval, req.sdp), self.loop)

        try:
            res.sdp = future.result(timeout=10)
        except Exception as e:
            res.message = "WebRTC negotiation failed: " + str(e)
            return res

        res.success = True
        return res

    def start_jpeg(self, topic, message_type, interval):
        with self.lock:
            stream = self.jpeg_streams.get(topic)

            if stream is None:
                stream = JpegStream(self, topic, message_type, interval, self.jpeg_quality)
                self.jpeg_streams[topic] = stream
            else:
                stream.request(interval)

            return stream.output_topic

    def cleanup(self):
        with self.lock:
            for topic, stream in list(self.jpeg_streams.items()):
                if stream.idle():
                    stream.close()
                    del self.jpeg_streams[topic]

    async def create_peer(self, topic, message_type, interval, sdp):
        stream = self.webrtc_streams.get(topic)

        if stream is None:
            stream = WebrtcStream(self, topic, message_type, self.webrtc_crf, self.loop)
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

def main(args=None):
    rclpy.init(args=args)

    image_streamer = ImageStreamer()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(image_streamer)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    executor.shutdown()
    image_streamer.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()
