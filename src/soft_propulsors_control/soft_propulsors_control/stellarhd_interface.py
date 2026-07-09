"""
stellarhd_interface.py — StellarHD Camera Interface Node
========================================================
DWE StellarHD camera interface — the single owner of the camera hardware.
https://dwe.ai/products/stellarhd

This node does two things with the one camera it owns:
  1. Publishes every captured frame on an image topic so perception nodes
     (e.g. apriltag_interface) can consume frames without opening the device
     themselves — the camera hardware is touched here and nowhere else.
  2. Records video to disk, segmented per mission: each mission gets its own
     video file, keyed on the mission label.

Communication
-------------
Subscribes : mission_cmd    (std_msgs/String) - JSON mission; new label = new segment
             mission_status (std_msgs/String) - JSON status; HOVER/idle stops recording
Publishes  : <image_topic>  (sensor_msgs/Image) - raw frames for perception
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import json
import time
import threading
from pathlib import Path
from datetime import datetime

# Silence OpenCV's own C++ VIDEOIO warnings (e.g. "can't open camera by index"),
# which it prints on every VideoCapture() attempt regardless of our logging.
try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
except Exception:
    pass

try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    HAVE_BRIDGE = True
except ImportError:
    HAVE_BRIDGE = False


class StellarHDCameraNode(Node):
    """
    StellarHD camera node — sole camera owner.

    Continuously captures frames, republishes them for perception, and saves one
    video segment per mission.  Files are named by mission label and timestamp.
    """

    def __init__(self):
        super().__init__('stellarhd_camera')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('video_width', 1920)
        self.declare_parameter('video_height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('output_directory', '/home/gingerstep/videos')
        self.declare_parameter('fourcc', 'mp4v')          # 'mp4v', 'XVID', 'H264'
        # Frame republishing (so perception never opens the camera itself)
        self.declare_parameter('publish_images', True)
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('publish_rate', 15.0)      # Hz cap for republished frames
        # How often to silently retry opening an absent camera (seconds).
        self.declare_parameter('reconnect_interval', 30.0)

        # ------------------------------------------------------------------
        # Initialize camera
        # ------------------------------------------------------------------
        camera_index = self.get_parameter('camera_index').value
        width = self.get_parameter('video_width').value
        height = self.get_parameter('video_height').value
        fps = self.get_parameter('fps').value

        # Force the V4L2 backend: OpenCV's default GStreamer backend fails to
        # start a v4l2src pipeline on this platform ("Internal data stream
        # error"), whereas the direct V4L2 backend opens the device cleanly.
        self._camera_index = camera_index
        self._req_width, self._req_height, self._req_fps = width, height, fps
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Missing camera is non-fatal: log an error and keep running (the capture
        # loop backs off and periodically retries opening it) so it never crashes
        # the node — the rest of the stack works without the camera.
        if self.cap.isOpened():
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(
                f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps} fps")
        else:
            self.get_logger().error(
                f"Camera {camera_index} not available — running WITHOUT the camera "
                f"(no frames/recording); will keep retrying.")
            actual_width, actual_height, actual_fps = width, height, fps

        # ------------------------------------------------------------------
        # Recording state
        # ------------------------------------------------------------------
        self.output_dir = Path(self.get_parameter('output_directory').value)
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            self.get_logger().error(
                f"Cannot create output dir {self.output_dir} ({e}) — recording disabled.")

        fourcc_str = self.get_parameter('fourcc').value
        self.fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
        self.fps = fps
        self.frame_size = (actual_width, actual_height)

        self.video_writer = None
        self.current_label = None
        self.recording_lock = threading.Lock()

        # ------------------------------------------------------------------
        # Frame republishing
        # ------------------------------------------------------------------
        self.publish_images = self.get_parameter('publish_images').value and HAVE_BRIDGE
        self._bridge = CvBridge() if self.publish_images else None
        self._pub_period = 1.0 / max(1.0, self.get_parameter('publish_rate').value)
        self._last_pub_time = 0.0
        if self.publish_images:
            topic = self.get_parameter('image_topic').value
            self.image_pub = self.create_publisher(Image, topic, 10)
            self.get_logger().info(f"Publishing frames on '{topic}' for perception.")
        elif self.get_parameter('publish_images').value and not HAVE_BRIDGE:
            self.get_logger().warn("cv_bridge unavailable — frame republishing disabled.")

        # ------------------------------------------------------------------
        # Mission subscriptions (recording is segmented per mission)
        # ------------------------------------------------------------------
        self.create_subscription(String, 'mission_cmd', self._mission_cmd_cb, 10)
        self.create_subscription(String, 'mission_status', self._mission_status_cb, 10)

        # ------------------------------------------------------------------
        # Capture thread
        # ------------------------------------------------------------------
        self._reconnect_interval = max(1.0, self.get_parameter('reconnect_interval').value)
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info(f"Recording videos to: {self.output_dir}")

    # ------------------------------------------------------------------
    # Mission-driven recording segmentation
    # ------------------------------------------------------------------

    def _mission_cmd_cb(self, msg: String):
        """A new mission begins a new video segment (one file per mission label)."""
        try:
            label = json.loads(msg.data).get('label')
        except json.JSONDecodeError:
            return
        if not label:
            return
        if str(label).upper() == 'HOVER':
            with self.recording_lock:
                self._stop_recording()
            return
        # Same label re-dispatched (a retry) keeps the existing segment open
        with self.recording_lock:
            if label != self.current_label:
                self._start_recording(label)

    def _mission_status_cb(self, msg: String):
        """Stop the segment when the robot goes idle (all missions done)."""
        try:
            event = json.loads(msg.data).get('event')
        except json.JSONDecodeError:
            return
        if event == 'ALL_MISSIONS_DONE':
            with self.recording_lock:
                self._stop_recording()

    def _start_recording(self, label):
        """Start a new video for the given mission label."""
        if self.video_writer is not None:
            self._stop_recording()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe = ''.join(c if c.isalnum() else '_' for c in str(label))
        filename = f"mission_{safe}_{timestamp}.mp4"
        writer = cv2.VideoWriter(
            str(self.output_dir / filename), self.fourcc, self.fps, self.frame_size)
        if not writer.isOpened():
            # Bad codec/path would otherwise yield a silent, empty file
            self.get_logger().error(
                f"Could not open VideoWriter for {filename} "
                f"(check fourcc '{self.get_parameter('fourcc').value}' and path).")
            self.video_writer = None
            self.current_label = None
            return
        self.video_writer = writer
        self.current_label = label
        self.get_logger().info(f"Started recording: {filename}")

    def _stop_recording(self):
        """Stop the current recording and flush the file."""
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Stopped recording for mission '{self.current_label}'")
            self.video_writer = None
            self.current_label = None

    # ------------------------------------------------------------------
    # Capture loop — records and republishes
    # ------------------------------------------------------------------

    def _capture_loop(self):
        while self.running:
            try:
                if self.cap is None or not self.cap.isOpened():
                    # Camera absent — quietly retry at a long interval so a
                    # missing camera doesn't spam the log.  The initial "not
                    # available" error was already logged once at startup.
                    try:
                        self.cap = cv2.VideoCapture(self._camera_index, cv2.CAP_V4L2)
                        if self.cap.isOpened():
                            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._req_width)
                            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._req_height)
                            self.cap.set(cv2.CAP_PROP_FPS, self._req_fps)
                            self.get_logger().info("Camera reconnected.")
                    except Exception:
                        pass
                    if self.cap is None or not self.cap.isOpened():
                        time.sleep(self._reconnect_interval)
                    continue

                ret, frame = self.cap.read()
                if not ret:
                    # Back off so a disconnected camera doesn't busy-spin / log-spam
                    self.get_logger().warn("Failed to capture frame", throttle_duration_sec=2.0)
                    time.sleep(0.1)
                    continue

                with self.recording_lock:
                    if self.video_writer is not None:
                        self.video_writer.write(frame)

                if self.publish_images:
                    now = self.get_clock().now().nanoseconds / 1e9
                    if (now - self._last_pub_time) >= self._pub_period:
                        self._last_pub_time = now
                        img = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                        img.header.stamp = self.get_clock().now().to_msg()
                        self.image_pub.publish(img)
            except Exception as e:
                # Never let one bad frame kill the capture thread (a dead thread
                # = silently blind perception + no recording for the rest of the run)
                self.get_logger().error(f"Capture loop error: {e}", throttle_duration_sec=2.0)
                time.sleep(0.1)

    def destroy_node(self):
        self.running = False
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        with self.recording_lock:
            if self.video_writer is not None:
                self._stop_recording()
        if self.cap.isOpened():
            self.cap.release()
        self.get_logger().info("Camera node shutdown complete")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StellarHDCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
