#!/usr/bin/env python3
"""
gazebo_stellarhd_interface.py — Hybrid Gazebo/Real StellarHD Interface
========================================================================
Automatically detects real StellarHD camera and uses it if available.

If real camera detected: records from real hardware
If real camera NOT detected: records from Gazebo camera

Recording is segmented per mission (one video file per mission label), mirroring
the hardware stellarhd_interface.

Topics
------
Subscribes: /camera/image_raw (sensor_msgs/Image) - from Gazebo (fallback)
            mission_cmd (std_msgs/String) - JSON mission; new label = new segment
            mission_status (std_msgs/String) - JSON status; idle stops recording
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import cv2
import threading
from pathlib import Path
from datetime import datetime


class GazeboStellarHDInterface(Node):
    """
    Hybrid StellarHD interface - auto-detects real camera, falls back to Gazebo.
    """
    
    def __init__(self):
        super().__init__('gazebo_stellarhd_interface')
        
        # Parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('gazebo_camera_topic', '/camera/image_raw')
        self.declare_parameter('output_directory', '/home/shafa/videos')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('fourcc', 'mp4v')
        self.declare_parameter('video_width', 1920)
        self.declare_parameter('video_height', 1080)
        
        # Try to detect real camera
        camera_index = self.get_parameter('camera_index').value
        width = self.get_parameter('video_width').value
        height = self.get_parameter('video_height').value
        fps = self.get_parameter('fps').value
        
        self.use_real_camera = False
        self.real_cap = None
        self.capture_thread = None
        
        self._detect_real_camera(camera_index, width, height, fps)
        
        # Common setup
        self.output_dir = Path(self.get_parameter('output_directory').value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        fourcc_str = self.get_parameter('fourcc').value
        self.fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
        self.fps = fps
        self.frame_size = None
        
        self.bridge = CvBridge()
        self.video_writer = None
        self.current_label = None
        self.recording_lock = threading.Lock()
        self.running = True
        
        # Subscribe to Gazebo camera (fallback)
        if not self.use_real_camera:
            gazebo_topic = self.get_parameter('gazebo_camera_topic').value
            self.image_sub = self.create_subscription(
                Image,
                gazebo_topic,
                self._gazebo_image_callback,
                10
            )
        
        # Subscribers for per-mission recording segmentation
        self.cmd_sub = self.create_subscription(
            String,
            'mission_cmd',
            self._mission_cmd_cb,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            'mission_status',
            self._mission_status_cb,
            10
        )
        
        if self.use_real_camera:
            self.get_logger().info(
                f"Using REAL StellarHD camera - recording to: {self.output_dir}"
            )
        else:
            self.get_logger().info(
                f"Using Gazebo simulated camera - recording to: {self.output_dir}"
            )
    
    def _detect_real_camera(self, camera_index, width, height, fps):
        """Detect if real camera is available."""
        try:
            cap = cv2.VideoCapture(camera_index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            
            if cap.isOpened():
                # Test read
                ret, frame = cap.read()
                if ret:
                    self.real_cap = cap
                    self.use_real_camera = True
                    self.frame_size = (frame.shape[1], frame.shape[0])
                    
                    # Start capture thread for real camera
                    self.capture_thread = threading.Thread(
                        target=self._real_camera_loop,
                        daemon=True
                    )
                    self.capture_thread.start()
                    
                    self.get_logger().info(
                        f"Detected REAL camera at /dev/video{camera_index} "
                        f"({self.frame_size[0]}x{self.frame_size[1]} @ {fps} fps)"
                    )
                else:
                    cap.release()
                    self.get_logger().info(
                        f"Camera {camera_index} exists but cannot read - using Gazebo"
                    )
            else:
                self.get_logger().info(
                    f"No camera at /dev/video{camera_index} - using Gazebo simulation"
                )
        
        except Exception as e:
            self.get_logger().info(f"Camera detection failed: {e} - using Gazebo simulation")
    
    def _real_camera_loop(self):
        """Continuous capture loop for REAL camera."""
        while self.running and self.real_cap:
            ret, frame = self.real_cap.read()
            
            if not ret:
                self.get_logger().warn("Real camera read failed")
                continue
            
            with self.recording_lock:
                if self.video_writer is not None:
                    self.video_writer.write(frame)
    
    def _gazebo_image_callback(self, msg: Image):
        """Receive frames from Gazebo camera (fallback)."""
        if self.use_real_camera:
            return  # Ignore Gazebo if using real camera
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.frame_size is None:
                self.frame_size = (frame.shape[1], frame.shape[0])
            
            with self.recording_lock:
                if self.video_writer is not None:
                    self.video_writer.write(frame)
        
        except Exception as e:
            self.get_logger().error(f"Gazebo image conversion error: {e}")
    
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
        """Start a new video recording for the given mission label."""
        if self.video_writer is not None:
            self._stop_recording()

        if self.frame_size is None:
            self.get_logger().warn("Cannot start recording - no frames received yet")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        source = "real" if self.use_real_camera else "sim"
        safe = ''.join(c if c.isalnum() else '_' for c in str(label))
        filename = f"mission_{safe}_{timestamp}_{source}.mp4"
        filepath = self.output_dir / filename

        self.video_writer = cv2.VideoWriter(
            str(filepath),
            self.fourcc,
            self.fps,
            self.frame_size
        )

        self.current_label = label
        self.get_logger().info(f"Started recording: {filename}")

    def _stop_recording(self):
        """Stop current recording."""
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Stopped recording for mission '{self.current_label}'")
            self.video_writer = None
            self.current_label = None
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.running = False
        
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        
        with self.recording_lock:
            if self.video_writer is not None:
                self._stop_recording()
        
        if self.real_cap and self.real_cap.isOpened():
            self.real_cap.release()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboStellarHDInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
