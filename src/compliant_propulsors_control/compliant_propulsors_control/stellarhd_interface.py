"""
stellarhd_camera.py — StellarHD Camera Interface Node
======================================================
DWE StellarHD camera interface for synchronized video recording.
https://dwe.ai/products/stellarhd

Records video continuously and segments recordings based on robot commands.
Each command execution (from start to finish) is saved as a separate video file.

Communication
-------------
Subscribes: robot_cmd  (std_msgs/String)   - to detect command start
            telemetry  (std_msgs/Float32MultiArray) - to track command completion
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import cv2
import threading
from pathlib import Path
from datetime import datetime


class StellarHDCameraNode(Node):
    """
    StellarHD camera node.
    
    Continuously captures video and saves segments for each robot command execution.
    Video files are named with command ID and timestamp.
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
        self.declare_parameter('output_directory', '/home/claude/videos')
        self.declare_parameter('fourcc', 'mp4v')  # Codec: 'mp4v', 'XVID', 'H264'
        
        # ------------------------------------------------------------------
        # Initialize camera
        # ------------------------------------------------------------------
        camera_index = self.get_parameter('camera_index').value
        width = self.get_parameter('video_width').value
        height = self.get_parameter('video_height').value
        fps = self.get_parameter('fps').value
        
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {camera_index}")
            raise RuntimeError("Camera initialization failed")
        
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(
            f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps} fps"
        )
        
        # ------------------------------------------------------------------
        # Video recording state
        # ------------------------------------------------------------------
        self.output_dir = Path(self.get_parameter('output_directory').value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        fourcc_str = self.get_parameter('fourcc').value
        self.fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
        self.fps = fps
        self.frame_size = (actual_width, actual_height)
        
        self.video_writer = None
        self.current_cmd_id = None
        self.recording_lock = threading.Lock()
        
        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.cmd_sub = self.create_subscription(
            String,
            'robot_cmd',
            self._cmd_callback,
            10
        )
        
        self.telemetry_sub = self.create_subscription(
            Float32MultiArray,
            'telemetry',
            self._telemetry_callback,
            10
        )
        
        # ------------------------------------------------------------------
        # Capture thread
        # ------------------------------------------------------------------
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        
        self.get_logger().info(f"Recording videos to: {self.output_dir}")
    
    def _cmd_callback(self, msg: String):
        """
        Parse robot_cmd to extract command ID and start new video recording.
        """
        try:
            # Parse cmd_id:[value] from message
            raw = msg.data.lower()
            if 'cmd_id:[' not in raw:
                return
            
            cmd_id_str = raw.split('cmd_id:[')[1].split(']')[0]
            cmd_id = float(cmd_id_str)
            
            # Start new recording for this command
            self._start_recording(cmd_id)
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse cmd_id: {e}")
    
    def _telemetry_callback(self, msg: Float32MultiArray):
        """
        Monitor telemetry to detect command completion.
        Telemetry format: [cmd_id, timestamp, ...]
        """
        if len(msg.data) < 1:
            return
        
        telemetry_cmd_id = msg.data[0]
        
        # Check if command has changed (indicating previous command finished)
        with self.recording_lock:
            if self.current_cmd_id is not None and telemetry_cmd_id != self.current_cmd_id:
                self._stop_recording()
    
    def _start_recording(self, cmd_id: float):
        """Start recording a new video for the given command ID."""
        with self.recording_lock:
            # Stop any existing recording
            if self.video_writer is not None:
                self._stop_recording()
            
            # Create new video file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"cmd_{int(cmd_id)}_{timestamp}.mp4"
            filepath = self.output_dir / filename
            
            self.video_writer = cv2.VideoWriter(
                str(filepath),
                self.fourcc,
                self.fps,
                self.frame_size
            )
            
            self.current_cmd_id = cmd_id
            
            self.get_logger().info(f"Started recording: {filename}")
    
    def _stop_recording(self):
        """Stop current recording and save video file."""
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Stopped recording for cmd_id={self.current_cmd_id}")
            self.video_writer = None
            self.current_cmd_id = None
    
    def _capture_loop(self):
        """Continuous frame capture loop (runs in separate thread)."""
        while self.running:
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn("Failed to capture frame")
                continue
            
            # Write frame to current video if recording
            with self.recording_lock:
                if self.video_writer is not None:
                    self.video_writer.write(frame)
    
    def destroy_node(self):
        """Cleanup on shutdown."""
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
