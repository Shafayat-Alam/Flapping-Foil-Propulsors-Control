"""
apriltag_interface.py — AprilTag Perception Node
================================================
Turns camera frames into the heading cues the controller needs.  It detects
AprilTags with OpenCV's ArUco/AprilTag detector, estimates each tag's pose, and
publishes a compact ``apriltag_detections`` array.  The controller consumes only
this interpreted output — it never sees raw images.

This node is the single owner of the camera frames it processes, so it can take
them from one of two sources (``source`` parameter):
  * ``camera`` : open a local capture device (real hardware)
  * ``topic``  : subscribe to a sensor_msgs/Image (Gazebo / replay)

Distance/bearing need camera intrinsics and the physical tag size — set the
``fx, fy, cx, cy`` and ``tag_size`` parameters from a calibration for real runs.

Topic
-----
Publishes : apriltag_detections (std_msgs/Float32MultiArray)
            [tag_id, distance_m, bearing_rad, elevation_rad, valid] per tag
            bearing > 0 = tag to the left, elevation > 0 = tag above centre,
            valid = 1.0 for a real detection.

When no tag is visible an empty array is published, which the controller treats
as "no detection" (and keeps its current search motion going).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import math
import numpy as np

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    HAVE_BRIDGE = True
except ImportError:
    HAVE_BRIDGE = False


class AprilTagNode(Node):

    def __init__(self):
        super().__init__('apriltag_interface')

        # Source + capture
        self.declare_parameter('source', 'camera')          # 'camera' or 'topic'
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detect_rate', 30.0)         # Hz (camera source)
        self.declare_parameter('tag_family', 'tag36h11')

        # Camera intrinsics + tag geometry (calibrate for real distances)
        self.declare_parameter('fx', 1000.0)
        self.declare_parameter('fy', 1000.0)
        self.declare_parameter('cx', 960.0)
        self.declare_parameter('cy', 540.0)
        self.declare_parameter('tag_size', 0.10)            # m, edge length

        self.source = self.get_parameter('source').value
        self.tag_size = self.get_parameter('tag_size').value
        self.camera_matrix = np.array([
            [self.get_parameter('fx').value, 0.0, self.get_parameter('cx').value],
            [0.0, self.get_parameter('fy').value, self.get_parameter('cy').value],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        self.pub = self.create_publisher(Float32MultiArray, 'apriltag_detections', 10)

        # Set before any early return so destroy_node() can always release safely
        self._cap = None

        if not HAVE_CV2:
            self.get_logger().error("OpenCV not available — AprilTag detection disabled.")
            return

        self._init_detector(self.get_parameter('tag_family').value)
        self._bridge = CvBridge() if HAVE_BRIDGE else None
        # IPPE_SQUARE is purpose-built for a planar square target and is far more
        # stable than the default iterative solver; fall back if it's unavailable.
        self._pnp_flag = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)

        if self.source == 'topic':
            if not HAVE_BRIDGE:
                self.get_logger().error("cv_bridge not available — cannot use topic source.")
                return
            topic = self.get_parameter('image_topic').value
            self.create_subscription(Image, topic, self._image_cb, 10)
            self.get_logger().info(f"AprilTag node ready — subscribed to {topic}.")
        else:
            idx = self.get_parameter('camera_index').value
            self._cap = cv2.VideoCapture(idx)
            if not self._cap.isOpened():
                self.get_logger().error(f"Could not open camera {idx}.")
            rate = self.get_parameter('detect_rate').value
            self.create_timer(1.0 / rate, self._capture_tick)
            self.get_logger().info(f"AprilTag node ready — capturing /dev/video{idx}.")

    # ------------------------------------------------------------------
    def _init_detector(self, family):
        """Build an ArUco AprilTag detector across OpenCV API versions."""
        family_map = {
            'tag36h11': getattr(cv2.aruco, 'DICT_APRILTAG_36h11', None),
            'tag25h9': getattr(cv2.aruco, 'DICT_APRILTAG_25h9', None),
            'tag16h5': getattr(cv2.aruco, 'DICT_APRILTAG_16h5', None),
        }
        dict_id = family_map.get(family) or cv2.aruco.DICT_APRILTAG_36h11
        self._dict = cv2.aruco.getPredefinedDictionary(dict_id)
        # OpenCV >= 4.7 uses ArucoDetector; older uses module-level detectMarkers
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(self._dict, params)
        else:
            self._detector = None
            self._params = cv2.aruco.DetectorParameters_create()

    def _detect(self, gray):
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        return corners, ids

    # ------------------------------------------------------------------
    def _capture_tick(self):
        if self._cap is None or not self._cap.isOpened():
            return
        ok, frame = self._cap.read()
        if ok:
            self._process(frame)

    def _image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert incoming image: {e}")
            return
        self._process(frame)

    # ------------------------------------------------------------------
    def _process(self, frame):
        """Detect tags, estimate pose, and publish the detection array."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)

        payload = []
        if ids is not None and len(ids) > 0:
            half = self.tag_size / 2.0
            # Tag corner model in the tag's own frame (matches ArUco corner order)
            obj = np.array([
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
                [-half, -half, 0.0],
            ], dtype=np.float64)
            for tag_id, corner in zip(ids.flatten(), corners):
                img_pts = corner.reshape(4, 2).astype(np.float64)
                ok, _rvec, tvec = cv2.solvePnP(
                    obj, img_pts, self.camera_matrix, self.dist_coeffs,
                    flags=self._pnp_flag)
                if not ok:
                    continue
                tx, ty, tz = (float(v) for v in tvec.reshape(3))
                distance = math.sqrt(tx * tx + ty * ty + tz * tz)
                bearing = math.atan2(-tx, tz)     # +left
                elevation = math.atan2(-ty, tz)   # +up
                payload.extend([float(tag_id), distance, bearing, elevation, 1.0])

        msg = Float32MultiArray()
        msg.data = payload
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
