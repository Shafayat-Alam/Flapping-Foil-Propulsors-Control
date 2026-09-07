"""
load_cell_interface.py — Load-Cell Array UDP Interface Node
===========================================================
Receives a load-cell force array streamed over UDP and republishes it on a ROS2
topic so the rest of the stack (and the session recorder) can consume it.

The sender pushes one UDP datagram per packet: an AXES×SAMPLES grid of
big-endian float32 values (row-major).  Each column is one time-step's 6-axis
force/torque reading — [Fx, Fy, Fz, Tx, Ty, Tz] — and a packet batches SAMPLES
of them.  We transpose to SAMPLES×AXES so each published row is one full F/T
reading, and publish the flattened grid as a Float32MultiArray with the shape
carried in the layout dimensions (outer = 'sample', inner = 'axis').

Using Float32MultiArray (instead of a custom LoadCell message) keeps this node
self-contained — no extra interface package to build — and the session recorder
already decodes Float32MultiArray, so the data lands in the CSV automatically
(sample s, axis a → column s<s>_<axis>, e.g. s0_Fx ... s19_Tz).

Communication
-------------
Publishes: <topic>  (std_msgs/Float32MultiArray) — flattened SAMPLES×AXES grid;
           layout.dim = [ {label:'sample', size:SAMPLES}, {label:'axis', size:AXES} ]
"""

import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


class LoadCellInterface(Node):
    """Bind a UDP port, decode the load-cell grid, republish on a ROS2 topic."""

    def __init__(self):
        super().__init__('load_cell_interface')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('rows', 6)            # incoming grid rows = F/T axes (Fx..Tz)
        self.declare_parameter('cols', 1000)         # incoming grid cols = samples per packet
        # NOTE: this MUST exactly match LabVIEW's UDP packet size (rows*cols*4
        # bytes) — _rx_loop drops any packet whose length differs at all, so a
        # mismatch here doesn't degrade the data, it goes silent.  LabVIEW now
        # sends 1000 samples/packet (6*1000*4 = 24000 bytes) at a 10 kHz sample
        # rate (10 packets/s).  Change only if the LabVIEW side changes.
        self.declare_parameter('topic', 'load_cell_data')
        # Sensor sample rate (Hz): the 'cols' samples in each packet are
        # 1/sample_rate apart in time (10 kHz → 100 µs between samples, so a
        # 1000-sample packet spans 100 ms).  Recorded as metadata for later
        # per-sample time reconstruction; does not affect decoding.
        self.declare_parameter('sample_rate', 10000.0)

        # Force/torque axis order within each 6-value sample (the incoming rows).
        self.AXES = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']

        self.rows = int(self.get_parameter('rows').value)
        self.cols = int(self.get_parameter('cols').value)
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self._port = int(self.get_parameter('udp_port').value)
        self._addr = self.get_parameter('bind_address').value
        # Each datagram is ROWS*COLS big-endian float32s (4 bytes each).
        self._n = self.rows * self.cols
        self._expected_bytes = self._n * 4
        self._unpack = struct.Struct(f'>{self._n}f')   # big-endian float32 grid

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('topic').value, 10)

        # ------------------------------------------------------------------
        # UDP socket — non-fatal: a bind failure logs an error and the node
        # keeps running (retrying the bind) so it never crashes the launch.
        # ------------------------------------------------------------------
        self.sock = None
        self._bind_warned = False
        self._open_socket()

        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        dt_us = 1e6 / self.sample_rate if self.sample_rate > 0 else 0.0
        self.get_logger().info(
            f"Load-cell interface publishing '{self.get_parameter('topic').value}' "
            f"({self.cols} samples × {self.rows} axes [Fx..Tz] per packet, "
            f"{self.sample_rate:.0f} Hz → {dt_us:.1f} µs/sample) "
            f"from UDP {self._addr}:{self._port}.")

    def _open_socket(self):
        """(Re)bind the UDP socket; non-fatal. Returns True on success."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self._addr, self._port))
            sock.settimeout(1.0)   # so the rx loop can notice shutdown
            self.sock = sock
            self._bind_warned = False
            self.get_logger().info(f"Bound UDP {self._addr}:{self._port}.")
            return True
        except OSError as e:
            self.sock = None
            if not self._bind_warned:
                self.get_logger().error(
                    f"Cannot bind UDP {self._addr}:{self._port} ({e}) — running "
                    f"WITHOUT load-cell data; will keep retrying.")
                self._bind_warned = True
            return False

    def _rx_loop(self):
        while self.running:
            if self.sock is None:
                # Socket down — retry the bind, backing off between attempts.
                if not self._open_socket():
                    threading.Event().wait(2.0)
                continue
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as e:
                self.get_logger().error(
                    f"UDP receive error ({e}) — reopening socket.")
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None
                continue

            if len(data) != self._expected_bytes:
                self.get_logger().warning(
                    f"Bad packet size: {len(data)} (expected {self._expected_bytes})")
                continue

            # Decode row-major AXES×SAMPLES, transpose to SAMPLES×AXES so each
            # output row is one time-step's full [Fx, Fy, Fz, Tx, Ty, Tz].
            values = self._unpack.unpack(data)   # tuple of AXES*SAMPLES floats
            transposed = []
            for s in range(self.cols):           # cols = samples per packet
                for a in range(self.rows):       # rows = 6 F/T axes
                    transposed.append(values[a * self.cols + s])

            msg = Float32MultiArray()
            msg.layout = MultiArrayLayout(dim=[
                MultiArrayDimension(label='sample', size=self.cols,
                                    stride=self.cols * self.rows),
                MultiArrayDimension(label='axis', size=self.rows, stride=self.rows),
            ], data_offset=0)
            msg.data = transposed
            self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=2.0)
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoadCellInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
