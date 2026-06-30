"""
crab.py — Mission Dispatcher & Configuration Master
===================================================
crab is the robot's deliberative layer.  It does two jobs and nothing else:

  1. Configuration master.  At launch it broadcasts the full robot
     configuration (actuator map, operating mode, rates, nominal gait) once on a
     latched topic.  The controller and any other node read it and build their
     own data structures — crab is the single source of truth for "what the
     robot is".

  2. Mission dispatcher.  It holds an unbounded FIFO queue of missions fed from
     a terminal/bash script, fires them one at a time to the controller, listens
     to high-level status coming back, and owns the retry / human-escalation
     policy.  It never touches sensors or servos.

Retry & escalation policy
-------------------------
When the controller reports a mission STUCK:
  * crab silently re-sends the mission up to ``max_retries`` times (default 2).
  * once those are exhausted it prompts the operator on the terminal and waits
    ``HUMAN_TIMEOUT`` seconds (10 s).  "y" grants a fresh budget of 2 retries;
    "n" or a timeout advances to the next mission.
When the controller reports ACHIEVED, crab advances to the next mission.
When the queue is empty, crab sends a HOVER mission so the robot holds station.

Mission intake format (one line on ``mission_input``)
-----------------------------------------------------
A mission is one of four kinds; the controller owns all sequencing (a heading
mission scans for its own tag(s), then heads — crab never micro-commands it):

    heading:<dir> velocity:<v> effort:<a> distance:<m> ...   swim a compass heading
    scan ...                                                 sweep / search only
    hover ...                                                hold station
    tag:<id> ...                                             legacy single-tag seek

  heading  (heading mission) one of N NE E SE S SW W NW.  Cardinals point at the
                      matching cardinal tag; intercardinals steer to the bisector
                      of the two adjacent cardinal tags (controller scans for both).
  distance (optional) arrival distance in metres — mission ACHIEVED when facing
                      the heading and within this of the reference tag
  velocity (optional) peak stroke rate rad/s; falls back to nominal gait_velocity
  effort   (optional) stroke amplitude rad; falls back to nominal gait_effort
  label    (optional) human-readable name (used to match status)
  retries  (optional) auto-retries before asking a human, default 2
  override (optional) none    : queue at the back (default)
                      discard : preempt current mission, drop it
                      requeue : preempt current mission, push it to the front

A bash feeder should pace lines ~1 every 3 s (see scripts/feed_missions.sh); the
queue itself is unbounded, so bursts are simply buffered.

Topics
------
Subscribes : mission_input   (std_msgs/String) from terminal / bash
             mission_status   (std_msgs/String) from controller
Publishes  : robot_config     (std_msgs/String, transient_local) to all nodes
             mission_cmd       (std_msgs/String) to controller
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import sys
import select
import threading
from collections import deque


class CrabMissionDispatcher(Node):

    HUMAN_TIMEOUT = 10.0     # s — how long to wait for an operator decision
    HUMAN_GRANT = 2          # retries granted when the operator says "y"
    VALID_HEADINGS = {'N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'}

    def __init__(self):
        super().__init__('crab_mission_dispatcher')

        # ------------------------------------------------------------------
        # Configuration parameters (this node is the source of truth).  Mirror
        # the structural settings the controller and hardware need to agree on.
        # ------------------------------------------------------------------
        # Actuator map entry: [id, homing_offset, set, min, max, custom?]
        #   homing_offset  rad written to the servo's Homing Offset register by
        #                  the hardware node so it reads 0 at mechanical home.
        #   set            servos sharing a set form one fin; within a set the
        #                  FIRST entry is the roll servo, the SECOND is pitch.
        #   min, max       mechanical position limits (rad).
        #   custom         optional spare per-servo value, passed through for a
        #                  motion_command function to use (unused for now).
        self.declare_parameter(
            'actuator_map',
            '[[4, 0.0, 1, -3.14, 3.14], [3, 0.0, 1, -1.57, 1.57]]'
        )
        self.declare_parameter('operating_mode', 'position')   # 'position' or 'velocity'
        self.declare_parameter('control_rate', 400.0)          # Hz
        self.declare_parameter('startup_delay', 10.0)          # s — deployment settling
        self.declare_parameter('gait_velocity', 3.77)          # rad/s — nominal peak stroke rate (2π·f·A)
        self.declare_parameter('gait_effort', 0.6)             # rad — nominal stroke amplitude
        self.declare_parameter('default_retries', 2)           # auto-retries per mission
        # Cardinal heading tags: which AprilTag id marks each compass direction.
        # The controller resolves a heading mission (N..NW) to these tag ids.
        self.declare_parameter('cardinal_map', '{"N": 0, "E": 1, "S": 2, "W": 3}')

        self.default_retries = int(self.get_parameter('default_retries').value)

        # ------------------------------------------------------------------
        # Mission state
        # ------------------------------------------------------------------
        self.queue = deque()             # pending missions (unbounded FIFO)
        self.current = None              # mission currently dispatched
        self.retries_used = 0            # STUCK retries spent on current mission
        self.awaiting_human = False      # blocked on operator decision?
        self.human_deadline = 0.0        # monotonic-ish deadline for the prompt
        self.hovering = False            # already told controller to hover (idle)
        self._stdin_lines = deque()      # lines captured by the reader thread
        self._lock = threading.Lock()

        # ------------------------------------------------------------------
        # ROS2 interfaces
        # ------------------------------------------------------------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.config_pub = self.create_publisher(String, 'robot_config', latched)
        self.mission_pub = self.create_publisher(String, 'mission_cmd', 10)
        self.create_subscription(String, 'mission_input', self._input_cb, 50)
        self.create_subscription(String, 'mission_status', self._status_cb, 50)

        self._publish_config()

        # Background stdin reader so the human prompt never blocks the executor
        self._stdin_thread = threading.Thread(target=self._stdin_reader, daemon=True)
        self._stdin_thread.start()

        # Housekeeping tick: dispatch when idle, enforce the human timeout
        self.create_timer(0.2, self._tick)

        self.get_logger().info("crab dispatcher ready — config published, awaiting missions.")

    # ======================================================================
    # Configuration broadcast
    # ======================================================================

    def _publish_config(self):
        """Broadcast the full robot configuration once on a latched topic."""
        actuator_map = json.loads(self.get_parameter('actuator_map').value)
        cardinal_map = json.loads(self.get_parameter('cardinal_map').value)
        cfg = {
            'actuator_map': actuator_map,
            'cardinal_map': cardinal_map,
            'operating_mode': self.get_parameter('operating_mode').value,
            'control_rate': self.get_parameter('control_rate').value,
            'startup_delay': self.get_parameter('startup_delay').value,
            'gait_velocity': self.get_parameter('gait_velocity').value,
            'gait_effort': self.get_parameter('gait_effort').value,
        }
        msg = String()
        msg.data = json.dumps(cfg)
        self.config_pub.publish(msg)
        self.get_logger().info(f"Published robot_config: {len(actuator_map)} servos, "
                               f"mode={cfg['operating_mode']}.")

    # ======================================================================
    # Mission intake (from bash / terminal)
    # ======================================================================

    def _input_cb(self, msg: String):
        """Parse one mission line and enqueue (or override) accordingly."""
        mission, override = self._parse_mission(msg.data)
        if mission is None:
            return

        if override == 'discard':
            # Drop whatever is running, start this immediately. Also cancels any
            # pending operator prompt for the mission being preempted.
            self.current = None
            self.awaiting_human = False
            self.queue.appendleft(mission)
            self.get_logger().info(f"[override:discard] '{mission['label']}' preempts current.")
        elif override == 'requeue':
            # Save current to the front, then this one ahead of it
            if self.current is not None:
                self.queue.appendleft(self.current)
            self.current = None
            self.awaiting_human = False
            self.queue.appendleft(mission)
            self.get_logger().info(f"[override:requeue] '{mission['label']}' preempts; "
                                   f"interrupted mission saved to front.")
        else:
            self.queue.append(mission)
            self.get_logger().info(f"Queued '{mission['label']}' "
                                   f"(queue depth {len(self.queue)}).")

    def _parse_mission(self, raw: str):
        """
        Parse one mission line into (mission, override).  Supported forms:
          heading:NE velocity:6 effort:0.6 distance:0.10   swim a compass heading
          scan                                             sweep / search only
          hover                                            hold station
          tag:3                                            legacy single-tag seek
        plus optional label:, retries:, override: on any of them.  The controller
        owns all sequencing — a heading mission scans for its tag(s) itself.
        """
        tokens, flags = {}, set()
        for part in raw.strip().split():
            if ':' in part:
                k, v = part.split(':', 1)
                tokens[k.lower()] = v
            else:
                flags.add(part.lower())

        if 'heading' in tokens:
            kind = 'heading'
        elif 'scan' in flags or tokens.get('kind') == 'scan':
            kind = 'scan'
        elif 'hover' in flags or tokens.get('kind') == 'hover':
            kind = 'hover'
        elif 'tag' in tokens:
            kind = 'tag'
        else:
            self.get_logger().error(
                f"Mission line has no heading/scan/hover/tag: {raw!r}")
            return None, None

        mission = {
            'kind': kind,
            'max_retries': int(tokens.get('retries', self.default_retries)),
        }
        if kind == 'heading':
            heading = tokens['heading'].upper()
            if heading not in self.VALID_HEADINGS:
                self.get_logger().error(f"Bad heading {heading!r} in: {raw!r}")
                return None, None
            mission['heading'] = heading
            mission['label'] = tokens.get('label', heading)
        elif kind == 'tag':
            try:
                tag = int(tokens['tag'])
            except ValueError:
                self.get_logger().error(f"Bad tag id in: {raw!r}")
                return None, None
            mission['target_tag_id'] = tag
            mission['label'] = tokens.get('label', f'tag{tag}')
        else:   # scan / hover
            mission['label'] = tokens.get('label', kind.upper())

        # Optional per-mission stroke + arrival overrides; absent → nominal.
        for key in ('velocity', 'effort', 'distance'):
            if key in tokens:
                try:
                    mission[key] = float(tokens[key])
                except ValueError:
                    self.get_logger().warn(f"Bad {key} in: {raw!r} — ignoring.")

        override = tokens.get('override', 'none').lower()
        if override not in ('none', 'discard', 'requeue'):
            override = 'none'
        return mission, override

    # ======================================================================
    # Status from controller
    # ======================================================================

    def _status_cb(self, msg: String):
        """React to interpreted mission status / events from the controller."""
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = status.get('event')
        if event is None:
            return   # plain progress update; nothing to decide

        label = status.get('label')
        if self.current is None or label != self.current['label']:
            return   # event for a mission we're no longer tracking

        if event == 'ACHIEVED':
            self.get_logger().info(f"'{label}' ACHIEVED — advancing.")
            self.current = None            # _tick dispatches the next one
        elif event == 'STUCK':
            self._handle_stuck(label)

    def _handle_stuck(self, label):
        """Auto-retry, then escalate to the operator once retries are spent."""
        if self.retries_used < self.current['max_retries']:
            self.retries_used += 1
            self.get_logger().warn(
                f"'{label}' STUCK — auto-retry {self.retries_used}/"
                f"{self.current['max_retries']}.")
            self._dispatch(self.current, fresh=False)
        else:
            self.awaiting_human = True
            self.human_deadline = self.get_clock().now().nanoseconds / 1e9 + self.HUMAN_TIMEOUT
            with self._lock:
                self._stdin_lines.clear()
            self.get_logger().warn(
                f"'{label}' STUCK and retries exhausted. "
                f"Retry? [y/N] ({int(self.HUMAN_TIMEOUT)}s to decide)")

    # ======================================================================
    # Housekeeping tick
    # ======================================================================

    def _tick(self):
        """Dispatch the next mission when idle; resolve any pending human prompt."""
        if self.awaiting_human:
            self._resolve_human()
            return

        if self.current is not None:
            return   # a mission is in flight

        if self.queue:
            self.hovering = False
            self.current = self.queue.popleft()
            self._dispatch(self.current, fresh=True)
        elif not self.hovering:
            # Nothing left to do — park the robot in a hover
            self._dispatch_hover()
            self.hovering = True

    def _resolve_human(self):
        """Check the operator's answer or the 10 s timeout."""
        # An override may have preempted the stuck mission while we waited.
        if self.current is None:
            self.awaiting_human = False
            return

        answer = None
        with self._lock:
            while self._stdin_lines:
                line = self._stdin_lines.popleft().strip().lower()
                if line in ('y', 'yes', 'n', 'no'):
                    answer = line
                    break

        now = self.get_clock().now().nanoseconds / 1e9
        timed_out = now >= self.human_deadline

        if answer in ('y', 'yes'):
            self.awaiting_human = False
            self.current['max_retries'] = self.HUMAN_GRANT
            self.retries_used = 0
            self.get_logger().info(
                f"Operator: retry '{self.current['label']}' "
                f"({self.HUMAN_GRANT} more attempts).")
            self._dispatch(self.current, fresh=False)
        elif answer in ('n', 'no') or timed_out:
            why = 'operator declined' if answer else 'timed out'
            self.get_logger().info(
                f"'{self.current['label']}' abandoned ({why}) — moving on.")
            self.awaiting_human = False
            self.current = None   # _tick advances next cycle

    # ======================================================================
    # Dispatch helpers
    # ======================================================================

    def _dispatch(self, mission, fresh=True):
        """Send a mission to the controller. ``fresh`` resets the retry counter."""
        if fresh:
            self.retries_used = 0
        payload = {
            'kind': mission['kind'],
            'label': mission['label'],
            'max_retries': mission['max_retries'],
        }
        # Forward only the fields this mission carries (kind-dependent + overrides).
        for key in ('target_tag_id', 'heading', 'velocity', 'effort', 'distance'):
            if key in mission:
                payload[key] = mission[key]
        msg = String()
        msg.data = json.dumps(payload)
        self.mission_pub.publish(msg)
        detail = mission.get('heading') or mission.get('target_tag_id') or mission['kind']
        self.get_logger().info(
            f"Dispatched '{mission['label']}' ({detail})"
            + ("" if fresh else " [retry]"))

    def _dispatch_hover(self):
        """Tell the controller to hold station (queue drained)."""
        msg = String()
        msg.data = json.dumps({'kind': 'hover', 'label': 'HOVER', 'max_retries': 0})
        self.mission_pub.publish(msg)
        self.get_logger().info("Queue empty — commanding HOVER.")

    # ======================================================================
    # Background stdin reader
    # ======================================================================

    def _stdin_reader(self):
        """
        Capture terminal input without blocking the ROS executor.  If stdin is
        not interactive (e.g. launched via ros2 launch), readline() returns ''
        at EOF — stop the thread so we don't busy-spin.  Operator prompts then
        simply time out and move on, which is the safe default.
        """
        if not sys.stdin or not sys.stdin.readable():
            return
        while rclpy.ok():
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.3)
                if not ready:
                    continue
                line = sys.stdin.readline()
                if line == '':
                    break   # EOF — stdin closed / non-interactive
                with self._lock:
                    self._stdin_lines.append(line)
            except (ValueError, OSError):
                break


def main(args=None):
    rclpy.init(args=args)
    node = CrabMissionDispatcher()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
