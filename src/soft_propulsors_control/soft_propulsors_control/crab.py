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

Answering y/n prompts
---------------------
Publish on ``/operator_response`` rather than typing into the launch
terminal — ros2 launch does not reliably route interactive keystrokes to any
one child process's stdin when running multiple nodes from a single launch
file, so raw terminal input cannot be trusted here (stdin reading is still
attempted as a fallback, but don't rely on it):
    ros2 topic pub --once /operator_response std_msgs/String "data: 'y'"

Retry & escalation policy
-------------------------
When the controller reports a mission STUCK:
  * crab silently re-sends the mission up to ``max_retries`` times (default 2).
  * once those are exhausted it prompts the operator (see "Answering y/n
    prompts" above) and waits ``HUMAN_TIMEOUT`` seconds (10 s).  "y" grants a
    fresh budget of 2 retries; "n" or a timeout advances to the next mission.
When the controller reports ACHIEVED, crab advances to the next mission.
When the queue is empty, crab sends a HOVER mission so the robot holds station.

Mission intake format (one line on ``mission_input``)
-----------------------------------------------------
A mission is one of six kinds; the controller owns all sequencing (a heading
mission scans for its own tag(s), then heads — crab never micro-commands it):

    heading:<dir> velocity:<v> effort:<a> distance:<m> ...   swim a compass heading
    scan ...                                                 sweep / search only
    hover ...                                                hold station (IMU-stabilised)
    home_state ...                                           drive every servo to 0 rad
    standby ...                                              mid-range rest pose (roll π, pitch π/2)
    tag:<id> ...                                             legacy single-tag seek

Init sequence (mandatory, runs before the queue):
  home_state runs first, interactively, one servo at a time — sets in
  ascending order, roll then pitch within each set.  For each servo: drive it
  to 0, confirm via feedback, then print a y/n prompt (answer via
  /operator_response — see "Answering y/n prompts" above) asking the operator
  to visually confirm it.  "y" advances to the next servo; "n"
  restarts the whole walk from the first servo (already-correct servos won't
  actually move again, since a servo already at its commanded position is
  never re-sent — see _command_targets).  Once every servo is confirmed, crab
  waits ``operational_readiness`` seconds, dispatches ``standby``, waits
  ``mission_readiness`` seconds, then begins processing mission_input.
  Neither init mission can be preempted; anything sent early just queues
  behind them.  ``home_state`` and ``standby`` can also be sent any time
  afterward, on demand (home_state sent this way targets every servo at once,
  not the interactive per-servo walk).

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
        # Actuator map entry: [id, set, custom?]
        #   Homing is calibrated once on the servo itself (e.g. via Dynamixel
        #   Wizard) and never touched by this stack — Present Position 0 is
        #   always trusted as home.
        #   set            servos sharing a set form one fin; within a set the
        #                  FIRST entry is the roll servo, the SECOND is pitch.
        #   custom         optional spare per-servo value, passed through for a
        #                  motion_command function to use (unused for now).
        #   Position limits are fixed by role in the controller (roll 0..2π,
        #   pitch 0..π), not carried here.
        self.declare_parameter(
            'actuator_map',
            '[[4, 1], [3, 1]]'
        )
        self.declare_parameter('operating_mode', 'position')   # 'position' or 'velocity'
        self.declare_parameter('control_rate', 400.0)          # Hz
        # Seconds to wait, after home_state completes, before dispatching the
        # mandatory standby pose during the init sequence.
        self.declare_parameter('operational_readiness', 10.0)
        # Seconds to wait, after standby completes, before crab starts
        # processing mission_input — the final gate before "operational".
        self.declare_parameter('mission_readiness', 5.0)
        self.declare_parameter('gait_velocity', 3.77)          # rad/s — nominal peak stroke rate (2π·f·A)
        self.declare_parameter('gait_effort', 0.6)             # rad — nominal stroke amplitude
        self.declare_parameter('default_retries', 2)           # auto-retries per mission
        # Cardinal heading tags: which AprilTag id marks each compass direction.
        # The controller resolves a heading mission (N..NW) to these tag ids.
        self.declare_parameter('cardinal_map', '{"N": 0, "E": 1, "S": 2, "W": 3}')

        self.default_retries = int(self.get_parameter('default_retries').value)
        self.operational_readiness = float(self.get_parameter('operational_readiness').value)
        self.mission_readiness = float(self.get_parameter('mission_readiness').value)

        # ------------------------------------------------------------------
        # Mission state
        # ------------------------------------------------------------------
        self.queue = deque()             # pending missions (unbounded FIFO)
        self.current = None              # mission currently dispatched
        self.retries_used = 0            # STUCK retries spent on current mission
        self.awaiting_human = False      # blocked on operator decision?
        self.human_deadline = 0.0        # monotonic-ish deadline for the prompt
        self.hovering = False            # already told controller to hover (idle)
        # Mandatory init sequence, gated ahead of the real queue:
        #   home_state → (wait operational_readiness s) → standby →
        #   (wait mission_readiness s) → done
        # Missions from mission_input queue up but aren't dispatched until 'done'.
        self.init_phase = 'home_state'
        # 'home_state' | 'waiting_operational' | 'standby' | 'waiting_mission' | 'done'
        self.init_wait_deadline = None   # when the current wait phase ends (None = not started)
        self._init_dispatched = False    # current init mission has been sent (vs. not yet)
        # Interactive per-servo home_state/standby walk: sets in ascending
        # order, roll then pitch within each set (see _build_home_walk_order).
        # Both walks reuse the same ordering, just with different targets.
        self.home_walk_order = self._build_home_walk_order()   # [(sid, set_id, role), ...]
        self.home_walk_index = 0            # which servo in home_walk_order is being homed
        self.awaiting_home_confirm = False  # blocked on operator y/n for the current servo
        self.standby_walk_index = 0             # which servo is being set to standby pose
        self.awaiting_standby_confirm = False   # blocked on operator y/n for the current servo
        self._stdin_lines = deque()      # lines captured by the reader thread
        self._lock = threading.Lock()

        # ------------------------------------------------------------------
        # ROS2 interfaces
        # ------------------------------------------------------------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.config_pub = self.create_publisher(String, 'robot_config', latched)
        # TRANSIENT_LOCAL (not the default volatile 10-depth queue): if crab
        # dispatches the first mission before the controller process has
        # finished starting up and subscribed, a volatile publish would be
        # lost forever (no replay), silently stalling the whole init sequence.
        # Latching the last mission_cmd guarantees a late-joining or
        # slow-starting controller still receives it.
        self.mission_pub = self.create_publisher(String, 'mission_cmd', latched)
        self.create_subscription(String, 'mission_input', self._input_cb, 50)
        # Must match controller's publisher QoS (TRANSIENT_LOCAL) — see the
        # comment on mission_pub above; same race, opposite direction.
        self.create_subscription(String, 'mission_status', self._status_cb, latched)
        # Answer y/n prompts (STUCK-retry, home_state/standby walk confirm)
        # via a topic, not raw terminal stdin — ros2 launch does not reliably
        # route interactive keystrokes to any one child process's stdin when
        # running multiple nodes from one launch file, so stdin alone cannot
        # be trusted here.  e.g.:
        #   ros2 topic pub --once /operator_response std_msgs/String "data: 'y'"
        self.create_subscription(String, 'operator_response', self._operator_response_cb, 10)

        self._publish_config()

        # Background stdin reader so the human prompt never blocks the executor
        self._stdin_thread = threading.Thread(target=self._stdin_reader, daemon=True)
        self._stdin_thread.start()

        # Housekeeping tick: dispatch when idle, enforce the human timeout.
        # Fast enough that answering a y/n prompt (home_state/standby walk,
        # STUCK-retry) feels immediate rather than polled.
        self.create_timer(0.05, self._tick)

        self.get_logger().info("crab dispatcher ready — config published, awaiting missions.")

    # ======================================================================
    # Configuration broadcast
    # ======================================================================

    def _build_home_walk_order(self):
        """
        Build the ordered [(sid, set_id, role), ...] list for the interactive
        per-servo home_state walk: sets in ascending order, and within each
        set, roll (first entry) then pitch (second entry) — the same
        first-is-roll/second-is-pitch convention the controller uses to pair
        fins from the actuator map.
        """
        actuator_map = json.loads(self.get_parameter('actuator_map').value)
        sets = {}   # set_id -> [sid, ...] in map order
        for entry in actuator_map:
            sid = int(round(entry[0]))
            set_id = int(entry[1])
            sets.setdefault(set_id, []).append(sid)

        order = []
        roles = ('roll', 'pitch')
        for set_id in sorted(sets):
            for i, sid in enumerate(sets[set_id]):
                role = roles[i] if i < len(roles) else f'extra{i}'
                order.append((sid, set_id, role))
        return order

    def _publish_config(self):
        """Broadcast the full robot configuration once on a latched topic."""
        actuator_map = json.loads(self.get_parameter('actuator_map').value)
        cardinal_map = json.loads(self.get_parameter('cardinal_map').value)
        cfg = {
            'actuator_map': actuator_map,
            'cardinal_map': cardinal_map,
            'operating_mode': self.get_parameter('operating_mode').value,
            'control_rate': self.get_parameter('control_rate').value,
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

        # The mandatory init sequence (home_state → wait → standby) is never
        # preemptible — while it's still running, an override just queues
        # normally behind it instead.
        if override in ('discard', 'requeue') and self.init_phase != 'done':
            override = 'none'

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
        elif 'home_state' in flags or tokens.get('kind') == 'home_state':
            kind = 'home_state'
        elif 'standby' in flags or tokens.get('kind') == 'standby':
            kind = 'standby'
        elif 'tag' in tokens:
            kind = 'tag'
        else:
            self.get_logger().error(
                f"Mission line has no heading/scan/hover/home_state/standby/tag: {raw!r}")
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
        else:   # scan / hover / home_state / standby
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

        # Mandatory init sequence gates everything until it reports 'done'.
        if self.init_phase != 'done':
            self._tick_init()
            return

        if self.current is not None:
            return   # a mission is in flight

        if self.queue:
            self.hovering = False
            self.current = self.queue.popleft()
            self._dispatch(self.current, fresh=True)
        # --- HOVER disabled for now; controller just sits in WAIT (neutral)
        # --- when the queue is empty.  Restore to re-enable:
        # elif not self.hovering:
        #     # Nothing left to do — park the robot in a hover
        #     self._dispatch_hover()
        #     self.hovering = True

    def _tick_init(self):
        """
        Drive the mandatory init sequence:
            home_state (interactive, per-servo, y/n-confirmed) →
            (wait operational_readiness s) →
            standby (interactive, per-servo, y/n-confirmed) →
            (wait mission_readiness s) → done
        Each init mission runs through the normal STUCK/retry/human path; we
        advance only once ``current`` clears (ACHIEVED, or abandoned by the
        operator).  ``_init_dispatched`` distinguishes "not sent yet" from
        "sent and now finished" while ``current`` is None in both.
        """
        now = self.get_clock().now().nanoseconds / 1e9

        if self.init_phase == 'home_state':
            self._tick_home_state_walk()

        elif self.init_phase == 'waiting_operational':
            # Deadline may be unset if we started here (home_state disabled) —
            # begin the countdown on first entry so it runs from now, not launch.
            if self.init_wait_deadline is None:
                self.init_wait_deadline = now + self.operational_readiness
                self.get_logger().info(
                    f"Init: waiting {self.operational_readiness:.1f}s "
                    f"(operational_readiness) before standby.")
            elif now >= self.init_wait_deadline:
                self.init_phase = 'standby'

        elif self.init_phase == 'standby':
            self._tick_standby_walk()

        elif self.init_phase == 'waiting_mission':
            if now >= self.init_wait_deadline:
                self.init_phase = 'done'
                self.get_logger().info("=== SYSTEM OPERATIONAL ===")

    def _tick_home_state_walk(self):
        """
        Walk home_walk_order one servo at a time: dispatch a single-servo
        home_state mission, let it run through the normal STUCK/retry/human
        path to ACHIEVED (or abandonment), then ask the operator to visually
        confirm that specific servo before moving on.  'n' restarts the whole
        walk from servo 0 — cheap, since a servo already at its commanded
        position is never re-sent (see controller._command_targets), so
        already-good servos just re-confirm instantly without moving again.
        """
        if self.awaiting_home_confirm:
            self._resolve_home_confirm()
            return

        if self.current is None and not self._init_dispatched:
            sid, set_id, role = self.home_walk_order[self.home_walk_index]
            self.hovering = False
            self.current = self._make_init_mission(
                'home_state', f'HOME_SET{set_id}_{role.upper()}_{sid}',
                target_servo_id=sid)
            self._dispatch(self.current, fresh=True)
            self._init_dispatched = True
            return

        if self.current is None and self._init_dispatched:
            # This servo reported ACHIEVED (or was abandoned via the normal
            # STUCK/retry/human path) — ask the operator to confirm it.
            self._init_dispatched = False
            self.awaiting_home_confirm = True
            with self._lock:
                self._stdin_lines.clear()
            sid, set_id, role = self.home_walk_order[self.home_walk_index]
            self.get_logger().warn(
                f"Servo {sid} (set {set_id}, {role}) homed to 0 — "
                f"look correct? [y/n]")

    def _resolve_home_confirm(self):
        """Check the operator's y/n answer for the current home_state servo."""
        answer = None
        with self._lock:
            pending = list(self._stdin_lines)
            while self._stdin_lines:
                line = self._stdin_lines.popleft().strip().lower()
                if line in ('y', 'yes', 'n', 'no'):
                    answer = line
                    break
        if pending:
            self.get_logger().info(f"[home_state confirm] stdin lines seen: {pending!r} -> answer={answer!r}")

        if answer in ('y', 'yes'):
            self.awaiting_home_confirm = False
            self.home_walk_index += 1
            self.get_logger().info(
                f"[home_state confirm] 'y' — advancing to walk index {self.home_walk_index} "
                f"of {len(self.home_walk_order)}.")
            if self.home_walk_index >= len(self.home_walk_order):
                now = self.get_clock().now().nanoseconds / 1e9
                self.init_phase = 'waiting_operational'
                self.init_wait_deadline = now + self.operational_readiness
                self.get_logger().info(
                    f"home_state walk complete — all servos confirmed. Waiting "
                    f"{self.operational_readiness:.1f}s (operational_readiness) before standby.")
        elif answer in ('n', 'no'):
            self.awaiting_home_confirm = False
            self.home_walk_index = 0
            self.get_logger().warn(
                "Operator rejected — restarting home_state walk from servo 1.")
        # else: no answer yet this tick — keep waiting (no timeout, by design;
        # this is a deliberate visual check, not a failure-recovery prompt).

    def _tick_standby_walk(self):
        """
        Same mechanics as _tick_home_state_walk, for the standby rest pose:
        walk home_walk_order one servo at a time, dispatch a single-servo
        standby mission (controller resolves the correct roll/pitch target
        per servo, mirrored for set 1), wait ACHIEVED, ask y/n, advance or
        restart from servo 0 on 'n'.
        """
        if self.awaiting_standby_confirm:
            self._resolve_standby_confirm()
            return

        if self.current is None and not self._init_dispatched:
            sid, set_id, role = self.home_walk_order[self.standby_walk_index]
            self.current = self._make_init_mission(
                'standby', f'STANDBY_SET{set_id}_{role.upper()}_{sid}',
                target_servo_id=sid)
            self._dispatch(self.current, fresh=True)
            self._init_dispatched = True
            return

        if self.current is None and self._init_dispatched:
            self._init_dispatched = False
            self.awaiting_standby_confirm = True
            with self._lock:
                self._stdin_lines.clear()
            sid, set_id, role = self.home_walk_order[self.standby_walk_index]
            self.get_logger().warn(
                f"Servo {sid} (set {set_id}, {role}) set to standby pose — "
                f"look correct? [y/n]")

    def _resolve_standby_confirm(self):
        """Check the operator's y/n answer for the current standby servo."""
        answer = None
        with self._lock:
            while self._stdin_lines:
                line = self._stdin_lines.popleft().strip().lower()
                if line in ('y', 'yes', 'n', 'no'):
                    answer = line
                    break

        if answer in ('y', 'yes'):
            self.awaiting_standby_confirm = False
            self.standby_walk_index += 1
            if self.standby_walk_index >= len(self.home_walk_order):
                now = self.get_clock().now().nanoseconds / 1e9
                self.init_phase = 'waiting_mission'
                self.init_wait_deadline = now + self.mission_readiness
                self.get_logger().info(
                    f"standby walk complete — all servos confirmed. Waiting "
                    f"{self.mission_readiness:.1f}s (mission_readiness) before going operational.")
        elif answer in ('n', 'no'):
            self.awaiting_standby_confirm = False
            self.standby_walk_index = 0
            self.get_logger().warn(
                "Operator rejected — restarting standby walk from servo 1.")
        # else: no answer yet this tick — keep waiting (no timeout, by design).

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
        for key in ('target_tag_id', 'heading', 'velocity', 'effort', 'distance',
                    'target_servo_id'):
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

    def _make_init_mission(self, kind, label, **extra):
        """Build one mandatory init mission (home_state or standby)."""
        m = {
            'kind': kind,
            'label': label,
            'max_retries': self.default_retries,
            'init': True,
        }
        m.update(extra)
        return m

    # ======================================================================
    # Operator responses
    # ======================================================================

    def _operator_response_cb(self, msg: String):
        """
        Primary path for answering y/n prompts: publish on /operator_response
        instead of typing into the launch terminal (see the subscription
        comment in __init__ for why stdin alone isn't reliable under
        ros2 launch).  Feeds the same _stdin_lines queue the stdin reader
        uses, so every existing y/n consumer (_resolve_human,
        _resolve_home_confirm, _resolve_standby_confirm) picks it up as-is.
        """
        self.get_logger().info(f"[operator_response] received: {msg.data!r}")
        with self._lock:
            self._stdin_lines.append(msg.data)

    # ======================================================================
    # Background stdin reader
    # ======================================================================

    def _stdin_reader(self):
        """
        Capture operator keystrokes from the controlling terminal so y/n
        prompts can be answered in the same terminal crab was launched from.

        Read from /dev/tty, NOT sys.stdin: under `ros2 launch`, a child node's
        inherited stdin (fd 0) isn't connected to the terminal in a way that
        delivers typed input — select() never reports it ready.  /dev/tty is
        the controlling terminal shared by the whole launch, so keystrokes
        typed in the launch terminal reach here regardless of how launch wired
        up fd 0.  Falls back to sys.stdin only if /dev/tty can't be opened
        (e.g. no controlling terminal, such as a fully non-interactive run).
        """
        try:
            source = open('/dev/tty', 'r')
            self.get_logger().info("[stdin] reading operator input from /dev/tty.")
        except OSError as e:
            self.get_logger().warn(f"[stdin] /dev/tty unavailable ({e!r}) — falling back to stdin.")
            source = sys.stdin
            if not source or not source.readable():
                self.get_logger().error("[stdin] stdin also unusable — y/n prompts can only "
                                        "be answered via the /operator_response topic.")
                return

        while rclpy.ok():
            try:
                ready, _, _ = select.select([source], [], [], 0.05)
                if not ready:
                    continue
                line = source.readline()
                if line == '':
                    self.get_logger().warn("[stdin] input source hit EOF — reader thread "
                                           "exiting. Answer prompts via /operator_response instead.")
                    break
                self.get_logger().info(f"[stdin] captured line: {line!r}")
                with self._lock:
                    self._stdin_lines.append(line)
            except (ValueError, OSError) as e:
                self.get_logger().error(f"[stdin] reader thread crashed: {e!r}")
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
