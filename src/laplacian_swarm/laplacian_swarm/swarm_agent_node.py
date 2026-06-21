#!/usr/bin/env python3
"""Decentralized position-based swarm agent (one per drone, /uavN).
Consensus formation from peers' broadcast positions; barrier sync at every
phase; collision avoidance active in ALL moving states; zone landing by known
coordinates refined by OpenCV color detection."""
import math
import zlib
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from laplacian_interfaces.msg import (AgentState, MissionCommand,
                                      QrDetection, ColorDetection,
                                      ControlSetpoint, ControlCommand, Px4Status,
                                      LocalizationState, ZoneObservation, ZoneAck)
from laplacian_swarm import formations, geo, field, qr_parse

NAN = float('nan')
IDLE, SYNC, TAKEOFF, NAV, READ, EXECUTE, SPLIT, RTL, DONE, BARRIER = range(10)
PEER_TIMEOUT = 2.0
NAV_STATE_AUTO_RTL = 5             # px4_msgs VehicleStatus.NAVIGATION_STATE_AUTO_RTL
HALF_FOV = 0.61086525              # half of camera horizontal_fov (70 deg, x500/model.sdf)
ASPECT_Y = 1080.0 / 1920.0         # offset_y is per half-HEIGHT (camera height/width)
CAM_FWD = 0.20                      # camera mounted 0.2 m ahead of base_link

# Madde 7 (SW-11): peer membership state, by age of last AgentState sample.
PEER_FRESH, PEER_STALE, PEER_DEGRADED, PEER_LOST = range(4)
FRESH_T = 1.0          # participates in control + barrier
STALE_T = 3.0          # short grace - no new phase starts
DEGRADED_T = 8.0        # beyond this -> swarm hold / mission return territory
# NAV_SETTLE_SPEED: live neighbor-feedback control has an irreducible
# residual jitter floor (each agent's own small error feeds neighbors'
# error terms) - position accuracy (arrive_radius), not near-zero velocity,
# is what "settled" actually means here.
NAV_SETTLE_SPEED = 1.0   # m/s
NAV_SETTLE_S = 1.5       # all active slots must hold error+speed this long

# Madde 8 (SW-12): 3D inter-UAV separation (architecture SS14).
RISK_SAFE, RISK_CAUTION, RISK_CONFLICT, RISK_CRITICAL = range(4)
SEP_HORIZON = 4.0       # s, closest-point-of-approach lookahead cap
VERT_SAFE_M = 2.0        # vertical gap beyond this = intentional (reader/split), not a conflict
UNCERTAINTY_M = 0.5      # fixed conservative pad for own+peer position/velocity uncertainty

# Madde 9 (SW-15/SW-16): QR mission sub-FSM.
R_APPROACH, R_SETTLE, R_SEARCH, R_CENTER, R_CONFIRM = range(5)
READ_SETTLE_SPEED = 0.5  # m/s, reader's own (single-agent) settle threshold
READ_SETTLE_S = 1.0
READ_SEARCH_RMAX = 4.0   # m, spiral search cap around the surveyed QR position
READ_CENTER_TOL = 0.2    # normalized image-plane offset considered "centered"
READ_CONFIRM_N = 3       # consecutive matching decodes required - stands in for a
                         # checksum field the real (baked) QR payloads do not carry
READ_TIMEOUT_S = 60.0    # explicit failure if no confirmed QR in time, not an infinite hang


class _Pos:
    """Mirrors the px4_gateway_node's Px4Status into the field names this
    FSM was written against (x/y/z/heading/vx/vy/vz in PX4 local NED).
    roll/pitch (Madde 10) are real vehicle attitude, used by the camera
    ground-plane projection - heading/yaw alone isn't enough since a
    tilted (laterally-accelerating) vehicle's downward camera doesn't
    point straight down."""
    __slots__ = ('x', 'y', 'z', 'heading', 'vx', 'vy', 'vz', 'roll', 'pitch')
    def __init__(self):
        self.x = self.y = self.z = self.heading = self.vx = self.vy = self.vz = 0.0
        self.roll = self.pitch = 0.0


class _St:
    __slots__ = ('armed', 'offboard_active', 'nav_state', 'arming_state', 'estimator_ready', 'landed')
    def __init__(self):
        self.armed = False; self.offboard_active = False
        self.nav_state = 0; self.arming_state = 0; self.estimator_ready = False
        self.landed = True


class Agent(Node):
    def __init__(self):
        super().__init__('swarm_agent_node')
        dp = self.declare_parameter
        dp('drone_id', 0); dp('system_id', 1); dp('num_uavs', 3); dp('team_id', 1)
        dp('spawn_gz', [0.0, 0.0, 0.1])
        dp('slot_table', [0])
        dp('spacing', 5.0); dp('cruise_alt', 15.0); dp('read_alt', 15.0)
        dp('arrive_radius', 1.5)
        dp('k_form', 0.35); dp('k_goal', 0.35); dp('k_avoid', 2.0)
        dp('d_safe', 3.5); dp('v_max', 3.0); dp('k_damp', 1.2)
        dp('reader_id', 1)
        dp('return_alt', 18.0)
        dp('auto_initial_arm', True); dp('auto_initial_offboard', True)
        g = self.get_parameter
        self.id = int(g('drone_id').value); self.sys_id = int(g('system_id').value)
        self.n = int(g('num_uavs').value); self.team = int(g('team_id').value)
        # Madde 7 (SW-07/SW-08): fixed uav_id -> slot_id, identical table on
        # every agent, sourced from the manifest (not a live per-leg ranking).
        table = [int(x) for x in g('slot_table').value]
        self.slot_table = table if len(table) == self.n else list(range(self.n))
        self.slot_id = self.slot_table[self.id] if 0 <= self.id < len(self.slot_table) else self.id
        # Madde 6: bootstrap value only, used before localization_node's first
        # GNSS-derived FIELD_ENU origin arrives - then self.spawn tracks that.
        self.spawn = [float(x) for x in g('spawn_gz').value]
        self.spacing = float(g('spacing').value)
        self.cruise = float(g('cruise_alt').value); self.read_alt = float(g('read_alt').value)
        self.return_alt = float(g('return_alt').value)
        self.rad = float(g('arrive_radius').value)
        self.K_FORM = float(g('k_form').value); self.K_GOAL = float(g('k_goal').value)
        self.K_AVOID = float(g('k_avoid').value); self.D_SAFE = float(g('d_safe').value)
        self.V_MAX = float(g('v_max').value); self.K_DAMP = float(g('k_damp').value)
        self.reader = int(g('reader_id').value)
        self.AUTO_ARM = bool(g('auto_initial_arm').value)
        self.AUTO_OFFBOARD = bool(g('auto_initial_offboard').value)

        px4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        rel = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        bus = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.p_setpoint = self.create_publisher(ControlSetpoint, 'control/setpoint', rel)
        self.p_command = self.create_publisher(ControlCommand, 'control/command', rel)
        self.create_subscription(Px4Status, 'px4/status', self._px4_cb, bus)
        self.create_subscription(LocalizationState, 'localization/state', self._loc_cb, bus)
        self.p_state = self.create_publisher(AgentState, '/swarm/agent_state', bus)
        self.create_subscription(AgentState, '/swarm/agent_state', self._peer_cb, bus)
        self.p_mission = self.create_publisher(MissionCommand, '/swarm/mission', rel)
        self.create_subscription(MissionCommand, '/swarm/mission', self._mis_cb, rel)
        self.create_subscription(QrDetection, 'vision/qr', self._qr_cb, 10)
        self.create_subscription(ColorDetection, 'vision/color', self._color_cb, 10)
        # Madde 10: structured zone observation/ack, replacing the old
        # unstructured 'Z:color:e:n' string smuggled through AgentState.note.
        self.p_zone = self.create_publisher(ZoneObservation, '/swarm/zone', bus)
        self.create_subscription(ZoneObservation, '/swarm/zone', self._zone_obs_cb, bus)
        self.p_zone_ack = self.create_publisher(ZoneAck, '/swarm/zone_ack', bus)
        self.create_subscription(ZoneAck, '/swarm/zone_ack', self._zone_ack_cb, bus)

        self.pos = _Pos(); self.st = _St()
        self.peers = {}; self.fsm = IDLE; self.ready = False; self.k = 0
        self.last_cmd = self.get_clock().now()
        self.was_offboard = False
        self.seq = 0; self.mis = None
        self.cur_qr = 0; self.next_qr = 0
        self.form = formations.LINE; self.tgt_alt = self.cruise
        self.pitch = 0.0; self.roll = 0.0
        self.qr_seen = None; self.published_for = -1
        self.qr_det = None                 # (QrDetection, t) - latest frame, for centering
        self.read_phase = R_APPROACH; self.read_t0 = None
        self.read_settle_t0 = None; self.read_search_t0 = None
        self.qr_confirm_key = None; self.qr_confirm_n = 0; self.qr_confirm_task = None
        self.qr_center_est = None          # live (E, N) QR estimate from camera projection
        self.zones = {}; self.zone_buf = {}; self.color_det = {}
        self.zone_conf = {}; self.zone_unc = {}; self.zone_hash = {}
        self.zone_acks = {}; self.zones_confirmed = {}
        self.t_state = self.get_clock().now()
        self.sp_phase = 0; self.land_t = None; self.split_done = False
        self.rejoin_settle_t0 = None       # Madde 11: rendezvous+slot-capture settle before rejoin
        self.ground_t0 = None              # Madde 11: grounded-duration fallback for landed detector
        self.disarm_t0 = None              # retry disarm until confirmed, don't fire-and-forget
        self.native_rtl = False            # Madde 11: PX4 native-RTL override in progress
        self.lock_yaw = None; self.read_slots = None; self.search_t0 = None
        self.prev_qr = None                # Madde 7: deterministic leg axis (no live position)
        self.nav_settle_t0 = None          # Madde 7: all-slots settle timer for NAV completion
        self.barrier_action = None        # ('NAV', qr) | ('RTL',) after barrier passes
        self.virt = None                  # integrated virtual target (E, N)
        self.cmd_v = (0.0, 0.0)           # last commanded vel (N, E) for accel slew
        self.cmd_yaw = None
        self.A_MAX = 2.0                  # m/s^2 accel limit
        self.YAW_RATE = 0.8               # rad/s yaw slew
        self.create_timer(0.05, self.loop)
        self.get_logger().info(f'agent uav{self.id} spawn={self.spawn} CODE v8-profile')

    # ---- callbacks ----
    def _px4_cb(self, m):
        self.pos.x, self.pos.y, self.pos.z = m.position_ned
        self.pos.vx, self.pos.vy, self.pos.vz = m.velocity_ned
        self.pos.heading = m.heading
        self.pos.roll = m.roll; self.pos.pitch = m.pitch
        self.st.armed = m.armed; self.st.offboard_active = m.offboard_active
        self.st.nav_state = m.nav_state; self.st.arming_state = m.arming_state
        self.st.estimator_ready = m.estimator_ready
        self.st.landed = m.landed

    def _loc_cb(self, m):
        if m.valid:
            self.spawn = list(m.px4_origin_field_enu)

    def _peer_cb(self, m):
        self.peers[m.drone_id] = (m, self.get_clock().now())

    def _qr_cb(self, m):
        """Madde 9: projects the detection into a live field-frame QR
        position estimate (CENTER sub-phase target) and runs the multi-frame
        confirmation streak (CONFIRM sub-phase) - the real baked QR payloads
        carry no checksum, so requiring READ_CONFIRM_N identical decodes in
        a row is what actually guards against a single corrupted OCR/zbar
        misread, instead of a checksum field that doesn't exist on the wire."""
        self.qr_det = (m, self.get_clock().now())
        if not m.valid:
            return
        self.qr_seen = m.payload
        e, nth, _ = self.gz_now()
        dn, de = geo.project_ground(m.offset_x, m.offset_y, max(0.5, -self.pos.z),
                                     self.pos.roll, self.pos.pitch, self.pos.heading,
                                     HALF_FOV, ASPECT_Y, CAM_FWD)
        self.qr_center_est = (e + de, nth + dn)
        if self.fsm != READ or self.id != self._acting_reader():
            return
        parsed = qr_parse.parse(m.payload, self.team, self.n)
        if not parsed or parsed['qr_id'] != self.cur_qr:
            self.qr_confirm_key = None; self.qr_confirm_n = 0
            return
        key = (parsed['qr_id'], parsed['formation'], parsed['altitude'],
               parsed['split'], parsed['split_id'], parsed['next'])
        if key == self.qr_confirm_key:
            self.qr_confirm_n += 1
        else:
            self.qr_confirm_key = key; self.qr_confirm_n = 1
        if self.qr_confirm_n >= READ_CONFIRM_N:
            self.qr_confirm_task = parsed

    def _zone_hash(self, color, e, n):
        """Madde 10: cheap consensus key - round to a 0.5 m grid so near-
        identical estimates collide on the same hash, but a meaningfully
        different position (a stale or competing lock) doesn't."""
        return zlib.crc32(f'{color}:{round(e * 2) / 2}:{round(n * 2) / 2}'.encode()) & 0xFFFFFFFF

    def _zone_confidence(self, area_frac, spread):
        """0..1: bigger blob + tighter buffer spread = more confident."""
        return max(0.0, min(1.0, 0.3 + 0.5 * min(1.0, area_frac / 0.02)
                             + 0.2 * (1.0 - min(1.0, spread / 3.0))))

    def _zone_uncertainty(self):
        """1-sigma-ish ground radius: grows with altitude (attitude error
        couples into ground-projection error roughly linearly with height) -
        a simple, defensible heuristic, not a full covariance propagation."""
        return max(0.3, -self.pos.z * 0.05)

    def _publish_zone(self, color, locked):
        e, n = self.zones[color]
        h = self._zone_hash(color, e, n)
        self.zone_hash[color] = h
        m = ZoneObservation(); m.header.stamp = self.get_clock().now().to_msg()
        m.drone_id = self.id; m.color = color; m.e = e; m.n = n
        m.confidence = self.zone_conf.get(color, 0.5)
        m.uncertainty_m = self.zone_unc.get(color, 1.0)
        m.hash = h; m.locked = locked
        self.p_zone.publish(m)

    def _color_cb(self, m):
        self.color_det[m.color.upper()] = (m, self.get_clock().now())
        if not (m.detected and m.area_frac > 0.0008):
            return
        col = m.color.upper()
        e, nth, _ = self.gz_now()
        dn, de = geo.project_ground(m.offset_x, m.offset_y, max(0.5, -self.pos.z),
                                     self.pos.roll, self.pos.pitch, self.pos.heading,
                                     HALF_FOV, ASPECT_Y, CAM_FWD)
        proj = (e + de, nth + dn)
        self.zone_unc[col] = self._zone_uncertainty()
        if col in self.zones:                       # locked: smooth, reject outliers
            ze, zn = self.zones[col]
            if math.hypot(proj[0] - ze, proj[1] - zn) < 4.0:
                self.zones[col] = (0.8 * ze + 0.2 * proj[0], 0.8 * zn + 0.2 * proj[1])
                self.zone_conf[col] = max(self.zone_conf.get(col, 0.5),
                                           self._zone_confidence(m.area_frac, 0.0))
                self._publish_zone(col, locked=True)
            return
        buf = self.zone_buf.setdefault(col, [])     # not locked: collect consistent hits
        buf.append(proj)
        if len(buf) > 6:
            buf.pop(0)
        if len(buf) >= 4:
            es = [p[0] for p in buf]; ns = [p[1] for p in buf]
            spread = max(max(es) - min(es), max(ns) - min(ns))
            if spread < 3.0:
                self.zones[col] = (sum(es) / len(es), sum(ns) / len(ns))   # LOCK
                self.zone_conf[col] = self._zone_confidence(m.area_frac, spread)
                self.get_logger().info(f'zone {col} locked at {self.zones[col]}')
                self._publish_zone(col, locked=True)

    def _zone_obs_cb(self, m):
        """Madde 10: adopt a peer's structured, hash-identified zone lock
        (replaces the old unstructured note-string smuggling) so every
        agent ends up with the EXACT same canonical estimate for a color,
        not an independently-rederived one - then ack receipt of it."""
        col = m.color.upper()
        if not m.locked:
            return
        if col in self.zones:
            ze, zn = self.zones[col]
            if math.hypot(m.e - ze, m.n - zn) > 5.0:
                self.get_logger().warn(
                    f'uav{self.id}: zone {col} obs from uav{m.drone_id} disagrees '
                    f'by {math.hypot(m.e - ze, m.n - zn):.1f}m - ignoring, not acking')
                return
        else:
            self.zones[col] = (m.e, m.n)
            self.zone_conf[col] = m.confidence; self.zone_unc[col] = m.uncertainty_m
        ack = ZoneAck(); ack.header.stamp = self.get_clock().now().to_msg()
        ack.drone_id = self.id; ack.color = col; ack.hash = m.hash
        self.p_zone_ack.publish(ack)

    def _zone_ack_cb(self, m):
        col = m.color.upper()
        acked = self.zone_acks.setdefault((col, m.hash), set())
        acked.add(m.drone_id)
        fresh_others = sum(1 for did in range(self.n)
                           if did != self.id and self._peer_state(did) == PEER_FRESH)
        if len(acked - {self.id}) >= fresh_others:
            self.zones_confirmed[col] = True

    def _mis_cb(self, m):
        if m.seq <= self.seq:
            return
        self.seq = m.seq; self.mis = m
        if m.phase == 0:
            self.form = m.formation
            self.tgt_alt = m.altitude if m.altitude > 0 else self.cruise
            self.spacing = m.spacing if m.spacing > 0 else self.spacing
            self.cur_qr = m.target_qr; self.ready = True
            if self.fsm == IDLE:
                self._to(SYNC)
        elif m.phase == 2:
            self._to(RTL)
        else:
            self.form = m.formation
            self.tgt_alt = m.altitude if m.altitude > 0 else self.tgt_alt
            self.spacing = m.spacing if m.spacing > 0 else self.spacing
            self.pitch = math.radians(m.pitch_deg); self.roll = math.radians(m.roll_deg)
            self.next_qr = m.target_qr
            self.split_done = False; self.sp_phase = 0; self.land_t = None
            self.rejoin_settle_t0 = None; self.ground_t0 = None
            self.read_slots = None; self.search_t0 = None
            self._to(EXECUTE)

    # ---- helpers ----
    def _to(self, s):
        self.fsm = s; self.t_state = self.get_clock().now()
        self.virt = None; self.cmd_v = (0.0, 0.0); self.nav_settle_t0 = None
        if s == READ:
            self.read_phase = R_APPROACH; self.read_t0 = None
            self.read_settle_t0 = None; self.read_search_t0 = None
            self.qr_confirm_key = None; self.qr_confirm_n = 0; self.qr_confirm_task = None
            self.qr_seen = None; self.read_slots = None; self.published_for = -1
        elif s == RTL:
            self.ground_t0 = None; self.disarm_t0 = None
    def in_state(self): return (self.get_clock().now() - self.t_state).nanoseconds / 1e9

    def gz_now(self):
        return geo.local_ned_to_gz((self.pos.x, self.pos.y, self.pos.z), self.spawn)

    def fresh_peers(self, include_detaching=False):
        now = self.get_clock().now()
        return {d: s for d, (s, t) in self.peers.items()
                if d != self.id and (now - t).nanoseconds < PEER_TIMEOUT * 1e9
                and (include_detaching or not s.detaching)}

    def centroid(self):
        e, nth, _ = self.gz_now(); cE, cN, c = e, nth, 1
        for s in self.fresh_peers().values():
            cE += s.position[0]; cN += s.position[1]; c += 1
        return cE / c, cN / c

    def dist_to(self, xy):
        e, nth, _ = self.gz_now()
        return math.hypot(e - xy[0], nth - xy[1])

    def centroid_dist_to(self, xy):
        c = self.centroid()
        return math.hypot(c[0] - xy[0], c[1] - xy[1])

    def all_armed(self):
        if not (self.st.armed and self.st.offboard_active):
            return False
        peers = self.fresh_peers(include_detaching=True)
        if len(peers) < self.n - 1:
            return False
        return all(s.armed and s.offboard_active for s in peers.values())

    def quorum(self):
        c = 1 if self.ready else 0
        for s in self.fresh_peers(include_detaching=True).values():
            if s.ready:
                c += 1
        return c

    def _peer_state(self, did):
        """Madde 7 (SW-11): fixed mission membership (range(self.n)), not
        whatever happens to still be in self.peers - LOST/DEGRADED peers
        stay known members, just in a worse state, instead of silently
        disappearing from barrier/NAV bookkeeping."""
        if did == self.id:
            return PEER_FRESH
        rec = self.peers.get(did)
        if rec is None:
            return PEER_LOST
        age = (self.get_clock().now() - rec[1]).nanoseconds / 1e9
        if age < FRESH_T:
            return PEER_FRESH
        if age < STALE_T:
            return PEER_STALE
        if age < DEGRADED_T:
            return PEER_DEGRADED
        return PEER_LOST

    def _acting_reader(self):
        """Madde 9: reader failover - if the manifest reader (reader_id
        param) isn't FRESH, the lowest-id FRESH member takes over for this
        leg. Computed independently on every agent from the same broadcast
        peer data, so it agrees swarm-wide without an explicit handoff msg."""
        if self._peer_state(self.reader) == PEER_FRESH:
            return self.reader
        for did in range(self.n):
            if self._peer_state(did) == PEER_FRESH:
                return did
        return self.reader

    def _fresh_qr(self, max_age=0.6):
        d = self.qr_det
        if d and (self.get_clock().now() - d[1]).nanoseconds < max_age * 1e9 and d[0].valid:
            return d[0]
        return None

    def _detect_manual_leader(self):
        """Semi-autonomous mode: a peer that was actively flying the
        mission (fsm not in IDLE/SYNC/TAKEOFF/DONE) and is armed but no
        longer in Offboard has been taken over manually by the operator via
        QGC - treat it as the formation leader. The IDLE/SYNC/TAKEOFF/DONE
        exclusion is what keeps this from ever firing during normal
        bootstrap (armed-but-not-yet-offboard is the NORMAL semi-auto
        startup window, not a takeover) or after landing. Lowest matching
        id wins if more than one peer somehow qualifies - deterministic,
        every agent computes the same answer from the same broadcast data."""
        for did in range(self.n):
            if did == self.id:
                continue
            if self._peer_state(did) != PEER_FRESH:
                continue
            s, _t = self.peers[did]
            if s.armed and not s.offboard_active and s.fsm_state not in (IDLE, SYNC, TAKEOFF, DONE):
                return did
        return None

    def _follow_leader(self, leader_id):
        """Fly this agent's own formation slot relative to the manually
        flown leader's LIVE position/heading (not a fixed QR target) -
        avoidance/separation still applies via vel_sp as normal. Resuming
        is automatic: self.fsm is never touched here, so once the operator
        hands Offboard back to the leader, _detect_manual_leader() simply
        stops firing and every agent falls straight back into whatever
        mission state it was already in."""
        s, _t = self.peers[leader_id]
        leader_xy = (s.position[0], s.position[1])
        tgt = formations.slot_target(self.form, self.slot_id, self.spacing, leader_xy, s.yaw)
        vN, vE = self.goto_vel(tgt)
        vN -= self.K_DAMP * self.pos.vx; vE -= self.K_DAMP * self.pos.vy
        vN, vE = self._clamp(vN, vE)
        self.vel_sp(vN, vE, -s.position[2], s.yaw)

    def _read_fail(self):
        """Madde 9 'Bitti sayilir': QR yoksa explicit failure, not an
        infinite hang - every agent (reader or not) times out independently
        off the same READ-entry clock, so a dead/failed-over reader doesn't
        leave followers stuck waiting for a MissionCommand that never comes."""
        self.get_logger().error(
            f'uav{self.id}: READ timeout at qr{self.cur_qr} - no confirmed QR, returning')
        self._enter_barrier(('RTL',))

    def _separation(self):
        """Madde 8 (SW-12): full 3D relative position/velocity + closest-
        point-of-approach per peer (architecture SS14), classified into
        SAFE/CAUTION/CONFLICT/CRITICAL. A peer vertically separated beyond
        VERT_SAFE_M (the reader hovering at read_alt while others sit at
        cruise_alt, or a SPLIT member descending) is intentional, not a
        conflict - it is excluded from both the push and the risk vote, so
        safely-separated UAVs are never needlessly shoved sideways. Returns
        (avoid_e, avoid_n, risk) - horizontal repulsion is unchanged in style
        (directionally weighted by closing speed) but now 3D-gated."""
        e, nth, up = self.gz_now()
        vE_own, vN_own, vU_own = self.pos.vy, self.pos.vx, -self.pos.vz
        aE = aN = 0.0
        risk = RISK_SAFE
        eff_safe = self.D_SAFE + UNCERTAINTY_M
        for s in self.fresh_peers(include_detaching=True).values():
            rU = s.position[2] - up
            if abs(rU) > VERT_SAFE_M:
                continue                        # intentional vertical separation
            rE, rN = s.position[0] - e, s.position[1] - nth
            sv = s.velocity
            vE = sv[0] - vE_own; vN = sv[1] - vN_own
            vU = (sv[2] if len(sv) > 2 else 0.0) - vU_own
            d3 = math.sqrt(rE * rE + rN * rN + rU * rU)
            vmag2 = vE * vE + vN * vN + vU * vU
            t_cpa = 0.0 if vmag2 < 1e-6 else max(
                0.0, min(SEP_HORIZON, -(rE * vE + rN * vN + rU * vU) / vmag2))
            cE, cN, cU = rE + vE * t_cpa, rN + vN * t_cpa, rU + vU * t_cpa
            d_cpa = math.sqrt(cE * cE + cN * cN + cU * cU)
            worst = min(d3, d_cpa)
            if worst < eff_safe * 0.25:
                risk = max(risk, RISK_CRITICAL)
            elif worst < eff_safe * 0.5:
                risk = max(risk, RISK_CONFLICT)
            elif worst < eff_safe:
                risk = max(risk, RISK_CAUTION)
            ddH = math.hypot(rE, rN)
            if 0.05 < ddH < self.D_SAFE:
                closing = max(0.0, (rE * vE_own + rN * vN_own) / ddH)
                w = self.K_AVOID * (self.D_SAFE - ddH) / ddH * (1.0 + 1.5 * closing)
                aE -= w * rE; aN -= w * rN      # push away from peer (own - peer direction)
        return aE, aN, risk

    def _clamp(self, vN, vE):
        sp = math.hypot(vE, vN)
        if sp > self.V_MAX:
            vE *= self.V_MAX / sp; vN *= self.V_MAX / sp
        return vN, vE

    def _slot_of(self, did):
        return self.slot_table[did] if 0 <= did < len(self.slot_table) else did

    def _formation_heading(self):
        """Madde 7 (SW-09): shared heading from static mission inputs only
        (previous QR -> current QR in field.QR_MAP) - never from live own/peer
        position, so every agent computes the exact same value independently."""
        if self.prev_qr is None:
            return 0.0
        p0 = field.QR_MAP[self.prev_qr][:2]; p1 = field.QR_MAP[self.cur_qr][:2]
        de, dn = p1[0] - p0[0], p1[1] - p0[1]
        if math.hypot(de, dn) < 0.01:
            return 0.0
        return math.atan2(de, dn)

    def _staging_ref(self):
        """Madde 11 (SW-20): a safety-offset staging point for the members
        NOT detaching - shifted from the current QR toward the next leg's
        bearing (or held at the QR if this is the last leg), deterministic
        and identical on every agent."""
        qr = field.QR_MAP[self.cur_qr][:2]
        if self.next_qr and self.next_qr in field.QR_MAP:
            nxt = field.QR_MAP[self.next_qr][:2]
            bearing = math.atan2(nxt[0] - qr[0], nxt[1] - qr[1])
            return (qr[0] + 8.0 * math.sin(bearing), qr[1] + 8.0 * math.cos(bearing))
        return qr

    def consensus_vel(self, goal_xy):
        """Laplacian formation + coherence-scaled migration + avoidance + damping.
        Migration pins the formation's own REFERENCE point (each member's
        position minus its slot offset) to goal_xy, not the raw position
        centroid - V/ARROW slot offsets don't sum to zero, so anchoring the
        raw centroid settles the swarm off-target by a constant bias and
        _nav_done()/slot_target() (which expect reference==goal) never agree."""
        e, nth, _ = self.gz_now(); P = (e, nth)
        nbrs = self.fresh_peers()
        fwd, right = formations.axis(self._formation_heading())
        fb, rb = formations.slot(self.form, self.slot_id, self.spacing)
        di = (fb * fwd[0] + rb * right[0], fb * fwd[1] + rb * right[1])
        eE = eN = 0.0
        refE, refN, c = P[0] - di[0], P[1] - di[1], 1
        for did, s in nbrs.items():
            fj, rj = formations.slot(self.form, self._slot_of(did), self.spacing)
            dj = (fj * fwd[0] + rj * right[0], fj * fwd[1] + rj * right[1])
            eE += (s.position[0] - P[0]) - (dj[0] - di[0])
            eN += (s.position[1] - P[1]) - (dj[1] - di[1])
            refE += s.position[0] - dj[0]; refN += s.position[1] - dj[1]; c += 1
        refE /= c; refN /= c
        coh = 1.0
        if nbrs:
            coh = max(0.15, 1.0 - (math.hypot(eE, eN) / len(nbrs)) / (2.0 * self.spacing))
        vE = self.K_FORM * eE + self.K_GOAL * coh * (goal_xy[0] - refE)
        vN = self.K_FORM * eN + self.K_GOAL * coh * (goal_xy[1] - refN)
        # rate damping: the spring-only law above has no decay term, so a
        # settled-but-perturbed swarm oscillates instead of holding still -
        # pull back against the agent's own measured velocity.
        vE -= self.K_DAMP * self.pos.vy; vN -= self.K_DAMP * self.pos.vx
        return self._clamp(vN, vE)

    def goto_vel(self, tgt_xy):
        """Individual goto. Avoidance is added unslewed inside vel_sp."""
        e, nth, _ = self.gz_now()
        vE = 0.8 * (tgt_xy[0] - e)
        vN = 0.8 * (tgt_xy[1] - nth)
        return self._clamp(vN, vE)

    def _nav_done(self, goal):
        """Madde 7 (SW-10): NAV only completes once EVERY expected member is
        within `rad` of its own fixed-slot target around `goal` AND slow
        enough, held for NAV_SETTLE_S - a lagging or STALE/DEGRADED/LOST
        member blocks completion instead of a lone centroid check."""
        heading = self._formation_heading()
        for did in range(self.n):
            if did == self.id:
                e, nth, _ = self.gz_now(); vE, vN = self.pos.vy, self.pos.vx
            else:
                if self._peer_state(did) != PEER_FRESH:
                    return False
                s, _t = self.peers[did]
                e, nth = s.position[0], s.position[1]
                vE, vN = s.velocity[0], s.velocity[1]
            sx, sy = formations.slot_target(self.form, self._slot_of(did),
                                            self.spacing, goal, heading)
            if math.hypot(e - sx, nth - sy) > self.rad or math.hypot(vE, vN) > NAV_SETTLE_SPEED:
                return False
        return True

    def nav_yaw(self, goal):
        if self.centroid_dist_to(goal) > 3.0:
            c = self.centroid()
            self.lock_yaw = math.atan2(goal[0] - c[0], goal[1] - c[1])
        return self.lock_yaw if self.lock_yaw is not None else self.pos.heading

    def _cmd(self, command):
        m = ControlCommand(); m.header.stamp = self.get_clock().now().to_msg()
        m.command = command
        self.p_command.publish(m)

    def arm(self):      self._cmd(ControlCommand.CMD_ARM)
    def disarm(self):   self._cmd(ControlCommand.CMD_DISARM)
    def offboard(self): self._cmd(ControlCommand.CMD_SET_OFFBOARD)

    def pos_sp(self, n, e, d, yaw):
        m = ControlSetpoint(); m.header.stamp = self.get_clock().now().to_msg()
        m.mode = ControlSetpoint.MODE_POSITION
        m.position_ned = [float(n), float(e), float(d)]
        m.velocity_ned = [0.0, 0.0, 0.0]; m.yaw_rad = float(yaw)
        self.p_setpoint.publish(m)

    def vel_sp(self, vN, vE, d_target, yaw):
        """PX4-doc-correct smooth motion: POSITION setpoint + velocity feedforward.
        The NOMINAL velocity is accel-slewed (smooth); the AVOIDANCE velocity is
        added UNSLEWED so collision response is instant. PX4's position loop
        provides the damping (no external damping -> no resonance).
        Madde 8 (SW-12): nominal (migration) velocity is scaled/cut by the
        worst 3D separation risk among peers BEFORE the accel slew, so a
        CONFLICT/CRITICAL risk produces a smooth accel-limited brake, not an
        instant snap; the avoidance push itself stays unslewed as before."""
        dt = 0.05
        aE, aN, risk = self._separation()
        scale = (1.0, 0.5, 0.0, 0.0)[risk]      # SAFE, CAUTION, CONFLICT, CRITICAL
        vN *= scale; vE *= scale
        if risk == RISK_CRITICAL:               # operator alert (no autonomous RTL/Land here -
            self.get_logger().error(            # that policy belongs to the later split/return item)
                f'uav{self.id}: CRITICAL separation risk - migration cut, braking',
                throttle_duration_sec=1.0)
        elif risk == RISK_CONFLICT:
            self.get_logger().warn(
                f'uav{self.id}: separation risk=CONFLICT - migration cut, braking',
                throttle_duration_sec=1.0)
        dvN = max(-self.A_MAX * dt, min(self.A_MAX * dt, vN - self.cmd_v[0]))
        dvE = max(-self.A_MAX * dt, min(self.A_MAX * dt, vE - self.cmd_v[1]))
        vN = self.cmd_v[0] + dvN; vE = self.cmd_v[1] + dvE
        self.cmd_v = (vN, vE)
        vN_t, vE_t = vN + aN, vE + aE           # avoidance push, unslewed
        sp = math.hypot(vN_t, vE_t)
        cap = self.V_MAX * 1.6                 # let avoidance exceed nominal cap
        if sp > cap:
            vN_t *= cap / sp; vE_t *= cap / sp
        e, nth, _ = self.gz_now()
        if self.virt is None:
            self.virt = (e, nth)
        vx, vy = self.virt[0] + vE_t * dt, self.virt[1] + vN_t * dt
        le, ln = vx - e, vy - nth
        d = math.hypot(le, ln)
        if d > 1.5:                            # shorter leash: tight tracking
            vx = e + le / d * 1.5; vy = nth + ln / d * 1.5
        self.virt = (vx, vy)
        if self.cmd_yaw is None:
            self.cmd_yaw = self.pos.heading
        dy = (yaw - self.cmd_yaw + math.pi) % (2 * math.pi) - math.pi
        self.cmd_yaw += max(-self.YAW_RATE * dt, min(self.YAW_RATE * dt, dy))
        n = vy - self.spawn[1]; e_ = vx - self.spawn[0]
        m = ControlSetpoint(); m.header.stamp = self.get_clock().now().to_msg()
        m.mode = ControlSetpoint.MODE_POSITION_VELOCITY
        m.position_ned = [float(n), float(e_), float(d_target)]
        m.velocity_ned = [float(vN_t), float(vE_t), 0.0]
        m.yaw_rad = float(self.cmd_yaw)
        self.p_setpoint.publish(m)

    def hold(self):
        self.pos_sp(self.pos.x, self.pos.y, self.pos.z, self.pos.heading)

    def maneuver_dz(self):
        fb, rb = formations.slot(self.form, self.slot_id, self.spacing)
        return fb * math.tan(self.pitch) + rb * math.tan(self.roll)

    def ensure_offboard_armed(self, force_arm=False):
        # Each half is gated by its OWN flag - AUTO_ARM and AUTO_OFFBOARD
        # are independent knobs (e.g. semi-autonomous: operator arms+takes
        # off manually via QGC, AUTO_ARM=false, but software still claims
        # Offboard immediately once armed, AUTO_OFFBOARD=true, instead of
        # requiring a SECOND manual mode-switch step). Previously this
        # claimed BOTH unconditionally once called, silently arming even
        # when AUTO_ARM was false.
        # force_arm bypasses AUTO_ARM for SPLIT_REARM only: re-arming after
        # a deliberate, already-mission-authorized landing is a software
        # continuation of a flight already underway, not a fresh operator
        # handoff - it must not wait on a manual re-arm from QGC every split.
        now = self.get_clock().now()
        if self.k >= 20 and (now - self.last_cmd).nanoseconds > 5e8:
            if self.AUTO_OFFBOARD and not self.st.offboard_active:
                self.offboard(); self.last_cmd = now
            elif (self.AUTO_ARM or force_arm) and not self.st.armed:
                self.arm(); self.last_cmd = now

    def broadcast_state(self):
        e, nth, u = self.gz_now()
        s = AgentState(); s.header.stamp = self.get_clock().now().to_msg()
        s.drone_id = self.id; s.position = [e, nth, u]
        s.velocity = [self.pos.vy, self.pos.vx, -self.pos.vz]; s.yaw = self.pos.heading
        s.fsm_state = self.fsm; s.ready = self.ready
        s.armed = self.st.armed
        s.offboard_active = self.st.offboard_active
        s.nav_state = self.st.nav_state
        s.detaching = (self.fsm == SPLIT)
        s.note = ''                          # Madde 10: zones now go over /swarm/zone (structured)
        self.p_state.publish(s)

    def _publish_task(self, m):
        msg = MissionCommand(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.seq = self.seq + 1; msg.phase = 1
        msg.target_qr = m['next']; msg.formation = m['formation']; msg.spacing = self.spacing
        msg.altitude = m['altitude']; msg.pitch_deg = m['pitch_deg']; msg.roll_deg = m['roll_deg']
        msg.wait_s = m['wait_s']; msg.split_active = m['split']; msg.split_drone_id = m['split_id']
        msg.split_color = m['split_color']; msg.split_wait_s = m['split_wait']
        self.p_mission.publish(msg)

    def _enter_barrier(self, action):
        self.barrier_action = action
        self._to(BARRIER)

    def _barrier_pass(self):
        """Madde 7 (SW-11): walks the FIXED mission membership (range(self.n)),
        not just whoever is still in self.peers - a STALE/DEGRADED/LOST member
        explicitly blocks the barrier instead of quietly aging out of a dict
        and letting it pass without them."""
        for did in range(self.n):
            if did == self.id:
                continue
            if self._peer_state(did) != PEER_FRESH:
                return False
            s, _t = self.peers[did]
            if s.detaching:
                return False
            if s.fsm_state not in (BARRIER, NAV, RTL):
                return False
        return True

    # ---- main loop ----
    def loop(self):
        self.k += 1
        f = self.fsm

        if f == IDLE or self.mis is None:
            self.hold(); self.broadcast_state(); return

        if f == SYNC:
            self.hold()
            if self.quorum() >= self.n:
                self._to(TAKEOFF)
            self.broadcast_state(); return

        # Madde 5: control authority is only ours to claim during the initial
        # TAKEOFF_PREPARE(estimator_ready)->ACK(all_armed)->COMMIT handshake,
        # or for SPLIT_REARM (handled separately in _split_step's phase 4).
        # No background re-arm/re-offboard fight in any other state - an
        # operator/failsafe override is respected, not contested.
        if f == TAKEOFF and (self.AUTO_ARM or self.AUTO_OFFBOARD) \
                and not self.all_armed() and self.st.estimator_ready:
            self.ensure_offboard_armed()

        if self.was_offboard and not self.st.offboard_active \
                and f not in (IDLE, SYNC, TAKEOFF, DONE):
            if self.st.nav_state == NAV_STATE_AUTO_RTL:
                self.native_rtl = True
                self.get_logger().error(
                    'Control authority lost to PX4 native RTL (failsafe) - '
                    'holding, not contesting', throttle_duration_sec=2.0)
            elif self.native_rtl:
                # Madde 11: native RTL has moved on (nav_state no longer
                # AUTO_RTL) but we're still not back in Offboard - reclaim
                # it so our own FSM can resume command authority.
                now = self.get_clock().now()
                if (now - self.last_cmd).nanoseconds > 5e8:
                    self.offboard(); self.last_cmd = now
                self.get_logger().warn(
                    f'uav{self.id}: PX4 native RTL finished, reclaiming Offboard',
                    throttle_duration_sec=2.0)
            else:
                self.get_logger().error(
                    'Control authority lost (operator override or failsafe): '
                    'holding current setpoint, not contesting it', throttle_duration_sec=2.0)
        else:
            self.native_rtl = False
        self.was_offboard = self.st.offboard_active

        # Semi-autonomous leader-follower: if a peer that was actively
        # flying the mission has been taken over manually (operator
        # switched it out of Offboard via QGC), follow it in formation
        # instead of running this agent's own normal per-state behavior -
        # pure addition, gated so it can NEVER fire during normal mission
        # bootstrap/sync/landing (see _detect_manual_leader), leaving the
        # fully-autonomous flow completely untouched when no one takes over.
        leader_id = self._detect_manual_leader()
        if leader_id is not None and f not in (IDLE, SYNC, TAKEOFF, DONE):
            self._follow_leader(leader_id)
            self.broadcast_state(); return

        if f == TAKEOFF:
            if not self.all_armed():
                self.pos_sp(self.pos.x, self.pos.y, self.pos.z, self.pos.heading)  # hold on ground, arm in place
            else:
                n, e, d = geo.gz_to_ned((self.spawn[0], self.spawn[1], self.tgt_alt), self.spawn)
                self.pos_sp(n, e, d, self.pos.heading)         # ALL armed -> climb together
            if abs(-self.pos.z - self.tgt_alt) < 0.6:
                self.published_for = -1; self.lock_yaw = None
                self._enter_barrier(('NAV', self.cur_qr))      # wait for ALL airborne

        elif f == BARRIER:
            # Madde 11 (SW-20): while a member is detaching (SPLIT), the
            # rest hold at a safety-offset staging point - shifted toward
            # the next leg's bearing, away from the color-search area -
            # instead of freezing wherever BARRIER happened to catch them.
            detaching = any(self.peers[did][0].detaching for did in range(self.n)
                            if did != self.id and did in self.peers)
            if detaching:
                ref = self._staging_ref()
                tgt = formations.slot_target(self.form, self.slot_id, self.spacing,
                                             ref, self._formation_heading())
                vN, vE = self.goto_vel(tgt)
                self.vel_sp(vN, vE, -self.tgt_alt, self.cmd_yaw or self.pos.heading)
            else:
                self.vel_sp(0.0, 0.0, -self.tgt_alt, self.cmd_yaw or self.pos.heading)  # still + avoidance live
            if self._barrier_pass() and self.in_state() > 1.0:
                act = self.barrier_action or ('NAV', self.cur_qr)
                self.barrier_action = None
                self.lock_yaw = None; self.read_slots = None; self.search_t0 = None
                if act[0] == 'RTL':
                    self._to(RTL)
                else:
                    self.prev_qr = self.cur_qr           # Madde 7: deterministic leg axis
                    self.cur_qr = act[1]; self.published_for = -1
                    self._to(NAV)

        elif f == NAV:
            goal = field.QR_MAP[self.cur_qr][:2]
            vN, vE = self.consensus_vel(goal)
            self.vel_sp(vN, vE, -self.tgt_alt, self.nav_yaw(goal))
            if self._nav_done(goal):
                now = self.get_clock().now()
                if self.nav_settle_t0 is None:
                    self.nav_settle_t0 = now
                if (now - self.nav_settle_t0).nanoseconds / 1e9 > NAV_SETTLE_S:
                    self._to(READ)
            else:
                self.nav_settle_t0 = None

        elif f == READ:
            # Madde 9: APPROACH/SETTLE/SEARCH/CENTER/CONFIRM sub-FSM, with
            # reader failover and an explicit failure timeout. Every agent
            # (reader or not) runs the same READ_TIMEOUT_S clock off its own
            # READ-entry time, so a dead/failed-over reader can't leave
            # followers waiting forever for a MissionCommand that never
            # arrives - they independently RTL too.
            goal = field.QR_MAP[self.cur_qr][:2]
            yaw = self.nav_yaw(goal)
            if self.read_slots is None:
                self.read_slots = formations.slot_target(
                    self.form, self.slot_id, self.spacing, goal, self._formation_heading())
            if self.read_t0 is None:
                self.read_t0 = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.read_t0).nanoseconds / 1e9
            if elapsed > READ_TIMEOUT_S and self.published_for != self.cur_qr:
                self._read_fail(); return
            # _qr_cb's confirm streak runs continuously whenever fsm==READ,
            # independent of which cosmetic sub-phase the loop below is in -
            # publish the instant it's ready instead of waiting for the
            # phase machine to land on R_CONFIRM (a flickering detection can
            # bounce SEARCH<->CENTER and starve that transition forever).
            if (self.id == self._acting_reader() and self.qr_confirm_task is not None
                    and self.published_for != self.cur_qr):
                self.published_for = self.cur_qr
                self._publish_task(self.qr_confirm_task); return

            if self.id != self._acting_reader():
                vN, vE = self.goto_vel(self.read_slots)        # follower: just hold the slot
                self.vel_sp(vN, vE, -self.tgt_alt, yaw)
            elif self.read_phase == R_APPROACH:
                vN, vE = self.goto_vel(goal)
                self.vel_sp(vN, vE, -self.read_alt, yaw)
                if self.dist_to(goal) < self.rad:
                    self.read_phase = R_SETTLE; self.read_settle_t0 = None
            elif self.read_phase == R_SETTLE:
                vN, vE = self.goto_vel(goal)
                self.vel_sp(vN, vE, -self.read_alt, yaw)
                speed = math.hypot(self.pos.vy, self.pos.vx)
                now = self.get_clock().now()
                if self.dist_to(goal) < self.rad and speed < READ_SETTLE_SPEED:
                    if self.read_settle_t0 is None:
                        self.read_settle_t0 = now
                    if (now - self.read_settle_t0).nanoseconds / 1e9 > READ_SETTLE_S:
                        self.read_phase = R_SEARCH; self.read_search_t0 = None
                else:
                    self.read_settle_t0 = None
            elif self.read_phase == R_SEARCH:
                qrd = self._fresh_qr()
                if qrd is not None:
                    self.read_phase = R_CENTER
                else:
                    if self.read_search_t0 is None:
                        self.read_search_t0 = self.get_clock().now()
                    t = (self.get_clock().now() - self.read_search_t0).nanoseconds / 1e9
                    r = min(0.5 + 0.3 * t, READ_SEARCH_RMAX)
                    sx = goal[0] + r * math.cos(0.5 * t); sy = goal[1] + r * math.sin(0.5 * t)
                    vN, vE = self.goto_vel((sx, sy))
                    self.vel_sp(vN, vE, -self.read_alt, yaw)
                if elapsed > READ_TIMEOUT_S * 0.7:    # don't hang just searching/centering -
                    self.read_phase = R_CONFIRM        # the unconditional check above still
            elif self.read_phase == R_CENTER:           # gates the actual publish on a real confirm
                qrd = self._fresh_qr()
                vN, vE = self.goto_vel(self.qr_center_est or goal)
                self.vel_sp(vN, vE, -self.read_alt, yaw)
                if qrd is None:
                    self.read_phase = R_SEARCH; self.read_search_t0 = None
                else:
                    centered = abs(qrd.offset_x) < READ_CENTER_TOL and abs(qrd.offset_y) < READ_CENTER_TOL
                    if centered:
                        self.read_phase = R_CONFIRM
                if elapsed > READ_TIMEOUT_S * 0.7:
                    self.read_phase = R_CONFIRM
            elif self.read_phase == R_CONFIRM:
                vN, vE = self.goto_vel(self.qr_center_est or goal)
                self.vel_sp(vN, vE, -self.read_alt, yaw)

        elif f == EXECUTE:
            goal = field.QR_MAP[self.cur_qr][:2]
            vN, vE = self.consensus_vel(goal)
            self.vel_sp(vN, vE, -self.tgt_alt - self.maneuver_dz(),
                        self.lock_yaw if self.lock_yaw is not None else self.pos.heading)
            if self.mis.split_active and self.id == self.mis.split_drone_id and not self.split_done:
                # Madde 11 (split precheck): don't commit to a lone, camera-
                # only maneuver on shaky health - wait (logged, not silent)
                # until estimator/arm/offboard are actually solid.
                if self.st.estimator_ready and self.st.armed and self.st.offboard_active:
                    self._to(SPLIT); return
                self.get_logger().warn(
                    f'uav{self.id}: SPLIT precheck failed (estimator/armed/offboard) - delaying',
                    throttle_duration_sec=2.0)
            elif self.in_state() >= max(2.0, self.mis.wait_s):
                self._enter_barrier(('RTL',) if self.next_qr == 0 else ('NAV', self.next_qr))

        elif f == SPLIT:
            self._split_step()

        elif f == RTL:
            home = (self.spawn[0], self.spawn[1])
            # Home pads are only ~1.6-3.6m apart (closer than D_SAFE=3.5m /
            # the separation risk radius) - the return_alt vertical stagger
            # protects the transit, but once everyone converges to ground
            # level together at final descent, mutual avoidance/migration-cut
            # can keep nudging each UAV away from its own pad and it never
            # settles within the 1.0m landing gate. Sequence final descent
            # by ID (a few seconds apart) so they don't all fight for the
            # same small patch of airspace at once.
            my_turn = self.in_state() > self.id * 6.0
            if self.dist_to(home) > 1.0 or not my_turn:
                vN, vE = self.goto_vel(home)                   # own pad, with avoidance
                # Madde 11 (SW-24-adjacent): transit at this UAV's OWN
                # return_alt (staggered per ID in vehicles.yaml), not the
                # shared mission tgt_alt - multiple UAVs returning at once
                # shouldn't share one altitude.
                self.vel_sp(vN, vE, -self.return_alt, self.pos.heading)
            else:
                self.pos_sp(0.0, 0.0, 0.3, self.pos.heading)
                # Madde 11 (SW-05): PX4's own land detector is the preferred
                # confirmation before disarm (an altitude threshold alone can
                # disarm mid-air on a bad estimate/bounce) - but it isn't
                # guaranteed to fire under our custom setpoint control (see
                # the identical fallback in _split_step), so a grounded-for-3s
                # backstop prevents hanging forever on a flag that may never
                # flip while still preferring the real detector when it does.
                grounded = -self.pos.z < 0.5
                if grounded:
                    if self.ground_t0 is None:
                        self.ground_t0 = self.get_clock().now()
                else:
                    self.ground_t0 = None
                settled = (self.ground_t0 is not None
                           and (self.get_clock().now() - self.ground_t0).nanoseconds / 1e9 > 3.0)
                if grounded and (self.st.landed or settled):
                    # Retry disarm every tick until PX4 actually confirms it
                    # (armed goes false) instead of firing once and assuming
                    # success - a single dropped command left the vehicle
                    # armed and hovering forever while the FSM believed DONE.
                    self.disarm()
                    if self.disarm_t0 is None:
                        self.disarm_t0 = self.get_clock().now()
                    waited = (self.get_clock().now() - self.disarm_t0).nanoseconds / 1e9
                    if not self.st.armed or waited > 5.0:
                        self._to(DONE)

        elif f == DONE:
            self.hold()
            if self.st.armed:           # defense-in-depth: shouldn't happen
                self.disarm()            # given the retry-until-confirmed RTL
                                          # gate above, but never just sit armed

        self.broadcast_state()

    # ---- split: CAMERA-ONLY (OpenCV). spiral search -> visual servo -> land ----
    def _fresh_det(self, color, max_age=0.6):
        d = self.color_det.get(color)
        if d and (self.get_clock().now() - d[1]).nanoseconds < max_age * 1e9 and d[0].detected:
            return d[0]
        return None

    def _zone_target(self, color):
        return self.zones.get(color)                            # camera estimate only

    def _split_step(self):
        color = (self.mis.split_color or '').upper()
        SPLIT_ALT = 8.0                                     # below formation: vertical separation + big blob
        det = self._fresh_det(color)
        zone = self._zone_target(color)

        def _slow(vN, vE, cap=1.5):
            s = math.hypot(vN, vE)
            return (vN * cap / s, vE * cap / s) if s > cap else (vN, vE)

        if self.sp_phase == 0:                                  # SEARCH: lane raster sweep
            if zone is not None:
                self.sp_phase = 1; return
            if self.search_t0 is None:
                self.search_t0 = self.get_clock().now()
            t = (self.get_clock().now() - self.search_t0).nanoseconds / 1e9
            # Madde 10: sweep this UAV's own east-west lane of the search
            # polygon (not a spiral pinned to the QR) - lane_id = slot_id so
            # simultaneous searchers (if ever more than one) never overlap.
            sx, sy = field.search_lane_target(t, self.slot_id, self.n)
            vN, vE = _slow(*self.goto_vel((sx, sy)))
            self.vel_sp(vN, vE, -SPLIT_ALT, self.pos.heading)
            if t > 75.0:                                        # nothing found -> rejoin
                self._rejoin()

        elif self.sp_phase == 1:                                # APPROACH the LOCK
            vN, vE = _slow(*self.goto_vel(zone))
            self.vel_sp(vN, vE, -SPLIT_ALT, self.pos.heading)
            if self.dist_to(zone) < 1.2:                        # over the lock -> descend
                self.sp_phase = 2

        elif self.sp_phase == 2:                                # LOCKED DESCENT, lock refined to ground
            zone = self._zone_target(color)                     # EMA-updated by _color_cb every frame
            tn = zone[1] - self.spawn[1]; te = zone[0] - self.spawn[0]
            td = self.pos.z + 1.2
            self.pos_sp(tn, te, td, self.pos.heading)
            # Madde 11 (SW-04): PX4's own land detector is the preferred
            # confirmation, but it isn't guaranteed to fire under our custom
            # offboard position-setpoint descent (observed in SITL: landed
            # stayed false indefinitely while sitting motionless at ~0.1m) -
            # a grounded-for-3s fallback prevents an indefinite hang on a
            # detector that may never flip, while still preferring the real
            # detector (faster) when it does.
            grounded = -self.pos.z < 0.6
            if grounded:
                if self.ground_t0 is None:
                    self.ground_t0 = self.get_clock().now()
            else:
                self.ground_t0 = None
            settled = (self.ground_t0 is not None
                       and (self.get_clock().now() - self.ground_t0).nanoseconds / 1e9 > 3.0)
            if grounded and (self.st.landed or settled):
                self.sp_phase = 3; self.land_t = self.get_clock().now()
        elif self.sp_phase == 3:                                # LANDED: disarm + wait sb (timer
            self.disarm()                                       # starts only once landed is confirmed)
            if (self.get_clock().now() - self.land_t).nanoseconds / 1e9 > self.mis.split_wait_s + 2.0:
                self.sp_phase = 4; self.last_cmd = self.get_clock().now()
                self.rejoin_settle_t0 = None

        elif self.sp_phase == 4:                                # re-arm, rendezvous + slot capture + settle
            self.ensure_offboard_armed(force_arm=True)
            ref = self._staging_ref()
            tgt = formations.slot_target(self.form, self.slot_id, self.spacing,
                                         ref, self._formation_heading())
            vN, vE = self.goto_vel(tgt)
            # goto_vel is a pure P-controller (no damping) - fine for a
            # one-shot approach, but rejoin needs to actually SETTLE near
            # the target, and a pure P-controller orbits/oscillates around
            # the setpoint forever instead of converging (the exact failure
            # mode consensus_vel had before K_DAMP was added for NAV).
            vN -= self.K_DAMP * self.pos.vx; vE -= self.K_DAMP * self.pos.vy
            vN, vE = self._clamp(vN, vE)
            self.vel_sp(vN, vE, -self.tgt_alt, self.pos.heading)
            # Madde 11 (SW-21): rejoin needs altitude AND horizontal slot
            # capture AND a settled (low-speed) hold, not just "reached
            # altitude" - a lone climb doesn't mean it's back in formation.
            speed = math.hypot(self.pos.vy, self.pos.vx)
            captured = (-self.pos.z > self.tgt_alt - 1.0
                        and self.dist_to(tgt) < self.rad and speed < NAV_SETTLE_SPEED)
            now = self.get_clock().now()
            if captured:
                if self.rejoin_settle_t0 is None:
                    self.rejoin_settle_t0 = now
                if (now - self.rejoin_settle_t0).nanoseconds / 1e9 > NAV_SETTLE_S:
                    self._rejoin()
            else:
                self.rejoin_settle_t0 = None

    def _rejoin(self):
        self.split_done = True
        self._enter_barrier(('RTL',) if self.next_qr == 0 else ('NAV', self.next_qr))


def main(args=None):
    rclpy.init(args=args)
    node = Agent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
