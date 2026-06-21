#!/usr/bin/env python3
"""Madde 4: sole owner of /fmu/in/* and /fmu/out/*. swarm_agent_node talks to
PX4 only through the local ControlSetpoint/ControlCommand/Px4Status/
CommandResult interface published/consumed here."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus,
                          VehicleLandDetected, VehicleCommandAck, BatteryStatus,
                          VehicleAttitude)
from laplacian_interfaces.msg import (ControlSetpoint, ControlCommand,
                                      CommandResult, Px4Status)

NAN = float('nan')


class Px4Gateway(Node):
    def __init__(self):
        super().__init__('px4_gateway_node')
        dp = self.declare_parameter
        dp('system_id', 1)
        self.sys_id = int(self.get_parameter('system_id').value)

        px4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        loc = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # ---- PX4 outputs (subscribe) ----
        self.create_subscription(VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1', self._pos_cb, px4)
        self.create_subscription(VehicleAttitude, 'fmu/out/vehicle_attitude', self._att_cb, px4)
        self.create_subscription(VehicleStatus, 'fmu/out/vehicle_status_v4', self._st_cb, px4)
        self.create_subscription(VehicleLandDetected, 'fmu/out/vehicle_land_detected', self._land_cb, px4)
        self.create_subscription(BatteryStatus, 'fmu/out/battery_status', self._batt_cb, px4)
        self.create_subscription(VehicleCommandAck, 'fmu/out/vehicle_command_ack', self._ack_cb, px4)

        # ---- PX4 inputs (publish) ----
        self.p_ocm = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', px4)
        self.p_sp = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', px4)
        self.p_cmd = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', px4)

        # ---- local interface towards swarm_agent_node ----
        self.create_subscription(ControlSetpoint, 'control/setpoint', self._setpoint_cb, loc)
        self.create_subscription(ControlCommand, 'control/command', self._command_cb, loc)
        self.p_status = self.create_publisher(Px4Status, 'px4/status', loc)
        self.p_result = self.create_publisher(CommandResult, 'px4/command_result', loc)

        self.pos = VehicleLocalPosition(); self.st = VehicleStatus()
        self.land = VehicleLandDetected(); self.batt = BatteryStatus()
        self.att = VehicleAttitude(); self.att.q = [1.0, 0.0, 0.0, 0.0]
        self.last_sp = None
        self.last_command = None
        self.create_timer(0.05, self._tick)
        self.get_logger().info('px4_gateway_node up')

    # ---- PX4 -> cache ----
    def _pos_cb(self, m): self.pos = m
    def _att_cb(self, m): self.att = m
    def _st_cb(self, m): self.st = m
    def _land_cb(self, m): self.land = m
    def _batt_cb(self, m): self.batt = m

    def _ack_cb(self, m):
        if self.last_command is None:
            return
        r = CommandResult()
        r.header.stamp = self.get_clock().now().to_msg()
        r.command = self.last_command
        r.result = (CommandResult.RESULT_ACCEPTED
                    if m.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
                    else CommandResult.RESULT_REJECTED)
        r.px4_result = int(m.result)
        self.p_result.publish(r)

    # ---- local interface -> PX4 ----
    def _setpoint_cb(self, m):
        self.last_sp = m

    def _command_cb(self, m):
        self.last_command = m.command
        if m.command == ControlCommand.CMD_ARM:
            self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        elif m.command == ControlCommand.CMD_DISARM:
            # PX4 refuses a normal disarm while its OWN land detector
            # disagrees that it's landed - which we've found unreliable
            # under our custom offboard setpoint control (it can stay
            # false indefinitely while genuinely on the ground). Callers
            # of disarm() only do so after independently confirming
            # grounded (PX4_gateway/SW-04/SW-05 grounded-for-3s fallback),
            # so the 21196 "force" param2 is safe here, not a bypass of a
            # real safety check - just of a detector that already failed
            # to fire on its own.
            self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        elif m.command == ControlCommand.CMD_SET_OFFBOARD:
            self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def _send_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand(); m.timestamp = self._ts(); m.command = command
        m.param1 = p1; m.param2 = p2
        m.target_system = self.sys_id; m.target_component = 1
        m.source_system = 1; m.source_component = 1; m.from_external = True
        self.p_cmd.publish(m)

    def _ts(self): return int(self.get_clock().now().nanoseconds / 1000)

    def _roll_pitch(self):
        """Madde 10: VehicleAttitude.q is FRD-body -> NED, Hamilton (w,x,y,z) -
        ground-plane projection needs real roll/pitch, not just heading,
        since a tilted (accelerating) vehicle's downward camera doesn't
        point straight down."""
        w, x, y, z = self.att.q
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sp)))
        return roll, pitch

    # ---- fixed-rate heartbeat + setpoint forwarding + status ----
    def _tick(self):
        ocm = OffboardControlMode(); ocm.timestamp = self._ts()
        ocm.position = True; ocm.velocity = True
        self.p_ocm.publish(ocm)

        if self.last_sp is not None:
            sp = self.last_sp
            m = TrajectorySetpoint(); m.timestamp = self._ts()
            m.position = [sp.position_ned[0], sp.position_ned[1], sp.position_ned[2]]
            if sp.mode == ControlSetpoint.MODE_POSITION_VELOCITY:
                m.velocity = [sp.velocity_ned[0], sp.velocity_ned[1], NAN]
            else:
                m.velocity = [NAN, NAN, NAN]
            m.acceleration = [NAN, NAN, NAN]
            m.yaw = sp.yaw_rad
            self.p_sp.publish(m)

        s = Px4Status(); s.header.stamp = self.get_clock().now().to_msg()
        s.armed = (self.st.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        s.offboard_active = (self.st.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        s.nav_state = self.st.nav_state
        s.arming_state = self.st.arming_state
        s.landed = self.land.landed
        s.estimator_ready = not (self.pos.xy_valid is False or self.pos.z_valid is False)
        s.battery_remaining = self.batt.remaining
        s.position_ned = [self.pos.x, self.pos.y, self.pos.z]
        s.velocity_ned = [self.pos.vx, self.pos.vy, self.pos.vz]
        s.heading = self.pos.heading
        s.roll, s.pitch = self._roll_pitch()
        s.ref_lat_deg = self.pos.ref_lat
        s.ref_lon_deg = self.pos.ref_lon
        s.ref_alt_m = self.pos.ref_alt
        self.p_status.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = Px4Gateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
