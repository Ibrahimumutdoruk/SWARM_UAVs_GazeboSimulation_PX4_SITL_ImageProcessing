#!/usr/bin/env python3
"""Madde 6: converts this UAV's PX4 EKF reference (ref_lat/ref_lon/ref_alt,
from px4_gateway_node's Px4Status) into the common FIELD_ENU frame, so all
UAVs report the same physical point in the same frame regardless of where
each one's PX4 local-NED origin actually is (architecture §6). Replaces the
static spawn_gz manifest constant with a value derived from real GNSS."""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from laplacian_interfaces.msg import Px4Status, LocalizationState
from laplacian_swarm import geo


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        dp = self.declare_parameter
        dp('field_origin_lat_deg', 0.0)
        dp('field_origin_lon_deg', 0.0)
        dp('field_origin_alt_m', 0.0)
        g = self.get_parameter
        self.origin_lat = float(g('field_origin_lat_deg').value)
        self.origin_lon = float(g('field_origin_lon_deg').value)
        self.origin_alt = float(g('field_origin_alt_m').value)

        bus = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Px4Status, 'px4/status', self._px4_cb, bus)
        self.p_loc = self.create_publisher(LocalizationState, 'localization/state', bus)
        self.get_logger().info(
            f'localization_node up, FIELD_ORIGIN=({self.origin_lat},{self.origin_lon},{self.origin_alt})')

    def _px4_cb(self, m):
        s = LocalizationState(); s.header.stamp = self.get_clock().now().to_msg()
        s.valid = m.estimator_ready and (m.ref_lat_deg != 0.0 or m.ref_lon_deg != 0.0)
        if s.valid:
            oe, on, ou = geo.wgs84_to_enu(m.ref_lat_deg, m.ref_lon_deg, m.ref_alt_m,
                                          self.origin_lat, self.origin_lon, self.origin_alt)
        else:
            oe = on = ou = 0.0
        s.px4_origin_field_enu = [oe, on, ou]
        # local NED (x=North,y=East,z=Down) -> ENU offset, then add the origin
        s.field_position_enu = [oe + m.position_ned[1],
                                on + m.position_ned[0],
                                ou - m.position_ned[2]]
        s.local_position_ned = list(m.position_ned)
        s.heading = m.heading
        self.p_loc.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
