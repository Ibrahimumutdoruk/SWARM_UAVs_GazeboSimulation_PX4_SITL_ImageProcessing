#!/usr/bin/env python3
"""One-shot START (Gorev1 'tek kalkis komutu'). Publishes a single latched
MissionCommand (phase=START, target_qr=FIRST_QR, initial formation/altitude) on
/swarm/mission, then exits. NOT a commander - agents act autonomously after."""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from laplacian_interfaces.msg import MissionCommand
from laplacian_swarm import formations, field


class Trigger(Node):
    def __init__(self):
        super().__init__('mission_trigger')
        self.declare_parameter('formation', 'CIZGI')   # V | CIZGI | OK
        self.declare_parameter('spacing', 5.0)
        self.declare_parameter('altitude', 15.0)
        self.declare_parameter('first_qr', field.FIRST_QR)
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(MissionCommand, '/swarm/mission', qos)
        self.sent = 0
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        m = MissionCommand(); m.header.stamp = self.get_clock().now().to_msg()
        m.seq = 1; m.phase = 0
        m.target_qr = int(self.get_parameter('first_qr').value)
        m.formation = formations.by_name(self.get_parameter('formation').value)
        m.spacing = float(self.get_parameter('spacing').value)
        m.altitude = float(self.get_parameter('altitude').value)
        self.pub.publish(m)
        self.sent += 1
        self.get_logger().info(f'START sent ({self.sent})  qr={m.target_qr} form={m.formation}')
        if self.sent >= 6:
            self.timer.cancel()
            rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Trigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()