#!/usr/bin/env python3
"""
send_command.py
───────────────
Envoie une commande joints au bridge ROS2 via le topic /staubli/joint_cmd.
Utilitaire de test rapide sans avoir à écrire un nœud ROS2 complet.

Usage :
  python3 send_command.py                        # position home
  python3 send_command.py 0 -30 90 0 45 0       # angles en degrés
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import math


class CommandSender(Node):
    def __init__(self, joints_deg):
        super().__init__('command_sender')
        self.pub = self.create_publisher(JointState, '/staubli/joint_cmd', 10)
        self.joints_deg = joints_deg
        # Petit délai pour laisser le publisher s'enregistrer
        self.timer = self.create_timer(0.5, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        msg.position = [math.radians(d) for d in self.joints_deg]

        self.pub.publish(msg)
        self.get_logger().info(
            f"✅ Commande envoyée : {[f'{d:.1f}°' for d in self.joints_deg]}"
        )
        self.sent = True
        # Quitter après envoi
        raise SystemExit


def main():
    # Lire les angles depuis les arguments, sinon position home
    if len(sys.argv) == 7:
        try:
            joints = [float(a) for a in sys.argv[1:]]
        except ValueError:
            print("❌ Angles invalides. Usage : python3 send_command.py j1 j2 j3 j4 j5 j6")
            sys.exit(1)
    else:
        # Position home RX160L
        joints = [0.0, -30.0, 90.0, 0.0, 45.0, 0.0]
        print(f"ℹ️  Aucun angle fourni, utilisation de la position home : {joints}")

    rclpy.init()
    node = CommandSender(joints)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
