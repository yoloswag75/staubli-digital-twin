#!/usr/bin/env python3
"""
TEST 2 - Petit deplacement X,Y a vitesse reduite
=================================================
Objectif : Valider l'envoi de commandes cartesiennes au robot reel.
           Deplace le robot de 20mm en X puis 20mm en Y a 5mm/s.

ATTENTION : 
  - Verifier que l'espace autour du robot est libre
  - Avoir le bouton d'arret d'urgence a portee de main
  - Vitesse limitee a 5mm/s
  - Amplitude limitee a 20mm

Sequence de test :
  1. Lit la position courante du robot
  2. Calcule 4 points : +20X, retour, +20Y, retour
  3. Envoie les points a 5mm/s
  4. Verifie que le robot a bien atteint les positions

Pre-requis :
  - Bridge lance : ros2 run staubli_bridge bridge
  - Robot connecte et en position securisee

Usage :
  python3 test_2_small_move.py
  python3 test_2_small_move.py --amplitude 10  # 10mm au lieu de 20mm
  python3 test_2_small_move.py --dry-run        # simulation sans envoyer
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import argparse
import time
import math

AMPLITUDE_MM = 20.0   # deplacement en mm
VELOCITY_MMS = 5.0    # vitesse en mm/s
TOLERANCE_MM = 2.0    # tolerance d'arrivee en mm

MOVEJ = 0
MOVEL = 1


class SmallMoveTester(Node):
    def __init__(self, amplitude: float, velocity: float, dry_run: bool):
        super().__init__('test_2_small_move')
        self.amplitude = amplitude
        self.velocity  = velocity
        self.dry_run   = dry_run

        self.current_pose = None
        self.send_index   = 0
        self.errors       = []
        self.phase        = 'wait_position'
        self.sent_points  = []
        self.ack_count    = 0

        # Publishers / Subscribers
        self.pub_cart = self.create_publisher(
            PoseStamped, '/staubli/cart_cmd', 10)
        self.sub_pose = self.create_subscription(
            PoseStamped, '/staubli/tcp_pose', self.on_pose, 10)
        self.sub_ack = self.create_subscription(
            JointState, '/staubli/cmd_ack', self.on_ack, 10)

        self.get_logger().info("=" * 55)
        self.get_logger().info("TEST 2 - Petit deplacement X,Y")
        self.get_logger().info(f"Amplitude : {amplitude}mm | Vitesse : {velocity}mm/s")
        if dry_run:
            self.get_logger().info("MODE DRY RUN - aucune commande envoyee")
        self.get_logger().info("=" * 55)
        self.get_logger().info("En attente de la position courante du robot...")

    def on_pose(self, msg: PoseStamped):
        self.current_pose = msg

        if self.phase == 'wait_position':
            x = msg.pose.position.x * 1000
            y = msg.pose.position.y * 1000
            z = msg.pose.position.z * 1000
            rx = msg.pose.orientation.x
            ry = msg.pose.orientation.y
            rz = msg.pose.orientation.z
            rw = msg.pose.orientation.w

            self.get_logger().info(
                f"Position courante : x={x:.1f} y={y:.1f} z={z:.1f}mm")

            # Calcul des 4 points de test
            self.test_points = [
                # +X
                {'x': x + self.amplitude, 'y': y, 'z': z,
                 'qx': rx, 'qy': ry, 'qz': rz, 'qw': rw,
                 'label': f'+{self.amplitude}mm X'},
                # retour
                {'x': x, 'y': y, 'z': z,
                 'qx': rx, 'qy': ry, 'qz': rz, 'qw': rw,
                 'label': 'retour origine'},
                # +Y
                {'x': x, 'y': y + self.amplitude, 'z': z,
                 'qx': rx, 'qy': ry, 'qz': rz, 'qw': rw,
                 'label': f'+{self.amplitude}mm Y'},
                # retour
                {'x': x, 'y': y, 'z': z,
                 'qx': rx, 'qy': ry, 'qz': rz, 'qw': rw,
                 'label': 'retour origine'},
            ]

            self.phase = 'ready'
            self._confirm_and_run()

    def _confirm_and_run(self):
        print("\nPoints de test :")
        for i, pt in enumerate(self.test_points):
            print(f"  {i+1}. {pt['label']} -> ({pt['x']:.1f}, {pt['y']:.1f}, {pt['z']:.1f})mm")

        if self.dry_run:
            print("\nDRY RUN - validation OK, aucun mouvement envoye")
            self._final_report(True)
            raise SystemExit

        print(f"\nATTENTION : Le robot va se deplacer de {self.amplitude}mm a {self.velocity}mm/s")
        print("Appuyez sur ENTREE pour confirmer ou Ctrl+C pour annuler...")
        try:
            input()
        except KeyboardInterrupt:
            print("Test annule")
            raise SystemExit

        self.phase = 'running'
        self._send_all_points()

    def _send_all_points(self):
        for i, pt in enumerate(self.test_points):
            move_type = MOVEJ if i == 0 else MOVEL
            msg = PoseStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = f'world_{"movej" if move_type == MOVEJ else "movel"}_{self.velocity:.1f}'
            msg.pose.position.x = pt['x'] / 1000.0
            msg.pose.position.y = pt['y'] / 1000.0
            msg.pose.position.z = pt['z'] / 1000.0
            msg.pose.orientation.x = pt['qx']
            msg.pose.orientation.y = pt['qy']
            msg.pose.orientation.z = pt['qz']
            msg.pose.orientation.w = pt['qw']
            self.pub_cart.publish(msg)
            self.send_index += 1
            self.sent_points.append(pt['label'])
            self.get_logger().info(
                f"Envoi point {i+1}/4 : {pt['label']} a {self.velocity}mm/s")
            time.sleep(0.05)

    def on_ack(self, msg: JointState):
        self.ack_count += 1
        parts = msg.header.frame_id.split(';')
        ack_index = int(parts[0]) if parts else -1
        self.get_logger().info(f"ACK recu #{ack_index} ({self.ack_count}/4)")

        if self.ack_count >= 4:
            self._validate_final_position()

    def _validate_final_position(self):
        if self.current_pose is None:
            self.errors.append("Pas de position finale recue")
            self._final_report(False)
            raise SystemExit

        # Le robot devrait etre revenu a l'origine
        x = self.current_pose.pose.position.x * 1000
        y = self.current_pose.pose.position.y * 1000
        origin = self.test_points[0]
        dx = abs(x - (origin['x'] - self.amplitude))
        dy = abs(y - origin['y'])

        ok_pos = dx < self.tolerance and dy < self.tolerance
        if not ok_pos:
            self.errors.append(
                f"Position finale incorrecte : dx={dx:.1f}mm dy={dy:.1f}mm")

        self._final_report(ok_pos)
        raise SystemExit

    def _final_report(self, ok: bool):
        print("\n" + "=" * 55)
        print("RAPPORT FINAL TEST 2")
        print("=" * 55)
        print(f"RESULTAT : {'PASS' if ok else 'FAIL'}")
        print(f"Points envoyes : {self.send_index}")
        print(f"ACK recus      : {self.ack_count}")
        if self.errors:
            print("Erreurs :")
            for e in self.errors:
                print(f"  - {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--amplitude', type=float, default=AMPLITUDE_MM)
    parser.add_argument('--velocity',  type=float, default=VELOCITY_MMS)
    parser.add_argument('--dry-run',   action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = SmallMoveTester(args.amplitude, args.velocity, args.dry_run)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
