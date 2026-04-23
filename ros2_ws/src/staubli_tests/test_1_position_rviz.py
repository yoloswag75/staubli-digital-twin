#!/usr/bin/env python3
"""
TEST 1 - Reception position robot + visualisation RViz
=======================================================
Objectif : Verifier que le bridge recoit correctement les positions
           du robot reel et les publie sur ROS2.

Criteres de validation :
  [OK] /joint_states publie a ~240Hz
  [OK] /staubli/tcp_pose publie a ~240Hz
  [OK] Le robot s'affiche et bouge dans RViz
  [OK] Les valeurs de joints sont coherentes (dans les limites du RX160L)
  [OK] La position TCP est dans l'espace de travail attendu

Pre-requis :
  - Bridge lance : ros2 run staubli_bridge bridge
  - RViz lance   : ros2 launch staubli_description display.launch.py
  - Robot connecte et programme VAL3 en cours d'execution

Usage :
  python3 test_1_position_rviz.py
  python3 test_1_position_rviz.py --duration 30  # test sur 30 secondes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import argparse
import time
import math

# Limites articulaires RX160L (degres)
JOINT_LIMITS = [
    (-160, 160),   # J1
    (-137, 137),   # J2
    (-150, 150),   # J3
    (-270, 270),   # J4
    (-105, 120),   # J5
    (-270, 270),   # J6
]

# Espace de travail approximatif (mm)
WORKSPACE = {
    'x': (-1000, 1000),
    'y': (-1000, 1000),
    'z': (-500,  1500),
}


class PositionTester(Node):
    def __init__(self, duration: int):
        super().__init__('test_1_position_rviz')
        self.duration   = duration
        self.start_time = time.time()

        # Compteurs
        self.joint_count  = 0
        self.pose_count   = 0
        self.errors       = []
        self.last_joints  = None
        self.last_pose    = None

        # Subscribers
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.on_joints, 10)
        self.sub_pose = self.create_subscription(
            PoseStamped, '/staubli/tcp_pose', self.on_pose, 10)

        # Timer de rapport
        self.timer = self.create_timer(5.0, self.report)

        self.get_logger().info("=" * 55)
        self.get_logger().info("TEST 1 - Reception position + RViz")
        self.get_logger().info(f"Duree : {duration}s")
        self.get_logger().info("Bougez le robot manuellement pour valider")
        self.get_logger().info("=" * 55)

    def on_joints(self, msg: JointState):
        self.joint_count += 1
        self.last_joints = msg

        # Verification des limites
        for i, pos in enumerate(msg.position):
            deg = math.degrees(pos)
            lo, hi = JOINT_LIMITS[i]
            if not (lo <= deg <= hi):
                self.errors.append(
                    f"J{i+1}={deg:.1f} hors limites [{lo},{hi}]")

    def on_pose(self, msg: PoseStamped):
        self.pose_count += 1
        self.last_pose = msg

        # Verification espace de travail
        x = msg.pose.position.x * 1000
        y = msg.pose.position.y * 1000
        z = msg.pose.position.z * 1000
        for axis, val, (lo, hi) in [('X', x, WORKSPACE['x']),
                                     ('Y', y, WORKSPACE['y']),
                                     ('Z', z, WORKSPACE['z'])]:
            if not (lo <= val <= hi):
                self.errors.append(f"TCP {axis}={val:.0f}mm hors workspace")

    def report(self):
        elapsed = time.time() - self.start_time
        hz_j = self.joint_count / elapsed if elapsed > 0 else 0
        hz_p = self.pose_count  / elapsed if elapsed > 0 else 0

        print(f"\n--- Rapport t={elapsed:.0f}s ---")
        print(f"  /joint_states   : {hz_j:.0f} Hz  ({self.joint_count} msgs)")
        print(f"  /tcp_pose       : {hz_p:.0f} Hz  ({self.pose_count} msgs)")

        if self.last_joints:
            degs = [f"{math.degrees(p):.1f}" for p in self.last_joints.position]
            print(f"  Joints (deg)    : {degs}")

        if self.last_pose:
            p = self.last_pose.pose.position
            print(f"  TCP (mm)        : x={p.x*1000:.1f} y={p.y*1000:.1f} z={p.z*1000:.1f}")

        # Validation
        ok_hz_j = hz_j > 200
        ok_hz_p = hz_p > 200
        ok_err  = len(self.errors) == 0

        print(f"\n  [{'OK' if ok_hz_j else 'FAIL'}] /joint_states freq > 200Hz : {hz_j:.0f}Hz")
        print(f"  [{'OK' if ok_hz_p else 'FAIL'}] /tcp_pose freq > 200Hz     : {hz_p:.0f}Hz")
        print(f"  [{'OK' if ok_err  else 'FAIL'}] Aucune erreur de limites   : {len(self.errors)} erreur(s)")

        if self.errors:
            for e in self.errors[-3:]:
                print(f"    -> {e}")

        if elapsed >= self.duration:
            self._final_report(ok_hz_j, ok_hz_p, ok_err)
            raise SystemExit

    def _final_report(self, ok_hz_j, ok_hz_p, ok_err):
        print("\n" + "=" * 55)
        print("RAPPORT FINAL TEST 1")
        print("=" * 55)
        all_ok = ok_hz_j and ok_hz_p and ok_err
        print(f"RESULTAT : {'PASS' if all_ok else 'FAIL'}")
        if not all_ok:
            print("Actions correctives :")
            if not ok_hz_j:
                print("  - Verifier que le bridge est lance")
                print("  - Verifier la connexion TCP avec le robot")
            if not ok_hz_p:
                print("  - Verifier tPosition.pgx sur le CS8")
            if not ok_err:
                print("  - Verifier la calibration du robot")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=int, default=20,
                        help='Duree du test en secondes (defaut: 20)')
    args = parser.parse_args()

    rclpy.init()
    node = PositionTester(args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
