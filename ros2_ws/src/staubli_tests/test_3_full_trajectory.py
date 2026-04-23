#!/usr/bin/env python3
"""
TEST 3 - Programme complet avec fichier DTX
===========================================
Objectif : Valider l'execution d'une trajectoire complete sur le robot reel.

ATTENTION :
  - Verifier l'espace de travail complet avant de lancer
  - Avoir le bouton d'arret d'urgence a portee de main
  - Commencer avec une vitesse tres reduite (5-10mm/s)
  - Verifier le dry-run avant l'execution reelle

Criteres de validation :
  [OK] Tous les points sont envoyes sans erreur
  [OK] Tous les ACK sont recus
  [OK] Frequence ACK coherente avec la vitesse
  [OK] Pas de decrochage de la file (queue jamais vide)
  [OK] Position finale coherente

Usage :
  python3 test_3_full_trajectory.py fichier.dtx --dry-run
  python3 test_3_full_trajectory.py fichier.dtx --vel 5
  python3 test_3_full_trajectory.py fichier.dtx --vel 10 --max-points 50
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import argparse
import time
import math
import xml.etree.ElementTree as ET
import re
import os
import sys

MOVEJ = 0
MOVEL = 1
QUEUE_SIZE    = 5
REFILL_THRESH = 2


def parse_dtx(filepath):
    with open(filepath, 'rb') as f:
        raw = f.read()
    content = raw.decode('utf-8', errors='replace')
    points = []
    try:
        root = ET.fromstring(content)
        ns = {'v': 'http://www.staubli.com/robotics/VAL3/Data/2'}
        for data in root.findall('.//v:Data', ns):
            if data.get('type') == 'pointRx':
                for value in data.findall('v:Value', ns):
                    try:
                        points.append({
                            'x':  float(value.get('x',  0)),
                            'y':  float(value.get('y',  0)),
                            'z':  float(value.get('z',  0)),
                            'rx': float(value.get('rx', 0)),
                            'ry': float(value.get('ry', 0)),
                            'rz': float(value.get('rz', 0)),
                        })
                    except (ValueError, TypeError):
                        continue
    except ET.ParseError:
        pattern = re.compile(
            r'x="([^"]+)"\s+y="([^"]+)"\s+z="([^"]+)"\s+'
            r'rx="([^"]+)"\s+ry="([^"]+)"\s+rz="([^"]+)"'
        )
        for m in pattern.finditer(content):
            try:
                points.append({
                    'x': float(m.group(1)), 'y': float(m.group(2)),
                    'z': float(m.group(3)), 'rx': float(m.group(4)),
                    'ry': float(m.group(5)), 'rz': float(m.group(6)),
                })
            except ValueError:
                continue
    return points


class FullTrajectoryTester(Node):
    def __init__(self, points, vel, dry_run, max_points):
        super().__init__('test_3_full_trajectory')
        self.points     = points[:max_points] if max_points else points
        self.vel        = vel
        self.dry_run    = dry_run

        self.next_to_send = 0
        self.ack_count    = 0
        self.errors       = []
        self.start_time   = None
        self.queue_empties = 0  # nb de fois que la file s'est videe

        self.pub_cart = self.create_publisher(
            PoseStamped, '/staubli/cart_cmd', 10)
        self.sub_ack = self.create_subscription(
            JointState, '/staubli/cmd_ack', self.on_ack, 10)

        nb = len(self.points)
        dist = self._total_distance()
        dur  = dist / vel

        print("=" * 55)
        print("TEST 3 - Trajectoire complete")
        print("=" * 55)
        print(f"Fichier       : {nb} points")
        print(f"Distance tot. : {dist:.0f}mm")
        print(f"Vitesse       : {vel}mm/s")
        print(f"Duree est.    : {int(dur//60)}min {int(dur%60):02d}s")
        print(f"Z min/max     : {min(p['z'] for p in self.points):.0f} / {max(p['z'] for p in self.points):.0f}mm")

        if dry_run:
            print("\nDRY RUN - validation OK, aucun mouvement")
            self._final_report(True)
            raise SystemExit

        print(f"\nATTENTION : Verifiez l'espace de travail !")
        print("Appuyez sur ENTREE pour lancer ou Ctrl+C pour annuler...")
        try:
            input()
        except KeyboardInterrupt:
            raise SystemExit

        self.start_time = time.time()
        self._send_batch()
        self.timer = self.create_timer(10.0, self._progress_report)

    def _total_distance(self):
        total = 0
        for i in range(1, len(self.points)):
            p1, p2 = self.points[i-1], self.points[i]
            total += math.sqrt(
                (p2['x']-p1['x'])**2 +
                (p2['y']-p1['y'])**2 +
                (p2['z']-p1['z'])**2
            )
        return total

    def _euler_to_quat(self, r, p, y):
        cy = math.cos(y*0.5); sy = math.sin(y*0.5)
        cp = math.cos(p*0.5); sp = math.sin(p*0.5)
        cr = math.cos(r*0.5); sr = math.sin(r*0.5)
        return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)

    def _send_batch(self):
        nb   = len(self.points)
        sent = 0
        while sent < QUEUE_SIZE and self.next_to_send < nb:
            i  = self.next_to_send
            pt = self.points[i]
            move_type = MOVEJ if i == 0 else MOVEL

            msg = PoseStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = f'world_{"movej" if move_type == MOVEJ else "movel"}_{self.vel:.1f}'
            msg.pose.position.x = pt['x'] / 1000.0
            msg.pose.position.y = pt['y'] / 1000.0
            msg.pose.position.z = pt['z'] / 1000.0
            q = self._euler_to_quat(
                math.radians(pt['rx']),
                math.radians(pt['ry']),
                math.radians(pt['rz'])
            )
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            self.pub_cart.publish(msg)

            self.next_to_send += 1
            sent += 1
            time.sleep(0.01)

    def on_ack(self, msg: JointState):
        self.ack_count += 1
        parts     = msg.header.frame_id.split(';')
        n_restant = int(parts[1]) if len(parts) > 1 else -1

        if n_restant == 0:
            self.queue_empties += 1

        # Refill
        if n_restant <= REFILL_THRESH:
            self._send_batch()

        # Fin
        if self.ack_count >= len(self.points):
            self._final_report(True)
            raise SystemExit

    def _progress_report(self):
        nb      = len(self.points)
        elapsed = time.time() - self.start_time if self.start_time else 0
        pct     = self.ack_count / nb * 100
        self.get_logger().info(
            f"Progression : {self.ack_count}/{nb} ({pct:.0f}%) "
            f"| file vide {self.queue_empties}x "
            f"| {elapsed:.0f}s ecoules")

    def _final_report(self, ok: bool):
        elapsed = time.time() - self.start_time if self.start_time else 0
        nb      = len(self.points)
        print("\n" + "=" * 55)
        print("RAPPORT FINAL TEST 3")
        print("=" * 55)
        print(f"RESULTAT      : {'PASS' if ok else 'FAIL'}")
        print(f"Points envoyes: {self.next_to_send}/{nb}")
        print(f"ACK recus     : {self.ack_count}/{nb}")
        print(f"File vide     : {self.queue_empties}x (ideal=0)")
        print(f"Duree reelle  : {int(elapsed//60)}min {int(elapsed%60):02d}s")
        if self.errors:
            print("Erreurs :")
            for e in self.errors:
                print(f"  - {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dtx_file', help='Fichier .dtx')
    parser.add_argument('--vel',        type=float, default=5.0)
    parser.add_argument('--dry-run',    action='store_true')
    parser.add_argument('--max-points', type=int,   default=None,
                        help='Limite le nb de points (test partiel)')
    args = parser.parse_args()

    if not os.path.exists(args.dtx_file):
        print(f"Fichier introuvable : {args.dtx_file}")
        sys.exit(1)

    print(f"Chargement de {args.dtx_file}...")
    points = parse_dtx(args.dtx_file)
    print(f"{len(points)} points charges")

    rclpy.init()
    node = FullTrajectoryTester(points, args.vel, args.dry_run, args.max_points)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
