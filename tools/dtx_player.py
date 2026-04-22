#!/usr/bin/env python3
"""
dtx_player.py
─────────────
Parse un fichier .dtx VAL3 (collection de pointRx cartésiens)
et envoie la trajectoire au robot via le bridge ROS2.

Protocole envoyé au robot :
  {index;type;x;y;z;rx;ry;rz}\n
  type = 0 → movej (premier point)
  type = 1 → movel (points suivants)

Usage :
  python3 tools/dtx_player.py chemin/vers/fichier.dtx
  python3 tools/dtx_player.py chemin/vers/fichier.dtx --speed 0.05
  python3 tools/dtx_player.py chemin/vers/fichier.dtx --dry-run
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET
import sys
import os
import time
import math
import argparse
import re


# ── TYPES DE MOUVEMENT ────────────────────────────────────────
MOVEJ = 0   # Premier point : optimise le trajet (cinématique inverse VAL3)
MOVEL = 1   # Points suivants : ligne droite cartésienne


# ── PARSER DTX ────────────────────────────────────────────────
def parse_dtx(filepath: str) -> list:
    """
    Parse un fichier .dtx VAL3 et retourne une liste ordonnée de points.
    Chaque point : {'key': str, 'x': float, 'y': float, 'z': float,
                    'rx': float, 'ry': float, 'rz': float}
    """
    # Le fichier peut avoir des caractères invalides — on nettoie
    with open(filepath, 'rb') as f:
        raw = f.read()

    # Décodage avec remplacement des caractères invalides
    content = raw.decode('utf-8', errors='replace')

    # Certains fichiers VAL3 ont des sauts de ligne dans les clés → on nettoie
    content = content.replace('\r\n', '\n').replace('\r', '\n')

    # Parser XML
    try:
        root = ET.fromstring(content)
    except ET.ParseError:
        # Fallback : parser ligne par ligne avec regex
        return parse_dtx_regex(content)

    ns = {'val3': 'http://www.staubli.com/robotics/VAL3/Data/2'}
    points = []

    for data in root.findall('.//val3:Data', ns):
        if data.get('type') == 'pointRx':
            for value in data.findall('val3:Value', ns):
                try:
                    points.append({
                        'key': value.get('key', ''),
                        'x':   float(value.get('x',  0)),
                        'y':   float(value.get('y',  0)),
                        'z':   float(value.get('z',  0)),
                        'rx':  float(value.get('rx', 0)),
                        'ry':  float(value.get('ry', 0)),
                        'rz':  float(value.get('rz', 0)),
                    })
                except (ValueError, TypeError):
                    continue

    return points


def parse_dtx_regex(content: str) -> list:
    """Fallback parser par regex si le XML est malformé."""
    pattern = re.compile(
        r'x="([^"]+)"\s+y="([^"]+)"\s+z="([^"]+)"\s+'
        r'rx="([^"]+)"\s+ry="([^"]+)"\s+rz="([^"]+)"'
    )
    points = []
    for i, m in enumerate(pattern.finditer(content)):
        try:
            points.append({
                'key': f'point_{i}',
                'x':  float(m.group(1)),
                'y':  float(m.group(2)),
                'z':  float(m.group(3)),
                'rx': float(m.group(4)),
                'ry': float(m.group(5)),
                'rz': float(m.group(6)),
            })
        except ValueError:
            continue
    return points


# ── NŒUD ROS2 ─────────────────────────────────────────────────
class DtxPlayer(Node):
    def __init__(self, points: list, speed: float, dry_run: bool):
        super().__init__('dtx_player')

        self.points  = points
        self.speed   = speed    # Délai entre les points (secondes)
        self.dry_run = dry_run
        self.index   = 0

        # Publisher vers le bridge
        self.pub = self.create_publisher(
            PoseStamped,
            '/staubli/cart_cmd',   # Nouveau topic cartésien
            10
        )

        self.get_logger().info(f"📂 {len(points)} points chargés")
        self.get_logger().info(f"⏱️  Délai entre points : {speed}s")

        if dry_run:
            self.get_logger().info("🔍 Mode DRY RUN — aucune commande envoyée")
            self.dry_run_display()
            raise SystemExit

        # Timer pour envoyer les points un par un
        self.timer = self.create_timer(self.speed, self.send_next_point)

    def send_next_point(self):
        if self.index >= len(self.points):
            self.get_logger().info("✅ Trajectoire terminée !")
            self.timer.cancel()
            raise SystemExit

        pt = self.points[self.index]

        # Type de mouvement
        move_type = MOVEJ if self.index == 0 else MOVEL
        type_str  = "movej" if move_type == MOVEJ else "movel"

        # Construction du message PoseStamped
        # On encode le type dans le frame_id : "world_movej" ou "world_movel"
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = f"world_{type_str}"

        # Position en mètres (VAL3 envoie en mm)
        msg.pose.position.x = pt['x'] / 1000.0
        msg.pose.position.y = pt['y'] / 1000.0
        msg.pose.position.z = pt['z'] / 1000.0

        # Orientation en quaternion (depuis Euler en degrés)
        q = self.euler_to_quaternion(
            math.radians(pt['rx']),
            math.radians(pt['ry']),
            math.radians(pt['rz'])
        )
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub.publish(msg)

        self.get_logger().info(
            f"[{self.index+1}/{len(self.points)}] {type_str} → "
            f"x={pt['x']:.1f} y={pt['y']:.1f} z={pt['z']:.1f} mm"
        )

        self.index += 1

    def dry_run_display(self):
        """Affiche les points sans envoyer."""
        print(f"\n{'─'*60}")
        print(f"{'#':>5}  {'TYPE':>6}  {'X':>8}  {'Y':>8}  {'Z':>8}  {'RZ':>8}")
        print(f"{'─'*60}")
        for i, pt in enumerate(self.points[:20]):  # Affiche les 20 premiers
            t = "movej" if i == 0 else "movel"
            print(f"{i+1:>5}  {t:>6}  {pt['x']:>8.2f}  {pt['y']:>8.2f}  "
                  f"{pt['z']:>8.2f}  {pt['rz']:>8.3f}")
        if len(self.points) > 20:
            print(f"  ... ({len(self.points) - 20} points supplémentaires)")
        print(f"{'─'*60}\n")

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw   * 0.5);  sy = math.sin(yaw   * 0.5)
        cp = math.cos(pitch * 0.5);  sp = math.sin(pitch * 0.5)
        cr = math.cos(roll  * 0.5);  sr = math.sin(roll  * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ]


# ── MAIN ──────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Joue une trajectoire depuis un fichier .dtx VAL3')
    parser.add_argument('dtx_file',          help='Chemin vers le fichier .dtx')
    parser.add_argument('--speed',  type=float, default=0.1,
                        help='Délai entre les points en secondes (défaut: 0.1)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Affiche les points sans envoyer au robot')
    args = parser.parse_args()

    if not os.path.exists(args.dtx_file):
        print(f"❌ Fichier introuvable : {args.dtx_file}")
        sys.exit(1)

    print(f"📂 Chargement de {args.dtx_file}...")
    points = parse_dtx(args.dtx_file)

    if not points:
        print("❌ Aucun point trouvé dans le fichier .dtx")
        sys.exit(1)

    print(f"✅ {len(points)} points chargés")

    rclpy.init()
    node = DtxPlayer(points, args.vel, args.dry_run)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
