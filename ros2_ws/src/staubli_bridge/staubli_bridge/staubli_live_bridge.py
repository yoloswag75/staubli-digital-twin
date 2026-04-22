#!/usr/bin/env python3
"""
staubli_live_bridge.py
──────────────────────
Serveur TCP bidirectionnel pour robot Stäubli.

Flux de données :
  Robot → PC  : {index;j1;j2;j3;j4;j5;j6;x;y;z;rx;ry;rz}  (toutes les 4 ms)
  PC    → Robot (joints)    : {index;j1;j2;j3;j4;j5;j6}      via /staubli/joint_cmd
  PC    → Robot (cartésien) : {index;type;x;y;z;rx;ry;rz}    via /staubli/cart_cmd
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import socket
import threading
import math
import csv
import os
import time
import yaml
from datetime import datetime

# ── CHARGEMENT CONFIG ─────────────────────────────────────────
def load_config():
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', '..', '..', '..', '..', '..', 'config', 'config.yaml'
    )
    config_path = os.path.abspath(config_path)
    if not os.path.exists(config_path):
        print("⚠️  config.yaml introuvable, valeurs par défaut utilisées")
        return {"network": {"pc_ip": "0.0.0.0", "port": 2005}}
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

CONFIG = load_config()
HOST   = CONFIG['network']['pc_ip']
PORT   = CONFIG['network']['port']

# Types de mouvement (doit correspondre à VAL3)
MOVEJ = 0
MOVEL = 1


class StaubliBridgeROS2(Node):
    def __init__(self):
        super().__init__('staubli_live_bridge')

        # ── Publieurs ─────────────────────────────────────────
        self.pub_joints = self.create_publisher(JointState,  '/joint_states',     10)
        self.pub_pose   = self.create_publisher(PoseStamped, '/staubli/tcp_pose', 10)
        self.pub_ack    = self.create_publisher(JointState,  '/staubli/cmd_ack',  10)

        # ── Subscribers ───────────────────────────────────────
        # Commandes articulaires (joints en radians)
        self.sub_joint_cmd = self.create_subscription(
            JointState, '/staubli/joint_cmd', self.joint_cmd_callback, 10
        )
        # Commandes cartésiennes (depuis dtx_player.py)
        self.sub_cart_cmd = self.create_subscription(
            PoseStamped, '/staubli/cart_cmd', self.cart_cmd_callback, 10
        )

        self.send_index = 0
        self.send_lock  = threading.Lock()

        # ── CSV ───────────────────────────────────────────────
        home_dir  = os.path.expanduser('~')
        desktop   = os.path.join(home_dir, 'Desktop')
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = os.path.join(desktop, f"robot_live_data_{timestamp}.csv")
        self.get_logger().info(f"📁 CSV : {self.csv_filename}")

        # ── Socket TCP ────────────────────────────────────────
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)
        self.get_logger().info(f"📡 Attente du robot sur {HOST}:{PORT}...")
        self.conn, addr = self.server_socket.accept()
        self.get_logger().info(f"✅ Robot connecté : {addr}")

        self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.recv_thread.start()

    # ──────────────────────────────────────────────────────────
    # RÉCEPTION : Robot → PC
    # ──────────────────────────────────────────────────────────
    def receive_loop(self):
        buffer = ""
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["PC_Timestamp", "Index",
                             "J1", "J2", "J3", "J4", "J5", "J6",
                             "X", "Y", "Z", "RX", "RY", "RZ"])
            while rclpy.ok():
                try:
                    data = self.conn.recv(1024).decode('ascii')
                    if not data:
                        self.get_logger().warn("🔌 Connexion coupée par le robot.")
                        break
                    self.get_logger().info(f"TCP recu: {repr(data[:100])}")
                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line.startswith("{") and line.endswith("}"):
                            content = line[1:-1]
                            parts   = content.split(';')
                            # Trame ACK : {ACK;index;n_restant}
                            if len(parts) >= 2 and parts[0] == 'ACK':
                                try:
                                    ack_index  = int(parts[1])
                                    n_restant  = int(parts[2]) if len(parts) > 2 else -1
                                    ack_msg = JointState()
                                    ack_msg.header.stamp    = self.get_clock().now().to_msg()
                                    ack_msg.header.frame_id = f"{ack_index};{n_restant}"
                                    self.pub_ack.publish(ack_msg)
                                    self.get_logger().info(f"ACK recu #{ack_index} | file: {n_restant} restants")
                                except ValueError:
                                    pass

                            if len(parts) == 13:
                                writer.writerow([time.time()] + parts)
                                file.flush()
                                js = JointState()
                                js.header.stamp = self.get_clock().now().to_msg()
                                js.name = ['joint_1','joint_2','joint_3',
                                           'joint_4','joint_5','joint_6']
                                js.position = [math.radians(float(p)) for p in parts[1:7]]
                                self.pub_joints.publish(js)
                                pose = PoseStamped()
                                pose.header.stamp    = self.get_clock().now().to_msg()
                                pose.header.frame_id = "world"
                                pose.pose.position.x = float(parts[7])  / 1000.0
                                pose.pose.position.y = float(parts[8])  / 1000.0
                                pose.pose.position.z = float(parts[9])  / 1000.0
                                rx = math.radians(float(parts[10]))
                                ry = math.radians(float(parts[11]))
                                rz = math.radians(float(parts[12]))
                                q  = self.euler_to_quaternion(rx, ry, rz)
                                pose.pose.orientation.x = q[0]
                                pose.pose.orientation.y = q[1]
                                pose.pose.orientation.z = q[2]
                                pose.pose.orientation.w = q[3]
                                self.pub_pose.publish(pose)
                except Exception as e:
                    self.get_logger().error(f"Erreur réception : {e}")
                    break

    # ──────────────────────────────────────────────────────────
    # ENVOI ARTICULAIRE : /staubli/joint_cmd → Robot
    # ──────────────────────────────────────────────────────────
    def joint_cmd_callback(self, msg: JointState):
        """Commande en angles joints (radians → degrés)."""
        if len(msg.position) < 6:
            self.get_logger().warn("Commande joints ignorée : moins de 6 joints.")
            return
        joints_deg = [math.degrees(p) for p in msg.position[:6]]
        with self.send_lock:
            self.send_index += 1
            # Type 0 = movej implicite (ancien format conservé)
            trame = "{{{};0;{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f}}}".format(
                self.send_index,
                joints_deg[0], joints_deg[1], joints_deg[2],
                joints_deg[3], joints_deg[4], joints_deg[5]
            )
            self._send(trame)
            self.get_logger().info(f"Trame envoyee au robot: {trame}")

    # ──────────────────────────────────────────────────────────
    # ENVOI CARTÉSIEN : /staubli/cart_cmd → Robot
    # ──────────────────────────────────────────────────────────
    def cart_cmd_callback(self, msg: PoseStamped):
        self.get_logger().info(f"cart_cmd recu: {msg.header.frame_id}")
        """
        Commande cartésienne depuis dtx_player.py.
        Le type (movej/movel) est encodé dans frame_id :
          'world_movej' → MOVEJ (0)
          'world_movel' → MOVEL (1)
        """
        # Décodage du type depuis le frame_id
        move_type = MOVEJ if 'movej' in msg.header.frame_id else MOVEL
        try:
            vel = float(msg.header.frame_id.split('_')[-1])
        except (ValueError, IndexError):
            vel = 50.0

        # Conversion m → mm (VAL3 travaille en mm)
        x  = msg.pose.position.x * 1000.0
        y  = msg.pose.position.y * 1000.0
        z  = msg.pose.position.z * 1000.0

        # Conversion quaternion → Euler degrés
        rx, ry, rz = self.quaternion_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        rx = math.degrees(rx)
        ry = math.degrees(ry)
        rz = math.degrees(rz)

        with self.send_lock:
            self.send_index += 1
            trame = "{{{};{};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f}}}".format(
                self.send_index, move_type,
                x, y, z, rx, ry, rz, vel
            )
            self._send(trame)
            self.get_logger().info(f"Trame envoyee au robot: {trame}")

    # ──────────────────────────────────────────────────────────
    # UTILITAIRES
    # ──────────────────────────────────────────────────────────
    def _send(self, trame: str):
        try:
            self.conn.sendall((trame + "\n").encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Erreur envoi : {e}")

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

    def quaternion_to_euler(self, x, y, z, w):
        """Quaternion → angles d'Euler (roll, pitch, yaw) en radians."""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll  = math.atan2(sinr_cosp, cosr_cosp)
        sinp  = 2 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw   = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = StaubliBridgeROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server_socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
