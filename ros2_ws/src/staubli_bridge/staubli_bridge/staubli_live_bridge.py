#!/usr/bin/env python3
"""
staubli_live_bridge.py
──────────────────────
Serveur TCP bidirectionnel pour robot Stäubli.

Flux de données :
  Robot → PC  : {index;j1;j2;j3;j4;j5;j6;x;y;z;rx;ry;rz}  (toutes les 4 ms)
  PC    → Robot : {index;j1;j2;j3;j4;j5;j6}                 (sur demande ROS2)
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
from datetime import datetime

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
HOST = "0.0.0.0"
PORT = 2005


class StaubliBridgeROS2(Node):
    def __init__(self):
        super().__init__('staubli_live_bridge')

        # ── Publieurs ROS2 ────────────────────────────────────────────────────
        self.pub_joints = self.create_publisher(JointState,   '/joint_states',      10)
        self.pub_pose   = self.create_publisher(PoseStamped,  '/staubli/tcp_pose',  10)

        # ── Abonné ROS2 : commandes joints à envoyer au robot ────────────────
        # Un autre nœud (ex: MoveIt, script de téléop) publie sur ce topic
        # Message attendu : JointState avec 6 positions en radians
        self.sub_cmd = self.create_subscription(
            JointState,
            '/staubli/joint_cmd',
            self.cmd_callback,
            10
        )

        # ── Index des trames envoyées au robot ────────────────────────────────
        self.send_index = 0
        self.send_lock  = threading.Lock()   # Protection accès concurrent au socket

        # ── Fichier CSV ───────────────────────────────────────────────────────
        home_dir  = os.path.expanduser('~')
        desktop   = os.path.join(home_dir, 'Desktop')
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = os.path.join(desktop, f"robot_live_data_{timestamp}.csv")
        self.get_logger().info(f"📁 CSV : {self.csv_filename}")

        # ── Socket TCP ────────────────────────────────────────────────────────
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)

        self.get_logger().info(f"📡 Attente du robot sur le port {PORT}...")
        self.conn, addr = self.server_socket.accept()
        self.get_logger().info(f"✅ Robot connecté : {addr}")

        # ── Lancement de la boucle de réception dans un thread séparé ─────────
        # (pour ne pas bloquer rclpy.spin)
        self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.recv_thread.start()

    # ──────────────────────────────────────────────────────────────────────────
    # RÉCEPTION : Robot → PC
    # ──────────────────────────────────────────────────────────────────────────
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

                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        # Trame position : {index;j1;j2;j3;j4;j5;j6;x;y;z;rx;ry;rz}
                        if line.startswith("{") and line.endswith("}"):
                            content = line[1:-1]
                            parts   = content.split(';')

                            if len(parts) == 13:
                                # Sauvegarde CSV
                                writer.writerow([time.time()] + parts)
                                file.flush()

                                # Publication joints (rad)
                                js = JointState()
                                js.header.stamp = self.get_clock().now().to_msg()
                                js.name = ['joint_1','joint_2','joint_3',
                                           'joint_4','joint_5','joint_6']
                                js.position = [math.radians(float(p)) for p in parts[1:7]]
                                self.pub_joints.publish(js)

                                # Publication pose cartésienne
                                pose = PoseStamped()
                                pose.header.stamp    = self.get_clock().now().to_msg()
                                pose.header.frame_id = "world"

                                pose.pose.position.x = float(parts[7])  / 1000.0  # mm → m
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

    # ──────────────────────────────────────────────────────────────────────────
    # ENVOI : PC → Robot  (appelé par le subscriber /staubli/joint_cmd)
    # ──────────────────────────────────────────────────────────────────────────
    def cmd_callback(self, msg: JointState):
        """
        Reçoit une commande JointState (6 positions en radians),
        construit la trame {index;j1;j2;j3;j4;j5;j6} et l'envoie au robot.
        """
        if len(msg.position) < 6:
            self.get_logger().warn("Commande ignorée : moins de 6 joints.")
            return

        # Conversion rad → degrés (VAL3 travaille en degrés)
        joints_deg = [math.degrees(p) for p in msg.position[:6]]

        with self.send_lock:
            self.send_index += 1
            trame = "{{{};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f}}}".format(
                self.send_index,
                joints_deg[0], joints_deg[1], joints_deg[2],
                joints_deg[3], joints_deg[4], joints_deg[5]
            )
            try:
                self.conn.sendall((trame + "\n").encode('ascii'))
                self.get_logger().debug(f"→ Robot : {trame}")
            except Exception as e:
                self.get_logger().error(f"Erreur envoi commande : {e}")

    # ──────────────────────────────────────────────────────────────────────────
    # UTILITAIRES
    # ──────────────────────────────────────────────────────────────────────────
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Angles d'Euler (radians) → Quaternion [x, y, z, w]"""
        cy = math.cos(yaw   * 0.5);  sy = math.sin(yaw   * 0.5)
        cp = math.cos(pitch * 0.5);  sp = math.sin(pitch * 0.5)
        cr = math.cos(roll  * 0.5);  sr = math.sin(roll  * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy,  # z  ← correction signe
            cr * cp * cy + sr * sp * sy   # w
        ]


# ── MAIN ──────────────────────────────────────────────────────────────────────
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
