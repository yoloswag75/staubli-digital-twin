#!/usr/bin/env python3
"""
robot_simulator.py
──────────────────
Simule le robot Stäubli RX160L côté TCP.
Joue le rôle du code VAL3 (tPosition + tCommande) sans avoir SRS.

Comportement :
  - Se connecte au bridge ROS2 (comme le ferait le vrai robot)
  - Envoie la position courante toutes les 4ms
  - Reçoit les commandes cartésiennes {index;type;x;y;z;rx;ry;rz}
  - Calcule la cinématique inverse (roboticstoolbox) → publie /joint_states
  - Le robot s'anime dans RViz en suivant la trajectoire
"""

import socket
import threading
import time
import math
import yaml
import pathlib
import numpy as np

# ── CINÉMATIQUE INVERSE ───────────────────────────────────────
try:
    import roboticstoolbox as rtb
    from spatialmath import SE3

    RX160L = rtb.DHRobot([
        rtb.RevoluteDH(d=0.550, a=0.150,  alpha=-np.pi/2),  # J1
        rtb.RevoluteDH(d=0,     a=0.825,  alpha=0),          # J2
        rtb.RevoluteDH(d=0,     a=0,      alpha=-np.pi/2),   # J3
        rtb.RevoluteDH(d=0.625, a=0,      alpha= np.pi/2),   # J4
        rtb.RevoluteDH(d=0,     a=0,      alpha=-np.pi/2),   # J5
        rtb.RevoluteDH(d=0.110, a=0,      alpha=0),          # J6
    ], name='RX160L')

    IK_AVAILABLE = True
    print("✅ Cinématique inverse disponible (roboticstoolbox)")

except ImportError:
    IK_AVAILABLE = False
    print("⚠️  roboticstoolbox non disponible — joints fixes à la position home")

# ── CONFIGURATION ─────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def load_config():
    config_path = pathlib.Path(__file__).parent.parent / 'config' / 'config.yaml'
    if not config_path.exists():
        return {"simulator": {"sim_ip": "127.0.0.1"},
                "network":   {"port": 2005},
                "robot":     {"send_freq": 0.004}}
    with open(config_path) as f:
        return yaml.safe_load(f)

_cfg = load_config()
HOST = _cfg['simulator']['sim_ip']
PORT = _cfg['network']['port']
FREQ = _cfg['robot']['send_freq']

MOVEJ = 0
MOVEL = 1

# Position home RX160L en radians
Q_HOME = np.radians([0.0, -30.0, 90.0, 0.0, 45.0, 0.0])


def ik_solve(x, y, z, rx, ry, rz, q_prev):
    """
    Cinématique inverse : (x,y,z mm + rx,ry,rz degrés) → angles joints (degrés)
    Retourne None si pas de solution.
    """
    if not IK_AVAILABLE:
        return None
    try:
        T = SE3(x/1000, y/1000, z/1000) * SE3.RPY(
            [math.radians(rx), math.radians(ry), math.radians(rz)]
        )
        sol = RX160L.ikine_LM(T, q0=np.radians(q_prev))
        if sol.success:
            return list(np.degrees(sol.q))
    except Exception as e:
        print(f"⚠️  IK échouée : {e}")
    return None


# ── NŒUD ROS2 pour publier /joint_states ─────────────────────
class JointPublisher(Node):
    def __init__(self):
        super().__init__('robot_simulator_joints')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

    def publish(self, joints_deg):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ['joint_1','joint_2','joint_3',
                        'joint_4','joint_5','joint_6']
        msg.position = [math.radians(d) for d in joints_deg]
        self.pub.publish(msg)


# ── SIMULATEUR ────────────────────────────────────────────────
class RobotSimulator:
    def __init__(self, ros_node: JointPublisher):
        self.frame_index = 0
        self.running     = True
        self.lock        = threading.Lock()
        self.ros_node    = ros_node

        # Position articulaire courante (degrés)
        self.joints = [0.0, -30.0, 90.0, 0.0, 45.0, 0.0]

        # Position cartésienne courante (mm + degrés)
        self.x,  self.y,  self.z  = 800.0, 0.0, 600.0
        self.rx, self.ry, self.rz = 0.0, 90.0, 0.0

        # Connexion TCP
        print(f"🔌 Connexion au bridge sur {HOST}:{PORT}...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        print("✅ Connecté au bridge ROS2 !")
        print("📌 Position initiale : home (0, -30, 90, 0, 45, 0)°\n")

    def send_loop(self):
        """Envoie la position toutes les 4ms — simule tPosition.pgx."""
        while self.running:
            start = time.time()
            self.frame_index += 1

            with self.lock:
                j              = self.joints[:]
                x, y, z        = self.x, self.y, self.z
                rx, ry, rz     = self.rx, self.ry, self.rz

            trame = "{{{};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f}}}\n".format(
                self.frame_index,
                j[0], j[1], j[2], j[3], j[4], j[5],
                x, y, z, rx, ry, rz
            )

            try:
                self.sock.sendall(trame.encode('ascii'))
            except Exception as e:
                print(f"❌ Erreur envoi : {e}")
                self.running = False
                break

            if self.frame_index % 250 == 0:
                with self.lock:
                    print(f"📡 #{self.frame_index} | "
                          f"TCP=({self.x:.1f}, {self.y:.1f}, {self.z:.1f})mm | "
                          f"j1={self.joints[0]:.1f}°")

            elapsed = time.time() - start
            sleep_time = FREQ - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def recv_loop(self):
        """Reçoit les commandes cartésiennes et calcule la cinématique inverse."""
        buffer = ""
        while self.running:
            try:
                data = self.sock.recv(1024).decode('ascii')
                if not data:
                    print("🔌 Bridge déconnecté.")
                    self.running = False
                    break

                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()

                    if line.startswith("{") and line.endswith("}"):
                        content = line[1:-1]
                        parts   = content.split(';')

                        # Format : {index;type;x;y;z;rx;ry;rz}
                        if len(parts) == 9:
                            idx       = parts[0]
                            move_type = int(parts[1])
                            type_str  = "movej" if move_type == MOVEJ else "movel"

                            nx  = float(parts[2])
                            ny  = float(parts[3])
                            nz  = float(parts[4])
                            nrx = float(parts[5])
                            nry = float(parts[6])
                            nrz = float(parts[7])
                            vel = float(parts[8])  # vitesse mm/s

                            # Cinématique inverse
                            with self.lock:
                                q_prev = self.joints[:]

                            new_joints = ik_solve(nx, ny, nz, nrx, nry, nrz, q_prev)

                            with self.lock:
                                self.x,  self.y,  self.z  = nx, ny, nz
                                self.rx, self.ry, self.rz = nrx, nry, nrz
                                if new_joints:
                                    self.joints = new_joints

                            # Publier les joints dans RViz
                            if new_joints:
                                self.ros_node.publish(new_joints)

                            print(f"🎯 [{type_str}] #{idx} → "
                                  f"({nx:.1f}, {ny:.1f}, {nz:.1f})mm "
                                  f"vel={vel:.0f}mm/s "
                                  f"{'✅' if new_joints else '⚠️ IK échouée'}")

                            # Envoi ACK : {ACK;index;n_restant}
                            import time as _t
                            _t.sleep(0.01)
                            ack = "{ACK;" + str(idx) + ";0}\n"
                            try:
                                self.sock.sendall(ack.encode('ascii'))
                                print(f"📤 ACK envoyé #{idx}")
                            except Exception as e:
                                print(f"❌ ACK erreur : {e}")

            except Exception as e:
                if self.running:
                    print(f"❌ Erreur réception : {e}")
                break

    def run(self):
        t_recv = threading.Thread(target=self.recv_loop, daemon=True)
        t_recv.start()

        print(f"🤖 Simulation démarrée à {1/FREQ:.0f} Hz — Ctrl+C pour arrêter\n")

        try:
            self.send_loop()
        except KeyboardInterrupt:
            print("\n🛑 Simulation arrêtée.")
        finally:
            self.running = False
            self.sock.close()


# ── MAIN ──────────────────────────────────────────────────────
def main():
    rclpy.init()
    ros_node = JointPublisher()

    # Spin ROS2 dans un thread séparé
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(ros_node),
        daemon=True
    )
    ros_thread.start()

    sim = RobotSimulator(ros_node)
    sim.run()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
