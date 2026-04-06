#!/usr/bin/env python3
"""
robot_simulator.py
──────────────────
Simule le robot Stäubli RX160L côté TCP.
Joue le rôle du code VAL3 (tPosition + tCommande) sans avoir SRS.

Comportement :
  - Se connecte au bridge ROS2 (comme le ferait le vrai robot)
  - Envoie des trames de position toutes les 4ms
  - Reçoit et affiche les commandes joints envoyées par le PC
  - Anime les joints avec un mouvement sinusoïdal pour tester RViz
"""

import socket
import threading
import time
import math

# ── CONFIGURATION ─────────────────────────────────────────────
HOST  = "127.0.0.1"   # Le bridge tourne sur la même machine
PORT  = 2005
FREQ  = 0.004         # 4ms = 250Hz comme tPosition


class RobotSimulator:
    def __init__(self):
        self.frame_index = 0
        self.running     = True

        # Position articulaire simulée (degrés)
        self.joints = [0.0, -30.0, 90.0, 0.0, 45.0, 0.0]

        # Position cartésienne simulée (mm)
        self.x, self.y, self.z    = 800.0, 0.0, 600.0
        self.rx, self.ry, self.rz = 0.0, 90.0, 0.0

        # Connexion TCP
        print(f"🔌 Connexion au bridge sur {HOST}:{PORT}...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        print("✅ Connecté au bridge ROS2 !")

    def animate_joints(self):
        """Anime les joints avec des sinusoïdes pour simuler un mouvement."""
        t = time.time()
        self.joints = [
            20.0  * math.sin(0.5 * t),           # j1
            -30.0 + 15.0 * math.sin(0.3 * t),    # j2
            90.0  + 10.0 * math.cos(0.4 * t),    # j3
            0.0   + 5.0  * math.sin(0.7 * t),    # j4
            45.0  + 10.0 * math.cos(0.5 * t),    # j5
            0.0   + 8.0  * math.sin(0.6 * t),    # j6
        ]
        # Position cartésienne qui suit
        self.x = 800.0 + 50.0 * math.sin(0.3 * t)
        self.y = 100.0 * math.sin(0.2 * t)
        self.z = 600.0 + 30.0 * math.cos(0.4 * t)

    def send_loop(self):
        """Envoie la position toutes les 4ms — simule tPosition.pgx."""
        while self.running:
            start = time.time()

            self.animate_joints()
            self.frame_index += 1

            # Format : {index;j1;j2;j3;j4;j5;j6;x;y;z;rx;ry;rz}\n
            trame = "{{{};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f};{:.3f}}}\n".format(
                self.frame_index,
                self.joints[0], self.joints[1], self.joints[2],
                self.joints[3], self.joints[4], self.joints[5],
                self.x, self.y, self.z,
                self.rx, self.ry, self.rz
            )

            try:
                self.sock.sendall(trame.encode('ascii'))
            except Exception as e:
                print(f"❌ Erreur envoi : {e}")
                self.running = False
                break

            # Log toutes les 250 trames (~1 seconde)
            if self.frame_index % 250 == 0:
                print(f"📡 Trame #{self.frame_index} | j1={self.joints[0]:.1f}° | TCP=({self.x:.0f}, {self.y:.0f}, {self.z:.0f})mm")

            # Respect de la période 4ms
            elapsed = time.time() - start
            sleep_time = FREQ - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def recv_loop(self):
        """Reçoit les commandes joints depuis le PC — simule tCommande.pgx."""
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

                    # Format attendu : {index;j1;j2;j3;j4;j5;j6}
                    if line.startswith("{") and line.endswith("}"):
                        content = line[1:-1]
                        parts   = content.split(';')
                        if len(parts) == 7:
                            idx  = parts[0]
                            joints_cmd = [float(p) for p in parts[1:]]
                            print(f"🎯 Commande reçue #{idx} | j1={joints_cmd[0]:.1f}° j2={joints_cmd[1]:.1f}° j3={joints_cmd[2]:.1f}°...")
                            # Sur un vrai robot → movej(jCmd, flange, mNomSpeed)
                            # Ici on met simplement à jour la position simulée
                            self.joints = joints_cmd

            except Exception as e:
                if self.running:
                    print(f"❌ Erreur réception : {e}")
                break

    def run(self):
        # Thread réception commandes
        t_recv = threading.Thread(target=self.recv_loop, daemon=True)
        t_recv.start()

        print(f"🤖 Simulation démarrée à {1/FREQ:.0f} Hz — Ctrl+C pour arrêter")
        print(f"📊 Log toutes les secondes\n")

        try:
            self.send_loop()
        except KeyboardInterrupt:
            print("\n🛑 Simulation arrêtée.")
        finally:
            self.running = False
            self.sock.close()


if __name__ == '__main__':
    sim = RobotSimulator()
    sim.run()
