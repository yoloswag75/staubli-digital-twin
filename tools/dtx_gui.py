#!/usr/bin/env python3
# dtx_gui.py
# Interface graphique pour charger et jouer une trajectoire .dtx

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import os
import math
import re
import xml.etree.ElementTree as ET
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

MOVEJ = 0
MOVEL = 1


def parse_dtx(filepath):
    with open(filepath, 'rb') as f:
        raw = f.read()
    content = raw.decode('utf-8', errors='replace')
    content = content.replace('\r\n', '\n').replace('\r', '\n')
    points = []
    try:
        root = ET.fromstring(content)
        ns = {'v': 'http://www.staubli.com/robotics/VAL3/Data/2'}
        for data in root.findall('.//v:Data', ns):
            if data.get('type') == 'pointRx':
                for value in data.findall('v:Value', ns):
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
    except ET.ParseError:
        pattern = re.compile(
            r'x="([^"]+)"\s+y="([^"]+)"\s+z="([^"]+)"\s+'
            r'rx="([^"]+)"\s+ry="([^"]+)"\s+rz="([^"]+)"'
        )
        for i, m in enumerate(pattern.finditer(content)):
            try:
                points.append({
                    'key': f'point_{i}',
                    'x': float(m.group(1)), 'y': float(m.group(2)),
                    'z': float(m.group(3)), 'rx': float(m.group(4)),
                    'ry': float(m.group(5)), 'rz': float(m.group(6)),
                })
            except ValueError:
                continue
    return points


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('dtx_gui_player')
        self.pub = self.create_publisher(PoseStamped, '/staubli/cart_cmd', 10)
        self._ack_callback = None
        # Subscriber ACK : recoit les confirmations d execution du robot
        from sensor_msgs.msg import JointState as JS
        self.sub_ack = self.create_subscription(
            JS, '/staubli/cmd_ack', self._on_ack_msg, 10)

    def set_ack_callback(self, cb):
        self._ack_callback = cb

    def _on_ack_msg(self, msg):
        try:
            parts     = msg.header.frame_id.split(";")
            ack_index = int(parts[0])
            n_restant = int(parts[1]) if len(parts) > 1 else -1
            if self._ack_callback:
                self._ack_callback(ack_index, n_restant)
        except (ValueError, IndexError):
            pass

    def send_point(self, index, move_type, pt, vel=50.0):
        print(f"DEBUG: send_point #{index} type={move_type} vel={vel}")
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = f'world_{"movej" if move_type == MOVEJ else "movel"}_{vel:.1f}'
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
        self.pub.publish(msg)
        print(f"DEBUG: publié sur /staubli/cart_cmd frame_id={msg.header.frame_id}")

    def _euler_to_quat(self, r, p, y):
        cy = math.cos(y*0.5); sy = math.sin(y*0.5)
        cp = math.cos(p*0.5); sp = math.sin(p*0.5)
        cr = math.cos(r*0.5); sr = math.sin(r*0.5)
        return [
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy
        ]


class DtxPlayerGUI:
    def __init__(self, root):
        self.root          = root
        self.points        = []
        self.running       = False
        self.thread        = None
        self.ros_node      = None
        self.next_to_send  = 0    # prochain index a envoyer
        self.queue_size    = 5    # taille file VAL3
        self.refill_thresh = 2    # seuil de refill (n_restant <= 2)
        self.send_lock     = threading.Lock()

        root.title("Staubli DTX Player")
        root.resizable(False, False)
        root.configure(bg='#2b2b2b')

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame',       background='#2b2b2b')
        style.configure('TLabel',       background='#2b2b2b',
                                         foreground='#ffffff',
                                         font=('Helvetica', 10))
        style.configure('Title.TLabel', background='#2b2b2b',
                                         foreground='#ff9900',
                                         font=('Helvetica', 14, 'bold'))
        style.configure('Info.TLabel',  background='#2b2b2b',
                                         foreground='#aaaaaa',
                                         font=('Helvetica', 9))
        style.configure('Value.TLabel', background='#2b2b2b',
                                         foreground='#00cc44',
                                         font=('Helvetica', 10, 'bold'))
        style.configure('TScale',       background='#2b2b2b')
        style.configure('TProgressbar', troughcolor='#444444',
                                         background='#ff9900')

        self._build_ui()

    def _build_ui(self):
        ttk.Label(self.root, text="Staubli DTX Player",
                  style='Title.TLabel')\
            .grid(row=0, column=0, columnspan=3, pady=(14, 6))

        ttk.Separator(self.root, orient='horizontal')\
            .grid(row=1, column=0, columnspan=3, sticky='ew', padx=12)

        ttk.Label(self.root, text="Fichier .dtx :")\
            .grid(row=2, column=0, sticky='w', padx=12, pady=6)

        self.file_var = tk.StringVar(value="Aucun fichier selectionne")
        ttk.Label(self.root, textvariable=self.file_var,
                  style='Info.TLabel', width=32)\
            .grid(row=2, column=1, sticky='w', padx=4)

        ttk.Button(self.root, text="Parcourir", command=self._browse)\
            .grid(row=2, column=2, padx=(0, 12), pady=6)

        ttk.Separator(self.root, orient='horizontal')\
            .grid(row=3, column=0, columnspan=3, sticky='ew', padx=12)

        info_frame = ttk.Frame(self.root)
        info_frame.grid(row=4, column=0, columnspan=3,
                        sticky='ew', padx=12, pady=4)

        labels = [('Points :', 'nb_points'), ('Duree est. :', 'duration'),
                  ('Z min :',  'z_min'),     ('Z max :',     'z_max')]

        self.info_vars = {}
        for i, (lbl, key) in enumerate(labels):
            ttk.Label(info_frame, text=lbl, style='Info.TLabel')\
                .grid(row=i//2, column=(i%2)*2,
                      sticky='w', padx=(0, 4), pady=2)
            var = tk.StringVar(value="-")
            self.info_vars[key] = var
            ttk.Label(info_frame, textvariable=var, style='Value.TLabel')\
                .grid(row=i//2, column=(i%2)*2+1,
                      sticky='w', padx=(0, 20), pady=2)

        ttk.Separator(self.root, orient='horizontal')\
            .grid(row=5, column=0, columnspan=3, sticky='ew', padx=12)

        ttk.Label(self.root, text="Vitesse TCP :")\
            .grid(row=6, column=0, sticky='w', padx=12, pady=6)

        self.speed_var = tk.DoubleVar(value=50.0)
        ttk.Scale(self.root, from_=1.0, to=500.0,
                  variable=self.speed_var, orient='horizontal',
                  length=180, command=self._update_speed_label)\
            .grid(row=6, column=1, sticky='w', padx=4)

        self.speed_label = tk.StringVar(value="50 mm/s")
        ttk.Label(self.root, textvariable=self.speed_label,
                  style='Value.TLabel')\
            .grid(row=6, column=2, padx=(0, 12))

        ttk.Separator(self.root, orient='horizontal')\
            .grid(row=7, column=0, columnspan=3, sticky='ew', padx=12)

        btn_frame = ttk.Frame(self.root)
        btn_frame.grid(row=8, column=0, columnspan=3, pady=10)

        self.play_btn = ttk.Button(btn_frame, text="PLAY",
                                   command=self._play, state='disabled')
        self.play_btn.grid(row=0, column=0, padx=8)

        self.stop_btn = ttk.Button(btn_frame, text="STOP",
                                   command=self._stop, state='disabled')
        self.stop_btn.grid(row=0, column=1, padx=8)

        ttk.Separator(self.root, orient='horizontal')\
            .grid(row=9, column=0, columnspan=3, sticky='ew', padx=12)

        self.progress_var = tk.DoubleVar(value=0)
        ttk.Progressbar(self.root, variable=self.progress_var,
                        maximum=100, length=320)\
            .grid(row=10, column=0, columnspan=3, padx=12, pady=(6, 2))

        self.progress_label = tk.StringVar(value="0 / 0 points")
        ttk.Label(self.root, textvariable=self.progress_label,
                  style='Info.TLabel')\
            .grid(row=11, column=0, columnspan=3, pady=(0, 6))

        self.status_var = tk.StringVar(value="En attente d'un fichier...")
        ttk.Label(self.root, textvariable=self.status_var,
                  style='Info.TLabel')\
            .grid(row=12, column=0, columnspan=3, pady=(0, 12))

    def _browse(self):
        path = filedialog.askopenfilename(
            title="Selectionner un fichier .dtx",
            filetypes=[("Fichiers DTX VAL3", "*.dtx"),
                       ("Tous les fichiers", "*.*")]
        )
        if not path:
            return
        self.status_var.set("Chargement...")
        self.root.update()
        self.points = parse_dtx(path)
        if not self.points:
            messagebox.showerror("Erreur", "Aucun point trouve dans ce fichier")
            self.status_var.set("Fichier invalide")
            return
        nb    = len(self.points)
        speed = self.speed_var.get()
        dur   = nb * speed
        z_min = min(p['z'] for p in self.points)
        z_max = max(p['z'] for p in self.points)
        self.file_var.set(os.path.basename(path))
        self.info_vars['nb_points'].set(str(nb))
        self.info_vars['duration'].set(
            f"{int(dur//60)}min {int(dur%60):02d}s")
        self.info_vars['z_min'].set(f"{z_min:.1f} mm")
        self.info_vars['z_max'].set(f"{z_max:.1f} mm")
        self.progress_var.set(0)
        self.progress_label.set(f"0 / {nb} points")
        self._update_duration()
        self.play_btn.config(state='normal')
        self.status_var.set(f"{nb} points charges - pret a jouer")

    def _update_speed_label(self, val=None):
        v = self.speed_var.get()
        self.speed_label.set(f"{v:.0f} mm/s")
        if self.points:
            # Estimer la duree depuis la distance totale et la vitesse
            self._update_duration()

    def _update_duration(self):
        if not self.points or len(self.points) < 2:
            return
        import math
        total_dist = sum(
            math.sqrt(
                (self.points[i+1]['x'] - self.points[i]['x'])**2 +
                (self.points[i+1]['y'] - self.points[i]['y'])**2 +
                (self.points[i+1]['z'] - self.points[i]['z'])**2
            )
            for i in range(len(self.points) - 1)
        )
        vel = max(1.0, self.speed_var.get())
        dur = total_dist / vel  # secondes
        self.info_vars['duration'].set(
            f"{int(dur//60)}min {int(dur%60):02d}s")
        self.info_vars['nb_points'].set(
            f"{len(self.points)} ({total_dist:.0f}mm)")

    def _init_ros(self):
        if not ROS_AVAILABLE:
            self.status_var.set("ROS2 non disponible - sourcer setup.bash")
            return False
        if self.ros_node is not None:
            return True
        try:
            rclpy.init()
            self.ros_node = TrajectoryPublisher()
            self.ros_node.set_ack_callback(self._on_ack)
            from rclpy.executors import MultiThreadedExecutor
            executor = MultiThreadedExecutor()
            executor.add_node(self.ros_node)
            threading.Thread(
                target=executor.spin,
                daemon=True
            ).start()
            return True
        except Exception as e:
            self.status_var.set(f"ROS2 erreur : {e}")
            return False

    def _play(self):
        if not self.points or self.running:
            return
        print("DEBUG: _play appelé")
        if not self._init_ros():
            print("DEBUG: _init_ros a échoué")
            return
        print("DEBUG: ROS2 initialisé, lancement trajectory")
        self.running = True
        self.play_btn.config(state='disabled')
        self.stop_btn.config(state='normal')
        self.status_var.set("Trajectoire en cours...")
        self.thread = threading.Thread(
            target=self._run_trajectory, daemon=True)
        self.thread.start()

    def _stop(self):
        self.running = False
        self.status_var.set("Arrete")
        self.play_btn.config(state='normal')
        self.stop_btn.config(state='disabled')

    def _on_ack(self, ack_index: int, n_restant: int = -1):
        """Callback appele quand le robot confirme l execution d un point."""
        nb = len(self.points)
        self.root.after(0, self.status_var.set,
                        f"ACK #{ack_index} | file: {n_restant} | "
                        f"envoi: {self.next_to_send}/{nb}")

        # Refill proactif si la file est presque vide
        if self.running and n_restant <= self.refill_thresh:
            threading.Thread(
                target=self._send_batch, daemon=True).start()

    def _send_batch(self):
        """Envoie un batch de queue_size points depuis next_to_send."""
        nb  = len(self.points)
        vel = max(1.0, self.speed_var.get())

        sent = 0
        print(f"DEBUG: _send_batch nb={nb} next={self.next_to_send} running={self.running}")
        while sent < self.queue_size and self.next_to_send < nb and self.running:
            i  = self.next_to_send
            pt = self.points[i]
            move_type = MOVEJ if i == 0 else MOVEL

            if self.ros_node:
                self.ros_node.send_point(i + 1, move_type, pt, vel)

            pct = (i + 1) / nb * 100
            self.root.after(0, self._update_progress, i + 1, nb, pct)

            self.next_to_send += 1
            sent += 1
            time.sleep(0.01)

        # Fin de trajectoire
        if self.next_to_send >= nb:
            self.root.after(0, self._on_done)

    def _run_trajectory(self):
        print("DEBUG: _run_trajectory appelé")
        self.next_to_send = 0
        self._send_batch()
        print("DEBUG: _send_batch terminé, next_to_send =", self.next_to_send)

    def _update_progress(self, current, total, pct):
        self.progress_var.set(pct)
        self.progress_label.set(f"{current} / {total} points")

    def _on_done(self):
        self.running = False
        self.play_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
        if self.progress_var.get() >= 99:
            self.status_var.set("Trajectoire terminee !")
        else:
            self.status_var.set("Trajectoire interrompue")

    def on_close(self):
        self.running = False
        if self.ros_node:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        self.root.destroy()


def main():
    root = tk.Tk()
    app  = DtxPlayerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
