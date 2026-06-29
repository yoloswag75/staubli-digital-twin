#!/usr/bin/env python3
# dt_gui.py - Interface principale du jumeau numerique Staubli RX160L

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import threading
import subprocess
import time
import os
import socket
import yaml
import pathlib

PROJECT_DIR = pathlib.Path(__file__).parent.parent
CONFIG_FILE = PROJECT_DIR / "config" / "config.yaml"

BG     = "#2b2b2b"
FG     = "#ffffff"
GREEN  = "#00cc44"
ORANGE = "#ff9900"
RED    = "#cc2200"
GREY   = "#aaaaaa"
BORDER = "#444444"


def load_config():
    if CONFIG_FILE.exists():
        with open(CONFIG_FILE) as f:
            return yaml.safe_load(f)
    return {"network": {"pc_ip": "0.0.0.0", "port": 2005},
            "robot":   {"robot_ip": "192.168.99.25"}}


def save_config(cfg):
    CONFIG_FILE.parent.mkdir(exist_ok=True)
    with open(CONFIG_FILE, "w") as f:
        yaml.dump(cfg, f)


class StatusLight(tk.Canvas):
    def __init__(self, parent, **kwargs):
        super().__init__(parent, width=18, height=18,
                         bg=BG, highlightthickness=0, **kwargs)
        self._oval = self.create_oval(2, 2, 16, 16, fill=RED, outline="")

    def set(self, color):
        self.itemconfig(self._oval, fill=color)


class DtGui:
    def __init__(self, root):
        self.root     = root
        self.config   = load_config()
        self.procs    = {}
        self.running  = True
        self._conn_status = "disconnected"
        self._traj_start  = None
        self._dt_start    = time.time()
        self._ros_node    = None
        self._ip_pub      = None
        self._data_dir    = str(pathlib.Path.home() / "Desktop")

        root.title("Staubli Digital Twin")
        root.configure(bg=BG)
        root.resizable(False, False)

        self._build_styles()
        self._build_ui()
        self._init_ros()
        self._start_monitor()

    def _init_ros(self):
        """Initialise ROS2 et fournit le service /staubli/get_robot_ip."""
        def _ros_thread():
            try:
                import rclpy
                from staubli_msgs.srv import GetRobotIp

                rclpy.init()
                node = rclpy.create_node("dt_gui")
                self._ros_node = node

                def handle_get_ip(request, response):
                    response.robot_ip = self.ip_var.get()
                    node.get_logger().info(
                        f"Service GetRobotIp -> {response.robot_ip}")
                    return response

                node.create_service(GetRobotIp, "/staubli/get_robot_ip",
                                    handle_get_ip)

                from staubli_msgs.srv import GetDataDir

                def handle_get_data_dir(request, response):
                    response.data_dir = self._data_dir
                    node.get_logger().info(
                        f"Service GetDataDir -> {response.data_dir}")
                    return response

                node.create_service(GetDataDir, "/staubli/get_data_dir",
                                    handle_get_data_dir)

                self.root.after(0, self.status_bar.set,
                                "ROS2 OK - service /staubli/get_robot_ip actif")

                while self.running and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)

                node.destroy_node()
                rclpy.shutdown()
            except Exception as e:
                self.root.after(0, self.status_bar.set,
                                f"ROS2 non disponible : {e}")

        threading.Thread(target=_ros_thread, daemon=True).start()

    def _build_styles(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure(".",              background=BG, foreground=FG,
                    font=("Helvetica", 10))
        s.configure("TFrame",         background=BG)
        s.configure("TButton",        background=BORDER, foreground=FG,
                    font=("Helvetica", 10, "bold"), padding=6)
        s.map("TButton",              background=[("active", "#555555")])
        s.configure("Green.TButton",  background="#004d1a", foreground=GREEN)
        s.map("Green.TButton",        background=[("active", "#006622")])
        s.configure("Red.TButton",    background="#4d0000", foreground=RED)
        s.map("Red.TButton",          background=[("active", "#660000")])
        s.configure("TLabel",         background=BG, foreground=FG)
        s.configure("Title.TLabel",   background=BG, foreground=ORANGE,
                    font=("Helvetica", 15, "bold"))
        s.configure("Value.TLabel",   background=BG, foreground=GREEN,
                    font=("Helvetica", 11, "bold"))
        s.configure("Grey.TLabel",    background=BG, foreground=GREY,
                    font=("Helvetica", 9))
        s.configure("Section.TLabelframe",
                    background=BG, foreground=ORANGE,
                    font=("Helvetica", 10, "bold"))
        s.configure("Section.TLabelframe.Label",
                    background=BG, foreground=ORANGE)

    def _build_ui(self):
        pad = {"padx": 10, "pady": 6}

        ttk.Label(self.root, text="Staubli Digital Twin",
                  style="Title.TLabel").grid(
            row=0, column=0, columnspan=2, pady=(14, 4))
        ttk.Separator(self.root, orient="horizontal").grid(
            row=1, column=0, columnspan=2, sticky="ew", padx=10)

        # Connexion
        conn = ttk.LabelFrame(self.root, text="Connexion",
                              style="Section.TLabelframe")
        conn.grid(row=2, column=0, columnspan=2, sticky="ew", padx=10, pady=6)

        f_status = ttk.Frame(conn)
        f_status.grid(row=0, column=0, sticky="w", **pad)
        ttk.Label(f_status, text="Statut :").pack(side="left")
        self.light = StatusLight(f_status)
        self.light.pack(side="left", padx=6)
        self.status_var = tk.StringVar(value="Non connecte")
        ttk.Label(f_status, textvariable=self.status_var,
                  style="Value.TLabel").pack(side="left")

        f_ip = ttk.Frame(conn)
        f_ip.grid(row=1, column=0, sticky="w", **pad)
        ttk.Label(f_ip, text="IP robot :").pack(side="left")
        self.ip_var = tk.StringVar(
            value=self.config.get("robot", {}).get("robot_ip", "192.168.99.25"))
        ttk.Label(f_ip, textvariable=self.ip_var,
                  style="Value.TLabel").pack(side="left", padx=8)
        ttk.Button(f_ip, text="Changer IP",
                   command=self._change_ip).pack(side="left")

        ttk.Label(conn, text="L'IP est publiee sur /staubli/robot_ip",
                  style="Grey.TLabel").grid(row=2, column=0, sticky="w",
                                            padx=10, pady=(0, 4))

        f_bridge = ttk.Frame(conn)
        f_bridge.grid(row=3, column=0, sticky="w", **pad)
        ttk.Button(f_bridge, text="Lancer Bridge",
                   style="Green.TButton",
                   command=self._start_bridge).pack(side="left", padx=(0, 6))
        ttk.Button(f_bridge, text="Arreter Bridge",
                   style="Red.TButton",
                   command=self._stop_bridge).pack(side="left")

        ttk.Separator(self.root, orient="horizontal").grid(
            row=3, column=0, columnspan=2, sticky="ew", padx=10)

        # Temps
        temps = ttk.LabelFrame(self.root, text="Temps",
                               style="Section.TLabelframe")
        temps.grid(row=4, column=0, columnspan=2, sticky="ew", padx=10, pady=6)

        f_t = ttk.Frame(temps)
        f_t.grid(row=0, column=0, sticky="w", **pad)
        ttk.Label(f_t, text="DT actif :").grid(row=0, column=0, sticky="w")
        self.dt_time_var = tk.StringVar(value="00:00:00")
        ttk.Label(f_t, textvariable=self.dt_time_var,
                  style="Value.TLabel").grid(row=0, column=1, padx=8)
        ttk.Label(f_t, text="Trajectoire :").grid(row=1, column=0, sticky="w")
        self.traj_time_var = tk.StringVar(value="--:--:--")
        ttk.Label(f_t, textvariable=self.traj_time_var,
                  style="Value.TLabel").grid(row=1, column=1, padx=8)

        ttk.Separator(self.root, orient="horizontal").grid(
            row=5, column=0, columnspan=2, sticky="ew", padx=10)

        # Actions
        actions = ttk.LabelFrame(self.root, text="Actions",
                                 style="Section.TLabelframe")
        actions.grid(row=6, column=0, columnspan=2, sticky="ew", padx=10, pady=6)

        btn_cfg = [
            ("Lancer RViz",        "Green.TButton", self._start_rviz),
            ("Fermer RViz",        "Red.TButton",   self._stop_rviz),
            ("Charger programme",  "TButton",        self._start_dtx_gui),
            ("Ouvrir repertoire",  "TButton",        self._open_data_dir),
        ]
        for i, (label, style, cmd) in enumerate(btn_cfg):
            ttk.Button(actions, text=label, style=style,
                       command=cmd, width=20).grid(
                row=i//2, column=i%2, padx=6, pady=4, sticky="ew")

        # Repertoire de sauvegarde
        f_dir = ttk.Frame(actions)
        f_dir.grid(row=2, column=0, columnspan=2, sticky="w", padx=6, pady=4)
        ttk.Label(f_dir, text="Repertoire :").pack(side="left")
        self.data_dir_var = tk.StringVar(value=str(pathlib.Path.home() / "Desktop"))
        ttk.Label(f_dir, textvariable=self.data_dir_var,
                  style="Grey.TLabel", width=25).pack(side="left", padx=4)
        ttk.Button(f_dir, text="Changer",
                   command=self._change_data_dir).pack(side="left")

        ttk.Separator(self.root, orient="horizontal").grid(
            row=7, column=0, columnspan=2, sticky="ew", padx=10)

        self.status_bar = tk.StringVar(value="ROS2 en initialisation...")
        ttk.Label(self.root, textvariable=self.status_bar,
                  style="Grey.TLabel").grid(
            row=8, column=0, columnspan=2, pady=(4, 10))

    def _change_ip(self):
        current = self.ip_var.get()
        new_ip = simpledialog.askstring(
            "Changer IP robot",
            f"IP actuelle : {current}\nNouvelle IP :",
            initialvalue=current, parent=self.root
        )
        if not new_ip or new_ip == current:
            return
        if messagebox.askyesno("Confirmer",
                               f"Changer l'IP\n{current} -> {new_ip} ?"):
            self.ip_var.set(new_ip)
            self.config.setdefault("robot", {})["robot_ip"] = new_ip
            save_config(self.config)
            self.status_bar.set(f"IP mise a jour : {new_ip}")

    def _start_bridge(self):
        if "bridge" in self.procs and self.procs["bridge"].poll() is None:
            self.status_bar.set("Bridge deja en cours")
            return
        env = os.environ.copy()
        cmd = ["ros2", "run", "staubli_bridge", "bridge"]
        self.procs["bridge"] = subprocess.Popen(
            cmd, env=env, preexec_fn=os.setsid)
        self._set_status("connecting")
        self.status_bar.set("Bridge lance - IP publiee sur /staubli/robot_ip")

    def _stop_bridge(self):
        if "bridge" in self.procs:
            import signal
            try:
                os.killpg(os.getpgid(self.procs["bridge"].pid), signal.SIGTERM)
            except Exception:
                self.procs["bridge"].terminate()
            del self.procs["bridge"]
        self._set_status("disconnected")
        self.status_bar.set("Bridge arrete")

    def _set_status(self, state):
        self._conn_status = state
        colors = {"connected": GREEN, "connecting": ORANGE, "disconnected": RED}
        labels = {"connected": "Connecte", "connecting": "En tentative...",
                  "disconnected": "Non connecte"}
        self.root.after(0, self.light.set, colors.get(state, RED))
        self.root.after(0, self.status_var.set, labels.get(state, "Non connecte"))

    def _check_bridge_connection(self):
        port = self.config.get("network", {}).get("port", 2005)
        try:
            s = socket.socket()
            s.settimeout(0.3)
            result = s.connect_ex(("127.0.0.1", port))
            s.close()
            if result == 0:
                self._set_status("connected")
            elif "bridge" in self.procs and self.procs["bridge"].poll() is None:
                self._set_status("connecting")
            else:
                self._set_status("disconnected")
        except Exception:
            self._set_status("disconnected")

    def _start_rviz(self):
        if "rviz" in self.procs and self.procs["rviz"].poll() is None:
            self.status_bar.set("RViz deja ouvert")
            return
        cmd = ["ros2", "launch", "staubli_description", "display.launch.py"]
        self.procs["rviz"] = subprocess.Popen(
            cmd, env=os.environ.copy(), preexec_fn=os.setsid)
        self.status_bar.set("RViz lance")

    def _stop_rviz(self):
        if "rviz" in self.procs:
            import signal
            try:
                os.killpg(os.getpgid(self.procs["rviz"].pid), signal.SIGTERM)
            except Exception:
                self.procs["rviz"].terminate()
            del self.procs["rviz"]
            self.status_bar.set("RViz ferme")

    def _start_dtx_gui(self):
        dtx_path = PROJECT_DIR / "tools" / "dtx_gui.py"
        if not dtx_path.exists():
            messagebox.showerror("Erreur", f"Introuvable : {dtx_path}")
            return
        if "dtx" in self.procs and self.procs["dtx"].poll() is None:
            self.status_bar.set("DTX Player deja ouvert")
            return
        self.procs["dtx"] = subprocess.Popen(
            ["python3", str(dtx_path)], preexec_fn=os.setsid)
        self._traj_start = time.time()
        self.status_bar.set("DTX Player lance")

    def _change_data_dir(self):
        from tkinter import filedialog
        new_dir = filedialog.askdirectory(
            title="Choisir le repertoire de sauvegarde",
            initialdir=self._data_dir,
            parent=self.root
        )
        if new_dir:
            self._data_dir = new_dir
            self.data_dir_var.set(new_dir)
            self.status_bar.set(f"Repertoire : {new_dir}")

    def _open_data_dir(self):
        data_path = pathlib.Path(self._data_dir)
        subprocess.Popen(["xdg-open", str(data_path)])

    def _start_monitor(self):
        def _loop():
            while self.running:
                try:
                    dt_elapsed = int(time.time() - self._dt_start)
                    h, r = divmod(dt_elapsed, 3600)
                    m, s = divmod(r, 60)
                    self.root.after(0, self.dt_time_var.set,
                                   f"{h:02d}:{m:02d}:{s:02d}")
                    if self._traj_start:
                        te = int(time.time() - self._traj_start)
                        h2, r2 = divmod(te, 3600)
                        m2, s2 = divmod(r2, 60)
                        self.root.after(0, self.traj_time_var.set,
                                       f"{h2:02d}:{m2:02d}:{s2:02d}")
                    self._check_bridge_connection()
                except Exception:
                    pass
                time.sleep(1)
        threading.Thread(target=_loop, daemon=True).start()

    def on_close(self):
        self.running = False
        import signal
        for name, p in self.procs.items():
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                try:
                    p.terminate()
                except Exception:
                    pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app  = DtGui(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
