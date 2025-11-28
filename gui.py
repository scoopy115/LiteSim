# gui.py
import tkinter as tk
from tkinter import ttk, filedialog, scrolledtext, messagebox
import threading
import queue
import socket
import os
import webbrowser
import runpy
import sys
import traceback
import types
import config
import sv_ttk
import platform
import json
import zipfile
from pathlib import Path
from urllib.request import urlretrieve, urlopen
from visualizer import RobotVisualizer
from robot_api import SimXArmAPI
from config import GLOBAL_API_INSTANCE, JOINT_COUNT, HISTORY_FILE, STL_HISTORY_FILE, ROBOT_SCAN_PORT, GITHUB_URL, PORTFOLIO_URL
from utils import QueueRedirector

# --- GUI CONTEXT ---
class AppContext:
    def __init__(self):
        self.log_queue = queue.Queue()
        self.joint_queue = queue.Queue(maxsize=2)
        self.stop_flag = False
        self.paused = False

# --- TKINTER GUI ---
class ControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        #Load icon
        self.app_icon = None
        self.header_icon = None

        try:
            if os.path.exists(config.ICON_PATH):
                self.app_icon = tk.PhotoImage(file=config.ICON_PATH)
                self.iconphoto(True, self.app_icon)
                self.wm_iconname("LiteSim")
                self.header_icon = self.app_icon.subsample(12, 12) 
        except Exception as e:
            print(f"[SYSTEM] Could not load app icon: {e}")
        
        self.title(f"{config.APP_NAME} {config.APP_VERSION} | UFACTORY Lite 6 Simulator | Controls")
        screen_w = self.winfo_screenwidth()
        screen_h = self.winfo_screenheight()
        ctrl_w = 800 
        request_h = screen_h 
        self.geometry(f"{ctrl_w}x{request_h}+0+0")
        
        self.minsize(800, 800)

        self.ctx = AppContext()
        self.data_lock = threading.Lock()

        self.viz = RobotVisualizer()
        self.ik_chain = self.viz.setup_scene()
        
        if not self.ik_chain:
            messagebox.showerror("Error", "Could not load URDF model.")
            self.destroy()
            return
            
        # Init API
        self.api = SimXArmAPI(self.ctx, self.ik_chain)
        
        self.script_history = []
        self.stl_history = []
        self.current_script_path = None

        self.loop_var = tk.BooleanVar(value=False)
        self.speed_var = tk.DoubleVar(value=1.0)
        self.rendering_paused = False
        self.scale_mm_var = tk.BooleanVar(value=True)
        self.trace_var = tk.BooleanVar(value=False)
        self.pending_update_data = None
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        
        self.rendering_paused = False

        self._start_update_check()
        self._build_ui()
        self._load_history()
        self._load_stl_history()
        
        self._update_3d_loop()
        self._process_queues()

        self._apply_modern_theme()

    def _load_history(self):
        self.script_history = []

        if os.path.exists(config.HISTORY_FILE):
            try:
                with open(config.HISTORY_FILE, "r") as f:
                    lines = f.readlines()
                    for line in lines:
                        path = line.strip()
                        if path and os.path.exists(path):
                            if path not in self.script_history:
                                self.script_history.append(path)
            except: pass

        if os.path.exists(config.EXAMPLES_DIR):
            try:
                for filename in os.listdir(config.EXAMPLES_DIR):
                    if filename.endswith(".py"):
                        full_path = os.path.join(config.EXAMPLES_DIR, filename)
                        if full_path not in self.script_history:
                            self.script_history.append(full_path)
            except Exception as e:
                print(f"[GUI] Error loading examples: {e}")

        display_names = []
        for p in self.script_history:
            name = os.path.basename(p)
            if config.EXAMPLES_DIR in p:
                name = f"[Example] {name}"
            display_names.append(name)
            
        self.combo_history['values'] = display_names

        if display_names:
            self.combo_history.current(0)
            self.current_script_path = self.script_history[0]
            self.btn_run.config(state=tk.NORMAL)

    def _apply_modern_theme(self):
        sv_ttk.set_theme("dark")
        style = ttk.Style()

        base_font = ("Segoe UI", 10) if sys.platform == "win32" else ("Helvetica", 12)
        style.configure(".", font=base_font)
        
        style.configure("Title.TLabel", font=("Segoe UI", 20, "bold") if sys.platform == "win32" else ("Helvetica", 20, "bold"))
        style.configure("Sub.TLabel", foreground="#bbbbbb") 
        style.configure("Link.TLabel", foreground="#4da6ff", font=base_font + ("underline",))
        
        style.configure("Accent.TButton", font=("Helvetica", 14), padding=(10, 5))
        style.configure("Small.Accent.TButton", font=("Helvetica", 9, "bold"), padding=(5, 0))

        self.colors = {
            "log_bg": "#1c1c1c", 
            "log_fg": "#ffffff",
            "log_insert": "#ffffff"
        }

    def _update_3d_loop(self):
        if self.rendering_paused:
            self.after(500, self._update_3d_loop)
            return
        
        if not self.viz or not self.viz.plotter:
            return

        # Idk why but this does not work
        if self.viz.plotter.ren_win is None:
            print("[SYSTEM] 3D Viewer closed by user. Shutting down...")
            self._on_close()
            return

        try:
            self.viz.render_frame()
            self.after(10, self._update_3d_loop)
        except Exception: pass

    def _process_queues(self):
        while not self.ctx.log_queue.empty():
            try:
                msg = self.ctx.log_queue.get_nowait()
                self.txt.insert(tk.END, str(msg) + "\n")
                self.txt.see(tk.END)
            except queue.Empty: break
            
        latest_joints = None
        while not self.ctx.joint_queue.empty():
            try: latest_joints = self.ctx.joint_queue.get_nowait()
            except queue.Empty: break
        
        if latest_joints:
            with self.data_lock:
                self.viz.update_joints(latest_joints)
                
            for i, val in enumerate(latest_joints):
                self.vars[i].set(val)
                self.joint_labels[i].config(text=f"{val:.1f}")
                
        self.after(30, self._process_queues)

    # Custom gripper callback
    def _load_custom_gripper(self):
        self.rendering_paused = True 
        self.update_idletasks() 
        
        try:
            path = filedialog.askopenfilename(
                title="Choose an STL file",
                filetypes=[("STL files", "*.stl"), ("All files", "*.*")]
            )
            if path:
                do_scale = self.scale_mm_var.get()
                success = self.viz.set_custom_gripper(path, scale_to_meters=do_scale)
                if success:
                    scaled_txt = " (scaled)" if do_scale else ""
                    self.ctx.log_queue.put(f"[GUI] Loaded: {os.path.basename(path)}")
                    self._add_to_stl_history(path)
                    self._force_trace_mode("Effector Tip")
                else:
                    messagebox.showerror("Error", "Could not load the STL file.")
        finally:
            self.rendering_paused = False

    def _load_specific_gripper(self, filename):
        self.rendering_paused = True
        self.update_idletasks()
        
        try:
            full_path = self.viz.get_mesh_path(filename)
            if not full_path:
                messagebox.showerror("Not found", f"Can not find file:\n{filename}")
                return

            success = self.viz.set_custom_gripper(full_path, scale_to_meters=False)
            if success:
                self.ctx.log_queue.put(f"[GUI] Loaded preset: {filename}")
                self._force_trace_mode("Effector Tip")
            else:
                messagebox.showerror("Error", f"Cannot load {filename}.")
        finally:
            self.rendering_paused = False

    def _load_std_gripper(self):
        self._load_specific_gripper("gripper_lite.stl")

    def _load_vac_gripper(self):
        self._load_specific_gripper("vacuum_gripper_lite.stl")

    def _remove_gripper(self):
        self.rendering_paused = True
        self.update_idletasks()
        try:
            success = self.viz.remove_gripper()
            if success:
                self.ctx.log_queue.put("[GUI] End-effector removed from model")
                self._force_trace_mode("Wrist")
        finally:
            self.rendering_paused = False

    def _force_trace_mode(self, mode_text):
        self.trace_mode_var.set(mode_text)
        self.viz.trace_source = mode_text.lower()
        self.viz.clear_trace()

    def _build_ui(self):
        # Main container
        main = ttk.Frame(self, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # HEADER
        header_frame = ttk.Frame(main)
        header_frame.pack(fill=tk.X, pady=(0, 15))

        # 1. LEFT CONTAINER
        header_left = ttk.Frame(header_frame)
        header_left.pack(side=tk.LEFT)

        if self.header_icon:
            icon_lbl = ttk.Label(header_left, image=self.header_icon)
            icon_lbl.pack(side=tk.LEFT, padx=(0, 10), anchor="center")

        text_container = ttk.Frame(header_left)
        text_container.pack(side=tk.LEFT, anchor="center")

        title_row = ttk.Frame(text_container)
        title_row.pack(anchor="w", fill=tk.X)

        # Title
        lbl_title = ttk.Label(
            title_row, 
            text=f"{config.APP_NAME} {config.APP_VERSION}", 
            font=("Helvetica", 20, "bold")
        )
        lbl_title.pack(side=tk.LEFT)

        # Update button
        self.btn_update_available = ttk.Button(
            title_row, 
            text="Update Available",
            style="Small.Accent.TButton", 
            command=self._on_update_click
        )

        # SUBTITLE
        lbl_credits = ttk.Label(
            text_container, 
            text="Made with ❤️ by Gemini 3 Pro & Dylan Kiesebrink", 
            font=("Helvetica", 10),
            foreground="#888888"
        )
        lbl_credits.pack(anchor="w")

        # Right
        header_right = ttk.Frame(header_frame)
        header_right.pack(side=tk.RIGHT)

        # Github Link
        lbl_github = ttk.Label(
            header_right, 
            text="View on GitHub ↗", 
            font=("Helvetica", 10, "underline"), 
            foreground="#ff5015",
            cursor="hand2"
        )
        lbl_github.pack(anchor="e", pady=(0, 2)) # 'e' = East (Rechts uitlijnen)
        lbl_github.bind("<Button-1>", lambda e: webbrowser.open(GITHUB_URL))

        # Portfolio Link
        lbl_portfolio = ttk.Label(
            header_right, 
            text="Visit Portfolio ↗", 
            font=("Helvetica", 10, "underline"), 
            foreground="#ff5015",
            cursor="hand2"
        )
        lbl_portfolio.pack(anchor="e")
        lbl_portfolio.bind("<Button-1>", lambda e: webbrowser.open(PORTFOLIO_URL))

        # Grid container
        top_container = ttk.Frame(main)
        top_container.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        
        top_container.columnconfigure(0, weight=1, uniform="group1") # Left (1/5)
        top_container.columnconfigure(1, weight=3, uniform="group1") # Middle (3/5)
        top_container.columnconfigure(2, weight=1, uniform="group1") # Right (1/5)
        
        top_container.rowconfigure(0, weight=1)

        # LEFT
        left_col = ttk.Frame(top_container)
        left_col.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        # 1. Connection Frame
        conn_frame = ttk.LabelFrame(left_col, text="Physical Connection", padding=10)
        conn_frame.pack(fill=tk.X, pady=5)
        
        conn_header = ttk.Frame(conn_frame)
        conn_header.pack(fill=tk.X, pady=(0, 2))
        
        ttk.Label(conn_header, text="IP Address:").pack(side=tk.LEFT)
        bg_color = ttk.Style().lookup("TFrame", "background")
        self.status_canvas = tk.Canvas(conn_header, width=14, height=14, bg=bg_color, highlightthickness=0)
        self.status_canvas.pack(side=tk.RIGHT)

        self.status_dot = self.status_canvas.create_oval(2, 2, 12, 12, fill="#ff5555", outline="")
        
        self.ent_ip = ttk.Entry(conn_frame)
        self.ent_ip.pack(fill=tk.X, pady=(0, 5))
        self.ent_ip.insert(0, "192.168.1.xxx")
        
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection)
        self.btn_connect.pack(fill=tk.X, pady=(0, 5))
        self.btn_scan = ttk.Button(conn_frame, text="Scan for Lite 6", command=self._scan_network)
        self.btn_scan.pack(fill=tk.X)

        # 2. Sliders (Joints)
        lf = ttk.LabelFrame(left_col, text="Manual Joint Control")
        # Let op: we packen deze straks pas, zodat we eerst de Quit button kunnen plaatsen
        
        self.vars = []
        self.joint_labels = [] 
        
        for i in range(JOINT_COUNT):
            row = ttk.Frame(lf)
            row.pack(fill=tk.X, pady=4)
            
            header = ttk.Frame(row)
            header.pack(fill=tk.X)
            ttk.Label(header, text=f"Joint {i+1}").pack(side=tk.LEFT)
            lbl_val = ttk.Label(header, text="0.0", width=5, anchor="e")
            lbl_val.pack(side=tk.RIGHT)
            self.joint_labels.append(lbl_val)
            
            v = tk.DoubleVar()
            self.vars.append(v)
            s = ttk.Scale(row, from_=-180, to=180, variable=v, 
                      command=lambda val, idx=i: self._slider_cb(idx, val))
            s.pack(fill=tk.X, pady=(0, 5))

        
        ttk.Button(lf, text="Reset to Home", command=self._home).pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(left_col, text="Quit to Desktop", command=self._on_close).pack(side=tk.BOTTOM, fill=tk.X, pady=10)

        lf.pack(fill=tk.BOTH, expand=True, pady=5)

        # RIGHT
        right_col = ttk.Frame(top_container)
        right_col.grid(row=0, column=1, sticky="nsew", padx=(5, 0))

        # 1. End-Effector Configuration
        ef_frame = ttk.LabelFrame(right_col, text="End-Effector Configuration")
        ef_frame.pack(fill=tk.X, pady=5)

        ef_top = ttk.Frame(ef_frame)
        ef_top.pack(fill=tk.X, pady=2)
        ttk.Checkbutton(ef_top, text="Custom end-effector is in millimeters?", variable=self.scale_mm_var).pack(side=tk.LEFT, padx=5)

        ef_mid = ttk.Frame(ef_frame)
        ef_mid.pack(fill=tk.X, pady=2)
        self.combo_stls = ttk.Combobox(ef_mid, state="readonly")
        self.combo_stls.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.combo_stls.bind("<<ComboboxSelected>>", self._on_stl_history_select)
        ttk.Button(ef_mid, text="Load .stl file...", command=self._load_custom_gripper).pack(side=tk.LEFT, padx=5)

        ef_bot = ttk.Frame(ef_frame)
        ef_bot.pack(fill=tk.X, pady=(5, 2))
        ttk.Label(ef_bot, text="Presets:").pack(side=tk.LEFT, padx=5)
        ttk.Button(ef_bot, text="Default Gripper", command=self._load_std_gripper).pack(side=tk.LEFT, padx=2)
        ttk.Button(ef_bot, text="Vacuum Gripper", command=self._load_vac_gripper).pack(side=tk.LEFT, padx=2)
        ttk.Button(ef_bot, text="Remove Effector", command=self._remove_gripper).pack(side=tk.LEFT, padx=2)

        # 2. Script Loading
        sf = ttk.LabelFrame(right_col, text="Script Loading")
        sf.pack(fill=tk.X, pady=5)
        
        sf_inner = ttk.Frame(sf)
        sf_inner.pack(fill=tk.X, pady=5)
        self.combo_history = ttk.Combobox(sf_inner, state="readonly")
        self.combo_history.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.combo_history.bind("<<ComboboxSelected>>", self._on_history_select)
        self.btn_load_script = ttk.Button(sf_inner, text="Load .py script...", command=self._browse_script)
        self.btn_load_script.pack(side=tk.LEFT, padx=5)

        # 3. General Settings
        setf = ttk.LabelFrame(right_col, text="General Settings")
        setf.pack(fill=tk.X, pady=5)
        
        set_inner = ttk.Frame(setf)
        set_inner.pack(fill=tk.X, pady=5)
        ttk.Checkbutton(set_inner, text="Loop Script", variable=self.loop_var).pack(side=tk.LEFT, padx=5)
        
        ttk.Separator(set_inner, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        
        ttk.Label(set_inner, text="Sim Speed:").pack(side=tk.LEFT)
        self.speed_scale = ttk.Scale(set_inner, from_=0.1, to=5.0, variable=self.speed_var, command=self._speed_cb)
        self.speed_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.speed_lbl = ttk.Label(set_inner, text="1.0x", width=4)
        self.speed_lbl.pack(side=tk.LEFT)

        # 4. Simulation Controls
        cf = ttk.LabelFrame(right_col, text="Simulation Controls")
        cf.pack(fill=tk.X, pady=5)
        cf.columnconfigure(0, weight=1)
        cf.columnconfigure(1, weight=1)
        cf.columnconfigure(2, weight=1)
        cf.columnconfigure(3, weight=1)

        # Big buttons
        self.btn_run = ttk.Button(cf, text="▶ Run", command=self._run_current_script, state=tk.DISABLED)
        self.btn_run.grid(row=0, column=0, sticky="ew", padx=5, pady=10)
        
        self.btn_stop = ttk.Button(cf, text="⏹ Stop", command=self._stop_script, state=tk.DISABLED)
        self.btn_stop.grid(row=0, column=3, sticky="ew", padx=5, pady=10)
        
        self.btn_pause = ttk.Button(cf, text="⏸ Pause", command=self._toggle_pause, state=tk.DISABLED)
        self.btn_pause.grid(row=0, column=1, sticky="ew", padx=5, pady=10)
        
        self.btn_restart = ttk.Button(cf, text="↻ Restart", command=self._restart_script, state=tk.DISABLED)
        self.btn_restart.grid(row=0, column=2, sticky="ew", padx=5, pady=10)

        # 5. View Controls
        vf = ttk.LabelFrame(right_col, text="Camera Controls")
        vf.pack(fill=tk.X, pady=5)
        
        # Grid layout for view buttons
        vf.columnconfigure(0, weight=1)
        vf.columnconfigure(1, weight=1)
        vf.columnconfigure(2, weight=1)
        vf.columnconfigure(3, weight=1)
        vf.columnconfigure(4, weight=1)
        vf.columnconfigure(5, weight=1)
        
        ttk.Button(vf, text="Front", command=lambda: self.viz.set_camera_view('front', 1.6)).grid(row=0, column=0, sticky="ew", padx=2, pady=5)
        ttk.Button(vf, text="Left", command=lambda: self.viz.set_camera_view('side-l', 1.6)).grid(row=0, column=1, sticky="ew", padx=2, pady=5)
        ttk.Button(vf, text="Right", command=lambda: self.viz.set_camera_view('side-r', 1.6)).grid(row=0, column=2, sticky="ew", padx=2, pady=5)
        ttk.Button(vf, text="Rear", command=lambda: self.viz.set_camera_view('rear', 1.6)).grid(row=0, column=3, sticky="ew", padx=2, pady=5)
        ttk.Button(vf, text="Top", command=lambda: self.viz.set_camera_view('top', 1.6)).grid(row=0, column=4, sticky="ew", padx=2, pady=5)
        ttk.Button(vf, text="Iso", command=self._reset_view).grid(row=0, column=5, sticky="ew", padx=2, pady=5)

        # Colour Settings
        cf_frame = ttk.LabelFrame(right_col, text="Colour Settings")
        cf_frame.pack(fill=tk.X, pady=5)

        def create_color_row(parent, label_text, default_val, target_key):
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=2)
            
            ttk.Label(row, text=label_text, width=10).pack(side=tk.LEFT, padx=5)
            
            var = tk.StringVar(value=default_val)
            ent = ttk.Entry(row, textvariable=var, width=10)
            ent.pack(side=tk.LEFT, padx=5)

            preview_lbl = tk.Label(row, width=3, relief="solid", borderwidth=1)
            preview_lbl.pack(side=tk.LEFT, padx=(0, 5))
            
            def update_preview(color):
                try:
                    self.winfo_rgb(color) 
                    preview_lbl.config(bg=color, text="")
                except Exception:
                    preview_lbl.config(bg="black", text="?")

            update_preview(default_val)

            var.trace_add("write", lambda *args: update_preview(var.get()))

            def reset_action():
                var.set(default_val)
                self._apply_color(target_key, default_val)

            ttk.Button(row, text="↺", width=3,
                       command=reset_action
            ).pack(side=tk.RIGHT, padx=(2, 5))
            
            ttk.Button(row, text="Apply", 
                       command=lambda: self._apply_color(target_key, var.get())
            ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))

        create_color_row(cf_frame, "Background:", "#0f0f0f", "bg")
        create_color_row(cf_frame, "Robot Arm:", "#f4f4f4", "arm")
        create_color_row(cf_frame, "Wrist:", "#ff5015", "wrist")
        create_color_row(cf_frame, "End-Effector:", "#f4f4f4", "eef")
        create_color_row(cf_frame, "Path Line:", "#159dff", "trace")
        
        ttk.Separator(cf_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)
        
        # ROW 1: TRACE
        trace_row = ttk.LabelFrame(right_col, text="Trace Settings")
        trace_row.pack(fill=tk.X, pady=5)

        def toggle_trace():
            self.viz.set_trace_enable(self.trace_var.get())
            
        ttk.Checkbutton(trace_row, text="Trace Path", 
                        variable=self.trace_var, 
                        command=toggle_trace).pack(side=tk.LEFT, padx=5)

        self.trace_mode_var = tk.StringVar(value="Wrist")
        
        def change_trace_source(event):
            mode = self.trace_mode_var.get().lower()
            self.viz.trace_source = mode
            self.viz.clear_trace() 

        trace_combo = ttk.Combobox(trace_row, textvariable=self.trace_mode_var, 
                                   values=["Wrist", "Effector Tip"], state="readonly", width=10)
        trace_combo.pack(side=tk.LEFT, padx=5)
        trace_combo.bind("<<ComboboxSelected>>", change_trace_source)


        # ROW 2: GHOST
        ghost_row = ttk.LabelFrame(right_col, text="Visibility Settings")
        ghost_row.pack(fill=tk.X, pady=5)

        self.ghost_mode_var = tk.BooleanVar(value=False)
        self.ignore_eef_var = tk.BooleanVar(value=False)

        def update_ghost_state():
            is_ghost = self.ghost_mode_var.get()
            ignore_eef = self.ignore_eef_var.get()    
            
            self.viz.set_ghost_mode(is_ghost, ignore_eef)
            
            if is_ghost:
                chk_ignore.state(["!disabled"]) 
            else:
                chk_ignore.state(["disabled"])

        ttk.Checkbutton(ghost_row, text="Ghost Mode", 
                        variable=self.ghost_mode_var, 
                        command=update_ghost_state).pack(side=tk.LEFT, padx=5)

        chk_ignore = ttk.Checkbutton(ghost_row, text="Ignore Effector", 
                                     variable=self.ignore_eef_var, 
                                     command=update_ghost_state,
                                     state="disabled")
        chk_ignore.pack(side=tk.LEFT, padx=5)


        # COLUMN 3
        log_col = ttk.Frame(top_container)
        log_col.grid(row=0, column=2, sticky="nsew", padx=(5, 0))

        logf = ttk.LabelFrame(log_col, text="System Log")
        logf.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.txt = scrolledtext.ScrolledText(logf, height=8, font=("Terminal", 11))
        self.txt.pack(fill=tk.BOTH, expand=True)

    def _speed_cb(self, val):
        v = float(val)
        self.speed_lbl.config(text=f"{v:.1f}x")
        self.api.speed_multiplier = v

    def _slider_cb(self, idx, val):
        with self.data_lock:
            current = [v.get() for v in self.vars]
            self.api.joints_deg = current 
            self.viz.update_joints(current)
            
        self.joint_labels[idx].config(text=f"{float(val):.1f}")

    def _home(self):
        self.ctx.log_queue.put("[GUI] Going home...")
        self.viz.clear_trace()
        self.api.joints_deg = [0.0] * JOINT_COUNT
        self.ctx.joint_queue.put([0.0]*JOINT_COUNT)
        if self.api.real_arm:
            self.api.set_servo_angle([0]*6, speed=30, wait=False)

    def _reset_view(self):
        if self.viz: self.viz.reset_camera_view()

    def _browse_script(self):
        path = filedialog.askopenfilename(filetypes=[("Python", "*.py")])
        if path: self._add_to_history(path)

    def _add_to_history(self, path):
        if path in self.script_history:
            self.script_history.remove(path)
        self.script_history.insert(0, path)
        
        self.current_script_path = path
        self.btn_run.config(state=tk.NORMAL)
        
        display_names = []
        for p in self.script_history:
            name = os.path.basename(p)
            if config.EXAMPLES_DIR in p: 
                name = f"[Example] {name}"
            display_names.append(name)
        
        self.combo_history['values'] = display_names
        self.combo_history.set(display_names[0])

        try:
            user_scripts = [p for p in self.script_history if config.EXAMPLES_DIR not in p]

            with open(config.HISTORY_FILE, "w") as f:
                for p in user_scripts[:10]: 
                    f.write(p + "\n")
        except Exception as e: 
            print(f"History save error: {e}")

    def _on_history_select(self, event):
        idx = self.combo_history.current()
        if idx >= 0:
            self.current_script_path = self.script_history[idx]
            self.btn_run.config(state=tk.NORMAL)

    def _toggle_controls(self, running):
        state = tk.NORMAL if running else tk.DISABLED
        inv_state = tk.DISABLED if running else tk.NORMAL

        self.btn_pause.config(state=state)
        self.btn_stop.config(state=state)
        self.btn_restart.config(state=state)
        self.btn_run.config(state=inv_state) 

        config_state = tk.DISABLED if running else tk.NORMAL
        
        self.ent_ip.config(state=config_state) # IP field
        self.btn_load_script.config(state=config_state) # Script load button
        self.speed_scale.config(state=config_state) # Speed slider
        # Script dropdown
        if running:
            self.combo_history.config(state="disabled")
        else:
            self.combo_history.config(state="readonly")

    def _toggle_pause(self):
        if self.ctx.paused:
            self.ctx.paused = False
            self.btn_pause.config(text="⏸ Pause")
            self.ctx.log_queue.put("[UI] Resumed")
        else:
            self.ctx.paused = True
            self.btn_pause.config(text="▶ Resume")
            self.ctx.log_queue.put("[UI] Paused")

    def _stop_script(self):
        self.ctx.stop_flag = True
        self.ctx.paused = False
        self._home()
        self.ctx.log_queue.put("[UI] Stop signal...")
        

    def _restart_script(self):
        self._stop_script()
        self.after(500, self._run_current_script)

    def _run_current_script(self):
        if not self.current_script_path: return
        self.viz.clear_trace()
        self.ctx.stop_flag = False
        self.ctx.paused = False
        self.btn_pause.config(text="⏸ Pause")
        self._toggle_controls(True)
        
        threading.Thread(target=self._run_script_thread, args=(self.current_script_path,), daemon=True).start()

    def _on_script_finished(self):
        self._toggle_controls(False)
        if self.ctx.stop_flag:
            self.ctx.log_queue.put("[LOOP] Stopped by user.")
            self.api.disconnect_real_robot()
            return
        if self.loop_var.get():
            self.ctx.log_queue.put("[LOOP] Looping...")
            self.after(0, self._run_current_script)
        else:
            self.api.disconnect_real_robot()

    def _apply_color(self, target, color_str):
        success = self.viz.set_color(target, color_str)
        if success:
            self.ctx.log_queue.put(f"[UI] Color updated for '{target}' -> {color_str}")
        else:
            messagebox.showerror("Color Error", f"Invalid color code: {color_str}\nUse Hex (e.g. #FF0000) or names (red, blue).")
    
    def _load_stl_history(self):
        self.stl_history = []
        
        if os.path.exists(config.STL_HISTORY_FILE):
            try:
                with open(config.STL_HISTORY_FILE, "r") as f:
                    lines = f.readlines()
                    for line in lines:
                        path = line.strip()
                        if path and os.path.exists(path):
                            if path not in self.stl_history:
                                self.stl_history.append(path)
            except Exception as e:
                print(f"[GUI] STL History load error: {e}")

        display_names = [os.path.basename(p) for p in self.stl_history]
        self.combo_stls['values'] = display_names
        
        if display_names:
            self.combo_stls.set("Select recent STL...")

    def _add_to_stl_history(self, path):
        if path in self.stl_history:
            self.stl_history.remove(path)
        self.stl_history.insert(0, path)
        
        self.stl_history = self.stl_history[:10]

        try:
            with open(config.STL_HISTORY_FILE, "w") as f:
                for p in self.stl_history:
                    f.write(p + "\n")
        except Exception as e:
            print(f"[GUI] STL History save error: {e}")

        # Update UI direct
        display_names = [os.path.basename(p) for p in self.stl_history]
        self.combo_stls['values'] = display_names
        self.combo_stls.current(0)

    def _on_stl_history_select(self, event):
        idx = self.combo_stls.current()
        if idx >= 0 and idx < len(self.stl_history):
            path = self.stl_history[idx]
            
            self.rendering_paused = True
            self.update_idletasks()
            try:
                do_scale = self.scale_mm_var.get()
                success = self.viz.set_custom_gripper(path, scale_to_meters=do_scale)
                if success:
                    self.ctx.log_queue.put(f"[GUI] Loaded from history: {os.path.basename(path)}")
            finally:
                self.rendering_paused = False

    # SCAN NETWORK FUNCTION
    def _set_status_color(self, color_code):
        self.status_canvas.itemconfig(self.status_dot, fill=color_code)

    def _toggle_connection(self):
        if self.api.is_connected:
            self.api.disconnect_real_robot()
            self.btn_connect.config(text="Connect", state=tk.NORMAL)
            self.ent_ip.config(state=tk.NORMAL)
            self._set_status_color("#ff5555") # Rood
            self.btn_scan.config(state=tk.NORMAL)
            self.ctx.log_queue.put("[GUI] Disconnected manually.")
        else:
            # CONNECT
            ip = self.ent_ip.get().strip()
            
            allowed = set("0123456789.")
            if not ip or not set(ip).issubset(allowed):
                messagebox.showwarning("Invalid IP", "IP Address contains invalid characters.\nOnly numbers (0-9) and dots (.) are allowed.")
                return

            self.btn_connect.config(state=tk.DISABLED, text="Connecting...")
            self.ent_ip.config(state=tk.DISABLED)
            self._set_status_color("#ffb86c") # Oranje
            
            threading.Thread(target=self._connect_thread, args=(ip,), daemon=True).start()

    def _connect_thread(self, ip):
        success, msg = self.api.connect_real_robot(ip)
        self.after(0, lambda: self._connect_complete(success, msg))

    def _connect_complete(self, success, msg):
        if success:
            self.btn_connect.config(text="Disconnect", state=tk.NORMAL)
            self._set_status_color("#50fa7b") # Groen
            self.ctx.log_queue.put(f"[GUI] Status: {msg}")
        else:
            self.btn_connect.config(text="Connect", state=tk.NORMAL)
            self.ent_ip.config(state=tk.NORMAL) 
            self._set_status_color("#ff5555") # Rood
            
            self.ctx.log_queue.put(f"[CONNECTION FAILED] {msg}")
            
            messagebox.showerror("Connection Failed", f"Could not connect:\n{msg}")

    def _scan_complete(self, ips):
        self.btn_scan.config(text="Scan for Lite 6", state=tk.NORMAL)
        self._set_status_color("#ff5555")
        
        if ips:
            found = ips[0]
            self.ent_ip.delete(0, tk.END)
            self.ent_ip.insert(0, found)
            
            # Auto Connect Logic
            self.ctx.log_queue.put(f"[GUI] Auto-connecting to found IP: {found}")
            self._toggle_connection() # Connect to ip
            
            self.ctx.log_queue.put("Found!", f"Robot found at {found} with port {ROBOT_SCAN_PORT}\nConnecting automatically...")
        else:
            messagebox.showinfo("Not found", f"No xArm/Lite 6 robots found on port {ROBOT_SCAN_PORT}.")

    def _scan_network(self):
        self.btn_scan.config(text="Scanning...", state=tk.DISABLED)
        self._set_status_color("#ffb86c") # Oranje tijdens scannen
        threading.Thread(target=self._scan_thread, daemon=True).start()

    def _scan_thread(self):
        found_ips = []
        base_ip = "192.168.1"
        # Find local subnet
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            base_ip = ".".join(local_ip.split(".")[:3])
        except: 
            pass 
        
        self.ctx.log_queue.put(f"[SCAN] Range: {base_ip}.1 - 254 | Port: {config.ROBOT_SCAN_PORT}")
        # Scan network for addresses 1 - 255 on port {ROBOT_SCAN_PORT}
        for i in range(1, 255):
            if self.ctx.stop_flag:
                break

            target_ip = f"{base_ip}.{i}"
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.05) # 50ms timeout per IP
            
            try:
                if sock.connect_ex((target_ip, config.ROBOT_SCAN_PORT)) == 0:
                    found_ips.append(target_ip)
                    self.ctx.log_queue.put(f"[SCAN] Ping succes: {target_ip}")
            except: 
                pass
            finally: 
                sock.close()
        self.after(0, lambda: self._scan_complete(found_ips))

    def _run_script_thread(self, path):
        if 'xarm' in sys.modules: del sys.modules['xarm']
        if 'xarm.wrapper' in sys.modules: del sys.modules['xarm.wrapper']
        xarm_mod = types.ModuleType('xarm')
        wrap_mod = types.ModuleType('xarm.wrapper')

        live_api_instance = self.api 

        def API_Factory(ip, **kwargs): 
            return live_api_instance
        
        wrap_mod.XArmAPI = API_Factory
        xarm_mod.wrapper = wrap_mod
        sys.modules['xarm'] = xarm_mod
        sys.modules['xarm.wrapper'] = wrap_mod
        
        self.ctx.log_queue.put(f"--- Start: {os.path.basename(path)} ---")
        try:
            runpy.run_path(path, run_name="__main__")
        except SystemExit as e:
            self.ctx.log_queue.put(f"--- {e} ---")
        except Exception as e:
            self.ctx.log_queue.put(f"Error: {e}")
            traceback.print_exc()
        finally:
            self.ctx.log_queue.put("--- Done ---")
            self.after(100, self._on_script_finished)

    def _on_close(self):
        print("[SYSTEM] Closing application...")
        self.ctx.stop_flag = True
        
        if self.api: 
            self.api.disconnect_real_robot()
            
        try: 
            if self.viz and self.viz.plotter and hasattr(self.viz.plotter, 'ren_win'):
                self.viz.plotter.close()
        except: pass
        try: self.destroy()
        except: pass
        sys.exit(0)

    # AUTO UPDATER
    def _start_update_check(self):
        threading.Thread(target=self._update_thread, daemon=True).start()

    def _update_thread(self):
        try:
            url = f"https://api.github.com/repos/{config.REPO_OWNER}/{config.REPO_NAME}/releases/latest"
            
            with urlopen(url, timeout=3) as response:
                data = json.loads(response.read().decode())
                
                latest_tag = data.get("tag_name", "").lstrip("v")
                assets = data.get("assets", [])
                
                # Check if GitHub release is newer than current app
                if self._is_version_newer(latest_tag, config.APP_VERSION):
                    system = platform.system().lower() 
                    
                    download_url = None
                    asset_name = ""
                    
                    search_term = ""
                    if "windows" in system:
                        search_term = "windows"
                    elif "darwin" in system:
                        search_term = "macos"
                    else:
                        return # Linux: Nothing
                    
                    # Search for match for OS
                    for asset in assets:
                        name = asset["name"].lower()
                        if search_term in name and name.endswith(".zip"):
                            download_url = asset["browser_download_url"]
                            asset_name = asset["name"]
                            break
                    
                    if download_url:
                        self.pending_update_data = {
                            "version": latest_tag,
                            "url": download_url,
                            "name": asset_name
                        }
                        self.after(0, lambda: self._show_update_dialog(latest_tag, download_url, asset_name))

        except Exception as e:
            pass 
    
    def _is_version_newer(self, latest, current):
        try:
            l_parts = [int(x) for x in latest.split('.') if x.isdigit()]
            c_parts = [int(x) for x in current.split('.') if x.isdigit()]
            return l_parts > c_parts
        except: return False

    def _show_update_dialog(self, version, download_url, asset_name):
        msg = f"A new version of LiteSim is available: {version}\n\n"
        # msg += f"Detected Platform: {platform.system()}\n"
        # msg += f"File found: {asset_name}\n\n"
        msg += "Do you want to download this update?"
        
        if messagebox.askyesno("Update Available", msg):
            self._start_download(download_url, asset_name)
        else:
            self.btn_update_available.pack(side=tk.LEFT, padx=(10, 0))

    def _on_update_click(self):
        if self.pending_update_data:
            self.btn_update_available.pack_forget()
            
            d = self.pending_update_data
            self._show_update_dialog(d["version"], d["url"], d["name"])

    def _start_download(self, url, filename):
        self.dl_win = tk.Toplevel(self)
        self.dl_win.title("Downloading Update...")
        self.dl_win.geometry("400x150")
        self.dl_win.resizable(False, False)
        
        ttk.Label(self.dl_win, text=f"Downloading {filename}...").pack(pady=(20, 10))
        
        self.progress = ttk.Progressbar(self.dl_win, orient="horizontal", length=300, mode="determinate")
        self.progress.pack(pady=10)
        
        threading.Thread(target=self._download_worker, args=(url, filename), daemon=True).start()

    def _download_worker(self, url, filename):
        try:
            download_dir = Path.home() / "Downloads"
            target_path = download_dir / filename
            
            def report_hook(block_num, block_size, total_size):
                downloaded = block_num * block_size
                if total_size > 0:
                    percent = (downloaded / total_size) * 100
                    self.after(0, lambda: self.progress.configure(value=percent))

            urlretrieve(url, target_path, reporthook=report_hook)
            self.after(0, lambda: self.dl_win.destroy())
            self.after(0, lambda: self._on_download_success(target_path, download_dir))
            
        except Exception as e:
            self.after(0, lambda: messagebox.showerror("Download Failed", str(e)))
            self.after(0, lambda: self.dl_win.destroy())

    def _on_download_success(self, zip_path, folder_path):
        try:
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                zip_ref.extractall(folder_path)
                extracted_item_name = zip_ref.namelist()[0]
                full_extracted_path = folder_path / extracted_item_name
                
            self.ctx.log_queue.put(f"[UPDATE] Successfully extracted to: {folder_path}")

            if platform.system() == "Windows":
                os.startfile(folder_path)
                
            elif platform.system() == "Darwin": # macOS
                try:
                    os.system(f"open -R '{full_extracted_path}'")
                except:
                    os.system(f"open '{folder_path}'")
            
            print("[UPDATE] Update finished. Closing application now.")
            self._on_close()
                    
        except Exception as e:
            messagebox.showerror("Extraction Error", f"Downloaded but failed to extract:\n{e}")

if __name__ == "__main__":
    app = ControlPanel()
    app.mainloop()