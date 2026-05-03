import tkinter as tk
from tkinter import ttk, messagebox
import math
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import time

class RobotSimulator3D:
    def __init__(self, root):
        self.root = root
        self.root.title("3D SCARA - Workspace Restored & Z-Axis Fixed")
        self.root.geometry("1600x950")
        
        # --- Physical State ---
        self.l1, self.l2, self.l3 = 150.0, 100.0, 120.0
        self.base_h = 150.0
        self.theta1, self.theta2, self.z_slide = math.radians(45), math.radians(45), 40.0 
        self.max_theta1, self.max_theta2 = 180.0, 150.0
        
        # --- Mission State ---
        self.stop_requested = False
        self.obs_enabled = tk.BooleanVar(value=False)
        self.motion_mode = tk.StringVar(value="SCURVE")
        self.obs_pos = [80.0, 80.0, 0.0]
        self.obs_size = [60.0, 60.0, 80.0]
        self.waypoint_a = None
        self.waypoint_b = None
        
        # --- Data Tracking ---
        self.history = {'t': [], 't1': [], 't2': [], 'v1': [], 'v2': [], 'det': []}
        self.trajectory_pts = []

        self.setup_ui()
        self.update_plot()

    def setup_ui(self):
        self.nav_frame = ttk.Frame(self.root, padding=5)
        self.nav_frame.pack(side=tk.TOP, fill='x')
        
        self.sys_mode = tk.StringVar(value="MANUAL")
        for m in [("Manual Mode", "MANUAL"), ("Mission Mode", "MISSION"), ("Hardware Mode", "HARDWARE")]:
            ttk.Radiobutton(self.nav_frame, text=m[0], variable=self.sys_mode, 
                            value=m[1], command=self.refresh_layout).pack(side=tk.LEFT, padx=15)

        self.main_content = ttk.Frame(self.root)
        self.main_content.pack(fill=tk.BOTH, expand=True)

        self.left_panel = ttk.Frame(self.main_content, padding=10, width=450)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y)
        self.left_panel.pack_propagate(False)

        self.viz_container = ttk.Frame(self.main_content)
        self.viz_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.fig = Figure(figsize=(10, 10), dpi=90)
        self.ax_3d = self.fig.add_subplot(211, projection='3d')
        self.ax_angle = self.fig.add_subplot(234)
        self.ax_vel = self.fig.add_subplot(235)
        self.ax_sing = self.fig.add_subplot(236)
        
        self.fig.tight_layout(pad=4.0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.viz_container)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.refresh_layout()

    def refresh_layout(self):
        for w in self.left_panel.winfo_children(): w.destroy()
        mode = self.sys_mode.get()
        if mode == "MANUAL": self.build_manual_ui()
        elif mode == "HARDWARE": self.build_hardware_ui()
        elif mode == "MISSION": self.build_mission_ui()

    def build_manual_ui(self):
        ttk.Label(self.left_panel, text="MANUAL CONTROL", font=('Arial', 12, 'bold')).pack(pady=5)
        self.ctrl_logic = tk.StringVar(value="IK")
        for t, m in [("FK Mode", "FK"), ("IK Mode", "IK"), ("Joystick", "JOY"), ("Sliders", "SLIDE")]:
            ttk.Radiobutton(self.left_panel, text=t, variable=self.ctrl_logic, value=m).pack(anchor='w')
        
        joy_frame = ttk.LabelFrame(self.left_panel, text="Planar Joystick", padding=5); joy_frame.pack(fill='x', pady=5)
        self.joy_canvas = tk.Canvas(joy_frame, width=200, height=200, bg="#f0f0f0", relief="sunken", bd=1); self.joy_canvas.pack()
        self.joy_pointer = self.joy_canvas.create_oval(95, 95, 105, 105, fill="red")
        self.joy_canvas.bind("<B1-Motion>", self.handle_joystick_event)
        
        self.slider_frame = ttk.LabelFrame(self.left_panel, text="X-Y-Z Sliders", padding=5); self.slider_frame.pack(fill='x', pady=5)
        self.s_x = self.create_slider(self.slider_frame, "X", -300, 300)
        self.s_y = self.create_slider(self.slider_frame, "Y", -300, 300)
        self.s_z = self.create_slider(self.slider_frame, "Z (Slide)", 0, self.l3)
        
        input_frame = ttk.LabelFrame(self.left_panel, text="Manual Entry", padding=5); input_frame.pack(fill='x', pady=5)
        self.in_a = self.create_input(input_frame, "X / θ1:", 150)
        self.in_b = self.create_input(input_frame, "Y / θ2:", 150)
        self.in_c = self.create_input(input_frame, "Z-Slide:", 40)
        ttk.Button(input_frame, text="Apply Changes", command=self.handle_apply).pack(fill='x', pady=5)

    def build_hardware_ui(self):
        ttk.Label(self.left_panel, text="HARDWARE CONFIGURATION", font=('Arial', 11, 'bold')).pack(pady=5)
        self.h_l1 = self.create_input(self.left_panel, "Link 1 Length:", self.l1)
        self.h_l2 = self.create_input(self.left_panel, "Link 2 Length:", self.l2)
        self.h_l3 = self.create_input(self.left_panel, "Link 3 Rigid Shaft:", self.l3)
        self.h_base = self.create_input(self.left_panel, "Base Height:", self.base_h)
        self.h_m1 = self.create_input(self.left_panel, "Max Motor 1 Angle:", self.max_theta1)
        self.h_m2 = self.create_input(self.left_panel, "Max Motor 2 Angle:", self.max_theta2)
        ttk.Button(self.left_panel, text="Save & Update", command=self.save_hardware).pack(fill='x', pady=10)

    def build_mission_ui(self):
        ttk.Label(self.left_panel, text="MISSION CONTROL", font=('Arial', 12, 'bold')).pack(pady=5)
        wf = ttk.LabelFrame(self.left_panel, text="Coordinates", padding=5); wf.pack(fill='x', pady=5)
        self.m_p1 = self.create_input(wf, "Point A:", "150, 100, 40")
        self.m_p2 = self.create_input(wf, "Point B:", "-50, 180, 20")
        ttk.Button(wf, text="Apply Waypoints", command=self.apply_waypoints).pack(fill='x', pady=2)
        
        pf = ttk.LabelFrame(self.left_panel, text="Motion Profile", padding=5); pf.pack(fill='x', pady=5)
        ttk.Radiobutton(pf, text="Step", variable=self.motion_mode, value="STEP").pack(side=tk.LEFT, padx=10)
        ttk.Radiobutton(pf, text="S-Curve", variable=self.motion_mode, value="SCURVE").pack(side=tk.LEFT)

        of = ttk.LabelFrame(self.left_panel, text="Avoidance Settings", padding=5); of.pack(fill='x', pady=5)
        ttk.Checkbutton(of, text="Enable Avoidance", variable=self.obs_enabled).pack(anchor='w')
        self.m_obs_p = self.create_input(of, "Position:", "80, 80, 0")
        self.m_obs_s = self.create_input(of, "Size (W,D):", "60, 60, 80")
        self.m_obs_dist = self.create_input(of, "Avoidance Dist:", "50")
        ttk.Button(of, text="Update Obstacle", command=self.apply_obstacle).pack(fill='x', pady=2)

        self.m_repeat = self.create_input(self.left_panel, "Repeat:", "1")
        
        btn_f = ttk.Frame(self.left_panel); btn_f.pack(fill='x', pady=10)
        self.start_btn = ttk.Button(btn_f, text="START MISSION", command=self.run_mission)
        self.start_btn.pack(side=tk.LEFT, fill='x', expand=True, padx=2)
        self.stop_btn = tk.Button(btn_f, text="STOP", bg="#ffcccc", fg="red", command=self.request_stop)
        self.stop_btn.pack(side=tk.RIGHT, fill='x', expand=True, padx=2)

    def request_stop(self):
        self.stop_requested = True

    def apply_waypoints(self):
        try:
            self.waypoint_a = [float(x.strip()) for x in self.m_p1.get().split(',')]
            self.waypoint_b = [float(x.strip()) for x in self.m_p2.get().split(',')]
            self.update_plot()
        except: messagebox.showerror("Error", "Check Coordinate Format (X,Y,Z)")

    def apply_obstacle(self):
        try:
            self.obs_pos = [float(x.strip()) for x in self.m_obs_p.get().split(',')]
            self.obs_size = [float(x.strip()) for x in self.m_obs_s.get().split(',')]
            self.update_plot()
        except: pass

    def run_mission(self):
        if not self.waypoint_a or not self.waypoint_b: return
        self.stop_requested = False
        self.history = {'t': [], 't1': [], 't2': [], 'v1': [], 'v2': [], 'det': []}
        self.trajectory_pts = []
        t_ref = time.time()
        
        try:
            repeats = int(self.m_repeat.get())
            for _ in range(repeats):
                if self.stop_requested: break
                if not self.execute_path(self.waypoint_a, self.waypoint_b, t_ref): break
                if self.stop_requested: break
                if not self.execute_path(self.waypoint_b, self.waypoint_a, t_ref): break
        except Exception as e:
            messagebox.showerror("Mission Error", str(e))
        finally:
            self.stop_requested = False

    def execute_path(self, start, end, t_ref):
        steps = 60
        prev_t1, prev_t2 = self.theta1, self.theta2
        
        try: avoid_dist = float(self.m_obs_dist.get())
        except: avoid_dist = 50.0

        for i in range(steps + 1):
            if self.stop_requested: return False
            ratio = i / steps
            if self.motion_mode.get() == "SCURVE":
                ratio = 1 / (1 + math.exp(-8 * (ratio - 0.5)))

            cx = start[0] + (end[0] - start[0]) * ratio
            cy = start[1] + (end[1] - start[1]) * ratio
            cz = start[2] + (end[2] - start[2]) * ratio

            if self.obs_enabled.get():
                ox, oy, _ = self.obs_pos
                ow, od, _ = self.obs_size
                cent_x, cent_y = ox + ow/2, oy + od/2
                dist = math.sqrt((cx - cent_x)**2 + (cy - cent_y)**2)
                threshold = avoid_dist + (max(ow, od) / 2)
                if dist < threshold:
                    strength = avoid_dist * ((threshold - dist) / threshold) * 1.5
                    vec_x, vec_y = cx - cent_x, cy - cent_y
                    mag = math.sqrt(vec_x**2 + vec_y**2)
                    if mag > 0:
                        cx += (vec_x / mag) * strength
                        cy += (vec_y / mag) * strength

            if self.solve_ik(cx, cy, cz):
                deg1, deg2 = math.degrees(self.theta1), math.degrees(self.theta2)
                if not (0 <= deg1 <= self.max_theta1) or not (0 <= deg2 <= self.max_theta2):
                    messagebox.showerror("Hardware Limit", f"Joint out of range!\nTheta1: {deg1:.1f}\nTheta2: {deg2:.1f}")
                    return False

                dt = 0.05
                self.history['t'].append(time.time() - t_ref)
                self.history['t1'].append(deg1)
                self.history['t2'].append(deg2)
                self.history['v1'].append((self.theta1 - prev_t1)/dt)
                self.history['v2'].append((self.theta2 - prev_t2)/dt)
                self.history['det'].append(abs(self.l1 * self.l2 * math.sin(self.theta2)))
                self.trajectory_pts.append((cx, cy, self.base_h - self.z_slide))
                prev_t1, prev_t2 = self.theta1, self.theta2
                self.update_plot(); self.root.update(); time.sleep(0.01)
            else:
                messagebox.showwarning("IK Error", "Target out of reach.")
                return False
        return True

    def update_plot(self):
        self.ax_3d.clear()
        
        # --- Workspace Visualization (Restored) ---
        res = 50
        phi = np.linspace(0, math.radians(self.max_theta1), res)
        # Outer Boundary
        outer_x = (self.l1 + self.l2) * np.cos(phi)
        outer_y = (self.l1 + self.l2) * np.sin(phi)
        self.ax_3d.plot(outer_x, outer_y, [self.base_h]*res, color='gray', linestyle='--', lw=0.8, alpha=0.5)
        # Inner Boundary
        inner_r = abs(self.l1 - self.l2)
        inner_x = inner_r * np.cos(phi)
        inner_y = inner_r * np.sin(phi)
        self.ax_3d.plot(inner_x, inner_y, [self.base_h]*res, color='gray', linestyle='--', lw=0.8, alpha=0.5)

        if self.waypoint_a: self.ax_3d.scatter(*self.waypoint_a, color='blue', s=50)
        if self.waypoint_b: self.ax_3d.scatter(*self.waypoint_b, color='purple', s=50)
        if self.obs_enabled.get(): self.draw_cube(*self.obs_pos, *self.obs_size, color='orange')
        
        if len(self.trajectory_pts) > 1:
            pts = np.array(self.trajectory_pts)
            self.ax_3d.plot(pts[:,0], pts[:,1], pts[:,2], color='gray', linestyle='dashdot', lw=1, alpha=0.6)
        
        x1, y1 = self.l1 * math.cos(self.theta1), self.l1 * math.sin(self.theta1)
        x2, y2 = x1 + self.l2 * math.cos(self.theta1+self.theta2), y1 + self.l2 * math.sin(self.theta1+self.theta2)
        
        # Z-Axis Fixed Drawing
        zb = self.base_h - self.z_slide
        self.ax_3d.plot([0, x1, x2], [0, y1, y2], [self.base_h, self.base_h, self.base_h], color='black', lw=5)
        # Shaft visualization (Rigidly tied to base_h and zb)
        self.ax_3d.plot([x2, x2], [y2, y2], [zb, self.base_h + 20], color='red', lw=3)
        
        self.ax_3d.set_xlim(-300, 300); self.ax_3d.set_ylim(-300, 300); self.ax_3d.set_zlim(0, 400)
        self.ax_3d.set_xlabel('X'); self.ax_3d.set_ylabel('Y'); self.ax_3d.set_zlabel('Z')

        self.ax_angle.clear(); self.ax_angle.plot(self.history['t'], self.history['t1'], label='L1'); self.ax_angle.plot(self.history['t'], self.history['t2'], label='L2')
        self.ax_angle.set_title("Angle (deg)"); self.ax_angle.legend(loc='lower right')
        self.ax_vel.clear(); self.ax_vel.plot(self.history['t'], self.history['v1'], color='red'); self.ax_vel.plot(self.history['t'], self.history['v2'], color='green')
        self.ax_vel.set_title("Angular Vel (rad/s)")
        self.ax_sing.clear(); self.ax_sing.plot(self.history['t'], self.history['det'], color='orange'); self.ax_sing.set_title("Singularity Index")
        self.canvas.draw()

    def draw_cube(self, x, y, z, w, d, h, color):
        v = np.array([[x,y,z], [x+w,y,z], [x+w,y+d,z], [x,y+d,z], [x,y,z+h], [x+w,y,z+h], [x+w,y+d,z+h], [x,y+d,z+h]])
        f = [[0,1,2,3], [4,5,6,7], [0,1,5,4], [2,3,7,6], [0,3,7,4], [1,2,6,5]]
        for face in f: self.ax_3d.plot(v[face + [face[0]], 0], v[face + [face[0]], 1], v[face + [face[0]], 2], color=color)

    def solve_ik(self, x, y, z):
        d_sq = x**2 + y**2
        if (self.l1-self.l2)**2 <= d_sq <= (self.l1+self.l2)**2:
            cos_t2 = (d_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            t2 = math.acos(max(-1, min(1, cos_t2)))
            t1 = math.atan2(y, x) - math.atan2(self.l2 * math.sin(t2), self.l1 + self.l2 * math.cos(t2))
            self.theta1, self.theta2 = t1, t2
            # Z-axis value is clamped to physical limit (l3)
            self.z_slide = max(0, min(self.l3, z))
            return True
        return False

    def create_slider(self, parent, label, f, t):
        fr = ttk.Frame(parent); fr.pack(fill='x')
        ttk.Label(fr, text=label, width=5).pack(side=tk.LEFT)
        s = ttk.Scale(fr, from_=f, to=t, orient='horizontal', command=lambda _: self.update_from_slider())
        s.pack(side=tk.RIGHT, fill='x', expand=True); return s

    def update_from_slider(self):
        if self.sys_mode.get() == "MANUAL" and self.ctrl_logic.get() == "SLIDE":
            self.solve_ik(self.s_x.get(), self.s_y.get(), self.s_z.get()); self.update_plot()

    def handle_joystick_event(self, event):
        if self.ctrl_logic.get() == "JOY":
            self.solve_ik((event.x-100)*3, (100-event.y)*3, self.z_slide); self.update_plot()

    def handle_apply(self):
        try:
            a, b, c = float(self.in_a.get()), float(self.in_b.get()), float(self.in_c.get())
            if self.ctrl_logic.get() == "FK": self.theta1, self.theta2, self.z_slide = math.radians(a), math.radians(b), c
            else: self.solve_ik(a, b, c)
            self.update_plot()
        except: pass

    def create_input(self, parent, label, default):
        f = ttk.Frame(parent); f.pack(fill='x', pady=1)
        ttk.Label(f, text=label, width=15).pack(side=tk.LEFT)
        e = ttk.Entry(f, width=15); e.insert(0, str(default)); e.pack(side=tk.RIGHT); return e

    def save_hardware(self):
        try:
            self.l1, self.l2, self.l3 = float(self.h_l1.get()), float(self.h_l2.get()), float(self.h_l3.get())
            self.base_h, self.max_theta1, self.max_theta2 = float(self.h_base.get()), float(self.h_m1.get()), float(self.h_m2.get())
            self.update_plot(); messagebox.showinfo("Info", "Hardware Updated.")
        except: pass

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulator3D(root)
    root.mainloop()
