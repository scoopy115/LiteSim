import time
import math
import sys
import threading
import numpy as np
import config
from utils import normalize_angles, rpy_to_matrix

try:
    from xarm.wrapper import XArmAPI as RealXArmAPI
    HAS_REAL_SDK = True
except ImportError:
    HAS_REAL_SDK = False

GLOBAL_API_INSTANCE = None 

class SimXArmAPI:
    def __init__(self, ctx, chain):
        global GLOBAL_API_INSTANCE
        GLOBAL_API_INSTANCE = self
        
        self.ctx = ctx
        self.chain = chain 
        self.joints_deg = [0.0] * 6
        self.speed_multiplier = 1.0
        self.last_rpy = [180, 0, 0]
        self.real_arm = None
        
        # Monitor & Data
        self._monitor_running = False
        self._monitor_thread = None
        self.last_error_code = 0
        self.real_xyz = [0.0, 0.0, 0.0]

        self._update_gui()

    @property
    def is_connected(self):
        if self.real_arm and HAS_REAL_SDK:
            return self.real_arm.connected 
        return False

    def connect_real_robot(self, ip):
        if not HAS_REAL_SDK: return False, "SDK Missing"
        if not ip or "xxx" in ip: return False, "Invalid IP"

        try:
            self._log(f"[REAL] Connecting to {ip}...")
            self.real_arm = RealXArmAPI(ip)
            time.sleep(0.5)

            if self.real_arm.connected:
                self.real_arm.clean_warn()
                self.real_arm.clean_error()
                self.real_arm.motion_enable(True)
                self.real_arm.set_mode(0)
                self.real_arm.set_state(0)
                
                code, angles = self.real_arm.get_servo_angle(is_radian=False)
                if code == 0 and angles:
                    self.joints_deg = list(angles)[:6]
                    self._update_gui()
                
                code_pos, pos = self.real_arm.get_position(is_radian=False)
                if code_pos == 0 and pos:
                    self.last_rpy = [pos[3], pos[4], pos[5]]
                    self.real_xyz = [pos[0], pos[1], pos[2]]

                self._start_monitoring()
                return True, "Connected"
            else:
                self.real_arm = None
                return False, "Connection timed out"
        except Exception as e:
            self.real_arm = None
            return False, str(e)

    def disconnect_real_robot(self):
        self._stop_monitoring()
        
        # Handover
        if self.real_arm and self.real_arm.connected:
            try:
                code, angles = self.real_arm.get_servo_angle(is_radian=False)
                if code == 0 and angles: 
                    self.joints_deg = list(angles)[:6]
                
                code, pos = self.real_arm.get_position(is_radian=False)
                if code == 0 and pos:
                    self.last_rpy = [pos[3], pos[4], pos[5]]
                    self.real_xyz = [pos[0], pos[1], pos[2]]
            except: pass

        if self.real_arm:
            try: self.real_arm.disconnect()
            except: pass
            self.real_arm = None
            
        self._update_gui()
        self._log("[REAL] Disconnected. Simulator active.")

    # MONITOR
    def _start_monitoring(self):
        if self._monitor_running: return
        self._monitor_running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()

    def _stop_monitoring(self):
        self._monitor_running = False

    def _monitor_loop(self):
        while self._monitor_running and self.is_connected:
            try:
                code, codes = self.real_arm.get_err_warn_code()
                if code == 0:
                    curr = codes[0]
                    if curr != 0 and curr != self.last_error_code:
                        self.last_error_code = curr
                        if curr == 1: self.ctx.alert_queue.put("ESTOP")
                        else: self.ctx.alert_queue.put(f"CRASH:{curr}")
                    elif curr == 0:
                        self.last_error_code = 0

                code, angles = self.real_arm.get_servo_angle(is_radian=False)
                if code == 0 and angles and len(angles) >= 6:
                    self.joints_deg = list(angles)[:6]
                    self._update_gui()

                code_pos, pos = self.real_arm.get_position(is_radian=False)
                if code_pos == 0 and pos and len(pos) >= 6:
                    self.real_xyz = [pos[0], pos[1], pos[2]]
                    self.last_rpy = [pos[3], pos[4], pos[5]]

                time.sleep(0.033)
            except Exception: 
                time.sleep(0.5)

    def sync_with_real_robot(self):
        if not self.is_connected: return
        try:
            code, angles = self.real_arm.get_servo_angle(is_radian=False)
            if code == 0 and angles: self.joints_deg = list(angles)[:6]
            
            code, pos = self.real_arm.get_position(is_radian=False)
            if code == 0 and pos: 
                self.real_xyz = [pos[0], pos[1], pos[2]]
                self.last_rpy = [pos[3], pos[4], pos[5]]
            
            self._update_gui()
        except: pass

    # COMMANDs

    def set_servo_angle(self, angle, speed=None, mvacc=None, is_radian=False, wait=True):
        self._check_controls()
        
        if is_radian: target_deg = [math.degrees(a) for a in angle]
        else: target_deg = [float(a) for a in angle]
        
        safe_target = []
        for i, val in enumerate(target_deg):
            if i < len(config.JOINT_LIMITS):
                min_l, max_l = config.JOINT_LIMITS[i]
                safe_target.append(max(min(val, max_l), min_l))
            else: safe_target.append(val)
        
        if speed is None or speed <= 0: speed = 50 

        # Real Robot
        if self.is_connected:
            self.real_arm.set_servo_angle(angle=safe_target, speed=speed, mvacc=mvacc, is_radian=False, wait=False)
            if wait: self._wait_for_joints(safe_target)
            return 0

        # Simulator
        max_diff = max([abs(t - c) for t, c in zip(safe_target, self.joints_deg)])
        calc_duration = max_diff / float(speed)
        
        if wait: self._interpolated_move(safe_target, calc_duration)
        else:
            self.joints_deg = safe_target
            self._update_gui()
        return 0
    
    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, speed=None, silent=False, **kwargs):
        self._check_controls()
        
        if self.is_connected:
            cur_x, cur_y, cur_z = self.real_xyz
        else:
            cur_x, cur_y, cur_z = self._get_current_fk_position()
        
        if x is None: x = cur_x
        if y is None: y = cur_y
        if z is None: z = cur_z
        if roll is None: roll = self.last_rpy[0]
        if pitch is None: pitch = self.last_rpy[1]
        if yaw is None: yaw = self.last_rpy[2]
        
        self.last_rpy = [roll, pitch, yaw]

        if not silent: self._log(f"[MOVE] Line to: x={x:.0f} y={y:.0f} z={z:.0f}")

        wait = kwargs.pop('wait', True) 

        # Real Robot
        if self.is_connected:
            try:
                code = self.real_arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, 
                                           speed=speed, is_radian=False, wait=False, **kwargs)
                if code == 9: 
                    self._log("[REAL ERROR] Kinematic Error (Code 9)")
                    return -2
                
                if wait: self._wait_for_position([x, y, z])
            except: pass
            return 0
            
        # Simulator
        if self.chain is None: return -1
        
        start_pos = np.array([cur_x, cur_y, cur_z]) / 1000.0
        end_pos = np.array([x, y, z]) / 1000.0
        dist = np.linalg.norm(end_pos - start_pos)
        
        steps = int(dist / 0.005) 
        if steps < 5: steps = 5 
        
        if speed is None or speed <= 0: speed = 100
        duration = dist * 1000 / float(speed) 
        if duration < 0.1: duration = 0.1
        dt = duration / steps

        xs = np.linspace(start_pos[0], end_pos[0], steps)
        ys = np.linspace(start_pos[1], end_pos[1], steps)
        zs = np.linspace(start_pos[2], end_pos[2], steps)
        
        target_orient = rpy_to_matrix(roll, pitch, yaw)
        
        curr_rads = [0] + [math.radians(j) for j in self.joints_deg] + [0]
        if len(curr_rads) < len(self.chain.links): curr_rads += [0] * (len(self.chain.links) - len(curr_rads))
        curr_rads = curr_rads[:len(self.chain.links)]
        
        if not wait:
            final = self._solve_ik(end_pos, target_orient, curr_rads)
            if final:
                self.joints_deg = final
                self._update_gui()
            else:
                if not silent: self._log("[SIM IK FAIL] Unreachable")
        else:
            for i in range(steps):
                self._check_controls()
                waypoint = [xs[i], ys[i], zs[i]]
                new_j = self._solve_ik(waypoint, target_orient, curr_rads)
                if new_j:
                    self.joints_deg = new_j
                    self._update_gui()
                    curr_rads = [0] + [math.radians(j) for j in new_j] + [0]
                    if len(curr_rads) < len(self.chain.links): curr_rads += [0] * (len(self.chain.links) - len(curr_rads))
                    curr_rads = curr_rads[:len(self.chain.links)]
                time.sleep(dt)
        return 0

    # HELPERS
    def _wait_for_joints(self, target, tolerance=0.5):
        start = time.time()
        while self.is_connected:
            self._check_controls()
            diff = max([abs(a-b) for a, b in zip(self.joints_deg, target)])
            if diff < tolerance: break
            if time.time() - start > 15.0: break
            time.sleep(0.05)

    def _wait_for_position(self, target_xyz, tolerance=1.0):
        start = time.time()
        while self.is_connected:
            self._check_controls()
            cx, cy, cz = self.real_xyz
            dist = math.sqrt((cx - target_xyz[0])**2 + (cy - target_xyz[1])**2 + (cz - target_xyz[2])**2)
            if dist < tolerance: 
                time.sleep(0.1)
                break
            if time.time() - start > 15.0: break
            time.sleep(0.05)

    def _solve_ik(self, target_pos, target_orient, initial_rads):
        try:
            real_joints = self.chain.inverse_kinematics(
                target_position=target_pos,
                target_orientation=target_orient, 
                orientation_mode="all",
                initial_position=initial_rads
            )
            if hasattr(real_joints, 'any') and np.isnan(real_joints).any(): return None
            new_deg = []
            for i in range(1, 7):
                if i < len(real_joints): new_deg.append(math.degrees(real_joints[i]))
                else: new_deg.append(0.0)
            norm = normalize_angles(new_deg)
            for i, val in enumerate(norm):
                if i < len(config.JOINT_LIMITS):
                    min_l, max_l = config.JOINT_LIMITS[i]
                    if val < (min_l - 0.1) or val > (max_l + 0.1): return None 
            return norm
        except: return None

    def _interpolated_move(self, target_deg, duration):
        start_arr = np.array(self.joints_deg)
        end_arr = np.array(target_deg)
        eff_speed = max(0.01, self.speed_multiplier)
        real_dur = duration / eff_speed / config.SIM_SPEED_FACTOR
        if real_dur < 0.1: real_dur = 0.1
        steps = int(real_dur * 30)
        if steps < 1: steps = 1
        dt = real_dur / steps
        for i in range(1, steps + 1):
            self._check_controls()
            t = i / steps
            curr = start_arr + (end_arr - start_arr) * t
            self.joints_deg = normalize_angles(curr.tolist())
            self._update_gui()
            time.sleep(dt)
        self.joints_deg = normalize_angles(target_deg)
        self._update_gui()

    def _log(self, msg): self.ctx.log_queue.put(msg)
    def _update_gui(self): 
        try: self.ctx.joint_queue.put_nowait(list(self.joints_deg))
        except: pass
    def _check_controls(self):
        if self.ctx.stop_flag:
            if self.real_arm: self.real_arm.set_state(4)
            raise SystemExit("Stop")
        while self.ctx.paused:
            time.sleep(0.1)
            if self.ctx.stop_flag: 
                if self.real_arm: self.real_arm.set_state(4)
                raise SystemExit("Stop")
    def motion_enable(self, enable=True): 
        if self.real_arm: self.real_arm.motion_enable(enable=enable)
        return 0
    def set_mode(self, mode): 
        if self.real_arm: self.real_arm.set_mode(mode)
        return 0
    def set_state(self, state): 
        if self.real_arm: self.real_arm.set_state(state)
        return 0
    def clean_warn(self): 
        if self.real_arm: self.real_arm.clean_warn()
        return 0
    def clean_error(self): 
        if self.real_arm: self.real_arm.clean_error()
        return 0
    def disconnect(self): return 0
    
    def _get_current_fk_position(self):
        if self.chain is None: return 0, 0, 0
        current_rads = [0] + [math.radians(j) for j in self.joints_deg] + [0]
        if len(current_rads) < len(self.chain.links):
            current_rads += [0] * (len(self.chain.links) - len(current_rads))
        current_rads = current_rads[:len(self.chain.links)]
        matrix = self.chain.forward_kinematics(current_rads)
        return matrix[0, 3] * 1000.0, matrix[1, 3] * 1000.0, matrix[2, 3] * 1000.0
    
    def _emergency_home(self):
        self.joints_deg = [0.0] * 6
        self._update_gui()
        if self.is_connected: self.real_arm.set_servo_angle([0]*6, speed=30, wait=False)