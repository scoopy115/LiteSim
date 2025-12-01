# robot_api.py
import time
import math
import sys
import numpy as np

try:
    from xarm.wrapper import XArmAPI as RealXArmAPI
    HAS_REAL_SDK = True
except ImportError:
    HAS_REAL_SDK = False

import config
from utils import normalize_angles, rpy_to_matrix

GLOBAL_API_INSTANCE = None

class SimXArmAPI:
    def __init__(self, ctx, chain):
        self.ctx = ctx
        self.chain = chain 
        self.joints_deg = [0.0] * config.JOINT_COUNT
        
        config.GLOBAL_API_INSTANCE = self
        
        self.speed_multiplier = 1.0
        self.last_rpy = [180, 0, 0]
        
        self.real_arm = None

    def _clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)
    
    @property
    def is_connected(self):
        if self.real_arm and HAS_REAL_SDK:
            return self.real_arm.connected # Use connected from xArm SDK
        return False

    def connect_real_robot(self, ip):
        if not ip or not ip.strip():
            return False, "IP address cannot be empty."
            
        if not HAS_REAL_SDK:
            return False, "xArm Python SDK is not installed."

        try:
            self._log(f"[REAL] Connecting to {ip}...")
            
            self.real_arm = RealXArmAPI(ip)
            
            if self.real_arm.connected:
                self.real_arm.clean_warn()
                self.real_arm.clean_error()
                self.real_arm.motion_enable(True)
                self.real_arm.set_mode(0)
                self.real_arm.set_state(0)
                self._log("[REAL] Connected successfully!")
                return True, "Connected"
            else:
                self.real_arm = None
                return False, "Connection timed out (Check IP/Cable)"
                
        except Exception as e:
            self.real_arm = None
            return False, f"Connection Error: {e}"

    def disconnect_real_robot(self):
        # GUI uses to disconnect real Lite 6
        if self.real_arm:
            try: 
                self.real_arm.disconnect()
            except: pass
            self.real_arm = None
            self._log("[REAL] Disconnected manually/on-exit.")

    # Ignoring disconnecting and connecting to robot because GUI will handle the connection/disconnection
    def disconnect(self): 
        self._log("[SCRIPT] Script requested disconnect -> IGNORED (Managed by GUI).")
        return 0

    def connect(self, *args, **kwargs):
        self._log("[SCRIPT] Script requested connect -> IGNORED (Managed by GUI).")
        return 0

    @property
    def version(self): return "sim-lite6-v12.0-scanner"
    @property
    def state(self): return 2
    @property
    def mode(self): return 0

    def _log(self, msg): self.ctx.log_queue.put(msg)
    
    def _update_gui(self): 
        self.ctx.joint_queue.put(list(self.joints_deg))

    def _check_controls(self):
        if self.ctx.stop_flag:
            if self.real_arm: self.real_arm.set_state(4)
            raise SystemExit("Script stopped by user.")
            
        while self.ctx.paused:
            time.sleep(0.1)
            if self.ctx.stop_flag: 
                if self.real_arm: self.real_arm.set_state(4)
                raise SystemExit("Script stopped during pause.")

    # --- API COMMANDS ---
    def motion_enable(self, enable=True): 
        self._check_controls()
        self._log(f"[SIM] Motion Enable: {enable}")
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
    def disconnect(self): 
        self._log("[SCRIPT] Disconnect requested (ignored by Sim).")
        return 0

    def _get_current_fk_position(self):
        if self.chain is None: return 0, 0, 0
        current_rads = [0] + [math.radians(j) for j in self.joints_deg] + [0]
        if len(current_rads) < len(self.chain.links):
            current_rads += [0] * (len(self.chain.links) - len(current_rads))
        matrix = self.chain.forward_kinematics(current_rads)
        return matrix[0, 3] * 1000.0, matrix[1, 3] * 1000.0, matrix[2, 3] * 1000.0

    def _interpolated_move(self, target_deg, duration):
        start_deg = list(self.joints_deg)
        diffs = [t - s for t, s in zip(target_deg, start_deg)]
        
        effective_speed = max(0.01, self.speed_multiplier)
        duration = duration / effective_speed
        duration = duration / config.SIM_SPEED_FACTOR

        if duration < 0.1: duration = 0.1
        dt = 0.03 
        steps = int(duration / dt)
        
        for i in range(1, steps + 1):
            self._check_controls()
            fraction = i / steps
            current_pos = [s + (d * fraction) for s, d in zip(start_deg, diffs)]
            self.joints_deg = normalize_angles(current_pos)
            self._update_gui()
            time.sleep(dt)

        self.joints_deg = normalize_angles(target_deg)
        self._update_gui()

    def set_servo_angle(self, angle, speed=None, mvacc=None, is_radian=False, wait=True):
        self._check_controls()
        
        if is_radian: 
            target_deg = [math.degrees(a) for a in angle]
        else: 
            target_deg = [float(a) for a in angle]
        
        # Clamp values to safe values for bot
        safe_target = []
        for i, val in enumerate(target_deg):
            if i < len(config.JOINT_LIMITS):
                min_l, max_l = config.JOINT_LIMITS[i]
                safe_val = self._clamp(val, min_l, max_l)
                safe_target.append(safe_val)
                
                if safe_val != val:
                    self._log(f"[WARN] Joint {i+1} limited: {val:.1f} -> {safe_val:.1f}")
            else:
                safe_target.append(val)
        
        final_target = safe_target 
    
        if speed is None or speed <= 0: speed = 50 

        if self.real_arm:
            self.real_arm.set_servo_angle(final_target, speed=speed, mvacc=mvacc, is_radian=False, wait=False)

        max_diff = max([abs(t - c) for t, c in zip(final_target, self.joints_deg)])
        calc_duration = max_diff / float(speed)
        
        if wait: 
            self._interpolated_move(final_target, calc_duration)
        else:
            self.joints_deg = final_target
            self._update_gui()
            
        return 0
    
    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, speed=None, silent=False, **kwargs):
        self._check_controls()
        
        cur_x, cur_y, cur_z = self._get_current_fk_position() # In mm
        
        if x is None: x = cur_x
        if y is None: y = cur_y
        if z is None: z = cur_z
        if roll is None: roll = self.last_rpy[0]
        if pitch is None: pitch = self.last_rpy[1]
        if yaw is None: yaw = self.last_rpy[2]
        
        self.last_rpy = [roll, pitch, yaw]

        if not silent:
            self._log(f"[MOVE] Line to: x={x:.0f} y={y:.0f} z={z:.0f}")

        wait = kwargs.get('wait', True)

        if self.is_connected:
            self.real_arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, 
                                       speed=speed, wait=wait, **kwargs)
            
        else:
            if self.chain is None: return -1
            
            start_pos = np.array([cur_x, cur_y, cur_z]) / 1000.0
            end_pos = np.array([x, y, z]) / 1000.0
            
            dist = np.linalg.norm(end_pos - start_pos)
            
            steps = int(dist / 0.005) 
            if steps < 5: steps = 5 
            
            if speed is None or speed <= 0: speed = 100
            duration = dist * 1000 / float(speed) # sec
            if duration < 0.1: duration = 0.1
            dt = duration / steps

            xs = np.linspace(start_pos[0], end_pos[0], steps)
            ys = np.linspace(start_pos[1], end_pos[1], steps)
            zs = np.linspace(start_pos[2], end_pos[2], steps)
            
            target_orient = rpy_to_matrix(roll, pitch, yaw)
            
            current_rads_full = [0] + [math.radians(j) for j in self.joints_deg] + [0]
            if len(current_rads_full) < len(self.chain.links):
                current_rads_full += [0] * (len(self.chain.links) - len(current_rads_full))
            
            if not wait:
                final_joints = self._solve_ik(end_pos, target_orient, current_rads_full)
                if final_joints:
                    self.joints_deg = final_joints
                    self._update_gui()
            else:
                for i in range(steps):
                    self._check_controls()
                    
                    waypoint = [xs[i], ys[i], zs[i]]
                    
                    new_joints = self._solve_ik(waypoint, target_orient, current_rads_full)
                    
                    if new_joints:
                        self.joints_deg = new_joints
                        self._update_gui()
                        
                        current_rads_full = [0] + [math.radians(j) for j in new_joints] + [0]
                        if len(current_rads_full) < len(self.chain.links):
                            current_rads_full += [0] * (len(self.chain.links) - len(current_rads_full))

                    time.sleep(dt)
                    
        return 0

    def _solve_ik(self, target_pos, target_orient, initial_rads):
        try:
            real_joints = self.chain.inverse_kinematics(
                target_position=target_pos,
                target_orientation=target_orient, 
                orientation_mode="all",
                initial_position=initial_rads[:len(self.chain.links)]
            )
            
            if hasattr(real_joints, 'any') and np.isnan(real_joints).any():
                return None

            new_degrees = []
            for i in range(1, 7):
                if i < len(real_joints): new_degrees.append(math.degrees(real_joints[i]))
                else: new_degrees.append(0.0)
            
            return normalize_angles(new_degrees)
        except:
            return None