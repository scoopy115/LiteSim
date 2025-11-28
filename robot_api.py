# robot_api.py
import time
import math
import sys

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

    def is_connected(self):
        if self.real_arm and HAS_REAL_SDK:
            return self.real_arm.connected # Use connected from xArm SDK
        return False

    def connect_real_robot(self, ip):
        # GUI uses to connect real Lite 6
        if not ip or not ip.strip() or "xxx" in ip:
            return False
            
        if not HAS_REAL_SDK:
            self._log("[WARN] xarm-python-sdk missing.")
            return False

        try:
            self._log(f"[REAL] Connecting to {ip}...")
            # Connect
            self.real_arm = RealXArmAPI(ip)
            
            # Check if connected
            if self.real_arm.connected:
                self.real_arm.clean_warn()
                self.real_arm.clean_error()
                self.real_arm.motion_enable(True)
                self.real_arm.set_mode(0)
                self.real_arm.set_state(0)
                self._log("[REAL] Connected successfully!")
                return True
            else:
                self._log("[REAL] Connection failed (SDK returned False).")
                self.real_arm = None
                return False
                
        except Exception as e:
            self._log(f"[REAL ERROR] {e}")
            self.real_arm = None
            return False

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
        
        if is_radian: target = [math.degrees(a) for a in angle]
        else: target = [float(a) for a in angle]
        target = normalize_angles(target)
        if speed is None or speed <= 0: speed = 50 

        if self.real_arm:
            self.real_arm.set_servo_angle(angle, speed=speed, mvacc=mvacc, is_radian=is_radian, wait=False)

        max_diff = max([abs(t - c) for t, c in zip(target, self.joints_deg)])
        calc_duration = max_diff / float(speed)
        
        if wait: self._interpolated_move(target, calc_duration)
        else:
            self.joints_deg = target
            self._update_gui()
        return 0
    
    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, speed=None, **kwargs):
        self._check_controls()
        if self.chain is None: return -1

        cur_x, cur_y, cur_z = self._get_current_fk_position()
        if x is None: x = cur_x
        if y is None: y = cur_y
        if z is None: z = cur_z
        if roll is None: roll = self.last_rpy[0]
        if pitch is None: pitch = self.last_rpy[1]
        if yaw is None: yaw = self.last_rpy[2]
        self.last_rpy = [roll, pitch, yaw]

        self._log(f"[MOVE] x={x:.0f} y={y:.0f} z={z:.0f}")

        if self.real_arm:
            self.real_arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, 
                                       speed=speed, wait=False, **kwargs)

        current_rads = [0] + [math.radians(j) for j in self.joints_deg] + [0]
        if len(current_rads) < len(self.chain.links):
            current_rads += [0] * (len(self.chain.links) - len(current_rads))

        target_pos = [x/1000.0, y/1000.0, z/1000.0]
        target_orient = rpy_to_matrix(roll, pitch, yaw)

        try:
            real_joints = self.chain.inverse_kinematics(
                target_position=target_pos,
                target_orientation=target_orient, 
                orientation_mode="all",
                initial_position=current_rads[:len(self.chain.links)]
            )
            
            new_degrees = []
            for i in range(1, 7):
                if i < len(real_joints): new_degrees.append(math.degrees(real_joints[i]))
                else: new_degrees.append(0.0)
            new_degrees = normalize_angles(new_degrees)
            
            if speed is None: speed = 100 
            dist = math.sqrt((x - cur_x)**2 + (y - cur_y)**2 + (z - cur_z)**2)
            estimated_duration = dist / float(speed)
            if estimated_duration < 0.1: estimated_duration = 0.2

            wait = kwargs.get('wait', True)
            if wait:
                self._interpolated_move(new_degrees, estimated_duration)
            else:
                self.joints_deg = new_degrees
                self._update_gui()

        except Exception as e:
            self._log(f"[IK ERROR] {e}")
            return 1
        return 0