# visualizer.py
import os
import math
import numpy as np
import traceback
import xml.etree.ElementTree as ET
import config

try:
    import pyvista as pv
    # PyVista settings
    pv.global_theme.allow_empty_mesh = True
    from ikpy.chain import Chain
except ImportError as e:
    print(f"CRITICAL: Module missing in Visualizer: {e}")

class RobotVisualizer:
    def __init__(self):
        self.current_joints = [0.0] * config.JOINT_COUNT
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.chain = None
        self.link_map = [] 
        self.plotter = None
        self.ee_actor = None

        self.trace_enabled = False
        self.trace_points = []    
        self.trace_actor = None   
        self.trace_color = config.COLOR_PATH
        self.last_trace_pos = None
        self.trace_source = 'wrist' 
        self.eef_offset_z = 0.0     
        self.is_in_collision_state = False
    
    def get_urdf_path(self):
        if not os.path.exists(config.MODEL_DIR):
            print(f"[ERR] Model directory not found: {config.MODEL_DIR}")
            return None
        for f in os.listdir(config.MODEL_DIR):
            if f.lower().endswith(".urdf"):
                return os.path.join(config.MODEL_DIR, f)
        return None

    def get_mesh_path(self, filename):
        if not filename: return None
        return os.path.join(config.VISUAL_DIR, filename)

    def get_urdf_root_link_name(self, urdf_path):
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            links = [link.attrib['name'] for link in root.findall('link')]
            if links: return links[0]
        except Exception: pass
        return "link_base"

    def setup_scene(self):
        self.plotter = pv.Plotter(window_size=[config.WINDOW_WIDTH, config.WINDOW_HEIGHT], 
                                  title=f"{config.APP_NAME} {config.APP_VERSION} | UFACTORY Lite 6 Simulator | 3D View")
        self.plotter.set_background(config.COLOR_BG)
        self.plotter.enable_lightkit()
        urdf_path = self.get_urdf_path()
        if not urdf_path:
            print("URDF not found!")
            return None
        
        print(f"[URDF] Loading: {os.path.basename(urdf_path)}")
        root_name = self.get_urdf_root_link_name(urdf_path)
        try:
            self.chain = Chain.from_urdf_file(urdf_path, base_elements=[root_name])
            mask = [False] + [True] * config.JOINT_COUNT + [False]
            if len(self.chain.links) != len(mask):
                mask = [False] + [True] * config.JOINT_COUNT
                if len(self.chain.links) > len(mask):
                    mask += [False] * (len(self.chain.links) - len(mask))
            self.chain.active_links_mask = mask
        except: return None

        # Base colors for the visualizer
        colors = [config.COLOR_BASE] * 6 + [config.COLOR_WRIST, config.COLOR_EEF]

        print("-" * 30)
        for i, link in enumerate(self.chain.links):
            lname = link.name.lower()
            expected_stl = None
            is_end_effector = False
            
            if i == 0 or "base" in lname: expected_stl = "base.stl"
            elif "link1" in lname or "l1" in lname or "joint1" in lname: expected_stl = "link1.stl"
            elif "link2" in lname or "l2" in lname or "joint2" in lname: expected_stl = "link2.stl"
            elif "link3" in lname or "l3" in lname or "joint3" in lname: expected_stl = "link3.stl"
            elif "link4" in lname or "l4" in lname or "joint4" in lname: expected_stl = "link4.stl"
            elif "link5" in lname or "l5" in lname or "joint5" in lname: expected_stl = "link5.stl"
            elif "link6" in lname or "l6" in lname or "joint6" in lname: expected_stl = "link6.stl"
            elif "eef" in lname or "flange" in lname:
                is_end_effector = True

            print(f"Link {i} ('{link.name}') -> Mapped to: {expected_stl}")

            mesh = None

            if is_end_effector:
                mesh = pv.PolyData() # Empty 3D object
            elif expected_stl:
                stl_path = self.get_mesh_path(expected_stl)
                if os.path.exists(stl_path):
                    try: 
                        mesh = pv.read(stl_path)
                        if mesh.n_points > 0:
                            mesh = mesh.compute_normals(cell_normals=False, point_normals=True, split_vertices=True, feature_angle=30.0)
                    except: pass
                else:
                    print(f"   [!] NOT FOUND: {expected_stl} in {config.VISUAL_DIR}")

            if mesh is None:
                self.link_map.append(None)
                continue

            color = colors[i % len(colors)]
            if i == 0: color = colors[0]

            actor = self.plotter.add_mesh(mesh, color=color, smooth_shading=True, specular=0.2, 
                                          pbr=False, metallic=0.3, roughness=0.6)
            self.link_map.append(actor)
            
            if is_end_effector:
                self.ee_actor = actor
        print("-" * 30)
        
        floor = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), i_size=1, j_size=1, i_resolution=20, j_resolution=20)
        self.plotter.add_mesh(floor, color='#333333', show_edges=True, opacity=0.5, line_width=1)
        
        self.plotter.add_axes()
        self.plotter.view_isometric()
        self.plotter.enable_anti_aliasing()
        
        self.plotter.show(interactive_update=True, auto_close=False)
        return self.chain
    
    def set_custom_gripper(self, stl_path, scale_to_meters=False):
        if not self.ee_actor:
            print("[GUI] Found no end-effector actor to replace.")
            return False
        
        if not os.path.exists(stl_path):
            return False

        try:
            new_mesh = pv.read(stl_path)
            
            # Model scaling
            if scale_to_meters:
                print("[GUI] Scaling from mm to meters (x0.001)")
                new_mesh.scale([0.001, 0.001, 0.001], inplace=True)

            if new_mesh.n_points > 0:
                new_mesh = new_mesh.compute_normals(cell_normals=False, point_normals=True, split_vertices=True, feature_angle=30.0)
            
            # Calculate effector height
            self.eef_offset_z = new_mesh.bounds[5] 
            print(f"[GUI] EEF Length calculated: {self.eef_offset_z:.4f}m")

            # Replace data in current actor
            self.ee_actor.mapper.dataset = new_mesh
            
            print(f"[GUI] Gripper replaced by: {os.path.basename(stl_path)}")
            return True
        except Exception as e:
            print(f"[GUI] Error when loading custom STL: {e}")
            traceback.print_exc()
            return False
        
    def remove_gripper(self):
        if not self.ee_actor:
            return False
        
        try:
            empty_mesh = pv.PolyData()
            self.ee_actor.mapper.dataset = empty_mesh
            return True
        except Exception as e:
            print(f"[GUI] Error removing gripper: {e}")
            return False

    def update_joints(self, joints):
        self.current_joints = joints

    def reset_camera_view(self):
        if self.plotter:
            self.plotter.view_isometric()
            self.plotter.reset_camera()
            self.plotter.camera.zoom(config.INITIAL_ZOOM_LEVEL)

    def set_camera_view(self, view_type, zoom_level):
        if not self.plotter: return

        if view_type == 'front':
            self.plotter.view_yz()                 
        elif view_type == 'side-l':
            self.plotter.view_xz()                  
        elif view_type == 'side-r':
            self.plotter.view_xz(negative=True)     
        elif view_type == 'rear':
            self.plotter.view_yz(negative=True)     
        elif view_type == 'top':
            self.plotter.view_yx(negative=True)
        elif view_type == 'iso':
            self.plotter.view_isometric()
        
        self.plotter.reset_camera()
        
        self.plotter.camera.zoom(zoom_level)
        self.plotter.render()

    def render_frame(self):
        if self.plotter is None or self.chain is None: return False
        if not hasattr(self.plotter, 'ren_win') or self.plotter.ren_win is None: return False

        try:
            target_vector = [0.0] * len(self.chain.links)
            joint_idx = 0
            for i, is_active in enumerate(self.chain.active_links_mask):
                if is_active and joint_idx < len(self.current_joints):
                    deg = self.current_joints[joint_idx]
                    target_vector[i] = math.radians(deg)
                    joint_idx += 1
            
            matrices = self.chain.forward_kinematics(target_vector, full_kinematics=True)
            current_ee_pos = None
            
            current_collision = False
            COLLISION_THRESHOLD = 0.001 

            for i, matrix in enumerate(matrices):
                if i == len(matrices) - 1:
                    wrist_x = matrix[0, 3]
                    wrist_y = matrix[1, 3]
                    wrist_z = matrix[2, 3] + config.ROBOT_Z_OFFSET

                    if wrist_z < COLLISION_THRESHOLD: current_collision = True

                    if 'tip' in self.trace_source.lower() and hasattr(self, 'eef_offset_z') and self.eef_offset_z > 0:
                        rot_matrix = matrix[:3, :3]
                        local_offset = np.array([0.0, 0.0, self.eef_offset_z])
                        world_offset = rot_matrix @ local_offset
                        current_ee_pos = [wrist_x + world_offset[0], wrist_y + world_offset[1], wrist_z + world_offset[2]]
                        
                        if current_ee_pos[2] < COLLISION_THRESHOLD: current_collision = True
                    else:
                        current_ee_pos = [wrist_x, wrist_y, wrist_z]

                if i < len(self.link_map):
                    actor = self.link_map[i]
                    if actor is not None:
                        mat_copy = matrix.copy()
                        mat_copy[2, 3] += config.ROBOT_Z_OFFSET
                        actor.user_matrix = mat_copy 
            
            if current_collision:
                if not self.is_in_collision_state:
                    self.set_color("arm", config.COLOR_COLLISION)
                    self.set_color("wrist", config.COLOR_COLLISION)
                    self.set_color("eef", config.COLOR_COLLISION)
                    self.is_in_collision_state = True
                
                self.plotter.render()
                return True 

            else:
                if self.is_in_collision_state:
                    self.is_in_collision_state = False

            if self.trace_enabled and current_ee_pos:
                should_add = False
                if self.last_trace_pos is None: should_add = True
                else:
                    dist = math.sqrt(sum([(a - b) ** 2 for a, b in zip(current_ee_pos, self.last_trace_pos)]))
                    if dist > 0.001: should_add = True
                if should_add:
                    self.trace_points.append(current_ee_pos)
                    self.last_trace_pos = current_ee_pos
                    if len(self.trace_points) > 1:
                        if self.trace_actor: self.plotter.remove_actor(self.trace_actor)
                        points_array = np.array(self.trace_points)
                        line_mesh = pv.lines_from_points(points_array)
                        self.trace_actor = self.plotter.add_mesh(line_mesh, color=self.trace_color, line_width=4, reset_camera=False)
            
            self.plotter.render() 
            return False

        except Exception: return False

    def set_color(self, target, color_hex):
        if not self.plotter: return False

        if len(color_hex) == 6 and all(c in '0123456789ABCDEFabcdef' for c in color_hex):
            color_hex = f"#{color_hex}"

        try:
            if target == 'bg': self.plotter.set_background(color_hex)
            elif target == 'arm':
                for i in range(6): 
                    if i < len(self.link_map) and self.link_map[i]: self.link_map[i].prop.color = color_hex
            elif target == 'wrist':
                if len(self.link_map) > 6 and self.link_map[6]: self.link_map[6].prop.color = color_hex
            elif target == 'eef':
                if self.ee_actor: self.ee_actor.prop.color = color_hex
            elif target == 'trace':
                self.trace_color = color_hex 
                if self.trace_actor: self.trace_actor.prop.color = color_hex

            if target != 'bg': self.plotter.render()
            return True
        except: return False
        
    def set_trace_enable(self, enable):
        self.trace_enabled = enable
        if not enable:
            self.clear_trace()
            pass

    def clear_trace(self):
        self.trace_points = []
        self.last_trace_pos = None
        if self.trace_actor:
            self.plotter.remove_actor(self.trace_actor)
            self.trace_actor = None

    def set_ghost_mode(self, enabled, keep_gripper_visible):
        if not self.plotter: return

        if not enabled:
            # Ghost mode off
            opacity_arm = 1.0
            opacity_eef = 1.0
        else:
            # Ghost mode on
            opacity_arm = 0.1
            # Gripper ignore
            opacity_eef = 1.0 if keep_gripper_visible else 0.1

        try:
            for actor in self.link_map:
                if actor:
                    actor.prop.opacity = opacity_arm
            
            if self.ee_actor:
                self.ee_actor.prop.opacity = opacity_eef
            
            self.plotter.render()
        except Exception as e:
            print(f"[VISUALIZER] Ghost mode error: {e}")