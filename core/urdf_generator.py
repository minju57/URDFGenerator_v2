import numpy as np
import os

class URDFManager:
    def __init__(self, robot_name="generated_robot"):
        self.robot_name = robot_name

    def _deg_to_rad(self, deg):
        try: return float(deg) * (np.pi / 180.0)
        except: return 0.0

    def _get_axis_vector(self, axis_name):
        return {"Roll": "1 0 0", "Pitch": "0 1 0", "Yaw": "0 0 1"}.get(axis_name, "1 0 0")

    def _calc_cylinder_geometry(self, dx, dy, dz):
        # 시작점(0,0,0)에서 목표점(dx,dy,dz)까지 이어지는 실린더의 중심과 회전, 길이 계산
        cx, cy, cz = dx * 0.5, dy * 0.5, dz * 0.5
        ax, ay, az = abs(dx), abs(dy), abs(dz)
        roll, pitch, yaw = 0.0, 0.0, 0.0
        length, radius = 0.0, 0.0
        
        # 가장 긴 축을 기준으로 회전 및 길이 설정
        if ax >= ay and ax >= az:     
            pitch, length = 1.5708, ax # Y축 회전 -> X축 정렬
            radius = 0.5 * np.sqrt(dy**2 + dz**2)
        elif ay >= ax and ay >= az:   
            roll, length = 1.5708, ay # X축 회전 -> Y축 정렬
            radius = 0.5 * np.sqrt(dx**2 + dz**2)
        else:                         
            length = az # 기본 Z축 정렬
            radius = 0.5 * np.sqrt(dx**2 + dy**2)
            
        if length < 1e-6: length = 0.05
        if radius < 0.02: radius = 0.03 # 최소 두께 보장
        return (cx, cy, cz), (roll, pitch, yaw), (length, radius)

    def _calc_box_geometry(self, dx, dy, dz):
        cx, cy, cz = dx * 0.5, dy * 0.5, dz * 0.5
        lx = abs(dx) if abs(dx) > 1e-3 else 0.05
        ly = abs(dy) if abs(dy) > 1e-3 else 0.05
        lz = abs(dz) if abs(dz) > 1e-3 else 0.05
        return (cx, cy, cz), (lx, ly, lz)

    def generate(self, joints, filename="robot.urdf", base_joint=None, inertia_data=None, robot_name="generated_robot",is_imported=False):
        writer = URDFWriter()
        L0, L1, L2, L3, L4 = "", "  ", "    ", "      ", "        "

        # if base_joint is None:
        #     from logic.joint_logic import get_default_base_joint
        #     base_joint = get_default_base_joint()

        # [1] 트리 구조 분석
        child_map = {}
        for j in joints:
            p = j['parent']
            if p not in child_map: child_map[p] = []
            child_map[p].append(j)

        writer.xml_header()
        writer.start(L0, "robot", f'name="{robot_name}"')
        writer.mujoco_setting(L1)

        has_base = not is_imported and base_joint is not None and not base_joint.get('no_base')

        if has_base:
            base_link_name = base_joint.get('child', 'base_link') if base_joint is not None else 'base_link'

            mode = base_joint.get('mode', 'Fixed')
            bj_x = base_joint.get('x', 0.0) / 1000.0
            bj_y = base_joint.get('y', 0.0) / 1000.0
            bj_z = base_joint.get('z', 1000.0) / 1000.0
            bj_r = self._deg_to_rad(base_joint.get('r', 0.0))
            bj_p = self._deg_to_rad(base_joint.get('p', 0.0))
            bj_yaw = self._deg_to_rad(base_joint.get('yaw', 0.0))

            if mode == 'Fixed':
                writer.tag(L1, "link", 'name="world"')
                writer.start(L1, "joint", 'name="world_to_base" type="fixed"')
                writer.tag(L2, "parent", 'link="world"')
                writer.tag(L2, "child", f'link="{base_link_name}"')
                writer.tag(L2, "origin", f'xyz="{bj_x} {bj_y} {bj_z}" rpy="{bj_r:.4f} {bj_p:.4f} {bj_yaw:.4f}"')
                writer.end(L1, "joint")
            else:
                writer.start(L1, "link", 'name="base"')
                writer.start(L2, "visual")
                writer.tag(L3, "origin", 'rpy="0 0 0" xyz="0 0 0"')
                writer.start(L3, "geometry")
                writer.tag(L4, "box", 'size="0.001 0.001 0.001"')
                writer.end(L3, "geometry")
                writer.end(L2, "visual")
                writer.end(L1, "link")
                writer.start(L1, "joint", 'name="floating_base" type="floating"')
                writer.tag(L2, "origin", f'xyz="{bj_x} {bj_y} {bj_z}" rpy="{bj_r:.4f} {bj_p:.4f} {bj_yaw:.4f}"')
                writer.tag(L2, "parent", 'link="base"')
                writer.tag(L2, "child", f'link="{base_link_name}"')
                writer.end(L1, "joint")

            # --- Base Link ---
            writer.start(L1, "link", f'name="{base_link_name}"')

            base_vis_type = base_joint.get('vis_type', 'Auto (Cylinder)')

            if base_vis_type != "None":
                writer.start(L2, "visual")

                if base_vis_type == 'Auto (Cylinder)':
                    body_radius = 0.08
                    connected_joints = child_map.get(base_link_name, [])
                    if connected_joints:
                        j0 = connected_joints[0]
                        dist = np.sqrt(j0['x']**2 + j0['y']**2)
                        if dist > 0.05: body_radius = dist * 1.1
                    writer.tag(L3, "origin", 'xyz="0 0 -0.5" rpy="0 0 0"')
                    writer.start(L3, "geometry")
                    writer.tag(L4, "cylinder", f'radius="{body_radius:.4f}" length="1.0"')
                    writer.end(L3, "geometry")

                elif base_vis_type == 'Auto (Box)':
                    body_size = 0.16
                    writer.tag(L3, "origin", 'xyz="0 0 -0.5" rpy="0 0 0"')
                    writer.start(L3, "geometry")
                    writer.tag(L4, "box", f'size="{body_size} {body_size} 1.0"')
                    writer.end(L3, "geometry")

                elif base_vis_type == 'Mesh':
                    v_xyz = f"{base_joint.get('vis_x', 0)/1000.0} {base_joint.get('vis_y', 0)/1000.0} {base_joint.get('vis_z', 0)/1000.0}"
                    v_rpy = f"{self._deg_to_rad(base_joint.get('vis_roll', 0)):.4f} {self._deg_to_rad(base_joint.get('vis_pitch', 0)):.4f} {self._deg_to_rad(base_joint.get('vis_yaw', 0)):.4f}"
                    writer.tag(L3, "origin", f'xyz="{v_xyz}" rpy="{v_rpy}"')
                    writer.start(L3, "geometry")
                    writer.tag(L4, "mesh", f'filename="{base_joint.get("vis_mesh", "")}" scale="0.001 0.001 0.001"')
                    writer.end(L3, "geometry")

                elif base_vis_type.startswith('Manual'):
                    shape = base_vis_type.split('(')[1][:-1]
                    dims = [base_joint.get('vis_dim1', 50.0)/1000.0,
                            base_joint.get('vis_dim2', 100.0)/1000.0,
                            base_joint.get('vis_dim3', 50.0)/1000.0]
                    v_xyz = f"{base_joint.get('vis_x', 0)/1000.0} {base_joint.get('vis_y', 0)/1000.0} {base_joint.get('vis_z', 0)/1000.0}"
                    v_rpy = f"{self._deg_to_rad(base_joint.get('vis_roll', 0)):.4f} {self._deg_to_rad(base_joint.get('vis_pitch', 0)):.4f} {self._deg_to_rad(base_joint.get('vis_yaw', 0)):.4f}"
                    writer.tag(L3, "origin", f'xyz="{v_xyz}" rpy="{v_rpy}"')
                    writer.start(L3, "geometry")
                    if shape == "Box": writer.tag(L4, "box", f'size="{dims[0]} {dims[1]} {dims[2]}"')
                    elif shape == "Sphere": writer.tag(L4, "sphere", f'radius="{dims[0]}"')
                    else: writer.tag(L4, "cylinder", f'radius="{dims[0]}" length="{dims[1]}"')
                    writer.end(L3, "geometry")

                writer.material(L3, "base_grey", 0.2, 0.2, 0.2)
                writer.end(L2, "visual")

            # Base Link Collision
            if base_joint.get('col_enabled', False):
                writer.start(L2, "collision")
                c_xyz = f"{base_joint.get('col_x', 0)/1000.0} {base_joint.get('col_y', 0)/1000.0} {base_joint.get('col_z', 0)/1000.0}"
                c_rpy = f"{self._deg_to_rad(base_joint.get('col_roll', 0)):.4f} {self._deg_to_rad(base_joint.get('col_pitch', 0)):.4f} {self._deg_to_rad(base_joint.get('col_yaw', 0)):.4f}"
                writer.tag(L3, "origin", f'xyz="{c_xyz}" rpy="{c_rpy}"')
                writer.start(L3, "geometry")
                c_type = base_joint.get('col_type', 'Cylinder')
                c_dims = [base_joint.get('col_dim1', 50.0)/1000.0,
                          base_joint.get('col_dim2', 100.0)/1000.0,
                          base_joint.get('col_dim3', 50.0)/1000.0]
                if c_type == "Box": writer.tag(L4, "box", f'size="{c_dims[0]} {c_dims[1]} {c_dims[2]}"')
                elif c_type == "Sphere": writer.tag(L4, "sphere", f'radius="{c_dims[0]}"')
                else: writer.tag(L4, "cylinder", f'radius="{c_dims[0]}" length="{c_dims[1]}"')
                writer.end(L3, "geometry")
                writer.end(L2, "collision")

            # Base Inertia
            if inertia_data and len(inertia_data) > 0:
                d = inertia_data[0]
                writer.inertial(L2, mass=d.get('mass', d.get('m', 1.0)), xyz=(d.get('com_x', d.get('x', 0)), d.get('com_y', d.get('y', 0)), d.get('com_z', d.get('z', 0))),
                                ixx=d.get('ixx', 0.01), ixy=d.get('ixy', 0), ixz=d.get('ixz', 0),
                                iyy=d.get('iyy', 0.01), iyz=d.get('iyz', 0), izz=d.get('izz', 0.01))
            else:
                writer.inertial(L2, mass=10.0)
            writer.end(L1, "link")

        # --- Joints Loop ---
        for i, j in enumerate(joints):
            j_name = j.get('name', f"joint_{i}")
            p_link = j.get('parent', base_link_name if has_base else 'base_link')
            c_link = j.get('child', f"link_{i}")
            
            xyz = f"{j['x']} {j['y']} {j['z']}"
            rpy = f"{self._deg_to_rad(j['r']):.4f} {self._deg_to_rad(j['p']):.4f} {self._deg_to_rad(j['yaw']):.4f}"
            axis = self._get_axis_vector(j['axis'])
            low, up = self._deg_to_rad(j['low']), self._deg_to_rad(j['up'])
            j_type = j.get('type', 'revolute')

            # 1. Write Joint
            writer.start(L1, "joint", f'name="{j_name}" type="{j_type}"')
            writer.tag(L2, "parent", f'link="{p_link}"')
            writer.tag(L2, "child", f'link="{c_link}"')
            writer.tag(L2, "origin", f'xyz="{xyz}" rpy="{rpy}"')
            writer.tag(L2, "axis", f'xyz="{axis}"')
            writer.tag(L2, "limit", f'lower="{low:.4f}" upper="{up:.4f}" effort="10" velocity="1"')
            writer.end(L1, "joint")

            # 2. Write Child Link
            writer.start(L1, "link", f'name="{c_link}"')

            vis_mode = j.get('vis_type', 'Auto (Cylinder)')
            
            # --- Visual Geometry Logic ---
            
            # Case A: Mesh
            if vis_mode == 'Mesh':
                writer.start(L2, "visual")
                v_xyz = f"{j.get('vis_x', 0)} {j.get('vis_y', 0)} {j.get('vis_z', 0)}"
                v_rpy = f"{self._deg_to_rad(j.get('vis_roll', 0)):.4f} {self._deg_to_rad(j.get('vis_pitch', 0)):.4f} {self._deg_to_rad(j.get('vis_yaw', 0)):.4f}"
                writer.tag(L3, "origin", f'xyz="{v_xyz}" rpy="{v_rpy}"')
                writer.start(L3, "geometry")
                path = j.get('vis_mesh', "package://ur_description/meshes/default.stl")
                writer.tag(L4, "mesh", f'filename="{path}" scale="0.001 0.001 0.001"')
                writer.end(L3, "geometry")
                writer.material(L3, f"mat_{i}", 0.6, 0.6, 0.6)
                writer.end(L2, "visual")
            
            # Case B: Manual
            elif vis_mode.startswith('Manual'):
                writer.start(L2, "visual")
                shape = vis_mode.split('(')[1][:-1]
                v_xyz = f"{j.get('vis_x', 0)} {j.get('vis_y', 0)} {j.get('vis_z', 0)}"
                v_rpy = f"{self._deg_to_rad(j.get('vis_roll', 0)):.4f} {self._deg_to_rad(j.get('vis_pitch', 0)):.4f} {self._deg_to_rad(j.get('vis_yaw', 0)):.4f}"
                writer.tag(L3, "origin", f'xyz="{v_xyz}" rpy="{v_rpy}"')
                
                writer.start(L3, "geometry")
                if shape == "Box":
                        writer.tag(L4, "box", f'size="{j.get("vis_dim1", 0.1)} {j.get("vis_dim2", 0.1)} {j.get("vis_dim3", 0.1)}"')
                elif shape == "Sphere":
                        writer.tag(L4, "sphere", f'radius="{j.get("vis_dim1", 0.1)}"')
                else: # Cylinder
                        writer.tag(L4, "cylinder", f'radius="{j.get("vis_dim1", 0.05)}" length="{j.get("vis_dim2", 0.2)}"')
                writer.end(L3, "geometry")
                writer.material(L3, f"mat_{i}", 0.6, 0.6, 0.6)
                writer.end(L2, "visual")

            # Case C: Auto (Tree Visualization)
            else:
                next_joints = child_map.get(c_link, [])
                
                # C-1. 자식 조인트가 없는 경우 (Leaf Node) -> 끝단 표시용 아주 작은 빨간 구
                if not next_joints:
                    writer.start(L2, "visual")
                    writer.tag(L3, "origin", 'xyz="0 0 0" rpy="0 0 0"')
                    writer.start(L3, "geometry")
                    # [수정] 반지름을 0.02로 작게, 색상은 빨간색으로
                    writer.tag(L4, "sphere", 'radius="0.02"') 
                    writer.end(L3, "geometry")
                    
                    # [수정] Material Red
                    writer.start(L3, "material", f'name="end_effector_{i}"')
                    writer.tag(L4, "color", 'rgba="1.0 0.0 0.0 1.0"')
                    writer.end(L3, "material")
                    
                    writer.end(L2, "visual")
                
                # C-2. 자식 조인트가 있는 경우 -> 각 자식 조인트 위치까지 도형 생성
                else:
                    # [수정] 색상 생성 로직: 인덱스에 따라 다채로운 색상 부여
                    # 예: (0.2, 0.5, 0.8) 베이스에 인덱스로 변화를 줌
                    r = (i * 37 % 100) / 100.0
                    g = (i * 59 % 100) / 100.0
                    b = (i * 83 % 100) / 100.0
                    # 너무 어두우면 밝게 보정
                    if r+g+b < 1.0: r+=0.2; g+=0.2; b+=0.2
                    
                    for nj in next_joints:
                        dx, dy, dz = float(nj['x']), float(nj['y']), float(nj['z'])
                        
                        writer.start(L2, "visual") 
                        
                        if vis_mode == 'Auto (Box)':
                            (cx, cy, cz), (lx, ly, lz) = self._calc_box_geometry(dx, dy, dz)
                            writer.tag(L3, "origin", f'xyz="{cx:.4f} {cy:.4f} {cz:.4f}" rpy="0 0 0"')
                            writer.start(L3, "geometry")
                            writer.tag(L4, "box", f'size="{lx:.4f} {ly:.4f} {lz:.4f}"')
                            writer.end(L3, "geometry")
                        else: # Auto (Cylinder)
                            (cx, cy, cz), (rr, pp, yy), (ln, rad) = self._calc_cylinder_geometry(dx, dy, dz)
                            writer.tag(L3, "origin", f'xyz="{cx:.4f} {cy:.4f} {cz:.4f}" rpy="{rr:.4f} {pp:.4f} {yy:.4f}"')
                            writer.start(L3, "geometry")
                            writer.tag(L4, "cylinder", f'radius="{rad:.4f}" length="{ln:.4f}"')
                            writer.end(L3, "geometry")
                        
                        # 생성한 고유 색상 적용
                        writer.material(L3, f"mat_branch_{i}", r, g, b)
                        writer.end(L2, "visual")

            # --- Collision ---
            if j.get('col_enabled', False):
                writer.start(L2, "collision")
                c_xyz = f"{j.get('col_x', 0)} {j.get('col_y', 0)} {j.get('col_z', 0)}"
                c_rpy = f"{self._deg_to_rad(j.get('col_roll', 0)):.4f} {self._deg_to_rad(j.get('col_pitch', 0)):.4f} {self._deg_to_rad(j.get('col_yaw', 0)):.4f}"
                writer.tag(L3, "origin", f'xyz="{c_xyz}" rpy="{c_rpy}"')
                writer.start(L3, "geometry")
                c_type = j.get('col_type', 'Cylinder')
                if c_type == "Box":
                    writer.tag(L4, "box", f'size="{j.get("col_dim1",0.1)} {j.get("col_dim2",0.1)} {j.get("col_dim3",0.1)}"')
                elif c_type == "Sphere":
                    writer.tag(L4, "sphere", f'radius="{j.get("col_dim1", 0.1)}"')
                else: 
                    writer.tag(L4, "cylinder", f'radius="{j.get("col_dim1",0.05)}" length="{j.get("col_dim2",0.2)}"')
                writer.end(L3, "geometry")
                writer.end(L2, "collision")

            # --- Inertia ---
            data_idx = i + 1
            if inertia_data and len(inertia_data) > data_idx:
                d = inertia_data[data_idx]
                writer.inertial(L2, mass=d.get('mass', d.get('m', 1.0)), xyz=(d.get('com_x', d.get('x', 0)), d.get('com_y', d.get('y', 0)), d.get('com_z', d.get('z', 0))),
                                ixx=d.get('ixx', 0.01), ixy=d.get('ixy', 0), ixz=d.get('ixz', 0),
                                iyy=d.get('iyy', 0.01), iyz=d.get('iyz', 0), izz=d.get('izz', 0.01))
            else:
                writer.inertial(L2)

            writer.end(L1, "link")

        writer.end(L0, "robot")
        final_xml = "\n".join(writer.lines)
        with open(filename, "w", encoding="utf-8") as f: f.write(final_xml)
        return final_xml, os.path.abspath(filename)

class URDFWriter:
    def __init__(self): self.lines = []
    def _add(self, indent, text): self.lines.append(f"{indent}{text}")
    def comment(self, indent, text): self._add(indent, f"")
    def tag(self, indent, name, attrs=""): self._add(indent, f"<{name} {attrs}/>")
    def start(self, indent, name, attrs=""): self._add(indent, f"<{name} {attrs}>")
    def end(self, indent, name): self._add(indent, f"</{name}>")
    def xml_header(self): self.lines.append('<?xml version="1.0"?>')
    def mujoco_setting(self, indent):
        self.start(indent, "mujoco"); self.tag(indent+"  ", "compiler", 'discardvisual="false"'); self.end(indent, "mujoco")
    def material(self, indent, name, r, g, b, a=1.0):
        self.start(indent, "material", f'name="{name}"'); self.tag(indent+"  ", "color", f'rgba="{r} {g} {b} {a}"'); self.end(indent, "material")
    
    def inertial(self, indent, mass=1.0, xyz=(0,0,0), ixx=0.01, ixy=0, ixz=0, iyy=0.01, iyz=0, izz=0.01):
        self.start(indent, "inertial")
        self.tag(indent+"  ", "origin", f'xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="0 0 0"')
        self.tag(indent+"  ", "mass", f'value="{mass}"')
        self.tag(indent+"  ", "inertia", f'ixx="{ixx}" ixy="{ixy}" ixz="{ixz}" iyy="{iyy}" iyz="{iyz}" izz="{izz}"')
        self.end(indent, "inertial")