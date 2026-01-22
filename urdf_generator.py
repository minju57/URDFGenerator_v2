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

    # [Logic] Cylinder Geometry 계산 (다음 조인트까지 연결)
    def _calc_cylinder_geometry(self, dx, dy, dz):
        cx, cy, cz = dx * 0.5, dy * 0.5, dz * 0.5
        ax, ay, az = abs(dx), abs(dy), abs(dz)
        roll, pitch, yaw = 0.0, 0.0, 0.0
        length, radius = 0.0, 0.0

        if ax >= ay and ax >= az:     # X축
            pitch, length = 1.5708, ax
            radius = 0.5 * np.sqrt(dy**2 + dz**2)
        elif ay >= ax and ay >= az:   # Y축
            roll, length = 1.5708, ay
            radius = 0.5 * np.sqrt(dx**2 + dz**2)
        else:                         # Z축
            length = az
            radius = 0.5 * np.sqrt(dx**2 + dy**2)

        if length < 1e-6: length = 0.05
        if radius < 0.03: radius = 0.03 
        return (cx, cy, cz), (roll, pitch, yaw), (length, radius)

    # [Logic] Box Geometry 계산 (다음 조인트까지 연결)
    def _calc_box_geometry(self, dx, dy, dz):
        cx, cy, cz = dx * 0.5, dy * 0.5, dz * 0.5
        # 두께 최소값 보정
        lx = abs(dx) if abs(dx) > 1e-3 else 0.05
        ly = abs(dy) if abs(dy) > 1e-3 else 0.05
        lz = abs(dz) if abs(dz) > 1e-3 else 0.05
        return (cx, cy, cz), (lx, ly, lz)

    def generate(self, joints, filename="robot.urdf"):
        writer = URDFWriter()
        L0, L1, L2, L3, L4 = "", "  ", "    ", "      ", "        "

        # 1. XML Header
        writer.xml_header()
        writer.start(L0, "robot", f'name="{self.robot_name}"')
        writer.mujoco_setting(L1)

        # 2. World & Base Joint
        writer.tag(L1, "link", 'name="world"')
        writer.start(L1, "joint", 'name="world_to_body" type="fixed"')
        writer.tag(L2, "parent", 'link="world"')
        writer.tag(L2, "child", 'link="body"')
        writer.tag(L2, "origin", 'xyz="0 0 1.0" rpy="0 0 0"') # 바디 프레임은 바닥에서 1m 위
        writer.end(L1, "joint")

        # ---------------------------------------------------------
        # [핵심 로직] Body Radius 자동 계산 (C++ 코드 반영)
        # ---------------------------------------------------------
        body_radius = 0.08
        if len(joints) > 0:
            first_x = float(joints[0]['x'])
            first_y = float(joints[0]['y'])
            # C++: bodyRadius = sqrt(firstX*firstX + firstY*firstY)
            body_radius = np.sqrt(first_x**2 + first_y**2)

            if body_radius < 0.05:
                body_radius = 0.08
            else:
                body_radius *= 1.1

        # 3. Body Link 생성
        writer.start(L1, "link", 'name="body"')
        writer.start(L2, "visual")
        # C++: origin xyz="0 0 -0.5" -> 바디의 Top(1.0)이 기준점이 됨
        writer.tag(L3, "origin", 'xyz="0 0 -0.5" rpy="0 0 0"') 
        writer.start(L3, "geometry")
        writer.tag(L4, "cylinder", f'radius="{body_radius:.4f}" length="1.0"')
        writer.end(L3, "geometry")
        writer.material(L3, "grey", 0.5, 0.5, 0.5)
        writer.end(L2, "visual")
        writer.inertial(L2, mass=10.0)
        writer.end(L1, "link")

        # 4. Joints & Links Loop
        for i, j in enumerate(joints):
            parent = "body" if i == 0 else f"link_{i-1}"
            child = f"link_{i}"
            
            # --- Joint Definition ---
            xyz = f"{j['x']} {j['y']} {j['z']}"
            rpy = f"{self._deg_to_rad(j['r']):.4f} {self._deg_to_rad(j['p']):.4f} {self._deg_to_rad(j['yaw']):.4f}"
            axis = self._get_axis_vector(j['axis'])
            low, up = self._deg_to_rad(j['low']), self._deg_to_rad(j['up'])

            writer.start(L1, "joint", f'name="joint_{i}" type="revolute"')
            writer.tag(L2, "parent", f'link="{parent}"')
            writer.tag(L2, "child", f'link="{child}"')
            writer.tag(L2, "origin", f'xyz="{xyz}" rpy="{rpy}"')
            writer.tag(L2, "axis", f'xyz="{axis}"')
            writer.tag(L2, "limit", f'lower="{low:.4f}" upper="{up:.4f}" effort="10" velocity="1"')
            writer.end(L1, "joint")

            # --- Link Definition ---
            writer.start(L1, "link", f'name="{child}"')

            # 마지막 링크가 아닐 때만 Visual 생성 (다음 조인트까지 연결)
            is_last_link = (i == len(joints) - 1)
            
            if not is_last_link:
                next_j = joints[i+1]
                dx, dy, dz = float(next_j['x']), float(next_j['y']), float(next_j['z'])
                vis_mode = j.get('vis_type', 'Auto (Cylinder)')

                writer.start(L2, "visual")
                
                # A. Mesh 모드
                if vis_mode == 'Mesh':
                    writer.tag(L3, "origin", 'xyz="0 0 0" rpy="0 0 0"')
                    writer.start(L3, "geometry")
                    path = j.get('vis_mesh', "package://ur_description/meshes/default.stl")
                    writer.tag(L4, "mesh", f'filename="{path}"')
                    writer.end(L3, "geometry")

                # B. Auto (Box) 모드
                elif vis_mode == 'Auto (Box)':
                    (cx, cy, cz), (lx, ly, lz) = self._calc_box_geometry(dx, dy, dz)
                    writer.tag(L3, "origin", f'xyz="{cx:.4f} {cy:.4f} {cz:.4f}" rpy="0 0 0"')
                    writer.start(L3, "geometry")
                    writer.tag(L4, "box", f'size="{lx:.4f} {ly:.4f} {lz:.4f}"')
                    writer.end(L3, "geometry")

                # C. Auto (Cylinder) 모드 (기본)
                else:
                    (cx, cy, cz), (rr, pp, yy), (ln, rad) = self._calc_cylinder_geometry(dx, dy, dz)
                    writer.tag(L3, "origin", f'xyz="{cx:.4f} {cy:.4f} {cz:.4f}" rpy="{rr:.4f} {pp:.4f} {yy:.4f}"')
                    writer.start(L3, "geometry")
                    writer.tag(L4, "cylinder", f'radius="{rad:.4f}" length="{ln:.4f}"')
                    writer.end(L3, "geometry")

                # Material
                mat_color = "1 0 0 1" if i % 2 == 0 else "0 0 1 1"
                writer.material(L3, f"mat_{i}", *map(float, mat_color.split()))
                writer.end(L2, "visual")

            # Collision (Optional)
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
    def inertial(self, indent, mass=1.0):
        self.start(indent, "inertial"); self.tag(indent+"  ", "mass", f'value="{mass}"'); self.tag(indent+"  ", "inertia", 'ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"'); self.end(indent, "inertial")