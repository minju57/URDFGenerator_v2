import xml.etree.ElementTree as ET
import numpy as np
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class URDFImporter:
    def __init__(self):
        pass

    def _parse_origin(self, elem):
        """XML 요소에서 xyz, rpy 추출 (없으면 0.0)"""
        if elem is None:
            return [0.0]*3, [0.0]*3
        
        origin = elem.find('origin')
        if origin is None:
            return [0.0]*3, [0.0]*3
            
        xyz_str = origin.get('xyz', '0 0 0').split()
        rpy_str = origin.get('rpy', '0 0 0').split()
        
        xyz = [float(x) for x in xyz_str]
        rpy = [float(r) for r in rpy_str]
        return xyz, rpy

    def _rad_to_deg(self, rad):
        return float(rad) * (180.0 / np.pi)

    def _parse_geometry(self, geom_elem):
        """Geometry 태그 분석하여 타입과 치수 반환"""
        if geom_elem is None:
            return 'None', [0,0,0], ''
        
        # Mesh
        mesh = geom_elem.find('mesh')
        if mesh is not None:
            filename = mesh.get('filename', '')
            return 'Mesh', [0,0,0], filename

        # Primitives
        box = geom_elem.find('box')
        if box is not None:
            s = [float(v) for v in box.get('size', '0.1 0.1 0.1').split()]
            # m -> mm 변환
            return 'Box', [s[0]*1000, s[1]*1000, s[2]*1000], ''
            
        cylinder = geom_elem.find('cylinder')
        if cylinder is not None:
            r = float(cylinder.get('radius', '0.05')) * 1000
            l = float(cylinder.get('length', '0.1')) * 1000
            return 'Cylinder', [r, l, 0], ''
            
        sphere = geom_elem.find('sphere')
        if sphere is not None:
            r = float(sphere.get('radius', '0.05')) * 1000
            return 'Sphere', [r, 0, 0], ''
            
        return 'None', [0,0,0], ''

    def parse(self, file_path):
        """URDF 파일을 읽어 (Joint리스트, Base정보) 튜플 반환"""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
        except Exception as e:
            print(f"Error parsing URDF: {e}")
            return [], None

        # 1. 모든 링크와 조인트 수집
        links_dict = {}
        for link in root.findall('link'):
            links_dict[link.get('name')] = link

        joints_list = root.findall('joint')
        
        # 2. Base Link 찾기 (어떤 조인트의 child도 아닌 링크)
        all_link_names = set(links_dict.keys())
        child_link_names = set()
        
        joints_data = []
        base_joint_origin_xyz = [0.0, 0.0, 1.0]
        base_joint_origin_rpy = [0.0, 0.0, 0.0]
        base_mode = 'Fixed'
        found_base_joint = False

        # base joint (world_to_base 또는 floating_base) 파싱
        for joint in joints_list:
            jname = joint.get('name', '')
            if jname in ('world_to_base', 'floating_base'):
                found_base_joint = True
                base_mode = 'Fixed' if jname == 'world_to_base' else 'Floating'
                origin = joint.find('origin')
                if origin is not None:
                    base_joint_origin_xyz = [float(v) for v in origin.get('xyz', '0 0 1').split()]
                    base_joint_origin_rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
                break

        for joint in joints_list:
            # child 등록
            c_name = joint.find('child').get('link')
            child_link_names.add(c_name)

            # --- 기존 조인트 파싱 로직 (그대로 유지) ---
            if joint.get('name') in ('world_to_base', 'floating_base'): continue

            j_type = joint.get('type', 'revolute')
            data = {
                'name': joint.get('name'),
                'parent': joint.find('parent').get('link'),
                'child': c_name,
                'type': j_type,
                'axis': 'Roll',
                'x':0, 'y':0, 'z':0, 'r':0, 'p':0, 'yaw':0,
                'low': -180.0, 'up': 180.0,
                'vis_type': 'Auto (Cylinder)', 'vis_mesh': '',
                'vis_dim1':0, 'vis_dim2':0, 'vis_dim3':0,
                'vis_x':0, 'vis_y':0, 'vis_z':0, 'vis_roll':0, 'vis_pitch':0, 'vis_yaw':0,
                'col_enabled': False, 'col_type': 'Cylinder',
                'col_dim1':0, 'col_dim2':0, 'col_dim3':0,
                'col_x':0, 'col_y':0, 'col_z':0, 'col_roll':0, 'col_pitch':0, 'col_yaw':0
            }

            # Origin
            xyz, rpy = self._parse_origin(joint)
            data['x'], data['y'], data['z'] = [v*1000 for v in xyz]
            data['r'], data['p'], data['yaw'] = [self._rad_to_deg(v) for v in rpy]

            # Axis
            axis_elem = joint.find('axis')
            if j_type == 'fixed': data['axis'] = 'Fixed'
            elif axis_elem is not None:
                ax_vec = axis_elem.get('xyz', '1 0 0').split()
                if abs(float(ax_vec[1])) > 0.9: data['axis'] = 'Pitch'
                elif abs(float(ax_vec[2])) > 0.9: data['axis'] = 'Yaw'
                else: data['axis'] = 'Roll'

            # Limits
            limit = joint.find('limit')
            if limit is not None:
                if limit.get('lower'): data['low'] = self._rad_to_deg(float(limit.get('lower')))
                if limit.get('upper'): data['up'] = self._rad_to_deg(float(limit.get('upper')))

            # Visual/Collision (Child Link)
            if c_name in links_dict:
                link_obj = links_dict[c_name]
                # Visual
                vis = link_obj.find('visual')
                if vis is not None:
                    v_xyz, v_rpy = self._parse_origin(vis)
                    data['vis_x'], data['vis_y'], data['vis_z'] = [v*1000 for v in v_xyz]
                    data['vis_roll'], data['vis_pitch'], data['vis_yaw'] = [self._rad_to_deg(v) for v in v_rpy]
                    
                    g_type, dims, path = self._parse_geometry(vis.find('geometry'))
                    if g_type == 'Mesh':
                        data['vis_type'] = 'Mesh'; data['vis_mesh'] = path
                    elif g_type != 'None':
                        data['vis_type'] = f"Manual ({g_type})"
                        data['vis_dim1'], data['vis_dim2'], data['vis_dim3'] = dims
                
                # Collision
                col = link_obj.find('collision')
                if col is not None:
                    data['col_enabled'] = True
                    c_xyz, c_rpy = self._parse_origin(col)
                    data['col_x'], data['col_y'], data['col_z'] = [v*1000 for v in c_xyz]
                    data['col_roll'], data['col_pitch'], data['col_yaw'] = [self._rad_to_deg(v) for v in c_rpy]
                    g_type, dims, _ = self._parse_geometry(col.find('geometry'))
                    if g_type != 'None' and g_type != 'Mesh':
                        data['col_type'] = g_type
                        data['col_dim1'], data['col_dim2'], data['col_dim3'] = dims
            
            joints_data.append(data)

        # 3. Base Joint 정보 구성 (URDF에 world_to_base/floating_base가 없으면 sentinel 반환)
        if not found_base_joint:
            return joints_data, {'no_base': True}

        from logic.joint_logic import get_default_base_joint
        base_joint = get_default_base_joint()
        base_joint['mode'] = base_mode
        base_joint['name'] = 'world_to_base' if base_mode == 'Fixed' else 'floating_base'
        base_joint['x'] = base_joint_origin_xyz[0] * 1000.0
        base_joint['y'] = base_joint_origin_xyz[1] * 1000.0
        base_joint['z'] = base_joint_origin_xyz[2] * 1000.0
        base_joint['r'] = self._rad_to_deg(base_joint_origin_rpy[0])
        base_joint['p'] = self._rad_to_deg(base_joint_origin_rpy[1])
        base_joint['yaw'] = self._rad_to_deg(base_joint_origin_rpy[2])

        # base_link visual/collision 파싱
        base_link_name = 'base_link'
        if base_link_name in links_dict:
            bl = links_dict[base_link_name]
            vis = bl.find('visual')
            if vis is not None:
                v_xyz, v_rpy = self._parse_origin(vis)
                base_joint['vis_x'] = v_xyz[0] * 1000
                base_joint['vis_y'] = v_xyz[1] * 1000
                base_joint['vis_z'] = v_xyz[2] * 1000
                base_joint['vis_roll'] = self._rad_to_deg(v_rpy[0])
                base_joint['vis_pitch'] = self._rad_to_deg(v_rpy[1])
                base_joint['vis_yaw'] = self._rad_to_deg(v_rpy[2])
                g_type, dims, path = self._parse_geometry(vis.find('geometry'))
                if g_type == 'Mesh':
                    base_joint['vis_type'] = 'Mesh'
                    base_joint['vis_mesh'] = path
                elif g_type != 'None':
                    base_joint['vis_type'] = f'Manual ({g_type})'
                    base_joint['vis_dim1'], base_joint['vis_dim2'], base_joint['vis_dim3'] = dims
                # else: keep Auto (Cylinder)
            col = bl.find('collision')
            if col is not None:
                base_joint['col_enabled'] = True
                c_xyz, c_rpy = self._parse_origin(col)
                base_joint['col_x'] = c_xyz[0] * 1000
                base_joint['col_y'] = c_xyz[1] * 1000
                base_joint['col_z'] = c_xyz[2] * 1000
                base_joint['col_roll'] = self._rad_to_deg(c_rpy[0])
                base_joint['col_pitch'] = self._rad_to_deg(c_rpy[1])
                base_joint['col_yaw'] = self._rad_to_deg(c_rpy[2])
                g_type, dims, _ = self._parse_geometry(col.find('geometry'))
                if g_type not in ('None', 'Mesh'):
                    base_joint['col_type'] = g_type
                    base_joint['col_dim1'], base_joint['col_dim2'], base_joint['col_dim3'] = dims

        # 리턴: (조인트 데이터 리스트, 베이스 조인트 딕셔너리)
        return joints_data, base_joint