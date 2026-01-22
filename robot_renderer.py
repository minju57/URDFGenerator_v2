import numpy as np
import xml.etree.ElementTree as ET
import json
import html

class RobotRenderer:
    def __init__(self):
        self.joints = {}
        self.links = {}
        self.tree = {}
        self.base_link = None
        self.ordered_joints = []

    def load_urdf(self, urdf_content):
        self.joints = {}
        self.links = {}
        self.tree = {}
        self.base_link = None
        self.ordered_joints = []
        
        try:
            root = ET.fromstring(urdf_content)
        except ET.ParseError:
            return False

        for link in root.findall('link'):
            name = link.get('name')
            # [수정 1] 기본 type을 'box'가 아니라 None으로 설정 (Visual 없으면 안 보이게)
            link_info = {'name': name, 'type': None, 'dim': [0.1, 0.1, 0.1], 'color': [0.5, 0.5, 0.5], 'origin': [0,0,0,0,0,0]}
            
            visual = link.find('visual')
            if visual is not None:
                origin = visual.find('origin')
                if origin is not None:
                    xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                    rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
                    link_info['origin'] = xyz + rpy
                
                geom = visual.find('geometry')
                if geom is not None:
                    if geom.find('cylinder') is not None:
                        cyl = geom.find('cylinder')
                        link_info['type'] = 'cylinder'
                        link_info['dim'] = [float(cyl.get('radius', 0.05)), float(cyl.get('length', 0.1))]
                    elif geom.find('box') is not None:
                        box = geom.find('box')
                        dims = [float(x) for x in box.get('size', '0.1 0.1 0.1').split()]
                        link_info['type'] = 'box'
                        link_info['dim'] = dims
                    elif geom.find('sphere') is not None:
                        sph = geom.find('sphere')
                        link_info['type'] = 'sphere'
                        link_info['dim'] = [float(sph.get('radius', 0.05))]
                    elif geom.find('mesh') is not None:
                        link_info['type'] = 'box' 
                        link_info['dim'] = [0.1, 0.1, 0.1] 

            if name == "world": 
                link_info['color'] = [0.9, 0.9, 0.9]
                link_info['type'] = 'ground'
            elif name == "body": 
                link_info['color'] = [0.5, 0.5, 0.5]
            else:
                try: idx = int(name.split('_')[-1]); link_info['color'] = [0.8, 0.2, 0.2] if idx % 2 == 0 else [0.2, 0.2, 0.8]
                except: link_info['color'] = [0.6, 0.6, 0.6]
            self.links[name] = link_info

        for joint in root.findall('joint'):
            name = joint.get('name'); type_ = joint.get('type')
            parent = joint.find('parent').get('link'); child = joint.find('child').get('link')
            origin = joint.find('origin')
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()] if origin is not None else [0,0,0]
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()] if origin is not None else [0,0,0]
            axis_elem = joint.find('axis')
            axis = [float(x) for x in axis_elem.get('xyz', '1 0 0').split()] if axis_elem is not None else [1,0,0]
            limit = joint.find('limit')
            lower, upper = -3.14, 3.14
            if limit is not None:
                if limit.get('lower'): lower = float(limit.get('lower'))
                if limit.get('upper'): upper = float(limit.get('upper'))
            
            self.joints[name] = {'parent': parent, 'child': child, 'xyz': xyz, 'rpy': rpy, 'axis': axis, 'limits': [np.degrees(lower), np.degrees(upper)], 'type': type_}
            if type_ != 'fixed': self.ordered_joints.append(name)
            if parent not in self.tree: self.tree[parent] = []
            self.tree[parent].append(name)

        children = set(j['child'] for j in self.joints.values()); roots = list(set(self.links.keys()) - children)
        if roots: self.base_link = roots[0]
        elif 'world' in self.links: self.base_link = 'world'
        return True

    def get_joint_list(self):
        return [{'name': n, 'min': self.joints[n]['limits'][0], 'max': self.joints[n]['limits'][1], 'axis': self.joints[n]['axis']} for n in self.ordered_joints]

    def get_viewer_html(self):
        robot_data = {'links': self.links, 'joints': self.joints, 'tree': self.tree, 'base': self.base_link, 'joint_order': self.ordered_joints}
        json_data = json.dumps(robot_data)
        
        html_code = f"""
        <!DOCTYPE html>
        <html><head><style>body {{ margin: 0; overflow: hidden; }}</style>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
        <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script></head><body><script>
            const robotData = {json_data}; const jointMeshes = {{}};
            const scene = new THREE.Scene(); scene.background = new THREE.Color(0xf5f5f5);
            const camera = new THREE.PerspectiveCamera(45, window.innerWidth/window.innerHeight, 0.01, 1000);
            camera.position.set(2, 2, 2); camera.up.set(0, 0, 1); camera.lookAt(0,0,0);
            const renderer = new THREE.WebGLRenderer({{antialias:true}}); renderer.setSize(window.innerWidth, window.innerHeight);
            document.body.appendChild(renderer.domElement);
            new THREE.OrbitControls(camera, renderer.domElement);
            scene.add(new THREE.AmbientLight(0xffffff, 0.6));
            const dl = new THREE.DirectionalLight(0xffffff, 0.8); dl.position.set(5,10,7); scene.add(dl);
            scene.add(new THREE.GridHelper(10, 10).rotateX(Math.PI/2)); scene.add(new THREE.AxesHelper(0.5));

            function createGeometry(info) {{
                let geo;
                // type이 없으면 null 반환 (그리지 않음)
                if (!info.type) return null;

                if(info.type === 'cylinder') {{ geo = new THREE.CylinderGeometry(info.dim[0], info.dim[0], info.dim[1], 32); geo.rotateX(Math.PI/2); }}
                else if(info.type === 'box') {{ geo = new THREE.BoxGeometry(info.dim[0], info.dim[1], info.dim[2]); }}
                else if(info.type === 'sphere') {{ geo = new THREE.SphereGeometry(info.dim[0], 32, 32); }}
                else if(info.type === 'ground') {{ geo = new THREE.BoxGeometry(10, 10, 0.01); }}
                else {{ return null; }} // 알 수 없는 타입도 그리지 않음
                
                const mat = new THREE.MeshPhongMaterial({{color: new THREE.Color(info.color[0], info.color[1], info.color[2])}});
                const mesh = new THREE.Mesh(geo, mat);
                
                if(info.origin) {{ 
                    const [x,y,z, r,p,yaw] = info.origin; 
                    mesh.position.set(x,y,z); 
                    mesh.setRotationFromEuler(new THREE.Euler(r,p,yaw, 'XYZ')); 
                }}
                if(info.type === 'ground') mesh.position.z -= 0.005;
                return mesh;
            }}

            function buildRobot(name, parent) {{
                const info = robotData.links[name]; if(!info) return;
                
                // [수정 2] type이 존재할 때만 Geometry 생성 및 추가
                if(info.type) {{
                    const mesh = createGeometry(info);
                    if (mesh) parent.add(mesh);
                }}

                if(robotData.tree[name]) {{
                    robotData.tree[name].forEach(jName => {{
                        const jInfo = robotData.joints[jName];
                        const jGroup = new THREE.Group();
                        const [jx,jy,jz] = jInfo.xyz; const [jr,jp,jyaw] = jInfo.rpy;
                        jGroup.position.set(jx,jy,jz); jGroup.setRotationFromEuler(new THREE.Euler(jr,jp,jyaw, 'XYZ'));
                        parent.add(jGroup);
                        jointMeshes[jName] = {{ group: jGroup, axis: new THREE.Vector3(...jInfo.axis) }};
                        buildRobot(jInfo.child, jGroup);
                    }});
                }}
            }}
            if(robotData.base) {{ const root = new THREE.Group(); scene.add(root); buildRobot(robotData.base, root); }}
            function animate() {{ requestAnimationFrame(animate); renderer.render(scene, camera); }} animate();
            window.addEventListener("message", (e) => {{
                const vals = e.data; if(!Array.isArray(vals)) return;
                robotData.joint_order.forEach((name, i) => {{
                    if(jointMeshes[name]) {{
                        const rad = vals[i] * (Math.PI/180);
                        const q = new THREE.Quaternion().setFromAxisAngle(jointMeshes[name].axis, rad);
                        jointMeshes[name].group.setRotationFromQuaternion(q);
                    }}
                }});
            }});
            window.addEventListener('resize', () => {{ camera.aspect = window.innerWidth/window.innerHeight; camera.updateProjectionMatrix(); renderer.setSize(window.innerWidth, window.innerHeight); }});
        </script></body></html>"""
        return html.escape(html_code)