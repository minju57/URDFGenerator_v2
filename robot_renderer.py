import numpy as np
import xml.etree.ElementTree as ET
import json
import html
import base64
import os

class RobotRenderer:
    def __init__(self):
        self.joints = {}
        self.links = {}
        self.tree = {}
        self.base_link = None
        self.ordered_joints = []     # 움직이는 조인트 (슬라이더용)
        self.all_joint_names = []    # 모든 조인트 (체크박스용)

    def _parse_geometry(self, element, mesh_map, is_collision=False):
        info = {'type': None, 'dim': [0.1, 0.1, 0.1], 'scale': [1, 1, 1]}
        
        geom = element.find('geometry')
        if geom is None: return None

        if geom.find('mesh') is not None:
            if is_collision: return None 
            
            mesh_tag = geom.find('mesh')
            filename = mesh_tag.get('filename', '')
            
            # [Smart Matching] 대소문자 무시
            target_filename = os.path.basename(filename).lower()
            
            if target_filename in mesh_map:
                real_path = mesh_map[target_filename]
                try:
                    with open(real_path, "rb") as f:
                        ext = os.path.splitext(target_filename)[1]
                        if ext == '.obj':
                            info['type'] = 'obj'
                            info['mesh_data'] = f.read().decode('utf-8', errors='ignore')
                        else:
                            # STL (Binary)
                            info['type'] = 'mesh'
                            info['mesh_data'] = base64.b64encode(f.read()).decode('utf-8')

                        scale = mesh_tag.get('scale', '1 1 1').split()
                        info['scale'] = [float(s) for s in scale]
                        print(f"✅ Loaded Mesh: {filename}")
                except Exception as e:
                    print(f"❌ Read Error {filename}: {e}")
                    info['type'] = 'error_box'
            else:
                print(f"⚠️ Missing Mesh: {filename}")
                info['type'] = 'error_box'

        elif geom.find('cylinder') is not None:
            cyl = geom.find('cylinder')
            info['type'] = 'cylinder'
            info['dim'] = [float(cyl.get('radius', 0.05)), float(cyl.get('length', 0.1))]
            
        elif geom.find('box') is not None:
            box = geom.find('box')
            info['type'] = 'box'
            info['dim'] = [float(x) for x in box.get('size', '0.1 0.1 0.1').split()]
            
        elif geom.find('sphere') is not None:
            sph = geom.find('sphere')
            info['type'] = 'sphere'
            info['dim'] = [float(sph.get('radius', 0.05))]
            
        return info

    def load_urdf(self, urdf_content, mesh_files=[]):
        print("\n--- Loading URDF (V7 All Joints) ---")
        self.joints = {}
        self.links = {}
        self.tree = {}
        self.base_link = None
        self.ordered_joints = []
        self.all_joint_names = [] 

        mesh_map = {}
        if mesh_files:
            for f_path in mesh_files:
                fname = os.path.basename(f_path).lower()
                mesh_map[fname] = f_path
            print(f"📂 Uploaded Files Map: {list(mesh_map.keys())}")

        try: root = ET.fromstring(urdf_content)
        except ET.ParseError: return False

        # Link Parsing
        for i, link in enumerate(root.findall('link')):
            name = link.get('name')
            
            # 홀짝 기본 색상 (urdf_generator와 싱크를 맞춤)
            color_val = 0.8 if i % 2 == 0 else 0.4
            default_color = [color_val, color_val, color_val]
            
            link_info = {'name': name, 'visual': None, 'collision': None, 'color': default_color}
            
            visual = link.find('visual')
            if visual is not None:
                vis_data = self._parse_geometry(visual, mesh_map, is_collision=False)
                if vis_data:
                    origin = visual.find('origin')
                    if origin is not None:
                        xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                        rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
                        vis_data['origin'] = xyz + rpy
                    else: vis_data['origin'] = [0,0,0,0,0,0]
                    
                    material = visual.find('material')
                    if material is not None:
                        color = material.find('color')
                        if color is not None:
                            rgba = [float(x) for x in color.get('rgba', '0.5 0.5 0.5 1').split()]
                            link_info['color'] = rgba[:3]
                    vis_data['color'] = link_info['color']
                    link_info['visual'] = vis_data

            collision = link.find('collision')
            if collision is not None:
                col_data = self._parse_geometry(collision, mesh_map, is_collision=True)
                if col_data:
                    origin = collision.find('origin')
                    if origin is not None:
                        xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                        rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
                        col_data['origin'] = xyz + rpy
                    else: col_data['origin'] = [0,0,0,0,0,0]
                    link_info['collision'] = col_data

            self.links[name] = link_info

        # Joint Parsing
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
            
            self.all_joint_names.append(name)
            
            if type_ != 'fixed': self.ordered_joints.append(name)
            if parent not in self.tree: self.tree[parent] = []
            self.tree[parent].append(name)

        children = set(j['child'] for j in self.joints.values())
        roots = list(set(self.links.keys()) - children)
        if roots: self.base_link = roots[0]
        elif 'world' in self.links: self.base_link = 'world'
        
        print(f"✅ URDF Loaded. All Joints: {len(self.all_joint_names)} (Moving: {len(self.ordered_joints)})")
        return True

    def get_joint_list(self):
        res = []
        for name in self.all_joint_names:
            j = self.joints[name]
            res.append({
                'name': name,
                'type': j['type'],
                'parent': j['parent'],
                'child': j['child'],
                'min': j['limits'][0],
                'max': j['limits'][1],
                'axis': j['axis']
            })
        return res

    def get_viewer_html(self):
        robot_data = {'links': self.links, 'joints': self.joints, 'tree': self.tree, 'base': self.base_link, 'joint_order': self.ordered_joints}
        json_data = json.dumps(robot_data)
        
        html_code = f"""
        <!DOCTYPE html>
        <html><head>
        <style>
            body {{ margin: 0; overflow: hidden; font-family: monospace; }}
            #ui {{ position: absolute; top: 10px; left: 10px; z-index: 100; display: flex; flex-direction: column; gap: 5px; }}
            #debug-log {{ 
                position: absolute; bottom: 10px; left: 10px; 
                background: rgba(0,0,0,0.7); color: lime; 
                padding: 10px; font-size: 12px; pointer-events: none;
                max-height: 200px; overflow-y: auto; width: 400px;
                border-radius: 5px;
                display: block; 
            }}
            .btn {{ padding: 6px 10px; background: #333; color: white; border: 1px solid #555; cursor: pointer; border-radius: 3px; font-size: 12px; }}
            .btn:hover {{ background: #555; }}
            .row {{ display: flex; gap: 5px; align-items: center; }}
        </style>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
        <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
        <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>
        <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/OBJLoader.js"></script>
        </head><body>
        
        <div id="ui">
            <div class="row">
                <button class="btn" onclick="fitCamera()">🔍 Auto Fit</button>
                <button class="btn" onclick="toggleLog()">📜 Log</button>
            </div>
            <div class="row" style="background:rgba(255,255,255,0.8); padding:5px; border-radius:3px; gap:10px;">
                <label><input type="checkbox" id="chkVisual" checked onchange="toggleVisual(this.checked)"> Show Visual</label>
                <label><input type="checkbox" id="chkCollision" onchange="toggleCollision(this.checked)"> Show Collision</label>
            </div>
        </div>
        <div id="debug-log">Initializing Renderer...</div>

        <script>
            function log(msg) {{
                const el = document.getElementById('debug-log');
                if(el) {{
                    el.innerHTML += "<div>> " + msg + "</div>";
                    el.scrollTop = el.scrollHeight;
                }}
                console.log(msg);
            }}

            function toggleLog() {{
                const el = document.getElementById('debug-log');
                el.style.display = (el.style.display === 'none') ? 'block' : 'none';
            }}

            const robotData = {json_data}; 
            const jointMeshes = {{}}; 
            const visualMeshes = [];      // [추가] 시각적 요소 리스트
            const collisionMeshes = [];
            const jointAxes = {{}};
            const rootGroup = new THREE.Group();

            const scene = new THREE.Scene(); scene.background = new THREE.Color(0xf0f0f0);
            
            // Camera (Z-Up)
            const camera = new THREE.PerspectiveCamera(50, window.innerWidth/window.innerHeight, 0.01, 10000);
            camera.up.set(0, 0, 1); 
            camera.position.set(2, 2, 2); 
            
            const renderer = new THREE.WebGLRenderer({{antialias:true}}); 
            renderer.setSize(window.innerWidth, window.innerHeight);
            renderer.shadowMap.enabled = true;
            document.body.appendChild(renderer.domElement);
            
            const controls = new THREE.OrbitControls(camera, renderer.domElement);
            
            scene.add(new THREE.AmbientLight(0xffffff, 0.6));
            const dl = new THREE.DirectionalLight(0xffffff, 0.8); 
            dl.position.set(5,10,7); scene.add(dl);
            
            const grid = new THREE.GridHelper(20, 20);
            grid.rotateX(Math.PI / 2); 
            scene.add(grid); 
            
            scene.add(new THREE.AxesHelper(1));
            scene.add(rootGroup);

            log("Renderer Started (V7).");

            function createThickFrame(len = 0.15, thick = 0.005) {{
                const group = new THREE.Group();
                const headLen = len * 0.2; 
                const headWidth = thick * 3; 
                
                function makeArrow(color, rot) {{
                    const arrow = new THREE.Group();
                    const mat = new THREE.MeshBasicMaterial({{color: color}});
                    const shaftGeo = new THREE.CylinderGeometry(thick, thick, len - headLen, 12);
                    const shaft = new THREE.Mesh(shaftGeo, mat);
                    shaft.position.y = (len - headLen) / 2;
                    arrow.add(shaft);
                    const headGeo = new THREE.ConeGeometry(headWidth, headLen, 12);
                    const head = new THREE.Mesh(headGeo, mat);
                    head.position.y = len - headLen / 2;
                    arrow.add(head);
                    arrow.rotation.set(...rot);
                    return arrow;
                }}
                group.add(makeArrow(0xff0000, [0, 0, -Math.PI/2]));
                group.add(makeArrow(0x00ff00, [0, 0, 0]));
                group.add(makeArrow(0x0000ff, [Math.PI/2, 0, 0]));
                return group;
            }}

            function createGeometry(info, isCollision=false) {{
                if (!info || !info.type) return null;
                let mesh = null;

                const mat = isCollision 
                    ? new THREE.MeshBasicMaterial({{color: 0xff0000, wireframe: true, transparent: true, opacity: 0.5}})
                    : new THREE.MeshPhongMaterial({{color: new THREE.Color(info.color[0], info.color[1], info.color[2]), shininess: 30, side: THREE.DoubleSide}});

                try {{
                    if(info.type === 'mesh' && info.mesh_data) {{
                        const loader = new THREE.STLLoader();
                        const binaryString = window.atob(info.mesh_data);
                        const len = binaryString.length;
                        const bytes = new Uint8Array(len);
                        for (let i = 0; i < len; i++) {{ bytes[i] = binaryString.charCodeAt(i); }}
                        const geo = loader.parse(bytes.buffer);
                        if(info.scale) geo.scale(info.scale[0], info.scale[1], info.scale[2]);
                        
                        geo.computeBoundingBox();
                        const size = new THREE.Vector3();
                        geo.boundingBox.getSize(size);
                        log("STL: " + size.x.toFixed(2) + " x " + size.y.toFixed(2) + " x " + size.z.toFixed(2));
                        
                        mesh = new THREE.Mesh(geo, mat);
                    }}
                    else if(info.type === 'obj' && info.mesh_data) {{
                        const loader = new THREE.OBJLoader();
                        const objGroup = loader.parse(info.mesh_data);
                        if(info.scale) objGroup.scale.set(info.scale[0], info.scale[1], info.scale[2]);
                        objGroup.traverse((child) => {{ if (child.isMesh) child.material = mat; }});
                        mesh = objGroup;
                        log("OBJ Loaded.");
                    }}
                    else if(info.type === 'cylinder') {{ 
                        const geo = new THREE.CylinderGeometry(info.dim[0], info.dim[0], info.dim[1], 32); 
                        geo.rotateX(Math.PI/2); 
                        mesh = new THREE.Mesh(geo, mat);
                    }}
                    else if(info.type === 'box') {{ 
                        const geo = new THREE.BoxGeometry(info.dim[0], info.dim[1], info.dim[2]); 
                        mesh = new THREE.Mesh(geo, mat);
                    }}
                    else if(info.type === 'sphere') {{ 
                        const geo = new THREE.SphereGeometry(info.dim[0], 32, 32); 
                        mesh = new THREE.Mesh(geo, mat);
                    }}
                    else if(info.type === 'error_box') {{
                        const geo = new THREE.BoxGeometry(0.2, 0.2, 0.2);
                        const errMat = new THREE.MeshBasicMaterial({{color: 0xff00ff, wireframe: true}});
                        mesh = new THREE.Mesh(geo, errMat);
                        log("⚠️ Fallback Box");
                    }}
                }} catch(e) {{
                    log("❌ Error: " + e.message);
                }}
                
                if(mesh && info.origin) {{ 
                    const [x,y,z, r,p,yaw] = info.origin; 
                    mesh.position.set(x,y,z); 
                    mesh.rotation.set(r,p,yaw, 'ZYX');
                }}
                
                // [수정] Visual/Collision 분리 저장
                if (isCollision && mesh) {{
                    mesh.visible = false; // 충돌체는 기본적으로 숨김
                    collisionMeshes.push(mesh);
                    if(mesh.isGroup) mesh.traverse(c => {{ if(c.isMesh) collisionMeshes.push(c); c.visible = false; }});
                }} 
                else if (!isCollision && mesh) {{
                    visualMeshes.push(mesh); // 시각체 리스트에 추가
                }}
                
                return mesh;
            }}

            function buildRobot(name, parent) {{
                const info = robotData.links[name]; 
                if(info) {{
                    if (info.visual) {{
                        const vMesh = createGeometry(info.visual, false);
                        if (vMesh) parent.add(vMesh);
                    }}
                    if (info.collision) {{
                        const cMesh = createGeometry(info.collision, true);
                        if (cMesh) parent.add(cMesh);
                    }}
                }}
                if(robotData.tree[name]) {{
                    robotData.tree[name].forEach(jName => {{
                        const jInfo = robotData.joints[jName];
                        const fixedGroup = new THREE.Group();
                        const [jx,jy,jz] = jInfo.xyz; 
                        const [jr,jp,jyaw] = jInfo.rpy;
                        fixedGroup.position.set(jx,jy,jz); 
                        fixedGroup.rotation.set(jr,jp,jyaw, 'ZYX');
                        parent.add(fixedGroup);

                        // [두꺼운 축 생성]
                        const axes = createThickFrame(0.15, 0.005);
                        axes.visible = false;
                        fixedGroup.add(axes);
                        jointAxes[jName] = axes;

                        const movingGroup = new THREE.Group();
                        fixedGroup.add(movingGroup);
                        jointMeshes[jName] = {{ group: movingGroup, axis: new THREE.Vector3(...jInfo.axis) }};
                        buildRobot(jInfo.child, movingGroup);
                    }});
                }}
            }}

            if(robotData.base) {{ buildRobot(robotData.base, rootGroup); }}

            window.fitCamera = function() {{
                const box = new THREE.Box3().setFromObject(rootGroup);
                if (box.isEmpty()) return;
                const size = box.getSize(new THREE.Vector3());
                const center = box.getCenter(new THREE.Vector3());
                if(size.length() < 0.0001) return;
                const maxDim = Math.max(size.x, size.y, size.z);
                const fov = camera.fov * (Math.PI / 180);
                let cameraZ = Math.abs(maxDim / 2 * Math.tan(fov * 2));
                cameraZ *= 2.5; 
                if (cameraZ < 0.1) cameraZ = 0.5;
                if (cameraZ > 1000) cameraZ = 1000;
                camera.position.set(center.x + cameraZ, center.y + cameraZ, center.z + cameraZ);
                camera.lookAt(center);
                controls.target.copy(center);
                controls.update();
                log("Camera fitted.");
            }};

            // [추가] 토글 기능들
            window.toggleVisual = function(isChecked) {{
                visualMeshes.forEach(mesh => {{ mesh.visible = isChecked; }});
            }};

            window.toggleCollision = function(isChecked) {{
                collisionMeshes.forEach(mesh => {{ mesh.visible = isChecked; }});
            }};

            function animate() {{ requestAnimationFrame(animate); renderer.render(scene, camera); }} 
            animate();

            setTimeout(fitCamera, 500);
            setTimeout(fitCamera, 1500);

            window.addEventListener("message", (e) => {{
                const data = e.data;
                if(Array.isArray(data)) {{
                    robotData.joint_order.forEach((name, i) => {{
                        if(jointMeshes[name]) {{
                            const rad = data[i] * (Math.PI/180);
                            const q = new THREE.Quaternion().setFromAxisAngle(jointMeshes[name].axis, rad);
                            jointMeshes[name].group.setRotationFromQuaternion(q);
                        }}
                    }});
                }}
                else if(data && data.type === 'frame') {{
                    if(jointAxes[data.name]) {{
                        jointAxes[data.name].visible = data.val;
                    }}
                }}
            }});
            window.addEventListener('resize', () => {{ 
                camera.aspect = window.innerWidth/window.innerHeight; 
                camera.updateProjectionMatrix(); 
                renderer.setSize(window.innerWidth, window.innerHeight); 
            }});
        </script></body></html>"""
        return html.escape(html_code)