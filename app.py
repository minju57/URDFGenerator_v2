import gradio as gr
import os
import copy
import pandas as pd
from urdf_generator import URDFManager
from robot_renderer import RobotRenderer

generator = URDFManager()
renderer = RobotRenderer()

# --- Data Helpers ---
def get_default_joint():
    return {
        'axis': 'Roll', 
        'x': 0.0, 'y': 0.0, 'z': 0.0, 
        'r': 0.0, 'p': 0.0, 'yaw': 0.0, 
        'low': -180.0, 'up': 180.0,
        'vis_type': 'Auto (Cylinder)', 
        'vis_mesh': 'meshes/link.STL',
        'vis_scale_x': 0.001, 'vis_scale_y': 0.001, 'vis_scale_z': 0.001,
        'col_enabled': False, 
        'col_type': 'Cylinder',
        'col_dim1': 50.0, 'col_dim2': 200.0, 'col_dim3': 50.0, 
        'col_x': 0.0, 'col_y': 0.0, 'col_z': 0.0, 
        'col_roll': 0.0, 'col_pitch': 0.0, 'col_yaw': 0.0
    }

def add_joint(data): return data + [get_default_joint()]

def delete_joint(data, index):
    if len(data) > 0: return data[:index] + data[index+1:]
    return data

# [수정] 값 업데이트 시 범위 제한 및 상/하한 논리 적용
def update_val(data, index, key, value):
    try:
        if key in ['vis_mesh', 'vis_type', 'col_type', 'col_enabled', 'axis']: 
            data[index][key] = value
        else: 
            val = float(value)
            
            # 1. 각도 관련 값은 -180 ~ 180 범위로 고정
            angle_keys = ['r', 'p', 'yaw', 'low', 'up', 'col_roll', 'col_pitch', 'col_yaw']
            if key in angle_keys:
                val = max(-180.0, min(180.0, val))
            
            data[index][key] = val

            # 2. Joint Limit 논리 보호 (Lower가 Upper보다 커지지 않게)
            if key == 'low':
                if data[index]['up'] < val:
                    data[index]['up'] = val
            elif key == 'up':
                if data[index]['low'] > val:
                    data[index]['low'] = val

    except: pass
    return data

def get_axis_name(axis_list):
    try:
        if axis_list == [1.0, 0.0, 0.0]: return "Roll"
        if axis_list == [0.0, 1.0, 0.0]: return "Pitch"
        if axis_list == [0.0, 0.0, 1.0]: return "Yaw"
        return "Custom"
    except: return "Custom"

# --- File Parsing Logic ---
def parse_inertia_file(file_obj):
    if not file_obj: return None
    data = []
    try:
        filepath = file_obj.name if hasattr(file_obj, 'name') else file_obj
        ext = os.path.splitext(filepath)[1].lower()
        
        df = None
        if ext == '.csv':
            df = pd.read_csv(filepath)
        elif ext in ['.xlsx', '.xls']:
            df = pd.read_excel(filepath)
        else:
            print(f"❌ Unsupported file extension: {ext}")
            return None

        if df is not None:
            df.columns = [str(c).strip() for c in df.columns]
            df = df.fillna(0.0)
            data = df.to_dict(orient='records')
            print(f"✅ Parsed {len(data)} rows from {ext} file.")
            
    except Exception as e:
        print(f"❌ File Parse Error: {e}")
        return None
    return data

# --- Core Logic ---
def generate_download(joints, filename, include_base, use_mesh_base, base_mesh_path, inertia_file):
    if not filename: filename = "robot"
    if not filename.endswith(".urdf"): filename += ".urdf"
    
    joints_meter = copy.deepcopy(joints)
    length_keys = ['x', 'y', 'z', 'col_dim1', 'col_dim2', 'col_dim3', 'col_x', 'col_y', 'col_z']
    
    for j in joints_meter:
        for key in length_keys:
            if key in j:
                j[key] = j[key] / 1000.0  

    inertia_data = parse_inertia_file(inertia_file)
    final_mesh_path = base_mesh_path if use_mesh_base else None

    xml, path = generator.generate(joints_meter, filename, include_base, final_mesh_path, inertia_data)
    return xml, os.path.abspath(path) if path else None

def load_urdf_to_iframe(file_objs):
    if not file_objs: return None, []
    
    urdf_content = None
    mesh_files = [] 
    
    for f in file_objs:
        filepath = f.name if hasattr(f, 'name') else f
        if filepath.lower().endswith(".urdf"):
            with open(filepath, "r", encoding="utf-8") as urdf_file:
                urdf_content = urdf_file.read()
        else:
            mesh_files.append(filepath)

    if urdf_content is None:
        return "<h3>Error: URDF file not found in upload.</h3>", []

    if not renderer.load_urdf(urdf_content, mesh_files=mesh_files): 
        return "<h3>Error parsing URDF</h3>", []

    html = renderer.get_viewer_html()
    iframe = f'<iframe id="vis_iframe" srcdoc="{html}" width="100%" height="600px" style="border:1px solid #ddd;" allowfullscreen></iframe>'
    return iframe, renderer.get_joint_list()

# --- GUI Layout ---
css = """#preview_code button { display: none !important; } .compact-row { gap: 5px; }"""

with gr.Blocks(title="URDF Builder") as demo:
    gr.Markdown("# 🤖 URDF Builder")
    
    with gr.Tabs():
        with gr.Tab("🛠️ URDF generator"):
            gen_state = gr.State([get_default_joint()])
            
            with gr.Row():
                with gr.Column(scale=1):
                    @gr.render(inputs=gen_state)
                    def render_builder(data):
                        for i, j in enumerate(data):
                            with gr.Group(key=f"group_{i}"):
                                with gr.Row(equal_height=True):
                                    with gr.Column(scale=1): gr.Markdown(f"### 🔗 Joint {i}")
                                    with gr.Column(scale=0, min_width=50):
                                        btn_del = gr.Button("❌", size="sm", variant="stop", key=f"del_{i}")
                                        btn_del.click(fn=delete_joint, inputs=[gen_state, gr.Number(value=i, visible=False, key=f"idx_{i}")], outputs=[gen_state])
                                
                                with gr.Accordion("Joint Configuration", open=True):
                                    ax = gr.Radio(["Roll", "Pitch", "Yaw"], value=j['axis'], label="Axis", interactive=True, key=f"axis_{i}")
                                    ax.change(lambda v, d, idx=i: update_val(d, idx, 'axis', v), inputs=[ax, gen_state], outputs=[gen_state])
                                    
                                    with gr.Row():
                                        for key, label in [('x', 'X (mm)'), ('y', 'Y (mm)'), ('z', 'Z (mm)')]:
                                            num = gr.Number(value=j[key], label=label, step=1.0, interactive=True, key=f"pos_{key}_{i}")
                                            num.blur(lambda v, d, idx=i, k=key: update_val(d, idx, k, v), inputs=[num, gen_state], outputs=[gen_state])
                                    
                                    # [수정] min/max 설정 추가
                                    with gr.Row():
                                        for key, label in [('r', 'Roll (deg)'), ('p', 'Pitch (deg)'), ('yaw', 'Yaw (deg)')]:
                                            num = gr.Number(value=j[key], label=label, minimum=-180, maximum=180, step=1.0, interactive=True, key=f"rot_{key}_{i}")
                                            num.blur(lambda v, d, idx=i, k=key: update_val(d, idx, k, v), inputs=[num, gen_state], outputs=[gen_state])
                                    
                                    # [수정] min/max 설정 추가 및 논리 보호
                                    with gr.Row():
                                        for key, label in [('low', 'Lower Limit (deg)'), ('up', 'Upper Limit (deg)')]:
                                            num = gr.Number(value=j[key], label=label, minimum=-180, maximum=180, step=1.0, interactive=True, key=f"lim_{key}_{i}")
                                            num.blur(lambda v, d, idx=i, k=key: update_val(d, idx, k, v), inputs=[num, gen_state], outputs=[gen_state])

                                with gr.Accordion("Visual Body", open=False):
                                    v_type = gr.Radio(["Auto (Cylinder)", "Auto (Box)", "Mesh"], value=j['vis_type'], label="Visual Type", interactive=True, key=f"vis_type_{i}")
                                    v_type.change(lambda v, d, idx=i: update_val(d, idx, 'vis_type', v), inputs=[v_type, gen_state], outputs=[gen_state])
                                    
                                    if j['vis_type'] == "Mesh":
                                        mesh_path = gr.Textbox(value=j['vis_mesh'], label="STL Path", placeholder="package://...", interactive=True, key=f"vis_mesh_{i}")
                                        mesh_path.blur(lambda v, d, idx=i: update_val(d, idx, 'vis_mesh', v), inputs=[mesh_path, gen_state], outputs=[gen_state])
                                    else:
                                        gr.Markdown(f"ℹ️ *{j['vis_type']} shape will be auto-generated connecting to the next joint.*")

                                with gr.Accordion("Collision Body (Manual)", open=False):
                                    c_enable = gr.Checkbox(value=j['col_enabled'], label="Enable Collision Geometry", interactive=True, key=f"col_enable_{i}")
                                    c_enable.input(lambda v, d, idx=i: update_val(d, idx, 'col_enabled', v), inputs=[c_enable, gen_state], outputs=[gen_state])
                                    
                                    if j['col_enabled']:
                                        c_type = gr.Dropdown(["Cylinder", "Box", "Sphere"], value=j['col_type'], label="Shape", interactive=True, key=f"col_type_{i}")
                                        c_type.input(lambda v, d, idx=i: update_val(d, idx, 'col_type', v), inputs=[c_type, gen_state], outputs=[gen_state])
                                        
                                        with gr.Row():
                                            l1 = "Radius (mm)" if j['col_type'] != "Box" else "Size X (mm)"
                                            l2 = "Length (mm)" if j['col_type'] == "Cylinder" else ("Size Y (mm)" if j['col_type'] == "Box" else "-")
                                            l3 = "Size Z (mm)" if j['col_type'] == "Box" else "-"
                                            
                                            n1 = gr.Number(value=j['col_dim1'], label=l1, step=1.0, interactive=True, key=f"col_d1_{i}")
                                            n1.blur(lambda v, d, idx=i: update_val(d, idx, 'col_dim1', v), inputs=[n1, gen_state], outputs=[gen_state])
                                            
                                            if j['col_type'] != "Sphere":
                                                n2 = gr.Number(value=j['col_dim2'], label=l2, step=1.0, interactive=True, key=f"col_d2_{i}")
                                                n2.blur(lambda v, d, idx=i: update_val(d, idx, 'col_dim2', v), inputs=[n2, gen_state], outputs=[gen_state])
                                            if j['col_type'] == "Box":
                                                n3 = gr.Number(value=j['col_dim3'], label=l3, step=1.0, interactive=True, key=f"col_d3_{i}")
                                                n3.blur(lambda v, d, idx=i: update_val(d, idx, 'col_dim3', v), inputs=[n3, gen_state], outputs=[gen_state])
                                        
                                        gr.Markdown("Collision Origin (Offset from Joint)")
                                        with gr.Row():
                                            for key in ['col_x','col_y','col_z']:
                                                num = gr.Number(value=j[key], label=f"{key[-1].upper()} (mm)", step=1.0, interactive=True, key=f"col_pos_{key}_{i}")
                                                num.blur(lambda v, d, idx=i, k=key: update_val(d, idx, k, v), inputs=[num, gen_state], outputs=[gen_state])
                                        with gr.Row():
                                            for key in ['col_roll','col_pitch','col_yaw']:
                                                # [수정] Collision Rotation도 -180~180 범위 제한
                                                num = gr.Number(value=j[key], label=key.split('_')[1].capitalize(), minimum=-180, maximum=180, step=1.0, interactive=True, key=f"col_rot_{key}_{i}")
                                                num.blur(lambda v, d, idx=i, k=key: update_val(d, idx, k, v), inputs=[num, gen_state], outputs=[gen_state])

                    gr.Button("➕ Add Joint", variant="secondary").click(add_joint, inputs=[gen_state], outputs=[gen_state])

                with gr.Column(scale=1):
                    gr.Markdown("#### Export URDF")
                    fname = gr.Textbox(value="my_robot", label="Robot Name")
                    
                    use_base = gr.Checkbox(value=True, label="Generate Base Stand (Auto Cylinder)")
                    use_mesh_base = gr.Checkbox(value=False, label="Use Custom Base Mesh")
                    
                    use_base.change(fn=lambda x: False if x else gr.update(), inputs=use_base, outputs=use_mesh_base)
                    use_mesh_base.change(fn=lambda x: False if x else gr.update(), inputs=use_mesh_base, outputs=use_base)

                    base_mesh = gr.Textbox(placeholder="meshes/body.STL", label="Base Mesh Path", visible=False)
                    use_mesh_base.change(lambda x: gr.update(visible=x), inputs=[use_mesh_base], outputs=[base_mesh])
                    
                    inertia_file = gr.File(label="Upload Inertia Data (CSV or Excel)", file_types=[".csv", ".xlsx", ".xls"])
                    
                    btn_gen = gr.Button("🚀 Generate URDF", variant="primary")
                    code = gr.Code(language="html", label="XML Output")
                    dl = gr.File(label="Download .urdf")
                    
                    btn_gen.click(generate_download, inputs=[gen_state, fname, use_base, use_mesh_base, base_mesh, inertia_file], outputs=[code, dl])

        with gr.Tab("👁️ Visualizer"):
            vis_state = gr.State((None, []))
            def on_upload(f): return load_urdf_to_iframe(f)
            
            with gr.Row():
                with gr.Column(scale=3):
                    vis_file = gr.File(label="Upload URDF & STL/OBJ Files", file_count="multiple")
                    @gr.render(inputs=vis_state)
                    def render_vis(data):
                        html, _ = data
                        if not html: return gr.HTML("<div style='height:600px;background:#f5f5f5;display:flex;align-items:center;justify-content:center;color:#999;'>Upload a URDF file to view</div>")
                        return gr.HTML(html)
                with gr.Column(scale=1):
                    @gr.render(inputs=vis_state)
                    def render_ctrl(data):
                        _, joints = data
                        if not joints: return
                        
                        sliders = []
                        js_angle = """(...args) => {
                            const f = document.getElementById('vis_iframe');
                            if(!f) return;
                            const vals = args.filter(a => typeof a === 'number');
                            f.contentWindow.postMessage(vals, '*');
                        }"""

                        with gr.Group():
                            gr.Markdown("### 🦾 Joint Control")
                            for j in joints:
                                if j['type'] != 'fixed':
                                    ax_name = get_axis_name(j['axis'])
                                    s = gr.Slider(int(j['min']), int(j['max']), value=0, step=1, label=f"{j['name']} ({ax_name})")
                                    sliders.append(s)
                        
                        for s in sliders: s.change(None, inputs=sliders, js=js_angle)

                        with gr.Group():
                            gr.Markdown("### 📏 Show Frames")
                            for j in joints:
                                label_text = f"Show {j['name']}"
                                if j['type'] == 'fixed': label_text += " (Fixed)"
                                
                                chk = gr.Checkbox(label=label_text, value=False)
                                js_frame = f"""(v) => {{
                                    const f = document.getElementById('vis_iframe');
                                    if(f) f.contentWindow.postMessage({{type: 'frame', name: '{j['name']}', val: v}}, '*');
                                }}"""
                                chk.change(None, inputs=[chk], js=js_frame)

            vis_file.change(on_upload, vis_file, vis_state)

if __name__ == "__main__":
    # demo.launch()
    demo.launch(server_name="0.0.0.0", server_port=50001)