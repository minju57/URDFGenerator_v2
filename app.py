import gradio as gr
import os
from urdf_generator import URDFManager
from robot_renderer import RobotRenderer

generator = URDFManager()
renderer = RobotRenderer()

# Default Data
def get_default_joint():
    return {
        'axis': 'Roll', 
        'x': 0.0, 'y': 0.0, 'z': 0.0, 
        'r': 0.0, 'p': 0.0, 'yaw': 0.0, 
        'low': -180.0, 'up': 180.0,
        'vis_type': 'Auto (Cylinder)', 
        'vis_mesh': 'package://ur_description/meshes/visual/link.stl',
        'col_enabled': False, 'col_type': 'Cylinder',
        'col_dim1': 0.05, 'col_dim2': 0.2, 'col_dim3': 0.05,
        'col_x': 0.0, 'col_y': 0.0, 'col_z': 0.0, 'col_roll': 0.0, 'col_pitch': 0.0, 'col_yaw': 0.0
    }

def add_joint(data): return data + [get_default_joint()]

def delete_joint(data, index):
    if len(data) > 0: return data[:index] + data[index+1:]
    return data

def update_val(data, index, key, value):
    try:
        if key in ['vis_mesh', 'vis_type', 'col_type', 'col_enabled', 'axis']: data[index][key] = value
        else: data[index][key] = float(value)
    except: pass
    return data

def generate_download(joints, filename):
    if not filename: filename = "robot"
    if not filename.endswith(".urdf"): filename += ".urdf"
    xml, path = generator.generate(joints, filename)
    return xml, os.path.abspath(path) if path else None

def get_axis_name(axis_list):
    try:
        if axis_list == [1.0, 0.0, 0.0]: return "Roll"
        if axis_list == [0.0, 1.0, 0.0]: return "Pitch"
        if axis_list == [0.0, 0.0, 1.0]: return "Yaw"
        return "Custom"
    except: return "Custom"

def load_urdf_to_iframe(file_objs):
    if not file_objs: return None, []
    
    # [수정] 업로드된 파일들 중에서 .urdf 파일 찾기
    urdf_content = None
    mesh_files = [] # STL 등 나머지 파일들의 경로 리스트
    
    # Gradio는 file_objs로 파일 경로 리스트(또는 객체 리스트)를 줍니다.
    # file_objs는 리스트 형태입니다.
    
    for f in file_objs:
        # f는 보통 temp 경로 문자열이거나 파일 객체입니다. Gradio 버전에 따라 다를 수 있으나
        # 보통 f.name 으로 접근하거나 f 자체가 경로입니다.
        try:
            filename = f.name # Gradio 파일 객체일 경우
        except:
            filename = f # 문자열 경로일 경우

        if filename.endswith(".urdf") or filename.endswith(".URDF"):
            with open(filename, "r", encoding="utf-8") as urdf_file:
                urdf_content = urdf_file.read()
        else:
            # URDF가 아닌 파일들은 전부 메쉬 파일 후보로 저장
            mesh_files.append(filename)

    if urdf_content is None:
        return "<h3>Error: URDF file not found in upload.</h3>", []

    # [수정] URDF 내용과 함께 메쉬 파일 리스트도 렌더러에 전달
    if not renderer.load_urdf(urdf_content, mesh_files=mesh_files): 
        return "<h3>Error parsing URDF</h3>", []

    html = renderer.get_viewer_html()
    iframe = f'<iframe id="vis_iframe" srcdoc="{html}" width="100%" height="600px" style="border:1px solid #ddd;" allowfullscreen></iframe>'
    return iframe, renderer.get_joint_list()

css = """#preview_code button { display: none !important; } .compact-row { gap: 5px; }"""

with gr.Blocks(title="URDF Builder") as demo:
    gr.Markdown("# 🤖 URDF Builder")
    
    with gr.Tabs():
        # TAB 1: GENERATOR
        with gr.Tab("🛠️ URDF generator"):
            gen_state = gr.State([get_default_joint()])
            
            with gr.Row():
                with gr.Column(scale=1):
                    @gr.render(inputs=gen_state)
                    def render_builder(data):
                        for i, j in enumerate(data):
                            with gr.Group():
                                with gr.Row(equal_height=True):
                                    with gr.Column(scale=1): gr.Markdown(f"### 🔗 Joint {i}")
                                    with gr.Column(scale=0, min_width=50):
                                        btn_del = gr.Button("❌", size="sm", variant="stop")
                                        btn_del.click(fn=delete_joint, inputs=[gen_state, gr.Number(value=i, visible=False)], outputs=[gen_state])
                                
                                with gr.Accordion("Joint Configuration", open=True):
                                    ax = gr.Radio(["Roll", "Pitch", "Yaw"], value=j['axis'], label="Axis", interactive=True)
                                    ax.change(lambda v, d, idx=i: update_val(d, idx, 'axis', v), inputs=[ax, gen_state], outputs=[gen_state])
                                    
                                    # [수정 포인트 1] Position (X, Y, Z) - 라벨과 키 분리
                                    with gr.Row():
                                        # key는 실제 데이터 키('x'), label은 보여줄 이름('X (m)')
                                        for key, label in [('x', 'X (m)'), ('y', 'Y (m)'), ('z', 'Z (m)')]:
                                            num = gr.Number(value=j[key], label=label, step=0.1, interactive=True)
                                            num.input(lambda v, d, idx=i, k=key: update_val(d, idx, k, v), inputs=[num, gen_state], outputs=[gen_state])
                                    
                                    # [수정 포인트 2] Limits - 라벨과 키 분리
                                    with gr.Row():
                                        for key, label in [('low', 'Lower Limit (deg)'), ('up', 'Upper Limit (deg)')]:
                                            num = gr.Number(value=j[key], label=label, step=0.1, interactive=True)
                                            num.input(lambda v, d, idx=i, k=key: update_val(d, idx, k, v), inputs=[num, gen_state], outputs=[gen_state])

                                with gr.Accordion("Visual Body", open=False):
                                    v_type = gr.Radio(["Auto (Cylinder)", "Auto (Box)", "Mesh"], value=j['vis_type'], label="Visual Type", interactive=True)
                                    v_type.change(lambda v, d, idx=i: update_val(d, idx, 'vis_type', v), inputs=[v_type, gen_state], outputs=[gen_state])
                                    
                                    if j['vis_type'] == "Mesh":
                                        mesh_path = gr.Textbox(value=j['vis_mesh'], label="STL Path", placeholder="package://...", interactive=True)
                                        mesh_path.input(lambda v, d, idx=i: update_val(d, idx, 'vis_mesh', v), inputs=[mesh_path, gen_state], outputs=[gen_state])
                                    else:
                                        gr.Markdown(f"ℹ️ *{j['vis_type']} shape will be auto-generated connecting to the next joint.*")

                                with gr.Accordion("Collision Body (Manual)", open=False):
                                    c_enable = gr.Checkbox(value=j['col_enabled'], label="Enable Collision", interactive=True)
                                    c_enable.input(lambda v, d, idx=i: update_val(d, idx, 'col_enabled', v), inputs=[c_enable, gen_state], outputs=[gen_state])
                                    if j['col_enabled']:
                                        c_type = gr.Dropdown(["Cylinder", "Box", "Sphere"], value=j['col_type'], label="Shape", interactive=True)
                                        c_type.input(lambda v, d, idx=i: update_val(d, idx, 'col_type', v), inputs=[c_type, gen_state], outputs=[gen_state])
                                        
                                        with gr.Row():
                                            l1 = "Radius" if j['col_type'] != "Box" else "X"
                                            l2 = "Length" if j['col_type'] == "Cylinder" else ("Y" if j['col_type'] == "Box" else "-")
                                            l3 = "Z" if j['col_type'] == "Box" else "-"
                                            
                                            n1 = gr.Number(value=j['col_dim1'], label=l1, step=0.1, interactive=True)
                                            n1.input(lambda v, d, idx=i: update_val(d, idx, 'col_dim1', v), inputs=[n1, gen_state], outputs=[gen_state])
                                            
                                            if j['col_type'] != "Sphere":
                                                n2 = gr.Number(value=j['col_dim2'], label=l2, step=0.1, interactive=True)
                                                n2.input(lambda v, d, idx=i: update_val(d, idx, 'col_dim2', v), inputs=[n2, gen_state], outputs=[gen_state])
                                            if j['col_type'] == "Box":
                                                n3 = gr.Number(value=j['col_dim3'], label=l3, step=0.1, interactive=True)
                                                n3.input(lambda v, d, idx=i: update_val(d, idx, 'col_dim3', v), inputs=[n3, gen_state], outputs=[gen_state])
                                        
                                        gr.Markdown("Collision Origin")
                                        with gr.Row():
                                            for key in ['col_x','col_y','col_z']:
                                                label_name = key[-1].upper() # x, y, z
                                                num = gr.Number(value=j[key], label=label_name, step=0.1, interactive=True)
                                                num.input(lambda v, d, idx=i, k=key: update_val(d, idx, k, v), inputs=[num, gen_state], outputs=[gen_state])

                    gr.Button("➕ Add Joint", variant="secondary").click(add_joint, inputs=[gen_state], outputs=[gen_state])

                with gr.Column(scale=1):
                    gr.Markdown("#### Export")
                    fname = gr.Textbox(value="robot", label="File Name")
                    btn_gen = gr.Button("🚀 Generate URDF", variant="primary")
                    code = gr.Code(language="html", label="XML Preview")
                    dl = gr.File(label="Download")
                    btn_gen.click(generate_download, inputs=[gen_state, fname], outputs=[code, dl])

        # TAB 2: VISUALIZER
        with gr.Tab("👁️ Visualizer"):
            vis_state = gr.State((None, []))
            def on_upload(f): return load_urdf_to_iframe(f)
            
            with gr.Row():
                with gr.Column(scale=3):
                    vis_file = gr.File(label="Upload URDF & STL Files", file_count="multiple")
                    @gr.render(inputs=vis_state)
                    def render_vis(data):
                        html, _ = data
                        if not html: return gr.HTML("<div style='height:600px;background:#eee;display:flex;align-items:center;justify-content:center;'>Upload URDF</div>")
                        return gr.HTML(html)
                with gr.Column(scale=1):
                    gr.Markdown("### Controls")
                    @gr.render(inputs=vis_state)
                    def render_ctrl(data):
                        _, joints = data
                        if not joints: return
                        sliders = []
                        with gr.Group():
                            for j in joints:
                                ax_name = get_axis_name(j['axis'])
                                s = gr.Slider(int(j['min']), int(j['max']), value=0, step=1, label=f"{j['name']} ({ax_name})")
                                sliders.append(s)
                        js = """(...v)=>{const f=document.getElementById('vis_iframe'); if(f) f.contentWindow.postMessage(v,'*');}"""
                        for s in sliders: s.change(None, sliders, js=js)
            vis_file.change(on_upload, vis_file, vis_state)

if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=50001, allowed_paths=["."], css=css)