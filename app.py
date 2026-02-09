import gradio as gr
import os
import copy
import pandas as pd
import collections
import numpy as np
import networkx as nx
from urdf_generator import URDFManager
from robot_renderer import RobotRenderer
from urdf_importer import URDFImporter
# --- Instances ---
generator = URDFManager()
renderer = RobotRenderer()
importer = URDFImporter()

# app.py 내부

def load_urdf_file(file_obj):
    # (1) 실패 시 리턴값 (개수를 맞춰줘야 에러가 안 납니다)
    # 총 17개의 출력을 반환해야 함 (Joint 4개 + Base 13개)
    if file_obj is None or not file_obj:
        return [gr.update()] * 17 
    
    # (2) 파싱 실행
    new_joints, new_base = importer.parse(file_obj.name)
    
    if not new_joints:
        return [gr.update()] * 17

    # (3) Joint 관련 업데이트
    new_choices = [(f"{i}: {j['name']}", i) for i, j in enumerate(new_joints)]
    
    # (4) Base 관련 값 추출
    b_type = "Auto (Cylinder)"
    b_mesh_path = ""
    d1, d2, d3 = 50.0, 100.0, 50.0
    bx, by, bz = 0.0, 0.0, 0.0
    br, bp, byaw = 0.0, 0.0, 0.0
    
    if new_base:
        b_type = new_base.get('type', "Auto (Cylinder)")
        b_mesh_path = new_base.get('mesh', "")
        if new_base.get('dim'):
            d1, d2, d3 = new_base['dim']
        bx, by, bz = new_base.get('xyz', [0,0,0])
        br, bp, byaw = new_base.get('rpy', [0,0,0])

    # (5) [핵심] Base Type에 따른 UI 보임/숨김 상태 결정
    is_mesh = (b_type == "Mesh")
    is_manual = "Manual" in b_type  # Manual (Box) 등

    # (6) 리턴 (순서를 아래 upload 이벤트의 outputs와 정확히 맞춰야 함)
    return (
        # --- Joint (4개) ---
        new_joints,                         
        0,                                  
        gr.update(choices=new_choices, value=0), 
        gr.update(choices=new_choices, value=0), 
        
        # --- Base (13개) ---
        gr.update(value=b_type),           # 1. Base Type 값
        gr.update(visible=is_mesh),        # 2. [추가됨] Mesh 그룹 보임 여부
        gr.update(value=b_mesh_path),      # 3. Base Mesh 파일명 값
        gr.update(visible=is_manual),      # 4. [추가됨] Manual 그룹 보임 여부
        d1, d2, d3,                        # 5,6,7. 치수
        bx, by, bz,                        # 8,9,10. 위치
        br, bp, byaw                       # 11,12,13. 회전
    )
    
# --- 1. Helpers (Data Logic) ---
def get_default_joint(index, parent_name="base_link"):
    name = f"joint_{index}"
    child_link = f"link_{index}"
    return {
        'name': name, 'parent': parent_name, 'child': child_link,    
        'type': 'revolute', 'axis': 'Roll', 
        'x': 0.0, 'y': 0.0, 'z': 0.0, 'r': 0.0, 'p': 0.0, 'yaw': 0.0, 
        'low': -180.0, 'up': 180.0,
        # Visual Defaults
        'vis_type': 'Auto (Cylinder)', 'vis_mesh': 'meshes/link.STL',
        'vis_dim1': 40.0, 'vis_dim2': 100.0, 'vis_dim3': 40.0, 
        'vis_x': 0.0, 'vis_y': 0.0, 'vis_z': 0.0, 
        'vis_roll': 0.0, 'vis_pitch': 0.0, 'vis_yaw': 0.0,
        # Collision Defaults
        'col_enabled': False, 'col_type': 'Cylinder',
        'col_dim1': 40.0, 'col_dim2': 100.0, 'col_dim3': 40.0, 
        'col_x': 0.0, 'col_y': 0.0, 'col_z': 0.0, 
        'col_roll': 0.0, 'col_pitch': 0.0, 'col_yaw': 0.0
    }

def add_joint_logic(data, selected_idx):
    if selected_idx is None: 
        selected_idx = -1
    parent_link = "base_link"
    # 선택된 인덱스가 유효하면, 그 조인트의 Child를 부모로 설정
    if 0 <= selected_idx < len(data):
        parent_link = data[selected_idx]['child']
    # 선택된 게 없으면 마지막 조인트의 Child를 부모로 (기존 로직 유지)
    elif data:
        parent_link = data[-1]['child']
        
    new_joint = get_default_joint(len(data), parent_name=parent_link)
    # 추가된 조인트를 바로 선택하기 위해 인덱스 반환
    return data + [new_joint], len(data)

def delete_joint_logic(data, index):
    if 0 <= index < len(data):
        return data[:index] + data[index+1:], -1
    return data, -1
def move_joint_up(data, idx):
    if idx is None or idx <= 0 or idx >= len(data): 
        return data, idx
    # Swap
    data[idx], data[idx-1] = data[idx-1], data[idx]
    return data, idx - 1

# [순서 변경 로직 (아래로 이동)
def move_joint_down(data, idx):
    if idx is None or idx < 0 or idx >= len(data) - 1: 
        return data, idx
    # Swap
    data[idx], data[idx+1] = data[idx+1], data[idx]
    return data, idx + 1
# Apply 버튼 클릭 시 데이터 업데이트 & Child 이름 변경 시 의존성 자동 업데이트
def apply_changes_logic(idx, data, *args):
    if idx < 0 or idx >= len(data):
        return data

    # 1. 변경 전의 Child Link 이름 저장 (나중에 비교용)
    old_child_name = data[idx]['child']

    # 2. 데이터 매핑 (기존 로직)
    keys = [
        'name', 'parent', 'child', 'axis',
        'x', 'y', 'z', 'r', 'p', 'yaw', 'low', 'up',
        'vis_type', 'vis_mesh', 'vis_dim1', 'vis_dim2', 'vis_dim3', 
        'vis_x', 'vis_y', 'vis_z', 'vis_roll', 'vis_pitch', 'vis_yaw',
        'col_enabled', 'col_type', 'col_dim1', 'col_dim2', 'col_dim3',
        'col_x', 'col_y', 'col_z', 'col_roll', 'col_pitch', 'col_yaw'
    ]
    
    for i, key in enumerate(keys):
        val = args[i]

        if key == 'axis' and val == 'Fixed':
            data[idx]['type'] = 'fixed'
        elif key == 'axis' and val in ['Roll', 'Pitch', 'Yaw']:
            data[idx]['type'] = 'revolute'
        # 숫자형 변환 및 예외 처리
        if key not in ['name', 'parent', 'child', 'vis_mesh', 'vis_type', 'col_type', 'col_enabled', 'axis']:
            try:
                val = float(val)
                # 각도 제한 (-180 ~ 180)
                if key in ['r', 'p', 'yaw', 'low', 'up', 'col_roll', 'col_pitch', 'col_yaw', 'vis_roll', 'vis_pitch', 'vis_yaw']:
                    val = max(-180.0, min(180.0, val))
            except:
                val = 0.0
        
        data[idx][key] = val
        
        # Min/Max 논리 보정
        if key == 'low' and data[idx]['up'] < val: data[idx]['up'] = val
        elif key == 'up' and data[idx]['low'] > val: data[idx]['low'] = val

    # 3. [핵심 기능] Child Link 이름이 바뀌었다면, 이를 부모로 쓰는 다른 조인트들도 업데이트
    new_child_name = data[idx]['child']
    
    if old_child_name != new_child_name:
        # 모든 조인트를 돌면서 부모 이름을 갱신
        count = 0
        for joint in data:
            if joint['parent'] == old_child_name:
                joint['parent'] = new_child_name
                count += 1
        
        if count > 0:
            print(f"🔄 Auto-updated {count} joints: Parent changed form '{old_child_name}' to '{new_child_name}'")

    return data

def update_data_by_key(data, index, key, value):
    try:
        if index < 0 or index >= len(data): return data
        
        if key in ['name', 'parent', 'child', 'vis_mesh', 'vis_type', 'col_type', 'col_enabled', 'axis']: 
            data[index][key] = value
        else: 
            val = float(value)
            if key in ['r', 'p', 'yaw', 'low', 'up', 'col_roll', 'col_pitch', 'col_yaw', 'vis_roll', 'vis_pitch', 'vis_yaw']:
                val = max(-180.0, min(180.0, val))
            data[index][key] = val
            if key == 'low' and data[index]['up'] < val: data[index]['up'] = val
            elif key == 'up' and data[index]['low'] > val: data[index]['low'] = val
    except: pass
    return data

# --- 2. Tree Diagram (JS Link) ---
# --- 2. Tree Diagram (Modified: Parent -> Joint -> Child) ---
def draw_tree_diagram(joints, selected_idx):
    if not joints: return ""

    G = nx.DiGraph()
    
    # 1. 모든 링크(Link) 노드 먼저 생성 (동그라미 모양)
    # 중복 없이 모든 링크 이름을 수집
    all_links = set()
    all_links.add("base_link")
    for j in joints:
        all_links.add(j['parent'])
        all_links.add(j['child'])
    
    # 링크 노드 추가 (스타일: 타원형, 심플하게)
    for link in all_links:
        # Base Link는 조금 더 특별하게 표시
        if link == "base_link":
            G.add_node(
                link, label=link, shape="doubleoctagon", 
                style="filled", fillcolor="#2c3e50", 
                fontcolor="white", fontname="Arial", fontsize="10", margin="0.05"
            )
        else:
            G.add_node(
                link, label=link, shape="ellipse", 
                style="filled", fillcolor="#ecf0f1", 
                fontcolor="#7f8c8d", fontname="Arial", fontsize="10", margin="0.05"
            )

    # 2. 조인트(Joint) 노드 및 엣지 생성 (사각형 모양, 클릭 가능)
    for i, j in enumerate(joints):
        # 조인트 노드의 고유 ID (이름이 겹칠 수 있으므로 인덱스 사용 권장하나, 여기선 이름 사용)
        # 그래프 상에서 Link 이름과 겹치지 않게 하기 위해 접두사 붙임
        joint_node_id = f"J_NODE_{i}" 
        
        info_label = (
            f"{j['name']}\n"
            f"[{j['axis']}]\n"
            f"xyz: {j['x']:.1f}, {j['y']:.1f}, {j['z']:.1f}"
        )
        
        is_selected = (i == selected_idx)
        fill_color = "#e67e22" if is_selected else "#f39c12" # 오렌지 계열
        font_color = "white"
        pen_width = "2.5" if is_selected else "0"
        
        # 조인트 노드 추가 (여기가 클릭되는 버튼 역할)
        G.add_node(
            joint_node_id, 
            label=info_label, 
            shape="component",       # 부품 모양 (사각형 옆에 작은 돌기)
            style="filled,rounded", 
            fillcolor=fill_color, 
            fontcolor=font_color,
            fontname="Arial",
            fontsize="11",
            penwidth=pen_width,
            color="#d35400",
            URL=f"javascript:window.select_joint_js({i});",
            target="_self"
        )
        
        # 3. 화살표 연결: Parent Link -> Joint -> Child Link
        parent = j['parent'] if j['parent'] else "base_link"
        child = j['child']
        
        # 왼쪽 화살표 (Parent -> Joint)
        G.add_edge(parent, joint_node_id, color="#95a5a6", arrowsize="0.6")
        
        # 오른쪽 화살표 (Joint -> Child)
        G.add_edge(joint_node_id, child, color="#95a5a6", arrowsize="0.6")

    try:
        A = nx.nx_agraph.to_agraph(G)
        # rankdir='LR': 왼쪽에서 오른쪽으로 흐르게 설정
        A.graph_attr.update(rankdir='LR', nodesep='0.3', ranksep='0.4', bgcolor='transparent', fontname="Arial")
        A.edge_attr.update(penwidth='1.2')
        
        svg_code = A.draw(format='svg', prog='dot').decode('utf-8')
        return f"<div style='width: 100%; overflow-x: auto; padding: 10px; text-align: center;'>{svg_code}</div>"
    except Exception as e:
        return f"<div style='color:red'>Graphviz Error: {e}</div>"
    
# --- 3. Export Helpers ---
def validate_structure(joints):
    logs = []
    is_valid = True

    # 1. 조인트 이름 중복 체크
    joint_names = [j['name'] for j in joints]
    dup_joints = [item for item, count in collections.Counter(joint_names).items() if count > 1]
    if dup_joints:
        logs.append(f"❌ [Error] Duplicate Joint Names found: {dup_joints}")
        is_valid = False

    # 2. 자식 링크 이름 중복 체크 (하나의 링크는 하나의 부모만 가져야 함)
    child_names = [j['child'] for j in joints]
    dup_children = [item for item, count in collections.Counter(child_names).items() if count > 1]
    if dup_children:
        logs.append(f"❌ [Error] Child Link used multiple times: {dup_children}. A link can only be a child of ONE joint.")
        is_valid = False

    # 3. 순환 참조(Cycle) 체크 (NetworkX 사용)
    try:
        G = nx.DiGraph()
        for j in joints:
            # 부모 -> 자식 연결
            p = j['parent'] if j['parent'] else "base_link"
            c = j['child']
            G.add_edge(p, c)
        
        # 사이클이 있는지 확인
        if not nx.is_directed_acyclic_graph(G):
            cycles = list(nx.simple_cycles(G))
            logs.append(f"❌ [Error] Infinite Loop (Cycle) detected in link structure: {cycles}")
            is_valid = False
            
    except Exception as e:
        logs.append(f"⚠️ [Warning] Graph validation failed: {str(e)}")

    return is_valid, "\n".join(logs)

def parse_inertia_file(file_obj):
    if not file_obj: return None
    try:
        p = file_obj.name if hasattr(file_obj, 'name') else file_obj
        df = pd.read_csv(p) if p.lower().endswith('.csv') else pd.read_excel(p)
        df.columns = [str(c).strip() for c in df.columns]
        return df.fillna(0.0).to_dict(orient='records')
    except: return None

def generate_download(joints, filename, base_mode, base_type, base_mesh, b_d1, b_d2, b_d3, b_x, b_y, b_z, b_r, b_p, b_yaw, inertia_file):
    # 1. 먼저 구조 검증 수행
    valid, log_msg = validate_structure(joints)
    
    if not valid:
        error_header = "\n"
        return error_header + log_msg, None

    # 2. 검증 통과 시 기존 로직 수행
    if not filename: filename = "robot"
    if not filename.endswith(".urdf"): filename += ".urdf"
    
    try:
        joints_m = copy.deepcopy(joints)
        # (단위 변환 로직 그대로 유지)
        for j in joints_m:
            for k in ['x','y','z','col_dim1','col_dim2','col_dim3','col_x','col_y','col_z','vis_dim1','vis_dim2','vis_dim3','vis_x','vis_y','vis_z']:
                if k in j: j[k] = j[k] / 1000.0
        
        base_data = {
            'type': base_type, 
            'mesh': base_mesh, 
            'dim': [b_d1/1000.0, b_d2/1000.0, b_d3/1000.0], 
            'xyz': [b_x/1000.0, b_y/1000.0, b_z/1000.0], 
            'rpy': [b_r, b_p, b_yaw]
        }
        
        inertia_data = parse_inertia_file(inertia_file)
        
        # [수정됨] generate 함수에 base_mode 전달
        xml, path = generator.generate(
            joints_m, filename, base_data, inertia_data, 
            robot_name=filename.replace(".urdf",""),
            base_mode=base_mode  # 추가된 인자
        )
        
        success_msg = f"\n\n" + xml
        return success_msg, os.path.abspath(path) if path else None

    except Exception as e:
        return f"\n{str(e)}", None

    except Exception as e:
        return f"\n{str(e)}", None
def load_urdf_to_iframe(file_objs):
    if not file_objs: return None, []
    content, meshes = None, []
    for f in file_objs:
        p = f.name if hasattr(f, 'name') else f
        if p.lower().endswith(".urdf"): 
            with open(p,"r") as r: content=r.read()
        else: meshes.append(p)
    if not content or not renderer.load_urdf(content, meshes): return "<h3>Error</h3>", []
    return f'<iframe id="vis_iframe" srcdoc="{renderer.get_viewer_html()}" width="100%" height="900px" style="border:1px solid #ddd;" allowfullscreen></iframe>', renderer.get_joint_list()


# --- 4. JS & CSS ---
js_head = """
<script>
window.select_joint_js = function(index) {
    const bridge = document.querySelector('#js_bridge_input input');
    if (bridge) {
        const descriptor = Object.getOwnPropertyDescriptor(window.HTMLInputElement.prototype, "value");
        descriptor.set.call(bridge, index);
        bridge.dispatchEvent(new Event('input', { bubbles: true }));
    } else {
        console.error("Bridge input not found. Is it visible?");
    }
}
</script>
"""

# [수정] #js_bridge_input을 CSS로 숨김 (display: none !important)
css = """
/* 기존 스타일 */
.compact-row { gap: 5px; }
.joint-card { border: 1px solid #ddd; border-radius: 8px; padding: 10px; background: #f9f9f9; margin-bottom: 10px; }
.slider-container { padding-left: 20px; border-left: 2px solid #eee; margin-top: 5px; }

/* [여기 수정] SVG 내부 링크 스타일 제거 */
svg a { 
    text-decoration: none !important; /* 밑줄 제거 */
    cursor: pointer; 
}

/* 글자 자체에도 밑줄이 안 생기게 이중 방지 */
svg a text { 
    text-decoration: none !important; 
    font-weight: bold; 
    fill: inherit; /* 원래 색상 유지 */
}

/* 마우스를 올렸을 때의 효과 (선택 사항) */
svg g.node:hover polygon, svg g.node:hover rect { 
    stroke: #d35400; 
    stroke-width: 2px; 
    fill: #ffe0b2 !important; 
    transition: all 0.2s; 
}

/* JS Bridge 숨김 */
#js_bridge_input { display: none !important; }
"""

# --- 5. Main UI ---
with gr.Blocks(title="URDF Builder") as demo:
    gr.Markdown("# 🤖 URDF Builder")
    
    # State
    gen_state = gr.State([get_default_joint(0)]) 
    selected_index = gr.State(-1)

    # [수정된 부분] Visual UI 업데이트 로직 (KeyError 방지)
    # [수정됨] 1. Visual UI 업데이트 (빈 껍데기 반환 방지)
    def update_vis_ui(v_type):
        is_mesh = (v_type == "Mesh")
        is_auto = "Auto" in v_type
        
        # 1. Mesh Path
        show_mesh = gr.update(visible=is_mesh)
        
        # 2. Dimensions & Offsets
        if is_auto:
            return [
                show_mesh, 
                gr.update(visible=False), # Dim Group
                gr.update(visible=False), # Offset Group
                # [중요] 아래처럼 label과 visible을 명시해야 에러가 안 납니다!
                gr.update(visible=False, label="Dim 1"), 
                gr.update(visible=False, label="Dim 2"), 
                gr.update(visible=False, label="Dim 3") 
            ]
        
        show_offset = True
        show_dims = (not is_mesh) 
        
        # 초기화 (일단 다 숨기고 라벨도 기본값 부여)
        d1 = gr.update(visible=False, label="Dim 1")
        d2 = gr.update(visible=False, label="Dim 2")
        d3 = gr.update(visible=False, label="Dim 3")
        
        if show_dims:
            shape = v_type.split('(')[-1].strip(')') if '(' in v_type else v_type
            if "Sphere" in shape:
                d1 = gr.update(visible=True, label="Radius")
                d2 = gr.update(visible=False, label="Dim 2")
                d3 = gr.update(visible=False, label="Dim 3")
            elif "Cylinder" in shape:
                d1 = gr.update(visible=True, label="Radius")
                d2 = gr.update(visible=True, label="Length")
                d3 = gr.update(visible=False, label="Dim 3")
            elif "Box" in shape:
                d1 = gr.update(visible=True, label="X")
                d2 = gr.update(visible=True, label="Y")
                d3 = gr.update(visible=True, label="Z")

        return [
            show_mesh, 
            gr.update(visible=show_dims), 
            gr.update(visible=show_offset),
            d1, d2, d3
        ]

    # [수정됨] 2. Collision UI 업데이트 
    def update_col_ui(enabled, c_type):
        # Enable 체크 해제 시
        if not enabled:
            return [
                gr.update(visible=False), # Inner Group
                gr.update(visible=False), # Dim Group
                gr.update(visible=False), # Offset Group
                # [여기가 핵심] 빈 gr.update() 대신 속성을 채워서 리턴해야 합니다.
                gr.update(visible=False, label="Dim 1"), 
                gr.update(visible=False, label="Dim 2"), 
                gr.update(visible=False, label="Dim 3") 
            ]
        
        # Enable 체크 시
        d1 = gr.update(visible=False, label="Dim 1")
        d2 = gr.update(visible=False, label="Dim 2")
        d3 = gr.update(visible=False, label="Dim 3")
        
        if c_type == "Sphere":
            d1 = gr.update(visible=True, label="Radius")
            d2 = gr.update(visible=False, label="Dim 2")
            d3 = gr.update(visible=False, label="Dim 3")
        elif c_type == "Cylinder":
            d1 = gr.update(visible=True, label="Radius")
            d2 = gr.update(visible=True, label="Length")
            d3 = gr.update(visible=False, label="Dim 3")
        elif c_type == "Box":
            d1 = gr.update(visible=True, label="X")
            d2 = gr.update(visible=True, label="Y")
            d3 = gr.update(visible=True, label="Z")
            
        return [
            gr.update(visible=True),  # Inner Group
            gr.update(visible=True),  # Dim Group
            gr.update(visible=True),  # Offset Group
            d1, d2, d3
        ]
    with gr.Tabs():
        # ================= TAB 1: Builder =================
        with gr.Tab("🛠️ URDF Builder"):
            
            with gr.Accordion("📂 Import Existing URDF", open=False):
                with gr.Row():
                    urdf_uploader = gr.File(label="Upload .urdf file", file_types=[".urdf"])
                    
            # [1] 트리 맵
            gr.Markdown("### Structure Map")
            tree_plot = gr.HTML()

            # [2] 컨트롤 패널
            with gr.Row(variant="panel"):
                with gr.Column(scale=2):
                    joint_dropdown = gr.Dropdown(label="📍 Select Joint", choices=[], interactive=True)

                    with gr.Accordion("🔄 Joint Order", open=False):
                        with gr.Row():
                            btn_up = gr.Button("⬆️ Move Up", size="sm")
                            btn_down = gr.Button("⬇️ Move Down", size="sm")
                        
                        # 순서를 눈으로 확인하고 선택할 수 있는 리스트 (Radio 사용 추천)
                        order_list = gr.Radio(label="Order List", choices=[], interactive=True, type="value")
                
                with gr.Column(scale=1):
                    # [수정] visible=True로 변경하여 DOM에 생성되게 함 (CSS로 숨김 처리)
                    js_debug = gr.Number(value=-1, elem_id="js_bridge_input", visible=True, precision=0)

                with gr.Column(scale=1):
                    btn_add = gr.Button("➕ Add Joint", variant="primary")

            # [3] 에디터 & Export
            with gr.Row():
                # [좌측] 에디터
                with gr.Column(scale=2):
                    with gr.Group(visible=False) as editor_group:
                        gr.Markdown("---")
                        with gr.Row(variant="panel"):
                            with gr.Column():
                                # Header
                                with gr.Row():
                                    lbl_editor_title = gr.Markdown("### ✏️ Edit Joint")
                                    btn_apply = gr.Button("💾 Apply", variant="primary", size="sm")
                                    btn_done = gr.Button("✅ Close", variant="secondary", size="sm", scale=0)
                                    btn_del = gr.Button("✖️ Delete", variant="stop", size="sm", scale=0)

                                # Basic Info
                                with gr.Row():
                                    ed_name = gr.Textbox(label="Joint Name")
                                    ed_parent = gr.Dropdown(label="Parent Link")
                                    ed_child = gr.Textbox(label="Child Link", interactive=True)
                                
                                # Kinematics
                                with gr.Accordion("Kinematics", open=True):
                                    ed_axis = gr.Radio(["Roll","Pitch","Yaw", "Fixed"], label="Axis / Type")
                                    
                                    # [1] Offset 그룹: Fixed여도 위치는 잡아야 하므로 '항상 보임' 처리
                                    with gr.Group(visible=True) as grp_offset:
                                        with gr.Row():
                                            ed_x = gr.Number(label="Off X"); ed_y = gr.Number(label="Off Y"); ed_z = gr.Number(label="Off Z")
                                        with gr.Row():
                                            ed_r = gr.Number(label="Rot R"); ed_p = gr.Number(label="Rot P"); ed_yaw = gr.Number(label="Rot Y")
                                    
                                    # [2] Limit 그룹: Fixed일 때만 숨겨야 함
                                    with gr.Group(visible=True) as grp_limit:
                                        with gr.Row():
                                            ed_min = gr.Number(label="Min"); ed_max = gr.Number(label="Max")

                                # [함수 수정] Limit 그룹만 제어하도록 변경
                                def update_axis_visibility(val):
                                    is_fixed = (val == "Fixed")
                                    return gr.update(visible=not is_fixed)
                                
                                # Visual (Dynamic)
                                with gr.Accordion("Visual", open=False):
                                    ed_vis_type = gr.Radio(["Auto (Cylinder)", "Auto (Box)", "Manual (Cylinder)", "Manual (Box)", "Manual (Sphere)", "Mesh"], label="Shape")
                                    ed_vis_mesh = gr.Textbox(label="Mesh Path")
                                    # 치수 입력 그룹
                                    with gr.Group(visible=False) as grp_vis_dims:
                                        with gr.Row():
                                            ed_vd1 = gr.Number(label="Dim 1"); ed_vd2 = gr.Number(label="Dim 2"); ed_vd3 = gr.Number(label="Dim 3")
                                    
                                    # 오프셋 입력 그룹
                                    with gr.Group(visible=False) as grp_vis_offset:
                                        with gr.Row():
                                            ed_vx = gr.Number(label="Off X"); ed_vy = gr.Number(label="Off Y"); ed_vz = gr.Number(label="Off Z")
                                        with gr.Row():
                                            ed_vr = gr.Number(label="Rot R"); ed_vp = gr.Number(label="Rot P"); ed_vyaw = gr.Number(label="Rot Y")
                                
                                # Collision (Dynamic)
                                with gr.Accordion("Collision", open=False):
                                    ed_col_en = gr.Checkbox(label="Enable Collision")
                                    # Inner Group (Shape 선택)
                                    with gr.Group(visible=False) as col_grp_inner:
                                        ed_col_type = gr.Dropdown(["Cylinder","Box","Sphere"], label="Shape")
                                        
                                        # Collision 치수 그룹
                                        with gr.Group(visible=True) as grp_col_dims:
                                            with gr.Row():
                                                ed_cd1 = gr.Number(label="Dim 1"); ed_cd2 = gr.Number(label="Dim 2"); ed_cd3 = gr.Number(label="Dim 3")
                                        
                                        # Collision 오프셋 그룹
                                        with gr.Group(visible=True) as grp_col_offset:
                                            with gr.Row():
                                                ed_cx = gr.Number(label="Off X"); ed_cy = gr.Number(label="Off Y"); ed_cz = gr.Number(label="Off Z")
                                            with gr.Row():
                                                ed_cr = gr.Number(label="Rot R"); ed_cp = gr.Number(label="Rot P"); ed_cyaw = gr.Number(label="Rot Y")                
                                                
                # [우측] Export
                with gr.Column(scale=1):
                    gr.Markdown("### Export")
                    fname = gr.Textbox(value="my_robot", label="Robot Name")
                    
                    base_mode = gr.Radio(
                        ["Fixed Base", "Floating Base"], 
                        value="Fixed Base", 
                        label="Base Mechanics"
                    )

                    # 1. Base Type 선택
                    base_type = gr.Radio(
                        ["Auto (Cylinder)", "Manual (Cylinder)", "Manual (Box)", "Manual (Sphere)", "Mesh", "None"], 
                        value="Auto (Cylinder)", 
                        label="Visual Base Type"
                    )
                    
                    # 2. Mesh 경로 입력 그룹
                    with gr.Group(visible=False) as grp_mesh:
                        base_mesh = gr.Textbox(value="meshes/body.STL", label="Mesh Path")
                    
                    # [수정] 3. Export 입력창 그룹 (Manual일 때만 보임)
                    with gr.Group(visible=False) as grp_manual:
                        gr.Markdown("###### Dimensions")
                        with gr.Row():
                            b_d1 = gr.Number(value=50.0, label="Radius", step=1.0) 
                            b_d2 = gr.Number(value=100.0, label="Length", step=1.0)
                            b_d3 = gr.Number(value=50.0, label="Z", step=1.0, visible=False)
                        gr.Markdown("###### Offset (XYZ)")
                        with gr.Row():
                            b_x = gr.Number(value=0.0, label="X", show_label=False)
                            b_y = gr.Number(value=0.0, label="Y", show_label=False)
                            b_z = gr.Number(value=0.0, label="Z", show_label=False)
                        gr.Markdown("###### Rotation (RPY)")
                        with gr.Row():
                            b_r = gr.Number(value=0.0, label="R", show_label=False)
                            b_p = gr.Number(value=0.0, label="P", show_label=False)
                            b_yaw = gr.Number(value=0.0, label="Y", show_label=False)

                    # [수정] 3-1. Export UI 업데이트 함수
                    def update_base_ui(val):
                        is_mesh = (val == "Mesh")
                        is_manual = "Manual" in val # Auto면 False가 됨 -> 입력창 숨김
                        
                        u_d1 = gr.update(visible=False); u_d2 = gr.update(visible=False); u_d3 = gr.update(visible=False)
                        
                        if is_manual:
                            shape = "Cylinder"
                            try:
                                if "Box" in val: shape = "Box"
                                elif "Sphere" in val: shape = "Sphere"
                                elif "Cylinder" in val: shape = "Cylinder"
                            except: pass
                            
                            if shape == "Box":
                                u_d1 = gr.update(visible=True, label="X")
                                u_d2 = gr.update(visible=True, label="Y")
                                u_d3 = gr.update(visible=True, label="Z")
                            elif shape == "Cylinder":
                                u_d1 = gr.update(visible=True, label="Radius")
                                u_d2 = gr.update(visible=True, label="Length")
                                u_d3 = gr.update(visible=False, label="Dim 3")
                            elif shape == "Sphere":
                                u_d1 = gr.update(visible=True, label="Radius")
                                u_d2 = gr.update(visible=False, label="Dim 2")
                                u_d3 = gr.update(visible=False, label="Dim 3")
                        return [
                            gr.update(visible=is_mesh),   # Mesh Group
                            gr.update(visible=is_manual), # Manual Group (Dims + Offsets)
                            u_d1, u_d2, u_d3
                        ]

                    base_type.change(update_base_ui, inputs=[base_type], outputs=[grp_mesh, grp_manual, b_d1, b_d2, b_d3])
                    
                    inertia_file = gr.File(label="Inertia", file_types=[".csv", ".xlsx"])
                    btn_gen = gr.Button("🚀 Generate", variant="primary")
                    dl = gr.File(label="Download")
                    out_code = gr.Code(language="html", label="Output", lines=5)
                    
                    btn_gen.click(
                        generate_download, 
                        inputs=[
                            gen_state, fname, base_mode, base_type, base_mesh, 
                            b_d1, b_d2, b_d3, b_x, b_y, b_z, b_r, b_p, b_yaw, inertia_file
                        ], 
                        outputs=[out_code, dl]
                    )
            # --- Logic Wiring ---
            # [수정] Visual Type Change: Outputs에 그룹들을 추가
            ed_vis_type.change(update_vis_ui, inputs=[ed_vis_type], outputs=[ed_vis_mesh, grp_vis_dims, grp_vis_offset, ed_vd1, ed_vd2, ed_vd3])
            
            # [수정] Collision Change: Outputs에 그룹들을 추가
            ed_col_en.change(update_col_ui, inputs=[ed_col_en, ed_col_type], outputs=[col_grp_inner, grp_col_dims, grp_col_offset, ed_cd1, ed_cd2, ed_cd3])
            ed_col_type.change(lambda en, t: update_col_ui(en, t), inputs=[ed_col_en, ed_col_type], outputs=[col_grp_inner, grp_col_dims, grp_col_offset, ed_cd1, ed_cd2, ed_cd3])
            
            def update_dropdown_choices(data, current_idx):
                # (라벨, 인덱스) 튜플 리스트 생성
                choices = [(f"{i}: {j['name']}", i) for i, j in enumerate(data)]
                val = current_idx if current_idx >= 0 and current_idx < len(data) else None
                
                # 메인 드롭다운과 순서 변경용 리스트 두 곳 모두 업데이트
                return [
                    gr.update(choices=choices, value=val), # joint_dropdown 용
                    gr.update(choices=choices, value=val)  # order_list 용
                ]
            
            # [수정] Sync Editor: 데이터 로드 시 UI 상태 동기화
            def sync_editor(idx, data):
                if idx is None or idx < 0 or idx >= len(data):
                    return [gr.update(visible=False)] + [gr.update()] * 40 # 개수 넉넉히
                
                j = data[idx]
                parents = ["base_link"] + [d['child'] for i, d in enumerate(data) if i != idx]
                
                # UI 상태 계산
                vis_updates = update_vis_ui(j['vis_type']) 
                col_updates = update_col_ui(j['col_enabled'], j['col_type']) 

                current_axis = j['axis']
                # 만약 데이터상 type이 fixed인데 axis값은 Roll 등으로 되어있을 수 있으니 보정
                if j.get('type') == 'fixed': 
                    current_axis = 'Fixed'
                
                is_limit_visible = (current_axis != 'Fixed')

                # 리턴 리스트 구성 (Inputs 순서와 정확히 일치해야 함)
                # editor_inputs 리스트 순서:
                # 1.Group, 2.Title, 3.Name, 4.Parent, 5.Child, 6.Axis
                # 7~14.Kinematics (x,y,z,r,p,y,min,max)
                # 15.VisType, 16.VisMesh, 17.VisDimGroup, 18.VisOffGroup, 19~21.VisDims, 22~27.VisOffs
                # 28.ColEn, 29.ColInnerGrp, 30.ColType, 31.ColDimGrp, 32.ColOffGrp, 33~35.ColDims, 36~41.ColOffs
                
                return [
                    gr.update(visible=True), # 1. Group
                    gr.update(value=f"### ✏️ Editing: {j['name']}"), # 2
                    gr.update(value=j['name']), # 3
                    gr.update(choices=parents, value=j['parent']), # 4
                    gr.update(value=j['child']), # 5
                    gr.update(value=current_axis), # 6. ed_axis
                    gr.update(visible=True), 
                    # Kinematics
                    gr.update(value=j['x']), gr.update(value=j['y']), gr.update(value=j['z']), 
                    gr.update(value=j['r']), gr.update(value=j['p']), gr.update(value=j['yaw']),
                    gr.update(visible=is_limit_visible),
                    gr.update(value=j['low']), gr.update(value=j['up']),
                    # Visual
                    gr.update(value=j['vis_type']), # 15
                    gr.update(value=j['vis_mesh'], visible=vis_updates[0]['visible']), # 16
                    gr.update(visible=vis_updates[1]['visible']), # 17. Vis Dim Group
                    gr.update(visible=vis_updates[2]['visible']), # 18. Vis Offset Group
                    gr.update(value=j['vis_dim1'], visible=vis_updates[3]['visible'], label=vis_updates[3]['label']), # 19
                    gr.update(value=j['vis_dim2'], visible=vis_updates[4]['visible'], label=vis_updates[4]['label']), # 20
                    gr.update(value=j['vis_dim3'], visible=vis_updates[5]['visible'], label=vis_updates[5]['label']), # 21
                    gr.update(value=j['vis_x']), gr.update(value=j['vis_y']), gr.update(value=j['vis_z']), # 22~24
                    gr.update(value=j['vis_roll']), gr.update(value=j['vis_pitch']), gr.update(value=j['vis_yaw']), # 25~27
                    # Collision
                    gr.update(value=j['col_enabled']), # 28
                    gr.update(visible=col_updates[0]['visible']), # 29. Col Inner Group
                    gr.update(value=j['col_type']), # 30
                    gr.update(visible=col_updates[1]['visible']), # 31. Col Dim Group
                    gr.update(visible=col_updates[2]['visible']), # 32. Col Offset Group
                    gr.update(value=j['col_dim1'], visible=col_updates[3]['visible'], label=col_updates[3]['label']), # 33
                    gr.update(value=j['col_dim2'], visible=col_updates[4]['visible'], label=col_updates[4]['label']), # 34
                    gr.update(value=j['col_dim3'], visible=col_updates[5]['visible'], label=col_updates[5]['label']), # 35
                    gr.update(value=j['col_x']), gr.update(value=j['col_y']), gr.update(value=j['col_z']), # 36~38
                    gr.update(value=j['col_roll']), gr.update(value=j['col_pitch']), gr.update(value=j['col_yaw']) # 39~41
                ]
            
            # [수정] Inputs 리스트 재정의 (Group들이 추가됨)
            editor_update_targets = [
                editor_group, lbl_editor_title, ed_name, ed_parent, ed_child, ed_axis, grp_offset,
                ed_x, ed_y, ed_z, ed_r, ed_p, ed_yaw, grp_limit,ed_min, ed_max,
                ed_vis_type, ed_vis_mesh, grp_vis_dims, grp_vis_offset, ed_vd1, ed_vd2, ed_vd3, ed_vx, ed_vy, ed_vz, ed_vr, ed_vp, ed_vyaw,
                ed_col_en, col_grp_inner, ed_col_type, grp_col_dims, grp_col_offset, ed_cd1, ed_cd2, ed_cd3, ed_cx, ed_cy, ed_cz, ed_cr, ed_cp, ed_cyaw
            ]
            input_value_list = [
                ed_name, ed_parent, ed_child, ed_axis,
                ed_x, ed_y, ed_z, ed_r, ed_p, ed_yaw, ed_min, ed_max,
                ed_vis_type, ed_vis_mesh, ed_vd1, ed_vd2, ed_vd3, ed_vx, ed_vy, ed_vz, ed_vr, ed_vp, ed_vyaw,
                ed_col_en, ed_col_type, ed_cd1, ed_cd2, ed_cd3, ed_cx, ed_cy, ed_cz, ed_cr, ed_cp, ed_cyaw
            ]

            # Events
            btn_add.click(add_joint_logic, [gen_state, selected_index], [gen_state, selected_index])
            js_debug.input(lambda x: int(x), js_debug, selected_index)
            joint_dropdown.input(lambda x: x, joint_dropdown, selected_index)
            order_list.input(lambda x: x, order_list, selected_index) # [추가] 리스트 선택 시 selected_index 변경

            ed_axis.change(update_axis_visibility, inputs=ed_axis, outputs=grp_limit)

            btn_up.click(
                move_joint_up, 
                inputs=[gen_state, selected_index], 
                outputs=[gen_state, selected_index]
            )
            btn_down.click(
                move_joint_down, 
                inputs=[gen_state, selected_index], 
                outputs=[gen_state, selected_index]
            )
            selected_index.change(sync_editor, [selected_index, gen_state], editor_update_targets)
            selected_index.change(draw_tree_diagram, [gen_state, selected_index], tree_plot)

            btn_apply.click(
                apply_changes_logic, 
                inputs=[selected_index, gen_state] + input_value_list, 
                outputs=[gen_state]
            ).then( # Apply가 끝나면(then) 트리와 드롭다운을 그린다
                draw_tree_diagram, [gen_state, selected_index], tree_plot
            ).then(
                update_dropdown_choices, [gen_state, selected_index], 
                [joint_dropdown, order_list]
            )

            # Gen State가 변경될 때 (Add/Delete/Apply) 트리 업데이트
            gen_state.change(draw_tree_diagram, [gen_state, selected_index], tree_plot)
            gen_state.change(update_dropdown_choices, [gen_state, selected_index], [joint_dropdown, order_list])

            # [삭제됨] 실시간 데이터 바인딩 (.input) 제거 
            # 이제 각 컴포넌트의 .input 이벤트는 데이터(gen_state)를 건드리지 않습니다.
            # 오직 btn_apply.click 만이 데이터를 수정합니다.

            btn_done.click(lambda: -1, outputs=selected_index)
            btn_del.click(delete_joint_logic, [gen_state, selected_index], [gen_state, selected_index])

            # Init
            demo.load(draw_tree_diagram, [gen_state, selected_index], tree_plot)
            demo.load(update_dropdown_choices, [gen_state, selected_index], [joint_dropdown, order_list])

            urdf_uploader.upload(
                load_urdf_file, 
                inputs=[urdf_uploader], 
                outputs=[
                    # Joint 관련 (4개)
                    gen_state, selected_index, joint_dropdown, order_list, 
                    
                    # Base 관련 (13개 - 순서 중요!)
                    base_type, 
                    grp_mesh,      # <--- [중요] Mesh 그룹 (visible 제어용)
                    base_mesh,     # <--- Mesh 파일명 텍스트박스
                    grp_manual,    # <--- [중요] Manual 그룹 (visible 제어용)
                    b_d1, b_d2, b_d3,
                    b_x, b_y, b_z,
                    b_r, b_p, b_yaw
                ]
            )
        # ================= TAB 2: Visualizer =================
        with gr.Tab("👁️ Visualizer"):
            vis_state = gr.State((None, []))
            vis_file = gr.File(label="Upload URDF", file_count="multiple")
            with gr.Row():
                with gr.Column(scale=3):
                    @gr.render(inputs=vis_state)
                    def render_vis(data):
                        return gr.HTML(data[0]) if data[0] else gr.HTML("<div>Upload URDF to visualize</div>")
                
                with gr.Column(scale=1):
                    @gr.render(inputs=vis_state)
                    def render_ctrl(data):
                        if not data[1]: return
                        joint_list = data[1]
                        
                        # [기존] 슬라이더 값 변경 시 실행될 JS (포즈 업데이트)
                        js_update_pose = """(...args)=>{
                            const f = document.getElementById('vis_iframe');
                            if(!f) return;
                            const vals = args.filter(v => typeof v === 'number');
                            f.contentWindow.postMessage(vals, '*');
                        }"""
                        
                        # [NEW] 프레임 토글 시 실행될 JS 생성 함수 (조인트 이름 포함)
                        def get_frame_js(j_name):
                            return f"""(val) => {{
                                const f = document.getElementById('vis_iframe');
                                if(f) f.contentWindow.postMessage({{type: 'frame', name: '{j_name}', val: val}}, '*');
                            }}"""

                        # 트리 구조 분석 로직 (기존 동일)
                        children_map = {}
                        roots = []
                        all_child_links = set()
                        for j in joint_list:
                            if j['type'] == 'fixed': continue
                            p = j['parent']
                            if p not in children_map: children_map[p] = []
                            children_map[p].append(j)
                            all_child_links.add(j['child'])

                        for j in joint_list:
                            if j['type'] != 'fixed' and j['parent'] not in all_child_links:
                                roots.append(j)
                        if not roots and joint_list: roots = [j for j in joint_list if j['type'] != 'fixed'][:1]

                        slider_map = {}
                        all_sliders_list = [] # Reset 기능을 위해 모든 슬라이더 수집

                        # 재귀적으로 UI 생성
                        def render_recursive_sliders(current_joints):
                            for j in current_joints:
                                with gr.Column(variant="panel"):
                                    # 축 이름 결정
                                    ax_name = "Roll"
                                    if j['axis'] == [0,1,0]: ax_name="Pitch"
                                    elif j['axis'] == [0,0,1]: ax_name="Yaw"
                                    
                                    # [NEW] 슬라이더와 프레임 체크박스를 한 줄에 배치
                                    with gr.Row():
                                        # Slider
                                        s = gr.Slider(
                                            int(j['min']), int(j['max']), 
                                            value=0, step=1, 
                                            label=f"{j['name']} ({ax_name})",
                                            scale=4
                                        )
                                        # Frame Checkbox (공간 적게 차지하도록 설정)
                                        chk = gr.Checkbox(label="Frame", value=False, scale=1, container=False, min_width=60)
                                        
                                        # 이벤트 연결
                                        slider_map[j['name']] = s
                                        all_sliders_list.append(s)
                                        
                                        # 체크박스 변경 시 JS 전송
                                        chk.change(None, inputs=[chk], js=get_frame_js(j['name']))

                                    # 자식 노드 재귀 호출
                                    if j['child'] in children_map:
                                        with gr.Row():
                                            gr.HTML("<div style='width:15px; border-left: 2px dashed #ccc;'></div>") 
                                            with gr.Column():
                                                render_recursive_sliders(children_map[j['child']])
                        
                        gr.Markdown("### 🕹️ Control")
                        
                        # [NEW] Reset All 버튼 (맨 위에 배치)
                        btn_reset = gr.Button("Reset All Joints (0°)", variant="secondary", size="sm")
                        
                        with gr.Group():
                            render_recursive_sliders(roots)
                        
                        # [기존] 슬라이더 순서 정렬 및 이벤트 바인딩
                        ordered_sliders = []
                        for j in joint_list:
                            if j['type'] != 'fixed' and j['name'] in slider_map:
                                ordered_sliders.append(slider_map[j['name']])
                        
                        for s in ordered_sliders:
                            s.change(None, inputs=ordered_sliders, js=js_update_pose)
                            
                        # 1. 움직이는 조인트의 개수 확인
                        count = len(all_sliders_list)
                        
                        # 2. Viewer(iframe)에 0으로 채워진 배열을 보내는 JS 코드 생성
                        # (파이썬 변수 count를 JS 코드 문자열 안에 넣습니다)
                        js_reset_code = f"""() => {{
                            const f = document.getElementById('vis_iframe');
                            if(!f) return;
                            // {count}개의 0이 담긴 배열 생성
                            const zeros = new Array({count}).fill(0);
                            f.contentWindow.postMessage(zeros, '*');
                        }}"""

                        # 3. 버튼 클릭 시: Python(값 초기화) + JS(뷰어 전송) 동시 실행
                        btn_reset.click(
                            lambda: [0] * count,     # Python: 슬라이더 값을 0으로 (UI 업데이트용)
                            inputs=None, 
                            outputs=all_sliders_list,
                            js=js_reset_code         # JS: iframe에 0 배열 전송 (로봇 움직임용)
                        )

            vis_file.change(load_urdf_to_iframe, vis_file, vis_state)

if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=50001,css=css, head=js_head, share=True)