import gradio as gr

from logic.joint_logic import (
    add_joint_logic, delete_joint_logic,
    move_joint_up, move_joint_down, apply_changes_logic,
    apply_base_changes_logic
)
from logic.export_logic import load_urdf_file, generate_download
from logic.tree_diagram import draw_tree_diagram


# --- UI Update Helpers ---

def update_vis_ui(v_type):
    is_mesh = (v_type == "Mesh")
    is_auto = "Auto" in v_type
    is_none = (v_type == "None")

    show_mesh = gr.update(visible=is_mesh)

    if is_auto or is_none:
        return [
            show_mesh,
            gr.update(visible=False),
            gr.update(visible=False),
            gr.update(visible=False, label="Dim 1"),
            gr.update(visible=False, label="Dim 2"),
            gr.update(visible=False, label="Dim 3")
        ]

    show_dims = not is_mesh

    d1 = gr.update(visible=False, label="Dim 1")
    d2 = gr.update(visible=False, label="Dim 2")
    d3 = gr.update(visible=False, label="Dim 3")

    if show_dims:
        shape = v_type.split('(')[-1].strip(')') if '(' in v_type else v_type
        if "Sphere" in shape:
            d1 = gr.update(visible=True, label="Radius")
        elif "Cylinder" in shape:
            d1 = gr.update(visible=True, label="Radius")
            d2 = gr.update(visible=True, label="Length")
        elif "Box" in shape:
            d1 = gr.update(visible=True, label="X")
            d2 = gr.update(visible=True, label="Y")
            d3 = gr.update(visible=True, label="Z")

    return [
        show_mesh,
        gr.update(visible=show_dims),
        gr.update(visible=True),
        d1, d2, d3
    ]


def update_col_ui(enabled, c_type):
    if not enabled:
        return [
            gr.update(visible=False),
            gr.update(visible=False),
            gr.update(visible=False),
            gr.update(visible=False, label="Dim 1"),
            gr.update(visible=False, label="Dim 2"),
            gr.update(visible=False, label="Dim 3")
        ]

    d1 = gr.update(visible=False, label="Dim 1")
    d2 = gr.update(visible=False, label="Dim 2")
    d3 = gr.update(visible=False, label="Dim 3")

    if c_type == "Sphere":
        d1 = gr.update(visible=True, label="Radius")
    elif c_type == "Cylinder":
        d1 = gr.update(visible=True, label="Radius")
        d2 = gr.update(visible=True, label="Length")
    elif c_type == "Box":
        d1 = gr.update(visible=True, label="X")
        d2 = gr.update(visible=True, label="Y")
        d3 = gr.update(visible=True, label="Z")

    return [
        gr.update(visible=True),
        gr.update(visible=True),
        gr.update(visible=True),
        d1, d2, d3
    ]


def update_dropdown_choices(data, current_idx):
    choices = [(f"{i}: {j['name']}", i) for i, j in enumerate(data)]
    val = current_idx if current_idx >= 0 and current_idx < len(data) else None
    return [
        gr.update(choices=choices, value=val),
        gr.update(choices=choices, value=val)
    ]


# --- Tab Builder ---

def build_builder_tab(demo, gen_state, selected_index, base_state, is_imported_state):

    def update_axis_visibility(val):
        return gr.update(visible=(val != "Fixed"))

    def sync_editor(idx, data):
        if idx is None or idx < 0 or idx >= len(data):
            return [gr.update(visible=False)] + [gr.update()] * 42

        j = data[idx]
        parents = ["base_link"] + [d['child'] for i, d in enumerate(data) if i != idx]

        vis_updates = update_vis_ui(j['vis_type'])
        col_updates = update_col_ui(j['col_enabled'], j['col_type'])

        current_axis = j['axis']
        if j.get('type') == 'fixed':
            current_axis = 'Fixed'

        is_limit_visible = (current_axis != 'Fixed')

        return [
            gr.update(visible=True),
            gr.update(value=f"### ✏️ Editing: {j['name']}"),
            gr.update(value=j['name']),
            gr.update(choices=parents, value=j['parent']),
            gr.update(value=j['child']),
            gr.update(value=current_axis),
            gr.update(visible=True),
            gr.update(value=j['x']), gr.update(value=j['y']), gr.update(value=j['z']),
            gr.update(value=j['r']), gr.update(value=j['p']), gr.update(value=j['yaw']),
            gr.update(visible=is_limit_visible),
            gr.update(value=j['low']), gr.update(value=j['up']),
            gr.update(value=j['vis_type']),
            gr.update(value=j['vis_mesh'], visible=vis_updates[0]['visible']),
            gr.update(visible=vis_updates[1]['visible']),
            gr.update(visible=vis_updates[2]['visible']),
            gr.update(value=j['vis_dim1'], visible=vis_updates[3]['visible'], label=vis_updates[3]['label']),
            gr.update(value=j['vis_dim2'], visible=vis_updates[4]['visible'], label=vis_updates[4]['label']),
            gr.update(value=j['vis_dim3'], visible=vis_updates[5]['visible'], label=vis_updates[5]['label']),
            gr.update(value=j['vis_x']), gr.update(value=j['vis_y']), gr.update(value=j['vis_z']),
            gr.update(value=j['vis_roll']), gr.update(value=j['vis_pitch']), gr.update(value=j['vis_yaw']),
            gr.update(value=j['col_enabled']),
            gr.update(visible=col_updates[0]['visible']),
            gr.update(value=j['col_type']),
            gr.update(visible=col_updates[1]['visible']),
            gr.update(visible=col_updates[2]['visible']),
            gr.update(value=j['col_dim1'], visible=col_updates[3]['visible'], label=col_updates[3]['label']),
            gr.update(value=j['col_dim2'], visible=col_updates[4]['visible'], label=col_updates[4]['label']),
            gr.update(value=j['col_dim3'], visible=col_updates[5]['visible'], label=col_updates[5]['label']),
            gr.update(value=j['col_x']), gr.update(value=j['col_y']), gr.update(value=j['col_z']),
            gr.update(value=j['col_roll']), gr.update(value=j['col_pitch']), gr.update(value=j['col_yaw'])
        ]

    def sync_base_editor(idx, base_data):
        if idx != -3:
            return [gr.update(visible=False)] + [gr.update()] * 34
        b = base_data
        vis_updates = update_vis_ui(b.get('vis_type', 'Auto (Cylinder)'))
        col_updates = update_col_ui(b.get('col_enabled', False), b.get('col_type', 'Cylinder'))
        return [
            gr.update(visible=True),
            gr.update(value=b.get('mode', 'Fixed')),
            gr.update(value=b.get('x', 0.0)),
            gr.update(value=b.get('y', 0.0)),
            gr.update(value=b.get('z', 1000.0)),
            gr.update(value=b.get('r', 0.0)),
            gr.update(value=b.get('p', 0.0)),
            gr.update(value=b.get('yaw', 0.0)),
            gr.update(value=b.get('vis_type', 'Auto (Cylinder)')),
            gr.update(value=b.get('vis_mesh', ''), visible=vis_updates[0]['visible']),
            gr.update(visible=vis_updates[1]['visible']),
            gr.update(visible=vis_updates[2]['visible']),
            gr.update(value=b.get('vis_dim1', 50.0), visible=vis_updates[3]['visible'], label=vis_updates[3]['label']),
            gr.update(value=b.get('vis_dim2', 100.0), visible=vis_updates[4]['visible'], label=vis_updates[4]['label']),
            gr.update(value=b.get('vis_dim3', 50.0), visible=vis_updates[5]['visible'], label=vis_updates[5]['label']),
            gr.update(value=b.get('vis_x', 0.0)),
            gr.update(value=b.get('vis_y', 0.0)),
            gr.update(value=b.get('vis_z', 0.0)),
            gr.update(value=b.get('vis_roll', 0.0)),
            gr.update(value=b.get('vis_pitch', 0.0)),
            gr.update(value=b.get('vis_yaw', 0.0)),
            gr.update(value=b.get('col_enabled', False)),
            gr.update(visible=col_updates[0]['visible']),
            gr.update(value=b.get('col_type', 'Cylinder')),
            gr.update(visible=col_updates[1]['visible']),
            gr.update(visible=col_updates[2]['visible']),
            gr.update(value=b.get('col_dim1', 50.0), visible=col_updates[3]['visible'], label=col_updates[3]['label']),
            gr.update(value=b.get('col_dim2', 100.0), visible=col_updates[4]['visible'], label=col_updates[4]['label']),
            gr.update(value=b.get('col_dim3', 50.0), visible=col_updates[5]['visible'], label=col_updates[5]['label']),
            gr.update(value=b.get('col_x', 0.0)),
            gr.update(value=b.get('col_y', 0.0)),
            gr.update(value=b.get('col_z', 0.0)),
            gr.update(value=b.get('col_roll', 0.0)),
            gr.update(value=b.get('col_pitch', 0.0)),
            gr.update(value=b.get('col_yaw', 0.0)),
        ]

    # ---- 사용 설명서 ----
    with gr.Accordion("📖 URDF Builder 사용 설명서 (클릭하여 펼치기)", open=False):
        gr.Markdown("""
        ### 🚀 기본 워크플로우
        1. **Joint 추가/관리**: `Add Joint` 버튼으로 관절을 추가하고, `Structure Map`에서 부모-자식 관계를 확인합니다.
        2. **속성 편집**: 추가된 Joint를 선택하여 위치(Kinematics), 외형(Visual), 충돌 영역(Collision)을 설정합니다.
        3. **Base 설정**: Structure Map에서 파란색 Base Joint 노드를 클릭하면 Base 설정(Fixed/Floating, Visual, Collision)을 편집할 수 있습니다.
        4. **내보내기**: 화면 우측 하단의 `Export` 탭에서 내보내기 파일 이름과 관성(Inertia) 파일을 설정하고 `.urdf` 파일을 생성합니다.

        ---

        ### ⚙️ 주요 설정 항목 설명

        #### 1. Kinematics
        * **Axis / Type**: 관절의 회전 축을 설정합니다. (Roll: X축, Pitch: Y축, Yaw: Z축, Fixed: 고정 축)
        * **Offset (XYZ, RPY)**: 이전 링크(Parent)로부터 현재 관절이 얼마나 떨어져 있고 회전해 있는지 설정합니다.
        * **Limit (Min / Max)**: 관절이 회전할 수 있는 최소/최대 각도를 제한합니다. (Fixed 제외)

        #### 2. Visual (시각적 외형) & Collision (충돌 영역)
        * **Shape**: 실린더, 박스, 구 형태를 기본 제공하며 치수를 직접 입력할 수 있습니다.
        * **Mesh (STL/DAE)**: 직접 디자인한 3D 모델(Mesh)을 사용할 경우 `Mesh`를 선택하고 파일 경로를 입력합니다.
        * **Offset**: 조인트의 중심점과 실제 외형/충돌영역의 중심점이 다를 경우 미세 조정하는 옵션입니다.

        ---

        ### ⚖️ Inertia 파일 적용 방법
        로봇의 물리 시뮬레이션을 위해 **`.csv`** 또는 **`.xlsx`** 파일을 우측 하단 `Inertia` 업로드 창에 넣으면 자동으로 URDF에 병합됩니다.

        #### 📄 파일 작성 양식
        | link_name | mass | ixx | iyy | izz | ixy | ixz | iyz | com_x | com_y | com_z |
        |---|---|---|---|---|---|---|---|---|---|---|
        | base_link | 5.0 | 0.01 | 0.01 | 0.01 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.1 |

        > **행 순서**: base_link → joint_0의 child → joint_1의 child … 순으로 작성합니다.
        > **단위**: mass(kg), ixx/iyy/izz/ixy/ixz/iyz(kg·m²), com_x/y/z(m)
        """)
        gr.File(
            value="assets/inertia_template.xlsx",
            label="📥 Inertia 양식 다운로드 (inertia_template.xlsx)",
            interactive=False,
        )

    # ---- URDF Import ----
    with gr.Accordion("📂 Import Existing URDF", open=False):
        with gr.Row():
            urdf_uploader = gr.File(label="Upload .urdf file", file_types=[".urdf"])

    # ---- 트리 맵 ----
    gr.Markdown("### Structure Map")
    tree_plot = gr.HTML()

    # ---- 컨트롤 패널 ----
    with gr.Row(variant="panel"):
        with gr.Column(scale=2):
            joint_dropdown = gr.Dropdown(label="📍 Select Joint", choices=[], interactive=True)

            with gr.Accordion("🔄 Joint Order", open=False):
                with gr.Row():
                    btn_up = gr.Button("⬆️ Move Up", size="sm")
                    btn_down = gr.Button("⬇️ Move Down", size="sm")
                order_list = gr.Radio(label="Order List", choices=[], interactive=True, type="value")

        with gr.Column(scale=1):
            js_debug = gr.Textbox(value="-1", elem_id="js_bridge_input", visible=True)

        with gr.Column(scale=1):
            btn_add = gr.Button("➕ Add Joint", variant="primary")

    # ---- 에디터 & Export ----
    with gr.Row():
        # [좌측] 에디터
        with gr.Column(scale=2):
            # --- 일반 조인트 에디터 ---
            with gr.Group(visible=False) as editor_group:
                gr.Markdown("---")
                with gr.Row(variant="panel"):
                    with gr.Column():
                        with gr.Row():
                            lbl_editor_title = gr.Markdown("### ✏️ Edit Joint")
                            btn_apply = gr.Button("💾 Apply", variant="primary", size="sm")
                            btn_done = gr.Button("✅ Close", variant="secondary", size="sm", scale=0)
                            btn_del = gr.Button("✖️ Delete", variant="stop", size="sm", scale=0)

                        with gr.Row():
                            ed_name = gr.Textbox(label="Joint Name")
                            ed_parent = gr.Dropdown(label="Parent Link")
                            ed_child = gr.Textbox(label="Child Link", interactive=True)

                        with gr.Accordion("Kinematics", open=True, elem_classes="title-accordion"):
                            ed_axis = gr.Radio(["Roll", "Pitch", "Yaw", "Fixed"], label="Axis / Type")

                            with gr.Group(visible=True) as grp_offset:
                                with gr.Row():
                                    ed_x = gr.Number(label="Off X")
                                    ed_y = gr.Number(label="Off Y")
                                    ed_z = gr.Number(label="Off Z")
                                with gr.Row():
                                    ed_r = gr.Number(label="Rot R")
                                    ed_p = gr.Number(label="Rot P")
                                    ed_yaw = gr.Number(label="Rot Y")

                            with gr.Group(visible=True) as grp_limit:
                                with gr.Row():
                                    ed_min = gr.Number(label="Min")
                                    ed_max = gr.Number(label="Max")

                        with gr.Accordion("Visual", open=False, elem_classes="title-accordion"):
                            ed_vis_type = gr.Radio(
                                ["Auto (Cylinder)", "Auto (Box)", "Manual (Cylinder)",
                                 "Manual (Box)", "Manual (Sphere)", "Mesh"],
                                label="Shape"
                            )
                            ed_vis_mesh = gr.Textbox(label="Mesh Path")
                            with gr.Group(visible=False) as grp_vis_dims:
                                with gr.Row():
                                    ed_vd1 = gr.Number(label="Dim 1")
                                    ed_vd2 = gr.Number(label="Dim 2")
                                    ed_vd3 = gr.Number(label="Dim 3")
                            with gr.Group(visible=False) as grp_vis_offset:
                                with gr.Row():
                                    ed_vx = gr.Number(label="Off X")
                                    ed_vy = gr.Number(label="Off Y")
                                    ed_vz = gr.Number(label="Off Z")
                                with gr.Row():
                                    ed_vr = gr.Number(label="Rot R")
                                    ed_vp = gr.Number(label="Rot P")
                                    ed_vyaw = gr.Number(label="Rot Y")

                        with gr.Accordion("Collision", open=False, elem_classes="title-accordion"):
                            ed_col_en = gr.Checkbox(label="Enable Collision")
                            with gr.Group(visible=False) as col_grp_inner:
                                ed_col_type = gr.Dropdown(["Cylinder", "Box", "Sphere"], label="Shape")
                                with gr.Group(visible=True) as grp_col_dims:
                                    with gr.Row():
                                        ed_cd1 = gr.Number(label="Dim 1")
                                        ed_cd2 = gr.Number(label="Dim 2")
                                        ed_cd3 = gr.Number(label="Dim 3")
                                with gr.Group(visible=True) as grp_col_offset:
                                    with gr.Row():
                                        ed_cx = gr.Number(label="Off X")
                                        ed_cy = gr.Number(label="Off Y")
                                        ed_cz = gr.Number(label="Off Z")
                                    with gr.Row():
                                        ed_cr = gr.Number(label="Rot R")
                                        ed_cp = gr.Number(label="Rot P")
                                        ed_cyaw = gr.Number(label="Rot Y")

            # --- 베이스 조인트 에디터 ---
            with gr.Group(visible=False) as base_editor_group:
                gr.Markdown("---")
                with gr.Row(variant="panel"):
                    with gr.Column():
                        with gr.Row():
                            gr.Markdown("### 🔧 Base Joint")
                            btn_base_apply = gr.Button("💾 Apply", variant="primary", size="sm")
                            btn_base_done = gr.Button("✅ Close", variant="secondary", size="sm", scale=0)

                        base_ed_mode = gr.Radio(
                            ["Fixed", "Floating"],
                            label="Base Mode",
                            value="Fixed",
                            info="Fixed: world_to_base (fixed joint) | Floating: floating_base (floating joint)"
                        )

                        with gr.Accordion("Joint Origin (world → base_link)", open=True, elem_classes="title-accordion"):
                            with gr.Row():
                                base_ed_x = gr.Number(label="Off X")
                                base_ed_y = gr.Number(label="Off Y")
                                base_ed_z = gr.Number(label="Off Z")
                            with gr.Row():
                                base_ed_r = gr.Number(label="Rot R")
                                base_ed_p = gr.Number(label="Rot P")
                                base_ed_yaw = gr.Number(label="Rot Y")

                        with gr.Accordion("Visual (base_link)", open=False, elem_classes="title-accordion"):
                            base_ed_vis_type = gr.Radio(
                                ["Auto (Cylinder)", "Auto (Box)", "Manual (Cylinder)",
                                 "Manual (Box)", "Manual (Sphere)", "Mesh", "None"],
                                label="Shape"
                            )
                            base_ed_vis_mesh = gr.Textbox(label="Mesh Path")
                            with gr.Group(visible=False) as base_grp_vis_dims:
                                with gr.Row():
                                    base_ed_vd1 = gr.Number(label="Dim 1")
                                    base_ed_vd2 = gr.Number(label="Dim 2")
                                    base_ed_vd3 = gr.Number(label="Dim 3")
                            with gr.Group(visible=False) as base_grp_vis_offset:
                                with gr.Row():
                                    base_ed_vx = gr.Number(label="Off X")
                                    base_ed_vy = gr.Number(label="Off Y")
                                    base_ed_vz = gr.Number(label="Off Z")
                                with gr.Row():
                                    base_ed_vr = gr.Number(label="Rot R")
                                    base_ed_vp = gr.Number(label="Rot P")
                                    base_ed_vyaw = gr.Number(label="Rot Y")

                        with gr.Accordion("Collision (base_link)", open=False, elem_classes="title-accordion"):
                            base_ed_col_en = gr.Checkbox(label="Enable Collision")
                            with gr.Group(visible=False) as base_col_grp_inner:
                                base_ed_col_type = gr.Dropdown(["Cylinder", "Box", "Sphere"], label="Shape")
                                with gr.Group(visible=True) as base_grp_col_dims:
                                    with gr.Row():
                                        base_ed_cd1 = gr.Number(label="Dim 1")
                                        base_ed_cd2 = gr.Number(label="Dim 2")
                                        base_ed_cd3 = gr.Number(label="Dim 3")
                                with gr.Group(visible=True) as base_grp_col_offset:
                                    with gr.Row():
                                        base_ed_cx = gr.Number(label="Off X")
                                        base_ed_cy = gr.Number(label="Off Y")
                                        base_ed_cz = gr.Number(label="Off Z")
                                    with gr.Row():
                                        base_ed_cr = gr.Number(label="Rot R")
                                        base_ed_cp = gr.Number(label="Rot P")
                                        base_ed_cyaw = gr.Number(label="Rot Y")

        # [우측] Export
        with gr.Column(scale=1):
            gr.Markdown("### Export")
            fname = gr.Textbox(value="my_robot", label="Robot Name")
            inertia_file = gr.UploadButton("📁 Upload Inertia (.csv, .xlsx)", file_types=[".csv", ".xlsx"], variant="secondary", size="sm")
            btn_gen = gr.Button("🚀 Generate", variant="primary")
            dl = gr.File(label="Download")
            out_code = gr.Code(language="html", label="Output", lines=5)

    # ---- 이벤트 연결 ----
    editor_update_targets = [
        editor_group, lbl_editor_title, ed_name, ed_parent, ed_child, ed_axis, grp_offset,
        ed_x, ed_y, ed_z, ed_r, ed_p, ed_yaw, grp_limit, ed_min, ed_max,
        ed_vis_type, ed_vis_mesh, grp_vis_dims, grp_vis_offset, ed_vd1, ed_vd2, ed_vd3,
        ed_vx, ed_vy, ed_vz, ed_vr, ed_vp, ed_vyaw,
        ed_col_en, col_grp_inner, ed_col_type, grp_col_dims, grp_col_offset,
        ed_cd1, ed_cd2, ed_cd3, ed_cx, ed_cy, ed_cz, ed_cr, ed_cp, ed_cyaw
    ]
    input_value_list = [
        ed_name, ed_parent, ed_child, ed_axis,
        ed_x, ed_y, ed_z, ed_r, ed_p, ed_yaw, ed_min, ed_max,
        ed_vis_type, ed_vis_mesh, ed_vd1, ed_vd2, ed_vd3, ed_vx, ed_vy, ed_vz, ed_vr, ed_vp, ed_vyaw,
        ed_col_en, ed_col_type, ed_cd1, ed_cd2, ed_cd3, ed_cx, ed_cy, ed_cz, ed_cr, ed_cp, ed_cyaw
    ]

    base_editor_update_targets = [
        base_editor_group,
        base_ed_mode,
        base_ed_x, base_ed_y, base_ed_z, base_ed_r, base_ed_p, base_ed_yaw,
        base_ed_vis_type, base_ed_vis_mesh,
        base_grp_vis_dims, base_grp_vis_offset,
        base_ed_vd1, base_ed_vd2, base_ed_vd3,
        base_ed_vx, base_ed_vy, base_ed_vz, base_ed_vr, base_ed_vp, base_ed_vyaw,
        base_ed_col_en,
        base_col_grp_inner, base_ed_col_type,
        base_grp_col_dims, base_grp_col_offset,
        base_ed_cd1, base_ed_cd2, base_ed_cd3,
        base_ed_cx, base_ed_cy, base_ed_cz, base_ed_cr, base_ed_cp, base_ed_cyaw,
    ]
    base_input_value_list = [
        base_ed_mode,
        base_ed_x, base_ed_y, base_ed_z, base_ed_r, base_ed_p, base_ed_yaw,
        base_ed_vis_type, base_ed_vis_mesh,
        base_ed_vd1, base_ed_vd2, base_ed_vd3,
        base_ed_vx, base_ed_vy, base_ed_vz, base_ed_vr, base_ed_vp, base_ed_vyaw,
        base_ed_col_en, base_ed_col_type,
        base_ed_cd1, base_ed_cd2, base_ed_cd3,
        base_ed_cx, base_ed_cy, base_ed_cz, base_ed_cr, base_ed_cp, base_ed_cyaw,
    ]

    # 조인트 추가/이동/선택
    btn_add.click(add_joint_logic, [gen_state, selected_index], [gen_state, selected_index])
    js_debug.input(lambda x: int(x.split('|')[0]), js_debug, selected_index)
    joint_dropdown.input(lambda x: x, joint_dropdown, selected_index)
    order_list.input(lambda x: x, order_list, selected_index)

    btn_up.click(move_joint_up, inputs=[gen_state, selected_index], outputs=[gen_state, selected_index])
    btn_down.click(move_joint_down, inputs=[gen_state, selected_index], outputs=[gen_state, selected_index])

    # 선택 변경 → 에디터 동기화 + 트리 업데이트
    selected_index.change(sync_editor, [selected_index, gen_state], editor_update_targets)
    selected_index.change(sync_base_editor, [selected_index, base_state], base_editor_update_targets)
    selected_index.change(draw_tree_diagram, [gen_state, selected_index, base_state], tree_plot)

    # 일반 조인트 Apply/Close/Delete
    ed_axis.change(update_axis_visibility, inputs=ed_axis, outputs=grp_limit)
    ed_vis_type.change(update_vis_ui, inputs=[ed_vis_type], outputs=[ed_vis_mesh, grp_vis_dims, grp_vis_offset, ed_vd1, ed_vd2, ed_vd3])
    ed_col_en.change(update_col_ui, inputs=[ed_col_en, ed_col_type], outputs=[col_grp_inner, grp_col_dims, grp_col_offset, ed_cd1, ed_cd2, ed_cd3])
    ed_col_type.change(lambda en, t: update_col_ui(en, t), inputs=[ed_col_en, ed_col_type], outputs=[col_grp_inner, grp_col_dims, grp_col_offset, ed_cd1, ed_cd2, ed_cd3])

    btn_apply.click(
        apply_changes_logic,
        inputs=[selected_index, gen_state] + input_value_list,
        outputs=[gen_state]
    ).then(
        draw_tree_diagram, [gen_state, selected_index, base_state], tree_plot
    ).then(
        update_dropdown_choices, [gen_state, selected_index], [joint_dropdown, order_list]
    )

    btn_done.click(lambda: -1, outputs=selected_index)
    btn_del.click(delete_joint_logic, [gen_state, selected_index], [gen_state, selected_index])

    # 베이스 조인트 Apply/Close
    base_ed_vis_type.change(update_vis_ui, inputs=[base_ed_vis_type], outputs=[base_ed_vis_mesh, base_grp_vis_dims, base_grp_vis_offset, base_ed_vd1, base_ed_vd2, base_ed_vd3])
    base_ed_col_en.change(update_col_ui, inputs=[base_ed_col_en, base_ed_col_type], outputs=[base_col_grp_inner, base_grp_col_dims, base_grp_col_offset, base_ed_cd1, base_ed_cd2, base_ed_cd3])
    base_ed_col_type.change(lambda en, t: update_col_ui(en, t), inputs=[base_ed_col_en, base_ed_col_type], outputs=[base_col_grp_inner, base_grp_col_dims, base_grp_col_offset, base_ed_cd1, base_ed_cd2, base_ed_cd3])

    btn_base_apply.click(
        apply_base_changes_logic,
        inputs=[base_state] + base_input_value_list,
        outputs=[base_state]
    ).then(
        draw_tree_diagram, [gen_state, selected_index, base_state], tree_plot
    )

    btn_base_done.click(lambda: -1, outputs=selected_index)

    # 상태 변경 → 트리/드롭다운 업데이트
    gen_state.change(draw_tree_diagram, [gen_state, selected_index, base_state], tree_plot)
    gen_state.change(update_dropdown_choices, [gen_state, selected_index], [joint_dropdown, order_list])
    base_state.change(draw_tree_diagram, [gen_state, selected_index, base_state], tree_plot)

    # 생성
    btn_gen.click(
        generate_download,
        inputs=[gen_state, fname, base_state, inertia_file, is_imported_state],
        outputs=[out_code, dl]
    )

    # 초기 로드
    demo.load(draw_tree_diagram, [gen_state, selected_index, base_state], tree_plot)
    demo.load(update_dropdown_choices, [gen_state, selected_index], [joint_dropdown, order_list])

    # URDF Import
    urdf_uploader.upload(
        load_urdf_file,
        inputs=[urdf_uploader],
        outputs=[gen_state, selected_index, joint_dropdown, order_list, base_state]
    ).then(
        lambda: True, outputs=[is_imported_state]
    ).then(
        draw_tree_diagram, [gen_state, selected_index, base_state], tree_plot
    ).then(
        sync_editor, [selected_index, gen_state], editor_update_targets
    ).then(
        sync_base_editor, [selected_index, base_state], base_editor_update_targets
    )
