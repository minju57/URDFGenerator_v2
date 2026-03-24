import gradio as gr

from logic.export_logic import load_urdf_to_iframe


def build_visualizer_tab():
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
                if not data[1]:
                    return
                joint_list = data[1]

                js_update_pose = """(...args)=>{
                    const f = document.getElementById('vis_iframe');
                    if(!f) return;
                    const vals = args.filter(v => typeof v === 'number');
                    f.contentWindow.postMessage(vals, '*');
                }"""

                def get_frame_js(j_name):
                    return f"""(val) => {{
                        const f = document.getElementById('vis_iframe');
                        if(f) f.contentWindow.postMessage({{type: 'frame', name: '{j_name}', val: val}}, '*');
                    }}"""

                # 트리 구조 분석
                children_map = {}
                roots = []
                all_child_links = set()
                for j in joint_list:
                    if j['type'] == 'fixed':
                        continue
                    p = j['parent']
                    if p not in children_map:
                        children_map[p] = []
                    children_map[p].append(j)
                    all_child_links.add(j['child'])

                for j in joint_list:
                    if j['type'] != 'fixed' and j['parent'] not in all_child_links:
                        roots.append(j)
                if not roots and joint_list:
                    roots = [j for j in joint_list if j['type'] != 'fixed'][:1]

                slider_map = {}
                all_sliders_list = []

                def render_recursive_sliders(current_joints):
                    for j in current_joints:
                        with gr.Column(variant="panel"):
                            ax_name = "Roll"
                            if j['axis'] == [0, 1, 0]:
                                ax_name = "Pitch"
                            elif j['axis'] == [0, 0, 1]:
                                ax_name = "Yaw"

                            with gr.Row():
                                s = gr.Slider(
                                    int(j['min']), int(j['max']),
                                    value=0, step=1,
                                    label=f"{j['name']} ({ax_name})",
                                    scale=4
                                )
                                chk = gr.Checkbox(label="Frame", value=False, scale=1, container=False, min_width=60)

                                slider_map[j['name']] = s
                                all_sliders_list.append(s)
                                chk.change(None, inputs=[chk], js=get_frame_js(j['name']))

                            if j['child'] in children_map:
                                with gr.Row():
                                    gr.HTML("<div style='width:15px; border-left: 2px dashed #ccc;'></div>")
                                    with gr.Column():
                                        render_recursive_sliders(children_map[j['child']])

                gr.Markdown("### 🕹️ Control")
                btn_reset = gr.Button("Reset All Joints (0°)", variant="secondary", size="sm")

                with gr.Group():
                    render_recursive_sliders(roots)

                ordered_sliders = [
                    slider_map[j['name']]
                    for j in joint_list
                    if j['type'] != 'fixed' and j['name'] in slider_map
                ]

                for s in ordered_sliders:
                    s.change(None, inputs=ordered_sliders, js=js_update_pose)

                count = len(all_sliders_list)
                js_reset_code = f"""() => {{
                    const f = document.getElementById('vis_iframe');
                    if(!f) return;
                    const zeros = new Array({count}).fill(0);
                    f.contentWindow.postMessage(zeros, '*');
                }}"""

                btn_reset.click(
                    lambda: [0] * count,
                    inputs=None,
                    outputs=all_sliders_list,
                    js=js_reset_code
                )

    vis_file.change(load_urdf_to_iframe, vis_file, vis_state)
