import gradio as gr

from logic.joint_logic import get_default_joint, get_default_base_joint
from ui.styles import css, js_head
from ui.builder_tab import build_builder_tab
from ui.visualizer_tab import build_visualizer_tab

with gr.Blocks(title="URDF Builder", css=css, head=js_head) as demo:
    gr.Markdown("# 🤖 URDF Builder")

    gen_state = gr.State([get_default_joint(0)])
    selected_index = gr.State(-1)
    base_state = gr.State(get_default_base_joint())
    is_imported_state = gr.State(False)

    with gr.Tabs():
        with gr.Tab("🛠️ URDF Builder"):
            build_builder_tab(demo, gen_state, selected_index, base_state, is_imported_state)

        with gr.Tab("👁️ Visualizer"):
            build_visualizer_tab()

if __name__ == "__main__":
    import os
    demo.launch(
        server_name=os.environ.get("GRADIO_SERVER_NAME", "0.0.0.0"),
        server_port=int(os.environ.get("PORT", os.environ.get("GRADIO_SERVER_PORT", 7860))),
        allowed_paths=["assets"],
    )
