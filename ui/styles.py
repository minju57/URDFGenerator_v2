js_head = """
<script>
window.select_joint_js = function(index) {
    const wrapper = document.getElementById('js_bridge_input');
    if (!wrapper) { console.error("Bridge wrapper not found."); return; }
    const bridge = wrapper.querySelector('input') || wrapper.querySelector('textarea');
    if (bridge) {
        const proto = (bridge.tagName.toLowerCase() === 'textarea')
            ? window.HTMLTextAreaElement.prototype
            : window.HTMLInputElement.prototype;
        const descriptor = Object.getOwnPropertyDescriptor(proto, 'value');
        descriptor.set.call(bridge, index + '|' + Date.now());
        bridge.dispatchEvent(new Event('input', { bubbles: true }));
    } else {
        console.error("Bridge element not found inside #js_bridge_input.");
    }
}
</script>
"""

css = """
.compact-row { gap: 5px; }
.joint-card { border: 1px solid #ddd; border-radius: 8px; padding: 10px; background: #f9f9f9; margin-bottom: 10px; }
.slider-container { padding-left: 20px; border-left: 2px solid #eee; margin-top: 5px; }

svg a {
    text-decoration: none !important;
    cursor: pointer;
}

svg a text {
    text-decoration: none !important;
    font-weight: bold;
    fill: inherit;
}

svg g.node:hover polygon, svg g.node:hover rect {
    stroke: #d35400;
    stroke-width: 2px;
    fill: #ffe0b2 !important;
    transition: all 0.2s;
}

#js_bridge_input { display: none !important; }

.title-accordion button span {
    font-size: 1.3rem !important;
    font-weight: 800 !important;
    color: #2c3e50 !important;
    letter-spacing: 0.5px !important;
}
"""
