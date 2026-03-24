import networkx as nx


def draw_tree_diagram(joints, selected_idx, base_joint):
    G = nx.DiGraph()

    has_base = base_joint is not None and not base_joint.get('no_base')

    if has_base:
        mode = base_joint.get('mode', 'Fixed')
        root_link = 'world' if mode == 'Fixed' else 'base'
        base_joint_name = base_joint.get('name', 'world_to_base')

        # --- Root link node ---
        G.add_node(
            root_link,
            label=root_link, shape='doubleoctagon',
            style='filled', fillcolor='#2c3e50',
            fontcolor='white', fontname='Arial', fontsize='10', margin='0.05',
            penwidth='0', color='#2c3e50'
        )

        # --- Base joint node (blue) ---
        is_base_selected = (selected_idx == -3)
        base_fill = '#1a5276' if is_base_selected else '#2e86c1'
        base_pen = '2.5' if is_base_selected else '0'
        base_label = (
            f"{base_joint_name}\n"
            f"[{mode}]\n"
            f"xyz: {base_joint.get('x', 0):.1f}, {base_joint.get('y', 0):.1f}, {base_joint.get('z', 0):.1f}"
        )
        G.add_node(
            'BASE_JOINT_NODE',
            label=base_label, shape='component',
            style='filled,rounded',
            fillcolor=base_fill, fontcolor='white',
            fontname='Arial', fontsize='11',
            penwidth=base_pen, color='#1a5276',
            URL='javascript:window.select_joint_js(-3);',
            target='_self'
        )

        G.add_edge(root_link, 'BASE_JOINT_NODE', color='#7f8c8d', arrowsize='0.6')
        G.add_edge('BASE_JOINT_NODE', 'base_link', color='#7f8c8d', arrowsize='0.6')

        # --- base_link node (clickable ellipse, -2 encoding) ---
        G.add_node(
            'base_link',
            label='base_link', shape='ellipse',
            style='filled', fillcolor='#ecf0f1',
            fontcolor='#7f8c8d', fontname='Arial', fontsize='10', margin='0.05',
            penwidth='0', color='#d35400',
            URL='javascript:window.select_joint_js(-2);',
            target='_self'
        )

    # --- Regular link nodes ---
    child_set = {j['child'] for j in joints}
    all_links = set()
    for j in joints:
        all_links.add(j['parent'])
        all_links.add(j['child'])

    if has_base:
        all_links.discard('base_link')
    else:
        # base 없는 경우: 실제 루트 링크들을 root node로 표시
        root_links = all_links - child_set
        for rl in root_links:
            if not G.has_node(rl):
                G.add_node(
                    rl,
                    label=rl, shape='doubleoctagon',
                    style='filled', fillcolor='#2c3e50',
                    fontcolor='white', fontname='Arial', fontsize='10', margin='0.05',
                    penwidth='0', color='#2c3e50'
                )
        all_links -= root_links

    for link in all_links:
        link_joint_idx = next((i for i, j in enumerate(joints) if j['child'] == link), None)
        node_attrs = dict(
            label=link, shape='ellipse',
            style='filled', fillcolor='#ecf0f1',
            fontcolor='#7f8c8d', fontname='Arial', fontsize='10', margin='0.05',
            penwidth='0'
        )
        if link_joint_idx is not None:
            node_attrs['URL'] = f'javascript:window.select_joint_js({-(link_joint_idx + 10)});'
            node_attrs['target'] = '_self'
        G.add_node(link, **node_attrs)

    # --- Regular joint nodes and edges ---
    for i, j in enumerate(joints):
        joint_node_id = f'J_NODE_{i}'
        info_label = (
            f"{j['name']}\n"
            f"[{j['axis']}]\n"
            f"xyz: {j['x']:.1f}, {j['y']:.1f}, {j['z']:.1f}"
        )
        is_selected = (i == selected_idx)
        fill_color = '#e67e22' if is_selected else '#f39c12'
        pen_width = '2.5' if is_selected else '0'

        G.add_node(
            joint_node_id,
            label=info_label, shape='component',
            style='filled,rounded',
            fillcolor=fill_color, fontcolor='white',
            fontname='Arial', fontsize='11',
            penwidth=pen_width, color='#d35400',
            URL=f'javascript:window.select_joint_js({i});',
            target='_self'
        )
        parent = j['parent'] if j['parent'] else ('base_link' if has_base else 'root')
        G.add_edge(parent, joint_node_id, color='#95a5a6', arrowsize='0.6')
        G.add_edge(joint_node_id, j['child'], color='#95a5a6', arrowsize='0.6')

    try:
        A = nx.nx_agraph.to_agraph(G)
        A.graph_attr.update(rankdir='LR', nodesep='0.3', ranksep='0.4', bgcolor='transparent', fontname='Arial')
        A.edge_attr.update(penwidth='1.2')
        svg_code = A.draw(format='svg', prog='dot').decode('utf-8')
        return f"<div style='width: 100%; overflow-x: auto; padding: 10px; text-align: center;'>{svg_code}</div>"
    except Exception as e:
        return f"<div style='color:red'>Graphviz Error: {e}</div>"
