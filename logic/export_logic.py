import collections
import copy
import os

import networkx as nx
import pandas as pd

from core.urdf_generator import URDFManager
from core.urdf_importer import URDFImporter
from core.robot_renderer import RobotRenderer

generator = URDFManager()
importer = URDFImporter()
renderer = RobotRenderer()


def load_urdf_file(file_obj):
    import gradio as gr
    if file_obj is None or not file_obj:
        return [gr.update()] * 5

    new_joints, new_base_joint = importer.parse(file_obj.name)

    if not new_joints:
        return [gr.update()] * 5

    new_choices = [(f"{i}: {j['name']}", i) for i, j in enumerate(new_joints)]

    return (
        new_joints,
        0,
        gr.update(choices=new_choices, value=0),
        gr.update(choices=new_choices, value=0),
        new_base_joint,
    )


def validate_structure(joints):
    logs = []
    is_valid = True

    joint_names = [j['name'] for j in joints]
    dup_joints = [item for item, count in collections.Counter(joint_names).items() if count > 1]
    if dup_joints:
        logs.append(f"❌ [Error] Duplicate Joint Names found: {dup_joints}")
        is_valid = False

    child_names = [j['child'] for j in joints]
    dup_children = [item for item, count in collections.Counter(child_names).items() if count > 1]
    if dup_children:
        logs.append(f"❌ [Error] Child Link used multiple times: {dup_children}. A link can only be a child of ONE joint.")
        is_valid = False

    try:
        G = nx.DiGraph()
        for j in joints:
            p = j['parent'] if j['parent'] else "base_link"
            G.add_edge(p, j['child'])

        if not nx.is_directed_acyclic_graph(G):
            cycles = list(nx.simple_cycles(G))
            logs.append(f"❌ [Error] Infinite Loop (Cycle) detected in link structure: {cycles}")
            is_valid = False
    except Exception as e:
        logs.append(f"⚠️ [Warning] Graph validation failed: {str(e)}")

    return is_valid, "\n".join(logs)


def parse_inertia_file(file_obj):
    if not file_obj:
        return None
    try:
        p = file_obj.name if hasattr(file_obj, 'name') else file_obj
        df = pd.read_csv(p) if p.lower().endswith('.csv') else pd.read_excel(p)
        df.columns = [str(c).strip() for c in df.columns]
        return df.fillna(0.0).to_dict(orient='records')
    except:
        return None


def generate_download(joints, filename, base_joint, inertia_file, is_imported=False):
    valid, log_msg = validate_structure(joints)

    if not valid:
        return "\n" + log_msg, None

    if not filename:
        filename = "robot"
    if not filename.endswith(".urdf"):
        filename += ".urdf"

    try:
        joints_m = copy.deepcopy(joints)
        for j in joints_m:
            for k in ['x', 'y', 'z', 'col_dim1', 'col_dim2', 'col_dim3',
                      'col_x', 'col_y', 'col_z', 'vis_dim1', 'vis_dim2', 'vis_dim3',
                      'vis_x', 'vis_y', 'vis_z']:
                if k in j:
                    j[k] = j[k] / 1000.0

        inertia_data = parse_inertia_file(inertia_file)

        xml, path = generator.generate(
            joints_m, filename, base_joint, inertia_data,
            robot_name=filename.replace(".urdf", ""),
            is_imported=is_imported
        )

        return "\n\n" + xml, os.path.abspath(path) if path else None

    except Exception as e:
        return f"\n{str(e)}", None


def load_urdf_to_iframe(file_objs):
    if not file_objs:
        return None, []
    content, meshes = None, []
    for f in file_objs:
        p = f.name if hasattr(f, 'name') else f
        if p.lower().endswith(".urdf"):
            with open(p, "r") as r:
                content = r.read()
        else:
            meshes.append(p)
    if not content or not renderer.load_urdf(content, meshes):
        return "<h3>Error</h3>", []
    iframe = f'<iframe id="vis_iframe" srcdoc="{renderer.get_viewer_html()}" width="100%" height="900px" style="border:1px solid #ddd;" allowfullscreen></iframe>'
    return iframe, renderer.get_joint_list()
