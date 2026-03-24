def get_default_base_joint():
    return {
        'mode': 'Fixed',
        'name': 'world_to_base',
        'child': 'base_link',
        'x': 0.0, 'y': 0.0, 'z': 1000.0,
        'r': 0.0, 'p': 0.0, 'yaw': 0.0,
        'vis_type': 'Auto (Cylinder)', 'vis_mesh': 'meshes/body.STL',
        'vis_dim1': 50.0, 'vis_dim2': 100.0, 'vis_dim3': 50.0,
        'vis_x': 0.0, 'vis_y': 0.0, 'vis_z': 0.0,
        'vis_roll': 0.0, 'vis_pitch': 0.0, 'vis_yaw': 0.0,
        'col_enabled': False, 'col_type': 'Cylinder',
        'col_dim1': 50.0, 'col_dim2': 100.0, 'col_dim3': 50.0,
        'col_x': 0.0, 'col_y': 0.0, 'col_z': 0.0,
        'col_roll': 0.0, 'col_pitch': 0.0, 'col_yaw': 0.0,
    }


def apply_base_changes_logic(base_data, *args):
    import copy
    b = copy.deepcopy(base_data)
    keys = [
        'mode', 'x', 'y', 'z', 'r', 'p', 'yaw',
        'vis_type', 'vis_mesh', 'vis_dim1', 'vis_dim2', 'vis_dim3',
        'vis_x', 'vis_y', 'vis_z', 'vis_roll', 'vis_pitch', 'vis_yaw',
        'col_enabled', 'col_type', 'col_dim1', 'col_dim2', 'col_dim3',
        'col_x', 'col_y', 'col_z', 'col_roll', 'col_pitch', 'col_yaw'
    ]
    str_keys = {'mode', 'vis_type', 'vis_mesh', 'col_type', 'col_enabled'}
    for i, key in enumerate(keys):
        val = args[i]
        if key not in str_keys:
            try:
                val = float(val)
            except Exception:
                val = 0.0
        b[key] = val
    b['name'] = 'world_to_base' if b['mode'] == 'Fixed' else 'floating_base'
    b['child'] = 'base_link'
    return b


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
    if selected_idx == -3:
        # base joint 가 선택된 경우 → base_link의 자식으로 추가
        parent_link = "base_link"
    elif selected_idx <= -10:
        # 링크 노드 선택: -(i+10) 인코딩 → 해당 joint의 child link를 parent로
        joint_idx = -(selected_idx + 10)
        if 0 <= joint_idx < len(data):
            parent_link = data[joint_idx]['child']
    elif 0 <= selected_idx < len(data):
        parent_link = data[selected_idx]['child']
    elif data:
        parent_link = data[-1]['child']

    new_joint = get_default_joint(len(data), parent_name=parent_link)
    return data + [new_joint], len(data)


def delete_joint_logic(data, index):
    if 0 <= index < len(data):
        return data[:index] + data[index+1:], -1
    return data, -1


def move_joint_up(data, idx):
    if idx is None or idx <= 0 or idx >= len(data):
        return data, idx
    data[idx], data[idx-1] = data[idx-1], data[idx]
    return data, idx - 1


def move_joint_down(data, idx):
    if idx is None or idx < 0 or idx >= len(data) - 1:
        return data, idx
    data[idx], data[idx+1] = data[idx+1], data[idx]
    return data, idx + 1


def apply_changes_logic(idx, data, *args):
    if idx < 0 or idx >= len(data):
        return data

    old_child_name = data[idx]['child']

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

        if key not in ['name', 'parent', 'child', 'vis_mesh', 'vis_type', 'col_type', 'col_enabled', 'axis']:
            try:
                val = float(val)
                if key in ['r', 'p', 'yaw', 'low', 'up', 'col_roll', 'col_pitch', 'col_yaw', 'vis_roll', 'vis_pitch', 'vis_yaw']:
                    val = max(-180.0, min(180.0, val))
            except:
                val = 0.0

        data[idx][key] = val

        if key == 'low' and data[idx]['up'] < val:
            data[idx]['up'] = val
        elif key == 'up' and data[idx]['low'] > val:
            data[idx]['low'] = val

    new_child_name = data[idx]['child']
    if old_child_name != new_child_name:
        count = 0
        for joint in data:
            if joint['parent'] == old_child_name:
                joint['parent'] = new_child_name
                count += 1
        if count > 0:
            print(f"🔄 Auto-updated {count} joints: Parent changed from '{old_child_name}' to '{new_child_name}'")

    return data


def update_data_by_key(data, index, key, value):
    try:
        if index < 0 or index >= len(data):
            return data

        if key in ['name', 'parent', 'child', 'vis_mesh', 'vis_type', 'col_type', 'col_enabled', 'axis']:
            data[index][key] = value
        else:
            val = float(value)
            if key in ['r', 'p', 'yaw', 'low', 'up', 'col_roll', 'col_pitch', 'col_yaw', 'vis_roll', 'vis_pitch', 'vis_yaw']:
                val = max(-180.0, min(180.0, val))
            data[index][key] = val
            if key == 'low' and data[index]['up'] < val:
                data[index]['up'] = val
            elif key == 'up' and data[index]['low'] > val:
                data[index]['low'] = val
    except:
        pass
    return data
