import maya.cmds as mc
import json
import os

# ==============================================================
#   🔹  NODE INSPECTION HELPERS
# ==============================================================

def get_node_type(nodename='node', input_channel=None):
    """
    Determines the node type, whether it is terminal, the main output channel, 
    and any extra attributes to store for a given node.
    """
    if not mc.objExists(nodename):
        raise ValueError(f"Node '{nodename}' does not exist in the scene.")

    nodetype = mc.nodeType(nodename)
    terminal = True
    output = None
    extras = []

    # Terminal types
    if nodetype in ['joint', 'blendShape']:
        terminal = True

    # Remap Value
    elif nodetype == 'remapValue':
        terminal = False
        output = 'outValue'
        extras = ['inputMax', 'outputMax', 'inputMin', 'outputMin']

    # MultiplyDivide
    elif nodetype == 'multiplyDivide':
        terminal = False
        if input_channel is None:
            raise ValueError("For multiplyDivide, input_channel must be provided.")
        axis = input_channel[-1].upper()
        output = f'output{axis}'
        extras = [f'input2{axis}']

    # Other utility types
    else:
        terminal = False

    return nodetype, terminal, output, extras


# ==============================================================
#   🔹  DOWNSTREAM + LIST HELPERS
# ==============================================================

def downstream_connections(output_connection):
    """
    Finds the direct downstream connections from a given output attribute.
    """
    if not mc.objExists(output_connection):
        raise ValueError(f"Attribute '{output_connection}' does not exist in the scene.")

    downstream_list = mc.listConnections(output_connection, plugs=True, source=False, destination=True) or []
    return len(downstream_list), downstream_list


def list_poseInterp_shapes():
    """Lists all poseInterpolator shape nodes in the current Maya scene."""
    return mc.ls(type='poseInterpolator') or []


def list_outputs(interp):
    """Lists all outputs of a given poseInterpolator shape node."""
    if not mc.objExists(interp):
        raise ValueError(f"Node '{interp}' does not exist in the scene.")
    outputs = mc.listAttr(f"{interp}.output", multi=True) or []
    return len(outputs), outputs


# ==============================================================
#   🔹  EXTRAS HANDLING
# ==============================================================

def write_extras(node, extras):
    """
    Collects values for a node's extra attributes.
    """
    extras_data = {}
    for attr in extras:
        full_attr = f"{node}.{attr}"
        if mc.objExists(full_attr):
            try:
                extras_data[attr] = mc.getAttr(full_attr)
            except Exception as e:
                print(f"⚠️ Could not read extra attr {full_attr}: {e}")
        else:
            print(f"⚠️ Extra attribute {full_attr} does not exist.")
    return extras_data


def rebuild_extras(node, extras_data):
    """
    Restores extra attributes on a node, skipping any that are connected.
    """
    for attr, value in extras_data.items():
        full_attr = f"{node}.{attr}"
        if not mc.objExists(full_attr):
            print(f"⚠️ Missing attr {full_attr}, skipping.")
            continue
        if mc.listConnections(full_attr, s=True, d=False):
            print(f"⏭️ {full_attr} already has a connection, skipping.")
            continue
        try:
            mc.setAttr(full_attr, value)
            print(f"✅ Set {full_attr} = {value}")
        except Exception as e:
            print(f"❌ Failed to set {full_attr}: {e}")


# ==============================================================
#   🔹  GRAPH WRITER
# ==============================================================

def trace_stream(input_attr, visited=None):
    """
    Recursively follow all downstream connections starting from input_attr.
    Stops at terminal nodes.
    """
    if visited is None:
        visited = set()
    if input_attr in visited:
        return None
    visited.add(input_attr)

    num_conn, downstream = downstream_connections(input_attr)
    if not downstream:
        return None

    stream_list = []

    for dest in downstream:
        dst_node, dst_plug = dest.split('.', 1)
        input_channel = dst_plug if mc.nodeType(dst_node) == "multiplyDivide" else None

        nodetype, terminal, output, extras = get_node_type(dst_node, input_channel=input_channel)
        extras_data = write_extras(dst_node, extras)

        entry = {
            "from": input_attr,
            "to": dest,
            "nodeType": nodetype,
            "terminal": terminal,
            "output": output,
            "extras": extras_data,
            "downstream": []
        }

        if not terminal and output:
            full_output_attr = f"{dst_node}.{output}"
            downstream_entries = trace_stream(full_output_attr, visited)
            if downstream_entries:
                entry["downstream"].extend(downstream_entries if isinstance(downstream_entries, list) else [downstream_entries])

        stream_list.append(entry)

    return stream_list if stream_list else None


def write_graph(filepath=r"G:/bobo/character/Rigs/Domingo/Poses/poseInterpolator_data.json"):
    """
    Build a full poseInterpolator graph and save it to JSON.
    """
    graph_data = {}
    interpList = list_poseInterp_shapes()

    for interp in interpList:
        node_data = {}
        num_outputs, outputs = list_outputs(interp)

        for i, out_attr in enumerate(outputs):
            if i < 3:
                continue
            full_attr = f"{interp}.{out_attr}"
            stream = trace_stream(full_attr)
            if stream:
                node_data[out_attr] = stream

        if node_data:
            graph_data[interp] = node_data

    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(graph_data, f, indent=4)
    print(f"✅ Pose Interpolator Graph saved to: {filepath}")


# ==============================================================
#   🔹  GRAPH REBUILDER
# ==============================================================

_created_nodes = {}

def get_unique_node_name(base_name, node_type):
    """Ensures a unique name for created nodes."""
    if not mc.objExists(base_name):
        return mc.createNode(node_type, name=base_name)

    i = 1
    while mc.objExists(f"{base_name}_{i}"):
        i += 1
    new_name = mc.createNode(node_type, name=f"{base_name}_{i}")
    print(f"⚠️ Node '{base_name}' exists. Created '{new_name}'")
    return new_name


def rebuild_stream(stream):
    """
    Recursively rebuilds a stream of connections from JSON.
    """
    for entry in stream:
        src_attr = entry['from']
        dst_attr = entry['to']
        node_type = entry['nodeType']
        terminal = entry['terminal']
        output = entry['output']
        extras = entry['extras']

        dst_node, dst_plug = dst_attr.split('.', 1)
        src_node, src_plug = src_attr.split('.', 1)

        if not mc.objExists(dst_node):
            if terminal:
                print(f"❌ Missing terminal: {dst_node}, skipping {src_attr} -> {dst_attr}")
                continue
            dst_node = get_unique_node_name(dst_node, node_type)
            _created_nodes[dst_node] = dst_node

        if not mc.objExists(src_node):
            print(f"❌ Source missing: {src_node}, skipping {src_attr} -> {dst_attr}")
            continue

        src_attr_fixed = f"{src_node}.{src_plug}"
        dst_attr_fixed = f"{dst_node}.{dst_plug}"

        try:
            if mc.objExists(src_attr_fixed) and mc.objExists(dst_attr_fixed):
                mc.connectAttr(src_attr_fixed, dst_attr_fixed, force=True)
                print(f"✅ Connected {src_attr_fixed} --> {dst_attr_fixed}")
        except Exception as e:
            print(f"❌ Failed connection {src_attr_fixed} --> {dst_attr_fixed}: {e}")

        # Restore extras
        if extras:
            rebuild_extras(dst_node, extras)

        # Recurse
        if entry['downstream']:
            rebuild_stream(entry['downstream'])


def rebuild_graph_from_json(filepath=r"G:/bobo/character/Rigs/Domingo/Poses/poseInterpolator_data.json"):
    """
    Rebuilds a poseInterpolator graph in Maya from a JSON file.
    """
    global _created_nodes
    _created_nodes = {}

    if not os.path.exists(filepath):
        print(f"❌ File not found: {filepath}")
        return

    with open(filepath, "r", encoding="utf-8") as f:
        data = json.load(f)

    print(f"📂 Loaded JSON from: {filepath}")

    for interp, outputs in data.items():
        if not mc.objExists(interp):
            print(f"❌ PoseInterpolator missing: {interp}, skipping")
            continue

        for out_attr, stream in outputs.items():
            print(f"\n🔁 Rebuilding {interp}.{out_attr} ...")
            rebuild_stream(stream)

    print("\n✅ Pose Interpolator Graph Rebuild Complete ✅")


# ==============================================================
#   🔹  Mirror Util
# ==============================================================

def mirror_side_name(name):
    """Flip _L ↔ _R in any given name."""
    if "_L" in name:
        return name.replace("_L", "_R")
    elif "_R" in name:
        return name.replace("_R", "_L")
    return name

def mirror_graph_from_json(json_path):
    """Rebuilds the mirrored network in Maya from an existing JSON file."""
    with open(json_path, 'r') as f:
        data = json.load(f)

    for interp_name, outputs in data.items():
        mirrored_interp = mirror_side_name(interp_name)
        if not mc.objExists(mirrored_interp):
            mc.createNode('poseInterpolator', name=mirrored_interp)

        for output_attr, connections in outputs.items():
            for conn in connections:
                src_attr = f"{mirrored_interp}.{output_attr}"
                dest_node = mirror_side_name(conn["to"].split('.')[0])
                dest_attr = conn["to"].split('.')[1]

                # ensure destination node exists
                if not mc.objExists(dest_node):
                    node_type = conn["nodeType"]
                    mc.createNode(node_type, name=dest_node)

                # build connection
                dest_full = f"{dest_node}.{dest_attr}"
                if mc.objExists(src_attr) and mc.objExists(dest_full):
                    try:
                        mc.connectAttr(src_attr, dest_full, f=True)
                    except:
                        pass

                # recursively build downstream
                build_downstream_recursive(conn.get("downstream", []), mirrored=True)

def build_downstream_recursive(downstream_list, mirrored=False):
    """Recursively connect downstream nodes."""
    for conn in downstream_list:
        src_node = mirror_side_name(conn["from"].split('.')[0]) if mirrored else conn["from"].split('.')[0]
        src_attr = conn["from"].split('.')[1]
        dest_node = mirror_side_name(conn["to"].split('.')[0]) if mirrored else conn["to"].split('.')[0]
        dest_attr = conn["to"].split('.')[1]
        node_type = conn["nodeType"]

        # ensure node exists
        if not mc.objExists(dest_node):
            mc.createNode(node_type, name=dest_node)

        src_full = f"{src_node}.{src_attr}"
        dest_full = f"{dest_node}.{dest_attr}"

        if mc.objExists(src_full) and mc.objExists(dest_full):
            try:
                mc.connectAttr(src_full, dest_full, f=True)
            except:
                pass

        # recurse deeper
        if conn.get("downstream"):
            build_downstream_recursive(conn["downstream"], mirrored=mirrored)
