import maya.cmds as mc
import json
import os
import re

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

    elif nodetype == 'addDL':
        terminal = False
        output = f'output'

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
#   🔹  Mirror Rebuild: L -> R (strict name mapping + translate flip)
# ==============================================================

def _map_L_to_R(name: str) -> str:
    """Map a single node name or full node.attr string from _L_ to _R_. Keeps string unchanged if no _L_ present."""
    return name.replace("_L_", "_R_") if "_L_" in name else name.replace("_L", "_R") if name.endswith("_L") else name


def _ensure_node_of_type(base_name: str, node_type: str):
    """
    Ensure a node with name base_name exists. If it does not, create it with node_type.
    Returns the node name (the existing or the newly created one).
    """
    if mc.objExists(base_name):
        return base_name
    try:
        return mc.createNode(node_type, name=base_name)
    except Exception as e:
        print(f"❌ Failed to create node {base_name} ({node_type}): {e}")
        return None


def _ensure_flip_md(dest_node: str, axis: str, modX, modY, modZ):
    """
    Ensure a multiplyDivide exists for flipping translates on dest_node.
    Name used: <dest_node>_TranslateFLIPMD
    Returns the MD node name (existing or created) and the output attribute for that axis e.g. 'mdNode.outputX'
    Also sets input2 to the mod multipliers on creation (or updates existing).
    """
    md_name = f"{dest_node}_TranslateFLIPMD"
    if not mc.objExists(md_name):
        try:
            md = mc.createNode("multiplyDivide", name=md_name)
            # set input2 to the multiplier values (we'll use input2X/Y/Z)
            mc.setAttr(f"{md}.input2X", modX)
            mc.setAttr(f"{md}.input2Y", modY)
            mc.setAttr(f"{md}.input2Z", modZ)
            print(f"          ⚙️ Created MD flip node {md_name} and set input2 to ({modX},{modY},{modZ})")
        except Exception as e:
            print(f"          ❌ Failed to create MD {md_name}: {e}")
            return None
    else:
        md = md_name
        # ensure input2 has the requested values (update so behavior predictable)
        try:
            mc.setAttr(f"{md}.input2X", modX)
            mc.setAttr(f"{md}.input2Y", modY)
            mc.setAttr(f"{md}.input2Z", modZ)
            print(f"          ⚙️ Reused MD {md_name}; ensured input2 = ({modX},{modY},{modZ})")
        except Exception as e:
            print(f"          ⚠️ Could not set MD input2 on {md_name}: {e}")
    return md


def mirror_rebuild_from_json_strict(json_path, modX=-1, modY=-1, modZ=-1):
    """
    Main entry: read JSON and rebuild mirrored networks for poseInterpolators named with _L_.
    Only processes poseInterpolators that have a mirrored counterpart (with _R_).
    Strictly maps names containing _L_ -> _R_ for node creation and connections.
    """
    print("\n🔍 Mirror rebuild (L->R strict) starting...")
    print(f"📂 Loading JSON: {json_path}")

    if not os.path.exists(json_path):
        print(f"❌ JSON not found: {json_path}")
        return

    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    total = len(data)
    print(f"🧩 Entries in JSON: {total}\n")

    # iterate only poseInterpolators that contain "_L_"
    for interp_name, outputs in data.items():
        if "_L_" not in interp_name and not interp_name.endswith("_L") and "_L" not in interp_name:
            continue  # skip entries that aren't left-side canonical names

        mirrored_interp = _map_L_to_R(interp_name)
        # require that mirrored interpolator exists in scene per your spec
        if not mc.objExists(mirrored_interp):
            print(f"  ⏭️ Mirrored interpolator not present in scene, skipping: {mirrored_interp}")
            continue

        print(f"🔹 Rebuilding for {interp_name} → {mirrored_interp}")
        # For each output attr from the left JSON, process the connections, but map all names to R
        for out_attr, conns in outputs.items():
            for conn in conns:
                # original JSON 'to' is something like "node.attr"
                orig_dest_node, orig_dest_attr = conn["to"].split(".", 1)
                # map all L -> R in node name
                dest_node_r = _map_L_to_R(orig_dest_node)
                dest_attr = orig_dest_attr  # attribute name itself doesn't need mapping

                node_type = conn.get("nodeType", "")
                # We'll only handle math and joint nodes here; skip others to avoid re-connecting upstream controls
                if not re.search(r'(joint|multiplyDivide|addDL|remapValue|blend|MD|add)', node_type, re.IGNORECASE):
                    print(f"    ⏭ Skipping nodeType {node_type} for {dest_node_r}")
                    continue

                # Ensure destination node exists (create mirrored node if missing)
                if not mc.objExists(dest_node_r):
                    created = _ensure_node_of_type(dest_node_r, node_type)
                    if created:
                        print(f"    ✅ Created mirrored node: {dest_node_r} ({node_type})")
                    else:
                        print(f"    ❌ Failed to create mirrored node: {dest_node_r} ({node_type}); skipping connection")
                        continue
                else:
                    print(f"    ⚠️ Mirrored node exists: {dest_node_r} ({node_type})")

                # Apply extras (if present) to the mirrored destination node
                extras = conn.get("extras", {})
                if extras:
                    print(f"    ✳ Applying extras to {dest_node_r}")
                    rebuild_extras(dest_node_r, extras)  # reuse your rebuild_extras helper (it safely sets attrs)

                # Build source: map left source to right (we want mirrored source attr)
                orig_src = conn["from"]  # e.g. "arm_L_01_JNT_poseInterpolatorShape.output[3]"
                src_node, src_attr = orig_src.split(".", 1)
                src_node_r = _map_L_to_R(src_node)
                src_full = f"{src_node_r}.{src_attr}"
                dest_full = f"{dest_node_r}.{dest_attr}"

                # If connection is terminal to a joint translate, insert or reuse multiplyDivide
                if conn.get("terminal") and node_type.lower() == "joint" and dest_attr.startswith("translate"):
                    # dest_node_r is the joint name on the R side; create or use MD named <dest_node_r>_TranslateFLIPMD
                    axis = dest_attr[-1].upper()
                    md_node = _ensure_flip_md(dest_node_r, axis, modX, modY, modZ)
                    if not md_node:
                        print(f"      ❌ Could not ensure MD for {dest_node_r}, skipping terminal connect")
                        continue

                    # Connect mirrored source -> md.input1X/Y/Z (depending on axis) and md.outputX -> dest.translateX
                    try:
                        # connect source to the appropriate input1 channel; do not duplicate same connection
                        input1_attr = f"{md_node}.input1{axis}"
                        if mc.objExists(src_full) and mc.objExists(input1_attr):
                            existing = mc.listConnections(input1_attr, s=True, d=False, p=True) or []
                            if any(src_full in e for e in existing):
                                print(f"      ⏭ MD input1 already connected: {src_full} → {input1_attr}")
                            else:
                                mc.connectAttr(src_full, input1_attr, force=True)
                                print(f"      🔗 Connected {src_full} → {input1_attr}")
                        else:
                            print(f"      ⚠ Missing src or md input: {src_full} / {input1_attr}")

                        # connect md.output[axis] to dest translate attr
                        output_attr = f"{md_node}.output{axis}"
                        if mc.objExists(output_attr) and mc.objExists(dest_full):
                            existing_out = mc.listConnections(dest_full, s=True, d=False, plugs=True) or []
                            if any(output_attr in e for e in existing_out):
                                print(f"      ⏭ Destination already connected from MD: {output_attr} → {dest_full}")
                            else:
                                mc.connectAttr(output_attr, dest_full, force=True)
                                print(f"      ✅ Connected {output_attr} → {dest_full}")
                        else:
                            print(f"      ⚠ Missing MD output or dest: {output_attr} / {dest_full}")

                    except Exception as e:
                        print(f"      ❌ Failed MD wiring for {dest_full}: {e}")
                        continue

                else:
                    # Non-terminal or rotation connections — connect directly (but map L->R names)
                    if mc.objExists(src_full) and mc.objExists(dest_full):
                        existing_conn = mc.listConnections(dest_full, s=True, d=False, plugs=True) or []
                        if any(src_full in e for e in existing_conn):
                            print(f"      ⏭ Already connected: {src_full} → {dest_full}")
                        else:
                            try:
                                mc.connectAttr(src_full, dest_full, force=True)
                                print(f"      ✅ Connected {src_full} → {dest_full}")
                            except Exception as e:
                                print(f"      ❌ Failed connect {src_full} → {dest_full}: {e}")
                    else:
                        print(f"      ⚠ Missing src or dest attr for direct connect: {src_full} / {dest_full}")

                # Recursively rebuild downstream for this mirrored branch, mapping names to R
                if conn.get("downstream"):
                    for downstream_entry in conn["downstream"]:
                        # build a temporary one-entry stream and call the same logic recursively by reusing this function
                        # to avoid code duplication, call a small helper that mirrors a single entry
                        _rebuild_mirrored_entry_recursive(downstream_entry, modX, modY, modZ)


    print("\n✅ Mirror rebuild complete.")


def _rebuild_mirrored_entry_recursive(entry, modX, modY, modZ):
    """
    Helper: process a single downstream JSON entry (same shape as one element in 'downstream' lists)
    This function mirrors names and recurses.
    """
    node_type = entry.get("nodeType", "")
    orig_dst_node, orig_dst_attr = entry["to"].split(".", 1)
    dest_node_r = _map_L_to_R(orig_dst_node)
    dest_attr = orig_dst_attr

    # skip node types we don't want to reconstruct
    if not re.search(r'(joint|multiplyDivide|addDL|remapValue|blend|MD|add)', node_type, re.IGNORECASE):
        print(f"        ⏭ Skipping downstream nodeType {node_type} for {dest_node_r}")
        # still recurse into deeper downstream to avoid missing nested math nodes
        for sub in entry.get("downstream", []):
            _rebuild_mirrored_entry_recursive(sub, modX, modY, modZ)
        return

    # ensure destination node
    if not mc.objExists(dest_node_r):
        created = _ensure_node_of_type(dest_node_r, node_type)
        if created:
            print(f"        ✅ Created mirrored downstream node: {dest_node_r} ({node_type})")
        else:
            print(f"        ❌ Failed to create mirrored downstream node: {dest_node_r}, skipping")
            return
    else:
        print(f"        ⚠ Mirrored downstream exists: {dest_node_r} ({node_type})")

    # apply extras
    if entry.get("extras"):
        print(f"        ✳ Applying extras to {dest_node_r}")
        rebuild_extras(dest_node_r, entry.get("extras", {}))

    # source mapping
    src_node, src_attr = entry["from"].split(".", 1)
    src_node_r = _map_L_to_R(src_node)
    src_full = f"{src_node_r}.{src_attr}"
    dest_full = f"{dest_node_r}.{dest_attr}"

    # handle terminal joint translate via MD
    if entry.get("terminal") and node_type.lower() == "joint" and dest_attr.startswith("translate"):
        axis = dest_attr[-1].upper()
        md_node = _ensure_flip_md(dest_node_r, axis, modX, modY, modZ)
        if md_node and mc.objExists(src_full):
            input1_attr = f"{md_node}.input1{axis}"
            try:
                mc.connectAttr(src_full, input1_attr, force=True)
                print(f"        🔗 Connected {src_full} → {input1_attr}")
            except Exception as e:
                print(f"        ❌ Failed connect to MD input: {e}")
            out_attr = f"{md_node}.output{axis}"
            try:
                mc.connectAttr(out_attr, dest_full, force=True)
                print(f"        ✅ Connected {out_attr} → {dest_full}")
            except Exception as e:
                print(f"        ❌ Failed connect MD output → dest: {e}")
    else:
        # normal connect
        if mc.objExists(src_full) and mc.objExists(dest_full):
            try:
                mc.connectAttr(src_full, dest_full, force=True)
                print(f"        ✅ Connected {src_full} → {dest_full}")
            except Exception as e:
                print(f"        ❌ Failed downstream connect: {e}")
        else:
            print(f"        ⚠ Missing src/dest for downstream conn: {src_full} / {dest_full}")

    # recurse
    for sub in entry.get("downstream", []):
        _rebuild_mirrored_entry_recursive(sub, modX, modY, modZ)
    
# Example usage:
#mirror_graph_from_json(r"G:/bobo/character/Rigs/Domingo/Poses/poseInterpolator_data.json", modX=-1, modY=-1, modZ=-1)
rebuild_graph_from_json()
mirror_rebuild_from_json_strict(r"G:/bobo/character/Rigs/Domingo/Poses/poseInterpolator_data.json",
                                modX=-1, modY=-1, modZ=-1)