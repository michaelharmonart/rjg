import maya.cmds as mc

def uv_pin_ribbon_group(rib_group):
    # Extract the base name from the given RIB group name (assumes format 'NAME_RIBgrp')
    base_name = rib_group.replace("_RIBgrp", "")
    
    # Define the ribbon object name
    ribbon_object = f"{base_name}_ribbon"
    
    # Get the child objects in the group
    children = mc.listRelatives(rib_group, children=True) or []
    
    # Filter objects that have the suffix "_grp"
    ctrl_pins = [obj for obj in children if obj.endswith("_grp")]
    
    # Sort the groups numerically (if they have _PIN_##_grp format)
    ctrl_pins.sort(key=lambda x: int(x.split("_")[-2]))  

    # Determine the number of groups
    num_pins = len(ctrl_pins)
    
    # Generate the pin locations list (normalized values between 0 and 1)
    pin_locations = [i / float(num_pins - 1) for i in range(num_pins)]
    
    # List to store world-space positions
    pin_positions = []

    # Get the world-space positions of the pins using UV coordinates (U varies, V is 0.5)
    for u in pin_locations:
        position = mc.pointOnSurface(ribbon_object, position=True, parameterU=u, parameterV=0.5)
        pin_positions.append(position)

    # Generate a list for points (formatted like 'NAME_point##_cjnt')
    point_cjnts = [f"{base_name}_point{str(i+1).zfill(2)}_cjnt" for i in range(num_pins)]

    # Get the world position and world rotation (orientation) for each joint
    world_positions = []
    world_orientations = []
    
    for cjnt in point_cjnts:
        # Get world position using xform
        position = mc.xform(cjnt, q=True, ws=True, t=True)
        world_positions.append(position)
        
        # Get world orientation (rotation in world space)
        rotation = mc.xform(cjnt, q=True, ws=True, ro=True)
        world_orientations.append(rotation)

    # Now, match the world positions and rotations of the joints to the _grps
    for i, grp in enumerate(ctrl_pins):
        # Match corresponding cjnt's world position and orientation
        target_pos = world_positions[i]
        target_rot = world_orientations[i]
        
        # Apply the world position and orientation to the _grp
        mc.xform(grp, ws=True, t=target_pos)  # Set the world position
        mc.xform(grp, ws=True, ro=target_rot)  # Set the world orientation

    # Perform UV pinning on the Ribbon and _grps
    # Iterate through the control pin _grps and pin them to the Ribbon's surface
    for i, grp in enumerate(ctrl_pins):
        u_value = pin_locations[i]  # Use the corresponding U value from the pin locations
        v_value = 0.5  # Fixed V value for UV pinning

        # Pin the control pin (group) to the ribbon surface using UV coordinates
    mc.select(ribbon_object)  # Add ribbon object to selection
    mc.select(ctrl_pins, add=True)
    mc.UVPin()  # Pin the selected objects to the ribbon

    print("UV Pinning applied to Ribbon and _grps.")

    try:
        ribbon_object_clone = ribbon_object + "_clone"
        source_history = mc.listHistory(ribbon_object_clone)
        source_skin = mc.ls(source_history, type="skinCluster")
        target_history = mc.listHistory(ribbon_object_clone)
        #target_skin = mc.ls(target_history, type="skinCluster")
        influences = mc.skinCluster(source_skin, query=True, influence=True)
        target_skin = mc.skinCluster(influences, ribbon_object, toSelectedBones=True)[0]
        mc.copySkinWeights(
        ss=source_skin[0],
        ds=target_skin[0],
        noMirror=True,
        surfaceAssociation="closestPoint",)
    except Exception as e:
        print(e)

# Example Usage
rib_group = "TEST_RIBgrp"  # Replace with your actual RIB group name
uv_pin_ribbon_group(rib_group)





from __future__ import annotations
import json
import os

import maya.cmds as mc
import maya.api.OpenMaya as om
import maya.OpenMayaUI as omui
import math

try:
    from PySide6 import QtWidgets, QtCore
except:
    from PySide2 import QtWidgets, QtCore


PART_CONFIG = {
    "arm": {
        "Axes": ["Y", "-X", "Z"],
        "Names": ["LeftArm", "LeftForeArm", "LeftHand"],
        "Delete_Last": True
    },
    "default": {
        "Axes": ["Y", "-X", "Z"],
        "Names": None,
        "Delete_Last": True
    }
}

GUIDE_PATH = r"G:\bobo\pipeline\pipeline\software\maya\scripts\rjg\build\guides\parts"

def get_maya_main_window():
    ptr = omui.MQtUtil.mainWindow()
    return wrapInstance(int(ptr), QtWidgets.QWidget)

# ------------------------------------------------------------
# HELPERS
# ------------------------------------------------------------


def get_selected_vert_ids_in_order():
    sel = om.MGlobal.getActiveSelectionList()

    verts = []
    for i in range(sel.length()):
        dag, comp = sel.getComponent(i)

        if comp.apiType() == om.MFn.kMeshVertComponent:
            fn = om.MFnSingleIndexedComponent(comp)
            verts.extend(fn.getElements())

    return verts

def get_upvect_position(mesh, vert_id):

    if vert_id is None:
        return None

    shapes = mc.listRelatives(mesh, s=True, ni=True) or []
    if shapes:
        mesh = shapes[0]

    vtx = f"{mesh}.vtx[{vert_id}]"
    if not mc.objExists(vtx):
        return None

    return mc.xform(vtx, q=True, ws=True, t=True)


def get_position_from_vert_ids(mesh, vert_ids):

    if not vert_ids:
        mc.warning("Vertex list is empty.")
        return None

    # resolve shape
    shapes = mc.listRelatives(mesh, s=True, ni=True) or []
    if shapes:
        mesh = shapes[0]

    positions = []

    for vid in vert_ids:
        vtx = f"{mesh}.vtx[{vid}]"
        if mc.objExists(vtx):
            pos = mc.xform(vtx, q=True, ws=True, t=True)
            positions.append(pos)

    if not positions:
        mc.warning("No valid vertices found on mesh.")
        return None

    count = len(positions)
    return [
        sum(p[i] for p in positions) / count
        for i in range(3)
    ]
        
def orient_primary_axis_to_next(start_pos, next_pos, primary_axis):
    """
    primary_axis: "X", "-X", "Y", "-Y", "Z", "-Z"
    Returns [rx, ry, rz]
    """

    start = om.MVector(*start_pos)
    nxt = om.MVector(*next_pos)

    aim_vec = (nxt - start).normal()

    # If axis is negative, flip aim
    if primary_axis.startswith("-"):
        aim_vec *= -1
        primary_axis = primary_axis[1:]

    # Build default basis using world up
    world_up = om.MVector(0,1,0)

    # Avoid parallel case
    if abs(aim_vec * world_up) > 0.99:
        world_up = om.MVector(1,0,0)

    side = aim_vec ^ world_up
    side.normalize()

    up = side ^ aim_vec
    up.normalize()

    # Assign aim to correct axis
    if primary_axis == "X":
        x = aim_vec
        y = up
        z = side
    elif primary_axis == "Y":
        y = aim_vec
        x = side
        z = up
    elif primary_axis == "Z":
        z = aim_vec
        x = side
        y = up

    mat = om.MMatrix([
        [x.x, x.y, x.z, 0],
        [y.x, y.y, y.z, 0],
        [z.x, z.y, z.z, 0],
        [0,   0,   0,   1]
    ])

    tm = om.MTransformationMatrix(mat)
    euler = tm.rotation()

    return [
        math.degrees(euler.x),
        math.degrees(euler.y),
        math.degrees(euler.z)
    ]


def orient_primary_axis_to_next(
    start_pos,
    next_pos,
    primary_axis,
    up_pos=None,
    up_axis="Y"
):
    """
    primary_axis : "X", "-X", "Y", "-Y", "Z", "-Z"
    up_axis      : axis to align toward up_pos (ex: "Y", "-Z")
    """

    start = om.MVector(*start_pos)
    nxt = om.MVector(*next_pos)

    aim_vec = (nxt - start).normal()

    # Handle negative primary axis
    if primary_axis.startswith("-"):
        aim_vec *= -1
        primary_axis = primary_axis[1:]

    # Initial world up
    world_up = om.MVector(0,1,0)
    if abs(aim_vec * world_up) > 0.99:
        world_up = om.MVector(1,0,0)

    side = aim_vec ^ world_up
    side.normalize()

    up = side ^ aim_vec
    up.normalize()

    # Build basis from aim only
    if primary_axis == "X":
        x, y, z = aim_vec, up, side
    elif primary_axis == "Y":
        y, x, z = aim_vec, side, up
    elif primary_axis == "Z":
        z, x, y = aim_vec, side, up

    # -------- APPLY UP VECTOR TWIST --------
    if up_pos:
        up_target = (om.MVector(*up_pos) - start).normal()

        # choose which axis is the up axis
        axis_map = {
            "X": x, "-X": -x,
            "Y": y, "-Y": -y,
            "Z": z, "-Z": -z
        }

        current_up = axis_map[up_axis]

        # Project onto plane perpendicular to aim
        def proj(v, n):
            return v - (v * n) * n

        cur_proj = proj(current_up, aim_vec).normal()
        tgt_proj = proj(up_target, aim_vec).normal()

        dot = max(min(cur_proj * tgt_proj, 1), -1)
        angle = math.acos(dot)

        cross = cur_proj ^ tgt_proj
        if cross * aim_vec < 0:
            angle *= -1

        rot = om.MQuaternion(angle, aim_vec)

        x = x.rotateBy(rot)
        y = y.rotateBy(rot)
        z = z.rotateBy(rot)

    # -------------------------------------

    mat = om.MMatrix([
        [x.x, x.y, x.z, 0],
        [y.x, y.y, y.z, 0],
        [z.x, z.y, z.z, 0],
        [0, 0, 0, 1]
    ])

    tm = om.MTransformationMatrix(mat)
    euler = tm.rotation()

    return [
        math.degrees(euler.x),
        math.degrees(euler.y),
        math.degrees(euler.z)
    ]







GUIDE_PATH = r"G:\bobo\pipeline\pipeline\software\maya\scripts\rjg\build\guides\parts"

class ReadGuidesUI(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Read Guides")
        self.setMinimumWidth(300)

        layout = QtWidgets.QVBoxLayout(self)

        self.combo = QtWidgets.QComboBox()
        layout.addWidget(QtWidgets.QLabel("Part"))
        layout.addWidget(self.combo)

        self.read_btn = QtWidgets.QPushButton("Read")
        layout.addWidget(self.read_btn)

        self.populate()

        self.read_btn.clicked.connect(self.read_selected)

    def populate(self):
        self.combo.clear()
        for f in os.listdir(GUIDE_PATH):
            if f.endswith(".json"):
                self.combo.addItem(f)

    def read_selected(self):
        file = self.combo.currentText()
        path = os.path.join(GUIDE_PATH, file)
        read_guides(path)



def show_read_guides():
    global _read_guides_ui
    try:
        _read_guides_ui.close()
    except:
        pass

    _read_guides_ui = ReadGuidesUI()
    _read_guides_ui.show()

"""get_selected_vert_ids_in_order()

list = get_selected_vert_ids_in_order()

pos = get_position_from_vert_ids('Basemesh_UBM', list)

print(pos)

jnt = mc.joint(name='test', p=pos)

pos2 = mc.xform('pos_2', q=True, ws=True, t=True)
pos3 = mc.xform('pos_3', q=True, ws=True, t=True)

rot = orient_primary_axis_to_next(
    pos,
    pos2,
    "Y",
    up_pos=pos3,
    up_axis="-X"
)


print(rot)

mc.xform(jnt, ws=True, ro=rot)"""