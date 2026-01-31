from __future__ import annotations
import json
import os
import maya.cmds as mc
import maya.api.OpenMaya as om

# ---------------- PYQT FALLBACK ----------------
try:
    from PySide6 import QtWidgets, QtCore
    from shiboken6 import wrapInstance
except:
    from PySide2 import QtWidgets, QtCore
    from shiboken2 import wrapInstance

import maya.OpenMayaUI as omui

# ---------------- PATH ----------------

GUIDE_PATH = r"G:\bobo\pipeline\pipeline\software\maya\scripts\rjg\build\guides\parts"

# ---------------- CONFIG ----------------

PART_CONFIG = {
    "arm": {
        "Axes": ["Y", "-Z", "X"],
        "Names": ["LeftArm", "LeftForeArm", "LeftHand"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":None

    },
    "default": {
        "Axes": ["Y", "-X", "Z"],
        "Names": None,
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "clavicle": {
        "Axes": ["Y", "-Z", "X"],
        "Names": ["LeftShoulder"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":None
    },

    "foot": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftFoot", "LeftToeBase", "LeftToe_End"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":["X", "leg"]
    },

    "indexfinger": {
        "Axes": ["Y", "-X", "Z"],
        "Names": ["LeftHandIndex0", "LeftHandIndex1", "LeftHandIndex2", "LeftHandIndex3", "LeftHandIndex4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":None
    },

    "middlefinger": {
        "Axes": ["Y", "-X", "Z"],
        "Names": ["LeftHandMiddle0", "LeftHandMiddle1", "LeftHandMiddle2", "LeftHandMiddle3", "LeftHandMiddle4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":None
    },

    "ringfinger": {
        "Axes": ["Y", "-X", "Z"],
        "Names": ["LeftHandRing0", "LeftHandRing1", "LeftHandRing2", "LeftHandRing3", "LeftHandRing4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":None
    },

    "pinkyfinger": {
        "Axes": ["Y", "-X", "Z"],
        "Names": ["LeftHandPinky0", "LeftHandPinky1", "LeftHandPinky2", "LeftHandPinky3", "LeftHandPinky4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":None
    },

    "thumbfinger": {
        "Axes": ["Y", "-X", "Z"],
        "Names": ["LeftHandThumb1", "LeftHandThumb2", "LeftHandThumb3", "LeftHandThumb4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":None
    },

    "leg": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["LeftUpLeg", "LeftLeg"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":["X", "leg"]
    },

    "spine": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Hips", "Spine", "Spine1", "Spine2", ],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":None
    },

    "neck": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Neck", "Neck1", "Neck2", "Head", "HeadTop_End"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":None
    },

    "UEmouth": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Mouth_M_UpperLip_01", "Mouth_L_UpperLip_02", "Mouth_L_UpperLip_03", "Mouth_L_UpperLip_04", "Mouth_L_UpperLip_05", "Mouth_L_CornerLip", "Mouth_L_LowerLip_05", "Mouth_L_LowerLip_04", "Mouth_L_LowerLip_03", "Mouth_L_LowerLip_02", "Mouth_M_LowerLip_01"],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEeyeball": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Eye_L_EyeCenterPivot", "Eye_L_Aim",],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEeyeiris": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Eye_L_Iris_01", "Eye_L_Iris_02", "Eye_L_Iris_03", "Eye_L_Iris_04", "Eye_L_Iris_05", "Eye_L_Iris_06", "Eye_L_Iris_07", "Eye_L_Iris_08", "Eye_L_Iris_09", "Eye_L_Iris_10", "Eye_L_Iris_11", "Eye_L_Iris_12", "Eye_L_Iris_13", "Eye_L_Iris_14", "Eye_L_Iris_15", "Eye_L_Iris_16"],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEeyepupil": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Eye_L_Pupil_01", "Eye_L_Pupil_02", "Eye_L_Pupil_03", "Eye_L_Pupil_04", "Eye_L_Pupil_05", "Eye_L_Pupil_06", "Eye_L_Pupil_07", "Eye_L_Pupil_08", "Eye_L_Pupil_09", "Eye_L_Pupil_10", "Eye_L_Pupil_11", "Eye_L_Pupil_12", "Eye_L_Pupil_13", "Eye_L_Pupil_14", "Eye_L_Pupil_15", "Eye_L_Pupil_16"],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEeyelid": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['Eye_L_Eyelid_InnerCorner', 'Eye_L_Eyelid_InnerUpper01', 'Eye_L_Eyelid_Upper', 'Eye_L_Eyelid_OuterUpper01', 'Eye_L_Eyelid_OuterCorner', 'Eye_L_Eyelid_OuterLower01', 'Eye_L_Eyelid_Lower', 'Eye_L_Eyelid_InnerLower01'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEeyesocket": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['Eye_L_Socket_InnerCorner', 'Eye_L_Socket_InnerUpper01', 'Eye_L_Socket_Upper', 'Eye_L_Socket_OuterUpper01', 'Eye_L_Socket_OuterCorner', 'Eye_L_Socket_OuterLower01', 'Eye_L_Socket_Lower', 'Eye_L_Socket_InnerLower01'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEbase": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['UpperHead_guide', 'LowerHead_guide'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEbase": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['UpperHead_guide', 'LowerHead_guide'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEbotteeth": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['botTeeth_L_Sub_03', 'botTeeth_L_Sub_02', 'botTeeth_M_Sub_01'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEtopteeth": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['topTeeth_L_Sub_03', 'topTeeth_L_Sub_02', 'topTeeth_M_Sub_01'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEteeth": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['topTeeth', 'botTeeth',],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEbrow": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['Brow_L_Crease', 'Brow_L_01_Upper', 'Brow_L_01', 'Brow_L_01_Lower', 'Brow_L_02_Upper', 'Brow_L_02', 'Brow_L_02_Lower', 'Brow_L_01_Major', 'Brow_L_03_Upper', 'Brow_L_03', 'Brow_L_03_Lower', 'Brow_L_02_Major', 'Brow_L_04_Upper', 'Brow_L_04', 'Brow_L_04_Lower', 'Brow_L_05_Upper', 'Brow_L_05', 'Brow_L_05_Lower'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEbrow": { #
        "Axes": ["Y", "Z", "X"],
        "Names": ['Brow_L_Crease', 'Brow_L_01_Upper', 'Brow_L_01', 'Brow_L_01_Lower', 'Brow_L_02_Upper', 'Brow_L_02', 'Brow_L_02_Lower', 'Brow_L_01_Major', 'Brow_L_03_Upper', 'Brow_L_03', 'Brow_L_03_Lower', 'Brow_L_02_Major', 'Brow_L_04_Upper', 'Brow_L_04', 'Brow_L_04_Lower', 'Brow_L_05_Upper', 'Brow_L_05', 'Brow_L_05_Lower'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEcheek": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Cheek_L_NLFold_01_inner', 'Cheek_L_NLFold_01_outer', 'Cheek_L_NLFold_02_inner', 'Cheek_L_NLFold_02_outer', 'Cheek_L_NLFold_03_inner', 'Cheek_L_NLFold_03_outer', 'Cheek_L_NLFold_04_inner', 'Cheek_L_NLFold_04_outer', 'Cheek_L_NLFold_05_inner', 'Cheek_L_NLFold_05_outer', 'Cheek_L_NLFold_04', 'Cheek_L_Puff', 'Cheek_L_CheekBone'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },

    "UEear": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Ear_L_Root', 'Ear_L_Upper', 'Ear_L_Outer', 'Ear_L_Lower'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    }, 

    "UEjaw": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Jaw_M_root', 'Jaw_M_ee',],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    }, 

    "UEjaw": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Jaw_M_root', 'Jaw_M_ee',],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    }, 

    "UEjawextras": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Jaw_M_larynx', 'Jaw_M_Chin',],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    }, 

    "UEnose": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Nose_M_NoseBridge', 'Nose_M_Tip', 'Nose_L_UpperCorner', 'Nose_L_Nostril_Outer', 'Nose_L_Nostril', 'Nose_M_Nostril_Inner'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    }, 

    "UEnosebase": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Nose_M_NoseRoot'],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    }, 

    "UEtongue": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Tongue_01', 'Tongue_02', 'Tongue_03', 'Tongue_04', 'Tongue_05', 'Tongue_06'],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    }, 

    "UEmouthcenter": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Mouth_M_center'],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":None
    },    
    
}

# ---------------- HELPERS ----------------

def get_maya_main_window():
    ptr = omui.MQtUtil.mainWindow()
    return wrapInstance(int(ptr), QtWidgets.QWidget)

def get_position_from_vert_ids(mesh, vert_ids):

    shapes = mc.listRelatives(mesh, s=True, ni=True) or []
    if shapes:
        mesh = shapes[0]

    positions = []

    for vid in vert_ids:
        vtx = f"{mesh}.vtx[{vid}]"
        if mc.objExists(vtx):
            positions.append(mc.xform(vtx, q=True, ws=True, t=True))

    if not positions:
        return None

    return [
        sum(p[i] for p in positions) / len(positions)
        for i in range(3)
    ]

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

def orient_joint_primary(jnt, start_pos, next_pos, primary_axis, up_pos=None, up_axis=None):
    start = om.MVector(start_pos)
    end = om.MVector(next_pos)

    aim = (end - start).normalize()

    axis_map = {
        "X": om.MVector(1,0,0),
        "-X": om.MVector(-1,0,0),
        "Y": om.MVector(0,1,0),
        "-Y": om.MVector(0,-1,0),
        "Z": om.MVector(0,0,1),
        "-Z": om.MVector(0,0,-1)
    }

    local_primary = axis_map[primary_axis]

    rot = local_primary.rotateTo(aim)

    tm = om.MTransformationMatrix()
    tm.setRotation(rot)

    if up_pos and up_axis:
        up_dir = (om.MVector(up_pos) - start).normalize()
        cur_up = axis_map[up_axis].rotateBy(rot)
        twist_axis = aim
        twist = cur_up.angle(up_dir)
        cross = cur_up ^ up_dir
        if cross * twist_axis < 0:
            twist *= -1
        tm.rotateBy(om.MQuaternion(twist, twist_axis), om.MSpace.kWorld)

    # FIXED: remove .asEulerRotation()
    euler = tm.rotation()

    mc.xform(jnt, ws=True, ro=[
        om.MAngle(euler.x).asDegrees(),
        om.MAngle(euler.y).asDegrees(),
        om.MAngle(euler.z).asDegrees()
    ])

# ---------------- MAIN READER ----------------

def read_type(json_file):
    with open(json_file, "r") as f:
        data = json.load(f)

    part = data["part"]
    part_type = data["type"]
    if part_type == "chain":
        read_chain_guides(json_file)
    elif part_type == "sequence":
        read_seg_guides(json_file)


def read_chain_guides(json_file):

    with open(json_file, "r") as f:
        data = json.load(f)

    part = data["part"]
    part_type = data["type"]
    parent = data.get("parent")

    cfg = PART_CONFIG.get(part, PART_CONFIG["default"])

    axes = cfg["Axes"]
    names = cfg["Names"]
    delete_last = cfg["Delete_Last"]
    BuildParent = cfg["BuildParent"]
    Guide_Type = cfg ["Guide_Type"]
    Bake_To_Orient = cfg["Bake_To_Orient"]

    guides = data["guides"]

    built = []

    keys = sorted(guides.keys())

    pre_guide = None

    for i, gname in enumerate(keys):

        if delete_last and i == len(keys)-1:
            break

        g = guides[gname]

        mesh = g["mesh"]
        vert_list = g["vert_list"]
        offset = g["offset"]
        if "rotoffset" in g:
            rotoffset = g["rotoffset"]
        else:
            rotoffset = None
        up_id = g["upvect"]

        pos = get_position_from_vert_ids(mesh, vert_list)
        if not pos:
            continue

        pos = [pos[0]+offset[0], pos[1]+offset[1], pos[2]+offset[2]]

        if names and i < len(names):
            jnt_name = names[i]
        else:
            jnt_name = f"{part}_guide_{i+1:02d}"

        mc.select(clear=True)
        if Guide_Type == 'Loc':
            jnt = mc.spaceLocator(name=jnt_name)[0]
        else:
            jnt = mc.joint(name=jnt_name)
        mc.xform(jnt, ws=True, t=pos)

        built.append(jnt)

        up_pos = get_upvect_position(mesh, up_id)
        """if up_pos:
            loc_name = f"{jnt}_UPVECT"
            if mc.objExists(loc_name):
                mc.delete(loc_name)
            loc = mc.spaceLocator(name=loc_name)[0]
            mc.xform(loc, ws=True, t=up_pos)"""

        if i < len(keys)-1:

            next_g = guides[keys[i+1]]
            next_pos = get_position_from_vert_ids(next_g["mesh"], next_g["vert_list"])

            if next_pos:
                orient_joint_primary(jnt, pos, next_pos, axes[0], up_pos, axes[1])
        if rotoffset:
            rot = mc.getAttr(jnt + ".rotate")[0]
            jo  = rotoffset

            mc.setAttr(
                jnt + ".rotate",
                jo[0] + rot[0],
                jo[1] + rot[1],
                jo[2] + rot[2]
            )


        if Guide_Type == 'Joints' and Bake_To_Orient == True:
            rot = mc.getAttr(jnt + ".rotate")[0]
            jo  = mc.getAttr(jnt + ".jointOrient")[0]

            mc.setAttr(
                jnt + ".jointOrient",
                jo[0] + rot[0],
                jo[1] + rot[1],
                jo[2] + rot[2]
            )

            mc.setAttr(jnt + ".rotate", 0, 0, 0)


        if pre_guide:
            mc.parent(jnt, pre_guide)
        pre_guide = jnt


    for i in range(1, len(built)):
        mc.parent(built[i], built[i-1])

    if parent and mc.objExists(parent):
        mc.parent(built[0], parent)
    elif BuildParent == True:
        if not mc.objExists(parent):
            mc.group(empty=True, name=parent)
        mc.parent(built[0], parent)


    return built

def read_seg_guides(json_file):

    with open(json_file, "r") as f:
        data = json.load(f)

    part = data["part"]
    part_type = data["type"]
    parent = data.get("parent")

    cfg = PART_CONFIG.get(part, PART_CONFIG["default"])

    axes = cfg["Axes"]
    names = cfg["Names"]
    delete_last = cfg["Delete_Last"]
    BuildParent = cfg["BuildParent"]
    Guide_Type = cfg ["Guide_Type"]
    Bake_To_Orient = cfg["Bake_To_Orient"]

    guides = data["guides"]

    built = []

    keys = sorted(guides.keys())


    if BuildParent == True:
        mc.group(empty=True, name=parent)

    for i, gname in enumerate(keys):

        if delete_last and i == len(keys)-1:
            break

        g = guides[gname]

        mesh = g["mesh"]
        vert_list = g["vert_list"]
        offset = g["offset"]
        if "rotoffset" in g:
            rotoffset = g["rotoffset"]
        else:
            rotoffset = None
        up_id = g["upvect"]

        pos = get_position_from_vert_ids(mesh, vert_list)
        if not pos:
            continue

        pos = [pos[0]+offset[0], pos[1]+offset[1], pos[2]+offset[2]]

        if names and i < len(names):
            jnt_name = names[i]
        else:
            jnt_name = f"{part}_guide_{i+1:02d}"

        mc.select(clear=True)
        if Guide_Type == 'Loc':
            jnt = mc.spaceLocator(name=jnt_name)[0]
        else:
            jnt = mc.joint(name=jnt_name)
        mc.xform(jnt, ws=True, t=pos)

        built.append(jnt)
        
        if rotoffset:
            rot = mc.getAttr(jnt + ".rotate")[0]
            jo  = rotoffset

            mc.setAttr(
                jnt + ".rotate",
                jo[0] + rot[0],
                jo[1] + rot[1],
                jo[2] + rot[2]
            )


        if Guide_Type == 'Joints' and Bake_To_Orient == True:
            rot = mc.getAttr(jnt + ".rotate")[0]
            jo  = mc.getAttr(jnt + ".jointOrient")[0]

            mc.setAttr(
                jnt + ".jointOrient",
                jo[0] + rot[0],
                jo[1] + rot[1],
                jo[2] + rot[2]
            )

            mc.setAttr(jnt + ".rotate", 0, 0, 0)


        if mc.objExists(parent):
            mc.parent(jnt, parent)

    return built

# ---------- Normailize_UBM ----------

def normalize_ubm_mesh():
    ubm_meshes = [
        m for m in mc.ls(type="transform")
        if m.endswith("_UBM") and mc.listRelatives(m, s=True, type="mesh")
    ]

    if not ubm_meshes:
        mc.warning("No *_UBM meshes found in scene.")
        return

    if "Basemesh_UBM" in ubm_meshes:
        print("Basemesh_UBM already exists.")
        return

    if len(ubm_meshes) > 1:
        mc.warning("More than one *_UBM mesh found. Using the first one.")

    original_mesh = ubm_meshes[0]

    renamed = mc.rename(original_mesh, "Basemesh_UBM")

    if not mc.objExists(f"{renamed}.OG_Name"):
        mc.addAttr(renamed, ln="OG_Name", dt="string")

    mc.setAttr(f"{renamed}.OG_Name", original_mesh, type="string")

    print(f"Renamed {original_mesh} → {renamed}")


def restore_ubm_mesh():
    mesh = "Basemesh_UBM"

    if not mc.objExists(mesh):
        mc.warning("Basemesh_UBM not found.")
        return

    attr = f"{mesh}.OG_Name"

    if not mc.objExists(attr):
        mc.warning("No OG_Name attribute found.")
        return

    original_name = mc.getAttr(attr)

    restored = mc.rename(mesh, original_name)

    if mc.objExists(f"{restored}.OG_Name"):
        mc.deleteAttr(f"{restored}.OG_Name")

    print(f"Restored mesh name to {restored}")

# ---------------- UI ----------------

class GuideReaderUI(QtWidgets.QDialog):

    def __init__(self, parent=get_maya_main_window()):
        super().__init__(parent)

        self.setWindowTitle("Read Guides")
        self.setMinimumWidth(320)
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.Window | QtCore.Qt.WindowStaysOnTopHint)

        self.build_ui()
        self.populate_parts()

    def build_ui(self):

        layout = QtWidgets.QVBoxLayout(self)

        form = QtWidgets.QFormLayout()

        self.part_combo = QtWidgets.QComboBox()
        form.addRow("Part:", self.part_combo)

        layout.addLayout(form)

        self.read_btn = QtWidgets.QPushButton("Read Guides")
        layout.addWidget(self.read_btn)

        self.read_btn.clicked.connect(self.read_guides)

        # ---------------- UBM buttons ----------------

        self.normalize_btn = QtWidgets.QPushButton("Normalize UBM Mesh")
        layout.addWidget(self.normalize_btn)
        self.normalize_btn.clicked.connect(normalize_ubm_mesh)

        self.restore_btn = QtWidgets.QPushButton("Restore Original UBM Name")
        layout.addWidget(self.restore_btn)
        self.restore_btn.clicked.connect(restore_ubm_mesh)

    def populate_parts(self):

        self.part_combo.clear()

        if not os.path.exists(GUIDE_PATH):
            return

        files = [f for f in os.listdir(GUIDE_PATH) if f.endswith(".json")]

        self.part_combo.addItems(files)

    def read_guides(self):

        file = self.part_combo.currentText()
        if not file:
            return

        path = os.path.join(GUIDE_PATH, file)

        read_type(path)

# ---------------- SHOW ----------------

def show_guide_reader():
    global guide_reader_ui
    try:
        guide_reader_ui.close()
    except:
        pass

    guide_reader_ui = GuideReaderUI()
    guide_reader_ui.show()

show_guide_reader()
