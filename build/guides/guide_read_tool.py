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
        "Axes": ["Y", "Z", "X"],
        "Names": ["LeftArm", "LeftForeArm", "LeftHand"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True

    },
    "default": {
        "Axes": ["Y", "-X", "Z"],
        "Names": None,
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":False
    },

    "clavicle": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["LeftShoulder"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "foot": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftFoot", "LeftToeBase", "LeftToe_End"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "indexfinger": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftHandIndex0", "LeftHandIndex1", "LeftHandIndex2", "LeftHandIndex3", "LeftHandIndex4"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "middlefinger": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftHandMiddle0", "LeftHandMiddle1", "LeftHandMiddle2", "LeftHandMiddle3", "LeftHandMiddle4"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "ringfinger": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftHandRing0", "LeftHandRing1", "LeftHandRing2", "LeftHandRing3", "LeftHandRing4"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "pinkyfinger": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftHandPinky0", "LeftHandPinky1", "LeftHandPinky2", "LeftHandPinky3", "LeftHandPinky4"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "thumbfinger": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftHandThumb1", "LeftHandThumb2", "LeftHandThumb3", "LeftHandThumb4"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "leg": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["LeftUpLeg", "LeftLeg"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "spine": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Hips", "Spine", "Spine1", "Spine2", ],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "neck": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Neck", "Neck1", "Neck2", "Head", "HeadTop_End"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True
    },

    "neck": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Mouth_M_UpperLip_01", "Mouth_L_UpperLip_02", "Mouth_L_UpperLip_03", "Mouth_L_UpperLip_04", "Mouth_L_UpperLip_05", "Mouth_L_CornerLip", "Mouth_L_LowerLip_05", "Mouth_L_LowerLip_04", "Mouth_L_LowerLip_03", "Mouth_L_LowerLip_02", "Mouth_M_LowerLip_01"],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False
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
