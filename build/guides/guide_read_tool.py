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

    guides = data["guides"]

    built = []

    keys = sorted(guides.keys())

    for i, gname in enumerate(keys):

        if delete_last and i == len(keys)-1:
            break

        g = guides[gname]

        mesh = g["mesh"]
        vert_list = g["vert_list"]
        offset = g["offset"]
        up_id = g["upvect"]

        pos = get_position_from_vert_ids(mesh, vert_list)
        if not pos:
            continue

        pos = [pos[0]+offset[0], pos[1]+offset[1], pos[2]+offset[2]]

        if names and i < len(names):
            jnt_name = names[i]
        else:
            jnt_name = f"{part}_guide_{i+1:02d}"

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

    for i in range(1, len(built)):
        mc.parent(built[i], built[i-1])

    if parent and mc.objExists(parent):
        mc.parent(built[0], parent)

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

        read_chain_guides(path)

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
