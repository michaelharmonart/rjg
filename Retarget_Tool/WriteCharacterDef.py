import os
import json

import maya.cmds as mc
try:
    from PySide6 import QtWidgets, QtCore
    from shiboken6 import wrapInstance
except:
    from PySide2 import QtWidgets, QtCore
    from shiboken2 import wrapInstance

import maya.OpenMayaUI as omui

# ------------------------------------------------------------
# CONFIG
# ------------------------------------------------------------

CHAR_LIB_PATH = r"G:\bobo\pipeline\pipeline\software\maya\scripts\rjg\Retarget_Tool\Character_libs"

PARTS = {
    "arm": ["FKShoulder", "FkElbow", "FkWrist", "IKHand", "IKPV"],
    "clav": ["Clavicle"],
    "leg": ["FKHip", "FKKnee", "FkAnkle", "FKToe", "IKFoot", "IKPV", "IKToe"],
    "spine": ["Spine"],
    "neck": ["Neck",],
    "head": ["Head"],
    "hip": ["Hip"],
}

TYPE_OPTIONS = ["FK", "IK", "FK_Distribute", "FK_IK", "Hybrid", "Root"]

# ------------------------------------------------------------
# UTILS
# ------------------------------------------------------------

def maya_main_window():
    ptr = omui.MQtUtil.mainWindow()
    return wrapInstance(int(ptr), QtWidgets.QWidget)

def list_characters():
    if not os.path.exists(CHAR_LIB_PATH):
        return []
    return [
        os.path.splitext(f)[0]
        for f in os.listdir(CHAR_LIB_PATH)
        if f.endswith(".json")
    ]

def character_path(name):
    return os.path.join(CHAR_LIB_PATH, f"{name}.json")

def load_character(name):
    path = character_path(name)
    if not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        return json.load(f)

def save_character(name, data):
    if not os.path.exists(CHAR_LIB_PATH):
        os.makedirs(CHAR_LIB_PATH)

    with open(character_path(name), "w") as f:
        json.dump(data, f, indent=4)

# ------------------------------------------------------------
# PICKER BUTTON
# ------------------------------------------------------------

class PickerButton(QtWidgets.QPushButton):
    assigned = QtCore.Signal(str, list)

    def __init__(self, label):
        super().__init__(label)
        self.part = label
        self.nodes = []   # ALWAYS A LIST
        self.armed = False

        self.update_color()
        self.clicked.connect(self.on_click)

    # ------------------------------------------------

    def update_color(self):
        if self.armed:
            self.setStyleSheet("background-color: #aa3333;")
        elif self.nodes:
            self.setStyleSheet("background-color: #339933;")
        else:
            self.setStyleSheet("background-color: #555;")

    # ------------------------------------------------

    def on_click(self):
        # Arm picker
        if not self.armed:
            self.armed = True
            self.update_color()
            print(f"[Picker] Armed for {self.part}")
            return

        # Capture selection(s)
        sel = mc.ls(sl=True)
        if not sel:
            mc.warning("Nothing selected")
            return

        for node in sel:
            if node not in self.nodes:
                self.nodes.append(node)

        self.armed = False
        self.update_color()
        self.assigned.emit(self.part, self.nodes)

        print(f"[Picker] {self.part} -> {self.nodes}")


# ------------------------------------------------------------
# MAIN UI
# ------------------------------------------------------------

class RetargetWriteUI(QtWidgets.QDialog):

    def __init__(self, parent=maya_main_window()):
        super().__init__(parent)

        self.setWindowTitle("Retarget Write Character Def")
        self.setMinimumWidth(450)
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.Window)

        self.control_map = {}
        self.picker_buttons = []

        self.build_ui()

    # --------------------------------------------------------

    def build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # ---------------- Character (typeahead) ----------------
        layout.addWidget(QtWidgets.QLabel("Character"))

        self.char_edit = QtWidgets.QLineEdit()
        self.char_edit.setPlaceholderText("Type character name")

        completer = QtWidgets.QCompleter(list_characters())
        completer.setCaseSensitivity(QtCore.Qt.CaseInsensitive)
        completer.setFilterMode(QtCore.Qt.MatchContains)

        self.char_edit.setCompleter(completer)
        layout.addWidget(self.char_edit)

        # ---------------- Part ----------------
        layout.addWidget(QtWidgets.QLabel("Part"))

        self.part_combo = QtWidgets.QComboBox()
        self.part_combo.addItems(sorted(PARTS.keys()))
        self.part_combo.currentTextChanged.connect(self.rebuild_pickers)
        layout.addWidget(self.part_combo)

        # ---------------- Picker area ----------------
        self.picker_layout = QtWidgets.QHBoxLayout()
        layout.addLayout(self.picker_layout)

        # ---------------- Type ----------------
        layout.addWidget(QtWidgets.QLabel("Type"))
        self.type_combo = QtWidgets.QComboBox()
        self.type_combo.addItems(TYPE_OPTIONS)
        layout.addWidget(self.type_combo)

        # ---------------- Options ----------------
        self.mo_chk = QtWidgets.QCheckBox("Maintain Offset")
        self.mo_chk.setChecked(True)

        self.mirror_chk = QtWidgets.QCheckBox("Mirror")

        layout.addWidget(self.mo_chk)
        layout.addWidget(self.mirror_chk)

        # ---------------- Mirror labels ----------------
        mirror_row = QtWidgets.QHBoxLayout()
        self.mirror_l = QtWidgets.QLineEdit("L")
        self.mirror_r = QtWidgets.QLineEdit("R")

        mirror_row.addWidget(QtWidgets.QLabel("Mirror L"))
        mirror_row.addWidget(self.mirror_l)
        mirror_row.addWidget(QtWidgets.QLabel("Mirror R"))
        mirror_row.addWidget(self.mirror_r)

        layout.addLayout(mirror_row)

        # ---------------- FKIK ----------------
        layout.addWidget(QtWidgets.QLabel("FK IK Switch"))
        self.fkik_edit = QtWidgets.QLineEdit()
        layout.addWidget(self.fkik_edit)

        # ---------------- Write ----------------
        write_btn = QtWidgets.QPushButton("Write to JSON")
        write_btn.clicked.connect(self.write_json)
        layout.addWidget(write_btn)

        self.rebuild_pickers(self.part_combo.currentText())

    # --------------------------------------------------------

    def rebuild_pickers(self, part):
        # Clear old
        for btn in self.picker_buttons:
            btn.deleteLater()
        self.picker_buttons = []
        self.control_map.clear()

        while self.picker_layout.count():
            self.picker_layout.takeAt(0)

        # Build new
        for subpart in PARTS[part]:
            btn = PickerButton(subpart)
            btn.assigned.connect(self.store_control)
            self.picker_layout.addWidget(btn)
            self.picker_buttons.append(btn)

    # --------------------------------------------------------

    def store_control(self, part, nodes):
        self.control_map[part] = nodes

    # --------------------------------------------------------

    def write_json(self):
        char = self.char_edit.text().strip()
        if not char:
            mc.warning("Character name required")
            return

        part = self.part_combo.currentText()
        data = load_character(char)

        data[part] = {
            "ControlList": self.control_map,
            "Type": self.type_combo.currentText(),
            "MO": self.mo_chk.isChecked(),
            "Mirror": self.mirror_chk.isChecked(),
            "Mirror_Label": [self.mirror_l.text(), self.mirror_r.text()],
            "FKIK_Switch": self.fkik_edit.text()
        }

        save_character(char, data)
        print(f"[Retarget] Wrote part '{part}' for character '{char}'")

# ------------------------------------------------------------
# LAUNCH
# ------------------------------------------------------------

def show_retarget_write_ui():
    global _retarget_write_ui
    try:
        _retarget_write_ui.close()
    except:
        pass

    _retarget_write_ui = RetargetWriteUI()
    _retarget_write_ui.show()

show_retarget_write_ui()













"""
CHAR_LIB_PATH = r"G:\bobo\pipeline\pipeline\software\maya\scripts\rjg\Retarget_Tool\Character_libs"

PART_CONFIG = {
    "arm": {
        "ControlList": ['Shoulder', 'Elbow', 'Wrist'],    #List of joints to control that need to happen
        "Type": "FK",                                     # FK, FK distribute, Ik, Hybrid
        'MO':True,                                        
        'Mirror':True,
        'Mirror_Label':['L', 'R']                         #Left, Right or L R etc
        'FKIK_Switch'['arm_L_02_fk_CTRL.armL_IKFK', '0']  #FKIK Switch
    },

}
"""