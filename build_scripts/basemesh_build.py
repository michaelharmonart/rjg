from __future__ import annotations
import json
import os
import maya.cmds as mc
import maya.api.OpenMaya as om
import platform

groups = 'G:' if platform.system() == 'Windows' else '/groups'

# ---------------- PYQT FALLBACK ----------------
try:
    from PySide6 import QtWidgets, QtCore
    from shiboken6 import wrapInstance
except:
    from PySide2 import QtWidgets, QtCore
    from shiboken2 import wrapInstance

import maya.OpenMayaUI as omui

# ---------------- PATH ----------------
# -----------------------------
# Maya Main Window
# -----------------------------
def maya_main_window():
    ptr = omui.MQtUtil.mainWindow()
    return wrapInstance(int(ptr), QtWidgets.QWidget)


# -----------------------------
# JSON Path
# -----------------------------

if "__file__" in globals():
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
else:
    SCRIPT_DIR = mc.internalVar(userScriptDir=True)

CHAR_JSON = os.path.join(SCRIPT_DIR, "characters.json")

print("TopoAutoRig JSON PATH:", CHAR_JSON)

# -----------------------------
# Dummy Functions
# -----------------------------
def check_compatibility(self):

    # TEMP result
    compatible = True

    if compatible:

        self.compat_button.setStyleSheet(
            "QPushButton {background-color: rgb(120,180,120);}"
        )

        print("Compatibility check PASSED")

    else:

        self.compat_button.setStyleSheet(
            "QPushButton {background-color: rgb(200,80,80);}"
        )

        print("Compatibility check FAILED")


def read_guides():
    print("Read Guides function worked")


def normalize_ubm():
    print("Normalize UBM function worked")


def restore_ubm():
    print("Restore UBM function worked")


def export_guides():
    print("Export Guides function worked")


def full_build():
    print("Full Build function worked")

def manual_build(self):
    print("Manual Build function worked")


# -----------------------------
# Character JSON Helpers
# -----------------------------
def load_characters():

    if not os.path.exists(CHAR_JSON):

        default_chars = ["CrowdB", "Test"]

        with open(CHAR_JSON, "w") as f:
            json.dump(default_chars, f, indent=4)

        return default_chars

    with open(CHAR_JSON, "r") as f:
        return json.load(f)


def save_characters(chars):
    with open(CHAR_JSON, "w") as f:
        json.dump(chars, f, indent=4)


# -----------------------------
# Main UI
# -----------------------------

class CollapsibleSection(QtWidgets.QWidget):

    def __init__(self, title="", parent=None):
        super().__init__(parent)

        self.toggle_button = QtWidgets.QToolButton(text=title, checkable=True, checked=True)
        self.toggle_button.setStyleSheet("font-weight: bold")
        self.toggle_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.toggle_button.setArrowType(QtCore.Qt.DownArrow)

        self.toggle_button.clicked.connect(self.toggle)

        self.content_area = QtWidgets.QWidget()
        self.content_layout = QtWidgets.QVBoxLayout(self.content_area)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.toggle_button)
        layout.addWidget(self.content_area)

    def toggle(self):

        visible = self.toggle_button.isChecked()

        self.content_area.setVisible(visible)

        self.toggle_button.setArrowType(
            QtCore.Qt.DownArrow if visible else QtCore.Qt.RightArrow
        )

class TopoAutoRigUI(QtWidgets.QDialog):

    WINDOW_TITLE = "Topo_Auto_Rig_Build"

    def __init__(self, parent=maya_main_window()):
        super().__init__(parent)

        self.setWindowTitle(self.WINDOW_TITLE)
        self.setMinimumWidth(360)
        self.setWindowFlags(self.windowFlags() ^ QtCore.Qt.WindowContextHelpButtonHint)

        # -----------------------
        # Topology Dictionary
        # -----------------------

        self.topo_dict = {
            "Basemesh": {
                "path": ""
            }
        }

        # -----------------------
        # Character JSON
        # -----------------------

        self.characters = load_characters()

        self.build_ui()
        self.create_connections()

    # ------------------------------------------------
    # UI
    # ------------------------------------------------

    def build_ui(self):

        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # -----------------------
        # Topology Reader
        # -----------------------

        topo_group = QtWidgets.QGroupBox("Topology Reader")

        topo_layout = QtWidgets.QHBoxLayout()

        self.topo_dropdown = QtWidgets.QComboBox()
        self.topo_dropdown.addItems(self.topo_dict.keys())

        self.compat_button = QtWidgets.QPushButton("Check Compatibility")

        topo_layout.addWidget(self.topo_dropdown)
        topo_layout.addWidget(self.compat_button)

        topo_group.setLayout(topo_layout)

        main_layout.addWidget(topo_group)

        # -----------------------
        # Character Section
        # -----------------------

        char_group = QtWidgets.QGroupBox("Character")

        char_layout = QtWidgets.QVBoxLayout()

        self.char_dropdown = QtWidgets.QComboBox()
        self.char_dropdown.setEditable(True)
        self.char_dropdown.addItems(self.characters)

        self.init_char_button = QtWidgets.QPushButton("Initialize Character")

        char_layout.addWidget(self.char_dropdown)
        char_layout.addWidget(self.init_char_button)

        char_group.setLayout(char_layout)

        main_layout.addWidget(char_group)

        # -----------------------
        # Manual Section
        # -----------------------

        manual_section = CollapsibleSection("Manual")
        

        self.read_guides_btn = QtWidgets.QPushButton("Read Guides")
        self.normalize_btn = QtWidgets.QPushButton("Normalize UBM")
        self.restore_btn = QtWidgets.QPushButton("Restore UBM")
        self.export_btn = QtWidgets.QPushButton("Export Guides")
        self.build_btn = QtWidgets.QPushButton("Build")

        manual_section.content_layout.addWidget(self.read_guides_btn)
        manual_section.content_layout.addWidget(self.normalize_btn)
        manual_section.content_layout.addWidget(self.restore_btn)
        manual_section.content_layout.addWidget(self.export_btn)
        manual_section.content_layout.addWidget(self.build_btn)
        

        main_layout.addWidget(manual_section)

        # -----------------------
        # Auto Build Section
        # -----------------------

        auto_section = CollapsibleSection("Auto Build")

        self.full_build_btn = QtWidgets.QPushButton("Full Build")

        auto_section.content_layout.addWidget(self.full_build_btn)

        main_layout.addWidget(auto_section)

        main_layout.addStretch()

    # ------------------------------------------------
    # Connections
    # ------------------------------------------------

    def create_connections(self):

        self.compat_button.clicked.connect(self.check_compatibility)

        self.read_guides_btn.clicked.connect(self.read_guides)
        self.normalize_btn.clicked.connect(self.normalize_ubm)
        self.restore_btn.clicked.connect(self.restore_ubm)
        self.export_btn.clicked.connect(self.export_guides)
        self.build_btn.clicked.connect(self.manual_build)

        self.full_build_btn.clicked.connect(self.full_build)

        self.init_char_button.clicked.connect(self.initialize_character)

        self.char_dropdown.lineEdit().textChanged.connect(self.check_character_state)

    # ------------------------------------------------
    # Dummy Functions
    # ------------------------------------------------

    def check_compatibility(self):

        # TEMP result
        compatible = True

        if compatible:

            self.compat_button.setStyleSheet(
                "QPushButton {background-color: rgb(120,180,120);}"
            )

            print("Compatibility check PASSED")

        else:

            self.compat_button.setStyleSheet(
                "QPushButton {background-color: rgb(200,80,80);}"
            )

            print("Compatibility check FAILED")

    def read_guides(self):
        print("Read Guides function worked")

    def normalize_ubm(self):
        print("Normalize UBM function worked")

    def restore_ubm(self):
        print("Restore UBM function worked")

    def manual_build(self):
        print("Manual Build function worked")

    def export_guides(self):
        print("Export Guides function worked")

    def full_build(self):
        print("Full Build function worked")

    # ------------------------------------------------
    # Character Logic
    # ------------------------------------------------

    def check_character_state(self):

        name = self.char_dropdown.currentText()

        if name in self.characters:

            self.char_dropdown.setStyleSheet(
                "QComboBox {background-color: rgb(120,180,120);}"
            )

        else:

            self.char_dropdown.setStyleSheet(
                "QComboBox {background-color: rgb(200,200,120);}"
            )

    def initialize_character(self):

        name = self.char_dropdown.currentText()

        if name not in self.characters:

            self.characters.append(name)

            save_characters(self.characters)

            self.char_dropdown.addItem(name)

            print(f"Character '{name}' initialized")

        self.check_character_state()


# -----------------------------
# Launch
# -----------------------------
def show_ui():

    global topo_auto_rig_ui

    try:
        topo_auto_rig_ui.close()
    except:
        pass

    topo_auto_rig_ui = TopoAutoRigUI()
    topo_auto_rig_ui.show()


show_ui()