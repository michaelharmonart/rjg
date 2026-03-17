import sys
modules = [name for name in sys.modules.keys() if name.startswith("rjg")]
for name in modules:
    del sys.modules[name]
import rjg


import platform
import sys
from importlib import reload

import maya.cmds as mc
import maya.mel as mel

groups = 'G:' if platform.system() == 'Windows' else '/groups'
mc.scriptEditorInfo(suppressWarnings=True,suppressInfo=True)

import rjg.build.buildPart as rBuild
import rjg.build.prop as rProp
import rjg.libs.file as rFile
import rjg.libs.util as rUtil
import rjg.post.dataIO.controls as rCtrlIO
import rjg.post.finalize as rFinal
import rjg.post.usd as rUSD
from rjg.build.parts.bipedLimb import BipedLimb
from rjg.build.parts.clavicle import Clavicle
from rjg.build.parts.hand import Hand
from rjg.libs.skin import auto_split_all_weights
from rjg.libs.profile import add_profiler_tag
import rjg.post.PoseInterpExtras as expi

reload(rUtil)
reload(rProp)
reload(rBuild)
reload(rFinal)
reload(rFile)
reload(rUSD)

import pipe.m.space_switch as spsw
from ngSkinTools2.api import plugin




rCtrlIO.read_ctrls(f"{groups}/bobo/character/Rigs/Basemesh/Controls/", curve_file="Basemesh_control_curves")  


import rjg.post.dataIO.controls as rCtrlIO
import rjg.libs.control.draw as draw

#rCtrlIO.write_ctrls("G:/bobo/character/Rigs/Bobo/Controls", force=True, name='bobo_control_curves')

import maya.cmds as mc
import maya.mel as mel
import sys, platform
from importlib import reload

groups = 'G:' if platform.system() == 'Windows' else '/groups'
    
"""
    first i need a  UI box that is rescalable and has a exit button
    that will allow me to select a Character in a list 
    CharList = ["Bobo", "Gretchen", "Bee"]
    Then I will need to select what type of export I want (check box style)
            Model
            Guides
            Extras
    at the very bottom of the box i will want a button that says export and one that says cancel
        
"""
# Global variables for UI elements
export_window = None
char_menu = None
all_checkbox = None



def export_character(*args):
    selected_char = mc.optionMenu(char_menu, query=True, value=True)
    obj_name = f"{selected_char}_UBM"

    # Check if the object exists in the scene
    if mc.objExists(obj_name):
        rCtrlIO.write_ctrls(f"{groups}/bobo/character/Rigs/{selected_char}/Controls", force=True, name=f'{selected_char}_control_curves')
    else:
        print(f"Warning: Object '{obj_name}' not found in the scene.")

def cancel_export(*args):
    """Closes the export window."""
    global export_window
    if mc.window(export_window, exists=True):
        mc.deleteUI(export_window, window=True)

def create_export_window():
    """Creates the character export UI window."""
    global export_window, char_menu, model_checkbox, guides_checkbox, extras_checkbox, all_checkbox

    # Close existing window if it exists
    if mc.window("exportWindow", exists=True):
        mc.deleteUI("exportWindow", window=True)

    # Create window
    export_window = mc.window("exportWindow", title="Control Exporter", widthHeight=(300, 200), sizeable=True)

    # Create a main layout
    mc.columnLayout(adjustableColumn=True)

    # Character selection dropdown
    mc.text(label="Select Character:")
    char_menu = mc.optionMenu()
    for character in ["Bobo", "Gretchen", "Luciana", "Domingo", "Susaka", "Drummer", 'Sharkguy', 'Basemesh']:
        mc.menuItem(label=character)

    # Buttons
    mc.separator(height=10, style='none')
    mc.rowLayout(numberOfColumns=2, columnAlign=(1, 'center'), columnWidth2=(140, 140))
    mc.button(label="Export", command=export_character)
    mc.button(label="Cancel", command=cancel_export)

    # Show window
    mc.showWindow(export_window)

# Create the UI window
create_export_window()


import rjg.build_scripts.bettercontrols as c

c.write_control_shapes(f"G:/bobo/character/Rigs/Basemesh/Controls/controls.json")

from rjg.build_scripts.bettercontrols import apply_control_file

apply_control_file(f"G:/bobo/character/Rigs/Basemesh/Controls/controls.json")



import maya.cmds as cmds

def flip_controls(ctrls, flip=True):
    
    for ctrl in ctrls:
        
        if flip:
            # add suffix
            new_name = f"{ctrl}_DONT"
            
            if cmds.objExists(ctrl) and not cmds.objExists(new_name):
                cmds.rename(ctrl, new_name)

        else:
            # remove suffix
            dont_name = f"{ctrl}_DONT"
            
            if cmds.objExists(dont_name):
                cmds.rename(dont_name, ctrl)
                                
controls = ["COG_M_CTRL", "global_M_CTRL", "foot_L_01_L_CTRL", "foot_R_01_R_CTRL", "hand_L_01_CTRL", "hand_R_01_CTRL", "RJG_M_CTRL"]

flip_controls(controls, flip=False)   # adds _DONT
#flip_controls(controls, flip=False)  # removes _DONT


from __future__ import annotations
import json
import os
import maya.cmds as mc
import maya.api.OpenMaya as om
import platform

import rjg.build.guides.guide_read_tool as gr
import rjg.build_scripts.basemesh_buildhelper as sb

import shutil

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

RIGS_ROOT = f"{groups}/bobo/character/Rigs"


# -----------------------------
# Maya Main Window
# -----------------------------
def maya_main_window():
    ptr = omui.MQtUtil.mainWindow()
    return wrapInstance(int(ptr), QtWidgets.QWidget)

# ---------------- UI COLORS ----------------

GOOD_COLOR = "rgb(70,140,70)"
WARN_COLOR = "rgb(160,140,60)"
BAD_COLOR  = "rgb(160,60,60)"

TEXT_COLOR = "white"
FIELD_TEXT = "black"


# -----------------------------
# JSON Path
# -----------------------------

if "__file__" in globals():
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
else:
    SCRIPT_DIR = mc.internalVar(userScriptDir=True)

CHAR_JSON = os.path.join(SCRIPT_DIR, "characters.json")

print("TopoAutoRig JSON PATH:", CHAR_JSON)


# ---------------- TOPOLOGY DATA ----------------

TOPOLOGY_DATA = {
    "Basemesh": {
        "ubm": "Basemesh_UBM",
        "meshes": [
            "Eyes",
            "Corneas",
            "botteeth",
            "topteeth",
            "tongue"
        ]
    }
}


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

#------------------------------
#Build Helper Class
#------------------------------
def manual_build_popup(character, rig_root, topology):
    """
    Opens a warning popup and runs the manual build script with fully resolved arguments.
    """

    # ----------------------------
    # Popup Dialog
    # ----------------------------
    class ManualBuildDialog(QtWidgets.QDialog):
        def __init__(self):
            super().__init__(maya_main_window())
            self.setWindowTitle("Manual Build Warning")
            self.setMinimumWidth(400)
            self.setWindowFlags(self.windowFlags() ^ QtCore.Qt.WindowContextHelpButtonHint)

            layout = QtWidgets.QVBoxLayout(self)

            warning_label = QtWidgets.QLabel(
                "Warning: The current scene will NOT be saved.\n"
                "Do you want to proceed with the manual build?"
            )
            warning_label.setWordWrap(True)
            layout.addWidget(warning_label)

            # Buttons
            btn_layout = QtWidgets.QHBoxLayout()
            self.proceed_btn = QtWidgets.QPushButton("Proceed")
            self.cancel_btn = QtWidgets.QPushButton("Cancel")
            btn_layout.addWidget(self.proceed_btn)
            btn_layout.addWidget(self.cancel_btn)

            layout.addLayout(btn_layout)

            self.proceed_btn.clicked.connect(self.accept)
            self.cancel_btn.clicked.connect(self.reject)

    # ----------------------------
    # Show Dialog
    # ----------------------------
    dialog = ManualBuildDialog()
    result = dialog.exec_()

    if result != QtWidgets.QDialog.Accepted:
        print("Manual build canceled.")
        return

    # ----------------------------
    # Resolve paths for sb.run
    # ----------------------------
    char_dir = os.path.join(rig_root, character)

    mp = os.path.join(char_dir, f"{character}_Model.mb")
    gp = os.path.join(char_dir, f"{character}_Guides.mb")
    ep = os.path.join(char_dir, f"{character}_Extras.mb")

    # Control curves file
    cp1 = os.path.join(char_dir, "controls", f"{character}_control_curves.json")
    cp2 = os.path.join(char_dir, "controls", f"{topology}_control_curves.json")
    cp = cp1 if os.path.exists(cp1) else cp2

    # Skin file
    sp1 = os.path.join(char_dir, "SkinFiles", f"{character}_Skin.json")
    sp2 = os.path.join(char_dir, "SkinFiles", f"{topology}_Skin.json")
    sp = sp1 if os.path.exists(sp1) else sp2

    # ----------------------------
    # Call the build
    # ----------------------------
    print(f"Running manual build for {character}...")
    print(f"Model: {mp}\nGuides: {gp}\nExtras: {ep}\nControls: {cp}\nSkin: {sp}")

    sb.run(
        character=character,
        mp=mp,
        gp=gp,
        ep=ep,
        cp=cp,
        sp=sp,
        pp=None,
        face=True,
        previs=False
    )

    print("Manual build complete.")

#------------------------------
#Export Helper class
#------------------------------
class GuideExportHelper():

    def __init__(self, character):

        self.character = character
        self.base_path = f"{groups}/bobo/character/Rigs/{character}"

        self.model_file = f"{self.base_path}/{character}_Model.mb"
        self.guides_file = f"{self.base_path}/{character}_Guides.mb"
        self.extras_file = f"{self.base_path}/{character}_Extras.mb"


    # -------------------------
    # VALIDATION
    # -------------------------

    def character_initialized(self):
        return os.path.exists(self.base_path)


    def existing_files(self):

        files = []

        if os.path.exists(self.model_file):
            files.append(self.model_file)

        if os.path.exists(self.guides_file):
            files.append(self.guides_file)

        if os.path.exists(self.extras_file):
            files.append(self.extras_file)

        return files


    # -------------------------
    # FILE MANAGEMENT
    # -------------------------

    def backup_file(self, filepath):

        base, ext = os.path.splitext(filepath)

        i = 1
        new_path = f"{base}_{i}{ext}"

        while os.path.exists(new_path):
            i += 1
            new_path = f"{base}_{i}{ext}"

        os.rename(filepath, new_path)

        print(f"Backed up {filepath} -> {new_path}")


    def conflict_popup(self):

        result = mc.confirmDialog(
            title="Export Conflict",
            message="Export files already exist.\nWhat would you like to do?",
            button=["Overwrite", "Backup", "Cancel"],
            defaultButton="Overwrite",
            cancelButton="Cancel",
            dismissString="Cancel"
        )

        return result


    # -------------------------
    # EXPORTS
    # -------------------------

    def export_model(self):

        obj = f"{self.character}_UBM"

        if mc.objExists(obj):

            mc.select(obj)

            mc.file(
                self.model_file,
                force=True,
                options="v=0;",
                type="mayaBinary",
                exportSelected=True
            )

        else:
            mc.warning(f"{obj} not found.")


    def export_guides(self):

        guides = "Guides"
        char_check = f"{self.character}_UBM"

        if mc.objExists(char_check):

            if mc.objExists(guides):

                mc.select(guides)

                mc.file(
                    self.guides_file,
                    force=True,
                    options="v=0;",
                    type="mayaBinary",
                    exportSelected=True
                )

            else:
                mc.warning("Guides group not found.")

        else:
            mc.warning(f"{char_check} not found.")


    def export_extras(self):

        obj = f"{self.character}_EXTRAS"

        if mc.objExists(obj):

            mc.select(obj)

            mc.file(
                self.extras_file,
                force=True,
                options="v=0;",
                type="mayaBinary",
                exportSelected=True
            )

        else:
            mc.warning(f"{obj} not found.")


    # -------------------------
    # MAIN RUN
    # -------------------------

    def run(self, force_backup=False):

        if not self.character_initialized():
            mc.warning(f"{self.character} has not been initialized.")
            return

        existing = self.existing_files()

        action = "Overwrite"

        if existing:

            if force_backup:
                action = "Backup"
            else:
                action = self.conflict_popup()

            if action == "Cancel":
                print("Export cancelled.")
                return

            if action == "Backup":
                for file in existing:
                    self.backup_file(file)

        self.export_model()
        self.export_guides()
        self.export_extras()

        print("Export complete.")



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

        # -----------------------
        # Global UI Style
        # -----------------------

        self.setStyleSheet(f"""
            QPushButton {{
                background-color: rgb(80,80,80);
                color: {TEXT_COLOR};
                border-radius: 4px;
                padding: 6px;
            }}

            QPushButton:hover {{
                background-color: rgb(95,95,95);
            }}

            QComboBox {{
                padding: 4px;
            }}

            QGroupBox {{
                font-weight: bold;
                border: 1px solid rgb(90,90,90);
                border-radius: 4px;
                margin-top: 6px;
            }}

            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px 0 3px;
            }}
        """)

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

        topology = self.topo_dropdown.currentText()
        topo_data = TOPOLOGY_DATA[topology]

        base_file = f"{RIGS_ROOT}/{topology}/{topology}_BASE.mb"

        print("\n----- Compatibility Check -----")

        if not os.path.exists(base_file):

            print("Base topology file missing:")
            print(base_file)

            self.compat_button.setStyleSheet(
                f"QPushButton {{background-color: {BAD_COLOR}; color: {TEXT_COLOR};}}"
            )
            return

        # --------------------------------
        # Import Base File
        # --------------------------------

        mc.file(
            base_file,
            i=True,
            namespace="TOPO_CHECK",
            preserveReferences=True
        )

        # --------------------------------
        # Find UBM In Scene
        # --------------------------------

        ubm_meshes = mc.ls("*_UBM", type="transform")

        if not ubm_meshes:

            print("No _UBM mesh found in scene")

            self.compat_button.setStyleSheet(
                f"QPushButton {{background-color: {BAD_COLOR}; color: {TEXT_COLOR};}}"
            )

            return

        scene_ubm = ubm_meshes[0]

        print(f"Checking UBM mesh: {scene_ubm}")

        if len(ubm_meshes) > 1:
            print("WARNING: Multiple UBM meshes detected")

        base_ubm = f"TOPO_CHECK:{topo_data['ubm']}"

        # --------------------------------
        # Compare UBM Topology
        # --------------------------------

        scene_vtx = mc.polyEvaluate(scene_ubm, v=True)
        base_vtx = mc.polyEvaluate(base_ubm, v=True)

        if scene_vtx != base_vtx:

            print("UBM vertex count mismatch")
            print(scene_vtx, "vs", base_vtx)

            self.compat_button.setStyleSheet(
                f"QPushButton {{background-color: {BAD_COLOR}; color: {TEXT_COLOR};}}"
            )

            ubm_failed = True

        else:

            print("UBM vertex count matches")
            ubm_failed = False

        # --------------------------------
        # Check Required Meshes
        # --------------------------------

        other_failures = False

        for mesh in topo_data["meshes"]:

            scene_mesh = mc.ls(mesh)

            if not scene_mesh:

                print(f"{mesh} missing from scene")
                other_failures = True
                continue

            base_mesh = f"TOPO_CHECK:{mesh}"

            if not mc.objExists(base_mesh):

                print(f"{mesh} missing in base file")
                other_failures = True
                continue

            scene_vtx = mc.polyEvaluate(scene_mesh[0], v=True)
            base_vtx = mc.polyEvaluate(base_mesh, v=True)

            if scene_vtx == base_vtx:

                print(f"{mesh} topology matches")

            else:

                print(f"{mesh} topology mismatch")
                other_failures = True

        # --------------------------------
        # Set Button Color
        # --------------------------------

        if ubm_failed:

            self.compat_button.setStyleSheet(
                f"QPushButton {{background-color: {BAD_COLOR}; color: {TEXT_COLOR};}}"
            )

        elif other_failures:

            self.compat_button.setStyleSheet(
                f"QPushButton {{background-color: {WARN_COLOR}; color: {TEXT_COLOR};}}"
            )

        else:

            self.compat_button.setStyleSheet(
                f"QPushButton {{background-color: {GOOD_COLOR}; color: {TEXT_COLOR};}}"
            )

        print("Compatibility check complete\n")

        # --------------------------------
        # Clean Up Imported Namespace
        # --------------------------------

        if mc.namespace(exists="TOPO_CHECK"):

            mc.namespace(setNamespace=":")

            objs = mc.ls("TOPO_CHECK:*")

            if objs:
                mc.delete(objs)

            mc.namespace(removeNamespace="TOPO_CHECK")
            print("Compatibility check complete\n")

    def read_guides(self):
        gr.build_all_guides()

    def normalize_ubm(self):
        gr.normalize_ubm_mesh()

    def restore_ubm(self):
        gr.restore_ubm_mesh()

    def manual_build(self):
        """
        Opens the manual build warning popup and runs the build if Proceed is clicked.
        """
        character = self.char_dropdown.currentText()
        rig_root = f"{groups}/bobo/character/Rigs"
        topology = self.topo_dropdown.currentText()
        
        manual_build_popup(character, rig_root, topology)

    def export_guides(self, *args):
        selected_char = self.char_dropdown.currentText().strip()
        print(f"Export button clicked. Selected character: '{selected_char}'")

        exporter = GuideExportHelper(selected_char)

        print("Character initialized?", exporter.character_initialized())
        print("Existing files:", exporter.existing_files())

        exporter.run()

    def full_build(self):

        character = self.char_dropdown.currentText().strip()
        topology = self.topo_dropdown.currentText()
        rig_root = f"{groups}/bobo/character/Rigs"

        print("\n===== FULL BUILD START =====")
        print(f"Character: {character}")
        print(f"Topology: {topology}")

        # -------------------------
        # Normalize UBM
        # -------------------------

        print("\n--- Normalizing UBM ---")
        gr.normalize_ubm_mesh()

        # -------------------------
        # Read Guides
        # -------------------------

        print("\n--- Reading Guides ---")
        gr.build_all_guides()

        # -------------------------
        # Restore UBM
        # -------------------------

        print("\n--- Restoring UBM ---")
        gr.restore_ubm_mesh()

        # -------------------------
        # Export Guides (Backup)
        # -------------------------

        print("\n--- Exporting Guides (Backup Enabled) ---")

        exporter = GuideExportHelper(character)
        exporter.run(force_backup=True)

        # -------------------------
        # Run Build
        # -------------------------

        print("\n--- Running Manual Build ---")

        manual_build_popup(character, rig_root, topology)

        print("\n===== FULL BUILD COMPLETE =====")

    # ------------------------------------------------
    # Character Logic
    # ------------------------------------------------

    def check_character_state(self):

        name = self.char_dropdown.currentText()

        if name in self.characters:

            self.char_dropdown.setStyleSheet(
                f"""
                QComboBox {{
                    background-color: {GOOD_COLOR};
                    color: {FIELD_TEXT};
                }}
                QComboBox QLineEdit {{
                    background-color: {GOOD_COLOR};
                    color: {FIELD_TEXT};
                }}
                """
            )

        else:

            self.char_dropdown.setStyleSheet(
                f"""
                QComboBox {{
                    background-color: {WARN_COLOR};
                    color: {FIELD_TEXT};
                }}
                QComboBox QLineEdit {{
                    background-color: {WARN_COLOR};
                    color: {FIELD_TEXT};
                }}
        """
    )

    def initialize_character(self):

        character = self.char_dropdown.currentText()
        topology = self.topo_dropdown.currentText()

        rigs_root = f"{groups}/bobo/character/Rigs"
        char_dir = os.path.join(rigs_root, character)

        print(f"\nInitializing Character: {character}")

        # -----------------------------------
        # Create Character Folder
        # -----------------------------------

        if not os.path.exists(char_dir):

            os.makedirs(char_dir)

            print(f"Created character folder: {char_dir}")

        else:

            print("Character folder already exists")

        # -----------------------------------
        # Controls / SkinFiles
        # -----------------------------------

        folders = ["Controls", "SkinFiles"]

        for folder in folders:

            target_folder = os.path.join(char_dir, folder)
            source_folder = os.path.join(rigs_root, topology, folder)

            if not os.path.exists(target_folder):

                if os.path.exists(source_folder):

                    shutil.copytree(source_folder, target_folder)

                    print(f"Copied {folder} folder")

                else:

                    print(f"Source {folder} folder missing: {source_folder}")

            else:

                print(f"{folder} Folder already exists")

        # -----------------------------------
        # Maya Files
        # -----------------------------------

        maya_files = [
            f"{character}_ALL.mb",
            f"{character}_Model.mb",
            f"{character}_Guides.mb",
            f"{character}_Extras.mb"
        ]

        for file in maya_files:

            file_path = os.path.join(char_dir, file)

            if os.path.exists(file_path):

                print(f"{file} exists")

            else:

                print(f"{file} missing")

        # -----------------------------------
        # JSON Character List
        # -----------------------------------

        if character not in self.characters:

            self.characters.append(character)

            save_characters(self.characters)

            self.char_dropdown.addItem(character)

            print(f"Character '{character}' added to JSON")


        all_exist = True

        for file in maya_files:

            file_path = os.path.join(char_dir, file)

            if os.path.exists(file_path):

                print(f"{file} exists")

            else:

                print(f"{file} missing")
                all_exist = False


        if all_exist:
            self.init_char_button.setStyleSheet(
                "QPushButton {background-color: rgb(120,180,120);}"
            )
        else:
            self.init_char_button.setStyleSheet(
                "QPushButton {background-color: rgb(200,200,120);}"
            )

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

import sys
modules = [name for name in sys.modules.keys() if name.startswith("rjg")]
for name in modules:
    del sys.modules[name]
import rjg

from __future__ import annotations
import json
import os

import maya.cmds as mc
import maya.api.OpenMaya as om

try:
    from PySide6 import QtWidgets, QtCore
except:
    from PySide2 import QtWidgets, QtCore


import numpy as np


def get_selected_mesh():
    sel = mc.ls(sl=True, o=True)
    if not sel:
        return None
    return sel[0]


def get_selected_verts():
    # Get current selection
    sel = mc.ls(sl=True, fl=True)

    # Filter only vertices
    verts = mc.filterExpand(sel, sm=31) or []

    return verts

def get_selected_vert_ids():
    sel = mc.ls(sl=True, fl=True)
    verts = mc.filterExpand(sel, sm=31) or []

    vert_ids = [int(v.split('[')[-1].rstrip(']')) for v in verts]

    return vert_ids

def get_selected_vert_ids_in_order():
    sel = om.MGlobal.getActiveSelectionList()

    verts = []
    for i in range(sel.length()):
        dag, comp = sel.getComponent(i)

        if comp.apiType() == om.MFn.kMeshVertComponent:
            fn = om.MFnSingleIndexedComponent(comp)
            verts.extend(fn.getElements())

    return verts

def get_middle_position_from_vert_ids(mesh, vert_ids):
    if not vert_ids:
        mc.warning("No vertex IDs provided.")
        return None

    positions = []

    for vid in vert_ids:
        vtx = f"{mesh}.vtx[{vid}]"
        if mc.objExists(vtx):
            pos = mc.xform(vtx, q=True, ws=True, t=True)
            positions.append(pos)

    if not positions:
        mc.warning("No valid vertex positions found.")
        return None

    count = len(positions)
    mid_pos = [
        sum(p[i] for p in positions) / count
        for i in range(3)
    ]

    return mid_pos

def build_plane_from_selected_verts():

    verts = mc.filterExpand(mc.ls(sl=True, fl=True), sm=31)
    if not verts:
        mc.warning("Select verts.")
        return

    # -----------------------------
    # Get positions
    # -----------------------------

    points = []

    for v in verts:
        pos = mc.xform(v, q=True, ws=True, t=True)
        points.append(pos)

    points = np.array(points)

    # -----------------------------
    # center
    # -----------------------------

    center = points.mean(axis=0)

    # -----------------------------
    # best-fit plane
    # -----------------------------

    cov = np.cov(points.T)
    eigvals, eigvecs = np.linalg.eig(cov)

    normal = eigvecs[:, eigvals.argmin()]
    normal = om.MVector(*normal).normalize()

    # -----------------------------
    # build axes
    # -----------------------------

    up = om.MVector(0,1,0)

    if abs(normal * up) > 0.99:
        up = om.MVector(1,0,0)

    tangent = normal ^ up
    tangent.normalize()

    bitangent = normal ^ tangent
    bitangent.normalize()

    # -----------------------------
    # plane-space bounds
    # -----------------------------

    u_vals = []
    v_vals = []

    center_vec = om.MVector(*center)

    for p in points:

        vec = om.MVector(*p) - center_vec

        u_vals.append(vec * tangent)
        v_vals.append(vec * bitangent)

    width = max(u_vals) - min(u_vals)
    height = max(v_vals) - min(v_vals)

    # -----------------------------
    # create plane with center loops
    # -----------------------------

    plane = mc.polyPlane(
        w=width,
        h=height,
        sx=2,
        sy=2
    )[0]

    # -----------------------------
    # build transform matrix
    # -----------------------------

    matrix = [
        tangent.x, tangent.y, tangent.z, 0,
        normal.x, normal.y, normal.z, 0,
        bitangent.x, bitangent.y, bitangent.z, 0,
        center[0], center[1], center[2], 1
    ]

    mc.xform(plane, matrix=matrix)

    # -----------------------------
    # store attrs
    # -----------------------------

    orient_data = [
        [tangent.x, tangent.y, tangent.z],
        [normal.x, normal.y, normal.z],
        [bitangent.x, bitangent.y, bitangent.z]
    ]

    size_data = [width, height]

    if not mc.attributeQuery("guideOrient", n=plane, ex=True):
        mc.addAttr(plane, ln="guideOrient", dt="string")

    if not mc.attributeQuery("guideSize", n=plane, ex=True):
        mc.addAttr(plane, ln="guideSize", dt="string")

    mc.setAttr(plane + ".guideOrient", json.dumps(orient_data), type="string")
    mc.setAttr(plane + ".guideSize", json.dumps(size_data), type="string")

    print("Created guide plane:", plane)

    return plane

def get_preview_grp():
    if not mc.objExists("preview_guide_grp"):
        return mc.group(em=True, n="preview_guide_grp")
    return "preview_guide_grp"

def get_next_chain_index():
    existing = mc.ls("chain_guide_*", type="joint")
    nums = []
    for j in existing:
        try:
            nums.append(int(j.split("_")[-1]))
        except:
            pass
    return max(nums)+1 if nums else 1

    # -----------------------------
# UI
# -----------------------------

class GuideWriteTool(QtWidgets.QDialog):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Guide Write Tool")
        self.setMinimumWidth(300)
        self.setWindowFlags(
        self.windowFlags()
        | QtCore.Qt.Window
        | QtCore.Qt.WindowStaysOnTopHint
    )

        self.last_joint = None

        self.ordered_verts = []
        self._script_job_id = None
        self._last_selection = set()

        self.build_ui()

    # -------------------------

    def build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # Part name
        layout.addWidget(QtWidgets.QLabel("Part Name"))
        self.part_le = QtWidgets.QLineEdit()
        layout.addWidget(self.part_le)

        # Type dropdown
        layout.addWidget(QtWidgets.QLabel("Type"))
        self.type_cb = QtWidgets.QComboBox()
        self.type_cb.addItems(["chain", "single", "sequence", "plane"])
        layout.addWidget(self.type_cb)

        # Parent field
        layout.addWidget(QtWidgets.QLabel("Parent"))
        self.parent_le = QtWidgets.QLineEdit()
        layout.addWidget(self.parent_le)

        # Buttons
        self.write_pos_btn = QtWidgets.QPushButton("Write Pos")
        self.upvect_btn = QtWidgets.QPushButton("Store Up Vector")
        self.write_part_btn = QtWidgets.QPushButton("Write Part")
        self.clear_preview_btn = QtWidgets.QPushButton("Clear Preview")

        layout.addWidget(self.write_pos_btn)
        layout.addWidget(self.upvect_btn)
        layout.addWidget(self.write_part_btn)
        layout.addWidget(self.clear_preview_btn)

        # Connections
        self.write_pos_btn.clicked.connect(self.write_pos)
        self.upvect_btn.clicked.connect(self.store_up_vector)
        self.write_part_btn.clicked.connect(self.write_part)
        self.write_part_btn.clicked.connect(self.clear_preview)

        # -------------------------
        # Sequence Recorder
        # -------------------------

        layout.addWidget(QtWidgets.QLabel("Sequence Recorder"))

        seq_layout = QtWidgets.QHBoxLayout()

        self.seq_start_btn = QtWidgets.QPushButton("Start")
        self.seq_stop_btn = QtWidgets.QPushButton("Stop")

        seq_layout.addWidget(self.seq_start_btn)
        seq_layout.addWidget(self.seq_stop_btn)

        layout.addLayout(seq_layout)

        # Initial button state
        self.seq_start_btn.setEnabled(True)
        self.seq_stop_btn.setEnabled(False)

        # Connections
        self.seq_start_btn.clicked.connect(self.start_sequence_recording)
        self.seq_stop_btn.clicked.connect(self.stop_sequence_recording)

    # -------------------------
    # WRITE POS
    # -------------------------

    def write_pos(self):


        if self.type_cb.currentText() == "chain":
            mesh = get_selected_mesh()
            if not mesh:
                mc.warning("No mesh selected.")
                return

            vert_ids = get_selected_vert_ids_in_order()
            if not vert_ids:
                mc.warning("No verts selected.")
                return

            pos = get_middle_position_from_vert_ids(mesh, vert_ids)
            if not pos:
                mc.warning("Could not compute position.")
                return

            idx = get_next_chain_index()

            null = mc.group(em=True, n=f"chain_guide_NULL_{idx:02d}")
            jnt = mc.joint(n=f"chain_guide_{idx:02d}")

            mc.parent(jnt, null)
            mc.parent(null, get_preview_grp())

            mc.xform(null, ws=True, t=pos)

            # Custom attrs
            if not mc.attributeQuery("vertList", n=jnt, ex=True):
                mc.addAttr(jnt, ln="vertList", dt="string")
            if not mc.attributeQuery("mesh", n=jnt, ex=True):
                mc.addAttr(jnt, ln="mesh", dt="string")
            if not mc.attributeQuery("upVectorVert", n=jnt, ex=True):
                mc.addAttr(jnt, ln="upVectorVert", dt="string")

            mc.setAttr(jnt+".vertList", json.dumps(vert_ids), type="string")
            mc.setAttr(jnt+".mesh", mesh, type="string")
            mc.setAttr(jnt+".upVectorVert", "None", type="string")

            self.last_joint = jnt

            print(f"Created {jnt}")

        elif self.type_cb.currentText() == "sequence":
            mesh = get_selected_mesh()
            if not mesh:
                mc.warning("No mesh selected.")
                return
            if not self.ordered_verts:
                mc.warning("No recorded vertex sequence. Click Start and select verts.")
                return

            vert_ids = self.get_recorded_vert_ids()
            if not vert_ids:
                mc.warning("No verts selected.")
                return
            idx = 1
            for vert in vert_ids:

                pos = get_middle_position_from_vert_ids(mesh, [vert])
                if not pos:
                    mc.warning("Could not compute position.")
                    return


                null = mc.group(em=True, n=f"seq_guide_NULL_{idx:02d}")
                jnt = mc.joint(n=f"seq_guide_{idx:02d}")

                mc.parent(jnt, null)
                mc.parent(null, get_preview_grp())

                mc.xform(null, ws=True, t=pos)

                # Custom attrs
                if not mc.attributeQuery("vertList", n=jnt, ex=True):
                    mc.addAttr(jnt, ln="vertList", dt="string")
                if not mc.attributeQuery("mesh", n=jnt, ex=True):
                    mc.addAttr(jnt, ln="mesh", dt="string")
                if not mc.attributeQuery("upVectorVert", n=jnt, ex=True):
                    mc.addAttr(jnt, ln="upVectorVert", dt="string")

                mc.setAttr(jnt+".vertList", json.dumps([vert]), type="string")
                mc.setAttr(jnt+".mesh", mesh, type="string")
                mc.setAttr(jnt+".upVectorVert", "None", type="string")

                self.last_joint = jnt

                print(f"Created {jnt}")
                idx = idx + 1

        if self.type_cb.currentText() == "plane":
            mesh = get_selected_mesh()
            
            if not mesh:
                mc.warning("No mesh selected.")
                return

            vert_ids = get_selected_vert_ids_in_order()
            if not vert_ids:
                mc.warning("No verts selected.")
                return

            pos = get_middle_position_from_vert_ids(mesh, vert_ids)
            if not pos:
                mc.warning("Could not compute position.")
                return

            plane = build_plane_from_selected_verts()

            idx = get_next_chain_index()

            null = mc.group(em=True, n=f"plane_guide_NULL_{idx:02d}")
            jnt = mc.joint(n=f"plane_guide_{idx:02d}")

            mc.parent(jnt, null)
            mc.parent(plane, get_preview_grp())
            mc.parent(null, get_preview_grp())

            mc.xform(null, ws=True, t=pos)

            # Custom attrs
            if not mc.attributeQuery("vertList", n=jnt, ex=True):
                mc.addAttr(jnt, ln="vertList", dt="string")
            if not mc.attributeQuery("mesh", n=jnt, ex=True):
                mc.addAttr(jnt, ln="mesh", dt="string")

            mc.setAttr(jnt+".vertList", json.dumps(vert_ids), type="string")
            mc.setAttr(jnt+".mesh", mesh, type="string")
            mc.setAttr(jnt+".upVectorVert", "None", type="string")

            self.last_joint = jnt

            print(f"Created {jnt}")


        
        else:
            #if self.type_cb.currentText() not in  ["chain", 'sequence']:
            mc.warning("Only chain sequence and plane mode implemented.")
            return

    # -------------------------
    # STORE UP VECTOR
    # -------------------------

    def store_up_vector(self):
        if not self.last_joint:
            mc.warning("No guide created yet.")
            return

        verts = get_selected_vert_ids_in_order()
        if not verts:
            mc.warning("Select a vert for up vector.")
            return

        mc.setAttr(self.last_joint+".upVectorVert", str(verts[0]), type="string")
        print("Stored up vector:", verts[0])

    # -------------------------
    # WRITE PART JSON
    # -------------------------

    def write_part(self):

        path = r"G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\build\guides\parts"
        os.makedirs(path, exist_ok=True)

        data = {
            "part": self.part_le.text(),
            "type": self.type_cb.currentText(),
            "parent": self.parent_le.text(),
            "guides": {}
        }

        if self.type_cb.currentText() == 'chain':
            guides = mc.ls("chain_guide_*", type="joint")
        elif self.type_cb.currentText() == 'sequence':
            guides = mc.ls("seq_guide_*", type="joint")
        elif self.type_cb.currentText() == 'plane':
            guides = mc.ls("seq_guide_*", type="joint")

        for jnt in guides:
            null = mc.listRelatives(jnt, p=True)[0]

            mesh = mc.getAttr(jnt+".mesh")
            vert_list = json.loads(mc.getAttr(jnt+".vertList"))
            if self.type_cb.currentText() == 'plane':
                if mc.objExists('pPlane1'):
                    upvect = mc.xform('pPlane1', q=True, s=True, r=True)
                else:
                    upvect=None
            else:
                upvect = mc.getAttr(jnt+".upVectorVert")

            pos = mc.xform(null, q=True, ws=True, t=True)

            data["guides"][jnt] = {
                "mesh": mesh,
                "pos": pos,
                "offset": mc.xform(jnt, q=True, t=True),
                "rotoffset": mc.xform(jnt, q=True, ro=True),
                "upvect": None if upvect=="None" else int(upvect),
                "vert_list": vert_list
            }

        file_path = os.path.join(path, f"{data['part']}.json")

        with open(file_path, "w") as f:
            json.dump(data, f, indent=4)

        print("Wrote guide file:", file_path)


    # -------------------------
    # SEQUENCE RECORDER LOGIC
    # -------------------------

    def _on_selection_changed(self):
        current = set(mc.ls(sl=True, fl=True) or [])

        added = current - self._last_selection

        for item in added:
            if '.vtx[' in item:
                self.ordered_verts.append(item)

        self._last_selection = current


    def start_sequence_recording(self):
        self.stop_sequence_recording()  # safety

        self.ordered_verts.clear()
        self._last_selection = set(mc.ls(sl=True, fl=True) or [])

        self._script_job_id = mc.scriptJob(
            event=["SelectionChanged", self._on_selection_changed],
            protected=True
        )

        # Button states
        self.seq_start_btn.setEnabled(False)
        self.seq_stop_btn.setEnabled(True)

        print("▶ Sequence recording started")


    def stop_sequence_recording(self):
        if self._script_job_id and mc.scriptJob(exists=self._script_job_id):
            mc.scriptJob(kill=self._script_job_id, force=True)

        self._script_job_id = None

        # Button states
        self.seq_start_btn.setEnabled(True)
        self.seq_stop_btn.setEnabled(False)

        print("■ Sequence recording stopped")
        print("Recorded verts:", self.ordered_verts)

    def get_recorded_vert_ids(self):
        """Return recorded vertex IDs as ints (no mesh name)."""
        vert_ids = []

        for v in self.ordered_verts:
            try:
                idx = int(v.split('[')[-1].rstrip(']'))
                vert_ids.append(idx)
            except ValueError:
                pass

        return vert_ids

    def clear_preview(self):
        mc.delete('preview_guide_grp')

# -----------------------------
# Launch
# -----------------------------

def show_guide_write_tool():
    global guide_write_tool
    try:
        guide_write_tool.close()
    except:
        pass

    guide_write_tool = GuideWriteTool()
    guide_write_tool.show()

show_guide_write_tool()


from __future__ import annotations
import json
import os
import maya.cmds as mc
import maya.api.OpenMaya as om
import numpy as np

# ---------------- PYQT FALLBACK ----------------
try:
    from PySide6 import QtWidgets, QtCore
    from shiboken6 import wrapInstance
except:
    from PySide2 import QtWidgets, QtCore
    from shiboken2 import wrapInstance

import maya.OpenMayaUI as omui

# ---------------- PATH ----------------

GUIDE_PATH = r"G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\build\guides\parts"

BUILD_ORDER = [
    "spine.json",
    "neck.json",
    "fullleg.json",
    "clavicle.json",
    "arm.json",
    "indexfinger.json",
    "middlefinger.json",
    "ringfinger.json",
    "pinkyfinger.json",
    "thumbfinger.json",
    "UEbase.json",
    "UEbrow.json",
    "UEcheek.json",
    "UEear.json",
    "UEeyeball.json",
    "UEeyeiris.json",
    "UEeyepupil.json",
    "UEeyelid.json",
    "UEeyesocket.json",
    "UEjaw.json",
    "UEjawextras.json",
    "UEmouth.json",
    "UEmouthcenter.json",
    "UEnose.json",
    "UEnosebase.json",
    "UEteeth.json",
    "UEbotteeth.json",
    "UEtopteeth.json",
    "UEtounge.json",


]


# ---------------- CONFIG ----------------

PART_CONFIG = {
    "arm": {
        "Axes": ["Y", "-Z", "X"],
        "Names": ["LeftArm", "LeftForeArm", "LeftHand"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":True

    },
    "default": {
        "Axes": ["Y", "-X", "Z"],
        "Names": None,
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "clavicle": {
        "Axes": ["Y", "-Z", "X"],
        "Names": ["LeftShoulder"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":False
    },

    "foot": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftFoot", "LeftToeBase", "LeftToe_End"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":False
    },

    "indexfinger": {
        "Axes": ["Y", "-X", "Z"],
        "Names": ["LeftHandIndex0", "LeftHandIndex1", "LeftHandIndex2", "LeftHandIndex3", "LeftHandIndex4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":False
    },

    "middlefinger": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftHandMiddle0", "LeftHandMiddle1", "LeftHandMiddle2", "LeftHandMiddle3", "LeftHandMiddle4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":False
    },

    "ringfinger": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftHandRing0", "LeftHandRing1", "LeftHandRing2", "LeftHandRing3", "LeftHandRing4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":False
    },

    "pinkyfinger": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftHandPinky0", "LeftHandPinky1", "LeftHandPinky2", "LeftHandPinky3", "LeftHandPinky4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":False
    },

    "thumbfinger": {
        "Axes": ["Y", "X", "Z"],
        "Names": ["LeftHandThumb1", "LeftHandThumb2", "LeftHandThumb3", "LeftHandThumb4"],
        "Delete_Last": False,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":False
    },

    "leg": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["LeftUpLeg", "LeftLeg"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":False
    },

    "fullleg": {
        "Axes": ["Y", "-Z", "X"],
        "Names": ["LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "LeftToe_End"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":True
    },

    "spine": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Hips", "Spine", "Spine1", "Spine2", ],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":True,
        "Bake_To_Orient":True,
        "Force_Planar":True
    },

    "neck": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Neck", "Neck1", "Neck2", "Head", "HeadTop_End"],
        "Delete_Last": True,
        "Guide_Type": 'Joints',
        "BuildParent":False,
        "Bake_To_Orient":True,
        "Force_Planar":True
    },

    "UEmouth": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Mouth_M_UpperLip_01", "Mouth_L_UpperLip_02", "Mouth_L_UpperLip_03", "Mouth_L_UpperLip_04", "Mouth_L_UpperLip_05", "Mouth_L_CornerLip", "Mouth_L_LowerLip_05", "Mouth_L_LowerLip_04", "Mouth_L_LowerLip_03", "Mouth_L_LowerLip_02", "Mouth_M_LowerLip_01"],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEeyeball": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Eye_L_EyeCenterPivot", "Eye_L_Aim",],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEeyeiris": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Eye_L_Iris_01", "Eye_L_Iris_02", "Eye_L_Iris_03", "Eye_L_Iris_04", "Eye_L_Iris_05", "Eye_L_Iris_06", "Eye_L_Iris_07", "Eye_L_Iris_08", "Eye_L_Iris_09", "Eye_L_Iris_10", "Eye_L_Iris_11", "Eye_L_Iris_12", "Eye_L_Iris_13", "Eye_L_Iris_14", "Eye_L_Iris_15", "Eye_L_Iris_16"],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEeyepupil": {
        "Axes": ["Y", "Z", "X"],
        "Names": ["Eye_L_Pupil_01", "Eye_L_Pupil_02", "Eye_L_Pupil_03", "Eye_L_Pupil_04", "Eye_L_Pupil_05", "Eye_L_Pupil_06", "Eye_L_Pupil_07", "Eye_L_Pupil_08", "Eye_L_Pupil_09", "Eye_L_Pupil_10", "Eye_L_Pupil_11", "Eye_L_Pupil_12", "Eye_L_Pupil_13", "Eye_L_Pupil_14", "Eye_L_Pupil_15", "Eye_L_Pupil_16"],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEeyelid": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['Eye_L_Eyelid_InnerCorner', 'Eye_L_Eyelid_InnerUpper01', 'Eye_L_Eyelid_Upper', 'Eye_L_Eyelid_OuterUpper01', 'Eye_L_Eyelid_OuterCorner', 'Eye_L_Eyelid_OuterLower01', 'Eye_L_Eyelid_Lower', 'Eye_L_Eyelid_InnerLower01'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEeyesocket": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['Eye_L_Socket_InnerCorner', 'Eye_L_Socket_InnerUpper01', 'Eye_L_Socket_Upper', 'Eye_L_Socket_OuterUpper01', 'Eye_L_Socket_OuterCorner', 'Eye_L_Socket_OuterLower01', 'Eye_L_Socket_Lower', 'Eye_L_Socket_InnerLower01'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEbase": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['UpperHead_guide', 'LowerHead_guide'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEbase": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['UpperHead_guide', 'LowerHead_guide'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEbotteeth": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['botTeeth_L_Sub_03', 'botTeeth_L_Sub_02', 'botTeeth_M_Sub_01'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEtopteeth": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['topTeeth_L_Sub_03', 'topTeeth_L_Sub_02', 'topTeeth_M_Sub_01'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEteeth": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['topTeeth', 'botTeeth',],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEbrow": {
        "Axes": ["Y", "Z", "X"],
        "Names": ['Brow_L_Crease', 'Brow_L_01_Upper', 'Brow_L_01', 'Brow_L_01_Lower', 'Brow_L_02_Upper', 'Brow_L_02', 'Brow_L_02_Lower', 'Brow_L_01_Major', 'Brow_L_03_Upper', 'Brow_L_03', 'Brow_L_03_Lower', 'Brow_L_02_Major', 'Brow_L_04_Upper', 'Brow_L_04', 'Brow_L_04_Lower', 'Brow_L_05_Upper', 'Brow_L_05', 'Brow_L_05_Lower'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEbrow": { #
        "Axes": ["Y", "Z", "X"],
        "Names": ['Brow_L_Crease', 'Brow_L_01_Upper', 'Brow_L_01', 'Brow_L_01_Lower', 'Brow_L_02_Upper', 'Brow_L_02', 'Brow_L_02_Lower', 'Brow_L_01_Major', 'Brow_L_03_Upper', 'Brow_L_03', 'Brow_L_03_Lower', 'Brow_L_02_Major', 'Brow_L_04_Upper', 'Brow_L_04', 'Brow_L_04_Lower', 'Brow_L_05_Upper', 'Brow_L_05', 'Brow_L_05_Lower'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEcheek": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Cheek_L_NLFold_01_inner', 'Cheek_L_NLFold_01_outer', 'Cheek_L_NLFold_02_inner', 'Cheek_L_NLFold_02_outer', 'Cheek_L_NLFold_03_inner', 'Cheek_L_NLFold_03_outer', 'Cheek_L_NLFold_04_inner', 'Cheek_L_NLFold_04_outer', 'Cheek_L_NLFold_05_inner', 'Cheek_L_NLFold_05_outer', 'Cheek_L_NLFold_04', 'Cheek_L_Puff', 'Cheek_L_CheekBone'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    },

    "UEear": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Ear_L_Root', 'Ear_L_Upper', 'Ear_L_Outer', 'Ear_L_Lower'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    }, 

    "UEjaw": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Jaw_M_root', 'Jaw_M_ee',],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    }, 

    "UEjaw": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Jaw_M_root', 'Jaw_M_ee',],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    }, 

    "UEjawextras": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Jaw_M_larynx', 'Jaw_M_Chin',],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    }, 

    "UEnose": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Nose_M_NoseBridge', 'Nose_M_Tip', 'Nose_L_UpperCorner', 'Nose_L_Nostril_Outer', 'Nose_L_Nostril', 'Nose_M_Nostril_Inner'],
        "Delete_Last": False,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    }, 

    "UEnosebase": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Nose_M_NoseRoot'],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    }, 

    "UEtounge": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Tongue_01', 'Tongue_02', 'Tongue_03', 'Tongue_04', 'Tongue_05', 'Tongue_06'],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
    }, 

    "UEmouthcenter": { 
        "Axes": ["Y", "Z", "X"],
        "Names": ['Mouth_M_center'],
        "Delete_Last": True,
        "Guide_Type": 'Loc',
        "BuildParent":True,
        "Bake_To_Orient":False,
        "Force_Planar":False
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

#

def build_plane_from_selected_verts():

    verts = mc.filterExpand(mc.ls(sl=True, fl=True), sm=31)
    if not verts:
        mc.warning("Select verts.")
        return

    # -----------------------------
    # Get positions
    # -----------------------------

    points = []

    for v in verts:
        pos = mc.xform(v, q=True, ws=True, t=True)
        points.append(pos)

    points = np.array(points)

    # -----------------------------
    # center
    # -----------------------------

    center = points.mean(axis=0)

    # -----------------------------
    # best-fit plane
    # -----------------------------

    cov = np.cov(points.T)
    eigvals, eigvecs = np.linalg.eig(cov)

    normal = eigvecs[:, eigvals.argmin()]
    normal = om.MVector(*normal).normalize()

    # -----------------------------
    # build axes
    # -----------------------------

    up = om.MVector(0,1,0)

    if abs(normal * up) > 0.99:
        up = om.MVector(1,0,0)

    tangent = normal ^ up
    tangent.normalize()

    bitangent = normal ^ tangent
    bitangent.normalize()

    # -----------------------------
    # plane-space bounds
    # -----------------------------

    u_vals = []
    v_vals = []

    center_vec = om.MVector(*center)

    for p in points:

        vec = om.MVector(*p) - center_vec

        u_vals.append(vec * tangent)
        v_vals.append(vec * bitangent)

    width = max(u_vals) - min(u_vals)
    height = max(v_vals) - min(v_vals)

    # -----------------------------
    # create plane with center loops
    # -----------------------------

    plane = mc.polyPlane(
        w=width,
        h=height,
        sx=2,
        sy=2
    )[0]

    # -----------------------------
    # build transform matrix
    # -----------------------------

    matrix = [
        tangent.x, tangent.y, tangent.z, 0,
        normal.x, normal.y, normal.z, 0,
        bitangent.x, bitangent.y, bitangent.z, 0,
        center[0], center[1], center[2], 1
    ]

    mc.xform(plane, matrix=matrix)

    # -----------------------------
    # store attrs
    # -----------------------------

    orient_data = [
        [tangent.x, tangent.y, tangent.z],
        [normal.x, normal.y, normal.z],
        [bitangent.x, bitangent.y, bitangent.z]
    ]

    size_data = [width, height]

    if not mc.attributeQuery("guideOrient", n=plane, ex=True):
        mc.addAttr(plane, ln="guideOrient", dt="string")

    if not mc.attributeQuery("guideSize", n=plane, ex=True):
        mc.addAttr(plane, ln="guideSize", dt="string")

    mc.setAttr(plane + ".guideOrient", json.dumps(orient_data), type="string")
    mc.setAttr(plane + ".guideSize", json.dumps(size_data), type="string")

    print("Created guide plane:", plane)

    return plane


# ---------------- Mirror ---------------------

def flip_arm():
    clav = 'LeftShoulder'
    if mc.objExists('RightShoulder'):
        mc.delete('RightShoulder')
    mc.select(clav)
    mc.mirrorJoint(mirrorYZ=True, mirrorBehavior=True, searchReplace=('Left', 'Right'))

def flip_legs():
    hip = 'LeftUpLeg'
    parent = mc.listRelatives('LeftFoot', parent=True)
    if parent and parent[0] == 'LeftLeg':
        print("LeftFoot is directly parented to LeftLeg")
        mc.parent("LeftFoot", 'Hips')
    if mc.objExists('RightUpLeg'):
        mc.delete('RightUpLeg')
        
    mc.select(hip)
    mc.mirrorJoint(mirrorYZ=True, mirrorBehavior=True, searchReplace=('Left', 'Right'))

def flip_feet():
    foot = 'LeftFoot'
    if mc.objExists('RightFoot'):
        mc.delete('RightFoot')
    mc.select(foot)
    mc.mirrorJoint(mirrorYZ=True, mirrorBehavior=False, searchReplace=('Left', 'Right'))
    rename_map = {
            'LeftHeelPiv1': 'RightHeelPiv',
            'LeftIn1': 'RightIn',
            'LeftOut1': 'RightOut',
            'LeftToePiv1': 'RightToePiv'
        }

    for old_name, new_name in rename_map.items():
        if mc.objExists(old_name):
            mc.rename(old_name, new_name)

def mirror_face():
    for grp in ['Brow_L_guides', 'Cheek_L_guides', 'Ear_L_guides', 'Eye_L_guides']:
        if not mc.objExists(grp):
            mc.warning(f"{grp} not found")
            return

        # children only (no root)
        children = mc.listRelatives(grp, children=True, f=True, type="transform") or []

        new_name = grp.replace("_L_", "_R_")
        
        flipgrp = mc.group(empty=True, name=new_name)
        mc.parent(flipgrp, 'UEFace_guides')

        for child in children:

            short = child.split('|')[-1]

            if "_L_" not in short:
                continue

            new_name = short.replace("_L_", "_R_")

            # duplicate transform + shapes
            dup = mc.duplicate(child, rr=True)[0]
            dup = mc.rename(dup, new_name)

            # -------- store rotation --------
            rot = mc.getAttr(dup + ".rotate")[0]   # (rx, ry, rz)

            # -------- zero rotations --------
            mc.setAttr(dup + ".rotate", 0, 0, 0)

            # -------- flip translate X --------
            tx = mc.getAttr(dup + ".translateX")
            mc.setAttr(dup + ".translateX", tx * -1)

            # -------- reapply rotations (invert Y) --------
            rx, ry, rz = rot
            mc.setAttr(dup + ".rotate", rx, ry* -1, rz* -1)
            mc.parent(dup, flipgrp)


    for grp in ['Mouth_guides', 'Nose_guides', 'Tongue_M_guides']:
        if not mc.objExists(grp):
            mc.warning(f"{grp} not found")
            return

        # children only (no root)
        children = mc.listRelatives(grp, children=True, f=True, type="transform") or []

        for child in children:

            short = child.split('|')[-1]

            if "_L_" not in short:
                continue

            new_name = short.replace("_L_", "_R_")

            # duplicate transform + shapes
            dup = mc.duplicate(child, rr=True)[0]
            dup = mc.rename(dup, new_name)

            # -------- store rotation --------
            rot = mc.getAttr(dup + ".rotate")[0]   # (rx, ry, rz)

            # -------- zero rotations --------
            mc.setAttr(dup + ".rotate", 0, 0, 0)

            # -------- flip translate X --------
            tx = mc.getAttr(dup + ".translateX")
            mc.setAttr(dup + ".translateX", tx * -1)

            # -------- reapply rotations (invert Y) --------
            rx, ry, rz = rot
            mc.setAttr(dup + ".rotate", rx, ry* -1, rz* -1)



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
    elif part_type == "plane":
        read_plane_guides(json_file)


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
    Guide_Type = cfg["Guide_Type"]
    Bake_To_Orient = cfg["Bake_To_Orient"]
    Force_Planar = cfg["Force_Planar"]

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

        rotoffset = g.get("rotoffset")
        up_id = g["upvect"]

        pos = get_position_from_vert_ids(mesh, vert_list)
        if not pos:
            continue

        pos = [
            pos[0] + offset[0],
            pos[1] + offset[1],
            pos[2] + offset[2]
        ]

        if names and i < len(names):
            jnt_name = names[i]
        else:
            jnt_name = f"{part}_guide_{i+1:02d}"

        # -------- CREATE GUIDE --------

        mc.select(clear=True)

        if Guide_Type == 'Loc':
            jnt = mc.spaceLocator(name=jnt_name)[0]
        else:
            jnt = mc.joint(name=jnt_name)

        mc.xform(jnt, ws=True, t=pos)

        built.append(jnt)

        up_pos = get_upvect_position(mesh, up_id)

        # -------- ORIENT --------

        if Guide_Type == 'Joints':

            if i < len(keys)-1:

                next_g = guides[keys[i+1]]
                next_pos = get_position_from_vert_ids(
                    next_g["mesh"],
                    next_g["vert_list"]
                )

                if next_pos:
                    orient_joint_primary(
                        jnt,
                        pos,
                        next_pos,
                        axes[0],
                        up_pos,
                        axes[1]
                    )

        # -------- ROT OFFSET --------

        if rotoffset:
            rot = mc.getAttr(jnt + ".rotate")[0]

            mc.setAttr(
                jnt + ".rotate",
                rotoffset[0] + rot[0],
                rotoffset[1] + rot[1],
                rotoffset[2] + rot[2]
            )

        # =====================================================
        # FORCE PLANAR MODE
        # =====================================================

        if Force_Planar:

            ax0 = axes[0].replace("-", "")
            ax1 = axes[1].replace("-", "")
            ax2 = axes[2].replace("-", "")

            # ---- Every joint ----

            mc.setAttr(f"{jnt}.rotate{ax0}", 0)

            r1 = mc.getAttr(f"{jnt}.rotate{ax1}")

            if abs(abs(r1) - 180) <= 15:
                mc.setAttr(f"{jnt}.rotate{ax1}", 180)

            # ---- First guide ----

            if i == 0:
                if parent and mc.objExists(parent):
                    mc.parent(jnt, parent)
                
                mc.makeIdentity(
                    jnt,
                    apply=True,
                    rotate=True,
                    translate=False,
                    scale=False
                )

                if jnt_name == 'Hips':
                    mc.setAttr(f'{jnt}.jointOrientZ', 0)

            # ---- Child guides ----

            else:
                mc.parent(jnt, pre_guide)

                mc.setAttr(f"{jnt}.translate{ax1}", 0)
                mc.setAttr(f"{jnt}.translate{ax2}", 0)

                mc.setAttr(f"{jnt}.rotate{ax0}", 0)

                mc.makeIdentity(
                    jnt,
                    apply=True,
                    rotate=True,
                    translate=False,
                    scale=False
                )

                # force true planar joint orient
                mc.setAttr(f"{jnt}.jointOrient{ax0}", 0)
                mc.setAttr(f"{jnt}.jointOrient{ax1}", 0)
                if i == 4:
                    mc.setAttr(f"{jnt}.jointOrient{ax2}", 0)

        # =====================================================
        # NORMAL MODE (unchanged behavior)
        # =====================================================

        else:

            if Guide_Type == 'Joints' and Bake_To_Orient:

                rot = mc.getAttr(jnt + ".rotate")[0]
                jo = mc.getAttr(jnt + ".jointOrient")[0]

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

    # -------- FINAL PARENTING --------

    if not Force_Planar:

        for i in range(1, len(built)):
            mc.parent(built[i], built[i-1])

        if parent and mc.objExists(parent):
            mc.parent(built[0], parent)

        elif BuildParent:
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
        if not mc.objExists(parent):

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

def read_plane_guides(json_file):

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
    Guide_Type = cfg["Guide_Type"]
    Bake_To_Orient = cfg["Bake_To_Orient"]
    Force_Planar = cfg["Force_Planar"]

    guides = data["guides"]

    built = []
    keys = sorted(guides.keys())
    pre_guide = None

    gname = keys[0]
    g = guides[gname]

    mesh = g["mesh"]
    vert_list = g["vert_list"]
    offset = g["offset"]

    rotoffset = g.get("rotoffset")
    up_id = g["upvect"]

    verts = [f"{mesh}.vtx[{i}]" for i in vert_list]

    mc.select(verts, r=True)
    plane = build_plane_from_selected_verts()


    vert_ids = [1, 3, 5, 7]
    names = ["LeftIn", "LeftHeelPiv", "LeftToePiv", "LeftOut"]

    built=[]
    for vid, name in zip(vert_ids, names):

        pos = get_position_from_vert_ids(plane, [vid])

        if not pos:
            continue

        loc = mc.spaceLocator(n=name)[0]

        mc.xform(loc, ws=True, t=pos)

        mc.parent(loc, "Guides")
        built.append(loc)

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


def build_all_guides():

    mc.group(empty=True, name ='Guides')

    if not os.path.exists(GUIDE_PATH):
        mc.warning("Guide path not found.")
        return

    for filename in BUILD_ORDER:

        path = os.path.join(GUIDE_PATH, filename)

        if not os.path.exists(path):
            mc.warning(f"Missing guide file: {filename}")
            continue

        print(f"Building: {filename}")
        read_type(path)

    guides = ['Jaw_M_ee', 'Eye_L_Aim', 'LowerHead_guide', 'botTeeth', 'Tongue_02', 'Tongue_03', 'Tongue_04', 'Tongue_05', 'Tongue_06' ]

    for guide in guides:
        # Get parent
        parent = mc.listRelatives(guide, parent=True, fullPath=True)
        if not parent:
            continue  # skip if no parent

        # Get grandparent
        grandparent = mc.listRelatives(parent[0], parent=True, fullPath=True)
        if grandparent:
            mc.parent(guide, grandparent[0])
            print(f"{guide} reparented to {grandparent[0]}")
        else:
            print(f"{guide} has no grandparent, skipping")


    flip_arm()
    flip_legs()
    flip_feet()
    mirror_face()

    mc.parent('Tongue_M_guides', 'Nose_guides', 'Mouth_guides', 'Jaw_M_guides', 'Eye_L_guides', 'Ear_L_guides', 'Cheek_L_guides', 'Brow_L_guides', 'UEFace_guides')
    mc.parent('UEFace_guides', 'Guides')





    # hand and foot Pivots #HipPivot # fix chain parenting #mouth_guides not Mouth_M_guides    #center mirror creating weird shape issues #top teeth not coming in  # center Mirror Rotations need to be baked 



    print("Build All complete.")


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

        self.build_all_btn = QtWidgets.QPushButton("Build All")
        layout.addWidget(self.build_all_btn)
        self.build_all_btn.clicked.connect(build_all_guides)

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


import maya.cmds as mc

class SplitJointTool:
    def __init__(self):
        self.root_joint = None
        self.chain_joints = []

        self.build_ui()

    def build_ui(self):
        if mc.window("splitJointToolWin", exists=True):
            mc.deleteUI("splitJointToolWin")

        self.window = mc.window("splitJointToolWin", title="Split Joint Setup", widthHeight=(250,120))
        mc.columnLayout(adjustableColumn=True, rowSpacing=10)

        mc.button(label="Set Root", height=30, command=self.set_root)
        mc.button(label="Set Chain", height=30, command=self.set_chain)
        mc.button(label="Apply", height=30, command=self.apply)

        mc.showWindow(self.window)

    # -------------------------
    # Button Functions
    # -------------------------

    def set_root(self, *args):
        sel = mc.ls(selection=True)

        if not sel:
            mc.warning("Please select a root joint.")
            return

        joint = sel[0]

        if mc.nodeType(joint) != "joint":
            mc.warning("Selected object is not a joint.")
            return

        self.root_joint = joint
        print(f"Root joint set to: {self.root_joint}")


    def set_chain(self, *args):
        # ensure ordered selection works
        mc.selectPref(trackSelectionOrder=True)

        sel = mc.ls(orderedSelection=True)

        if not sel:
            mc.warning("Please select joints for the chain.")
            return

        for obj in sel:
            if mc.nodeType(obj) != "joint":
                mc.warning(f"{obj} is not a joint.")
                return

        self.chain_joints = sel
        print("Chain joints set to:")
        for j in self.chain_joints:
            print(f"  {j}")


    def apply(self, *args):
        if not self.root_joint:
            mc.warning("Root joint not set.")
            return

        if not self.chain_joints:
            mc.warning("Chain joints not set.")
            return

        # create attribute
        if not mc.attributeQuery("split_joints", node=self.root_joint, exists=True):
            mc.addAttr(self.root_joint, longName="split_joints", dataType="string")

        mc.setAttr(
            f"{self.root_joint}.split_joints",
            repr(self.chain_joints),
            type="string"
        )

        print("Split joints attribute applied.")
        print(f"Root: {self.root_joint}")
        print(f"Chain: {self.chain_joints}")


def run():
    SplitJointTool()


run()

from __future__ import annotations
import os
import maya.cmds as mc

# ---------------- PYQT FALLBACK ----------------
try:
    from PySide6 import QtWidgets, QtCore
except ImportError:
    from PySide2 import QtWidgets, QtCore


# ----------------------------
# Snapshot Camera Utilities
# ----------------------------

def check_snapshot_camera():
    if mc.objExists("snapshot_cam_grp"):
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Snapshot Camera Exists")
        msg.setText("A snapshot camera already exists.")
        clear_btn = msg.addButton("Clear Camera", QtWidgets.QMessageBox.AcceptRole)
        cancel_btn = msg.addButton("Cancel", QtWidgets.QMessageBox.RejectRole)
        msg.exec_()
        if msg.clickedButton() == clear_btn:
            mc.delete("snapshot_cam_grp")
            return True
        return False
    return True


def create_snapshot_camera():
    grp = mc.group(em=True, name="snapshot_cam_grp")
    swivel = mc.group(em=True, name="snapshot_cam_swivel", parent=grp)
    tilt = mc.group(em=True, name="snapshot_cam_tilt", parent=swivel)

    cam, shape = mc.camera(name="snapshot_cam")
    cam = mc.rename(cam, "snapshot_cam")
    mc.parent(cam, tilt)
    return cam


def calculate_auto_zoom(obj, fit_factor=1.2):
    bbox = mc.exactWorldBoundingBox(obj)
    size_x = bbox[3] - bbox[0]
    size_y = bbox[4] - bbox[1]
    size_z = bbox[5] - bbox[2]
    max_size = max(size_x, size_y, size_z)
    distance = max_size * fit_factor
    return distance

def auto_frame_camera(camera, obj):
    """
    Frame the object using Maya's native framing (same as pressing F).
    """
    if not mc.objExists(obj):
        return

    # Look through the camera
    mc.lookThru(camera)

    # Select the object
    mc.select(obj)

    # Frame selection (same as hitting F)
    mc.viewFit(camera, all=False)

    # Clear selection
    mc.select(clear=True)

def place_snapshot_camera(obj, swivel=0, tilt=0, focal_length=35):
    cam = create_snapshot_camera()

    if not mc.objExists(obj):
        mc.warning("Object does not exist.")
        return None

    bbox = mc.exactWorldBoundingBox(obj)
    center = [
        (bbox[0] + bbox[3]) / 2,
        (bbox[1] + bbox[4]) / 2,
        (bbox[2] + bbox[5]) / 2
    ]

    mc.xform("snapshot_cam_grp", ws=True, t=center)

    mc.setAttr("snapshot_cam_swivel.rotateY", swivel)
    mc.setAttr("snapshot_cam_tilt.rotateX", tilt)

    # Set focal length
    shapes = mc.listRelatives(cam, shapes=True)
    if shapes:
        mc.setAttr(f"{shapes[0]}.focalLength", focal_length)

    # NEW: frame the object automatically
    auto_frame_camera(cam, obj)

    return cam

def take_snapshot(camera, path, width=512, height=512, name="snapshot"):
    if not os.path.exists(path):
        os.makedirs(path)

    mc.lookThru(camera)
    file = os.path.join(path, name)
    mc.playblast(
        frame=[mc.currentTime(q=True)],
        format="image",
        filename=file,
        viewer=False,
        compression="png",
        percent=100,
        widthHeight=(width, height),
        forceOverwrite=True,
        offScreen=True
    )


def take_playblast(camera, path, start, end, width=512, height=512, name="playblast"):
    if not os.path.exists(path):
        os.makedirs(path)

    mc.lookThru(camera)
    file = os.path.join(path, name)
    mc.playblast(
        startTime=start,
        endTime=end,
        format="image",
        filename=file,
        viewer=False,
        compression="png",
        percent=100,
        widthHeight=(width, height),
        forceOverwrite=True,
        offScreen=True
    )


# ----------------------------
# Collapsible Section Widget
# ----------------------------

class CollapsibleSection(QtWidgets.QWidget):
    def __init__(self, title="Section"):
        super().__init__()
        self.toggle_btn = QtWidgets.QToolButton()
        self.toggle_btn.setText(title)
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setChecked(True)
        self.toggle_btn.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.toggle_btn.setArrowType(QtCore.Qt.DownArrow)
        self.toggle_btn.clicked.connect(self.toggle)
        self.content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.toggle_btn)
        layout.addWidget(self.content)

    def toggle(self):
        visible = self.toggle_btn.isChecked()
        self.content.setVisible(visible)
        self.toggle_btn.setArrowType(
            QtCore.Qt.DownArrow if visible else QtCore.Qt.RightArrow
        )


# ----------------------------
# Snapshot UI
# ----------------------------

class SnapshotUI(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Snapshot Tool")
        self.setMinimumWidth(400)
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)

        main_layout = QtWidgets.QVBoxLayout(self)

        # -------------------
        # Snapshot Mode Section
        # -------------------
        mode_section = CollapsibleSection("Snapshot Mode")
        mode_layout = QtWidgets.QVBoxLayout(mode_section.content)

        self.mode_dropdown = QtWidgets.QComboBox()
        self.mode_dropdown.addItems(["Headshot", "Front", "3_Quarters", "Turntable", "Manual"])
        mode_layout.addWidget(self.mode_dropdown)

        obj_layout = QtWidgets.QHBoxLayout()
        self.obj_field = QtWidgets.QLineEdit()
        self.pick_btn = QtWidgets.QPushButton("Pick")
        self.pick_btn.clicked.connect(self.pick_object)
        obj_layout.addWidget(self.obj_field)
        obj_layout.addWidget(self.pick_btn)
        mode_layout.addLayout(obj_layout)

        main_layout.addWidget(mode_section)

        # -------------------
        # Camera Settings Section
        # -------------------
        cam_section = CollapsibleSection("Camera Settings")
        cam_layout = QtWidgets.QFormLayout(cam_section.content)

        self.swivel = QtWidgets.QDoubleSpinBox()
        self.swivel.setRange(-360, 360)
        self.tilt = QtWidgets.QDoubleSpinBox()
        self.tilt.setRange(-360, 360)
        self.focal_length = QtWidgets.QDoubleSpinBox()
        self.focal_length.setRange(1, 300)
        self.focal_length.setValue(35)

        cam_layout.addRow("Swivel", self.swivel)
        cam_layout.addRow("Tilt", self.tilt)
        cam_layout.addRow("Focal Length", self.focal_length)

        # Width/Height inputs
        size_layout = QtWidgets.QHBoxLayout()
        self.img_width = QtWidgets.QSpinBox()
        self.img_width.setRange(16, 8192)
        self.img_width.setValue(512)
        self.img_width.setPrefix("W: ")
        self.img_height = QtWidgets.QSpinBox()
        self.img_height.setRange(16, 8192)
        self.img_height.setValue(512)
        self.img_height.setPrefix("H: ")
        size_layout.addWidget(self.img_width)
        size_layout.addWidget(self.img_height)
        cam_layout.addRow("Image Size", size_layout)

        cam_section.content.setLayout(cam_layout)
        main_layout.addWidget(cam_section)

        # -------------------
        # Output Section
        # -------------------
        output_section = CollapsibleSection("Output Settings")
        output_layout = QtWidgets.QHBoxLayout(output_section.content)

        # Default output path and snapshot name
        default_path = r"G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\camera_util"
        default_name = "snapshot"

        self.path_field = QtWidgets.QLineEdit(default_path)
        browse_btn = QtWidgets.QPushButton("Browse")
        browse_btn.clicked.connect(self.browse_folder)

        self.snapshot_name = QtWidgets.QLineEdit(default_name)
        self.snapshot_name.setPlaceholderText("Snapshot Name")

        output_layout.addWidget(self.path_field)
        output_layout.addWidget(browse_btn)
        output_layout.addWidget(self.snapshot_name)

        output_section.content.setLayout(output_layout)
        main_layout.addWidget(output_section)

        # -------------------
        # Capture Section
        # -------------------
        capture_section = CollapsibleSection("Capture Options")
        capture_layout = QtWidgets.QVBoxLayout(capture_section.content)

        # Frame range
        frame_layout = QtWidgets.QHBoxLayout()
        self.start_frame = QtWidgets.QSpinBox()
        self.start_frame.setValue(1)
        self.end_frame = QtWidgets.QSpinBox()
        self.end_frame.setValue(30)
        timeline_btn = QtWidgets.QPushButton("Use Timeline")
        timeline_btn.clicked.connect(self.copy_timeline_range)
        frame_layout.addWidget(self.start_frame)
        frame_layout.addWidget(self.end_frame)
        frame_layout.addWidget(timeline_btn)
        capture_layout.addLayout(frame_layout)

        # Buttons
        snap_btn = QtWidgets.QPushButton("Take Snapshot")
        snap_btn.clicked.connect(self.snapshot)
        playblast_btn = QtWidgets.QPushButton("Playblast")
        playblast_btn.clicked.connect(self.playblast)
        capture_layout.addWidget(snap_btn)
        capture_layout.addWidget(playblast_btn)

        capture_section.content.setLayout(capture_layout)
        main_layout.addWidget(capture_section)

        # -------------------
        # Manual Section
        # -------------------
        manual_section = CollapsibleSection("Manual Camera Controls")
        manual_layout = QtWidgets.QHBoxLayout(manual_section.content)

        self.place_manual_cam_btn = QtWidgets.QPushButton("Place Camera")
        self.place_manual_cam_btn.clicked.connect(self.place_manual_camera)

        self.take_manual_snap_btn = QtWidgets.QPushButton("Take Snapshot")
        self.take_manual_snap_btn.clicked.connect(self.take_manual_snapshot)

        manual_layout.addWidget(self.place_manual_cam_btn)
        manual_layout.addWidget(self.take_manual_snap_btn)

        manual_section.content.setLayout(manual_layout)
        main_layout.addWidget(manual_section)

    # -------------------
    # UI Functions
    # -------------------
    def pick_object(self):
        sel = mc.ls(sl=True)
        if sel:
            self.obj_field.setText(sel[0])
            self.pick_btn.setStyleSheet("background-color: green;")
        else:
            self.pick_btn.setStyleSheet("background-color: yellow;")

    def browse_folder(self):
        folder = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Output Folder")
        if folder:
            self.path_field.setText(folder)

    def copy_timeline_range(self):
        start = mc.playbackOptions(q=True, min=True)
        end = mc.playbackOptions(q=True, max=True)
        self.start_frame.setValue(int(start))
        self.end_frame.setValue(int(end))

    # -------------------
    # Capture Actions
    # -------------------

        # -------------------
    # Manual Camera Functions
    # -------------------

    def place_manual_camera(self):
        obj = self.obj_field.text()

        if not mc.objExists(obj):
            mc.warning("Object does not exist.")
            return

        # delete existing camera if present
        if mc.objExists("snapshot_cam_grp"):
            mc.delete("snapshot_cam_grp")

        self.manual_camera = create_snapshot_camera()

        # auto frame the object
        auto_frame_camera(self.manual_camera, obj)

        mc.select(clear=True)
        mc.inViewMessage(amg='Manual camera placed', pos='midCenter', fade=True)


    def take_manual_snapshot(self):

        if not hasattr(self, "manual_camera") or not self.manual_camera:
            mc.warning("No manual camera placed.")
            return

        path = self.path_field.text()
        width = self.img_width.value()
        height = self.img_height.value()
        name = self.snapshot_name.text() or "manual_snapshot"

        take_snapshot(self.manual_camera, path, width, height, name)


    def build_camera(self):
        if not check_snapshot_camera():
            return None
        obj = self.obj_field.text()
        if not mc.objExists(obj):
            mc.warning("Object does not exist.")
            return None
        cam = place_snapshot_camera(
            obj,
            swivel=self.swivel.value(),
            tilt=self.tilt.value(),
            focal_length=self.focal_length.value()
        )
        return cam

    def snapshot(self):
        cam = self.build_camera()
        if not cam:
            return
        path = self.path_field.text()
        width = self.img_width.value()
        height = self.img_height.value()
        name = self.snapshot_name.text() or "snapshot"
        take_snapshot(cam, path, width, height, name)

    def playblast(self):
        cam = self.build_camera()
        if not cam:
            return
        path = self.path_field.text()
        width = self.img_width.value()
        height = self.img_height.value()
        name = self.snapshot_name.text() or "playblast"
        take_playblast(cam, path, self.start_frame.value(), self.end_frame.value(), width, height, name)


# ----------------------------
# Launch Tool
# ----------------------------
def run():
    global snapshot_ui
    try:
        snapshot_ui.close()
    except:
        pass
    snapshot_ui = SnapshotUI()
    snapshot_ui.show()

run()


import os
import json
import dwpicker

try:
    from PySide6.QtGui import QImage
except ImportError:
    from PySide2.QtGui import QImage


def create_picker_from_snapshot(snapshot_path, output_picker_path):

    if not os.path.exists(snapshot_path):
        raise RuntimeError("Snapshot image does not exist.")

    # Read image size
    img = QImage(snapshot_path)

    width = img.width()
    height = img.height()

    picker_data = {
        "version": 1,
        "tabs": [
            {
                "title": "Main",
                "background": snapshot_path.replace("\\", "/"),
                "width": width,
                "height": height,
                "items": []
            }
        ]
    }

    with open(output_picker_path, "w") as f:
        json.dump(picker_data, f, indent=4)

    print("Picker created:", output_picker_path)


create_picker_from_snapshot(
    r"G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\camera_util\snapshot.0.png",
    r"G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\camera_util\snapshot_picker.json"
)

dwpicker.open_picker(r"G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\camera_util\snapshot_picker.json")


import json
import os
import uuid
import dwpicker
from dwpicker.templates import BUTTON


def _get_image_dimensions(image_path):
    """Read image dimensions - tries PySide6 then PySide2 (depends on Maya version)."""
    try:
        from PySide6.QtGui import QImage
        img = QImage(image_path)
        if img.isNull():
            raise ValueError("QImage returned null.")
        return img.width(), img.height()
    except ImportError:
        pass
    try:
        from PySide2.QtGui import QImage
        img = QImage(image_path)
        if img.isNull():
            raise ValueError("QImage returned null.")
        return img.width(), img.height()
    except ImportError:
        pass
    print("Warning: could not read image dimensions, defaulting to 1920x1080")
    return 1920, 1080


def _make_background_image_shape(image_path):
    """
    Build a locked, full-size background image shape using the real BUTTON template.
    """
    width, height = _get_image_dimensions(image_path)

    shape = BUTTON.copy()
    shape.update({
        # Required unique ID (needed by document.py)
        'id':                     str(uuid.uuid4()),

        # Position and size
        'shape.left':             0.0,
        'shape.top':              0.0,
        'shape.width':            float(width),
        'shape.height':           float(height),

        # Image settings
        'image.path':             image_path.replace("\\", "/"),
        'image.width':            width,
        'image.height':           height,
        'image.fit':              True,
        'image.ratio':            True,

        # Background - no text, no visible border
        'background':             True,
        'text.content':           '',
        'border':                 False,
        'bgcolor.normal':         '#000000',
        'bgcolor.hovered':        '#000000',
        'bgcolor.clicked':        '#000000',
        'bgcolor.transparency':   255,

        # No selection behaviour
        'action.targets':         [],
        'action.commands':        [],
        'action.menu_commands':   [],

        # Locked - can't be accidentally moved in editor
        'shape.ignored_by_focus': True,
    })

    return shape, width, height


def initialize_picker_from_shot(
        character_name,
        path,
        image,
        json_path,
        picker_des):
    """
    Create a blank dwpicker document with a locked background image and open it.

    Args:
        character_name (str): Name of the character e.g. 'Drago'
        path          (str): Directory where the image lives
        image         (str): Image filename e.g. 'drago_bg.png'
        json_path     (str): Full path to save the .json file to
        picker_des    (str): Short descriptor e.g. 'body'
    """
    picker_name = f"{character_name}_Picker_{picker_des}"
    image_path  = os.path.join(path, image)

    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Image not found: {image_path}")

    bg_shape, img_w, img_h = _make_background_image_shape(image_path)

    data = {
        'general': {
            'version':            [1, 0, 0],
            'name':               picker_name,
            'panels':             [[1.0, [1.0]]],
            'panels.orientation': 'vertical',
            'panels.zoom_locked': [False],
            'panels.as_sub_tab':  False,
            'panels.colors':      [None],
            'panels.names':       ['Panel 1'],
            'hidden_layers':      [],
            'menu_commands':      [],
        },
        'shapes': [bg_shape]
    }

    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"Picker '{picker_name}' saved to: {json_path}")
    print(f"Background: {image_path} ({img_w} x {img_h}px)")

    dwpicker.open_picker_file(json_path)
    return data
    
initialize_picker_from_shot(
    character_name = 'CrowdB',
    path           = r'G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\picker_files',
    image          = 'CrowdB.0.png',
    json_path      = r'G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\picker_files\crowdB_Picker.json',
    picker_des     = 'body'
)

import math
import maya.cmds as cmds


# ─── MATH HELPERS ─────────────────────────────────────────────────────────────

def _deg_to_rad(deg):
    return deg * math.pi / 180.0


def _multiply_matrix_vector(m, v):
    """Multiply a 3x3 rotation matrix by a 3D vector."""
    return [
        m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
        m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
        m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2],
    ]


def _rotation_matrix_y(deg):
    """Rotation matrix around Y axis (swivel)."""
    r = _deg_to_rad(deg)
    c, s = math.cos(r), math.sin(r)
    return [
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c],
    ]


def _rotation_matrix_x(deg):
    """Rotation matrix around X axis (tilt)."""
    r = _deg_to_rad(deg)
    c, s = math.cos(r), math.sin(r)
    return [
        [1,  0,  0],
        [0,  c, -s],
        [0,  s,  c],
    ]


def _multiply_matrices(a, b):
    """Multiply two 3x3 matrices."""
    result = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                result[i][j] += a[i][k] * b[k][j]
    return result


# ─── CORE PROJECTION ──────────────────────────────────────────────────────────

def get_2d_position(control, camera_data, image_width, image_height):
    """
    Project a Maya control's world position onto 2D image/picker space
    using the orthographic snapshot camera settings.

    Args:
        control      (str):  Maya control node name
        camera_data  (dict): Loaded camera JSON from export_camera_json()
        image_width  (int):  Snapshot image width in pixels
        image_height (int):  Snapshot image height in pixels

    Returns:
        (float, float): (x, y) position in picker pixel space
    """
    # World position of the control
    world_pos = cmds.xform(control, query=True, worldSpace=True, translation=True)

    # Camera pivot (group_translate is the world position of snapshot_cam_grp)
    pivot = camera_data['group_translate']

    # Vector from camera pivot to control in world space
    relative = [
        world_pos[0] - pivot[0],
        world_pos[1] - pivot[1],
        world_pos[2] - pivot[2],
    ]

    # Reconstruct the camera's combined rotation: swivel (Y) then tilt (X)
    # This mirrors how snapshot_cam_grp > snapshot_cam_swivel > snapshot_cam_tilt is built
    swivel_mat = _rotation_matrix_y(camera_data['swivel'])
    tilt_mat   = _rotation_matrix_x(camera_data['tilt'])
    view_mat   = _multiply_matrices(tilt_mat, swivel_mat)

    # Transform relative world vector into camera space
    cam_space = _multiply_matrix_vector(view_mat, relative)

    # cam_space[0] = right/left  (maps to image X)
    # cam_space[1] = up/down     (maps to image Y)
    # cam_space[2] = depth       (ignored for orthographic)

    ortho_width  = camera_data['orthographicWidth']
    ortho_height = ortho_width * (image_height / image_width)

    # Normalize to -0.5 .. +0.5 then map to pixel space
    norm_x =  cam_space[0] / ortho_width
    norm_y = -cam_space[1] / ortho_height   # flip Y (image Y goes downward)

    pixel_x = (norm_x + 0.5) * image_width
    pixel_y = (norm_y + 0.5) * image_height

    return pixel_x, pixel_y


# ─── CONTROL COLOR ────────────────────────────────────────────────────────────

def _get_control_color(control):
    """
    Get the wireframe override color of a Maya control as a hex string.
    Falls back to a neutral grey if no override is set.
    """
    # Maya color index -> approximate hex
    maya_index_colors = {
        0:  '#808080', 1:  '#000000', 2:  '#404040', 3:  '#808080',
        4:  '#800000', 5:  '#000040', 6:  '#000080', 7:  '#002020',
        8:  '#200020', 9:  '#800040', 10: '#804020', 11: '#404000',
        12: '#802000', 13: '#FF0000', 14: '#00FF00', 15: '#0040FF',
        16: '#FFFFFF', 17: '#FFFF00', 18: '#40C0FF', 19: '#40FF40',
        20: '#FF8080', 21: '#FF8040', 22: '#FFFF80', 23: '#008040',
        24: '#804040', 25: '#808040', 26: '#408040', 27: '#408080',
        28: '#4040FF', 29: '#FF40FF', 30: '#FF8040', 31: '#FF4040',
    }

    shapes = cmds.listRelatives(control, shapes=True) or []
    node = shapes[0] if shapes else control

    # Check for RGB override first (Maya 2015+)
    try:
        if cmds.getAttr(node + '.overrideRGBColors'):
            r = cmds.getAttr(node + '.overrideColorR')
            g = cmds.getAttr(node + '.overrideColorG')
            b = cmds.getAttr(node + '.overrideColorB')
            return '#{:02X}{:02X}{:02X}'.format(
                int(r * 255), int(g * 255), int(b * 255)
            )
    except Exception:
        pass

    # Fall back to color index
    try:
        if cmds.getAttr(node + '.overrideEnabled'):
            index = cmds.getAttr(node + '.overrideColor')
            return maya_index_colors.get(index, '#888888')
    except Exception:
        pass

    return '#888888'


# ─── BUTTON PLACEMENT ─────────────────────────────────────────────────────────

def place_button_on_picker(picker, control, position_2d, width_offset=0, height_offset=0):
    """
    Add a button to the picker at the given 2D position,
    colored to match the control's wireframe color.

    Args:
        picker        : Active dwpicker instance from dwpicker.current()
        control  (str): Maya control node name
        position_2d   : (x, y) in picker pixel space
        width_offset  : Extra width added to default button size
        height_offset : Extra height added to default button size
    """
    import uuid
    from dwpicker.templates import BUTTON

    color = _get_control_color(control)
    x, y  = position_2d

    btn = BUTTON.copy()
    btn.update({
        'id':                     str(uuid.uuid4()),
        'text.content':           control.split(':')[-1],  # strip namespace
        'shape.left':             x,
        'shape.top':              y,
        'shape.width':            BUTTON['shape.width']  + width_offset,
        'shape.height':           BUTTON['shape.height'] + height_offset,
        'bgcolor.normal':         color,
        'bgcolor.hovered':        color,
        'bgcolor.clicked':        color,
        'action.targets':         [control],
        'panel':                  0,
        'shape.space':            'world',
        'shape.anchor':           'top_left',
        'shape.ignored_by_focus': False,
    })

    picker.document.add_shapes([btn])


# ─── MAIN HELPER ──────────────────────────────────────────────────────────────

def add_selected_controls_to_picker(camera_json_path, image_width, image_height,
                                    width_offset=0, height_offset=0):
    """
    For each selected Maya control, project its 3D position onto the 2D
    picker image and place a color-matched button at that position.

    Args:
        camera_json_path (str): Path to the exported camera JSON
        image_width      (int): Snapshot image width in pixels
        image_height     (int): Snapshot image height in pixels
        width_offset     (float): Optional extra button width
        height_offset    (float): Optional extra button height
    """
    import json
    import dwpicker

    picker = dwpicker.current()
    if picker is None:
        cmds.warning("No active picker found. Open a picker first.")
        return

    selection = cmds.ls(selection=True)
    if not selection:
        cmds.warning("Nothing selected.")
        return

    with open(camera_json_path, 'r') as f:
        camera_data = json.load(f)

    for control in selection:
        pos_2d = get_2d_position(control, camera_data, image_width, image_height)
        print(f"  {control}  ->  ({pos_2d[0]:.1f}, {pos_2d[1]:.1f})")
        place_button_on_picker(
            picker        = picker,
            control       = control,
            position_2d   = pos_2d,
            width_offset  = width_offset,
            height_offset = height_offset,
        )

    # Single UI refresh after all buttons added
    picker.document.changed.emit()
    print(f"Added {len(selection)} buttons to picker.")
    
add_selected_controls_to_picker(
    camera_json_path = r'G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\picker_files\CrowdB_camera.json',
    image_width      = 2048,
    image_height     = 2048,
)


import math
import json
import uuid
import os
import maya.cmds as cmds
import dwpicker
from dwpicker.templates import BUTTON


# ─── IMAGE DIMENSIONS ─────────────────────────────────────────────────────────

def _get_image_dimensions(image_path):
    """Read image dimensions from file using PySide6 or PySide2."""
    try:
        from PySide6.QtGui import QImage
        img = QImage(image_path)
        if not img.isNull():
            return img.width(), img.height()
    except ImportError:
        pass
    try:
        from PySide2.QtGui import QImage
        img = QImage(image_path)
        if not img.isNull():
            return img.width(), img.height()
    except ImportError:
        pass
    raise RuntimeError(f"Could not read image dimensions from: {image_path}")


# ─── MATH HELPERS ─────────────────────────────────────────────────────────────

def _deg_to_rad(deg):
    return deg * math.pi / 180.0


def _multiply_matrix_vector(m, v):
    return [
        m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
        m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
        m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2],
    ]


def _rotation_matrix_y(deg):
    r = _deg_to_rad(deg)
    c, s = math.cos(r), math.sin(r)
    return [[ c, 0, s], [ 0, 1, 0], [-s, 0, c]]


def _rotation_matrix_x(deg):
    r = _deg_to_rad(deg)
    c, s = math.cos(r), math.sin(r)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _multiply_matrices(a, b):
    result = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                result[i][j] += a[i][k] * b[k][j]
    return result


# ─── CORE PROJECTION ──────────────────────────────────────────────────────────

def get_2d_position(control, camera_data, image_width, image_height,
                    btn_width, btn_height):
    """
    Project a Maya control's world position onto 2D picker space.
    Returns position centered on the control (top-left corner for dwpicker).

    Args:
        control      (str):   Maya control node name
        camera_data  (dict):  Loaded camera JSON
        image_width  (int):   Snapshot image width in pixels
        image_height (int):   Snapshot image height in pixels
        btn_width    (float): Button width  (used to center the button on the point)
        btn_height   (float): Button height (used to center the button on the point)

    Returns:
        (float, float): (x, y) top-left corner position in picker pixel space
    """
    world_pos = cmds.xform(control, query=True, worldSpace=True, translation=True)
    pivot     = camera_data['group_translate']

    relative = [
        world_pos[0] - pivot[0],
        world_pos[1] - pivot[1],
        world_pos[2] - pivot[2],
    ]

    swivel_mat = _rotation_matrix_y(camera_data['swivel'])
    tilt_mat   = _rotation_matrix_x(camera_data['tilt'])
    view_mat   = _multiply_matrices(tilt_mat, swivel_mat)

    cam_space = _multiply_matrix_vector(view_mat, relative)

    ortho_width  = camera_data['orthographicWidth']
    ortho_height = ortho_width * (image_height / image_width)

    norm_x =  cam_space[0] / ortho_width
    norm_y = -cam_space[1] / ortho_height   # flip Y (image Y goes downward)

    pixel_x = (norm_x + 0.5) * image_width
    pixel_y = (norm_y + 0.5) * image_height

    # Offset by half button size so the button is centered on the control point
    centered_x = pixel_x - (btn_width  / 2.0)
    centered_y = pixel_y - (btn_height / 2.0)

    return centered_x, centered_y


# ─── CONTROL COLOR ────────────────────────────────────────────────────────────

def _get_control_color(control):
    """Get the wireframe override color of a Maya control as a hex string."""
    maya_index_colors = {
        0:  '#808080', 1:  '#000000', 2:  '#404040', 3:  '#808080',
        4:  '#800000', 5:  '#000040', 6:  '#000080', 7:  '#002020',
        8:  '#200020', 9:  '#800040', 10: '#804020', 11: '#404000',
        12: '#802000', 13: '#FF0000', 14: '#00FF00', 15: '#0040FF',
        16: '#FFFFFF', 17: '#FFFF00', 18: '#40C0FF', 19: '#40FF40',
        20: '#FF8080', 21: '#FF8040', 22: '#FFFF80', 23: '#008040',
        24: '#804040', 25: '#808040', 26: '#408040', 27: '#408080',
        28: '#4040FF', 29: '#FF40FF', 30: '#FF8040', 31: '#FF4040',
    }

    shapes = cmds.listRelatives(control, shapes=True) or []
    node   = shapes[0] if shapes else control

    try:
        if cmds.getAttr(node + '.overrideRGBColors'):
            r = cmds.getAttr(node + '.overrideColorR')
            g = cmds.getAttr(node + '.overrideColorG')
            b = cmds.getAttr(node + '.overrideColorB')
            return '#{:02X}{:02X}{:02X}'.format(
                int(r * 255), int(g * 255), int(b * 255)
            )
    except Exception:
        pass

    try:
        if cmds.getAttr(node + '.overrideEnabled'):
            index = cmds.getAttr(node + '.overrideColor')
            return maya_index_colors.get(index, '#888888')
    except Exception:
        pass

    return '#888888'


# ─── BUTTON PLACEMENT ─────────────────────────────────────────────────────────

def place_button_on_picker(picker, control, position_2d,
                            color=None,
                            shape='square',
                            btn_width=120.0,
                            btn_height=25.0):
    """
    Add a button to the picker at the given 2D position.

    Args:
        picker       : Active dwpicker instance from dwpicker.current()
        control (str): Maya control node name
        position_2d  : (x, y) top-left corner in picker pixel space
        color   (str): Hex color string e.g. '#4080FF'. 
                       Defaults to None (uses control wireframe color)
        shape   (str): 'square' or 'round'. Defaults to 'square'
        btn_width (float):  Button width in pixels.  Defaults to 120.0
        btn_height (float): Button height in pixels. Defaults to 25.0
    """
    resolved_color = color if color else _get_control_color(control)
    x, y = position_2d

    btn = BUTTON.copy()
    btn.update({
        'id':                     str(uuid.uuid4()),
        'text.content':           control.split(':')[-1],   # strip namespace
        'shape.left':             x,
        'shape.top':              y,
        'shape.width':            float(btn_width),
        'shape.height':           float(btn_height),
        'shape':                  shape,                    # 'square' or 'round'
        'bgcolor.normal':         resolved_color,
        'bgcolor.hovered':        resolved_color,
        'bgcolor.clicked':        resolved_color,
        'action.targets':         [control],
        'panel':                  0,
        'shape.space':            'world',
        'shape.anchor':           'top_left',
        'shape.ignored_by_focus': False,
    })

    picker.document.add_shapes([btn])


# ─── MAIN HELPER ──────────────────────────────────────────────────────────────

def add_selected_controls_to_picker(camera_json_path, image_path,
                                     color=None,
                                     shape='square',
                                     btn_width=120.0,
                                     btn_height=25.0):
    """
    For each selected Maya control, project its 3D position onto the 2D
    picker image and place a button at that position.

    Args:
        camera_json_path (str):   Path to the exported camera JSON
        image_path       (str):   Path to the snapshot image (dimensions read automatically)
        color      (str):         Hex color e.g. '#4080FF'. None = use control color.
                                  Defaults to '#4080FF' (blue)
        shape      (str):         'square' or 'round'. Defaults to 'square'
        btn_width  (float):       Button width in pixels.  Defaults to 120.0
        btn_height (float):       Button height in pixels. Defaults to 25.0
    """
    # Default color is blue
    resolved_color = color if color else '#4080FF'

    picker = dwpicker.current()
    if picker is None:
        cmds.warning("No active picker found. Open a picker first.")
        return

    selection = cmds.ls(selection=True)
    if not selection:
        cmds.warning("Nothing selected.")
        return

    # Read image dimensions from the actual file
    image_width, image_height = _get_image_dimensions(image_path)
    print(f"Image dimensions: {image_width} x {image_height}px")

    with open(camera_json_path, 'r') as f:
        camera_data = json.load(f)

    for control in selection:
        pos_2d = get_2d_position(
            control, camera_data,
            image_width, image_height,
            btn_width, btn_height
        )
        print(f"  {control}  ->  ({pos_2d[0]:.1f}, {pos_2d[1]:.1f})")
        place_button_on_picker(
            picker     = picker,
            control    = control,
            position_2d= pos_2d,
            color      = resolved_color,
            shape      = shape,
            btn_width  = btn_width,
            btn_height = btn_height,
        )

    picker.document.changed.emit()
    print(f"Added {len(selection)} buttons to picker.")
add_selected_controls_to_picker(
    camera_json_path = r'G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\picker_files\CrowdB_camera.json',
    image_path       = r'G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\picker_files\CrowdB.0.png',

)


def add_group_button_to_picker(controls,
                                position,
                                color='#4080FF',
                                shape='square',
                                btn_width=120.0,
                                btn_height=25.0,
                                label=None):
    """
    Add a single button to the picker that selects multiple controls at once.

    Args:
        controls   (list):  List of Maya control node names to select on click
        position   (tuple): Manual (x, y) position in picker pixel space
        color      (str):   Hex color string e.g. '#4080FF'. Defaults to blue
        shape      (str):   'square' or 'round'. Defaults to 'square'
        btn_width  (float): Button width in pixels.  Defaults to 120.0
        btn_height (float): Button height in pixels. Defaults to 25.0
        label      (str):   Button label. Defaults to None (uses first control name)
    """
    import uuid
    import dwpicker
    from dwpicker.templates import BUTTON

    picker = dwpicker.current()
    if picker is None:
        cmds.warning("No active picker found. Open a picker first.")
        return

    if not controls:
        cmds.warning("No controls provided.")
        return

    x, y = position

    # Default label to first control name (namespace stripped) if not provided
    resolved_label = label if label else controls[0].split(':')[-1]

    btn = BUTTON.copy()
    btn.update({
        'id':                     str(uuid.uuid4()),
        'text.content':           resolved_label,
        'shape.left':             float(x),
        'shape.top':              float(y),
        'shape.width':            float(btn_width),
        'shape.height':           float(btn_height),
        'shape':                  shape,
        'bgcolor.normal':         color,
        'bgcolor.hovered':        color,
        'bgcolor.clicked':        color,
        'action.targets':         list(controls),   # all controls selected on click
        'panel':                  0,
        'shape.space':            'world',
        'shape.anchor':           'top_left',
        'shape.ignored_by_focus': False,
    })

    picker.document.add_shapes([btn])
    picker.document.changed.emit()

    print(f"Added group button '{resolved_label}' -> {controls}")
    


# Fully custom
add_group_button_to_picker(
    controls   = ['l_arm_ctrl', 'l_elbow_ctrl', 'l_wrist_ctrl'],
    position   = (200, 350),
    color      = '#FF4040',
    shape      = 'round',
    btn_width  = 60.0,
    btn_height = 60.0,
    label      = 'L Arm',
)