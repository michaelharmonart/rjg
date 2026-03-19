from __future__ import annotations
import json
import os
import maya.cmds as mc
import maya.api.OpenMaya as om
import platform

import rjg.build.guides.guide_read_tool as gr
import rjg.build_scripts.basemesh_buildhelper as sb
import rjg.build_scripts.SteveUtils.guide_flip as rib
import rjg.build_scripts.SteveUtils.control_filp_helper as cflip
import rjg.post.dataIO.controls as rCtrlIO
import rjg.libs.control.draw as draw
import rjg.build_scripts.bettercontrols as c
import rjg.post.character_defaults as char_def

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
        self.load_all_btn = QtWidgets.QPushButton("Load ALL File")

        char_layout.addWidget(self.char_dropdown)
        char_layout.addWidget(self.init_char_button)
        char_layout.addWidget(self.load_all_btn)

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


        # -----------------------
        # UTILS Section
        # -----------------------

        utils_section = CollapsibleSection("UTILS")

        self.flip_guides_btn = QtWidgets.QPushButton("Flip Guides")
        self.save_controls_btn = QtWidgets.QPushButton("Save Out Controls")
        self.save_defaults_btn = QtWidgets.QPushButton("Save Control Defaults")

        utils_section.content_layout.addWidget(self.flip_guides_btn)
        utils_section.content_layout.addWidget(self.save_controls_btn)
        utils_section.content_layout.addWidget(self.save_defaults_btn)

        main_layout.addWidget(utils_section)

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

        self.flip_guides_btn.clicked.connect(self.flip_guides)
        self.save_controls_btn.clicked.connect(self.save_out_controls)
        self.save_defaults_btn.clicked.connect(self.save_control_defaults)
        self.load_all_btn.clicked.connect(self.load_all_file)

    # ------------------------------------------------
    # Class Functions
    # ------------------------------------------------

    def load_all_file(self):
        """
        Warn user, then open the character ALL file.
        """

        character = self.char_dropdown.currentText().strip()
        rig_root = f"{groups}/bobo/character/Rigs"

        # ----------------------------
        # Confirmation Popup
        # ----------------------------

        result = QtWidgets.QMessageBox.warning(
            self,
            "Load ALL File",
            "Warning: The current scene will NOT be saved.\n\nDo you want to proceed?",
            QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel,
            QtWidgets.QMessageBox.Cancel
        )

        if result != QtWidgets.QMessageBox.Ok:
            print("Load ALL cancelled.")
            return

        # ----------------------------
        # Build File Path
        # ----------------------------

        all_file = os.path.join(
            rig_root,
            character,
            f"{character}_ALL.mb"
        )

        if not os.path.exists(all_file):
            mc.warning(f"ALL file not found: {all_file}")
            return

        print(f"Opening: {all_file}")

        # ----------------------------
        # Open File
        # ----------------------------

        mc.file(all_file, open=True, force=True)

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
    # UTILS - Dummy Functions
    # ------------------------------------------------

    def flip_guides(self):
        rib.launch_ui()

    def save_out_controls(self):
        character = self.char_dropdown.currentText().strip()
        controls = ["COG_M_CTRL", "global_M_CTRL", "foot_L_01_L_CTRL", "foot_R_01_R_CTRL", "hand_L_01_CTRL", "hand_R_01_CTRL", "RJG_M_CTRL"]
        cflip.flip_controls(controls, flip=True)
        rCtrlIO.write_ctrls(f"{groups}/bobo/character/Rigs/{character}/Controls", force=True, name=f'{character}_control_curves')
        cflip.flip_controls(controls, flip=False)
        c.write_control_shapes(f"{groups}/bobo/character/Rigs/{character}/Controls/controls.json")


    def save_control_defaults(self):
        char_def.run()

    # ------------------------------------------------a
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