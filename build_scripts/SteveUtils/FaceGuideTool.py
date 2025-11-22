# ===============================================
#   FACE GUIDE UI — with Overwrite + Side Logic
#   + Auto-increment 00 → 01 → 02 → 03 ...
# ===============================================

from Qt import QtWidgets, QtCore, QtCompat
import maya.OpenMayaUI as omui
import maya.cmds as mc
import re
import maya.api.OpenMaya as om
import math
import random


# -------------------------------
#  Your FACE_PARTS list
# -------------------------------
side = "*"
FACE_PARTS = {
    "Eye": [f"Eye_{side}_EyeCenterPivot", f"Eye_{side}_Socket_OuterUpper01", f"Eye_{side}_Eyelid_InnerUpper01",
            f"Eye_{side}_Eyelid_OuterLower01", f"Eye_{side}_Eyelid_Upper", f"Eye_{side}_Socket_OuterCorner",
            f"Eye_{side}_Socket_Lower", f"Eye_{side}_Socket_InnerCorner", f"Eye_{side}_Eyelid_OuterUpper01",
            f"Eye_{side}_Eyelid_OuterCorner", f"Eye_{side}_Socket_Upper", f"Eye_{side}_Aim",
            f"Eye_{side}_Socket_InnerLower01", f"Eye_{side}_Eyelid_Lower", f"Eye_{side}_Eyelid_InnerCorner",
            f"Eye_{side}_Eyelid_InnerLower01", f"Eye_{side}_Socket_OuterLower01", f"Eye_{side}_Socket_InnerUpper01",
            f"Eye_{side}_Pupil_00", f"Eye_{side}_Iris_00"],

    "Brow": [f'Brow_{side}_01', f'Brow_{side}_02', f'Brow_{side}_03', f'Brow_{side}_04', f'Brow_{side}_05',
             f'Brow_{side}_01_Upper', f'Brow_{side}_02_Upper', f'Brow_{side}_03_Upper', f'Brow_{side}_04_Upper',
             f'Brow_{side}_05_Upper', f'Brow_{side}_01_Lower', f'Brow_{side}_02_Lower', f'Brow_{side}_03_Lower',
             f'Brow_{side}_04_Lower', f'Brow_{side}_05_Lower', f'Brow_{side}_01_Major', f'Brow_{side}_02_Major'],

    "Mouth": ['Mouth_M_UpperLip_01', 'Mouth_M_center', f'Mouth_{side}_CornerLip',
              'Mouth_M_LowerLip_01', f'Mouth_{side}_UpperLip_02', f'Mouth_{side}_LowerLip_02',
              f'Mouth_{side}_UpperLip_03', f'Mouth_{side}_LowerLip_03',
              f'Mouth_{side}_UpperLip_04', f'Mouth_{side}_LowerLip_04',
              f'Mouth_{side}_UpperLip_05', f'Mouth_{side}_LowerLip_05',],

    "Nose": ["NoseTip", "Nostril"],
    "Ear": ["EarBase", "EarTip"],
    "NL": ["Nasolabial_01", "Nasolabial_02"],
    "Cheek": [f"Cheek_{side}_CheekBone", f"Cheek_{side}_Puff"],
    "Jaw": ['Jaw_M_ee', 'Jaw_M_root', 'Jaw_M_larynx'],
    "Tongue": ["Tongue_00", 'topTeeth', 'botTeeth', 'topTeeth_M_Sub_01', 'botTeeth_M_Sub_01',
               f'botTeeth_{side}_Sub_03', f'botTeeth_{side}_Sub_02', f'topTeeth_{side}_Sub_02', f'topTeeth_{side}_Sub_03'],
    "Head": ["UpperHead_guide", "UpperHead_guide"]
}


# -------------------------------
#   Maya window parenting helper
# -------------------------------
def get_maya_window():
    ptr = omui.MQtUtil.mainWindow()
    return QtCompat.wrapInstance(int(ptr), QtWidgets.QWidget)


# ==========================================
#   MAIN UI
# ==========================================
class FaceGuideUI(QtWidgets.QDialog):

    def __init__(self, parent=get_maya_window()):
        super(FaceGuideUI, self).__init__(parent)

        self.setWindowTitle("Face Guide Placer")
        self.setMinimumWidth(350)

        self.layout = QtWidgets.QVBoxLayout(self)

        # ---- Dropdown ----
        self.part_dropdown = QtWidgets.QComboBox()
        self.part_dropdown.addItems(FACE_PARTS.keys())
        self.part_dropdown.currentTextChanged.connect(self.build_rows)
        self.layout.addWidget(self.part_dropdown)

        # ---- Dynamic rows container ----
        self.row_container = QtWidgets.QWidget()
        self.row_layout = QtWidgets.QVBoxLayout(self.row_container)
        self.row_layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.row_container)

        self.build_rows(self.part_dropdown.currentText())

    # ==========================================
    #   Create rows dynamically
    # ==========================================
    def build_rows(self, part_name):

        # clear rows
        while self.row_layout.count():
            itm = self.row_layout.takeAt(0)
            if itm.widget():
                itm.widget().deleteLater()

        for item_name in FACE_PARTS.get(part_name, []):
            row = self.make_button_row(item_name)
            self.row_layout.addWidget(row)

        self.row_layout.addStretch()

    # ==========================================
    #   Single row UI
    # ==========================================
    def make_button_row(self, label):
        row = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)

        # Label
        layout.addWidget(QtWidgets.QLabel(label))

        # Overwrite checkbox
        overwrite = QtWidgets.QCheckBox("O")
        overwrite.setToolTip("Overwrite existing guide")
        layout.addWidget(overwrite)

        # Detect if this uses a side
        has_side = "*" in label

        # Buttons
        for side_label in ["L", "R", "M", "LR"]:
            btn = QtWidgets.QPushButton(side_label)
            btn.setFixedWidth(35)

            # Callback
            #btn.clicked.connect(
            #    lambda checked=False, n=label, s=side_label, ow=overwrite:
            #    self.button_pressed(n, s, ow, pressed_side)
            #)
            btn.clicked.connect(lambda checked=False, n=label, s=side_label, ps=side_label, ow=overwrite:
                                self.button_pressed(n, s, ow, ps))

            # ---- enable/disable logic ----
            if has_side:
                if side_label == "M":
                    btn.setEnabled(False)
            else:
                if side_label in ["L", "R", "LR"]:
                    btn.setEnabled(False)

            layout.addWidget(btn)

        return row

    # ==========================================
    #   BUTTON LOGIC
    # ==========================================
    def button_pressed(self, base_name, side, overwrite_checkbox, pressed_side):

        overwrite = overwrite_checkbox.isChecked()

        # Single side
        self.process_side(base_name, side, overwrite, pressed_side)

    # ==========================================
    #   Auto-increment numeric suffix
    # ==========================================
    def next_available_index(self, base_name):

        m = re.search(r"(.*_)(\d\d)$", base_name)
        if not m:
            return base_name  # no numeric suffix

        prefix = m.group(1)
        start_index = int(m.group(2))

        current = start_index
        while True:
            test_name = f"{prefix}{current:02d}"
            if not mc.objExists(test_name):
                return test_name
            current += 1

    # ==========================================
    #   Process a single side
    # ==========================================
    def process_side(self, base_name, side, overwrite, pressed_side):

        # Replace * with L or R
        if side in ["L", "R"]:
            name = base_name.replace("*", side)
        else:
            name = base_name  # M already includes M

        print(f"\n[PROCESS] Starting Name: {name}")

        if overwrite:
            print(" → Overwrite ON: skipping existence check.")
            self.place_at_vert(overwrite=overwrite, side=side, base_name=base_name)
            return True

        # Auto-increment if needed
        #final_name = self.next_available_index(name)

        #if final_name != name:
        #    print(f" → Name '{name}' exists, using '{final_name}'")
        #else:
        #    print(f" → Using: {final_name}")

        # Final safety check
        if side == 'LR':
            for side in ['L', 'R']:
                name = base_name.replace("*", side)
                if mc.objExists(name):
                    print(" → ERROR: Guide already exists even after increment!")
                    return False
                else:
                    print(f" → Guide ready to create: {name}")

        else:
            if mc.objExists(name):
                print(" → ERROR: Guide already exists even after increment!")
                return False

            print(f" → Guide ready to create: {name}")
        self.place_at_vert(overwrite=overwrite, side=side, base_name=base_name)
        return True

    # ==========================================
    #   Place at vert
    # ==========================================
    def place_at_vert(self, overwrite=True, side='L', base_name=None, object_type='locator'):
        # --- Compute final name ---
        if side in ["L", "R"]:
            name = base_name.replace("*", side)
            resolved_name = base_name.replace("*", side)
        else:
            name = base_name  # M already includes M
            resolved_name = base_name

        sel = mc.ls(sl=True, fl=True)
        verts = mc.filterExpand(sel, sm=31)  # vertex components
        print(name)



        print(name)
        if side == "LR":
            if not verts:
                mc.warning(f"No vertex selected for {base_name} (LR).")
                return None

            if len(verts) > 1:
                mc.warning(f"Multiple vertices selected for {base_name} (LR). Picking the first one.")
            vtx = verts[0]
            pos = mc.xform(vtx, q=True, ws=True, t=True)

            # Determine which side the vertex is on based on X
            if pos[0] >= 0:
                primary_side, mirror_side = "L", "R"
            else:
                primary_side, mirror_side = "R", "L"

            # Place primary guide
            primary_name = base_name.replace("*", primary_side)
            self._create_object_at_pos(primary_name, pos,)
            print(f"Placed {primary_name} at {pos}")

            # Place mirrored guide
            mirror_name = base_name.replace("*", mirror_side)
            mirror_pos = [-pos[0], pos[1], pos[2]]
            self._create_object_at_pos(mirror_name, mirror_pos,)
            print(f"Placed {mirror_name} mirrored at {mirror_pos}")

            return True



        elif name in ['Jaw_M_Root', "Eye_L_EyeCenterPivot", "Eye_R_EyeCenterPivot", 'botTeeth', 'topTeeth', 'Mouth_M_center']:
            if not verts:
                mc.warning("Please select at least one vertex for Jaw_M_guide.")
                return None

            # Compute average position
            total_pos = [0.0, 0.0, 0.0]
            for vtx in verts:
                pos = mc.xform(vtx, q=True, ws=True, t=True)
                total_pos[0] += pos[0]
                total_pos[1] += pos[1]
                total_pos[2] += pos[2]
            num_verts = len(verts)
            center_pos = [c / num_verts for c in total_pos]

            # Create object
            if object_type == 'locator':
                obj = mc.spaceLocator(name=name)[0]
            elif object_type == 'joint':
                obj = mc.joint(name=name)
            else:
                obj = mc.createNode(object_type, name=name)

            mc.xform(obj, ws=True, t=center_pos)
            return obj
        


        elif base_name in ["Eye_L_Aim", 'Eye_R_Aim']:
            # Determine pivot name
            pivot_name = f"Eye_{side}_EyeCenterPivot"
            if not mc.objExists(pivot_name):
                mc.warning(f"Pivot not found: {pivot_name}")
                return None

            # Query pivot world position
            pos = mc.xform(pivot_name, q=True, ws=True, t=True)

            # Add +20 to Z
            pos[2] += 20

            # Create object
            if object_type == 'locator':
                obj = mc.spaceLocator(name=name)[0]
            elif object_type == 'joint':
                obj = mc.joint(name=name)
            else:
                obj = mc.createNode(object_type, name=name)

            # Move to position
            mc.xform(obj, ws=True, t=pos)
            print(f"Placed {name} at Eye Aim position: {pos}")
            return obj


        elif ("Pupil_00" in resolved_name) or ("Iris_00" in resolved_name) or ("Tongue_00" in resolved_name):

            print("[MULTI-VERT] Triggered for:", resolved_name)

            # ordered vertex list (preserves user click order)
            verts = [v for v in mc.ls(sl=True, fl=True) if ".vtx[" in v]

            if not verts:
                mc.error("Please select one or more vertices for pupil/iris placement.")

            # Build name prefix (e.g. Eye_L_Pupil_)
            prefix = resolved_name.replace("00", "")

            # ------------------------------------------------------------
            # Find existing numbered guides and determine next index
            # ------------------------------------------------------------
            existing = mc.ls(prefix + "*", type="transform") or []

            numbers = []
            for e in existing:
                try:
                    suffix = e.split("_")[-1]
                    if suffix.isdigit():
                        numbers.append(int(suffix))
                except:
                    pass

            next_index = (max(numbers) + 1) if numbers else 1

            created = []

            # ------------------------------------------------------------
            # Create one guide per vert IN SELECTION ORDER
            # ------------------------------------------------------------
            for i, vtx in enumerate(verts):
                pos = mc.pointPosition(vtx, world=True)
                num = next_index + i
                new_name = f"{prefix}{num:02d}"

                print(f"[MULTI-VERT] Creating: {new_name} at {pos}")

                guide = self._create_object_at_pos(new_name, pos)

                # Add scale_mult attr
                if not mc.objExists(f"{guide}.scale_mult"):
                    mc.addAttr(guide, ln="scale_mult", at="double", dv=1.0, keyable=True)

                created.append(guide)

            print("[MULTI-VERT] Created:", created)
            return created

        else:
              # For all other guides, must have exactly one vertex
            if not verts:
                mc.warning(f"No vertex selected for {base_name}.")
                return None
            if len(verts) > 1:
                mc.warning(f"Multiple vertices selected for {base_name}. Picking the first one.")

            vtx = verts[0]
            pos = mc.xform(vtx, q=True, ws=True, t=True)

            if object_type == 'locator':
                obj = mc.spaceLocator(name=name)[0]
            elif object_type == 'joint':
                obj = mc.joint(name=name)
            else:
                obj = mc.createNode(object_type, name=name)

            mc.xform(obj, ws=True, t=pos)
            return obj

    
    def _create_object_at_pos(self, name, pos):
        """Creates the guide object (locator/joint/etc) at a given world position."""
        obj_type = 'locator'  # <-- you can promote this to a UI option later
        if obj_type == 'locator':
            obj = mc.spaceLocator(n=name)[0]
        else:
            obj = mc.joint(n=name)

        mc.xform(obj, ws=True, t=pos)
        return obj





# ==========================================
#   Launch Window
# ==========================================
def open_face_guide_ui():
    global _faceGuideUI
    try:
        _faceGuideUI.close()
    except:
        pass

    _faceGuideUI = FaceGuideUI()
    _faceGuideUI.show()


open_face_guide_ui()
