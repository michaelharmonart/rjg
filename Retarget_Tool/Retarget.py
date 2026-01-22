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

VALID_TYPES = ["FK", "IK", "FK_Distribute", "FK_IK", "Hybrid", "Root"]
CONSTRAINT_SET = "retarget_constraint_set"
FKIK_OPTIONS = ["FK", "IK"]

# ------------------------------------------------------------
# UTILS
# ------------------------------------------------------------

def maya_main_window():
    ptr = omui.MQtUtil.mainWindow()
    return wrapInstance(int(ptr), QtWidgets.QWidget)

def list_defs():
    if not os.path.exists(CHAR_LIB_PATH):
        return []
    return sorted(
        os.path.splitext(f)[0]
        for f in os.listdir(CHAR_LIB_PATH)
        if f.endswith(".json")
    )

def load_def(name):
    path = os.path.join(CHAR_LIB_PATH, f"{name}.json")
    if not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        return json.load(f)

# ------------------------------------------------------------
# UI
# ------------------------------------------------------------

class RetargetToolUI(QtWidgets.QDialog):

    def __init__(self, parent=maya_main_window()):
        super().__init__(parent)
        self.setWindowTitle("Retarget Tool")
        self.setMinimumWidth(420)
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.Window)

        self.source_data = {}
        self.target_data = {}
        self.shared_parts = []

        self.build_ui()

    # --------------------------------------------------------

    def build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # ---------- Source ----------
        layout.addWidget(QtWidgets.QLabel("Source Def"))
        self.source_combo = QtWidgets.QComboBox()
        self.source_combo.addItems(list_defs())
        layout.addWidget(self.source_combo)

        self.source_ns = QtWidgets.QLineEdit()
        self.source_ns.setPlaceholderText("Source Namespace")
        layout.addWidget(self.source_ns)

        # ---------- Target ----------
        layout.addWidget(QtWidgets.QLabel("Target Def"))
        self.target_combo = QtWidgets.QComboBox()
        self.target_combo.addItems(list_defs())
        layout.addWidget(self.target_combo)

        self.target_ns = QtWidgets.QLineEdit()
        self.target_ns.setPlaceholderText("Target Namespace")
        layout.addWidget(self.target_ns)

        # ---------- FK / IK Overrides ----------
        layout.addWidget(QtWidgets.QLabel("FK / IK Mode Overrides"))

        fkik_row = QtWidgets.QHBoxLayout()

        self.arm_fkik = QtWidgets.QComboBox()
        self.arm_fkik.addItems(FKIK_OPTIONS)
        fkik_row.addWidget(QtWidgets.QLabel("Arm"))
        fkik_row.addWidget(self.arm_fkik)

        self.leg_fkik = QtWidgets.QComboBox()
        self.leg_fkik.addItems(FKIK_OPTIONS)
        fkik_row.addWidget(QtWidgets.QLabel("Leg"))
        fkik_row.addWidget(self.leg_fkik)

        self.other_fkik = QtWidgets.QComboBox()
        self.other_fkik.addItems(FKIK_OPTIONS)
        fkik_row.addWidget(QtWidgets.QLabel("Other"))
        fkik_row.addWidget(self.other_fkik)

        layout.addLayout(fkik_row)

        # ---------- Compatibility ----------
        self.compat_btn = QtWidgets.QPushButton("Check Compatibility")
        self.compat_btn.setStyleSheet("background-color: #555;")
        self.compat_btn.clicked.connect(self.check_compatibility)
        layout.addWidget(self.compat_btn)

        # ---------- Actions ----------
        self.constrain_btn = QtWidgets.QPushButton("Constrain")
        self.constrain_btn.clicked.connect(self.constrain)
        layout.addWidget(self.constrain_btn)

        self.bake_btn = QtWidgets.QPushButton("Bake Retarget")
        self.bake_btn.clicked.connect(self.bake_retarget)
        layout.addWidget(self.bake_btn)

    # --------------------------------------------------------

    def check_compatibility(self):
        src = self.source_combo.currentText()
        tgt = self.target_combo.currentText()

        self.source_data = load_def(src)
        self.target_data = load_def(tgt)

        src_parts = set(self.source_data.keys())
        tgt_parts = set(self.target_data.keys())

        self.shared_parts = sorted(src_parts & tgt_parts)

        missing_src = tgt_parts - src_parts
        missing_tgt = src_parts - tgt_parts

        if not self.shared_parts:
            self.compat_btn.setStyleSheet("background-color: #aa3333;")
            print("[Compat] No compatible parts found")
        elif src_parts == tgt_parts:
            self.compat_btn.setStyleSheet("background-color: #339933;")
            print("[Compat] Fully compatible")
        else:
            self.compat_btn.setStyleSheet("background-color: #aaaa33;")
            print("[Compat] Partial compatibility")

        print("Shared parts:", self.shared_parts)
        if missing_src:
            print("Missing on Source:", list(missing_src))
        if missing_tgt:
            print("Missing on Target:", list(missing_tgt))

    # --------------------------------------------------------

    def constrain_fk(self, part):
        """
        Constrain a single FK part.
        Adds orient constraints for all subparts.
        """
        src_part = self.source_data[part]
        tgt_part = self.target_data[part]

        if src_part.get("Type") != "FK":
            mc.warning(f"[FK Constrain] {part} is not FK")
            return False

        if not mc.objExists(CONSTRAINT_SET):
            mc.sets(name=CONSTRAINT_SET, empty=True)

        src_ns = self.source_ns.text()
        tgt_ns = self.target_ns.text()

        failed = []
        created = []

        src_controls = src_part.get("ControlList", {})
        tgt_controls = tgt_part.get("ControlList", {})

        for subpart, src_list in src_controls.items():
            tgt_list = tgt_controls.get(subpart, [])
            for src_ctrl, tgt_ctrl in zip(src_list, tgt_list):
                src_node = f"{src_ns}:{src_ctrl}" if src_ns else src_ctrl
                tgt_node = f"{tgt_ns}:{tgt_ctrl}" if tgt_ns else tgt_ctrl

                if not mc.objExists(src_node) or not mc.objExists(tgt_node):
                    mc.warning(f"[FK Constrain] Missing node: {src_node} or {tgt_node}")
                    failed.append(part)
                    continue

                con = mc.orientConstraint(src_node, tgt_node, mo=src_part.get("MO", True))[0]
                mc.sets(con, add=CONSTRAINT_SET)
                created.append(con)

        return len(failed) == 0

    def constrain_root(self, part):
        """
        Constrain a single Root part.
        Adds both orient and point constraints for all subparts.
        """
        src_part = self.source_data[part]
        tgt_part = self.target_data[part]

        if not mc.objExists(CONSTRAINT_SET):
            mc.sets(name=CONSTRAINT_SET, empty=True)

        src_ns = self.source_ns.text()
        tgt_ns = self.target_ns.text()

        failed = []
        created = []

        src_controls = src_part.get("ControlList", {})
        tgt_controls = tgt_part.get("ControlList", {})

        for subpart, src_list in src_controls.items():
            tgt_list = tgt_controls.get(subpart, [])
            for src_ctrl, tgt_ctrl in zip(src_list, tgt_list):
                src_node = f"{src_ns}:{src_ctrl}" if src_ns else src_ctrl
                tgt_node = f"{tgt_ns}:{tgt_ctrl}" if tgt_ns else tgt_ctrl

                if not mc.objExists(src_node) or not mc.objExists(tgt_node):
                    mc.warning(f"[Root Constrain] Missing node: {src_node} or {tgt_node}")
                    failed.append(part)
                    continue

                # Orient + Point constraints
                con = mc.orientConstraint(src_node, tgt_node, mo=src_part.get("MO", True))[0]
                mc.sets(con, add=CONSTRAINT_SET)
                con = mc.pointConstraint(src_node, tgt_node, mo=src_part.get("MO", True))[0]
                mc.sets(con, add=CONSTRAINT_SET)
                created.append(con)

        return len(failed) == 0

    def constrain_ik(self, part):
        """
        Constrain a single IK part (arm or leg).
        Parent/point/orient constraints depending on type.
        """
        src_part = self.source_data[part]
        tgt_part = self.target_data[part]

        src_ns = self.source_ns.text()
        tgt_ns = self.target_ns.text()

        if not mc.objExists(CONSTRAINT_SET):
            mc.sets(name=CONSTRAINT_SET, empty=True)

        failed = []
        created = []

        # -------------------------
        # Editable arm/leg lists
        # -------------------------
        ARM_PARTS = ["arm"]   # You can expand this later
        LEG_PARTS = ["leg"]   # You can expand this later

        part_lower = part.lower()

        if part_lower in ARM_PARTS:
            # -------------------------
            # ARM IK LOGIC
            # -------------------------
            src_controls = src_part.get("ControlList", {})
            tgt_controls = tgt_part.get("ControlList", {})

            # Expected subparts: Shoulder, Elbow, Wrist / Hand
            try:
                src_fk_hand = src_controls["FkWrist"][0]
                src_fk_elbow = src_controls["FkElbow"][0]

                tgt_ik_hand = tgt_controls.get("IKHand", [None])[0]
                tgt_ik_pv = tgt_controls.get("IKPV", [None])[0]

                if not src_fk_hand or not tgt_ik_hand or not src_fk_elbow or not tgt_ik_pv:
                    mc.warning(f"[IK Constrain] Missing arm controls for part '{part}'")
                    return False

                # ParentConstraint FK Hand -> IK Hand
                pc = mc.parentConstraint(f"{src_ns}:{src_fk_hand}" if src_ns else src_fk_hand,
                                        f"{tgt_ns}:{tgt_ik_hand}" if tgt_ns else tgt_ik_hand,
                                        mo=True)[0]
                mc.sets(pc, add=CONSTRAINT_SET)
                created.append(pc)

                # PointConstraint FK Elbow -> IK PV
                pt = mc.pointConstraint(f"{src_ns}:{src_fk_elbow}" if src_ns else src_fk_elbow,
                                        f"{tgt_ns}:{tgt_ik_pv}" if tgt_ns else tgt_ik_pv,
                                        mo=True)[0]
                mc.sets(pt, add=CONSTRAINT_SET)
                created.append(pt)

            except KeyError as e:
                mc.warning(f"[IK Constrain] Missing subpart for arm: {e}")
                failed.append(part)
                return False

        elif part_lower in LEG_PARTS:
            # -------------------------
            # LEG IK LOGIC
            # -------------------------
            src_controls = src_part.get("ControlList", {})
            tgt_controls = tgt_part.get("ControlList", {})

            # Expected subparts: Hip, Knee, Ankle, Toes
            try:
                src_fk_ankle = src_controls["FkAnkle"][0]
                src_fk_knee = src_controls["FKKnee"][0]
                src_fk_toes = src_controls["FKToe"][0]

                tgt_ik_foot = tgt_controls.get("IKFoot", [None])[0]
                tgt_ik_pv = tgt_controls.get("IKPV", [None])[0]
                tgt_ik_toes = tgt_controls.get("IKToe", [None])[0]

                if not all([src_fk_ankle, src_fk_knee, src_fk_toes, tgt_ik_foot, tgt_ik_pv, tgt_ik_toes]):
                    mc.warning(f"[IK Constrain] Missing leg controls for part '{part}'")
                    return False

                # ParentConstraint FK Ankle -> IK Foot
                pc = mc.parentConstraint(f"{src_ns}:{src_fk_ankle}" if src_ns else src_fk_ankle,
                                        f"{tgt_ns}:{tgt_ik_foot}" if tgt_ns else tgt_ik_foot,
                                        mo=True)[0]
                mc.sets(pc, add=CONSTRAINT_SET)
                created.append(pc)

                # PointConstraint FK Knee -> IK PV
                pt = mc.pointConstraint(f"{src_ns}:{src_fk_knee}" if src_ns else src_fk_knee,
                                        f"{tgt_ns}:{tgt_ik_pv}" if tgt_ns else tgt_ik_pv,
                                        mo=True)[0]
                mc.sets(pt, add=CONSTRAINT_SET)
                created.append(pt)

                # OrientConstraint FK Toes -> IK Toes
                oc = mc.orientConstraint(f"{src_ns}:{src_fk_toes}" if src_ns else src_fk_toes,
                                        f"{tgt_ns}:{tgt_ik_toes}" if tgt_ns else tgt_ik_toes,
                                        mo=True)[0]
                mc.sets(oc, add=CONSTRAINT_SET)
                created.append(oc)

            except KeyError as e:
                mc.warning(f"[IK Constrain] Missing subpart for leg: {e}")
                failed.append(part)
                return False

        else:
            mc.warning(f"[IK Constrain] Logic not written for part '{part}', try FK instead.")
            failed.append(part)
            return False

        return len(failed) == 0

    def constrain(self):
        if not self.shared_parts:
            mc.warning("No compatible parts to constrain")
            return

        if not mc.objExists(CONSTRAINT_SET):
            mc.sets(name=CONSTRAINT_SET, empty=True)

        failed = []


        for part in self.shared_parts:
            
            src_part = self.source_data[part]
            tgt_part = self.target_data[part]
            part_type = tgt_part.get("Type")

            if part_type not in VALID_TYPES:
                mc.warning(f"[Constrain] Invalid type '{part_type}' on {part}")
                failed.append(part)
                continue

            # Call the single-part constrain functions
            if part_type == "FK":
                success = self.constrain_fk(part)
                if not success:
                    failed.append(part)
            elif part_type == "IK":
                success = self.constrain_ik(part)
                if not success:
                    failed.append(part)
            elif part_type == "FK_Distribute":
                pass
            elif part_type == "FK_IK":
                arm_mode = self.arm_fkik.currentText()
                leg_mode = self.leg_fkik.currentText()
                other_mode = self.other_fkik.currentText()
                mode_map = {
                    "arm": arm_mode,
                    "leg": leg_mode
                }
                part_lower = part.lower()
                # Determine mode: arm_mode / leg_mode / other_mode
                if part_lower.startswith("arm"):
                    mode = mode_map["arm"]
                elif part_lower.startswith("leg"):
                    mode = mode_map["leg"]
                else:
                    mode = other_mode

                if mode == "FK":
                    success = self.constrain_fk(part)
                else:
                    print(mode)
                    success = self.constrain_ik(part)

                if not success:
                    failed.append(part)
            elif part_type == "Hybrid":
                pass
            elif part_type == "Root":
                success = self.constrain_root(part)
                if not success:
                    failed.append(part)

        if failed:
            print("[Constrain] Failed parts:", failed)
        else:
            print("[Constrain] All parts constrained successfully")

    # --------------------------------------------------------

    def bake_retarget(self):
        if not mc.objExists(CONSTRAINT_SET):
            mc.warning("No retarget constraint set found")
            return

        # Collect target controls
        bake_nodes = set()

        for part in self.target_data.values():
            for ctrl_list in part.get("ControlList", {}).values():
                bake_nodes.update(ctrl_list)

        bake_nodes = list(bake_nodes)

        if not bake_nodes:
            mc.warning("No target controls to bake")
            return

        start = mc.playbackOptions(q=True, min=True)
        end = mc.playbackOptions(q=True, max=True)

        mc.bakeResults(
            bake_nodes,
            t=(start, end),
            at=["tx", "ty", "tz", "rx", "ry", "rz"],
            simulation=True
        )

        # Cleanup
        members = mc.sets(CONSTRAINT_SET, q=True) or []
        if members:
            mc.delete(members)
        mc.delete(CONSTRAINT_SET)

        print("[Bake] Retarget bake complete and cleaned up")

# ------------------------------------------------------------
# LAUNCH
# ------------------------------------------------------------

def show_retarget_tool():
    global _retarget_tool_ui
    try:
        _retarget_tool_ui.close()
    except:
        pass

    _retarget_tool_ui = RetargetToolUI()
    _retarget_tool_ui.show()

show_retarget_tool()
