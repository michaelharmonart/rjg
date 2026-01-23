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

    def constrain_fk(self, part, src_controls_override=None, tgt_controls_override=None):
        """
        Constrain a single FK part.
        Adds orient constraints for all subparts.
        Optional src/tgt overrides allow mirrored control lists.
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

        src_controls = src_controls_override or src_part.get("ControlList", {})
        tgt_controls = tgt_controls_override or tgt_part.get("ControlList", {})

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

    
    def fk_distribute_constraint(self, part, src_controls_override=None, tgt_controls_override=None):
        """
        Distribute summed FK rotations across multiple target controls.
        Uses plusMinusAverage + multiplyDivide to average rotations.
        Optional src/tgt overrides allow mirrored control lists.
        """

        src_part = self.source_data[part]
        tgt_part = self.target_data[part]

        if src_part.get("Type") != "FK_Distribute":
            mc.warning(f"[FK Distribute] {part} is not FK_Distribute")
            return False

        if not mc.objExists(CONSTRAINT_SET):
            mc.sets(name=CONSTRAINT_SET, empty=True)

        src_ns = self.source_ns.text()
        tgt_ns = self.target_ns.text()

        failed = []
        created = []

        src_controls = src_controls_override or src_part.get("ControlList", {})
        tgt_controls = tgt_controls_override or tgt_part.get("ControlList", {})

        for subpart, src_list in src_controls.items():
            tgt_list = tgt_controls.get(subpart, [])

            if not src_list or not tgt_list:
                mc.warning(f"[FK Distribute] Empty control list for {part}:{subpart}")
                failed.append(part)
                continue

            src_nodes = [
                f"{src_ns}:{c}" if src_ns else c
                for c in src_list
                if mc.objExists(f"{src_ns}:{c}" if src_ns else c)
            ]

            tgt_nodes = [
                f"{tgt_ns}:{c}" if tgt_ns else c
                for c in tgt_list
                if mc.objExists(f"{tgt_ns}:{c}" if tgt_ns else c)
            ]

            if len(src_nodes) != len(src_list) or len(tgt_nodes) != len(tgt_list):
                mc.warning(f"[FK Distribute] Missing source or target controls for {part}")
                failed.append(part)
                continue

            # plusMinusAverage node (sum rotations)
            pma = mc.createNode(f"plusMinusAverage", name=f"{part}_FKDist_add_PMA")
            mc.setAttr(f"{pma}.operation", 1)  # Sum

            for i, src in enumerate(src_nodes):
                mc.connectAttr(f"{src}.rotate", f"{pma}.input3D[{i}]", force=True)

            # multiplyDivide node (average)
            count = len(tgt_nodes)
            inv = 1.0 / float(count)

            md = mc.createNode(f"multiplyDivide", name=f"{part}_FKDist_avg_MD")
            mc.setAttr(f"{md}.operation", 1)  # Multiply
            mc.connectAttr(f"{pma}.output3D", f"{md}.input1", force=True)
            mc.setAttr(f"{md}.input2X", inv)
            mc.setAttr(f"{md}.input2Y", inv)
            mc.setAttr(f"{md}.input2Z", inv)

            # Connect to target rotates
            for tgt in tgt_nodes:
                mc.connectAttr(f"{md}.output", f"{tgt}.rotate", force=True)

            # Add nodes to set
            mc.sets(pma, add=CONSTRAINT_SET)
            mc.sets(md, add=CONSTRAINT_SET)
            created.extend([pma, md])

        return len(failed) == 0



    def constrain_root(self, part, src_controls_override=None, tgt_controls_override=None):
        """
        Constrain a single Root part.
        Adds both orient and point constraints for all subparts.
        Optional src/tgt overrides allow mirrored control lists.
        """
        src_part = self.source_data[part]
        tgt_part = self.target_data[part]

        if not mc.objExists(CONSTRAINT_SET):
            mc.sets(name=CONSTRAINT_SET, empty=True)

        src_ns = self.source_ns.text()
        tgt_ns = self.target_ns.text()

        failed = []
        created = []

        src_controls = src_controls_override or src_part.get("ControlList", {})
        tgt_controls = tgt_controls_override or tgt_part.get("ControlList", {})

        for subpart, src_list in src_controls.items():
            tgt_list = tgt_controls.get(subpart, [])
            for src_ctrl, tgt_ctrl in zip(src_list, tgt_list):
                src_node = f"{src_ns}:{src_ctrl}" if src_ns else src_ctrl
                tgt_node = f"{tgt_ns}:{tgt_ctrl}" if tgt_ns else tgt_ctrl

                if not mc.objExists(src_node) or not mc.objExists(tgt_node):
                    mc.warning(f"[Root Constrain] Missing node: {src_node} or {tgt_node}")
                    failed.append(part)
                    continue

                con = mc.orientConstraint(src_node, tgt_node, mo=src_part.get("MO", True))[0]
                mc.sets(con, add=CONSTRAINT_SET)
                con = mc.pointConstraint(src_node, tgt_node, mo=src_part.get("MO", True))[0]
                mc.sets(con, add=CONSTRAINT_SET)
                created.append(con)

        return len(failed) == 0


    def constrain_ik(self, part, src_controls_override=None, tgt_controls_override=None):
        """
        Constrain a single IK part (arm or leg).
        Optional src/tgt overrides allow mirrored control lists.
        """
        src_part = self.source_data[part]
        tgt_part = self.target_data[part]

        src_ns = self.source_ns.text()
        tgt_ns = self.target_ns.text()

        if not mc.objExists(CONSTRAINT_SET):
            mc.sets(name=CONSTRAINT_SET, empty=True)

        failed = []
        created = []

        src_controls = src_controls_override or src_part.get("ControlList", {})
        tgt_controls = tgt_controls_override or tgt_part.get("ControlList", {})

        ARM_PARTS = ["arm"]
        LEG_PARTS = ["leg"]
        part_lower = part.lower()

        if part_lower in ARM_PARTS:
            # Arm IK logic
            try:
                src_fk_hand = src_controls["FkWrist"][0]
                src_fk_elbow = src_controls["FkElbow"][0]

                tgt_ik_hand = tgt_controls.get("IKHand", [None])[0]
                tgt_ik_pv = tgt_controls.get("IKPV", [None])[0]

                if not src_fk_hand or not tgt_ik_hand or not src_fk_elbow or not tgt_ik_pv:
                    mc.warning(f"[IK Constrain] Missing arm controls for part '{part}'")
                    return False

                pc = mc.parentConstraint(f"{src_ns}:{src_fk_hand}" if src_ns else src_fk_hand,
                                        f"{tgt_ns}:{tgt_ik_hand}" if tgt_ns else tgt_ik_hand,
                                        mo=True)[0]
                mc.sets(pc, add=CONSTRAINT_SET)
                created.append(pc)

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
            # Leg IK logic
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

                pc = mc.parentConstraint(f"{src_ns}:{src_fk_ankle}" if src_ns else src_fk_ankle,
                                        f"{tgt_ns}:{tgt_ik_foot}" if tgt_ns else tgt_ik_foot,
                                        mo=True)[0]
                mc.sets(pc, add=CONSTRAINT_SET)
                created.append(pc)

                pt = mc.pointConstraint(f"{src_ns}:{src_fk_knee}" if src_ns else src_fk_knee,
                                        f"{tgt_ns}:{tgt_ik_pv}" if tgt_ns else tgt_ik_pv,
                                        mo=True)[0]
                mc.sets(pt, add=CONSTRAINT_SET)
                created.append(pt)

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

    
    def get_mirrored_control_lists(self, part):
        """
        Returns temporary mirrored copies of source and target control lists for this part.
        Does NOT mutate JSON.
        """
        src_part = self.source_data[part]
        tgt_part = self.target_data[part]

        mirror_labels = src_part.get("Mirror_Label")
        if not mirror_labels or len(mirror_labels) != 2:
            return src_part.get("ControlList", {}), tgt_part.get("ControlList", {})

        left, right = mirror_labels

        def mirror_list(ctrls):
            return [c.replace(left, right) for c in ctrls]

        # Build mirrored dicts
        mirrored_src = {subpart: mirror_list(ctrls)
                        for subpart, ctrls in src_part.get("ControlList", {}).items()}
        mirrored_tgt = {subpart: mirror_list(ctrls)
                        for subpart, ctrls in tgt_part.get("ControlList", {}).items()}

        return mirrored_src, mirrored_tgt

    def run_constraint_with_optional_lists(self, func, part, src_override=None, tgt_override=None):
        """
        Calls a constraint function with optional overridden control lists.
        The function will temporarily use the overridden lists if provided.
        """
        # Store original lists
        src_part = self.source_data[part]
        tgt_part = self.target_data[part]

        orig_src = src_part.get("ControlList")
        orig_tgt = tgt_part.get("ControlList")

        try:
            if src_override is not None:
                src_part["ControlList"] = src_override
            if tgt_override is not None:
                tgt_part["ControlList"] = tgt_override

            return func(part)
        finally:
            # Restore original lists
            src_part["ControlList"] = orig_src
            tgt_part["ControlList"] = orig_tgt





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
            part_mirror = src_part.get("Mirror", False)

            if part_type not in VALID_TYPES:
                mc.warning(f"[Constrain] Invalid type '{part_type}' on {part}")
                failed.append(part)
                continue

            # Select the correct constraint function
            if part_type == "FK":
                func = self.constrain_fk
            elif part_type == "IK":
                func = self.constrain_ik
            elif part_type == "FK_Distribute":
                func = self.fk_distribute_constraint
            elif part_type == "FK_IK":
                # Determine mode (FK or IK)
                part_lower = part.lower()
                arm_mode = self.arm_fkik.currentText()
                leg_mode = self.leg_fkik.currentText()
                other_mode = self.other_fkik.currentText()

                if part_lower.startswith("arm"):
                    mode = arm_mode
                elif part_lower.startswith("leg"):
                    mode = leg_mode
                else:
                    mode = other_mode

                func = self.constrain_fk if mode == "FK" else self.constrain_ik
            elif part_type == "Root":
                func = self.constrain_root
            else:
                mc.warning(f"[Constrain] No logic for part '{part}'")
                failed.append(part)
                continue

            # -------------------------
            # Run normal constraint
            # -------------------------
            success = func(part)
            if not success:
                failed.append(part)

            # -------------------------
            # Run mirrored constraint if needed
            # -------------------------
            if part_mirror and part_type != "Root":  # Root usually not mirrored
                mirrored_src, mirrored_tgt = self.get_mirrored_control_lists(part)
                success = func(part, src_controls_override=mirrored_src, tgt_controls_override=mirrored_tgt)
                if not success:
                    failed.append(f"{part} (mirror)")

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
