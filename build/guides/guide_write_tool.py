from __future__ import annotations
import json
import os

import maya.cmds as mc
import maya.api.OpenMaya as om

try:
    from PySide6 import QtWidgets, QtCore
except:
    from PySide2 import QtWidgets, QtCore

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
        self.type_cb.addItems(["chain", "single", "sequence"])
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
        
        else:
            #if self.type_cb.currentText() not in  ["chain", 'sequence']:
            mc.warning("Only chain and sequence mode implemented.")
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

        path = r"G:\bobo\pipeline\pipeline\software\maya\scripts\rjg\build\guides\parts"
        os.makedirs(path, exist_ok=True)

        data = {
            "part": self.part_le.text(),
            "type": self.type_cb.currentText(),
            "parent": self.parent_le.text(),
            "guides": {}
        }

        if self.type_cb.currentText() == 'chain':
            guides = mc.ls("chain_guide_*", type="joint")

        for jnt in guides:
            null = mc.listRelatives(jnt, p=True)[0]

            mesh = mc.getAttr(jnt+".mesh")
            vert_list = json.loads(mc.getAttr(jnt+".vertList"))
            upvect = mc.getAttr(jnt+".upVectorVert")

            pos = mc.xform(null, q=True, ws=True, t=True)

            data["guides"][jnt] = {
                "mesh": mesh,
                "pos": pos,
                "offset": mc.xform(jnt, q=True, t=True),
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