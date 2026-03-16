
from __future__ import annotations
import os
import json
import maya.cmds as mc

# ---------------- PYQT FALLBACK ----------------
try:
    from PySide6 import QtWidgets, QtCore
except ImportError:
    from PySide2 import QtWidgets, QtCore


# ----------------------------
# Snapshot Camera Utilities
# ----------------------------

def check_snapshot_camera(parent=None):

    if mc.objExists("snapshot_cam_grp"):

        msg = QtWidgets.QMessageBox(parent)
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

    if mc.objExists("snapshot_cam_grp"):
        mc.delete("snapshot_cam_grp")

    grp = mc.group(em=True, name="snapshot_cam_grp")
    swivel = mc.group(em=True, name="snapshot_cam_swivel", parent=grp)
    tilt = mc.group(em=True, name="snapshot_cam_tilt", parent=swivel)

    cam, shape = mc.camera(name="snapshot_cam")
    cam = mc.rename(cam, "snapshot_cam")

    mc.parent(cam, tilt)

    return cam


def auto_frame_camera(camera, obj):

    if not mc.objExists(obj):
        return

    mc.lookThru(camera)
    mc.select(obj)

    mc.viewFit(camera, obj)

    mc.select(clear=True)


def place_snapshot_camera(obj, swivel=0, tilt=0, focal_length=35, orthographic=False):

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

    shape = mc.listRelatives(cam, shapes=True)[0]

    size_x = bbox[3] - bbox[0]
    size_y = bbox[4] - bbox[1]
    size_z = bbox[5] - bbox[2]

    if orthographic:

        mc.setAttr(shape + ".orthographic", 1)

        max_size = max(size_x, size_y)
        mc.setAttr(shape + ".orthographicWidth", max_size * 1.2)

        depth_offset = max(size_x, size_y, size_z) * 2
        mc.setAttr(cam + ".translateZ", depth_offset)

    else:

        mc.setAttr(shape + ".orthographic", 0)
        mc.setAttr(shape + ".focalLength", focal_length)

        auto_frame_camera(cam, obj)

    mc.grid(toggle=False)

    return cam


def export_camera_json(camera, path, name="camera_settings"):

    if not mc.objExists(camera):
        return

    shape = mc.listRelatives(camera, shapes=True)[0]

    data = {}

    data["camera"] = camera

    data["group_translate"] = mc.xform("snapshot_cam_grp", q=True, ws=True, t=True)

    data["swivel"] = mc.getAttr("snapshot_cam_swivel.rotateY")
    data["tilt"] = mc.getAttr("snapshot_cam_tilt.rotateX")

    data["orthographic"] = mc.getAttr(shape + ".orthographic")

    if data["orthographic"]:
        data["orthographicWidth"] = mc.getAttr(shape + ".orthographicWidth")
    else:
        data["focalLength"] = mc.getAttr(shape + ".focalLength")

    file_path = os.path.join(path, name + ".json")

    with open(file_path, "w") as f:
        json.dump(data, f, indent=4)


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
        offScreen=True,
        showOrnaments=False,
        framePadding=0
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
        offScreen=True,
        showOrnaments=False
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

        # Snapshot Mode
        mode_section = CollapsibleSection("Snapshot Mode")
        mode_layout = QtWidgets.QVBoxLayout(mode_section.content)

        self.mode_dropdown = QtWidgets.QComboBox()
        self.mode_dropdown.addItems(["Headshot","Front","3_Quarters","Turntable","Manual"])
        self.mode_dropdown.currentIndexChanged.connect(self.apply_mode_preset)

        mode_layout.addWidget(self.mode_dropdown)

        obj_layout = QtWidgets.QHBoxLayout()

        self.obj_field = QtWidgets.QLineEdit()

        self.pick_btn = QtWidgets.QPushButton("Pick")
        self.pick_btn.clicked.connect(self.pick_object)

        obj_layout.addWidget(self.obj_field)
        obj_layout.addWidget(self.pick_btn)

        mode_layout.addLayout(obj_layout)

        main_layout.addWidget(mode_section)

        # Camera Settings
        cam_section = CollapsibleSection("Camera Settings")
        cam_layout = QtWidgets.QFormLayout(cam_section.content)

        self.swivel = QtWidgets.QDoubleSpinBox()
        self.swivel.setRange(-360,360)

        self.tilt = QtWidgets.QDoubleSpinBox()
        self.tilt.setRange(-360,360)

        self.focal_length = QtWidgets.QDoubleSpinBox()
        self.focal_length.setRange(1,300)
        self.focal_length.setValue(35)

        self.orthographic = QtWidgets.QCheckBox("Orthographic Camera")

        cam_layout.addRow("Swivel",self.swivel)
        cam_layout.addRow("Tilt",self.tilt)
        cam_layout.addRow("Focal Length",self.focal_length)
        cam_layout.addRow("Orthographic",self.orthographic)

        main_layout.addWidget(cam_section)

        # Image Size
        size_layout = QtWidgets.QHBoxLayout()

        self.img_width = QtWidgets.QSpinBox()
        self.img_width.setRange(16,8192)
        self.img_width.setValue(512)

        self.img_height = QtWidgets.QSpinBox()
        self.img_height.setRange(16,8192)
        self.img_height.setValue(512)

        size_layout.addWidget(self.img_width)
        size_layout.addWidget(self.img_height)

        cam_layout.addRow("Image Size", size_layout)

        # Output
        output_section = CollapsibleSection("Output Settings")
        output_layout = QtWidgets.QFormLayout(output_section.content)

        default_path = r"G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\camera_util"

        self.path_field = QtWidgets.QLineEdit(default_path)

        browse_btn = QtWidgets.QPushButton("Browse")
        browse_btn.clicked.connect(self.browse_folder)

        path_layout = QtWidgets.QHBoxLayout()
        path_layout.addWidget(self.path_field)
        path_layout.addWidget(browse_btn)

        self.snapshot_name = QtWidgets.QLineEdit("snapshot")

        output_layout.addRow("Output Folder", path_layout)
        output_layout.addRow("File Name", self.snapshot_name)
        self.export_json = QtWidgets.QCheckBox("Export Camera JSON")
        self.export_json.setChecked(False)

        output_layout.addRow("", self.export_json)

        main_layout.addWidget(output_section)

        # Capture
        capture_section = CollapsibleSection("Capture Options")
        capture_layout = QtWidgets.QVBoxLayout(capture_section.content)

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

        snap_btn = QtWidgets.QPushButton("Take Snapshot")
        snap_btn.clicked.connect(self.snapshot)

        playblast_btn = QtWidgets.QPushButton("Playblast")
        playblast_btn.clicked.connect(self.playblast)

        capture_layout.addWidget(snap_btn)
        capture_layout.addWidget(playblast_btn)

        main_layout.addWidget(capture_section)

        

        # -------------------
        # Manual Controls
        # -------------------

        manual_section = CollapsibleSection("Manual Mode")
        manual_layout = QtWidgets.QVBoxLayout(manual_section.content)

        manual_cam_btn = QtWidgets.QPushButton("Manual Camera")
        manual_cam_btn.clicked.connect(self.manual_camera)

        manual_snap_btn = QtWidgets.QPushButton("Manual Snapshot")
        manual_snap_btn.clicked.connect(self.manual_snapshot)

        manual_layout.addWidget(manual_cam_btn)
        manual_layout.addWidget(manual_snap_btn)

        main_layout.addWidget(manual_section)

    def apply_mode_preset(self):

        mode = self.mode_dropdown.currentText()

        if mode == "Front":
            self.swivel.setValue(0)
            self.tilt.setValue(0)

        elif mode == "Headshot":
            self.swivel.setValue(0)
            self.tilt.setValue(10)

        elif mode == "3_Quarters":
            self.swivel.setValue(45)
            self.tilt.setValue(10)

        elif mode == "Turntable":
            self.swivel.setValue(0)
            self.tilt.setValue(10)

    def build_turntable(self):

        start = self.start_frame.value()
        end = self.end_frame.value()

        mc.setKeyframe("snapshot_cam_swivel.rotateY", t=start, v=0)
        mc.setKeyframe("snapshot_cam_swivel.rotateY", t=end, v=360)

        mc.selectKey("snapshot_cam_swivel.rotateY")
        mc.keyTangent(itt="linear", ott="linear")

    def pick_object(self):

        sel = mc.ls(sl=True)

        if sel:
            obj = sel[0]
            self.obj_field.setText(obj)

            # turn field green when valid object is picked
            self.obj_field.setStyleSheet(
                "QLineEdit { background-color: rgb(60,120,60); color: white; }"
            )

    def browse_folder(self):

        folder = QtWidgets.QFileDialog.getExistingDirectory(self,"Select Output Folder")

        if folder:
            self.path_field.setText(folder)

    def copy_timeline_range(self):

        start = mc.playbackOptions(q=True,min=True)
        end = mc.playbackOptions(q=True,max=True)

        self.start_frame.setValue(int(start))
        self.end_frame.setValue(int(end))

    def build_camera(self):

        if not check_snapshot_camera(self):
            return None

        obj = self.obj_field.text()

        if not mc.objExists(obj):
            mc.warning("Object does not exist.")
            return None

        cam = place_snapshot_camera(
            obj,
            swivel=self.swivel.value(),
            tilt=self.tilt.value(),
            focal_length=self.focal_length.value(),
            orthographic=self.orthographic.isChecked()
        )

        if self.mode_dropdown.currentText() == "Turntable":
            self.build_turntable()

        return cam

    def snapshot(self):
        cam = self.build_camera()
        if not cam:
            return

        path   = self.path_field.text()
        width  = self.img_width.value()   # ✓ read from UI
        height = self.img_height.value()  # ✓ read from UI
        name   = self.snapshot_name.text()

        take_snapshot(cam, path, width, height, name)

        if self.export_json.isChecked():
            export_camera_json(cam, path, name + "_camera")
    def playblast(self):
        cam = self.build_camera()
        if not cam:
            return

        path   = self.path_field.text()
        width  = self.img_width.value()   # ✓ read from UI
        height = self.img_height.value()  # ✓ read from UI
        name   = self.snapshot_name.text()

        take_playblast(
            cam, path,
            self.start_frame.value(),
            self.end_frame.value(),
            width, height,
            name
        )

        if self.export_json.isChecked():
            export_camera_json(cam, path, name + "_camera")

    def manual_camera(self):

        if not check_snapshot_camera(self):
            return

        obj = self.obj_field.text()

        if not mc.objExists(obj):
            mc.warning("Object does not exist.")
            return

        place_snapshot_camera(
            obj,
            swivel=self.swivel.value(),
            tilt=self.tilt.value(),
            focal_length=self.focal_length.value(),
            orthographic=self.orthographic.isChecked()
        )

    def manual_snapshot(self):

        cam = "snapshot_cam"

        if not mc.objExists(cam):
            mc.warning("Snapshot camera does not exist.")
            return

        path = self.path_field.text()
        width = self.img_width.value()
        height = self.img_height.value()

        name = self.snapshot_name.text()

        take_snapshot(cam, path, width, height, name)

        if self.export_json.isChecked():
            export_camera_json(cam, path, name + "_camera")


def run():

    global snapshot_ui

    try:
        snapshot_ui.close()
    except:
        pass

    snapshot_ui = SnapshotUI()
    snapshot_ui.show()
