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