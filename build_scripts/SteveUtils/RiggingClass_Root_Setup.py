import maya.cmds as mc
import re

try:
    from PySide6 import QtWidgets, QtCore
    from PySide6.QtWidgets import QDialog
except ImportError:
    from PySide2 import QtWidgets, QtCore
    from PySide2.QtWidgets import QDialog

import maya.OpenMayaUI as omui
try:
    from shiboken6 import wrapInstance
except ImportError:
    from shiboken2 import wrapInstance

def get_maya_main_window():
    main_window_ptr = omui.MQtUtil.mainWindow()
    return wrapInstance(int(main_window_ptr), QtWidgets.QWidget)


def build_basic_control(name='Main', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=(0, 0, 0), rotation=(0, 0, 0)):
    """
    Builds a basic control with an offset group. The offset group holds the transform.
    Uses RGB override color instead of color index.

    Args:
        name (str): Control name.
        shape (str): Shape type (currently just 'circle' supported).
        size (float): Size of the control.
        color_rgb (tuple): RGB color override.
        position (tuple): World position (x, y, z).
        rotation (tuple): World rotation (x, y, z).

    Returns:
        ctrl (str): The name of the control.
        offset_grp (str): The name of the offset group.
    """
    # Create the control
    ctrl = mc.circle(name=f'{name}_CTRL', normal=[0, 1, 0], radius=size, ch=False)[0]

    # Create offset group
    offset_grp = mc.group(empty=True, name=f"{name}_GRP")
    sdk_grp = mc.group(empty=True, name=f"{name}_SDK_GRP")
    mc.parent(ctrl, sdk_grp)
    mc.parent(sdk_grp, offset_grp)

    # Apply world-space transform to the group
    mc.xform(offset_grp, ws=True, translation=position, rotation=rotation)

    # Set control color using RGB
    mc.setAttr(f"{ctrl}.overrideEnabled", 1)
    mc.setAttr(f"{ctrl}.overrideRGBColors", 1)
    mc.setAttr(f"{ctrl}.overrideColorRGB", color_rgb[0], color_rgb[1], color_rgb[2], type="double3")

    return ctrl, offset_grp, sdk_grp


def build_simple_prop_rig(
        geo_grp_name: str = None,
        pivot: str = 'origin',
        rig_size: float = 1,
        zootools: bool = True,
        keep_rig: bool = False,
        rig_prefix: str = "TEMP",
        auto_skin: bool = True,
        keep_rig_geo: bool = True,
        clear_cache: bool = False,
        clear_guide: bool = False,
    ):

    if rig_prefix == "":
        rig_prefix='A'

    if mc.objExists("ROOT"):
        if keep_rig:
            mc.error('"ROOT" already exists in the scene and keep_rig is set to True.')
        else:
            if keep_rig_geo:
                model_grp = 'MODEL'
                if mc.objExists(model_grp):
                    # Check if the geo group exists, if not create it
                    geo_grp = mc.group(em=True, name='temp')

                    # Get children of MODEL group
                    children = mc.listRelatives(model_grp, children=True, fullPath=True) or []

                    # Parent each child to the geo group
                    for child in children:
                        mc.parent(child, geo_grp)


            mc.delete("ROOT")
            #mc.delete(geo_grp)

    # Create top-level groups
    root_grp = mc.group(em=True, name='ROOT')
    model_grp = mc.group(em=True, name='MODEL', parent=root_grp)
    rig_grp = mc.group(em=True, name='RIG', parent=root_grp)
    skel_grp = mc.group(em=True, name='SKEL', parent=root_grp)

    # Create the root joint under SKEL
    mc.select(clear=True)
    root_joint = mc.joint(name='root_BIND_JNT')
    mc.parent(root_joint, skel_grp)
    mc.makeIdentity(root_joint, apply=True, t=1, r=1, s=1, n=0)  # Freeze transforms

    if pivot == 'origin':
        MainControlPos = [0, 0, 0]
        MainControlRot = [0, 0, 0]
    
    # Build Root_CTRL
    build_basic_control(
        name=f'{rig_prefix}_Root',
        size=5 * rig_size,
        color_rgb=(1, 0.5, 0.01),
        position=(0, 0, 0),  # Positioned at the origin
        rotation=(0, 0, 0)   # No rotation
        )
    
    # Build Offset_CTRL
    build_basic_control(
        name=f'{rig_prefix}_Offset',
        size=4 * rig_size,  # 4 times the rig_size argument
        color_rgb=(0.95, 0.63, 0.37),  # #f2a15f in RGB
        position=(0, 0, 0),  # Positioned at the origin
        rotation=(0, 0, 0)   # No rotation
    )

    # === Build Main Control ===
    build_basic_control(
        name=f'{rig_prefix}_Main',  # Name with rig_prefix
        size=3 * rig_size,  # 3 times the rig_size argument
        color_rgb=(0.90, 0.38, 0.34),  # #e66057 in RGB
        position=MainControlPos,  # Use the calculated position
        rotation=MainControlRot   # Use the calculated rotation
    )
    # === Parent Controls ===
    mc.parent(f'{rig_prefix}_Offset_GRP', f'{rig_prefix}_Root_CTRL')  # Parent Offset_CTRL under Root_CTRL
    mc.parent(f'{rig_prefix}_Main_GRP', f'{rig_prefix}_Offset_CTRL')  # Parent Main_CTRL under Offset_CTRL
    mc.parent(f'{rig_prefix}_Root_GRP', 'RIG')

    # === Create Main Joint ===
    # Create the joint at the MainControlPos
    main_JNT = mc.joint(p=MainControlPos, orientation=MainControlRot)
    main_JNT = mc.rename(main_JNT, f'{rig_prefix}_main_BIND_JNT')
    mc.parent(main_JNT, 'root_BIND_JNT')

    # Constrain root joint to root control
    mc.parentConstraint(f"{rig_prefix}_Root_CTRL", f"root_BIND_JNT", mo=False)
    mc.scaleConstraint(f"{rig_prefix}_Root_CTRL", f"root_BIND_JNT", mo=False)

    # Constrain main joint to main control
    mc.parentConstraint(f"{rig_prefix}_Main_CTRL", f"{rig_prefix}_main_BIND_JNT", mo=False)
    mc.scaleConstraint(f"{rig_prefix}_Main_CTRL", f"{rig_prefix}_main_BIND_JNT", mo=False)

    cache_set_name = 'cache_SET'

    # Check if cache_SET already exists
    if mc.objExists(cache_set_name):
        if not clear_cache:
            mc.error("Cache Set Already Exists")
        else:
            mc.delete(cache_set_name)
            mc.sets("MODEL", name=cache_set_name)
    else:
        # Create the new selection set
        mc.sets("MODEL", name=cache_set_name)


    children = mc.listRelatives(geo_grp, children=True, fullPath=True) or []
    # Parent each child to the geo group
    for child in children:
        mc.parent(child, model_grp)

    mc.delete(geo_grp)



class SimplePropAutoRiggerUI(QDialog):
    def __init__(self, parent=get_maya_main_window()):
        super().__init__(parent)
        self.setWindowTitle("Simple Prop Auto Rigger")
        self.setMinimumWidth(300)
        self.setWindowFlags(self.windowFlags() ^ QtCore.Qt.WindowContextHelpButtonHint)

        self.build_ui()
        self.create_connections()

    def build_ui(self):
        # Widgets
        self.geo_input = QtWidgets.QLineEdit()
        self.prefix_input = QtWidgets.QLineEdit()
        self.rig_size_input = QtWidgets.QDoubleSpinBox()
        self.rig_size_input.setValue(2.0)
        self.rig_size_input.setSingleStep(0.1)
        self.rig_size_input.setMinimum(0.0)

        self.auto_skin_checkbox = QtWidgets.QCheckBox("Auto Skin")
        self.clear_cache_checkbox = QtWidgets.QCheckBox("Clear Cache")
        self.clear_guide_checkbox = QtWidgets.QCheckBox("Clear Guide")

        self.build_guides_button = QtWidgets.QPushButton("Build Guides")

        self.build_button = QtWidgets.QPushButton("Build Rig")

        # Layouts
        form_layout = QtWidgets.QFormLayout()
        form_layout.addRow("Rig Size:", self.rig_size_input)
        form_layout.addRow("Rig Prefix:", self.prefix_input)
        form_layout.addRow(self.build_button)

        self.setLayout(form_layout)

    def create_connections(self):
        self.build_button.clicked.connect(self.run_rig_function)


    def run_rig_function(self):
        rig_size = self.rig_size_input.value()
        rig_prefix = self.prefix_input.text()


        build_simple_prop_rig(
            geo_grp_name=None,
            pivot="origin",
            rig_size=rig_size,
            rig_prefix=rig_prefix,
            auto_skin=False,
        )



def show_simple_prop_rigger():
    global simple_prop_rigger_win
    try:
        simple_prop_rigger_win.close()
        simple_prop_rigger_win.deleteLater()
    except:
        pass

    simple_prop_rigger_win = SimplePropAutoRiggerUI()
    simple_prop_rigger_win.show()

show_simple_prop_rigger()