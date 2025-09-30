from rjg.libs.control.ctrl import Control, tag_as_controller


import maya.cmds as mc
from importlib import reload

import rjg.build.rigModule as rModule
import rjg.build.chain as rChain
import rjg.libs.spline as spline
import rjg.libs.control.ctrl as rCtrl
import rjg.libs.attribute as rAttr
import rjg.libs.transform as rXform 

reload(rModule)
reload(rChain)
reload(spline)
reload(rCtrl)
reload(rAttr)


class HybridSpine(rModule.RigModule):
    def __init__(
        self,
        side: str,
        part: str,
        guide_list: list[str],
        ctrl_scale: float = 1,
        joint_num: int = 5,
    ):
        super().__init__(side=side, part=part, guide_list=guide_list, ctrl_scale=ctrl_scale)
        if len(self.guide_list) != 4:
            raise ValueError(
                "HybridSpine needs a list of 4 guides [Base, Bend Point, Chest, End]"
                f"current guides: {guide_list}"
            )
        self.joint_num: int = joint_num
        self.__dict__.update(locals())

        self.base_name = self.part + "_" + self.side

        self.create_module()

    def create_module(self):
        super().create_module()

        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.compatibility_transforms()
        self.add_plugs()

    def control_rig(self):
        # Get Guides
        base_jnt = self.guide_list[0]
        chest_jnt = self.guide_list[1]
        chest_top_jnt = self.guide_list[2]
        end_jnt = self.guide_list[3]

        # Build FK controls
        base_ctrl: Control = rCtrl.Control(
            name="torso",
            parent=self.control_grp,
            shape="circle",
            side=self.side,
            axis="y",
            group_type="main",
            rig_type="primary",
            translate=base_jnt,
            rotate=base_jnt,
            ctrl_scale=self.ctrl_scale * 14,
            rotate_order=1
        )
        tag_as_controller(base_ctrl.ctrl)
        self.base_ctrl = base_ctrl

        spine_mid_ctrl: Control = rCtrl.Control(
            name="spine_mid",
            parent=base_ctrl.ctrl,
            shape="circle",
            side=self.side,
            axis="y",
            group_type="main",
            rig_type="primary",
            translate=chest_jnt,
            rotate=chest_jnt,
            ctrl_scale=self.ctrl_scale * 12,
            rotate_order=1
        )
        tag_as_controller(spine_mid_ctrl.ctrl)
        self.spine_mid_ctrl = spine_mid_ctrl

        chest_ctrl: Control = rCtrl.Control(
            name="chest",
            parent=base_ctrl.ctrl,
            shape="circle",
            side=self.side,
            axis="y",
            group_type="main",
            rig_type="primary",
            translate=chest_jnt,
            rotate=chest_jnt,
            ctrl_scale=self.ctrl_scale * 14,
            shape_translate=end_jnt,
            shape_rotate=chest_top_jnt,
            rotate_order=1
        )
        tag_as_controller(chest_ctrl.ctrl)
        self.chest_ctrl = chest_ctrl

        ik_chest_ctrl: Control = rCtrl.Control(
            name="IK_chest",
            parent=chest_ctrl.ctrl,
            shape="hexagon",
            side=self.side,
            axis="y",
            group_type="main",
            rig_type="primary",
            translate=chest_jnt,
            rotate=chest_jnt,
            ctrl_scale=self.ctrl_scale * 14,
            shape_translate=chest_top_jnt,
            shape_rotate=chest_top_jnt,
            rotate_order=1
        )
        tag_as_controller(ik_chest_ctrl.ctrl)
        self.ik_chest_ctrl = ik_chest_ctrl

        chest_top_ctrl: Control = rCtrl.Control(
            name="chest_top",
            parent=ik_chest_ctrl.ctrl,
            shape="hexagon",
            side=self.side,
            axis="y",
            group_type="main",
            rig_type="primary",
            translate=chest_top_jnt,
            rotate=chest_top_jnt,
            ctrl_scale=self.ctrl_scale * 14,
            shape_translate=(0, self.ctrl_scale * 2, 0),
            rotate_order=1
        )
        tag_as_controller(chest_top_ctrl.ctrl)
        self.chest_top_ctrl = chest_top_ctrl

        spine_start: str = mc.group(empty=True, parent=self.module_grp, name=f"{self.part}_startPoint")
        rXform.match_pose(node=spine_start, translate=base_jnt, rotate=base_jnt)
        rXform.matrix_constraint(base_ctrl.ctrl, spine_start)

        spine_mid: str =  mc.group(empty=True, parent=self.module_grp, name=f"{self.part}_midPoint")
        rXform.match_pose(node=spine_mid, translate=chest_jnt, rotate=chest_jnt)
        rXform.matrix_constraint(base_ctrl.ctrl, spine_mid)
        
        spine_end: str = mc.group(empty=True, parent=self.module_grp, name=f"{self.part}_endPoint")
        rXform.match_pose(node=spine_end, translate=chest_top_jnt, rotate=chest_top_jnt)
        rXform.matrix_constraint(ik_chest_ctrl.ctrl, spine_end)

        # Twist for mid joint
        mult_matrix = mc.createNode('multMatrix', name=f"{self.part}_TwistRelativeMatrix") # Put the end joint into the space of the start joint
        mc.connectAttr(f"{spine_end}.worldMatrix[0]", f"{mult_matrix}.matrixIn[0]")
        mc.connectAttr(f"{spine_start}.worldInverseMatrix[0]", f"{mult_matrix}.matrixIn[1]") # Retrieve the rotation from the resulting matrix
        decompose_matrix = mc.createNode('decomposeMatrix', name=f"{self.part}_Twist_DCM") 
        mc.connectAttr(f"{mult_matrix}.matrixSum", f"{decompose_matrix}.inputMatrix")
        # Create a quaternion from only the Y (down the chain axis) and W (scalar component)
        # The resulting quaternion is the twist part of a swing twist decomposition. 
        quat_to_euler = mc.createNode('quatToEuler', name=f"{self.part}_Twist_QTE")
        mc.connectAttr(f"{decompose_matrix}.outputQuatY", f"{quat_to_euler}.inputQuatY")
        mc.connectAttr(f"{decompose_matrix}.outputQuatW", f"{quat_to_euler}.inputQuatW")
        mc.setAttr(f"{quat_to_euler}.inputRotateOrder", 1) # Make sure the rotate order is set so that the Y is the twist axis
        # Use the resulting twist value
        twist_mult = mc.createNode("multiply", name=f"{self.part}_Twist_Mid_MLT")
        mc.connectAttr(f"{quat_to_euler}.outputRotateY", f"{twist_mult}.input[0]")
        mc.setAttr(f"{twist_mult}.input[1]", 0.5)
        mc.connectAttr(f"{twist_mult}.output", f"{spine_mid_ctrl.ctrl_name}_SDK_GRP.rotateY")

        spline.matrix_spline_from_transforms(
            name=f"{self.part}_Mid",
            transforms=[base_ctrl.ctrl, spine_mid, spine_end],
            transforms_to_pin=[spine_mid_ctrl.top],
            twist=False,
            stretch=False,
            primary_axis=(0, 1, 0),
            arc_length=False,
            degree=2,
            parent=self.module_grp,
        )
        self.spine_cvs: list[str] = [spine_start, spine_mid_ctrl.ctrl, spine_end]

        self.tweak_ctrls: list[Control] = []
        self.tweak_transforms: list[str] = []
        self.joint_drivers: list[str] = []
        # Create spine tweak controls for mid joints
        for i in range(self.joint_num):
            # Skip making controls for the first and last tweak points
            if i == 0 or i == self.joint_num - 1:
                tweak_point = mc.group(empty=True, name=f"{self.part}_Tweak_{i:02}", parent=self.module_grp)
                self.tweak_transforms.append(tweak_point)
                if i == self.joint_num - 1:
                    self.joint_drivers.append(chest_top_ctrl.ctrl)
                else:
                    self.joint_drivers.append(tweak_point)
            else:
                tweak_ctrl: Control = rCtrl.Control(
                    name=f"{self.part}_Tweak_{i:02}",
                    parent=ik_chest_ctrl.ctrl,
                    shape="circle",
                    side=self.side,
                    axis="y",
                    group_type="main",
                    rig_type="bendy",
                    translate=chest_top_jnt,
                    rotate=chest_top_jnt,
                    ctrl_scale=self.ctrl_scale * 1,
                    shape_translate=(0, 0, self.ctrl_scale * 12),
                    shape_rotate=(90, 0, 0),
                    rotate_order=1
                )
                tag_as_controller(tweak_ctrl.ctrl)
                self.tweak_ctrls.append(tweak_ctrl)
                self.tweak_transforms.append(tweak_ctrl.top)
                self.joint_drivers.append(tweak_ctrl.ctrl)
        
        

    def output_rig(self):
        spline.matrix_spline_from_transforms(
            name=f"{self.part}_Spline",
            transforms=self.spine_cvs,
            transforms_to_pin=self.tweak_transforms,
            padded=False,
            stretch=False,
            parent=self.module_grp,
            primary_axis=(0,1,0),
            secondary_axis=(0,0,1),
            degree=2,
        )

    def skeleton(self):
        spine_chain = rChain.Chain(
            transform_list=self.joint_drivers, side=self.side, suffix="JNT", name=self.part
        )
        spine_chain.create_from_transforms(parent=self.skel)
        self.bind_joints = spine_chain.joints
        self.tag_bind_joints(self.bind_joints[:-1])

    def compatibility_transforms(self) -> None:
        # These are needed until we refactor the modules to actually be modular instead of having hardcoded connections between each other.

        # Arm, Clavicle, and Neck dependencies:
        chest_01 = mc.group(name="chest_M_01_CTRL", empty=True, parent=self.module_grp)
        rXform.matrix_constraint(self.ik_chest_ctrl.ctrl, chest_01)
        chest_02 = mc.group(name="chest_M_02_CTRL", empty=True, parent=self.module_grp)
        rXform.matrix_constraint(self.chest_top_ctrl.ctrl, chest_02)
        chest_02_jnt = mc.group(name="chest_M_02_JNT", empty=True, parent=self.module_grp)
        rXform.matrix_constraint(self.chest_top_ctrl.ctrl, chest_02_jnt)

        # Leg dependencies:

        pass

    def add_plugs(self):
        if self.part == "spine":
            # add skeleton plugs
            rAttr.Attribute(
                node=self.part_grp,
                type="plug",
                value=["COG_M_JNT"],
                name="skeletonPlugs",
                children_name=[self.bind_joints[0]],
            )

            # add parentConstraint rig plugs
            driver_list = ["waist_M_CTRL"]
            driven_list = [self.base_name + "_torso_M_CTRL_CNST_GRP"]
            rAttr.Attribute(
                node=self.part_grp,
                type="plug",
                value=driver_list,
                name="pacRigPlugs",
                children_name=driven_list,
            )
