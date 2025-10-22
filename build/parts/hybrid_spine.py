from rjg.libs.control.ctrl import Control, tag_as_controller


import maya.cmds as mc
import maya.api.OpenMaya as om2
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
        base_guide: str,
        hip_pivot_guide: str,
        chest_pivot_guide: str,
        upper_chest_pivot_guide: str,
        spine_end_guide: str,
        ctrl_scale: float = 1,
        joint_num: int = 5,
        base_tangent: float = 0.15,
        mid_tangent: float = 0.2,
        end_tangent: float = 0.15,
    ):
        super().__init__(
            side=side,
            part=part,
            guide_list=[
                base_guide,
                hip_pivot_guide,
                chest_pivot_guide,
                upper_chest_pivot_guide,
                spine_end_guide,
            ],
            ctrl_scale=ctrl_scale,
        )

        self.joint_num: int = joint_num
        self.__dict__.update(locals())

        self.base_guide: str = base_guide
        self.hip_pivot_guide: str = hip_pivot_guide
        self.chest_pivot_guide: str = chest_pivot_guide
        self.upper_chest_pivot_guide: str = upper_chest_pivot_guide
        self.spine_end_guide: str = spine_end_guide

        self.base_name = self.part + "_" + self.side
        self.base_tangent: float = base_tangent
        self.mid_tangent: float = mid_tangent
        self.end_tangent: float = end_tangent
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
        base_jnt = self.base_guide
        hip_pivot_jnt = self.hip_pivot_guide
        chest_jnt = self.chest_pivot_guide
        chest_top_jnt = self.upper_chest_pivot_guide
        end_jnt = self.spine_end_guide

        # Build FK controls
        hip_ctrl: Control = rCtrl.Control(
            name="hip",
            parent=self.control_grp,
            shape="circle",
            side=self.side,
            axis="y",
            group_type="main",
            rig_type="primary",
            translate=hip_pivot_jnt,
            rotate=hip_pivot_jnt,
            ctrl_scale=self.ctrl_scale * 12,
            rotate_order=1,
            shape_translate=base_jnt,
            shape_rotate=base_jnt,
        )
        tag_as_controller(hip_ctrl.ctrl)
        self.hip_ctrl = hip_ctrl

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
            rotate_order=1,
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
            rotate_order=1,
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
            rotate_order=1,
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
            rotate_order=1,
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
            rotate_order=1,
        )
        tag_as_controller(chest_top_ctrl.ctrl)
        self.chest_top_ctrl = chest_top_ctrl

        # Create transforms to make a spline to drive the mid control position
        spine_start: str = mc.group(
            empty=True, parent=self.module_grp, name=f"{self.part}_startPoint"
        )
        rXform.match_pose(node=spine_start, translate=base_jnt, rotate=base_jnt)
        rXform.matrix_constraint(hip_ctrl.ctrl, spine_start)
        spine_mid: str = mc.group(empty=True, parent=self.module_grp, name=f"{self.part}_midPoint")
        rXform.match_pose(node=spine_mid, translate=chest_jnt, rotate=chest_jnt)
        rXform.matrix_constraint(base_ctrl.ctrl, spine_mid)
        spine_end: str = mc.group(empty=True, parent=self.module_grp, name=f"{self.part}_endPoint")
        rXform.match_pose(node=spine_end, translate=chest_top_jnt, rotate=chest_top_jnt)
        rXform.matrix_constraint(chest_top_ctrl.ctrl, spine_end)

        # Twist for mid joint
        # Retrieve the rotation from the top and bottom matrices
        start_decompose_matrix = mc.createNode("decomposeMatrix", name=f"{self.part}_TwistStart_DCM")
        mc.connectAttr(f"{spine_start}.matrix", f"{start_decompose_matrix}.inputMatrix")
        end_decompose_matrix = mc.createNode("decomposeMatrix", name=f"{self.part}_TwistEnd_DCM")
        mc.connectAttr(f"{spine_end}.matrix", f"{end_decompose_matrix}.inputMatrix")

        # Create a quaternion from only the Y (down the chain axis) and W (scalar component)
        # The resulting quaternion is the twist part of a swing twist decomposition.
        # Slerp to the rotation halfway between the two twist quaternions.
        slerp = mc.createNode("quatSlerp", name=f"{self.part}_Twist_Slerp")
        mc.connectAttr(f"{start_decompose_matrix}.outputQuatY", f"{slerp}.input1QuatY")
        mc.connectAttr(f"{start_decompose_matrix}.outputQuatW", f"{slerp}.input1QuatW")
        mc.connectAttr(f"{end_decompose_matrix}.outputQuatY", f"{slerp}.input2QuatY")
        mc.connectAttr(f"{end_decompose_matrix}.outputQuatW", f"{slerp}.input2QuatW")
        mc.setAttr(f"{slerp}.inputT", 0.5)

        quat_to_euler = mc.createNode("quatToEuler", name=f"{self.part}_Twist_QTE")
        mc.connectAttr(f"{slerp}.outputQuat", f"{quat_to_euler}.inputQuat")
        # Make sure the rotate order is set so that the Y is the twist axis
        mc.setAttr(f"{quat_to_euler}.inputRotateOrder", 1)
        # Use the resulting twist value
        mc.connectAttr(f"{quat_to_euler}.outputRotateY", f"{spine_mid_ctrl.ctrl_name}_SDK_GRP.rotateY")

        # Create the spline to drive the mid control position. (3 control points, quadratic)
        spline.matrix_spline_from_transforms(
            name=f"{self.part}_Mid",
            transforms=[spine_start, spine_mid, spine_end],
            transforms_to_pin=[spine_mid_ctrl.top],
            twist=False,
            stretch=False,
            primary_axis=(0, 1, 0),
            arc_length=False,
            degree=2,
            parent=self.module_grp,
        )

        self.tweak_ctrls: list[Control] = []
        self.tweak_transforms: list[str] = []
        self.joint_drivers: list[str] = []
        # Create spine tweak controls for mid joints
        for i in range(self.joint_num):
            # Skip making controls for the first and last tweak points
            if i == 0 or i == self.joint_num - 1:
                tweak_point = mc.group(
                    empty=True, name=f"{self.part}_Tweak_{i:02}", parent=self.module_grp
                )
                self.tweak_transforms.append(tweak_point)
                if i == self.joint_num - 1:
                    self.joint_drivers.append(chest_top_ctrl.ctrl)
                else:
                    self.joint_drivers.append(tweak_point)
            else:
                tweak_ctrl: Control = rCtrl.Control(
                    name=f"{self.part}_Tweak_{i:02}",
                    parent=self.control_grp,
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
                    rotate_order=1,
                )
                tag_as_controller(tweak_ctrl.ctrl)
                self.tweak_ctrls.append(tweak_ctrl)
                self.tweak_transforms.append(tweak_ctrl.top)
                self.joint_drivers.append(tweak_ctrl.ctrl)

    def output_rig(self):
        # Get spine length
        start: om2.MPoint = om2.MPoint(mc.xform(self.base_guide, q=True, ws=True, t=True))
        end: om2.MPoint = om2.MPoint(mc.xform(self.upper_chest_pivot_guide, q=True, ws=True, t=True))
        length: float = start.distanceTo(end)

        # Create the transforms to drive the actual spine curve/spline (two control points for each control, start mid and end)
        spine_start_driver: str = mc.spaceLocator(name=f"{self.part}_startDriver")[0]
        mc.parent(spine_start_driver, self.module_grp)
        rXform.match_pose(spine_start_driver, self.base_guide, rotate=self.base_guide)
        rXform.matrix_constraint(self.hip_ctrl.ctrl, spine_start_driver, keep_offset=True)
        spine_start_tangent = mc.spaceLocator(name=f"{self.part}_startTangent")[0]
        mc.parent(spine_start_tangent, spine_start_driver)
        rXform.match_pose(
            node=spine_start_tangent, translate=spine_start_driver, rotate=spine_start_driver
        )
        mc.move(0, self.base_tangent * length, 0, spine_start_tangent, objectSpace=True)

        spine_mid_driver: str = mc.group(
            empty=True, parent=self.module_grp, name=f"{self.part}_midPointDriver"
        )
        rXform.matrix_constraint(self.spine_mid_ctrl.ctrl, spine_mid_driver, keep_offset=False)
        spine_mid_tangent1: str = mc.spaceLocator(name=f"{self.part}_midTangent1")[0]
        mc.parent(spine_mid_tangent1, spine_mid_driver)
        rXform.match_pose(
            node=spine_mid_tangent1, translate=spine_mid_driver, rotate=spine_mid_driver
        )
        mc.move(0, -self.mid_tangent * 0.5 * length, 0, spine_mid_tangent1, objectSpace=True)

        spine_mid_tangent2: str = mc.spaceLocator(name=f"{self.part}_midTangent2")[0]
        mc.parent(spine_mid_tangent2, spine_mid_driver)
        rXform.match_pose(
            node=spine_mid_tangent2, translate=spine_mid_driver, rotate=spine_mid_driver
        )
        mc.move(0, self.mid_tangent * 0.5 * length, 0, spine_mid_tangent2, objectSpace=True)

        spine_end_driver: str = mc.spaceLocator(name=f"{self.part}_endDriver")[0]
        mc.parent(spine_end_driver, self.module_grp)
        rXform.matrix_constraint(self.chest_top_ctrl.ctrl, spine_end_driver, keep_offset=False)
        spine_end_tangent: str = mc.spaceLocator(name=f"{self.part}_endTangent")[0]
        mc.parent(spine_end_tangent, spine_end_driver)
        rXform.match_pose(
            node=spine_end_tangent, translate=spine_end_driver, rotate=spine_end_driver
        )
        mc.move(0, -self.end_tangent * length, 0, spine_end_tangent, objectSpace=True)

        spline.matrix_spline_from_transforms(
            name=f"{self.part}_Spline",
            transforms=[
                spine_start_driver,
                spine_mid_tangent1,
                spine_mid_tangent2,
                spine_end_tangent,
                spine_end_driver,
            ],
            transforms_to_pin=self.tweak_transforms,
            padded=False,
            stretch=False,
            parent=self.module_grp,
            primary_axis=(0, 1, 0),
            secondary_axis=(1, 0, 0),
            degree=3,
        )

    def skeleton(self):
        cog_chain = rChain.Chain(
            transform_list=[self.joint_drivers[0]], side=self.side, suffix="JNT", name="COG",
        )
        cog_chain.create_from_transforms(parent=self.skel, pad=False)
        spine_chain = rChain.Chain(
            transform_list=self.joint_drivers[1:], side=self.side, suffix="JNT", name=self.part
        )
        spine_chain.create_from_transforms(parent=self.skel)
        mc.parent(spine_chain.joints[0], cog_chain.joints[0], relative=True)
        self.bind_joints = cog_chain.joints + spine_chain.joints
        self.tag_bind_joints(self.bind_joints)

    def compatibility_transforms(self) -> None:
        # These are needed until we refactor the modules to actually be modular instead of having hardcoded connections between each other.

        # Arm, Clavicle, and Neck dependencies:
        chest_01 = mc.group(name="chest_M_01_CTRL", empty=True, parent=self.module_grp)
        rXform.matrix_constraint(self.ik_chest_ctrl.ctrl, chest_01)
        chest_02 = mc.group(name="chest_M_02_CTRL", empty=True, parent=self.module_grp)
        rXform.matrix_constraint(self.chest_top_ctrl.ctrl, chest_02)
        chest_02_jnt = mc.group(name="chest_M_02_JNT", empty=True, parent=self.module_grp)
        rXform.matrix_constraint(self.chest_top_ctrl.ctrl, chest_02_jnt)

        # Waist Dependencies
        waist = mc.group(name="waist_M_CTRL", empty=True, parent=self.module_grp)
        rXform.matrix_constraint(self.hip_ctrl.ctrl, waist)

        # Rename Joint for Skinning and Parenting.
        mc.rename(self.bind_joints[-1], "chest_M_JNT")
        self.bind_joints[-1] = "chest_M_JNT"
        pass

    def add_plugs(self):
        if self.part == "spine":
            # add skeleton plugs
            rAttr.Attribute(
                node=self.part_grp,
                type="plug",
                value=["root_M_JNT"],
                name="skeletonPlugs",
                children_name=[self.bind_joints[0]],
            )

            # add parentConstraint rig plugs
            driver_list = ["COG_M_CTRL", "COG_M_CTRL"]
            driven_list = [self.control_grp, self.module_grp]
            rAttr.Attribute(
                node=self.part_grp,
                type="plug",
                value=driver_list,
                name="pacRigPlugs",
                children_name=driven_list,
            )
