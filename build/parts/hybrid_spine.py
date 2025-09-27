from rjg.libs.control.ctrl import Control


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
        joint_num: int = 4,
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
        )

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
        )

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
        )

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
        )

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
        )

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


        print(base_ctrl.ctrl)
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

    def output_rig(self):
        # build spline joints
        self.build_spline_chain(scale_attr=self.global_scale)

        # build joints to bind the curve to
        c_jnt_grp = mc.group(
            empty=True, parent=self.module_grp, name=self.base_name + "_curve_bind_JNT_GRP"
        )
        base_jnt = mc.joint(c_jnt_grp, name=self.base_ctrl.ctrl.replace("CTRL", "JNT"))
        tip_jnt = mc.joint(c_jnt_grp, name=self.tip_ctrl.ctrl.replace("CTRL", "JNT"))

        mc.matchTransform(base_jnt, self.spline_joints[0])
        mc.matchTransform(tip_jnt, self.spline_joints[-1])
        mc.parentConstraint(self.base_driver, base_jnt, maintainOffset=True)
        mc.parentConstraint(self.tip_driver, tip_jnt, maintainOffset=True)
        print(self.fk_ctrl_list[-1].ctrl)
        mc.parentConstraint(self.fk_ctrl_list[-1].ctrl, self.tip_ctrl.top, maintainOffset=True)

        if self.mid_ctrl:
            blend = rAttr.Attribute(
                node=self.mid_01_ctrl.ctrl,
                type="double",
                value=0,
                min=0,
                max=1,
                keyable=True,
                name="blendBetween",
            )
            mid_jnt = mc.joint(c_jnt_grp, name=self.mid_01_ctrl.ctrl.replace("CTRL", "JNT"))
            mc.parentConstraint(self.mid_02_ctrl.ctrl, mid_jnt, maintainOffset=False)

            # blend locator between start and end
            mid_loc = mc.spaceLocator(name=mid_jnt.replace("JNT", "LOC"))[0]
            mc.matchTransform(mid_loc, mid_jnt)
            mc.pointConstraint(base_jnt, tip_jnt, mid_loc, maintainOffset=True)
            aim = mc.aimConstraint(
                tip_jnt,
                mid_loc,
                aimVector=(0, 1, 0),
                upVector=(0, 0, 1),
                worldUpType="vector",
                worldUpVector=(0, 0, 1),
                maintainOffset=True,
            )[0]
            b_vp = mc.createNode("vectorProduct", name=base_jnt.replace("JNT", "VP"))
            t_vp = mc.createNode("vectorProduct", name=tip_jnt.replace("JNT", "VP"))
            pma = mc.createNode("plusMinusAverage", name=mid_jnt.replace("JNT", "PMA"))
            rev = mc.createNode("reverse", name=mid_jnt.replace("JNT", "REV"))
            mc.connectAttr(base_jnt + ".worldMatrix", b_vp + ".matrix")
            mc.connectAttr(tip_jnt + ".worldMatrix", t_vp + ".matrix")
            mc.connectAttr(b_vp + ".output", pma + ".input3D[0]")
            mc.connectAttr(t_vp + ".output", pma + ".input3D[1]")
            mc.connectAttr(pma + ".output3D", aim + ".worldUpVector")
            mc.setAttr(b_vp + ".input1Z", 1)
            mc.setAttr(t_vp + ".input1Z", 1)
            mc.setAttr(b_vp + ".operation", 3)
            mc.setAttr(t_vp + ".operation", 3)
            mc.setAttr(pma + ".operation", 3)

            pac = mc.parentConstraint(
                self.fk_ctrl_list[1].ctrl, mid_loc, self.mid_01_ctrl.top, maintainOffset=True
            )[0]
            wal = mc.parentConstraint(pac, query=True, weightAliasList=True)
            mc.connectAttr(blend.attr, rev + ".inputX")
            mc.connectAttr(rev + ".outputX", pac + "." + wal[0])
            mc.connectAttr(blend.attr, pac + "." + wal[1])

            mc.parent(mid_loc, self.loc_grp)
        mc.parent(self.loc_grp, self.module_grp)

        # bind curve to control joint
        bind_list = mc.listRelatives(c_jnt_grp) + [self.curve]
        mc.skinCluster(bind_list, toSelectedBones=True, name=self.curve.replace("CRV", "SKC"))

        # build spline ik handle
        self.build_spline_ikh()
        ikh_grp = mc.group(
            self.spline_ikh,
            self.curve,
            parent=self.module_grp,
            name=self.base_name + "_spline_IKH_GRP",
        )
        mc.setAttr(ikh_grp + ".inheritsTransform", 0)
        mc.group(
            self.spline_joints[0], parent=self.module_grp, name=self.base_name + "_driver_JNT_GRP"
        )

        # setup advanced twist
        mc.setAttr(self.spline_ikh + ".dTwistControlEnable", 1)
        mc.setAttr(self.spline_ikh + ".dWorldUpType", 4)
        mc.setAttr(self.spline_ikh + ".dForwardAxis", 2)
        mc.setAttr(self.spline_ikh + ".dWorldUpAxis", 3)
        mc.setAttr(self.spline_ikh + ".dTwistControlEnable", 1)
        mc.setAttr(self.spline_ikh + ".dWorldUpVector", 0, 0, 1)
        mc.setAttr(self.spline_ikh + ".dWorldUpVectorEnd", 0, 0, 1)
        mc.connectAttr(base_jnt + ".worldMatrix[0]", self.spline_ikh + ".dWorldUpMatrix")
        mc.connectAttr(tip_jnt + ".worldMatrix[0]", self.spline_ikh + ".dWorldUpMatrixEnd")

        if self.mid_ctrl:
            self.switch = rAttr.Attribute(
                node=self.part_grp, type="double", min=0, max=1, keyable=True, name="switch"
            )
            sw_rev = mc.createNode("reverse", name=self.base_name + "_SW_REV")
            mc.connectAttr(self.switch.attr, sw_rev + ".inputX")
            mc.connectAttr(sw_rev + ".outputX", self.tip_ctrl.ctrl + ".stretch")
            mc.connectAttr(sw_rev + ".outputX", self.mid_01_ctrl.ctrl + ".v")
            mc.connectAttr(sw_rev + ".outputX", self.mid_01_ctrl.ctrl + ".blendBetween")
            mc.connectAttr(self.switch.attr, self.fk_ctrl_list[0].ctrl + ".v")

    def skeleton(self):
        spine_chain = rChain.Chain(
            transform_list=self.spline_joints, side=self.side, suffix="JNT", name=self.part
        )
        spine_chain.create_from_transforms(parent=self.skel)
        self.bind_joints = spine_chain.joints

        self.tag_bind_joints(self.bind_joints[:-1])

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
            driver_list = ["waist_M_CTRL", "COG_M_CTRL", "chest_M_02_CTRL"]
            driven_list = [
                self.base_name + "_base_CTRL_CNST_GRP",
                self.base_name[:-2] + "_01_FK_M_CTRL_CNST_GRP",
                self.base_name + "_tip_CTRL_CNST_GRP",
            ]
            rAttr.Attribute(
                node=self.part_grp,
                type="plug",
                value=driver_list,
                name="pacRigPlugs",
                children_name=driven_list,
            )

        elif self.part == "neck":
            rAttr.Attribute(
                node=self.part_grp,
                type="plug",
                value=["chest_M_JNT"],
                name="skeletonPlugs",
                children_name=[self.bind_joints[0]],
            )

            driver_list = ["chest_M_02_CTRL", "chest_M_02_CTRL", "head_M_01_CTRL"]
            driven_list = [
                self.base_name + "_base_CTRL_CNST_GRP",
                self.base_name[:-2] + "_01_FK_M_CTRL_CNST_GRP",
                self.base_name + "_tip_CTRL_CNST_GRP",
            ]
            rAttr.Attribute(
                node=self.part_grp,
                type="plug",
                value=driver_list,
                name="pacRigPlugs",
                children_name=driven_list,
            )

        # add hide rig plugs
        hide_list = [self.base_name + "_base_CTRL_CNST_GRP", self.base_name + "_tip_CTRL_CNST_GRP"]
        rAttr.Attribute(
            node=self.part_grp,
            type="plug",
            value=[" ".join(hide_list)],
            name="hideRigPlugs",
            children_name=["hideNodes"],
        )

        if self.mid_ctrl:
            switch_attr = self.part.lower() + self.side.capitalize() + "_IKFK"
            rAttr.Attribute(
                node=self.part_grp,
                type="plug",
                value=[switch_attr],
                name="switchRigPlugs",
                children_name=["ikFkSwitch"],
            )
