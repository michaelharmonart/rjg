from importlib import reload
from math import radians

import maya.cmds as mc
import rjg.build.chain as rChain
import rjg.build.fk as rFk
import rjg.build.ik as rIk
import rjg.build.rigModule as rModule
import rjg.libs.attribute as rAttr
from maya.api.OpenMaya import MEulerRotation, MMatrix, MSpace, MTransformationMatrix
from rjg.libs.control.ctrl import Control
from rjg.libs.maya_api import node
from rjg.libs.space import space_switch
from rjg.libs.transform import (
    drive_transform_with_matrix,
    get_matrix_values,
    get_parent_inverse_matrix,
    get_parent_matrix,
    get_world_matrix,
    is_identity_matrix,
    match_pose,
    match_transform,
    matrix_constraint,
    set_local_matrix,
)

reload(rModule)
reload(rChain)
reload(rFk)
reload(rIk)
reload(rAttr)


class BipedLimb(rModule.RigModule, rIk.Ik, rFk.Fk):
    def __init__(
        self,
        side: str = None,
        part: str = None,
        guide_list: list[str] = None,
        ctrl_scale: float = 1,
        create_ik: bool = True,
        create_fk: bool = True,
        stretchy: bool = True,
        twisty: bool = True,
        bendy: bool = True,
        segments: int = 4,
        sticky=None,
        solver=None,
        pv_guide="auto",
        offset_pv=0,
        slide_pv=None,
        gimbal=True,
        offset=True,
        pad="auto",
        fk_shape="circle",
        gimbal_shape="circle",
        offset_shape="square",
        model_path=None,
        guide_path=None,
        spinejnt_count=4,
        swing: bool = False,
        swing_parent: str | None = None,
        independent_swing: bool = False,
        independent_swing_parent: str | None = None,
        independent_swing_connection_target: str | None = None,
        orient_spaces: dict[str, str] | None = None,
        remove_first_joint_twist: bool = False,
        twist_distribute_name: str | None = None,
    ):
        super().__init__(side=side, part=part, guide_list=guide_list, ctrl_scale=ctrl_scale, model_path=model_path, guide_path=guide_path)
        self.create_ik = create_ik
        self.create_fk = create_fk
        self.stretchy = stretchy
        self.twisty = twisty
        self.bendy = bendy
        self.segments = segments
        self.sticky = sticky
        self.solver = solver
        self.pv_guide = pv_guide
        self.offset_pv = offset_pv
        self.slide_pv = slide_pv
        self.gimbal = gimbal
        self.offset = offset
        self.pad = pad
        self.fk_shape = fk_shape
        self.gimbal_shape = gimbal_shape
        self.offset_shape = offset_shape
        self.spinejnt_count = spinejnt_count

        self.swing = swing
        self.swing_parent = swing_parent
        self.independent_swing = independent_swing
        self.independent_swing_parent = independent_swing_parent
        self.independent_swing_connection_target = independent_swing_connection_target
        self.orient_spaces = orient_spaces
        self.remove_first_joint_twist = remove_first_joint_twist
        if twist_distribute_name:
            self.twist_distribute_name = twist_distribute_name
        else:
            self.twist_distribute_name = ""
        if self.independent_swing:
            if self.independent_swing_parent is None:
                self.independent_swing = False
                raise ValueError(
                    f"""{self.part} has no independent_swing_parent! output_swing requires a space to be relative to
                    (in order to avoid cyclical dependencies.)
                    The space should be something like the chest for the arm."""
                )
            if self.swing_parent is None:
                self.swing_parent = independent_swing_parent
        self.independent_swing_output = None
        if self.swing:
            if self.swing_parent is None:
                self.swing = False
                raise ValueError(
                    f"""{self.part} has no swing_parent! output_swing requires a space to be relative to
                    (in order to avoid cyclical dependencies.)
                    The space should be something like the chest for the arm."""
                        )
        self.swing_output = None
                        
        if self.twisty or self.bendy and not self.segments:
            self.segments = 4

        if self.pad == "auto":
            self.pad = len(str(len(self.guide_list))) + 1

        self.create_module()

    def create_module(self):
        super(BipedLimb, self).create_module()

        self.check_solvers()
        self.check_pv_guide()

        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.create_orient_spaces()
        self.add_plugs()

    def create_inputs(self, group: str) -> None:
        self.input_group = mc.group(empty=True, name=f"{self.base_name}_INPUTS", parent=group)
        self.orient_input = mc.group(
            empty=True, name=f"{self.base_name}_Orient_IN", parent=self.input_group
        )

    def control_rig(self):
        # fk
        if self.create_fk:
            self.build_fk_controls()
            mc.parent(self.fk_ctrls[0].top, self.control_grp)

        # ik
        if self.create_ik:
            self.pv_control = self.build_ik_controls()
            mc.parent(self.ik_ctrl_grp, self.control_grp)

    def output_rig(self):
        self.limb_grp = mc.group(em=True, parent=self.module_grp, name=self.base_name + "_RIG_GRP")
        mc.matchTransform(self.limb_grp, self.guide_list[0])

        self.create_inputs(group=self.limb_grp)
        if self.side == 'R':
            self.mirror = True
        else:
            self.mirror = False
            
        # fk
        if self.create_fk:
            self.build_fk_chain()
            mc.parent(self.fk_joints[0], self.limb_grp)
            self.src_chain = self.fk_chain
            self.src_joints = self.fk_joints
            up_twist = self.fk_ctrls[0].ctrl
            lo_twist = self.fk_ctrls[-1].ctrl

        # ik
        if self.create_ik:
            self.build_ik_chain(force_planar=True)
            self.build_ikh(scale_attr=self.global_scale)
            mc.parent(self.ikh, self.ik_joints[0], self.limb_grp)
            self.src_chain = self.ik_chain
            self.src_joints = self.ik_joints
            up_twist = self.base_ctrl.ctrl
            lo_twist = self.main_ctrl.ctrl

        if self.create_ik and self.create_fk:
            blend_chain = rChain.Chain(transform_list=self.src_joints,
                                        side=self.side,
                                        suffix='switch_JNT',
                                        name=self.part)

            blend_chain.create_blend_chain(
                switch_node=self.base_name,
                chain_a=self.fk_joints,
                chain_b=self.ik_joints,
                handle_offsets=True,
            )
            mc.parent(blend_chain.joints[0], self.limb_grp)
            self.src_chain = blend_chain
            self.src_joints = blend_chain.joints
            self.ik_switch_attr: str = blend_chain.switch.attr

            # twist
            up_twist = mc.spaceLocator(name=self.base_name + '_up_twist_LOC')[0]
            lo_twist = mc.spaceLocator(name=self.base_name + '_lo_twist_LOC')[0]
            mc.matchTransform(up_twist, self.guide_list[0])
            mc.matchTransform(lo_twist, self.guide_list[-1])

            rev = mc.createNode('reverse', name=self.base_name + '_REV')
            pac = mc.parentConstraint(self.fk_ctrls[-1].ctrl,
                                        self.main_ctrl.ctrl,
                                        lo_twist, maintainOffset=True)[0]
            wal = mc.parentConstraint(pac, query=True, weightAliasList=True)
            mc.setAttr(pac + '.interpType', 2)
            mc.connectAttr(blend_chain.switch.attr, rev + '.inputY')
            mc.connectAttr(rev + '.outputY', pac + '.' + wal[1])
            mc.connectAttr(blend_chain.switch.attr, pac + '.' + wal[0])
            mc.parent(lo_twist, up_twist, self.limb_grp)
            mc.hide(lo_twist, up_twist)

            # vis switch
            mc.connectAttr(blend_chain.switch.attr, rev + '.inputZ')
            mc.connectAttr(blend_chain.switch.attr,
                             self.fk_ctrls[0].top + '.visibility')
            mc.connectAttr(rev + '.outputZ', self.ik_ctrl_grp + '.visibility')



        if self.segments:
            self.src_chain.split_chain(segments=self.segments)
            self.src_joints = []
            for jnt in self.src_chain.joints[:-1]:
                split_list = self.src_chain.split_jnt_dict[jnt]
                for s_jnt in split_list:
                    self.src_joints.append(s_jnt)
            self.src_joints.append(self.src_chain.joints[-1])
        
        if self.swing:
            self.output_swing()
            
        if self.bendy:
            if self.remove_first_joint_twist:
                twist_distribute_attr_name = f"{self.twist_distribute_name}TwistDistribute"
                twist_distribute = rAttr.Attribute(node=self.limb_grp, type="double", min=0, max=1, keyable=True, name=twist_distribute_attr_name, value=1)
                
                swing_only = mc.group(empty=True, name=f"{self.base_name}_SwingOnly", parent=self.limb_grp)
                matrix_constraint(self.swing_output, swing_only, keep_offset=False, scale=False)
                matrix_constraint(self.src_chain.joints[0], swing_only, keep_offset=False, translate=False, scale=True, rotate=False, shear=False)
                swing_twist = mc.group(empty=True, name=f"{self.base_name}_SwingTwist", parent=self.limb_grp)
                matrix_constraint(self.src_chain.joints[0], swing_twist, keep_offset=False)
                blend_node = node.BlendMatrixNode(name=f"{self.base_name}_TwistBlend")
                mc.connectAttr(f"{swing_twist}.matrix", blend_node.input_matrix)
                mc.connectAttr(f"{swing_only}.matrix", blend_node.target[0].target_matrix)
                mc.connectAttr(twist_distribute.attr, blend_node.target[0].weight)
                for control in [self.fk_ctrls[0].ctrl, self.base_ctrl.ctrl]:
                    mc.addAttr(control, longName=twist_distribute_attr_name, proxy=twist_distribute.attr)
                blend_transform = mc.group(empty=True, name=f"{self.base_name}_TwistBlendTransform", parent=self.limb_grp)
                drive_transform_with_matrix(blend_node.output_matrix,blend_transform)

                bend = self.src_chain.bend_twist_chain(
                    ctrl_scale=self.ctrl_scale,
                    mirror=self.mirror,
                    global_scale=self.global_scale.attr,
                    first_joint_space=blend_transform,
                )
            else:
                bend = self.src_chain.bend_twist_chain(
                    ctrl_scale=self.ctrl_scale,
                    mirror=self.mirror,
                    global_scale=self.global_scale.attr,
                )

            mc.parent(bend["control"], self.control_grp)
            mc.parent(bend["module"], self.module_grp)

        if self.independent_swing:
            self.output_independent_swing()

    def create_orient_spaces(self):
        if self.orient_spaces is not None:
            targets = []
            names = []
            for name, target in self.orient_spaces.items():
                names.append(name)
                targets.append(target)

            orient_control: Control = self.fk_ctrls[0]
            space_switch(
                node=self.orient_input,
                driver=orient_control.ctrl,
                target_list=targets,
                name_list=names,
                name="orientSpace",
                constraint_type="orient",
                value=3,
            )
            mc.orientConstraint(self.orient_input, orient_control.top, maintainOffset=True)
            
    def output_swing(self):
        """Outputs a simple swing transform for use with bendbow twist, etc. (not to be used for clavicle as it has dependencies on that)"""
        swing_group = mc.group(empty=True, name=f"{self.base_name}_Swing", parent=self.limb_grp)
        anchor_group = mc.group(empty=True, name=f"{self.base_name}_Anchor", parent=swing_group)
        match_transform(swing_group, self.fk_joints[0])
        matrix_constraint(self.swing_parent, swing_group, keep_offset=True)
        matrix_constraint(self.fk_ctrls[0].ctrl, anchor_group, rotate=False, scale=False, shear=False)
        
        self.swing_output = mc.group(empty=True, name=f"{self.base_name}_Swing_OUT", parent=anchor_group)
        mc.aimConstraint(self.src_chain.joints[1], self.swing_output, aimVector=(0, 1 if not self.mirror else -1, 0), upVector=(0,0,0), worldUpType=4, maintainOffset=False)
        self.twist_driver_output = mc.joint(name=f"{self.base_name}_TwistDriver")
        mc.parent(self.twist_driver_output, self.swing_output, relative=True)
        rotation_matrix: MMatrix = MEulerRotation(radians(-90),0,0, MSpace.kTransform).asMatrix()
        set_local_matrix(self.twist_driver_output, rotation_matrix)
        print(rotation_matrix)
        matrix_constraint(self.src_chain.joints[0], self.twist_driver_output, translate=False, scale=False, shear=False)
        pass
        
    def output_independent_swing(self):
        swing_group = mc.group(empty=True, name=f"{self.base_name}_IndependentSwing", parent=self.limb_grp)
        anchor_group = mc.group(empty=True, name=f"{self.base_name}_IndependentAnchor", parent=swing_group)
        match_transform(anchor_group, self.fk_joints[0])
        matrix_constraint(self.independent_swing_parent, anchor_group)

        orient_offset = mc.group(
            empty=True, name=f"{self.base_name}_OrientOffset", parent=anchor_group
        )
        mc.orientConstraint(self.orient_input, orient_offset, maintainOffset=True)

        parent = anchor_group
        swing_joints: list[str] = []
        for i, joint in enumerate(self.ik_joints):
            swing_joint: str = mc.joint(name=f"{self.base_name}_IndependentSwing_{i:02d}")
            mc.parent(swing_joint, parent)
            parent = swing_joint
            match_pose(swing_joint, translate=joint, rotate=joint)
            match_transform(swing_joint, joint)
            swing_joints.append(swing_joint)
        first_joint = swing_joints[0]

        if self.create_fk:
            # Set up FK
            i = 0
            driver_matrix = mc.createNode("multMatrix", name=f"{first_joint}_Matrix")
            offset_matrix =  get_world_matrix(first_joint) * get_world_matrix(self.fk_ctrls[0].ctrl).inverse()
            if not is_identity_matrix(offset_matrix):
                mc.setAttr(f"{driver_matrix}.matrixIn[{i}]", offset_matrix, type="matrix")
                i += 1
            mc.connectAttr(f"{self.fk_ctrls[0].ctrl}.matrix", f"{driver_matrix}.matrixIn[{i}]")
            i += 1
            parent_offset_matrix = get_parent_matrix(self.fk_ctrls[0].ctrl) * get_parent_inverse_matrix(first_joint)
            if not is_identity_matrix(parent_offset_matrix):
                mc.setAttr(f"{driver_matrix}.matrixIn[{i}]", parent_offset_matrix, type="matrix")
                i += 1
            mc.connectAttr(f"{orient_offset}.matrix", f"{driver_matrix}.matrixIn[{i}]")
            i += 1
            drive_transform_with_matrix(f"{driver_matrix}.matrixSum", first_joint)
        if self.create_ik:
            # Set up IK
            swing_ik_handle: str = mc.ikHandle(
                name=f"{self.base_name}_IndpendentSwing_IK",
                startJoint=swing_joints[0],
                endEffector=swing_joints[-1],
                sticky=self.sticky,
                solver=self.solver,
            )[0]
            mc.parent(swing_ik_handle, anchor_group)
            mc.poleVectorConstraint(self.pv_ctrl.ctrl, swing_ik_handle)
            matrix_constraint(self.main_ctrl.ctrl, swing_ik_handle, keep_offset=False)
            invert = mc.createNode("subtract", name=f"{swing_ik_handle}_ikBlend_Invert")
            mc.setAttr(f"{invert}.input1", 1)
            mc.connectAttr(self.ik_switch_attr, f"{invert}.input2")
            mc.connectAttr(f"{invert}.output", f"{swing_ik_handle}.ikBlend")

        # Swing output
        self.independent_swing_output = mc.group(empty=True, name=f"{self.base_name}_IndependentSwing_OUT", parent=anchor_group)
        mc.aimConstraint(swing_joints[1], self.independent_swing_output, aimVector=(0, 1 if not self.mirror else -1, 0), upVector=(0,0,0), worldUpType=4, maintainOffset=False)
        
        # Connect swing
        matrix_constraint(
            self.independent_swing_output,
            self.independent_swing_connection_target,
            translate=False,
            shear=False,
            scale=False,
        )

    def skeleton(self):
        limb_chain = rChain.Chain(transform_list=self.src_joints,
                                   side=self.side,
                                   suffix='JNT',
                                   name=self.part)
        if self.create_fk:
            poc = True
        else:
            poc = False
        limb_chain.create_from_transforms(orient_constraint=True,
                                          point_constraint=poc,
                                          parent=self.skel)
        self.bind_joints = limb_chain.joints
        self.tag_bind_joints(self.bind_joints[:-1])

    def add_plugs(self):
        #print(self.pv_control)
        if self.part == 'leg':
            par = 'COG_M_JNT'
            driver_list = ['waist_M_CTRL',
                           'waist_M_CTRL',
                           #'waist_M_CTRL',
                           self.base_name + '_IK_BASE_CTRL',
                           'foot_' + self.side + '_01_ik_JNT',
                           'root_02_M_CTRL']
            driven_list = [self.limb_grp,
                           self.base_name + '_IK_BASE_CTRL_CNST_GRP',
                           #self.base_name + '_01_fk_CTRL_CNST_GRP',
                           self.base_name + '_up_twist_LOC',
                           self.base_name + '_IK_MAIN_CTRL_CNST_GRP',
                           self.pv_control + '_CNST_GRP']
            hide_list = [
                         self.base_name + '_IK_MAIN_CTRL_CNST_GRP',
                         #self.base_name + '_IK_BASE_CTRL_CNST_GRP',
                         self.fk_ctrls[-1].top
                        ]
            pv_targets = ['ROOT',
                          'global_M_CTRL',
                          'root_02_M_CTRL',
                          'COG_M_CTRL',
                          #'leg_' + self.side + '_IK_BASE_CTRL',
                          'foot_' + self.side + '_02_' + self.side + '_CTRL',
                          '4']
            pv_names = ['world', 'global', 'root', 'hip', 'foot', 'default_value']
            ik_ctrl = ['foot_' + self.side + '_01_' + self.side + '_CTRL']

            target_list = ['ROOT', 'global_M_CTRL', 'root_02_M_CTRL', 'COG_M_CTRL', 'waist_M_CTRL', '4']
            name_list = ['world', 'global', 'root', 'COG', 'waist', 'default_value']
            orient_names = ['point' + name.title() for name in name_list]
            rAttr.Attribute(node=self.part_grp, type='plug', value=target_list, name=self.fk_ctrls[0].ctrl + '_point', children_name=orient_names)

            target_list = ['ROOT', 'global_M_CTRL', 'root_02_M_CTRL', 'COG_M_CTRL', 'waist_M_CTRL', '4']
            name_list = ['world', 'global', 'root', 'COG', 'waist', 'default_value']
            orient_names = ['orient' + name.title() for name in name_list]
            if self.orient_spaces is None:
                rAttr.Attribute(
                    node=self.part_grp,
                    type="plug",
                    value=target_list,
                    name=self.fk_ctrls[0].ctrl + "_orient",
                    children_name=orient_names,
                )
        elif self.part == 'arm':
            par = 'clavicle_' + self.side + '_02_JNT'
            driver_list = [
                           'clavicle_' + self.side + '_02_driver_JNT',
                           'clavicle_' + self.side + '_02_driver_JNT',
                           'hand_' + self.side + '_01_ik_JNT',
                           'root_02_M_CTRL']
            driven_list = [
                           self.base_name + '_IK_BASE_CTRL_CNST_GRP',
                           self.base_name + '_up_twist_LOC',
                           self.base_name + '_IK_MAIN_CTRL_CNST_GRP',
                           self.pv_control + '_CNST_GRP']
            hide_list = [self.base_name + '_IK_MAIN_CTRL_CNST_GRP',
                         #self.base_name + '_IK_BASE_CTRL_CNST_GRP'
                         ]
            #hide_list = None
            pv_targets = ['ROOT',
                          'global_M_CTRL',
                          'root_02_M_CTRL',
                          'chest_M_01_CTRL',
                          'hand_' + self.side + '_local_CTRL',
                          '2']
            pv_names = ['world', 'global', 'root', 'chest', 'hand', 'default_value']
            ik_ctrl = ['hand_' + self.side + '_01_CTRL']

            rAttr.Attribute(node=self.part_grp, type='plug', value=['clavicle_' + self.side + '_02_driver_JNT'], name='pocRigPlugs', children_name=['arm_' + self.side + '_01_fk_CTRL_CNST_GRP'])
            target_list = ['ROOT', 'global_M_CTRL', 'root_02_M_CTRL', 'chest_M_01_CTRL', 'chest_M_02_CTRL', 'clavicle_' + self.side + '_02_driver_JNT', '3']
            name_list = ['world', 'global', 'root', 'chest01', 'chest02', 'clavicle', 'default_value']
            orient_names = ['orient' + name.title() for name in name_list]
            if self.orient_spaces is None:
                rAttr.Attribute(
                    node=self.part_grp,
                    type="plug",
                    value=target_list,
                    name=self.fk_ctrls[0].ctrl + "_orient",
                    children_name=orient_names,
                )
        elif 'finger' in self.part:
            #print("plugging finger!")
            par = 'hand_' + self.side + '_JNT'
            driver_list = ['hand_' + self.side + '_JNT',
                           'hand_' + self.side + '_JNT',
                           'hand_' + self.side + '_JNT',
                           'root_02_M_CTRL',
                           'root_02_M_CTRL']
            driven_list = [self.limb_grp,
                           self.base_name + '_IK_BASE_CTRL_CNST_GRP',
                           self.base_name + '_01_fk_CTRL_CNST_GRP',
                           self.pv_control + '_CNST_GRP',
                           self.base_name + '_IK_MAIN_CTRL_CNST_GRP']
            hide_list = [self.base_name + '_IK_BASE_CTRL_CNST_GRP']
            pv_targets = ['ROOT',
                          'global_M_CTRL',
                          'root_02_M_CTRL',
                          'chest_M_01_CTRL',
                          'hand_' + self.side + '_local_CTRL',
                          '2']
            pv_names = ['world', 'global', 'root', 'hip', 'hand', 'default_value']
            ik_ctrl = None
        elif self.part == 'neck':

            rAttr.Attribute(node=self.part_grp, type='plug', value=['chest_M_JNT'], name='skeletonPlugs', children_name=[self.bind_joints[0]])

            driver_list = ['chest_M_02_JNT', 'head_M_02_CTRL']
            driven_list = [self.base_name + '_base_CTRL_CNST_GRP', self.base_name + '_tip_CTRL_CNST_GRP']

            driver_list = ['chest_M_02_JNT']#, 'head_M_02_CTRL']
            driven_list = ['neck_M_01_fk_CTRL_CNST_GRP']#, 'neck_M_03_fk_CTRL_CNST_GRP']
            rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)

            hide_list = [self.base_name + '_tip_CTRL_CNST_GRP', self.base_name + '_base_CTRL_CNST_GRP', f'neck_0{self.spinejnt_count - 1}_FK_M_CTRL_CNST_GRP']
            rAttr.Attribute(node=self.part_grp, type='plug', value=[' '.join(hide_list)], name='hideRigPlugs', children_name=['hideNodes'])

            mc.hide(f'neck_M_0{self.spinejnt_count - 1}_fk_CTRL_CNST_GRP')
            if self.create_ik:
                if mc.objExists('switch_CTRL'):
                    mc.addAttr('switch_CTRL', longName='Neck_M_IKFK', attributeType='bool', keyable=True)
                    mc.connectAttr('switch_CTRL.Neck_M_IKFK', 'neck_M.switch', force=True)
                for obj in ['chest_M_02_CTRL', 'neck_M_IK_CTRL_GRP','neck_M_IK_BASE_CTRL', 'neck_M_IK_MAIN_CTRL_CNST_GRP']:
                    print(obj, mc.objExists(obj))
                mc.parentConstraint('chest_M_02_CTRL', 'neck_M_IK_CTRL_GRP', mo=True)
                mc.parentConstraint('neck_M_IK_BASE_CTRL', 'neck_M_IK_MAIN_CTRL_CNST_GRP', mo=True)
            return
        elif self.part == 'fin':
            rAttr.Attribute(node=self.part_grp, type='plug', value=['chest_M_JNT'], name='skeletonPlugs', children_name=[self.bind_joints[0]])
            par = 'neck_M_02_JNT'
            driver_list = ['neck_02_FK_M_CTRL']
            driven_list = ['fin_M_01_fk_CTRL_CNST_GRP']
            hide_list = [self.base_name + '_tip_CTRL_CNST_GRP', self.base_name + '_base_CTRL_CNST_GRP', f'fin_0{self.spinejnt_count - 1}_FK_M_CTRL_CNST_GRP']
            rAttr.Attribute(node=self.part_grp, type='plug', value=[' '.join(hide_list)], name='hideRigPlugs', children_name=['hideNodes'])

        else:
            par = 'insert limb plug here'
            driver_list = ['driver list']
            driven_list = ['driven list']
            hide_list = ['hide list']
            ik_ctrl = ['ik ctrl']

        switch_attr = self.part.lower() + self.side.capitalize() + '_IKFK'
        switch_ctrls: list[str] = [ctrl.ctrl_name for ctrl in self.fk_ctrls] + [ctrl.ctrl_name for ctrl in self.ik_ctrls]
        rAttr.Attribute(node=self.part_grp, type='plug', value=[par], name='skeletonPlugs', children_name=[self.bind_joints[0]])
        rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)
        rAttr.Attribute(node=self.part_grp, type='plug', value=[' '.join(hide_list)], name='hideRigPlugs', children_name=['hideNodes']) if hide_list else None
        rAttr.Attribute(node=self.part_grp, type='plug', value=pv_targets, name=self.pv_ctrl.ctrl + '_parent', children_name=pv_names)
        rAttr.Attribute(node=self.part_grp, type='plug', value=[switch_attr, str(switch_ctrls)], name='switchRigPlugs', children_name=['ikFkSwitch', 'ikFKSwitchControls'])
        rAttr.Attribute(node=self.part_grp, type='plug', value=ik_ctrl, name='transferAttributes', children_name=[self.main_ctrl.ctrl]) if ik_ctrl else None
