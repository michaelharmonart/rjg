from importlib import reload
from math import radians

import maya.cmds as mc
import rjg.build.chain as rChain
import rjg.build.fk as rFk
import rjg.build.ik as rIk
import rjg.build.rigModule as rModule
import rjg.libs.attribute as rAttr
import rjg.libs.control.ctrl as rCtrl
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



class autoneck(rModule.RigModule, rIk.Ik, rFk.Fk):
    def __init__(
        self,
        side: str = None,
        part: str = None,
        guide_list: list[str] = None,
        ctrl_scale: float = 1,
        create_ik: bool = True,
        create_fk: bool = True,
        stretchy: bool = True,
        segments: int = 2,
        pad="auto",
        fk_shape="circle",
        gimbal_shape="circle",
        offset_shape="square",
        orient_spaces: dict[str, str] | None = None,
        auto=True,
        model_path=None,
        guide_path=None,
        pv_guide="auto",
        slide_pv=None,
        spinejnt_count = 4,
        split=True,
        fkauto = True
    ):
        super().__init__(side=side, part=part, guide_list=guide_list, ctrl_scale=ctrl_scale, model_path=model_path, guide_path=guide_path)
        self.create_ik = create_ik
        self.create_fk = create_fk
        self.stretchy = stretchy
        self.segments = segments
        self.pad = pad
        self.fk_shape = fk_shape
        self.gimbal_shape = gimbal_shape
        self.offset_shape = offset_shape
        self.auto = auto
        self.orient_spaces = orient_spaces
        self.sticky=False
        self.twisty=False
        self.pv_guide = pv_guide
        self.slide_pv = slide_pv
        self.offset_pv=0
        self.gimbal=True
        self.offset=True
        self.spinejnt_count = spinejnt_count
        self.fk_ctrls = []
        self.ik_ctrls = []  # always define, even if empty
        self.split=split
        self.fkauto = fkauto

        if self.pad == "auto":
            self.pad = len(str(len(self.guide_list))) + 1

        self.solver = 'ikRPsolver' if self.segments == 2 else 'ikSplineSolver'

        self.create_module()

    def create_inputs(self, group: str) -> None:
        self.input_group = mc.group(empty=True, name=f"{self.base_name}_INPUTS", parent=group)
        self.orient_input = mc.group(
            empty=True, name=f"{self.base_name}_Orient_IN", parent=self.input_group
        )

    def create_module(self):
        super().create_module()

        self.check_solvers()
        self.check_pv_guide()

        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.add_plugs()

    def control_rig(self):
        #FK
        if self.create_fk:
            self.build_fk_controls()
            mc.parent(self.fk_ctrls[0].top, self.control_grp)
            if self.fkauto:
                self.fkauto_ctrl = rCtrl.Control(parent=None, shape="ZTArrows", side=None, suffix='CTRL', name=f'Head_M', axis='y', group_type='main', rig_type='primary', translate='Head', rotate='Head')
                md = mc.createNode('multiplyDivide',name=f'AutoNeck_rotDist_md')
                distribute_value = 1 / self.segments
                mc.addAttr(self.fkauto_ctrl.ctrl, longName="Auto_Amount", at='double', dv=distribute_value, k=True)
                rev = mc.createNode('multiplyDivide',name=f'AutoNeck_rotDist_rev')
                mc.parentConstraint(self.fk_ctrls[-1].ctrl, self.fkauto_ctrl.top, mo=True)
                mc.parent(self.fkauto_ctrl.top, self.control_grp)
                for axe in ['X', 'Y', 'Z']:
                    mc.connectAttr(f'{self.fkauto_ctrl.ctrl}.rotate{axe}', f'{md}.input1{axe}')
                    mc.connectAttr(f'{self.fkauto_ctrl.ctrl}.Auto_Amount', f'{md}.input2{axe}')
                    mc.setAttr(f'{rev}.input2{axe}', -1)
                    mc.connectAttr(f'{self.fkauto_ctrl.ctrl}.rotate{axe}', f'{rev}.input1{axe}')
                    mc.connectAttr( f'{rev}.output{axe}', f'Head_M_CTRL_OFF_GRP.rotate{axe}')
                for i in range(1, self.segments + 1, 1): #neck_M_01_fk_CTRL_OFF_GRP
                    for axe in ['X', 'Y', 'Z']:
                        mc.connectAttr(f'{md}.output{axe}', f'neck_M_0{i}_fk_CTRL_OFF_GRP.rotate{axe}')
                    




        # ik
        if self.create_ik:
            if self.segments == 2:
                self.pv_control = self.build_ik_controls()
                mc.parent(self.ik_ctrl_grp, self.control_grp)
            elif self.segments > 2:
                self.build_ikspline_controls()
                mc.parent(self.ik_ctrl_grp, self.control_grp)
            else:
                print('Not enough neck segments for IK')
        

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
            if self.segments == 2:
                self.build_ik_chain(force_planar=True)
                self.build_ikh(scale_attr=self.global_scale)
                mc.parent(self.ikh, self.ik_joints[0], self.limb_grp)
                self.src_chain = self.ik_chain
                self.src_joints = self.ik_joints
                up_twist = self.base_ctrl.ctrl
                lo_twist = self.main_ctrl.ctrl
            if self.segments > 2:
                self.build_ikspline_chain()
                self.build_spline_ikh()

                mc.parent(self.ikspline_joints[0], self.limb_grp)
                mc.parent(self.ik_ctrl_grp, self.control_grp)

                self.src_chain = self.ikspline_chain
                self.src_joints = self.ikspline_joints

                # twist drivers (for advanced twist)
                up_twist = self.start_ctrl.ctrl
                lo_twist = self.end_ctrl.ctrl
            else:
                print('Not enough neck segments for IK')

        if self.create_ik and self.create_fk:
            # Determine correct IK controllers and joints based on segments
            if self.segments == 2:
                ik_ctrl_upper = self.base_ctrl.ctrl
                ik_ctrl_lower = self.main_ctrl.ctrl
                ik_chain_joints = self.ik_joints
            else:  # IK spline
                ik_ctrl_upper = self.start_ctrl.ctrl
                ik_ctrl_lower = self.end_ctrl.ctrl
                ik_chain_joints = self.ikspline_joints

            # Create the blend chain
            blend_chain = rChain.Chain(transform_list=self.src_joints,
                                        side=self.side,
                                        suffix='switch_JNT',
                                        name=self.part)

            blend_chain.create_blend_chain(
                switch_node=self.base_name,
                chain_a=self.fk_joints,
                chain_b=ik_chain_joints,
                handle_offsets=True,
            )
            mc.parent(blend_chain.joints[0], self.limb_grp)
            self.src_chain = blend_chain
            self.src_joints = blend_chain.joints
            self.ik_switch_attr: str = blend_chain.switch.attr

            # Twist locators
            up_twist = mc.spaceLocator(name=self.base_name + '_up_twist_LOC')[0]
            lo_twist = mc.spaceLocator(name=self.base_name + '_lo_twist_LOC')[0]
            mc.matchTransform(up_twist, self.guide_list[0])
            mc.matchTransform(lo_twist, self.guide_list[-1])

            # Twist parent constraint with reverse node
            rev = mc.createNode('reverse', name=self.base_name + '_REV')
            pac = mc.parentConstraint(self.fk_ctrls[-1].ctrl,
                                    ik_ctrl_lower,
                                    lo_twist, maintainOffset=True)[0]
            wal = mc.parentConstraint(pac, query=True, weightAliasList=True)
            mc.setAttr(pac + '.interpType', 2)
            mc.connectAttr(blend_chain.switch.attr, rev + '.inputY')
            mc.connectAttr(rev + '.outputY', pac + '.' + wal[1])
            mc.connectAttr(blend_chain.switch.attr, pac + '.' + wal[0])
            mc.parent(lo_twist, up_twist, self.limb_grp)
            mc.hide(lo_twist, up_twist)

            # Visibility switching
            mc.connectAttr(blend_chain.switch.attr, rev + '.inputZ')
            mc.connectAttr(blend_chain.switch.attr,
                        self.fk_ctrls[0].top + '.visibility')
            mc.connectAttr(rev + '.outputZ', self.ik_ctrl_grp + '.visibility')

            if self.create_ik:
                self.Ikhead = rCtrl.Control(parent=self.control_grp, shape="brackets", side=None, suffix='CTRL', name=f'Head_M_ik', axis='y', group_type='main', rig_type='primary', translate=self.guide_list[-1], rotate=self.guide_list[-1])
                self.Ikhead.tag_as_controller()
                if self.segments > 2:
                    mc.parentConstraint(self.Ikhead.ctrl, self.iklist[-1].top, mo=True)
                    #mc.orientConstraint(self.Ikhead.ctrl, self.ikspline_joints[-3], mo=True) #-3 cause head jank, should be -1
                    #mc.parentConstraint(self.Ikhead.ctrl, self.iklist[-2].top, mo=True)
                    mc.addAttr(self.Ikhead.ctrl, longName='Stretchy', at='double', dv=.2, k=True, max=1, min=0 )

                    mc.addAttr(self.Ikhead.ctrl, ln='roll', at='double', k=True)
                    mc.addAttr(self.Ikhead.ctrl, ln='twist', at='double', k=True)
                    mc.connectAttr(f'{self.Ikhead.ctrl}.Stretchy', f'{self.iklist[-1].ctrl}.Stretchy')
                    mc.connectAttr(f'{self.Ikhead.ctrl}.roll', f'{self.iklist[-1].ctrl}.roll')
                    mc.connectAttr(f'{self.Ikhead.ctrl}.twist', f'{self.iklist[-1].ctrl}.twist')
                    mc.hide(self.iklist[-1].ctrl)

        

    




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
        if self.split:
            split_joint = self.bind_joints[0]
            split_joints: list[str] = self.bind_joints
            mc.addAttr(split_joint, longName="split_joints", dataType="string")
            mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")
    
    
    def add_plugs(self):
        """Sets up plugs for the neck rig, handling FK/IK chains, hidden nodes, and IK/FK switch."""

        # ------------------------------
        # Skeleton plug
        # ------------------------------
        if hasattr(self, 'bind_joints') and self.bind_joints:
            rAttr.Attribute(
                node=self.part_grp,
                type='plug',
                value=['chest_M_JNT'],
                name='skeletonPlugs',
                children_name=[self.bind_joints[0]]
            )

        # ------------------------------
        # PAC (parent/child) rig plugs
        # ------------------------------
        driver_list = ['chest_M_02_JNT']  # base driver
        driven_list = []

        # FK driven nodes
        if self.create_fk and hasattr(self, 'fk_ctrls'):
            driven_list += [ctrl.top for ctrl in self.fk_ctrls]

        # IK driven nodes
        if self.create_ik:
            if self.segments == 2 and hasattr(self, 'ik_ctrls'):
                driven_list += [ctrl.ctrl for ctrl in self.ik_ctrls]
            elif self.segments > 2 and hasattr(self, 'spline_ctrls'):
                driven_list += [ctrl.ctrl for ctrl in self.ikspline_ctrls]

        rAttr.Attribute(
            node=self.part_grp,
            type='plug',
            value=driver_list,
            name='pacRigPlugs',
            children_name=driven_list
        )

        # ------------------------------
        # Hidden nodes
        # ------------------------------
        hide_list = []
        #if self.create_fk and hasattr(self, 'fk_ctrls'):
        #    hide_list += [ctrl.top for ctrl in self.fk_ctrls[1:]]  # hide all except first
        for node in ['neck_M_06_fk_CTRL_CNST_GRP', 'neck_M_IKSPLINE_START_CTRL_CNST_GRP', 'neck_M_IKSPLINE_MID_CTRL_CNST_GRP', 'neck_M_IKSPLINE_END_CTRL_CNST_GRP']:
            hide_list.append(node)
        if self.create_ik:
            if self.segments == 2 and hasattr(self, 'ik_ctrls'):
                hide_list += [ctrl.ctrl for ctrl in self.ik_ctrls[1:]]  # hide IK handles except base
            elif self.segments > 2 and hasattr(self, 'spline_ctrls'):
                hide_list += [ctrl.ctrl for ctrl in self.ikspline_ctrls[1:]]

        if hide_list:
            rAttr.Attribute(
                node=self.part_grp,
                type='plug',
                value=[' '.join(hide_list)],
                name='hideRigPlugs',
                children_name=['hideNodes']
            )
            for node_name in hide_list:
                if mc.objExists(node_name):
                    mc.hide(node_name)

        # ------------------------------
        # IK/FK switch on limb group
        # ------------------------------
        if self.create_ik and self.create_fk:
            switch_attr_name = 'IKFK'
            # Add attribute on limb group if it doesn't exist
            if not mc.objExists(f"{self.limb_grp}.{switch_attr_name}"):
                mc.addAttr(self.limb_grp, longName=switch_attr_name, attributeType='bool', keyable=True)

            # Store attribute name for reference
            self.ik_switch_attr = f"{self.limb_grp}.{switch_attr_name}"
            if self.fkauto:
                mc.connectAttr('neck_M_01_fk_CTRL_CNST_GRP.visibility', f'{self.fkauto_ctrl.top}.visibility')

        mc.parentConstraint('chest_top_M_CTRL', 'neck_M_IK_CTRL_GRP', mo=True)
        mc.parentConstraint('chest_top_M_CTRL', self.Ikhead.top, mo=True)
        if self.create_ik and self.create_fk:
            ikfkswitchattr = []
            ikfkswitchattr.extend(self.iklist)
            ikfkswitchattr.append(self.Ikhead)
            ikfkswitchattr.append(self.fkauto_ctrl)
            ikfkswitchattr.extend(self.fk_ctrls)
            for ctrl in ikfkswitchattr:
                mc.addAttr(ctrl.ctrl, longName='FK_IK_Switch', proxy='neck_M.switch')
            mc.connectAttr('neck_M_REV.outputZ', f'{self.Ikhead.top}.visibility')






