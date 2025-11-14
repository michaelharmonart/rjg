import maya.cmds as mc
from importlib import reload

import rjg.build.rigModule as rModule
import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.build.fk as rFk
import rjg.build.ik as rIk

reload(rModule)
reload(rAttr)
reload(rChain)
reload(rFk)
reload(rIk)

class Finger(rModule.RigModule, rFk.Fk, rIk.Ik):
    def __init__(self, side=None, part=None, guide_list=None, ctrl_scale=1, model_path=None, guide_path=None, pad='auto', remove_last=True, fk_shape='circle', par_ctrl=None, segments=4, 
                bendy=False, stretchy=True, twisty=True, create_ik=True, create_fk=True, expression_control=True, sticky=None, solver=None, pv_guide='auto', offset_pv=0, slide_pv=None,
                gimbal=True, offset=True, gimbal_shape='circle', offset_shape='square'):
        super().__init__(side=side, part=part, guide_list=guide_list, ctrl_scale=ctrl_scale, model_path=model_path, guide_path=guide_path)

        self.__dict__.update(locals())
        self.gimbal = None
        self.offset = None
        self.bendy = bendy

        self.guide_list=guide_list
        self.OGguide_list= list(guide_list)
        self.create_ik = create_ik
        self.create_fk = create_fk
        self.expression_control = expression_control
        
        if self.pad == 'auto':
            self.pad = len(str(len(self.guide_list))) + 1
        is_right = self.side in ["R", "r", "Right", "right"]
        if is_right:
            self.mirror = True
        else:
            self.mirror = False

        self.create_module()


    def create_module(self):
        super().create_module()
        if self.create_ik == True:
            self.guide_list.pop(0)
            self.guide_list.pop(-1)
            self.check_solvers()
            self.check_pv_guide()
            self.guide_list = list(self.OGguide_list)

        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.add_plugs()
        if self.bendy == True:
            self.fk_chain.split_chain(segments=4)

            self.add_bendy_twist(ctrl_scale=self.ctrl_scale * 0.8, mirror=self.mirror)

    def control_rig(self):
        if self.create_ik == False:
            self.build_fk_controls()
            mc.parent(self.fk_ctrls[0].top, self.control_grp)
        else:
            self.build_fk_controls()
            mc.parent(self.fk_ctrls[0].top, self.control_grp)
            
            self.guide_list.pop(0)
            self.guide_list.pop(-1)

            self.pv_control = self.build_ik_controls()
            mc.parent(self.ik_ctrl_grp, self.control_grp)

            self.guide_list = list(self.OGguide_list)


    def output_rig(self):
        if self.create_ik == False:
            self.build_fk_chain()
            mc.parent(self.fk_joints[0], self.module_grp)
            
        else:
            #self.build_fk_chain()
            #mc.parent(self.fk_joints[0], self.module_grp)
            self.limb_grp = mc.group(em=True, parent=self.module_grp,
                                   name=self.base_name + '_RIG_GRP')
            mc.matchTransform(self.limb_grp, self.guide_list[0])

            #FK
            self.guide_list.pop(-1)
            self.build_fk_chain()
            mc.parent(self.fk_joints[0], self.limb_grp)
            self.src_chain = self.fk_chain
            self.src_joints = self.fk_joints
            up_twist = self.fk_ctrls[0].ctrl
            lo_twist = self.fk_ctrls[-1].ctrl

            #IK
            self.guide_list.pop(0)
            self.build_ik_chain()
            self.build_ikh(scale_attr=self.global_scale)
            mc.parent(self.ikh, self.ik_joints[0], self.limb_grp)
            self.src_chain = self.ik_chain
            self.src_joints = self.ik_joints
            up_twist = self.base_ctrl.ctrl
            lo_twist = self.main_ctrl.ctrl

            blend_chain = rChain.Chain(transform_list=self.src_joints,
                                        side=self.side,
                                        suffix='switch_JNT',
                                        name=self.part)

            blend_chain.create_blend_chain(switch_node=self.base_name,
                                           chain_a=self.fk_joints,
                                           chain_b=self.ik_joints)
            mc.parent(blend_chain.joints[0], self.limb_grp)
            self.src_chain = blend_chain
            self.src_joints = blend_chain.joints

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

            if self.bendy:
                if self.side == 'R':
                    mirror = True
                else:
                    mirror = False
                bend = self.src_chain.bend_twist_chain(
                                                    ctrl_scale=self.ctrl_scale,
                                                    mirror=mirror,
                                                    global_scale=self.global_scale.attr)

                mc.parent(bend['control'], self.control_grp)
                mc.parent(bend['module'], self.module_grp)

    def skeleton(self):
        if self.create_ik == False:
            fk_chain = rChain.Chain(transform_list=self.fk_joints, side=self.side, suffix='JNT', name=self.part)
            fk_chain.create_from_transforms(parent=self.skel, scale_constraint=False)

            if self.remove_last:
                mc.delete(self.fk_ctrls[-1].top)
                self.bind_joints = fk_chain.joints[:-1]
            else:
                self.bind_joints = fk_chain.joints

            self.tag_bind_joints(self.bind_joints)
            self.fk_chain = fk_chain
        else:
            fk_chain = rChain.Chain(transform_list=self.fk_joints, side=self.side, suffix='JNT', name=self.part)
            fk_chain.create_from_transforms(parent=self.skel, scale_constraint=False)

            if self.remove_last:
                mc.delete(self.fk_ctrls[-1].top)
                self.bind_joints = fk_chain.joints[:-1]
            else:
                self.bind_joints = fk_chain.joints

            self.tag_bind_joints(self.bind_joints)
            self.fk_chain = fk_chain

    def add_bendy_twist(self, ctrl_scale=None, mirror=True, global_scale_attr=None):
        """
        Adds a bend-twist chain rig for the finger.
        This creates Start, Mid, and End bendy controls per segment in the finger chain,
        skipping the first (metacarpal) joint.
        """
        if not hasattr(self, 'fk_chain') or not self.fk_chain:
            mc.error("Cannot add bendy twist: Chain not built. Run skeleton() first.")

        if ctrl_scale is None:
            ctrl_scale = getattr(self, 'ctrl_scale', 1.0)

        # Use bind joints as base
        self.fk_chain.joints = self.bind_joints

        # Skip first joint (metacarpal) for bendy setup
        original_joints = list(self.fk_chain.joints)
        self.fk_chain.joints = original_joints[1:]  # skip metacarpal

        # Build the bendy rig
        rig_dict = self.fk_chain.bend_twist_chain(
            ctrl_scale=ctrl_scale,
            mirror=mirror,
            global_scale=global_scale_attr,
            sec_axis=(0, 0, 1)
        )

        # Restore full joint list
        self.fk_chain.joints = original_joints

        # Get groups and parent them properly
        ctrl_grp = rig_dict.get('control')
        module_grp = rig_dict.get('module')

        if ctrl_grp and mc.objExists(ctrl_grp):
            mc.parent(ctrl_grp, self.control_grp)
        if module_grp and mc.objExists(module_grp):
            mc.parent(module_grp, self.module_grp)

        # Store for later
        self.bendy_ctrl_grp = ctrl_grp
        self.bendy_module_grp = module_grp

        return rig_dict

    def add_plugs(self):
        #rAttr.Attribute(node=self.part_grp, type='plug', value=['hand_' + self.side + '_JNT'], name='skeletonPlugs', children_name=[self.bind_joints[0]])

        if not self.par_ctrl:
            driver_list = ['hand_' + self.side + '_01_switch_JNT']
            rAttr.Attribute(node=self.part_grp, type='plug', value=['hand_' + self.side + '_JNT'], name='skeletonPlugs', children_name=[self.bind_joints[0]])
        else:
            driver_list = [self.par_ctrl]
            rAttr.Attribute(node=self.part_grp, type='plug', value=[self.par_ctrl], name='skeletonPlugs', children_name=[self.bind_joints[0]])
        driven_list = [self.base_name + '_01_fk_CTRL_CNST_GRP']
        if self.part == 'fingerThumb':
            rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)
        else:
            if self.expression_control:
                if mc.objExists(f'hand_{self.side}_express_CTRL'):
                    falloff = None
                    if self.part == 'fingerIndex':
                        mult1value = 1
                        mult2value = 0
                    elif self.part == 'fingerMiddle':
                        mult1value = 0
                        mult2value = 0
                        falloff = 'inner'
                    elif self.part == 'fingerRing':
                        mult1value = 0
                        mult2value = 0 
                        falloff = 'outer'
                    elif self.part == 'fingerPinky':
                        mult1value = 0
                        mult2value = 1
                    else:
                        mult1value = 0
                        mult2value = 1
                    mc.parentConstraint(driver_list, driven_list, mo=True)
                    mc.parentConstraint(f'hand_{self.side}_express_CTRL', driven_list, mo=True)
                    mc.setAttr(f'{self.base_name}_01_fk_CTRL_CNST_GRP_parentConstraint1.hand_{self.side}_01_switch_JNTW0', mult1value)
                    mc.setAttr(f'{self.base_name}_01_fk_CTRL_CNST_GRP_parentConstraint1.hand_{self.side}_express_CTRLW1', mult2value)
                    if falloff == 'inner':
                        mc.connectAttr( f"hand_{self.side}_express_CTRL_HIGHER.outValue", f'{self.base_name}_01_fk_CTRL_CNST_GRP_parentConstraint1.hand_{self.side}_01_switch_JNTW0')
                        mc.connectAttr( f"hand_{self.side}_express_CTRL_LOWER.outValue", f'{self.base_name}_01_fk_CTRL_CNST_GRP_parentConstraint1.hand_{self.side}_express_CTRLW1')
                    elif falloff == 'outer':
                        mc.connectAttr( f"hand_{self.side}_express_CTRL_LOWER.outValue", f'{self.base_name}_01_fk_CTRL_CNST_GRP_parentConstraint1.hand_{self.side}_01_switch_JNTW0')
                        mc.connectAttr( f"hand_{self.side}_express_CTRL_HIGHER.outValue", f'{self.base_name}_01_fk_CTRL_CNST_GRP_parentConstraint1.hand_{self.side}_express_CTRLW1')



                else:
                    rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)
            else:
                rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)