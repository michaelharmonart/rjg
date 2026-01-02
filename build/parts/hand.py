from importlib import reload

import maya.cmds as mc
import rjg.build.chain as rChain
import rjg.build.rigModule as rModule
import rjg.libs.attribute as rAttr
import rjg.libs.control.ctrl as rCtrl

reload(rAttr)
reload(rModule)
reload(rChain)
reload(rCtrl)


class Hand(rModule.RigModule):
    def __init__(
        self,
        side=None,
        part=None,
        guide_list=None,
        ctrl_scale=None,
        local_orient=False,
        model_path=None,
        guide_path=None,
        expression_control=True,
        bendy_visibility: bool | None = None,
        handroll = False
    ):
        super().__init__(
            side=side,
            part=part,
            guide_list=guide_list,
            ctrl_scale=ctrl_scale,
            model_path=model_path,
            guide_path=guide_path,
        )

        self.base_name = self.part + "_" + self.side
        self.expression_control = expression_control

        self.local_orient = local_orient
        self.bendy_visibility = bendy_visibility
        self.handroll = handroll
        
        self.create_module()

    def create_module(self):
        super().create_module()

        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.add_plugs()

    def control_rig(self):
        self.hand_01 = rCtrl.Control(parent=self.control_grp, shape='cube', side=None, suffix='CTRL', name=self.base_name + '_01', axis='y', group_type='main',
                                     rig_type='primary', translate=self.guide_list[0], rotate=(0, 0, 0), ctrl_scale=self.ctrl_scale)
        self.hand_02 = rCtrl.Control(parent=self.hand_01.ctrl, shape='cube', side=None, suffix='CTRL', name=self.base_name + '_02', axis='y', group_type='main',
                                     rig_type='secondary', translate=self.guide_list[0], rotate=(0, 0, 0), ctrl_scale=self.ctrl_scale * 0.85)
        self.hand_local = rCtrl.Control(parent=self.hand_02.ctrl, shape='quad_arrow', side=None, suffix='CTRL', name=self.base_name + '_local', axis='y', group_type='main',
                                     rig_type='secondary', translate=self.guide_list[0], rotate=self.guide_list[0], ctrl_scale=self.ctrl_scale)
        self.hand_fk = rCtrl.Control(parent=self.control_grp, shape='circle', side=None, suffix='CTRL', name=self.base_name + '_fk', axis='y', group_type='main',
                                     rig_type='fk', translate=self.guide_list[0], rotate=self.guide_list[0], ctrl_scale=self.ctrl_scale)

        for c in [self.hand_01, self.hand_02, self.hand_local, self.hand_fk]:
            c.tag_as_controller()

        if self.expression_control:
            self.hand_express = rCtrl.Control(parent=self.control_grp, shape="square", side=None, suffix='CTRL', name=f'{self.base_name}_express', axis='y', group_type='main', rig_type='primary', translate=self.guide_list[0], rotate=self.guide_list[0], ctrl_scale=self.ctrl_scale)
            self.hand_express.tag_as_controller()
            mc.addAttr(self.hand_express.ctrl, longName='Falloff', attributeType='float', min=0.0, max=10.0, defaultValue=5.0, keyable=True)
            #mc.createNode('multilpyDivide', name=f"{self.hand_express.ctrl}_MD1")
            #mc.setAttr(f"{self.hand_express.ctrl}_MD1.input2X", .1)
            #mc.createNode('multilpyDivide', name=f"{self.hand_express.ctrl}_MD2")
            #mc.connectAttr(f"{self.hand_express.ctrl}_MD1.outputX", f"{self.hand_express.ctrl}_MD2.input2X")
            #mc.connectAttr(f"{self.hand_express.ctrl}_MD1.outputX", f"{self.hand_express.ctrl}_MD2.input2Y")
            #mc.connectAttr(f'{self.hand_express.ctrl}.Falloff', f"{self.hand_express.ctrl}_MD1.input1X")
            mc.createNode('remapValue', name=f"{self.hand_express.ctrl}_HIGHER")
            mc.setAttr(f"{self.hand_express.ctrl}_HIGHER.inputMax", 7.5)
            mc.createNode('remapValue', name=f"{self.hand_express.ctrl}_LOWER")
            mc.setAttr(f"{self.hand_express.ctrl}_LOWER.inputMax", 10)
            mc.setAttr(f"{self.hand_express.ctrl}_LOWER.inputMin", 2.5)
            mc.connectAttr(f'{self.hand_express.ctrl}.Falloff', f"{self.hand_express.ctrl}_HIGHER.inputValue")
            mc.connectAttr(f'{self.hand_express.ctrl}.Falloff', f"{self.hand_express.ctrl}_LOWER.inputValue")
            
            self.bendy_vis_attr: str | None = None
            if self.bendy_visibility is not None:
                self.bendy_vis_attr = rAttr.Attribute(
                    node=self.module_grp,
                    type="double",
                    min=0,
                    max=1,
                    keyable=True,
                    name="handBendyVisibility",
                    value=1 if self.bendy_visibility else 0,
                ).attr

        if self.handroll:
            if self.side == 'L':
                sidelong= 'Left'
            else:
                sidelong='Right'

            mc.addAttr(self.hand_01.ctrl, longName='roll', at='double', k=True)
            rollattr = f'{self.hand_01.ctrl}.roll'
            mc.addAttr(self.hand_01.ctrl, longName='bank', at='double', k=True)
            bankattr = f'{self.hand_01.ctrl}.bank'
            prejnt=False
            for guide in [f'HandRollBack', f'HandRollFront', f'HandRollMid', f'HandBankIn', f'HandBankOut', f'Hand']:
                pos = mc.xform(f'{sidelong}{guide}', q=True, ws=True, t=True)
                rot = mc.xform(f'{sidelong}{guide}', q=True, ws=True, ro=True)

                mc.select(clear=True)
                jnt = mc.joint(name=f'{self.side}_{guide}_PV', p=pos, o=rot)
                if prejnt:
                    mc.parent(jnt, prejnt)
                prejnt=jnt


            self.rollpiv = f'{self.side}_Hand_PV'
            remap_HandRollBack = mc.createNode('remapValue', name=f'{self.side}_HandRollBack_remap')
            mc.setAttr(f'{remap_HandRollBack}.inputMax', -90)
            mc.setAttr(f'{remap_HandRollBack}.outputMax', -90)
            mc.connectAttr(rollattr, f'{remap_HandRollBack}.inputValue')
            mc.connectAttr(f'{remap_HandRollBack}.outValue', f'{self.side}_HandRollBack_PV.rotateX')

            remap_HandRollFront = mc.createNode('remapValue', name=f'{self.side}_HandRollFront_remap')
            mc.setAttr(f'{remap_HandRollFront}.inputMax', 140)
            mc.setAttr(f'{remap_HandRollFront}.inputMin', 90)
            mc.setAttr(f'{remap_HandRollFront}.outputMax', 50)
            mc.connectAttr(rollattr, f'{remap_HandRollFront}.inputValue')
            mc.connectAttr(f'{remap_HandRollFront}.outValue', f'{self.side}_HandRollFront_PV.rotateX')

            remap_HandRollMid = mc.createNode('remapValue', name=f'{self.side}_HandRollMid_remap')
            remap_HandRollMid2 = mc.createNode('remapValue', name=f'{self.side}_HandRollMid2_remap')
            mc.setAttr(f'{remap_HandRollMid}.inputMax', 40)
            mc.setAttr(f'{remap_HandRollMid}.outputMax', 40)
            mc.setAttr(f'{remap_HandRollMid2}.inputMax', 90)
            mc.setAttr(f'{remap_HandRollMid2}.inputMin', 40)
            mc.setAttr(f'{remap_HandRollMid2}.outputMax', -40)
            mc.connectAttr(rollattr, f'{remap_HandRollMid}.inputValue')
            mc.connectAttr(rollattr, f'{remap_HandRollMid2}.inputValue')

            adddl = mc.createNode('addDL', name=f'{self.base_name}_RolladdDL')

            mc.connectAttr(f'{remap_HandRollMid}.outValue', f'{adddl}.input1')
            mc.connectAttr(f'{remap_HandRollMid2}.outValue', f'{adddl}.input2')
            mc.connectAttr(f'{adddl}.output', f'{self.side}_HandRollMid_PV.rotateX')


            #mc.connectAttr(f'{remap_HandRollMid}.outValue', f'{self.side}_HandRollMid_PV.rotateX')



            remap_HandBankIn = mc.createNode('remapValue', name=f'{self.side}_HandBankIn_remap')
            mc.setAttr(f'{remap_HandBankIn}.inputMax', -90)
            mc.setAttr(f'{remap_HandBankIn}.outputMax', 90)
            mc.connectAttr(bankattr, f'{remap_HandBankIn}.inputValue')
            mc.connectAttr(f'{remap_HandBankIn}.outValue', f'{self.side}_HandBankIn_PV.rotateZ')


            remap_HandBankOut = mc.createNode('remapValue', name=f'{self.side}_HandBankOut_remap')
            mc.setAttr(f'{remap_HandBankOut}.inputMax', 90)
            mc.setAttr(f'{remap_HandBankOut}.outputMax', -90)
            mc.connectAttr(bankattr, f'{remap_HandBankOut}.inputValue')
            mc.connectAttr(f'{remap_HandBankOut}.outValue', f'{self.side}_HandBankOut_PV.rotateZ')

            self.rollroot = mc.joint(name=f'{self.side}_HandRootRoll_PV', p=pos, o=rot)
            mc.parent(f'{self.side}_HandRollBack_PV', self.rollroot)
            mc.hide(self.rollroot) 
            #mc.parent(self.rootroll, self.module_grp)

            


            

    def output_rig(self):
        ik_jnt = mc.joint(self.hand_local.ctrl, name=self.hand_01.ctrl.replace("CTRL", "ik_JNT"))
        fk_jnt = mc.joint(self.hand_local.ctrl, name=self.hand_01.ctrl.replace("CTRL", "JNT"))

        if self.handroll:
            mc.parentConstraint(self.rollpiv, ik_jnt, mo=True)
            mc.parentConstraint(self.hand_local.ctrl, self.rollroot, mo=True)
        else:    
            mc.parentConstraint(self.hand_local.ctrl, ik_jnt, mo=True)
        mc.parentConstraint(self.hand_fk.ctrl, fk_jnt, mo=True)
        mc.connectAttr(self.hand_local.ctrl + '.scale', ik_jnt + '.scale')
        mc.connectAttr(self.hand_fk.ctrl + '.scale', fk_jnt + '.scale')

        self.blend_chain = rChain.Chain(transform_list=[ik_jnt], side=self.side, suffix='switch_JNT', name=self.part)
        self.blend_chain.create_blend_chain(switch_node=self.base_name, chain_a=[fk_jnt], chain_b=[ik_jnt], translate=False)

        rev = mc.createNode("reverse", name=self.base_name + '_REV')
        mc.connectAttr(self.blend_chain.switch.attr, rev + '.inputX')
        mc.connectAttr(rev + '.outputX', self.hand_01.top + '.visibility')

        mc.group(ik_jnt, fk_jnt, self.blend_chain.joints[0], parent=self.module_grp, name=self.base_name + '_JNT_GRP')
        mc.matchTransform(self.base_name + '_JNT_GRP', self.guide_list[0])

    def skeleton(self):
        jnt = mc.joint(self.skel, name=self.base_name + '_JNT')
        mc.parentConstraint(self.blend_chain.joints[0], jnt, mo=False)
        mc.connectAttr(self.blend_chain.joints[0] + '.scale', jnt + '.scale')
        self.bind_joints = [jnt]
        self.tag_bind_joints(self.bind_joints)

    def add_plugs(self):
        rAttr.Attribute(node=self.part_grp, type='plug', value=['mc.ls("arm_' + self.side + '_??_JNT")[-1]'], name='skeletonPlugs', children_name=[self.bind_joints[0]])

        driver_list = ['arm_'+ self.side +'_03_switch_JNT', 'COG_M_CTRL']
        driven_list = [self.base_name + '_fk_CTRL_CNST_GRP', self.base_name + '_JNT_GRP']
        rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)

        driver_list = ['arm_'+ self.side +'_03_switch_JNT']
        driven_list = [self.base_name + '_01_switch_JNT']
        rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pocRigPlugs', children_name=driven_list)

        hide_list = ['hand_'+ self.side + '_fk_CTRL_CNST_GRP']
        rAttr.Attribute(node=self.part_grp, type='plug', value=[' '.join(hide_list)], name='hideRigPlugs', children_name=['hideNodes'])

        target_list = ['ROOT',
                       'global_M_CTRL',
                       'root_02_M_CTRL',
                       'COG_M_CTRL',
                       'chest_M_01_CTRL', '2']
        name_list = ['world', 'global', 'root', 'hip', 'chest', 'default_value']
        rAttr.Attribute(node=self.part_grp, type='plug', value=target_list, name=self.hand_01.ctrl +'_parent', children_name=name_list)

        switch_attr = self.side.lower() + 'ArmIKFK'
        switch_attr = 'arm' + self.side + '_IKFK'
        rAttr.Attribute(node=self.part_grp, type='plug', value=[switch_attr], name='switchRigPlugs', children_name=['ikFkSwitch'])

        if self.expression_control:
            mc.parentConstraint(f'hand_{self.side}_01_switch_JNT', self.hand_express.top)
