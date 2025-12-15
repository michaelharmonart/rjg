import maya.cmds as mc
from importlib import reload

import rjg.build.rigModule as rModule
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
import rjg.libs.attribute as rAttr
from rjg.libs.maya_api import node
from rjg.libs.transform import drive_transform_with_matrix, get_world_matrix
reload(rModule)
reload(rChain)
reload(rCtrl)
reload(rAttr)

class Toe(rModule.RigModule):
    def __init__(self, side=None, part=None, guide_list=None, ctrl_scale=None, shape='circle', model_path=None, guide_path=None, par_ctrl=None, par_jnt=None, express=False):
        super().__init__(side=side, part=part, guide_list=guide_list, ctrl_scale=ctrl_scale, model_path=model_path, guide_path=guide_path)

        self.par_ctrl = par_ctrl
        self.par_jnt = par_jnt
        self.express = express

        self.create_module()


    def create_module(self):
        super().create_module()

        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.add_plugs()


    def control_rig(self):
        Gtranslate = mc.getAttr(self.guide_list + '.translate')
        Grotate = mc.getAttr(self.guide_list + '.rotate')

        self.arbit_ctrl = rCtrl.Control(parent=self.control_grp, shape='circle', side=self.side, suffix='CTRL', name=self.base_name, axis='z', group_type='main', rig_type='primary', translate=Gtranslate[0], rotate=Grotate[0], ctrl_scale=self.ctrl_scale)
        self.arbit_ctrl.tag_as_controller()

    def output_rig(self):
        arbit_jnt_grp = mc.group(parent=self.module_grp, empty=True, name=self.base_name + '_JNT_GRP')
        mc.matchTransform(arbit_jnt_grp, self.arbit_ctrl.ctrl)

        self.arbit_jnt = mc.joint(arbit_jnt_grp, name=self.arbit_ctrl.ctrl.replace('CTRL', 'JNT'))
        mc.parentConstraint(self.arbit_ctrl.ctrl, self.arbit_jnt, mo=True)

    def skeleton(self):
        arbit_chain = rChain.Chain(transform_list=[self.arbit_jnt], side=self.side, suffix='JNT', name=self.part)
        arbit_chain.create_from_transforms(parent=self.skel, pad=False)
        self.bind_joints = arbit_chain.joints
        self.tag_bind_joints(self.bind_joints)

    def add_plugs(self):


    
        if self.express:
            driver_list = [f'foot_' + self.side + '_03_switch_JNT', 'NULL']
            driven_list = [f"{self.arbit_ctrl.ctrl}_CNST_GRP", 'NULL']
            if mc.objExists(f'foot_{self.side}_express_CTRL'):
                falloff = None
                if self.part == f'{self.side}_Innertoe':
                    mult1value = 1
                    mult2value = 0
                elif self.part == f'{self.side}_Middletoe':
                    mult1value = 0
                    mult2value = 0
                    falloff = 'inner'
                elif self.part == f'{self.side}_Outertoe':
                    mult1value = 0
                    mult2value = 0 
                    falloff = 'outer'
                elif self.part == f'{self.side}_fingerPinky':
                    mult1value = 0
                    mult2value = 1
                else:
                    mult1value = 0
                    mult2value = 1
                    
                expression_blend_node = node.BlendMatrixNode(name=f"{self.base_name}_ExpressionBlend")
                mc.connectAttr(f"{driver_list[0]}.worldMatrix[0]", expression_blend_node.input_matrix)
                mc.connectAttr(f'foot_{self.side}_express_CTRL.worldMatrix[0]', expression_blend_node.target[0].target_matrix)
                mc.setAttr(expression_blend_node.target[0].weight, mult2value)

                if falloff == 'inner':
                    mc.connectAttr(f"foot_{self.side}_express_CTRL_LOWER.outValue", expression_blend_node.target[0].weight)
                elif falloff == 'outer':
                    mc.connectAttr( f"foot_{self.side}_express_CTRL_HIGHER.outValue", expression_blend_node.target[0].weight)
                    
                offset_node = node.MultMatrixNode(name=f"{self.base_name}_ExpressionOffset")
                offset_matrix = get_world_matrix(driven_list[0]) * get_world_matrix(driver_list[0]).inverse()
                
                mc.setAttr(offset_node.matrix_in[0], offset_matrix, type="matrix")
                mc.connectAttr(expression_blend_node.output_matrix, offset_node.matrix_in[1])
                mc.connectAttr(f"{driven_list[0]}.parentInverseMatrix[0]", offset_node.matrix_in[2])
                
                drive_transform_with_matrix(offset_node.matrix_sum, driven_list[0])


            else:
                rAttr.Attribute(
                    node=self.part_grp,
                    type='plug',
                    value=[self.par_jnt],
                    name='skeletonPlugs',
                    children_name=[self.bind_joints[0]]
                )

                rAttr.Attribute(
                    node=self.part_grp,
                    type='plug',
                    value=[self.par_ctrl],
                    name='pacRigPlugs',
                    children_name=[self.arbit_ctrl.ctrl + '_CNST_GRP']
                )





    """def add_plugs(self):
        rAttr.Attribute(node=self.part_grp, type='plug', value=[self.par_jnt], name='skeletonPlugs', children_name=[self.bind_joints[0]])

        rAttr.Attribute(node=self.part_grp, type='plug', value=[self.par_ctrl], name='pacRigPlugs', children_name=[self.base_name + '_' + self.side + '_CTRL_CNST_GRP'])
    """

