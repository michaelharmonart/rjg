from importlib import reload

import maya.cmds as mc
from rjg.libs.transform import drive_transform_with_matrix, get_world_matrix
import rjg.build.chain as rChain
import rjg.build.fk as rFk
import rjg.build.ik as rIk
import rjg.build.rigModule as rModule
import rjg.libs.attribute as rAttr
from rjg.libs.maya_api import node
import rjg.libs.control.ctrl as rCtrl

reload(rModule)
reload(rAttr)
reload(rChain)
reload(rFk)
reload(rIk)
reload(rCtrl)

class Finger(rModule.RigModule, rFk.Fk, rIk.Ik):
    def __init__(
        self,
        side=None,
        part=None,
        guide_list=None,
        ctrl_scale=1,
        model_path=None,
        guide_path=None,
        pad="auto",
        remove_last=True,
        fk_shape="circle",
        par_ctrl=None,
        bendy=False,
        create_ik: bool =False,
        create_fk: bool =True,
        expression_control = True,
        bendy_vis_attr: str | None = None,
        curl: bool = True,
        curlaxis: str = 'Z',
        handroll=False,
        pv_guide="auto",
    ):
        super().__init__(
            side=side,
            part=part,
            guide_list=guide_list,
            ctrl_scale=ctrl_scale,
            model_path=model_path,
            guide_path=guide_path,
        )

        self.__dict__.update(locals())
        self.gimbal = None
        self.offset = None
        self.bendy = bendy

        self.create_ik = create_ik
        self.create_fk = create_fk
        self.expression_control = expression_control
        self.bendy_vis_attr = bendy_vis_attr
        self.curl = curl
        self.curlaxis = curlaxis
        self.handroll=handroll
        self.pv_guide = pv_guide
        self.slide_pv = None
        self.offset_pv = None
        self.sticky = False
        self.solver= None
        self.stretchy = False
        
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
        
        self.check_pv_guide()
        self.check_solvers()

        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.add_plugs()
        if self.bendy == True:
            self.deformation_chain.split_chain(segments=4)

            self.add_bendy_twist(ctrl_scale=self.ctrl_scale * 0.8, mirror=self.mirror)

    
    def add_bendy_twist(self, ctrl_scale=None, mirror=True, global_scale_attr=None):
        """
        Adds a bend-twist chain rig for the finger.
        This creates Start, Mid, and End bendy controls per segment in the finger chain,
        skipping the first (metacarpal) joint.
        """
        if not hasattr(self, 'fk_chain') or not self.deformation_chain:
            mc.error("Cannot add bendy twist: Chain not built. Run skeleton() first.")

        if ctrl_scale is None:
            ctrl_scale = getattr(self, 'ctrl_scale', 1.0)

        # Use bind joints as base
        self.deformation_chain.joints = self.bind_joints

        # Skip first joint (metacarpal) for bendy setup
        original_joints = list(self.deformation_chain.joints)
        self.deformation_chain.joints = original_joints[1:]  # skip metacarpal

        # Build the bendy rig
        rig_dict = self.deformation_chain.bend_twist_chain(
            ctrl_scale=ctrl_scale,
            mirror=mirror,
            global_scale=global_scale_attr,
            sec_axis=(0, 0, 1),
            bendy_vis_attr=self.bendy_vis_attr
        )

        # Restore full joint list
        self.deformation_chain.joints = original_joints

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


    def control_rig(self):
        self.fk_ctrl_group = mc.group(empty=True, name=f"{self.base_name}_FK_CTLS", parent=self.control_grp) 
        if self.build_fk:
            self.build_fk_controls()
            mc.parent(self.fk_ctrls[0].top, self.control_grp)
        if self.build_ik:
            self.build_ik_controls(guide_list=self.guide_list[1:])
            mc.parent(self.ik_ctrl_grp, self.control_grp)
            mc.parent(self.fk_ctrls[1].top, self.fk_ctrl_group)
        if self.curl:
            if self.part == 'fingerThumb':
                self.curl_ctrl = rCtrl.Control(parent=self.control_grp, shape="curl", side=None, suffix='CTRL', name=f'{self.base_name}_curl', axis='y', group_type='main', rig_type='primary', translate=self.guide_list[0], rotate=self.guide_list[0], ctrl_scale=self.ctrl_scale)
            else:
                self.curl_ctrl = rCtrl.Control(parent=self.control_grp, shape="curl", side=None, suffix='CTRL', name=f'{self.base_name}_curl', axis='y', group_type='main', rig_type='primary', translate=self.guide_list[1], rotate=self.guide_list[1], ctrl_scale=self.ctrl_scale)

    def output_rig(self):
        if self.build_fk:
            self.build_fk_chain()
            mc.parent(self.fk_joints[0], self.module_grp)
        if self.build_ik:
            self.build_ik_chain(guide_list=self.guide_list[1:])
            self.build_ikh(scale_attr=self.global_scale)
            mc.parent(self.ikh, self.ik_joints[0], self.module_grp)
        

    def skeleton(self):
        deformation_chain = rChain.Chain(transform_list=self.fk_joints, side=self.side, suffix='JNT', name=self.part)
        deformation_chain.create_from_transforms(parent=self.skel, scale_constraint=False)

        if self.remove_last:
            mc.delete(self.fk_ctrls[-1].top)
            self.bind_joints = deformation_chain.joints[:-1]
        else:
            self.bind_joints = deformation_chain.joints

        self.tag_bind_joints(self.bind_joints)
        self.deformation_chain = deformation_chain

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
                        
                    expression_blend_node = node.BlendMatrixNode(name=f"{self.base_name}_ExpressionBlend")
                    mc.connectAttr(f"{driver_list[0]}.worldMatrix[0]", expression_blend_node.input_matrix)
                    mc.connectAttr(f'hand_{self.side}_express_CTRL.worldMatrix[0]', expression_blend_node.target[0].target_matrix)
                    mc.setAttr(expression_blend_node.target[0].weight, mult2value)

                    if falloff == 'inner':
                        mc.connectAttr(f"hand_{self.side}_express_CTRL_LOWER.outValue", expression_blend_node.target[0].weight)
                    elif falloff == 'outer':
                        mc.connectAttr( f"hand_{self.side}_express_CTRL_HIGHER.outValue", expression_blend_node.target[0].weight)
                        
                    offset_node = node.MultMatrixNode(name=f"{self.base_name}_ExpressionOffset")
                    offset_matrix = get_world_matrix(driven_list[0]) * get_world_matrix(driver_list[0]).inverse()
                    
                    mc.setAttr(offset_node.matrix_in[0], offset_matrix, type="matrix")
                    mc.connectAttr(expression_blend_node.output_matrix, offset_node.matrix_in[1])
                    mc.connectAttr(f"{driven_list[0]}.parentInverseMatrix[0]", offset_node.matrix_in[2])
                    
                    drive_transform_with_matrix(offset_node.matrix_sum, driven_list[0])


                else:
                    rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)
            else:
                rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)
        if self.curl:
            sec_axes = [a for a in ['X', 'Y', 'Z'] if a != self.curlaxis]
            #mc.pointConstraint(f'{self.base_name}_02_fk_CTRL', f'{self.curl_ctrl.ctrl}', mo=True)
            if self.part == 'fingerThumb': 
                mc.pointConstraint(f'{self.base_name}_01_fk_CTRL', f'{self.base_name}_curl_CTRL_CNST_GRP', mo=True)
                mc.orientConstraint({driver_list[0]}, f'{self.base_name}_curl_CTRL_CNST_GRP', mo=True)
                for num in ['01', '02', '03',]:
                    mc.connectAttr(f'{self.curl_ctrl.ctrl}.rotate{self.curlaxis}', f'{self.base_name}_{num}_fk_CTRL_SDK_GRP.rotate{self.curlaxis}')
                for ax in sec_axes:
                    mc.connectAttr(f'{self.curl_ctrl.ctrl}.rotate{ax}', f'{self.base_name}_01_fk_CTRL_SDK_GRP.rotate{ax}')
            else:
                mc.pointConstraint(f'{self.base_name}_02_fk_CTRL', f'{self.curl_ctrl.top}', mo=True)
                mc.orientConstraint(f'{self.base_name}_01_fk_CTRL', f'{self.curl_ctrl.top}', mo=True)
                for num in ['02', '03', '04']:
                    mc.connectAttr(f'{self.curl_ctrl.ctrl}.rotate{self.curlaxis}', f'{self.base_name}_{num}_fk_CTRL_SDK_GRP.rotate{self.curlaxis}')
                for ax in sec_axes:
                    mc.connectAttr(f'{self.curl_ctrl.ctrl}.rotate{ax}', f'{self.base_name}_02_fk_CTRL_SDK_GRP.rotate{ax}')
            if self.expression_control:
                mc.addAttr(f'hand_{self.side}_express_CTRL', longName=f'{self.base_name}curl', proxy=f'{self.curl_ctrl.ctrl}.rotate{self.curlaxis}')
        
        if self.handroll:
            rollattr = f'hand_{self.side}_01_CTRL.roll'
            remap_HandRollFront = mc.createNode('remapValue', name=f'{self.base_name}_HandRollFront_remap')
            remap_HandRollMid = mc.createNode('remapValue', name=f'{self.base_name}_HandRollMid_remap')
            mc.setAttr(f'{remap_HandRollMid}.inputMax', 40)
            mc.setAttr(f'{remap_HandRollMid}.outputMax', 40)
            
            mc.setAttr(f'{remap_HandRollFront}.inputMax', 90)
            mc.setAttr(f'{remap_HandRollFront}.inputMin', 40)
            mc.setAttr(f'{remap_HandRollFront}.outputMax', -40)


            mc.connectAttr(rollattr, f'{remap_HandRollFront}.inputValue') #fingerMiddle_L_02_fk_CTRL_OFF_GRP
            mc.connectAttr(rollattr, f'{remap_HandRollMid}.inputValue')

            adddl = mc.createNode('addDL', name=f'{self.base_name}_RolladdDL')

            mc.connectAttr(f'{remap_HandRollFront}.outValue', f'{adddl}.input1')
            mc.connectAttr(f'{remap_HandRollMid}.outValue', f'{adddl}.input2')
            mc.connectAttr(f'{adddl}.output', f'{self.base_name}_02_fk_CTRL_OFF_GRP.rotateZ')
