import maya.cmds as mc
from importlib import reload

import rjg.build.rigModule as rModule
import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.build.fk as rFk
reload(rModule)
reload(rAttr)
reload(rChain)
reload(rFk)

class Finger(rModule.RigModule, rFk.Fk):
    def __init__(self, side=None, part=None, guide_list=None, ctrl_scale=1, model_path=None, guide_path=None, pad='auto', remove_last=True, fk_shape='circle', par_ctrl=None, bendy=False):
        super().__init__(side=side, part=part, guide_list=guide_list, ctrl_scale=ctrl_scale, model_path=model_path, guide_path=guide_path)

        self.__dict__.update(locals())
        self.gimbal = None
        self.offset = None
        self.bendy = bendy
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

        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.add_plugs()
        if self.bendy == True:
            self.fk_chain.split_chain(segments=4)

            self.add_bendy_twist(ctrl_scale=self.ctrl_scale * 0.8, mirror=self.mirror)

    
    def add_bendy_twist(self, ctrl_scale=None, mirror=True, global_scale_attr=None):
        """
        Adds a bend-twist chain rig for the finger.
        This creates Start, Mid, and End bendy controls per segment in the finger chain.
        """
        if not hasattr(self, 'fk_chain') or not self.fk_chain:
            mc.error("Cannot add bendy twist: Chain not built. Run skeleton() first.")
        if ctrl_scale is None:
            ctrl_scale = self.ctrl_scale if hasattr(self, 'ctrl_scale') else 1.0

        # We’ll use the existing bind joints as the main chain
        self.fk_chain.joints = self.bind_joints

        # Create the bendy twist chain
        rig_dict = self.fk_chain.bend_twist_chain(
            ctrl_scale=ctrl_scale,
            mirror=mirror,  # you can flip this if you have naming conventions for L/R
            global_scale=global_scale_attr,
            sec_axis=(0, 0, 1)
        )

        # rig_dict typically returns control and module groups for organization
        ctrl_grp = rig_dict.get('control')
        module_grp = rig_dict.get('module')

        # parent the bendy groups into your module structure
        if ctrl_grp and mc.objExists(ctrl_grp):
            mc.parent(ctrl_grp, self.control_grp)
        if module_grp and mc.objExists(module_grp):
            mc.parent(module_grp, self.module_grp)

        # Store for future reference
        self.bendy_ctrl_grp = ctrl_grp
        self.bendy_module_grp = module_grp

        return rig_dict

    def control_rig(self):
        self.build_fk_controls()
        mc.parent(self.fk_ctrls[0].top, self.control_grp)

    def output_rig(self):
        self.build_fk_chain()
        mc.parent(self.fk_joints[0], self.module_grp)

    def skeleton(self):
        fk_chain = rChain.Chain(transform_list=self.fk_joints, side=self.side, suffix='JNT', name=self.part)
        fk_chain.create_from_transforms(parent=self.skel, scale_constraint=False)

        if self.remove_last:
            mc.delete(self.fk_ctrls[-1].top)
            self.bind_joints = fk_chain.joints[:-1]
        else:
            self.bind_joints = fk_chain.joints

        self.tag_bind_joints(self.bind_joints)
        self.fk_chain = fk_chain

    def add_plugs(self):
        #rAttr.Attribute(node=self.part_grp, type='plug', value=['hand_' + self.side + '_JNT'], name='skeletonPlugs', children_name=[self.bind_joints[0]])

        if not self.par_ctrl:
            driver_list = ['hand_' + self.side + '_01_switch_JNT']
            rAttr.Attribute(node=self.part_grp, type='plug', value=['hand_' + self.side + '_JNT'], name='skeletonPlugs', children_name=[self.bind_joints[0]])
        else:
            driver_list = [self.par_ctrl]
            rAttr.Attribute(node=self.part_grp, type='plug', value=[self.par_ctrl], name='skeletonPlugs', children_name=[self.bind_joints[0]])

        driven_list = [self.base_name + '_01_fk_CTRL_CNST_GRP']

        rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)