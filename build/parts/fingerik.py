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


class FingerIK(rModule.RigModule, rFk.Fk, rIk.Ik):
    """
    Finger rig module supporting FK, optional IK, IK/FK blend, and optional bendy/twist.
    Designed to be compatible with the BipedLimb patterns in the project.
    """
    def __init__(self, side=None, part=None, guide_list=None, ctrl_scale=1,
                 model_path=None, guide_path=None, pad='auto', remove_last=True,
                 fk_shape='circle', par_ctrl=None, bendy=False, create_ik=True,
                 create_fk=True):
        # call parent init (RigModule handles many common attrs)
        super(FingerIK, self).__init__(side=side, part=part, guide_list=guide_list,
                                     ctrl_scale=ctrl_scale, model_path=model_path,
                                     guide_path=guide_path)

        # copy constructor locals into instance, keep defaults from signature
        self.__dict__.update(locals())

        # tidy up some flags/derived values
        self.gimbal = None
        self.offset = None
        self.bendy = bendy
        self.remove_last = remove_last
        self.fk_shape = fk_shape
        self.par_ctrl = par_ctrl
        self.create_ik = create_ik
        self.create_fk = create_fk

        if self.pad == 'auto':
            self.pad = len(str(len(self.guide_list))) + 1

        self.mirror = True if self.side in ["R", "r", "Right", "right"] else False

        # run the module build
        self.create_module()


    def compute_pv_position(self, joints, ctrl_scale=1.0):
        """
        Compute a pole vector position for a finger based on given joints.
        Skips first joint (metacarpal), offsets by slope direction.
        """
        if len(joints) < 2:
            mc.warning("Not enough joints for PV calculation.")
            return mc.xform(joints[0], q=True, ws=True, t=True)

        # get world positions
        positions = [mc.xform(j, q=True, ws=True, t=True) for j in joints]
        start = positions[0]
        end = positions[-1]

        # average position of all joints
        avg = [sum(v[i] for v in positions) / len(positions) for i in range(3)]

        # vector from start to end
        vec = [end[i] - start[i] for i in range(3)]

        # normalize vector
        length = (vec[0]**2 + vec[1]**2 + vec[2]**2) ** 0.5
        if length == 0:
            mc.warning("Zero-length finger vector, cannot compute PV offset.")
            return avg
        norm_vec = [v / length for v in vec]

        # offset scaled relative to slope
        offset_scale = ctrl_scale * 3
        pv_pos = [avg[i] + norm_vec[i] * offset_scale for i in range(3)]

        return pv_pos


    def create_module(self):
        super(FingerIK, self).create_module()

        # build controls and outputs
        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.add_plugs()

        # optional bendy handling (operate on src_chain if blend exists)
        if self.bendy:
            # ensure src_chain exists (either blend or fk/ik chain set in output_rig)
            if hasattr(self, 'src_chain') and self.src_chain:
                self.src_chain.split_chain(segments=4)
                self.add_bendy_twist(ctrl_scale=self.ctrl_scale * 0.8, mirror=self.mirror,
                                     global_scale_attr=getattr(self, 'global_scale', None))
            else:
                mc.warning(f"{self.base_name}: cannot add bendy - src_chain not built yet.")

    def control_rig(self):
        """
        Build FK and IK controls and parent under control_grp.
        Uses the FK/IK builders present in rFk.Fk and rIk.Ik mixins.
        """
        # FK controls
        if self.create_fk:
            self.build_fk_controls()
            # place top FK control under the module control group
            if self.fk_ctrls:
                mc.parent(self.fk_ctrls[0].top, self.control_grp)

        # IK controls
        if self.create_ik:
            # build ik controls; the rIk.Ik mixin should populate ik_ctrl_grp and ik_ctrls
            self.pv_control = self.build_ik_controls()
            if hasattr(self, 'ik_ctrl_grp') and self.ik_ctrl_grp:
                mc.parent(self.ik_ctrl_grp, self.control_grp)

    def output_rig(self):
        """
        Build FK and IK chains for the finger.
        IK skips the metacarpal joint (first in guide list), parents under metacarpal control,
        and generates a PV guide automatically based on the finger slope and control scale.
        """
        # ensure module group for rig
        self.limb_grp = mc.group(em=True, parent=self.module_grp, name=self.base_name + '_RIG_GRP')
        mc.matchTransform(self.limb_grp, self.guide_list[0])

        # ----------------------------------------------------------
        # FK CHAIN
        # ----------------------------------------------------------
        if self.create_fk:
            self.build_fk_chain()
            mc.parent(self.fk_joints[0], self.limb_grp)
            self.src_chain = getattr(self, 'fk_chain', None)
            self.src_joints = getattr(self, 'fk_joints', None)

        # ----------------------------------------------------------
        # IK CHAIN (skip metacarpal)
        # ----------------------------------------------------------
        if self.create_ik:
            if len(self.guide_list) < 3:
                mc.warning(f"{self.base_name}: Not enough joints to build IK chain.")
            else:
                ik_joints = self.guide_list[1:]  # skip metacarpal
                self.ik_chain = rChain.Chain(
                    transform_list=ik_joints,
                    side=self.side,
                    suffix='ik_JNT',
                    name=self.part
                )
                self.ik_chain.create_from_transforms(parent=self.limb_grp)

                # parent IK chain under metacarpal FK control if available
                if self.create_fk and getattr(self, 'fk_ctrls', None):
                    try:
                        mc.parent(self.ik_chain.joints[0], self.fk_ctrls[0].ctrl)
                    except Exception:
                        mc.warning(f"{self.base_name}: Could not parent IK chain under metacarpal FK ctrl.")

                # compute PV position
                pv_pos = self.compute_pv_position(ik_joints, ctrl_scale=self.ctrl_scale)

                # create a visual locator for PV guide
                pv_loc = mc.spaceLocator(name=f"{self.base_name}_PV_GUIDE_LOC")[0]
                mc.xform(pv_loc, ws=True, t=pv_pos)
                mc.parent(pv_loc, self.control_grp)

                # now build IK handle and controls
                try:
                    gscale = getattr(self, 'global_scale', None)
                    if gscale:
                        self.build_ikh(scale_attr=gscale)
                    else:
                        self.build_ikh()
                except Exception as e:
                    mc.warning(f"{self.base_name}: build_ikh failed - {e}")

                if hasattr(self, 'ikh') and mc.objExists(self.ikh):
                    mc.parent(self.ikh, self.limb_grp)
                else:
                    mc.warning(f"{self.base_name}: IK handle not created.")

                self.ik_joints = self.ik_chain.joints

                # update src_chain defaults
                self.src_chain = self.ik_chain
                self.src_joints = self.ik_joints

        # ----------------------------------------------------------
        # FK/IK BLEND
        # ----------------------------------------------------------
        if self.create_fk and self.create_ik:
            blend_chain = rChain.Chain(
                transform_list=self.src_joints,
                side=self.side,
                suffix='switch_JNT',
                name=self.part
            )

            blend_chain.create_blend_chain(
                switch_node=self.base_name,
                chain_a=self.fk_joints,
                chain_b=self.ik_joints
            )
            mc.parent(blend_chain.joints[0], self.limb_grp)

            self.src_chain = blend_chain
            self.src_joints = blend_chain.joints

            # visibility toggle between FK and IK
            rev = mc.createNode('reverse', name=self.base_name + '_REV')
            try:
                mc.connectAttr(blend_chain.switch.attr, rev + '.inputZ')
                if self.fk_ctrls:
                    mc.connectAttr(blend_chain.switch.attr, self.fk_ctrls[0].top + '.visibility')
                if getattr(self, 'ik_ctrl_grp', None):
                    mc.connectAttr(rev + '.outputZ', self.ik_ctrl_grp + '.visibility')
            except Exception:
                mc.warning(f"{self.base_name}: Could not wire FK/IK visibility switch.")

    def skeleton(self):
        """
        Create bind skeleton from src_joints and tag bind joints.
        Mirrors the BipedLimb create_from_transforms usage.
        """
        if not getattr(self, 'src_joints', None):
            mc.error(f"{self.base_name}: No source joints to create skeleton from.")

        limb_chain = rChain.Chain(transform_list=self.src_joints,
                                   side=self.side,
                                   suffix='JNT',
                                   name=self.part)

        # if FK existed, orient constraints are useful; otherwise keep defaults
        poc = True if self.create_fk else False

        limb_chain.create_from_transforms(orient_constraint=True,
                                          point_constraint=poc,
                                          scale_constraint=False,
                                          parent=self.skel)
        self.bind_joints = limb_chain.joints

        # tag bind joints (exclude last typically)
        if len(self.bind_joints) > 1:
            self.tag_bind_joints(self.bind_joints[:-1])
        else:
            self.tag_bind_joints(self.bind_joints)

        # keep reference to fk_chain for bendy building
        self.fk_chain = getattr(self, 'fk_chain', None)

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

        # Use bind joints as input to bend_twist_chain
        # For fingers, skip the first joint (metacarpal) so that bendy affects phalanges only
        original_joints = list(self.fk_chain.joints)
        if len(original_joints) < 2:
            mc.warning(f"{self.base_name}: Not enough joints for bendy twist.")
            return

        # create a working list skipping first joint
        self.fk_chain.joints = original_joints[1:]

        rig_dict = self.fk_chain.bend_twist_chain(
            ctrl_scale=ctrl_scale,
            mirror=mirror,
            global_scale=global_scale_attr,
            sec_axis=(0, 0, 1)
        )

        # restore fk_chain joints to original
        self.fk_chain.joints = original_joints

        # parent returned groups into module/control groups
        ctrl_grp = rig_dict.get('control')
        module_grp = rig_dict.get('module')

        if ctrl_grp and mc.objExists(ctrl_grp):
            mc.parent(ctrl_grp, self.control_grp)
        if module_grp and mc.objExists(module_grp):
            mc.parent(module_grp, self.module_grp)

        # store references
        self.bendy_ctrl_grp = ctrl_grp
        self.bendy_module_grp = module_grp

        return rig_dict

    def add_plugs(self):
        """
        Register plugs for external pipeline usage.
        Mirrors logic used for 'finger' in BipedLimb.add_plugs() and preserves your previous behavior.
        """
        # skeleton plug: either hand driver or specified parent control
        if not self.par_ctrl:
            skeleton_driver = 'hand_' + self.side + '_01_switch_JNT'
            rAttr.Attribute(node=self.part_grp, type='plug', value=['hand_' + self.side + '_JNT'],
                            name='skeletonPlugs', children_name=[self.bind_joints[0]])
            driver_list = [skeleton_driver]
        else:
            skeleton_driver = self.par_ctrl
            rAttr.Attribute(node=self.part_grp, type='plug', value=[self.par_ctrl],
                            name='skeletonPlugs', children_name=[self.bind_joints[0]])
            driver_list = [self.par_ctrl]

        # driven list usually the first FK ctrl constraint group
        driven_list = [self.base_name + '_01_fk_CTRL_CNST_GRP']

        # pac plugs
        rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list,
                        name='pacRigPlugs', children_name=driven_list)

        # hide list (IK base group usually hidden if using FK by default)
        hide_list = []
        if getattr(self, 'create_ik', False):
            hide_list.append(self.base_name + '_IK_BASE_CTRL_CNST_GRP')

        if hide_list:
            rAttr.Attribute(node=self.part_grp, type='plug', value=[' '.join(hide_list)],
                            name='hideRigPlugs', children_name=['hideNodes'])

        # If both FK and IK present add switch rig plugs (stores switch attr name and control names)
        if self.create_fk and self.create_ik:
            switch_attr = self.part.lower() + self.side.capitalize() + '_IKFK'
            # collect control names safely
            fk_ctrl_names = [getattr(ctrl, 'ctrl_name', None) for ctrl in getattr(self, 'fk_ctrls', [])]
            ik_ctrl_names = [getattr(ctrl, 'ctrl_name', None) for ctrl in getattr(self, 'ik_ctrls', [])]
            ctrl_list = [name for name in (fk_ctrl_names + ik_ctrl_names) if name]
            rAttr.Attribute(node=self.part_grp, type='plug',
                            value=[switch_attr, str(ctrl_list)],
                            name='switchRigPlugs',
                            children_name=['ikFkSwitch', 'ikFKSwitchControls'])

