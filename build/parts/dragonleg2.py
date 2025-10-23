import maya.cmds as mc
from importlib import reload

import rjg.build.rigModule as rModule
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
import rjg.libs.attribute as rAttr
reload(rModule)
reload(rChain)
reload(rCtrl)
reload(rAttr)


class DragonLeg2(rModule.RigModule):
    def __init__(self, side=None, part=None, guide_list=None, ctrl_scale=None, model_path=None, guide_path=None):
        super().__init__(side=side,
                         part=part,
                         guide_list=guide_list,
                         ctrl_scale=ctrl_scale,
                         model_path=model_path,
                         guide_path=guide_path)

        self.__dict__.update(locals())
        self.base_name = f'{self.part}_{self.side}'
        self.sidelong = 'Left' if self.side == 'L' else 'Right'
        self.create_module()

    # ----------------------------
    # Static helpers
    # ----------------------------
    @staticmethod
    def get_side_from_guide(guide):
        parts = guide.split('_')
        return parts[1] if len(parts) >= 2 and parts[1] else 'M'

    @staticmethod
    def build_basic_control(name='Main', shape='circle', size=5.0, color_rgb=(1, 1, 0),
                            position=(0, 0, 0), rotation=(0, 0, 0), getside=True, sidetype='M'):
        side = DragonLeg2.get_side_from_guide(name) if getside else sidetype
        rCtrl.ctrl = rCtrl.Control(parent=None,
                                   shape=shape,
                                   side=side,
                                   suffix='CTRL',
                                   name=name,
                                   axis='y',
                                   group_type='main',
                                   rig_type='primary',
                                   translate=position,
                                   rotate=rotation,
                                   ctrl_scale=size)
        return rCtrl.ctrl.ctrl, rCtrl.ctrl.top

    @staticmethod
    def create_condition(name, first, second, op=0, true_val=1, false_val=0):
        cond = mc.createNode("condition", name=name)
        mc.setAttr(f'{cond}.firstTerm', first)
        mc.setAttr(f'{cond}.secondTerm', second)
        mc.setAttr(f'{cond}.operation', op)
        mc.setAttr(f'{cond}.colorIfTrueR', true_val)
        mc.setAttr(f'{cond}.colorIfFalseR', false_val)
        return cond

    # ----------------------------
    # Control rig
    # ----------------------------
    def control_rig(self):
        self.fk_pre_jnt = None
        self.ik_pre_jnt = None
        self.pre_ctrl = None

        # Map guides to control/joint base names
        self.guide_map = {
            f'{self.sidelong}UpLeg': 'Leg_01',
            f'{self.sidelong}Leg': 'Leg_02',
            f'{self.sidelong}Knee': 'Leg_03',
            f'{self.sidelong}Foot': 'Leg_04',
            f'{self.sidelong}ToeBase': 'Leg_05'
        }

        for guide in self.guide_list:
            base = self.guide_map.get(guide, guide)
            ctrlname = f'{base}_{self.side}' if 'Leg' in base else base
            FKjntname = f'{base}_{self.side}_FK'
            IKjntname = f'{base}_{self.side}_IK'

            if not mc.objExists(guide):
                continue
            pos = mc.xform(guide, q=True, ws=True, t=True)
            rot = mc.xform(guide, q=True, ws=True, ro=True)

            # FK Control + joint
            ctrl, offset_grp = self.build_basic_control(name=ctrlname, shape='circle', size=10.0, position=pos, rotation=rot)
            self._parent_ctrl(offset_grp, ctrl, guide)
            FKjnt = self._create_joint(FKjntname, pos, rot, self.fk_pre_jnt)
            mc.parentConstraint(ctrl, FKjnt, mo=True)
            self.fk_pre_jnt = FKjnt if guide not in self._toe_end_guides() else f'Leg_{self.side}_05_FK'

            # IK joint
            IKjnt = self._create_joint(IKjntname, pos, rot, self.ik_pre_jnt)
            self.ik_pre_jnt = IKjnt if guide not in self._toe_end_guides() else f'Leg_{self.side}_05_IK'

        self.build_leg_ik_system()

    def _parent_ctrl(self, offset_grp, ctrl, guide):
        if self.pre_ctrl:
            mc.parent(offset_grp, self.pre_ctrl)
            self.pre_ctrl = ctrl
        else:
            self.pre_ctrl = ctrl

    def _create_joint(self, name, pos, rot, parent_jnt=None):
        mc.select(clear=True)
        jnt = mc.joint(name=name, position=pos)
        mc.xform(jnt, ws=True, ro=rot)
        mc.makeIdentity(jnt, apply=True, r=1, n=0)
        if parent_jnt:
            mc.parent(jnt, parent_jnt)
        return jnt

    def _toe_end_guides(self):
        return [f'{self.sidelong}IndexToe_EE', f'{self.sidelong}ThumbToe_EE',
                f'{self.sidelong}MiddleToe_EE', f'{self.sidelong}RingToe_EE', f'{self.sidelong}PinkyToe_EE']

    # ----------------------------
    # Build IK handles, foot, pole vectors
    # ----------------------------
    def build_leg_ik_system(self):
        # IK Handles
        ikh1, eff1 = mc.ikHandle(sj=f'Leg_{self.side}_01_IK', ee=f'Leg_{self.side}_03_IK', sol="ikRPsolver", n=f"{self.side}_leg_IK1")
        ikh2, eff2 = mc.ikHandle(sj=f'Leg_{self.side}_02_IK', ee=f'Leg_{self.side}_04_IK', sol="ikRPsolver", n=f"{self.side}_leg_IK2")

        # Hip
        hip_pos = mc.xform(f'{self.sidelong}UpLeg', q=True, ws=True, t=True)
        hip_ctrl, hip_grp = self.build_basic_control(name=f'{self.side}_Hip', shape='cube', size=10.0, position=hip_pos)
        mc.parentConstraint(hip_ctrl, f'Leg_{self.side}_01_IK', mo=True)

        # Huck / Knee Control
        knee_pos = mc.xform(f'{self.sidelong}Knee', q=True, ws=True, t=True)
        huck_ctrl, huck_grp = self.build_basic_control(name=f'{self.side}_Huck', shape='ZTArrows', size=10.0, position=knee_pos)
        mc.parentConstraint(huck_ctrl, ikh1, mo=True)

        # Foot Control
        foot_pos = mc.xform(f'{self.sidelong}Foot', q=True, ws=True, t=True)
        foot_ctrl, foot_grp = self.build_basic_control(name=f'{self.side}_Foot', shape='ZTArrows', size=10.0, position=foot_pos)
        mc.parentConstraint(foot_ctrl, ikh2, mo=True)
        mc.orientConstraint(foot_ctrl, f'Leg_{self.side}_04_IK', mo=True)

        # Foot root
        toe_base_pos = mc.xform(f'{self.sidelong}ToeBase', q=True, ws=True, t=True)
        root_ctrl, root_grp = self.build_basic_control(name=f'{self.side}_FootRoot', shape='circle', size=10.0, position=toe_base_pos)
        mc.parent(foot_grp, root_ctrl)
        mc.parent(huck_grp, root_ctrl)

        # Pole vectors
        for i, aim_name in enumerate(['IKAim01', 'IKAim02'], start=1):
            aim_pos = mc.xform(f'{self.sidelong}{aim_name[-2:]}', q=True, ws=True, t=True)
            aim_ctrl, aim_grp = self.build_basic_control(name=f'{self.side}_{aim_name}', shape='locator_3D', size=10.0, position=aim_pos)
            ikh = ikh1 if i == 1 else ikh2
            mc.poleVectorConstraint(aim_ctrl, ikh)
            mc.parent(aim_grp, root_ctrl)

        # TODO: Foot roll, bank, toes setup (can refactor similarly)

    # ----------------------------
    # Output rig: IK stretch
    # ----------------------------
    def output_rig(self):
        ctrlnames = f'{self.side}_CTRL'
        pos = mc.xform(f'{self.sidelong}Options', q=True, ws=True, t=True)
        ctrl, offset_grp = self.build_basic_control(name=f'{self.side}_Options', shape='ZTgear', size=10.0, position=pos)
        mc.addAttr(ctrl, longName="FK_IK_Switch", attributeType="bool", defaultValue=False, keyable=True)
        mc.addAttr(ctrl, longName="IKStretch", attributeType="bool", defaultValue=False, keyable=True)

        # IK Stretch setup
        joints = [f'Leg_{self.side}_01_IK', f'Leg_{self.side}_02_IK', f'Leg_{self.side}_03_IK', f'Leg_{self.side}_04_IK']
        restlength = sum([self._distance_between(j1, j2) for j1, j2 in zip(joints[:-1], joints[1:])])

        dist = mc.createNode('distanceBetween', name=f"{self.side}_IKMaths_stretch_DIST")
        mc.connectAttr(f'{self.side}_FootRoot_{ctrlnames}.worldMatrix[0]', dist + '.inMatrix1')
        mc.connectAttr(f'{self.side}_Hip_{ctrlnames}.worldMatrix[0]', dist + '.inMatrix2')

        mdl = mc.createNode('multDL', name=f"{self.side}_IKMaths_stretch_MDL")
        mc.setAttr(mdl + '.input1', 1)
        mc.setAttr(mdl + '.input2', restlength)

        mdn = mc.createNode('multiplyDivide', name=f"{self.side}_IKMaths_stretch_MDN")
        mc.connectAttr(dist + '.distance', mdn + '.input1X')
        mc.connectAttr(mdl + '.output', mdn + '.input2X')
        mc.setAttr(mdn + '.operation', 2)

        stretch_cond = mc.createNode('condition', name=f"{self.side}_IKMaths_stretch_COND")
        mc.connectAttr(dist + '.distance', stretch_cond + '.firstTerm')
        mc.connectAttr(mdl + '.output', stretch_cond + '.secondTerm')
        mc.connectAttr(mdn + '.outputX', stretch_cond + '.colorIfTrueR')
        mc.setAttr(stretch_cond + '.operation', 3)
        mc.setAttr(stretch_cond + '.colorIfFalseR', 1)

        stretch_bta = mc.createNode('blendTwoAttr', name=f"{self.side}_IKMaths_stretch_switch_BTA")
        mc.setAttr(stretch_bta + '.input[0]', 1)
        mc.connectAttr(stretch_cond + '.outColorR', stretch_bta + '.input[1]')
        mc.connectAttr(ctrl + '.IKStretch', stretch_bta + '.attributesBlender')

        mult = mc.createNode('multiplyDivide', name=f"{self.side}_IKMaths_squash_stretch_MDN")
        mc.connectAttr(stretch_bta + '.output', mult + '.input1X')

        for j in joints[:-1]:
            mc.connectAttr(mult + '.outputX', j + '.scaleY')

    def _distance_between(self, j1, j2):
        p1 = mc.xform(j1, q=True, ws=True, t=True)
        p2 = mc.xform(j2, q=True, ws=True, t=True)
        return sum([(a - b) ** 2 for a, b in zip(p1, p2)]) ** 0.5
