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

class DragonLeg(rModule.RigModule):
    # constructor. see init parameter comments for info.
    def __init__(self,
                 side=None,             # string. which side of the rig (L, R, M)
                 part=None,             # string. part name (spine, arm, etc.)
                 guide_list=None,       # [string]. list of names of guides to be iterated over during build.
                 ctrl_scale=None,       # float. base scale value for all controls on this part.
                 model_path=None,       # string. path to model.
                 guide_path=None):      # string. path to guides.
        
        # initialize super class RigModule
        super().__init__(side=side,
                     part=part,
                     guide_list=guide_list,
                     ctrl_scale=ctrl_scale,
                     model_path=model_path,
                     guide_path=guide_path)
        
        self.__dict__.update(locals())
        
        # base name conventions: part_side. used for naming objects.
        self.base_name = self.part + '_' + self.side

        self.create_module()

    @staticmethod
    def get_side_from_guide(guide):
            parts = guide.split('_')
            if len(parts) >= 2 and parts[1]:  # Check if there's a second part and it's not empty
                return parts[1]
            else:
                return 'M'
    @staticmethod
    def build_basic_control( name='Main', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=(0, 0, 0), rotation=(0, 0, 0)):
        side = DragonLeg.get_side_from_guide(name)
        rCtrl.ctrl = rCtrl.Control(parent=None, 
                                        shape=shape, 
                                        side=side, 
                                        suffix='CTRL', 
                                        name= name, 
                                        axis='y', 
                                        group_type='main', 
                                        rig_type='primary', 
                                        translate=position, 
                                        rotate=rotation, 
                                        ctrl_scale= size)
        ctrl_name = rCtrl.ctrl.ctrl      # This is the shape transform node (e.g., 'Main_CTRL')
        top_group = rCtrl.ctrl.top       # This is the topmost group (e.g., 'Main_CTRL_OFF')

        return ctrl_name, top_group
        




        # main function to call other functions.
    def control_rig(self):
        ctrlnames = f'{self.side}_CTRL'
        #buildRig
        if self.side == 'L':
            sidelong = 'Left' 
        elif self.side == 'R':
            sidelong = 'Right'
        #FK rig  
        FKpre_jnt = None
        IKpre_jnt = None
        pre_ctrl = None
        for guide in self.guide_list:
            if guide == f'{sidelong}UpLeg':
                ctrlname = f'Leg_{self.side}_01'
                FKjntname = f'Leg_{self.side}_01_FK'
                IKjntname = f'Leg_{self.side}_01_IK'
            elif guide == f'{sidelong}Leg':
                ctrlname = f'Leg_{self.side}_02'
                FKjntname = f'Leg_{self.side}_02_FK'
                IKjntname = f'Leg_{self.side}_02_IK'
            elif guide == f'{sidelong}Knee':
                ctrlname = f'Leg_{self.side}_03'
                FKjntname = f'Leg_{self.side}_03_FK'
                IKjntname = f'Leg_{self.side}_03_IK'
            elif guide == f'{sidelong}Foot':
                ctrlname = f'Leg_{self.side}_04'
                FKjntname = f'Leg_{self.side}_04_FK'
                IKjntname = f'Leg_{self.side}_04_IK'
            elif guide == f'{sidelong}ToeBase':
                ctrlname = f'Foot_{self.side}'
                FKjntname = f'Leg_{self.side}_05_FK'
                IKjntname = f'Leg_{self.side}_05_IK'
            else:
                ctrlname = f'{guide}' 
                FKjntname = f'{guide}_FK'
                IKjntname = f'{guide}_IK'
            pos = mc.xform(guide, q=True, ws=True, t=True)
            rot = mc.xform(guide, q=True, ws=True, ro=True)
            #FK Controls
            ctrl, offset_grp = DragonLeg.build_basic_control(name=ctrlname, shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=rot)
            if pre_ctrl != None:
                mc.parent(offset_grp, pre_ctrl)
                if guide in [f'{sidelong}IndexToe_EE', f'{sidelong}ThumbToe_EE', f'{sidelong}MiddleToe_EE', f'{sidelong}RingToe_EE', f'{sidelong}PinkyToe_EE']:
                    pre_ctrl = f'Foot_{self.side}_{ctrlnames}'
                else:
                    pre_ctrl = ctrl
            else:
                pre_ctrl = ctrl
            mc.select(clear=True)
            #FK Skel
            FKjnt = mc.joint(name=FKjntname, position=pos)
            mc.xform(FKjnt, ws=True, ro=rot)
            mc.makeIdentity(FKjnt, apply=True, r=1, n=0)
            if FKpre_jnt != None:
                mc.parent(FKjnt, FKpre_jnt)
                if guide in [f'{sidelong}IndexToe_EE', f'{sidelong}ThumbToe_EE', f'{sidelong}MiddleToe_EE', f'{sidelong}RingToe_EE', f'{sidelong}PinkyToe_EE']:
                    FKpre_jnt = f'Leg_{self.side}_05_FK'
                else:
                    FKpre_jnt = FKjnt
            else:
                FKpre_jnt = FKjnt
            mc.parentConstraint(ctrl, FKjnt, mo=True)

            #IK Skel
            mc.select(clear=True)
            IKjnt = mc.joint(name=IKjntname, position=pos)
            mc.xform(IKjnt, ws=True, ro=rot)
            mc.makeIdentity(IKjnt, apply=True, r=1, n=0)
            if IKpre_jnt != None:
                mc.parent(IKjnt, IKpre_jnt)
                if guide in [f'{sidelong}IndexToe_EE', f'{sidelong}ThumbToe_EE', f'{sidelong}MiddleToe_EE', f'{sidelong}RingToe_EE', f'{sidelong}PinkyToe_EE']:
                    IKpre_jnt = f'Leg_{self.side}_05_IK'
                else:
                    IKpre_jnt = IKjnt
            else:
                IKpre_jnt = IKjnt
        
        #IK rig
        #leg
        ikh1, eff1 = mc.ikHandle(sj=f'Leg_{self.side}_01_IK', ee=f'Leg_{self.side}_03_IK', sol=f"ikRPsolver", n=f"{self.side}_leg_IK1")
        ikh2, eff2 = mc.ikHandle(sj=f'Leg_{self.side}_02_IK', ee=f'Leg_{self.side}_04_IK', sol=f"ikRPsolver", n=f"{self.side}_leg_IK2")
        pos = mc.xform(f'{sidelong}UpLeg', q=True, ws=True, t=True)
        ctrl, offset_grp = DragonLeg.build_basic_control(name=f'{self.side}_Hip', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
        mc.parentConstraint(ctrl, f'Leg_{self.side}_01_IK', mo=True)

        pos = mc.xform(f'{sidelong}Knee', q=True, ws=True, t=True)
        huckctrl, huckoffset_grp = DragonLeg.build_basic_control(name=f'{self.side}_Huck', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
        mc.parentConstraint(huckctrl, ikh1, mo=True)

        pos = mc.xform(f'{sidelong}Foot', q=True, ws=True, t=True)
        footctrl, footoffset_grp = DragonLeg.build_basic_control(name=f'{self.side}_Foot', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
        mc.parentConstraint(footctrl, ikh2, mo=True)
        mc.orientConstraint(footctrl, f'Leg_{self.side}_04_IK', mo=True)
        pos = mc.xform(f'{sidelong}ToeBase', q=True, ws=True, t=True)
        rootctrl, rootoffset_grp = DragonLeg.build_basic_control(name=f'{self.side}_FootRoot', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
        mc.parent(footoffset_grp, rootctrl)
        mc.parent(huckoffset_grp, rootctrl)

        pos = mc.xform(f'{sidelong}IKAim01', q=True, ws=True, t=True)
        IKAim01ctrl, IKAim01offset_grp = DragonLeg.build_basic_control(name=f'{self.side}_IKAim01', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
        mc.poleVectorConstraint(IKAim01ctrl, ikh1)

        pos = mc.xform(f'{sidelong}IKAim02', q=True, ws=True, t=True)
        IKAim02ctrl, IKAim02offset_grp = DragonLeg.build_basic_control(name=f'{self.side}_IKAim02', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
        mc.poleVectorConstraint(IKAim02ctrl, ikh2)

        #Foot Roll and Bank
        posout = mc.xform(f'{sidelong}Out', q=True, ws=True, t=True)
        posin = mc.xform(f'{sidelong}In', q=True, ws=True, t=True)
        posheel = mc.xform(f'{sidelong}HeelPiv', q=True, ws=True, t=True)
        postoemid = mc.xform(f'{sidelong}Toe_Mid', q=True, ws=True, t=True)
        postoeee = mc.xform(f'{sidelong}Toe_End', q=True, ws=True, t=True)
        poschild = mc.xform(f'{sidelong}ToeBase', q=True, ws=True, t=True)
        midpoint = [(a + b) / 2.0 for a, b in zip(posout, posin)]
        mc.select(clear=True)
        mc.joint(name=f'{self.side}_bankrollparent', position=midpoint)
        mc.joint(name=f'{self.side}_inner_bank', position=posin)
        mc.joint(name=f'{self.side}_outer_bank', position=posout)
        mc.joint(name=f'{self.side}_heel_roll', position=posheel)
        mc.joint(name=f'{self.side}_Toe_EE_roll', position=postoeee)
        mc.joint(name=f'{self.side}_Mid_roll', position=postoemid)
        mc.joint(name=f'{self.side}_child_Piv', position=poschild)

        mc.parentConstraint(f'{self.side}_child_Piv', footoffset_grp, mo=True)
        mc.parentConstraint(f'{self.side}_child_Piv', huckoffset_grp, mo=True)
        #mc.parentConstraint(f'{side}_child_Piv', rootoffset_grp, mo=True) #huckoffset_grp
        mc.parentConstraint(rootctrl, f'{self.side}_bankrollparent', mo=True)
        #mc.parent(f'{side}_inner_bank', rootctrl)

        pos = mc.xform(f'{sidelong}Options', q=True, ws=True, t=True)
        ctrl, offset_grp = DragonLeg.build_basic_control(name=f'{self.side}_FootRoll', shape='circle', size=7.5, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
        mc.addAttr(ctrl, longName="FullRoll_Switch", attributeType="bool", defaultValue=False, keyable=True)
        Roll_Reverse = mc.shadingNode("reverse", asUtility=True, n=f"{self.side}Rol_rev")
        mc.connectAttr(f'{ctrl}.FullRoll_Switch', f'{Roll_Reverse}.inputX')
        #mc.connectAttr(f'{Roll_Reverse}.outputX', f'{footoffset_grp}_parentConstraint1.{side}_child_PivW0')
        mc.connectAttr(f'{ctrl}.FullRoll_Switch', f'{huckoffset_grp}_parentConstraint1.{self.side}_child_PivW0')

        bankin = mc.shadingNode('remapValue', asUtility=True, name=f'{self.side}bankin')
        bankout = mc.shadingNode('remapValue', asUtility=True, name=f'{self.side}bankout')
        rollheel = mc.shadingNode('remapValue', asUtility=True, name=f'{self.side}rollheel')
        rollend = mc.shadingNode('remapValue', asUtility=True, name=f'{self.side}rollend')
        rollmid = mc.shadingNode('remapValue', asUtility=True, name=f'{self.side}rollmid')
        if self.side == 'L':
            mod = 1
        if self.side == 'R':
            mod = -1
        mc.setAttr(f'{bankin}.inputMax', mod * -20)
        mc.setAttr(f'{bankout}.inputMax', mod * 20)
        mc.setAttr(f'{rollheel}.inputMax', -20)
        mc.setAttr(f'{rollmid}.inputMax', 20)
        mc.setAttr(f'{rollend}.inputMin', 20)
        mc.setAttr(f'{rollend}.inputMax', 40)
        mc.setAttr(f'{bankin}.outputMax', mod * 90)
        mc.setAttr(f'{bankout}.outputMax', mod * -90)
        mc.setAttr(f'{rollheel}.outputMax', -90)
        mc.setAttr(f'{rollend}.outputMax', 45)
        mc.setAttr(f'{rollmid}.outputMax', 45)
        for remap in [bankin, bankout]:
            mc.connectAttr(f'{ctrl}.translateX', f'{remap}.inputValue')
        for remap in [rollheel, rollend, rollmid]:
            mc.connectAttr(f'{ctrl}.translateZ', f'{remap}.inputValue')
        mc.connectAttr(f'{bankin}.outValue', f'{self.side}_inner_bank.rotateZ')
        mc.connectAttr(f'{bankout}.outValue', f'{self.side}_outer_bank.rotateZ')
        mc.connectAttr(f'{rollheel}.outValue', f'{self.side}_heel_roll.rotateX')
        mc.connectAttr(f'{rollend}.outValue', f'{self.side}_Toe_EE_roll.rotateX')
        mc.connectAttr(f'{rollmid}.outValue', f'{self.side}_Mid_roll.rotateX')

        mc.parent(offset_grp, rootctrl)

        #IK Toes
        slide1 = mc.shadingNode('remapValue', asUtility=True, name=f'{self.side}toeslide1')
        slide2 = mc.shadingNode('remapValue', asUtility=True, name=f'{self.side}toeslide2')
        mc.setAttr(f'{slide1}.inputMax', 40)
        mc.setAttr(f'{slide1}.outputMax', 10)
        mc.setAttr(f'{slide2}.inputMax', -20)
        mc.setAttr(f'{slide2}.outputMax', -10)
        mc.connectAttr(f'{ctrl}.translateZ', f'{slide1}.inputValue')
        mc.connectAttr(f'{ctrl}.translateZ', f'{slide2}.inputValue')

        for toe in ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']:
            ikhtoe, efftoe = mc.ikHandle(sj=f'{sidelong}{toe}Toe_Root_IK', ee=f'{sidelong}{toe}Toe_EE_IK', sol=f"ikRPsolver", n=f"{self.side}{toe}_IKHandle")
            pos = mc.xform(f'{sidelong}{toe}Toe_EE_IK', q=True, ws=True, t=True)
            ctrl, offset_grp = DragonLeg.build_basic_control(name=f'{sidelong}{toe}Toe_IK', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
            mc.parentConstraint(ctrl, ikhtoe, mo=True)
            mc.parent(offset_grp, rootctrl)
            if toe == 'Thumb':
                slide = slide2
            else:
                slide = slide1
            grp = mc.group(em=True, name=f'{sidelong}{toe}Toe_IK_Slide')
            mc.xform(grp, ws=True, t=pos)
            mc.parent(grp, offset_grp)
            mc.parent(ctrl, grp)
            mc.connectAttr(f'{slide}.outValue', f'{grp}.translateZ')
            pc = mc.parentConstraint(footctrl, offset_grp, mo=True)
            mc.addAttr(ctrl, longName="FollowFoot", attributeType="bool", defaultValue=False, keyable=True)
            weights = mc.parentConstraint(pc[0], q=True, wal=True)[0]
            mc.connectAttr(f'{ctrl}.FollowFoot', f'{pc[0]}.{weights}')

    # create rig systems and joints
    def output_rig(self):
        if self.side == 'L':
            sidelong = 'Left' 
        elif self.side == 'R':
            sidelong = 'Right' 
        ctrlnames = f'{self.side}_CTRL'
        pos = mc.xform(f'{sidelong}Options', q=True, ws=True, t=True)
        ctrl, offset_grp = DragonLeg.build_basic_control(name=f'{self.side}_Options', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
        mc.addAttr(ctrl, longName="FK_IK_Switch", attributeType="bool", defaultValue=False, keyable=True)
        mc.addAttr(ctrl, longName="IKStretch", attributeType="bool", defaultValue=True, keyable=True)
        mc.addAttr(ctrl, longName="Aim_01", at="enum", en="FootRoot:CTRL:World:Root", keyable=True)
        mc.addAttr(ctrl, longName="Aim_02", at="enum", en="FootRoot:CTRL:World:Root", keyable=True)

        #IK Stretch
        prejnt = None
        restlength = 0
        for joint in [f'Leg_{self.side}_01_IK', f'Leg_{self.side}_02_IK', f'Leg_{self.side}_03_IK', f'Leg_{self.side}_04_IK' ]:
            if prejnt != None: 
                p1 = mc.xform(prejnt,   q=True, ws=True, t=True)
                p2 = mc.xform(joint, q=True, ws=True, t=True)
                seg_len = ((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2) ** 0.5
                restlength = restlength + seg_len
                prejnt = joint
            else:
                prejnt = joint

        dist = mc.createNode('distanceBetween', name=f"{self.side}_IKMaths" + "_stretch_DIST")
        mc.connectAttr(f'{self.side}_FootRoot_{ctrlnames}' + '.worldMatrix[0]', dist + '.inMatrix1')
        mc.connectAttr(f'{self.side}_Hip_{ctrlnames}' + '.worldMatrix[0]', dist + '.inMatrix2')
        mdl = mc.createNode('multDoubleLinear', name=f"{self.side}_IKMaths" + "_stretch_MDL")
        mc.setAttr(mdl + '.input1', 1)              # no global scale, just identity
        #mc.connectAttr(scale_attr, mdl + '.input1')
        mc.setAttr(mdl + '.input2', restlength)

        mdn = mc.createNode('multiplyDivide', name=f"{self.side}_IKMaths" + "_stretch_MDN")
        mc.connectAttr(dist + '.distance', mdn + '.input1X')
        mc.connectAttr(mdl + '.output',    mdn + '.input2X')
        mc.setAttr(mdn + '.operation', 2)
        stretch_cond = mc.createNode('condition', name=f"{self.side}_IKMaths" + "_stretch_COND")

        # stretch when distance ≥ rest
        mc.connectAttr(dist + '.distance', stretch_cond + '.firstTerm')
        mc.connectAttr(mdl  + '.output',   stretch_cond + '.secondTerm')
        mc.connectAttr(mdn  + '.outputX',  stretch_cond + '.colorIfTrueR')
        mc.setAttr(stretch_cond + '.operation', 3)   # Greater or Equal
        mc.setAttr(stretch_cond + '.colorIfFalseR', 1)  # neutral value when inactive

        stretch_bta = mc.createNode('blendTwoAttr', name=f"{self.side}_IKMaths" + "_stretch_switch_BTA")

        mc.setAttr(stretch_bta + '.input[0]', 1)
        mc.connectAttr(stretch_cond + '.outColorR', stretch_bta + '.input[1]')
        mc.connectAttr(f'{ctrl}.IKStretch', stretch_bta + '.attributesBlender')


        # ---------------------------------------------------
        # 5) Combine stretch & squash and drive the joints
        # ---------------------------------------------------
        mult = mc.createNode('multiplyDivide', name=f"{self.side}_IKMaths" + '_squash_stretch_MDN')
        mc.connectAttr(stretch_bta + '.output', mult + '.input1X')

        # Apply final factor to the length axis (Y here) of each joint except end
        for joint in [f'Leg_{self.side}_01_IK', f'Leg_{self.side}_02_IK', f'Leg_{self.side}_03_IK']:
            mc.connectAttr(mult + '.outputX', joint + '.scaleY')
        

    # create chain from joints
    def skeleton(self):
        # create chain
        # tag any bind joints
        if self.side == 'L':
            sidelong = 'Left' 
        elif self.side == 'R':
            sidelong = 'Right' 
        pre_jnt = None

        for guide in self.guide_list:
            if guide == f'{sidelong}UpLeg':
                jntname = f'Leg_{self.side}_01_bindJNT'
            elif guide == f'{sidelong}Leg':
                jntname = f'Leg_{self.side}_02_bindJNT'
            elif guide == f'{sidelong}Knee':
                jntname = f'Leg_{self.side}_03_bindJNT'
            elif guide == f'{sidelong}Foot':
                jntname = f'Leg_{self.side}_04_bindJNT'
            elif guide == f'{sidelong}ToeBase':
                jntname = f'Leg_{self.side}_05_bindJNT'
            else:
                jntname = f'{guide}_bindJNT'
            mc.select(clear=True)
            pos = mc.xform(guide, q=True, ws=True, t=True)
            rot = mc.xform(guide, q=True, ws=True, ro=True)
            jnt = mc.joint(name=jntname, position=pos)
            mc.xform(jnt, ws=True, ro=rot)
            mc.makeIdentity(jnt, apply=True, r=1, n=0)
            if pre_jnt != None:
                mc.parent(jnt, pre_jnt)
                if guide in [f'{sidelong}IndexToe_EE', f'{sidelong}ThumbToe_EE', f'{sidelong}MiddleToe_EE', f'{sidelong}RingToe_EE', f'{sidelong}PinkyToe_EE']:
                    pre_jnt = f'Leg_{self.side}_05_bindJNT'
                else:
                    pre_jnt = jnt
            else:
                pre_jnt = jnt

    # create extra attributes to be used during finalization stage (see ../post/finalize.py)
    def add_plugs(self):
        #addPlugs
        if self.side == 'L':
            sidelong = 'Left' 
        elif self.side == 'R':
            sidelong = 'Right'
        grpname = 'CTRL_CNST_GRP'
        ctrlnames = f'{self.side}_CTRL'

        mc.parentConstraint(f'Leg_{self.side}_03_bindJNT', f'{self.side}_Options_{grpname}', mo=True)

        bindjnts = []
        #IKFK Switch
        mc.group(f'Leg_{self.side}_01_{grpname}', name=f'{self.side}_FK_{grpname}')
        mc.group(f'{self.side}_Hip_{grpname}', f'{self.side}_FootRoot_{grpname}', f'{self.side}_IKAim01_{grpname}', f'{self.side}_IKAim02_{grpname}', name=f'{self.side}_IK_{grpname}')
        IK_Reverse = mc.shadingNode("reverse", asUtility=True, n=f"{self.side}ikFk_rev")
        mc.connectAttr(f'{self.side}_Options_{ctrlnames}.FK_IK_Switch', f'{IK_Reverse}.inputX')
        mc.connectAttr(f'{self.side}_Options_{ctrlnames}.FK_IK_Switch', f'{self.side}_FK_{grpname}.visibility')
        mc.connectAttr(f'{IK_Reverse}.outputX', f'{self.side}_IK_{grpname}.visibility')

        for guide in self.guide_list:
            if guide == f'{sidelong}UpLeg':
                basename = f'Leg_{self.side}_01'
            elif guide == f'{sidelong}Leg':
                basename = f'Leg_{self.side}_02'
            elif guide == f'{sidelong}Knee':
                basename = f'Leg_{self.side}_03'
            elif guide == f'{sidelong}Foot':
                basename = f'Leg_{self.side}_04'
            elif guide == f'{sidelong}ToeBase':
                basename = f'Leg_{self.side}_05'
            else:
                basename = f'{guide}'
            
            mc.parentConstraint(f'{basename}_FK', f'{basename}_bindJNT', mo=True)
            mc.parentConstraint(f'{basename}_IK', f'{basename}_bindJNT', mo=True)
            mc.connectAttr(f'{self.side}_Options_{ctrlnames}.FK_IK_Switch', f'{basename}_bindJNT_parentConstraint1.{basename}_FKW0')
            mc.connectAttr(f'{IK_Reverse}.outputX', f'{basename}_bindJNT_parentConstraint1.{basename}_IKW1')
            bindjnts.append(f'{basename}_bindJNT')
        
        #Aim Spaces
        for aim in ['01', '02']:
            pc = mc.parentConstraint(f'{self.side}_FootRoot_{ctrlnames}', f'{self.side}_IKAim{aim}_{grpname}', mo=True)
            weights = mc.parentConstraint(pc[0], q=True, wal=True)
            cond1 = mc.createNode("condition", n=f"{self.side}{aim}foot_cond")
            mc.connectAttr(f'{self.side}_Options_{ctrlnames}.Aim_{aim}', f"{cond1}.firstTerm")
            mc.setAttr(f"{cond1}.secondTerm", 0)
            mc.setAttr(f"{cond1}.operation", 0)
            mc.setAttr(f"{cond1}.colorIfTrueR", 1)
            mc.setAttr(f"{cond1}.colorIfFalseR", 0)
            mc.connectAttr(f"{cond1}.outColorR", f'{self.side}_IKAim{aim}_{grpname}_parentConstraint1.{weights[0]}')
            #L_IKAim01_GRP_parentConstraint1.L_FootRoot_CTRLW0

            control = f"{self.side}_Huck_{ctrlnames}" if aim == "01" else f"{self.side}_Foot_{ctrlnames}"
            pc = mc.parentConstraint(control, f'{self.side}_IKAim{aim}_{grpname}', mo=True)
            weights = mc.parentConstraint(pc[0], q=True, wal=True)
            cond2 = mc.createNode("condition", n=f"{self.side}{aim}{control}_cond")
            mc.connectAttr(f'{self.side}_Options_{ctrlnames}.Aim_{aim}', f"{cond2}.firstTerm")
            mc.setAttr(f"{cond2}.secondTerm", 1)
            mc.setAttr(f"{cond2}.operation", 0)
            mc.setAttr(f"{cond2}.colorIfTrueR", 1)
            mc.setAttr(f"{cond2}.colorIfFalseR", 0)
            mc.connectAttr(f"{cond2}.outColorR", f'{self.side}_IKAim{aim}_{grpname}_parentConstraint1.{weights[1]}')

            if mc.objExists('global_M_CTRL'):
                pc = mc.parentConstraint('global_M_CTRL', f'{self.side}_IKAim{aim}_{grpname}', mo=True)
                weights = mc.parentConstraint(pc[0], q=True, wal=True)
                cond3 = mc.createNode("condition", n=f"{self.side}{aim}world_cond")
                mc.connectAttr(f'{self.side}_Options_{ctrlnames}.Aim_{aim}', f"{cond3}.firstTerm")
                mc.setAttr(f"{cond3}.secondTerm", 3)
                mc.setAttr(f"{cond3}.operation", 0)
                mc.setAttr(f"{cond3}.colorIfTrueR", 1)
                mc.setAttr(f"{cond3}.colorIfFalseR", 0)
                mc.connectAttr(f"{cond3}.outColorR", f'{self.side}_IKAim{aim}_{grpname}_parentConstraint1.{weights[2]}')
            
        mc.group(f'Leg_{self.side}_01_FK', f'Leg_{self.side}_01_IK', f'{self.side}_leg_IK1', f'{self.side}_leg_IK2', f'{self.side}_leg_IK1', f'{self.side}_bankrollparent', f'{sself.ide}Thumb_IKHandle', f'{self.side}Index_IKHandle', f'{self.side}Middle_IKHandle', f'{self.side}Ring_IKHandle', f'{self.side}Pinky_IKHandle', name=f'{self.side}_extra_GRP')
        mc.hide(f'{self.side}_extra_{grpname}')
        mc.group(f'{self.side}_extra_GRP', f'{self.side}_FK_GRP', f'{self.side}_IK_GRP', f'{self.side}_Options_{grpname}', name = f'leg_{self.side}')
        #Plugs f'Leg_{self.side}_01_bindJNT' f'{self.side}_Hip_{grpname}' f'{self.side}_FootRoot_{grpname}' f'Leg_{self.side}_01_{grpname}'
        rAttr.Attribute(node=self.part_grp, type='plug', value=['COG_M_JNT'], name='skeletonPlugs', children_name=[f'Leg_{self.side}_01_bindJNT'])
        driven_list = [f'{self.side}_Hip_{grpname}', f'Leg_{self.side}_01_{grpname}']
        rAttr.Attribute(node=self.part_grp, type='plug', value=['waist_M_CTRL'], name='pacRigPlugs', children_name=driven_list)
        target_list = ['ROOT',
                       'global_M_CTRL',
                       'root_02_M_CTRL',
                       'COG_M_CTRL',
                       'leg_'+ self.side +'_IK_BASE_CTRL',
                       '2']
        name_list = ['world', 'global', 'root', 'hip', 'leg', 'default_value']
        rAttr.Attribute(node=self.part_grp, type='plug', value=[f'{self.side}_FootRoot_{grpname}'], name=self.main_ctrl.ctrl + '_parent', children_name=name_list)

    def create_module(self):

        self.control_rig()
        self.output_rig()
        self.skeleton()
        self.add_plugs()
        # create extra attributes to be used during finalization stage (see ../post/finalize.py)
    
    
    """def add_plugs(self):
            # skeletonPlugs
            # indicate input/output joints in order to connect skeleton components
            # rAttr.Attribute(node=self.part_grp, type='plug', value=['PARENT_JNT'], name='skeletonPlugs', children_name=['CONNECTED_CHILD_JNT'])

            # hideRigPlugs, deleteRigPlugs
            # indicate controls to be hidden or deleted during finalization
            #                                                        space separated list
            # rAttr.Attribute(node=self.part_grp, type='plug', value=[' '.join(hide_list)], name='hideRigPlugs', children_name=['hideNodes'])

            # pacRigPlugs, pacPocRigPlugs, pocRigPlugs, orcRigPlugs
            # parent,      parent no rotate, point,     orient     
            # given a list of drivers and drivens, set up constraints based on these plug attrs
            # rAttr.Attribute(node=self.part_grp, type='plug', value=[driver1, driver2], name='pacRigPlugs', children_name=[driven1, driven2])

            # switchRigPlugs
            # give an object this plug attribute if it has IKFK switch attributes to be connected to the switch control
            # rAttr.Attribute(node=self.part_grp, type='plug', value=[switch_attr], name='switchRigPlugs', children_name=['ikFkSwitch'])

            # transferAttributes
            # tell an object which attributes to transfer to other objects and where
            # rAttr.Attribute(node=self.part_grp, type='plug', value=['transfer target'], name='transferAttributes', children_name=['obj to transfer'])
            pass"""
