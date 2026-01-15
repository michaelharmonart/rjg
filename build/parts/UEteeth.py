
import maya.cmds as mc
from importlib import reload
import re

import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
import rjg.build.guide as rGuide
import rjg.libs.transform as rXform
from rjg.build.UEface import UEface
from rjg.libs.profile import auto_profiler_tag
reload(rAttr)
reload(rChain)
reload(rCtrl)
reload (rGuide)
reload(rXform)


class UEteeth(UEface):
    def __init__(self, grp_name=None, ctrl_scale=1, skin=None, toungecurl=True, tongue_spit = True):
        super().__init__(part='Brow', grp_name=grp_name, ctrl_scale=ctrl_scale)
        self.skin = skin
        self.toungecurl = toungecurl
        self.tongue_spit = tongue_spit

    @auto_profiler_tag
    def build(self):
        prefix = UEface.get_prefix_from_group(self.grp_name)
        UEface.Simple_joint_and_Control(
            guide=f'topTeeth',
            overwrite=True,
            overwrite_name=f'TopTeeth',
            orient=True,
            CTRL_Size=.5,
            JNT_Size=0.5,
            CTRL_Color=(1, 0.6, 0),
            bind = False
        )
        UEface.Simple_joint_and_Control(
            guide=f'botTeeth',
            overwrite=True,
            overwrite_name=f'BotTeeth',
            orient=True,
            CTRL_Size=.5,
            JNT_Size=0.5,
            CTRL_Color=(1, 0.6, 0),
            bind = False
        )

        index = 1
        guides = []


        for type in ['top', 'bot']:
            bindjnts = []
            for guide in [f'{type}Teeth_M_Sub_01',f'{type}Teeth_L_Sub_02',f'{type}Teeth_R_Sub_02',f'{type}Teeth_L_Sub_03',f'{type}Teeth_R_Sub_03',]:
                UEface.Simple_joint_and_Control(
                    guide=guide,
                    orient=True,
                    CTRL_Size=.2,
                    JNT_Size=0.5,
                    bind = False
                )
                mc.parent(f'{guide}_JNT', f'{type.capitalize()}Teeth_JNT')
                side = guide.split('_')[1]
                mc.parent(f'{guide}_{side}_CTRL_CNST_GRP', f'{type.capitalize()}Teeth_M_CTRL')
                bindjnts.append(f'{guide}_JNT')
            if self.skin:
                mc.skinCluster(*bindjnts, f'{type}teeth')





        bindjnts = []
        if self.toungecurl:
            curlcontrol =  rCtrl.Control(parent=None, shape="square", side=None, suffix='CTRL', name=f'Tongue_M_Curl', axis='y', group_type='main', rig_type='primary', translate=f'Tongue_01', rotate=None)
            mc.parent(curlcontrol.top, 'RIG')
        if self.tongue_spit:
            split_joint = 'Tongue_01_JNT'


        """split_joint = basejnt
                split_joints: list[str] = [basejnt,mid1jnt,eejnt]
                #mc.addAttr(basejnt, longName="split_joints", niceName="Split Joints", dataType="string")
                #value = f"['{basejnt}','{midjnt}','{eejnt}']"
                #mc.setAttr(f'{basejnt}.split_joints', value, type='string')
                mc.addAttr(split_joint, longName="split_joints", dataType="string")
                mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")"""
            

        while True:
            guide_name = f'Tongue_{index:02d}'  # formats as 01, 02, 03, etc.
            if mc.objExists(guide_name):
                guides.append(guide_name)
                jnt, ctrl, ctrl_offset = UEface.Simple_joint_and_Control(
                    guide=guide_name,
                    overwrite=True,
                    overwrite_name=guide_name,
                    orient=True,
                    CTRL_Size=.5,
                    JNT_Size=0.5,
                    CTRL_Color=(1, 0.6, 0),
                    bind = False
                )
                bindjnts.append(jnt)
                if index != 1:
                    old_index = index - 1 
                    mc.parent(f'{guide_name}_0{index}_CTRL_CNST_GRP', f'{last_guide}_0{old_index}_CTRL')
                    if self.toungecurl:
                        mc.connectAttr(f'{curlcontrol.ctrl}.translate', f'Tongue_{index:02d}_{index:02d}_CTRL_OFF_GRP.translate')
                        mc.connectAttr(f'{curlcontrol.ctrl}.rotate', f'Tongue_{index:02d}_{index:02d}_CTRL_OFF_GRP.rotate')



                last_guide = guide_name
                index += 1
            else:
                break  # stop if the guide doesn't exist
        
        UEface.chain_parts(guides, joints=True, controls=True)
        print(bindjnts)
        mc.skinCluster(*bindjnts, f'tongue')
        if self.tongue_spit:
            split_joints: list[str] = bindjnts
            mc.addAttr(split_joint, longName="split_joints", dataType="string")
            mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")




#mc.skincluster(*combined, self.skin[0])
