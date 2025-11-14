import maya.cmds as mc
from importlib import reload
import re

import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
import rjg.build.guide as rGuide
import rjg.libs.transform as rXform
from rjg.build.UEface import UEface
reload(rAttr)
reload(rChain)
reload(rCtrl)
reload (rGuide)
reload(rXform)


class UEstache(UEface):
    def __init__(self, grp_name=None, ctrl_scale=1, beard=False, Sguide_num=7, Bguide_num=3):
        super().__init__(part='stache', grp_name=grp_name, ctrl_scale=ctrl_scale)
        self.beard=beard
        self.Sguide_num = Sguide_num
        self.Bguide_num = Bguide_num

    def build(self):
        prefix = UEface.get_prefix_from_group(self.grp_name)
        
        main_jnt, main_ctrl, main_offset = UEface.Simple_joint_and_Control(
            guide='Stache_M_01',
            overwrite=True,
            overwrite_name=f'Stache_M_01',
            orient=True,
            CTRL_Size=.5,
            JNT_Size=0.5,
        )
        for side in ['L', 'R']:
            guidelist = []
            prejnt = None
            prectrl = None
            for i in range(2, self.Sguide_num + 1):
                guidelist.append(f"Stache_{side}_{i:02d}")

            for guide in guidelist:
                sub_jnt, sub_ctrl, sub_offset = UEface.Simple_joint_and_Control(
                        guide=guide,
                        overwrite=True,
                        overwrite_name=guide,
                        orient=True,
                        CTRL_Size=.5,
                        JNT_Size=0.5,)

                if prejnt is None:
                    mc.parent(sub_jnt, main_jnt)
                    mc.parent(sub_offset, main_ctrl)
                    prejnt=sub_jnt
                    prectrl=sub_ctrl
                else:
                    mc.parent(sub_jnt, prejnt)
                    mc.parent(sub_offset, prectrl)
                    prejnt=sub_jnt
                    prectrl=sub_ctrl

        if self.beard:
            guidelist = []
            prejnt = None
            for i in ['01', '02', '03']:
                guidelist.append(f"Beard_M_{i}")
            for guide in guidelist:
                sub_jnt, sub_ctrl, sub_offset = UEface.Simple_joint_and_Control(
                        guide=guide,
                        overwrite=True,
                        overwrite_name=guide,
                        orient=True,
                        CTRL_Size=.5,
                        JNT_Size=0.5,)

                if prejnt is None:
                    prejnt=sub_jnt
                    prectrl=sub_ctrl
                else:
                    mc.parent(sub_jnt, prejnt)
                    mc.parent(sub_offset, prectrl)
                    prejnt=sub_jnt
                    prectrl=sub_ctrl




                



    
       

