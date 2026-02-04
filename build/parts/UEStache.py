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


class UEstache(UEface):
    def __init__(self, grp_name=None, ctrl_scale=1, beard=False, Sguide_num=7, Bguide_num=3, skin = True):
        super().__init__(part='stache', grp_name=grp_name, ctrl_scale=ctrl_scale)
        self.beard=beard
        self.Sguide_num = Sguide_num
        self.Bguide_num = Bguide_num
        self.skin = skin
        
    @auto_profiler_tag
    def build(self):
        prefix = UEface.get_prefix_from_group(self.grp_name)

        subgrp = mc.group(empty=True, name=f'{prefix}_subgrp')
        maingrp = mc.group(empty=True, name=f'{prefix}_M')
        mc.parent(subgrp, maingrp)
        mc.parent(maingrp, "RIG")

        pos = mc.xform('Stache_M_01', q=True, ws=True, t=True)
        mc.select(clear=True)
        main_jnt = mc.joint(name='Stache_M_01_JNT', p=pos)
        mc.setAttr(f'{main_jnt}.segmentScaleCompensate', 0)
        bindjnts = []

        for side in ['L', 'R']:
            prejnt=None
            pregrp=None
            for num in ['04', '05', '06', '07']:
                sub_jnt, sub_ctrl, sub_offset = UEface.Simple_joint_and_Control(
                        guide=f'Stache_{side}_{num}',
                        orient=True,
                        CTRL_Size=.5,
                        JNT_Size=0.5,
                        bind=False)

                if prejnt is None:
                    mc.parent(sub_jnt, main_jnt)
                    mc.parent(sub_offset, maingrp)
                    prejnt=sub_jnt
                    pregrp=sub_ctrl
                else:
                    mc.parent(sub_jnt, prejnt)
                    mc.parent(sub_offset, pregrp)
                    prejnt=sub_jnt
                    pregrp=sub_ctrl

                bindjnts.append(sub_jnt)
            
        if self.skin:
            for jnt in ['Cheek_L_NLFold_01_JNT', 'Cheek_L_NLFold_02_JNT', 'Cheek_L_NLFold_03_JNT', 'Cheek_L_NLFold_04_JNT', 'Cheek_L_NLFold_05_JNT', 'Cheek_R_NLFold_01_JNT', 'Cheek_R_NLFold_02_JNT', 'Cheek_R_NLFold_03_JNT', 'Cheek_R_NLFold_04_JNT', 'Cheek_R_NLFold_05_JNT', 'lowermouth_JNT', 'uppermouth_JNT', 'Mouth_L_CornerLip_JNT', 'Mouth_R_CornerLip_JNT', 'Mouth_L_UpperLip_05_JNT', 'Mouth_R_UpperLip_05_JNT', 'Mouth_L_UpperLip_04_JNT', 'Mouth_R_UpperLip_04_JNT',]:
                bindjnts.append(jnt)
            mc.skinCluster(*bindjnts, 'mustache', tsb=True)

        mc.scaleConstraint('head_M_01_CTRL', 'Stache_L_04_L_CTRL')
        mc.scaleConstraint('head_M_01_CTRL', 'Stache_R_04_R_CTRL')

        
        

        if self.beard:
            guidelist = []
            jntlist = []
            prejnt = None
            bindjnts = []
            for i in ['01', '02', '03']:
                guidelist.append(f"Beard_M_{i}")

                if i in ['01', '03']:
                    guide = f"Beard_M_{i}"
                    maj_jnt, maj_ctrl, maj_offset = UEface.Simple_joint_and_Control(
                        guide=guide,
                        overwrite=True,
                        overwrite_name=f"{guide}_Major",
                        orient=True,
                        CTRL_Size=.5,
                        JNT_Size=0.5,
                        bind=False)
                    mc.parent(maj_jnt, maj_offset, maingrp)
                    mc.hide(maj_jnt)
                    jntlist.append(maj_jnt)
                    mc.scaleConstraint('head_M_01_CTRL', maj_offset)
                    




            upper_curve = UEface.build_curve(guidelist, prefix + '_Upper')
            mc.xform(upper_curve, ws=False, t=(1, 0, 0))
            
            lower_curve = UEface.build_curve(guidelist, prefix + '_Lower')
            mc.xform(lower_curve, ws=False, t=(-1, 0, 0))

            loft_surface = mc.loft(upper_curve, lower_curve, ch=True, u=True, c=False, ar=True, d=3, ss=1, rn=False, po=0)[0]
            loft_surface = mc.rename(loft_surface, f'Beard_{side}_ribbon')
            mc.parent(loft_surface, maingrp)
            mc.hide(loft_surface)
            mc.delete(upper_curve, lower_curve)

            mc.select([loft_surface] + jntlist)
            mc.skinCluster(tsb=True)


            for guide in guidelist:
                sub_jnt, sub_ctrl, sub_offset = UEface.Simple_joint_and_Control(
                        guide=guide,
                        overwrite=True,
                        overwrite_name=guide,
                        orient=True,
                        CTRL_Size=.5,
                        JNT_Size=0.5,
                        bind=False)

                if prejnt is None:
                    prejnt=sub_jnt
                else:
                    mc.parent(sub_jnt, prejnt)
                    prejnt=sub_jnt
                
                mc.parent(sub_offset, subgrp)
                bindjnts.append(sub_jnt)

                mc.select(clear=True)
                mc.select(loft_surface)
                mc.select(sub_offset, add=True)
                mc.UVPin()
                mc.scaleConstraint('head_M_01_CTRL', sub_offset)

            if self.skin:
                for jnt in ['Cheek_L_NLFold_01_JNT', 'Cheek_L_NLFold_02_JNT', 'Cheek_L_NLFold_03_JNT', 'Cheek_L_NLFold_04_JNT', 'Cheek_L_NLFold_05_JNT', 'Cheek_R_NLFold_01_JNT', 'Cheek_R_NLFold_02_JNT', 'Cheek_R_NLFold_03_JNT', 'Cheek_R_NLFold_04_JNT', 'Cheek_R_NLFold_05_JNT', 'lowermouth_JNT', 'uppermouth_JNT', 'Mouth_L_CornerLip_JNT', 'Mouth_R_CornerLip_JNT', 'Mouth_L_UpperLip_05_JNT', 'Mouth_R_UpperLip_05_JNT', 'Mouth_L_UpperLip_04_JNT', 'Mouth_R_UpperLip_04_JNT',]:
                    bindjnts.append(jnt)
                mc.skinCluster(*bindjnts, 'beard', tsb=True)

            


            



"""main_jnt, main_ctrl, main_offset = UEface.Simple_joint_and_Control(
            guide='Stache_M_01',
            overwrite=True,
            overwrite_name='Stache_M_01_Major',
            orient=True,
            CTRL_Size=.5,
            JNT_Size=0.5,
            bind=False
        )

        mc.parent(main_offset,main_jnt, maingrp)
        mc.hide(main_jnt)

        mainsub_jnt, mainsub_ctrl, mainsub_offset = UEface.Simple_joint_and_Control(
            guide='Stache_M_01',
            orient=True,
            CTRL_Size=.5,
            JNT_Size=0.5,
        )
        mc.parent(mainsub_offset, subgrp)

        mc.parentConstraint(main_ctrl, mainsub_offset)

        bindjnts = [mainsub_jnt]
        for side in ['L', 'R']:
            guidelist = [f'Stache_{side}_03', f'Stache_{side}_06']
            prejnt = None
            prectrl = None
            jntlist = [main_jnt]

            for guide in guidelist:
                sub_jnt, sub_ctrl, sub_offset = UEface.Simple_joint_and_Control(
                        guide=guide,
                        overwrite=True,
                        overwrite_name=f'{guide}_Major',
                        orient=True,
                        CTRL_Size=.5,
                        JNT_Size=0.5,
                        bind=False)

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

                jntlist.append(sub_jnt)

        
            offsetlist = []
            guidelist = ['Stache_M_01']
            for i in range(2, self.Sguide_num + 1):
                guidelist.append(f"Stache_{side}_{i:02d}")

            upper_curve = UEface.build_curve(guidelist, prefix + '_Upper')
            mc.xform(upper_curve, ws=False, t=(0, 0, 1))
            
            lower_curve = UEface.build_curve(guidelist, prefix + '_Lower')
            mc.xform(lower_curve, ws=False, t=(0, 0, -1))

            loft_surface = mc.loft(upper_curve, lower_curve, ch=True, u=True, c=False, ar=True, d=3, ss=1, rn=False, po=0)[0]
            loft_surface = mc.rename(loft_surface, f'Stache_{side}_ribbon')
            mc.parent(loft_surface, maingrp)
            mc.hide(loft_surface)
            mc.delete(upper_curve, lower_curve)

            mc.select([loft_surface] + jntlist)
            mc.skinCluster(tsb=True)
                


            for guide in guidelist:
                if guide == guidelist[0]:
                    pass
                else:
                    sub_jnt, sub_ctrl, sub_offset = UEface.Simple_joint_and_Control(
                            guide=guide,
                            orient=True,
                            CTRL_Size=.5,
                            JNT_Size=0.5,
                            bind=False)
                    offsetlist.append(sub_offset)
                    bindjnts.append(sub_jnt)

                    mc.parent(sub_jnt, mainsub_jnt)
                    mc.parent(sub_offset, subgrp)

                    mc.select(clear=True)
                    mc.select(loft_surface)
                    mc.select(sub_offset, add=True)
                    mc.UVPin()


        if self.skin:
            for jnt in ['Cheek_L_NLFold_01_JNT', 'Cheek_L_NLFold_02_JNT', 'Cheek_L_NLFold_03_JNT', 'Cheek_L_NLFold_04_JNT', 'Cheek_L_NLFold_05_JNT', 'Cheek_R_NLFold_01_JNT', 'Cheek_R_NLFold_02_JNT', 'Cheek_R_NLFold_03_JNT', 'Cheek_R_NLFold_04_JNT', 'Cheek_R_NLFold_05_JNT', 'lowermouth_JNT', 'uppermouth_JNT', 'Mouth_L_CornerLip_JNT', 'Mouth_R_CornerLip_JNT', 'Mouth_L_UpperLip_05_JNT', 'Mouth_R_UpperLip_05_JNT', 'Mouth_L_UpperLip_04_JNT', 'Mouth_R_UpperLip_04_JNT',]:
                bindjnts.append(jnt)
            mc.skinCluster(*bindjnts, 'mustache', tsb=True)"""



                



    
       

