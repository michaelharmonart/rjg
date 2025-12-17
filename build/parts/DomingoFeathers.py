import maya.cmds as mc
from importlib import reload
import re
import math

import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
import rjg.build.guide as rGuide
import rjg.libs.transform as rXform
from rjg.build.UEface import UEface
reload(rAttr)
reload(rChain)
reload(rCtrl)
reload(rGuide)
reload(rXform)


class DomingoFeathers(UEface):
    def __init__(self, grp_name=None, ctrl_scale=1,):
        super().__init__(part='domfeathers', grp_name=grp_name, ctrl_scale=ctrl_scale)
        self.grp_name = grp_name
        #group='Wing_L_guides'

    # @staticmethod
    # def get_namestruc(prefix = 'Wing_L', rjg=True):
    #     if rjg == False: 
    #         ctrlname = 'CTRL'
    #         grpname = 'GRP'
    #     else:
    #         parts = prefix.split("_")   # ["wing", "L"]
    #         side = parts[-1]            # "L"
    #         ctrlname = f'{side}_CTRL'
    #         grpname = f'{side}_CTRL_CNST_GRP'
    #     return ctrlname, grpname

    

    # @staticmethod
    # def build_curve(guide_list, prefix, degree=3):
    #     ctrlname, grpname = DomingoFeathers.get_namestruc(prefix)
    #     """
    #     Builds a NURBS curve using a list of guide names (in order), with a given prefix.

    #     Args:
    #         guide_list (list): Ordered list of guide names (strings).
    #         prefix (str): Prefix for the curve name (e.g., 'Eyelid', 'Brow', 'Wing').
    #         degree (int): Degree of the curve (default = 3).

    #     Returns:
    #         str: The name of the created curve.
    #     """
    #     if not guide_list or len(guide_list) < 2:
    #         print(f"[WARN] Not enough guides to build a curve for prefix '{prefix}'")
    #         return None

    #     # Get world positions from guides
    #     try:
    #         points = [mc.xform(g, q=True, ws=True, t=True) for g in guide_list]
    #     except Exception as e:
    #         print(f"[ERROR] Failed to query guide transforms: {e}")
    #         return None

    #     # Choose linear curve if not enough points for specified degree
    #     curve_degree = min(degree, len(points) - 1)

    #     # Build curve
    #     curve_name = f"{prefix}_curve"
    #     result = mc.curve(name=curve_name, degree=curve_degree, point=points)
    #     print(f"[INFO] Created curve: {result}")
    #     return result
    
    # @staticmethod
    # def look_at_rotation(pos1, pos2):
    #     """
    #     Returns rotation (rx, ry, rz) in degrees so Z+ points from pos1 to pos2.
    #     """
    #     dx = pos2[0] - pos1[0]
    #     dy = pos2[1] - pos1[1]
    #     dz = pos2[2] - pos1[2]

    #     # Yaw (rotation around Y axis)
    #     ry = math.degrees(math.atan2(dx, dz))  # note order: x/z

    #     # Pitch (rotation around X axis)
    #     dist_xz = math.sqrt(dx*dx + dz*dz)
    #     rx = -math.degrees(math.atan2(dy, dist_xz))

    #     # Roll is 0 if you don't care
    #     rz = 0

    #     return (rx, ry, rz)

    # @staticmethod
    # def count_feather_guides(prefix, feather):
    #     ctrlname, grpname = DomingoFeathers.get_namestruc(prefix)
    #     count = 0
    #     guides = []
        
    #     # We'll just loop up to some reasonable high number to check
    #     for i in range(1, 200):  
    #         num_str = f"{i:02}" if i < 10 else str(i)  # pad with zero if < 10
    #         name = f"{prefix}_{feather}_Root_{num_str}"
            
    #         if mc.objExists(name):
    #             guides.append(name)
    #             count += 1
        
    #     print(f"Found {count} feather guide(s): {guides}")
    #     return count, guides

    @staticmethod
    def get_namestruc(prefix = 'Wing_L', rjg=True):
        if rjg == False: 
            ctrlname = 'CTRL'
            grpname = 'GRP'
        else:
            parts = prefix.split("_")   # ["wing", "L"]
            side = parts[-1]            # "L"
            ctrlname = f'{side}_CTRL'
            grpname = f'{side}_CTRL_CNST_GRP'
        return ctrlname, grpname


    @staticmethod
    def build_curve(guide_list, prefix, degree=3,):
        ctrlname, grpname = DomingoFeathers.get_namestruc(prefix)
        """
        Builds a NURBS curve using a list of guide names (in order), with a given prefix.

        Args:
            guide_list (list): Ordered list of guide names (strings).
            prefix (str): Prefix for the curve name (e.g., 'Eyelid', 'Brow', 'Wing').
            degree (int): Degree of the curve (default = 3).

        Returns:
            str: The name of the created curve.
        """
        if not guide_list or len(guide_list) < 2:
            print(f"[WARN] Not enough guides to build a curve for prefix '{prefix}'")
            return None

        # Get world positions from guides
        try:
            points = [mc.xform(g, q=True, ws=True, t=True) for g in guide_list]
        except Exception as e:
            print(f"[ERROR] Failed to query guide transforms: {e}")
            return None

        # Choose linear curve if not enough points for specified degree
        curve_degree = min(degree, len(points) - 1)

        # Build curve
        curve_name = f"{prefix}_curve"
        result = mc.curve(name=curve_name, degree=curve_degree, point=points)
        print(f"[INFO] Created curve: {result}")
        return result
    
    @staticmethod
    def look_at_rotation(pos1, pos2):
        """
        Returns rotation (rx, ry, rz) in degrees so Z+ points from pos1 to pos2.
        """
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        dz = pos2[2] - pos1[2]

        # Yaw (rotation around Y axis)
        ry = math.degrees(math.atan2(dx, dz))  # note order: x/z

        # Pitch (rotation around X axis)
        dist_xz = math.sqrt(dx*dx + dz*dz)
        rx = -math.degrees(math.atan2(dy, dist_xz))

        # Roll is 0 if you don't care
        rz = 0

        return (rx, ry, rz)

    @staticmethod
    def count_feather_guides(prefix, feather):
        ctrlname, grpname = DomingoFeathers.get_namestruc(prefix)
        count = 0
        guides = []
        
        # We'll just loop up to some reasonable high number to check #{i:02d}
        for i in range(1, 200):  
            num_str = f"{i:02}" if i < 10 else str(i)  # pad with zero if < 10
            name = f"{prefix}_{feather}_{num_str}_guide"
            
            if mc.objExists(name):
                guides.append(name)
                count += 1
        
        print(f"Found {count} feather guide(s): {guides}")
        return count, guides


    @staticmethod
    def build_stretchy_spline(source='GuideList', offsetname=None, inputlist = [], inputcurve = None, buildControls=True, guidecurve=None, feathernum=None, Stretch=True, StretchControl = None, prefix=None, autoconstrain=True, finalclustertwist=False, featherrot=[0,0,0] ):
        #'GuideList', 'Curve', 'jointlist'
        ctrlname, grpname = DomingoFeathers.get_namestruc(prefix)
        parts = prefix.split("_")   # ["wing", "L"]
        side = parts[-1]

        if source == 'GuideList':
            prejnt = None
            prepos = None
            jntlist = []
            for i, guide in enumerate(inputlist, start=1):
                mc.select(clear=True)
                pos = mc.xform(guide, q=True, ws=True, t=True)
                if feathernum:
                    idx = f"{i:02d}"   # pads to 2 digits → 01, 02, 03...
                    jnt = mc.joint(name=f"feather{side}_{feathernum}_{idx}_jnt")
                    jntlist.append(jnt)
                elif guidecurve:
                    idx = f"{i:02d}"
                    jnt = mc.joint(name=f'{guidecurve}_{side}_{idx}_jnt')
                    jntlist.append(jnt)
                else:
                    jnt = mc.joint(name=f'{guide}_{side}_{offsetname}_jnt')
                    jntlist.append(jnt)
                    
                
                if prejnt:
                    rot = DomingoFeathers.look_at_rotation(pos, prepos)
                else:
                    posfuture = mc.xform(inputlist[1], q=True, ws=True, t=True)
                    rot = DomingoFeathers.look_at_rotation(posfuture, pos)

                mc.xform(jnt, ws=True, t=pos)
                #mc.xform(jnt, ws=True, ro=rot)
                mc.setAttr(f"{jnt}.jointOrientX", rot[0])
                mc.setAttr(f"{jnt}.jointOrientY", rot[1])
                mc.setAttr(f"{jnt}.jointOrientZ", rot[2])

                if prejnt:
                    mc.parent(jnt, prejnt)
                
                prejnt = jnt
                prepos = pos
            
            #######################################################
        ik_handle, effector, curve = mc.ikHandle(
            sj=jntlist[0],
            ee=jntlist[-1],
            sol='ikSplineSolver',
            ccv=True,
            pcv=False
        )

        if feathernum:
            name = f"{side}_feather{feathernum}_spline"
        elif guidecurve:
            name = f"{side}_{guidecurve}_spline"
        else:
            name = f"{side}_{jntlist[0]}_spline"

        curve = mc.rename(curve, f'{side}_{name}_curve')
        ik_handle = mc.rename(ik_handle, f'{side}_{name}_handle')
        mc.parent(ik_handle, f'{prefix}_handle_{grpname}')
        mc.parent(curve, f'{prefix}_handle_{grpname}')


        if guidecurve:
            mc.rebuildCurve(
                curve,
                ch=False,   # no construction history
                rpo=True,   # keep original position
                rt=0,
                end=1,
                kr=0,
                kcp=0,
                kep=1,
                kt=0,
                s=5,        # spans (5 + degree 3 = 8 CVs)
                d=3         # cubic curve
                )


        # Step 2: For each CV on the curve, create cluster + control
        cvs = mc.ls(f"{curve}.cv[*]", fl=True)
        ctrl_list = []
        offset_list = []
        print(jntlist)
        
        for i, cv in enumerate(cvs, start=1):
            # Make cluster for the CV
            cluster, cluster_handle = mc.cluster(cv, n=f"{name}_{i:02}_cluster")
            mc.parent(cluster_handle, f'{prefix}_handle_{grpname}')
            # Get cluster position
            pos = mc.pointPosition(cv, w=True)
            if buildControls:
                # Make control
                if feathernum:
                    ctrl_name = f"feather_{side}_{feathernum}_{i:02}"
                elif guidecurve:
                    ctrl_name = f"{guidecurve}_{side}_{i:02}"
                else:
                    ctrl_name = f"{jntlist[0]}_{side}_{i:02}"
                #ctrl_name = f"{prefix}_Main_Feather_aim_{i:02}"
                ctrl, offset = UEface.build_basic_control(
                    name=ctrl_name,
                    shape='ZTsphere',
                    size=2.5,
                    color_rgb=(1, 1, 0),
                    position=pos,
                    rotation=(0, 0, 0)
                )
                ctrl_list.append(ctrl)
                offset_list.append(offset)
                mc.parent(offset, f'{prefix}_feather_{grpname}')

                # Parent cluster to control
                mc.parentConstraint(ctrl, cluster_handle, mo=True)

            if autoconstrain and feathernum:
                if i == 1:
                    mc.parentConstraint(f'main_{side}_{feathernum}_jnt', cluster_handle, mo=True)
                elif i == 2:
                    mc.parentConstraint(f'main_{side}_{feathernum}_jnt', cluster_handle, mo=True)
                    mc.parentConstraint(f'mid_{side}_{feathernum}_jnt', cluster_handle, mo=True)
                elif i == 3:
                    mc.parentConstraint(f'aim_{side}_{feathernum}_jnt', cluster_handle, mo=True)
                    mc.parentConstraint(f'mid_{side}_{feathernum}_jnt', cluster_handle, mo=True)
                else:
                    clusteroffset = mc.group(empty=True)
                    clusteroffset2 = mc.group(empty=True)
                    mc.xform(clusteroffset, ws=True, t=pos, ro=rot)
                    mc.xform(clusteroffset2, ws=True, t=pos, ro=rot)
                    mc.parent(clusteroffset, clusteroffset2)
                    mc.parent(cluster_handle, clusteroffset)
                    mc.parent(clusteroffset2, f'{prefix}_handle_{grpname}' )
                    mc.parentConstraint(f'aim_{side}_{feathernum}_jnt', clusteroffset, mo=True)
            if finalclustertwist:
                if i == 4:
                    mc.addAttr(StretchControl, ln='twist', at='double', k=True)
                    mc.addAttr(StretchControl, ln='roll', at='double', k=True)
                    #mc.connectAttr(f'{StretchControl}.roll', f'{ik_handle}.roll')
                    mc.connectAttr(f'{StretchControl}.twist', f'{ik_handle}.twist')
                    rolladl = mc.createNode("addDL", name=f"{ik_handle}_rolladl")
                    rotadl = mc.createNode("addDL", name=f"{ik_handle}_rotadl")
                    mc.addAttr(StretchControl, ln='autoroll_mult', at='double', k=False)
                    try:
                        mod = mc.getAttr(f"Wing_{side}_MainFeather_{num}_guide.autoroll_mult")
                    except:
                        mod = 0.5

                    if side == 'R':
                        mod = mod * -1
                    flipnode = mc.createNode("multiplyDivide", name=f"{ik_handle}_rollflip")
                    #autoattr = f'{flipnode}.outputX'
                    mc.connectAttr(f"{clusteroffset}.rotateX", f'{flipnode}.input1X')
                    mc.connectAttr(f"{clusteroffset}.rotateZ", f'{flipnode}.input1Z')
                    mc.connectAttr(f"{flipnode}.outputX", f'{rotadl}.input1')
                    mc.connectAttr(f"{flipnode}.outputZ", f'{rotadl}.input2')
                    autoattr = f'{rotadl}.output'
                    

                    mc.connectAttr(f'{StretchControl}.autoroll_mult', f'{flipnode}.input2X')
                    mc.connectAttr(f'{StretchControl}.autoroll_mult', f'{flipnode}.input2Z')
                    #mc.setAttr( f'{flipnode}.input2X', mod)
                    #mc.setAttr( f'{flipnode}.input2Z', mod)
                    #else:
                    #    #autoattr = f"{clusteroffset}.rotateX"
                    #    #mc.connectAttr(f"{clusteroffset}.rotateX", f'{rotadl}.input1')
                    #    #mc.connectAttr(f"{clusteroffset}.rotateZ", f'{rotadl}.input2')
                    #    #autoattr = f'{rotadl}.output'
                        
                    mc.connectAttr(autoattr, f'{rolladl}.input1')
                    mc.connectAttr(f'{StretchControl}.roll', f'{rolladl}.input2')
                    mc.connectAttr(f'{rolladl}.output', f'{ik_handle}.roll')


                



                #close_offset = mc.group(empty=True, name=f'{prefix}Aim_{i:02}_ArmClose_offset')
                #mc.xform(close_offset, ws=True, t=pos,)
                #mc.parent(close_offset, offset)
                #mc.parent(ctrl, close_offset)

            if Stretch:
                if buildControls:
                    if i == 1:
                        mc.addAttr(ctrl, ln='stretchy', at='double', dv=1, k=True, max=1, min=0 )
                        Stretch_attr = f'{ctrl_list[0]}.stretchy'
                    else:
                        mc.addAttr(ctrl, ln='stretchy', proxy=f'{ctrl_list[0]}.stretchy') #ctrl_list
                elif StretchControl:
                    if i == 1:
                        mc.addAttr(StretchControl, ln='stretchy', at='double', dv=1, k=True, max=1, min=0 )
                        Stretch_attr = f'{StretchControl}.stretchy'
        if Stretch:
            if Stretch_attr:
                postcurve = mc.listRelatives(curve, s=True, ni=True)[0]
                precurve = mc.listRelatives(curve, s=True, ni=False, type='nurbsCurve')
                precurve = [s for s in precurve if mc.getAttr(s + ".intermediateObject")]
                precurve = precurve[0]

                postci = mc.createNode("curveInfo", name=f"{curve}_postci")
                preci = mc.createNode("curveInfo", name=f"{curve}_preci")
                mc.connectAttr(f"{postcurve}.worldSpace[0]", f"{postci}.inputCurve", force=True)
                mc.connectAttr(f"{precurve}.worldSpace[0]", f"{preci}.inputCurve", force=True)

                frac = mc.createNode("multiplyDivide", name=f"{curve}_Frac")

                # Set the operation to DIVIDE (2)
                mc.setAttr(f"{frac}.operation", 2)

                # Connect inputs
                mc.connectAttr(f"{postci}.arcLength", f"{frac}.input1X", force=True)
                mc.connectAttr(f"{preci}.arcLength", f"{frac}.input2X", force=True) 

                md = mc.createNode("multiplyDivide", name=f"{curve}_MD")

                mc.connectAttr(f"{frac}.outputX", f"{md}.input1X", force=True)
                mc.connectAttr(Stretch_attr, f"{md}.input2X", force=True)

                for newjnt in jntlist:
                    mc.connectAttr(f"{md}.outputX", f"{newjnt}.scaleZ")

            else:
                postcurve = mc.listRelatives(curve, s=True, ni=True)[0]
                precurve = mc.listRelatives(curve, s=True, ni=False, type='nurbsCurve')
                precurve = [s for s in precurve if mc.getAttr(s + ".intermediateObject")]
                precurve = precurve[0]

                postci = mc.createNode("curveInfo", name=f"{curve}_postci")
                preci = mc.createNode("curveInfo", name=f"{curve}_preci")
                mc.connectAttr(f"{postcurve}.worldSpace[0]", f"{postci}.inputCurve", force=True)
                mc.connectAttr(f"{precurve}.worldSpace[0]", f"{preci}.inputCurve", force=True)

                frac = mc.createNode("multiplyDivide", name=f"{curve}_Frac")

                # Set the operation to DIVIDE (2)
                mc.setAttr(f"{frac}.operation", 2)

                # Connect inputs
                mc.connectAttr(f"{postci}.arcLengtht", f"{frac}.input1X", force=True)
                mc.connectAttr(f"{preci}.arcLength", f"{frac}.input2X", force=True) 

                #md = mc.createNode("multiplyDivide", name=f"{curve}_MD")

                #mc.connectAttr(f"{frac}.outputX", f"{md}.input1X", force=True)
                #mc.connectAttr(Stretch_attr, f"{md}.input2X", force=True)

                for newjnt in jntlist:
                    mc.connectAttr(f"{frac}.outputX", f"{newjnt}.scaleZ")

        mc.parent(jntlist[0], f'{prefix}_net_{grpname}')
        mc.setAttr(f'{jntlist[0]}.template', 1)
        
        return ik_handle, curve, ctrl_list, offset_list, jntlist

    

    def build_wing(self):
        #group='Wing_L_guides'
        prefix = UEface.get_prefix_from_group(self.grp_name)
        grp = self.grp_name
        ctrlname, grpname = DomingoFeathers.get_namestruc(prefix)
        parts = prefix.split("_")   # ["wing", "L"]
        side = parts[-1]

        #Build Groups
        module_grp_name = mc.group(em=True, name=f'{prefix}_{grpname}')
        feather_grp = mc.group(em=True, name=f'{prefix}_feather_{grpname}')
        handle_grp = mc.group(em=True, name=f'{prefix}_handle_{grpname}')
        upAim_grp = mc.group(em=True, name=f'{prefix}_upAim_{grpname}')
        netgrp = mc.group(em=True, name=f'{prefix}_net_{grpname}')
        mc.select(clear=True)
        
        #if side == 'L':
        #    pos = mc.xform(f'LeftArm', q=True, ws=True, t=True)
        #elif side == 'R':
        #    pos = mc.xform(f'RightArm', q=True, ws=True, t=True)
        #else:
        #    pos = (0,0,0)
        #root_joint = mc.joint(name=f'{prefix}_root_jnt', p=pos)

        #Build Shaper and Span controls
        #build_shaper_and_span_controls(prefix)

        maincount, mainguides = DomingoFeathers.count_feather_guides(prefix = prefix, feather='MainFeather')

        buildType = 'splines'
        
        if buildType == 'splines':
            aim_list = []
            mid_list = []
            root_list = []
            for i in range(1, maincount + 1):
                new_num = f'{i:02d}'
                aim_list.append(f'{prefix}_MainFeather_{new_num}_aim') #Wing_L_MainFeather_07_aim
                mid_list.append(f'{prefix}_MainFeather_{new_num}_ee_guide')  #Wing_L_MainFeather_07_ee_guide
                root_list.append(f'{prefix}_MainFeather_{new_num}_guide') #Wing_L_MainFeather_07_guide

            #Feathershaping

            mainik_handle, maincurve, mainctrl_list, mainoffset_list, mainjntlist = DomingoFeathers.build_stretchy_spline(source='GuideList',  inputlist = root_list, inputcurve = None, buildControls=True, guidecurve='main', Stretch=True, prefix=prefix)
            midik_handle, midcurve, midctrl_list, midoffset_list, midjntlist = DomingoFeathers.build_stretchy_spline(source='GuideList',  inputlist = mid_list, inputcurve = None, buildControls=True, guidecurve='mid', Stretch=True, prefix=prefix)
            aimik_handle, aimcurve, aimctrl_list, aimoffset_list, aimjntlist = DomingoFeathers.build_stretchy_spline(source='GuideList',  inputlist = aim_list, inputcurve = None, buildControls=True, guidecurve='aim', Stretch=True, prefix=prefix)

            #bind
            bind_joints = [f'arm_{side}_01_JNT', f'arm_{side}_02_JNT', f'arm_{side}_03_JNT', f'arm_{side}_04_JNT', f'arm_{side}_05_JNT', f'arm_{side}_06_JNT', f'arm_{side}_07_JNT', f'arm_{side}_08_JNT']
            # pre_jnt = None
            # for obj in [f'{prefix}_01_guide', f'{prefix}_02_guide', f'{prefix}_03_guide', f'{prefix}_04_guide']:
            #     # Get the base name and generate joint name
            #     base_name = obj.split('|')[-1].replace('_guide', '')
            #     joint_name = f"{base_name}_bind_jnt"

            #     # Clear selection before creating the joint to avoid parenting
            #     mc.select(clear=True)
            #     joint = mc.joint(name=joint_name)
            #     bind_joints.append(joint)

            #     # Match translation and rotation in world space
            #     pos = mc.xform(obj, q=True, ws=True, t=True)
            #     rot = mc.xform(obj, q=True, ws=True, ro=True)
            #     mc.xform(joint, ws=True, t=pos)
            #     mc.xform(joint, ws=True, ro=rot)
            #     if pre_jnt != None:
            #         mc.parent(joint_name, pre_jnt)
            #     pre_jnt = joint_name
            # #mc.skinCluster(bind_joints, main_surf)
            pre_jnt = None

            parjnts = ['01', '02', '03', '04' , '05', '06', '07', '08', '09']

            #Build Feather :)
            rot_offset_list = []
            base_offsets = []
            def_jnts = []
            for guide in mainguides:
                num = guide.split("_")[-2]
                ee_guide = f'{prefix}_MainFeather_{num}_ee_guide'
                aim_guide = f'{prefix}_MainFeather_{num}_aim'
                #if side == 'R':
                    #y = mc.getAttr(ee_guide + ".translateY")
                    #mc.setAttr(ee_guide + ".translateY", -y)
                
                if mc.attributeQuery('parent_joint', node=guide, exists=True):
                    tempnum = mc.getAttr(f'{guide}.parent_joint')
                    root_num = parjnts[tempnum]
                else:
                    root_num = '01'

                obj = f'{prefix}_{root_num}_guide'
                # Get the base name and generate joint name
                base_name = obj.split('|')[-1].replace('_guide', '')
                root_joint = f'arm_{side}_{root_num}_JNT'

                basepos = mc.xform(guide, q=True, ws=True, t=True)
                eepos = mc.xform(ee_guide, q=True, ws=True, t=True)
                #if side == 'R':
                #    eepos[0] = -eepos[0]  # mirror across X axis
                mid1pos = [basepos[i] + (eepos[i] - basepos[i]) * .5 for i in range(3)]
                #mid1pos = [basepos[i] + (eepos[i] - basepos[i]) * (1/3) for i in range(3)]
                rot = mc.xform(guide, q=True, ws=True, ro=True)
                mid1_guide = mc.spaceLocator(p=mid1pos, name=f"interp_locator{guide}")[0]
                #mid2_guide = mc.spaceLocator(p=mid2pos, name=f"interp_locator{guide}")[0]
                mc.xform(mid1_guide, ws=True, ro=rot, t=mid1pos)
                #mc.xform(mid2_guide, ws=True, ro=rot, t=mid2pos)

                #main
                main_rot = rot #[a + b for a, b in zip(rot, add)]
                Main_Ctrl, Main_Group = UEface.build_basic_control( name=f'{prefix}_Feather_{num}', shape='ZTpoint', size=2.5, position=basepos, rotation=main_rot)
                mc.parent(Main_Group, feather_grp)
                for ax in ["X", "Y", "Z"]:
                    mc.setAttr(f'{Main_Ctrl}.translate{ax}', lock=True, channelBox=False)
                    mc.setAttr(f'{Main_Ctrl}.scale{ax}', lock=True, channelBox=False)

                feather_list = [guide, mid1_guide, ee_guide, aim_guide]
                featherik_handle, feathercurve, featherctrl_list, featheroffset_list, featherjntlist = DomingoFeathers.build_stretchy_spline(source='GuideList', offsetname=None, inputlist = feather_list, inputcurve = None, buildControls=False, guidecurve=None, feathernum=num, Stretch=True, StretchControl = Main_Ctrl, prefix=prefix, autoconstrain=True, finalclustertwist=True, featherrot = rot)

                basejnt, basectrl, basectrl_offset =UEface.Simple_joint_and_Control(
                    guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}MainFeather_{num}_base',
                    scale=True,
                    check_side=True,
                    CTRL_Size=1,
                    JNT_Size=0.5
                )

                mid1jnt, mid1ctrl, mid1ctrl_offset =UEface.Simple_joint_and_Control(
                    mid1_guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}MainFeather_{num}_mid1',
                    scale=True,
                    check_side=True,
                    CTRL_Size=1,
                    JNT_Size=0.5
                )
                eejnt, eectrl, eectrl_offset =UEface.Simple_joint_and_Control(
                    ee_guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}MainFeather_{num}_ee',
                    scale=True,
                    check_side=True,
                    CTRL_Size=1,
                    JNT_Size=0.5
                )

                split_joint = basejnt
                split_joints: list[str] = [basejnt,mid1jnt,eejnt]
                #mc.addAttr(basejnt, longName="split_joints", niceName="Split Joints", dataType="string")
                #value = f"['{basejnt}','{midjnt}','{eejnt}']"
                #mc.setAttr(f'{basejnt}.split_joints', value, type='string')
                mc.addAttr(split_joint, longName="split_joints", dataType="string")
                mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")

                '''split_joints: list[str] = bind_joints
                    mc.addAttr(split_joint, longName="split_joints", dataType="string")
                    mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")'''

                mc.pointConstraint(basectrl, Main_Group, mo=True)
                #mc.pointConstraint(basectrl, aim_loc_off, mo=True)
                pre_jnt = None
                pre_ctrl = None
                for part in [guide, mid1_guide, ee_guide]:
                    if part == guide: 
                        trans = basepos
                        nameing = 'base'
                        offset = basectrl_offset
                        ctrl = basectrl
                        jnt = basejnt
                        mdspot = 'Y'
                    elif part == mid1_guide:
                        trans = mid1pos
                        nameing = 'mid1'
                        offset = mid1ctrl_offset
                        ctrl = mid1ctrl
                        jnt = mid1jnt
                        mdspot = 'X'
                    else:
                        trans = eepos 
                        nameing = 'ee'
                        offset = eectrl_offset
                        ctrl = eectrl
                        jnt = eejnt
                        mdspot = 'Z'
                    rot_offset = mc.group(empty=True, name=f'{prefix}_MainFeather_{num}_{nameing}_rotOffset')
                    mc.xform(rot_offset, ws=True, t=trans, ro=rot)
                    mc.parent(rot_offset, offset)
                    mc.parent(ctrl, rot_offset)
                    rot_offset_list.append(rot_offset)


                    if pre_jnt != None:
                        mc.parent(jnt, pre_jnt)
                        #mc.parentConstraint(pre_ctrl, offset, mo=True)
                        mc.parent(offset,feather_grp)
                        pre_jnt = jnt
                        pre_ctrl = ctrl
                    else:
                        pre_jnt = jnt
                        pre_ctrl = ctrl
                        mc.parent(jnt,root_joint)
                        mc.parent(offset,feather_grp)
                    #if mdspot: #AddSesitivity
                    #    mc.connectAttr(f'{Main_Ctrl}.rotateX', f'{rot_offset}.rotateX')
                    #    mc.connectAttr(f'{Main_Ctrl}.rotateY', f'{rot_offset}.rotateY')
                    #    mc.connectAttr(f'{Main_Ctrl}.rotateZ', f'{rot_offset}.rotateZ')

                    #IF i want to do a switch it needs to be a on or off (no blending), then it will pre to post 

                mc.parentConstraint(featherjntlist[0], basectrl_offset, mo=True)
                mc.parentConstraint(featherjntlist[1], mid1ctrl_offset, mo=True)
                #mc.parentConstraint(featherjntlist[2], mid2ctrl_offset, mo=True)
                mc.parentConstraint(featherjntlist[3], eectrl_offset, mo=True)





                def_jnts.append(eejnt)
                def_jnts.append(mid1jnt)
                #def_jnts.append(mid2jnt)
                def_jnts.append(basejnt)
                mc.delete(mid1_guide)

                for i, bind_jnt in enumerate(bind_joints):
                    if i < 7:
                        main   = mainoffset_list[i]
                        mid    = midoffset_list[i]
                        aim    = aimoffset_list[i]
                        mc.parentConstraint(bind_jnt, main, mo=True)
                        mc.parentConstraint(bind_jnt, mid, mo=True)
                        mc.parentConstraint(bind_jnt, aim, mo=True)
                    elif i == 8:
                        main   = mainoffset_list[i]
                        mid    = midoffset_list[i]
                        aim    = aimoffset_list[i]
                        mc.parentConstraint(f'fingerPinky_{side}_02_JNT', main, mo=True)
                        mc.parentConstraint(f'fingerPinky_{side}_03_JNT', mid, mo=True)
                        mc.parentConstraint(f'fingerPinky_{side}_04_JNT', aim, mo=True)


            

            































































        #curve_offset = 1
        #main_curve = DomingoFeathers.build_curve(mainguides, prefix)
        #main_curve2 = mc.duplicate(main_curve)
        #mc.move(0, 0,curve_offset, main_curve, r=True)
        #mc.move(0, 0,-curve_offset, main_curve2, r=True)
        #main_surf = mc.loft(main_curve, main_curve2, name=f'{prefix}_Main_loft')
        
        #for feathergrps in ['MainFeather']:
        #    if feathergrps == 'MainFeather':
        #        sub = False
        #    else:
        #        sub = True
        #    base = feathergrps.removesuffix("Feather")
        #    maincount, mainguides = DomingoFeathers.count_feather_guides(prefix = prefix, feather=feathergrps)
        #     full_aimjnt_list = []
        #     aim_joints, upgrps, upctrs = DomingoFeathers.place_joints_on_guide_curve([f'{prefix}_CV_01', f'{prefix}_CV_02', f'{prefix}_CV_03', f'{prefix}_CV_04'], maincount, prefix, base, up_controls=True) 
        #     full_aimjnt_list.extend(aim_joints)
        #     DomingoFeathers.build_ik_spline_with_controls(aim_joints=aim_joints, prefix=prefix, sub=sub, FeatherType=base)
            
            
        #     rot_offset_list = []
        #     base_offsets = []
        #     def_jnts = []
        #     for guide in mainguides:
        #         num = guide.split("_")[-1]
        #         ee_guide = f'{prefix}_{feathergrps}_EE_{num}'
        #         basepos = mc.xform(guide, q=True, ws=True, t=True)
        #         eepos = mc.xform(ee_guide, q=True, ws=True, t=True)
        #         midpos = [basepos[i] + (eepos[i] - basepos[i]) * .5 for i in range(3)]
        #         rot = mc.xform(guide, q=True, ws=True, ro=True)
        #         mid_guide = mc.spaceLocator(p=midpos, name=f"interp_locator{guide}")[0]
        #         mc.xform(mid_guide, ws=True, ro=rot, t=midpos)
        #         basejnt, basectrl, basectrl_offset =UEface.Simple_joint_and_Control(
        #             guide,
        #             orient=True,
        #             overwrite=True,
        #             overwrite_name=f'{prefix}{feathergrps}_{num}_base',
        #             scale=True,
        #             check_side=True,
        #             CTRL_Size=2,
        #             JNT_Size=0.5
        #         )
        #         midjnt, midctrl, midctrl_offset =UEface.Simple_joint_and_Control(
        #             mid_guide,
        #             orient=True,
        #             overwrite=True,
        #             overwrite_name=f'{prefix}{feathergrps}_{num}_mid',
        #             scale=True,
        #             check_side=True,
        #             CTRL_Size=2,
        #             JNT_Size=0.5
        #         )
        #         eejnt, eectrl, eectrl_offset =UEface.Simple_joint_and_Control(
        #             ee_guide,
        #             orient=True,
        #             overwrite=True,
        #             overwrite_name=f'{prefix}{feathergrps}_{num}_ee',
        #             scale=True,
        #             check_side=True,
        #             CTRL_Size=2,
        #             JNT_Size=0.5
        #         )
        #         pre_jnt = None
        #         for part in [guide, mid_guide, ee_guide]:
        #             if part == guide: 
        #                 trans = basepos
        #                 nameing = 'base'
        #                 offset = basectrl_offset
        #                 ctrl = basectrl
        #                 jnt = basejnt
        #             elif part == mid_guide:
        #                 trans = midpos
        #                 nameing = 'mid'
        #                 offset = midctrl_offset
        #                 ctrl = midctrl
        #                 jnt = midjnt
        #             else:
        #                 trans = eepos 
        #                 nameing = 'ee'
        #                 offset = eectrl_offset
        #                 ctrl = eectrl
        #                 jnt = eejnt
        #             rot_offset = mc.group(empty=True, name=f'{prefix}_{feathergrps}_{num}_{nameing}_rotOffset')
        #             mc.xform(rot_offset, ws=True, t=trans, ro=rot)
        #             mc.parent(rot_offset, offset)
        #             mc.parent(ctrl, rot_offset)
        #             rot_offset_list.append(rot_offset)
        #             if pre_jnt != None:
        #                 mc.parent(jnt, pre_jnt)
        #                 mc.parent(offset, pre_ctrl)
        #                 pre_jnt = jnt
        #                 pre_ctrl = ctrl
        #             else:
        #                 pre_jnt = jnt
        #                 pre_ctrl = ctrl
        #                 mc.parent(jnt,root_joint)
        #                 mc.parent(offset,feather_grp)
        #         #def_jnts.append(eejnt) #THIS IS THE THING YOU ARE LOOKING FOR WHEN YOU COME BACK TO FIX IT
        #         #def_jnts.append(midjnt)
        #         def_jnts.append(basejnt)
        #         mc.delete(mid_guide)
        #         #mc.skinCluster(eejnt, midjnt, basejnt, f'{prefix}_MainFeather_{num}_GEO', tsb=True)
        #         #if feathergrps == 'Sub02Feather':

        #         mc.select(clear=True)
        #         mc.select(main_surf[0])
        #         mc.select(basectrl_offset, add=True)
        #         print(main_surf)
        #         mc.UVPin()
        #         for attr in ["rotateX", "rotateY", "rotateZ"]:
        #             # Find any nodes driving this attribute
        #             connections = mc.listConnections(f"{basectrl_offset}.{attr}", s=True, d=False, plugs=True)
        #             if connections:
        #                 for conn in connections:
        #                     mc.disconnectAttr(conn, f"{basectrl_offset}.{attr}")
        #         mc.aimConstraint(
        #             f'{prefix}_MainFeatherAim_{num}_jnt',
        #             basectrl_offset,
        #             aimVector=(0, 1, 0),
        #             upVector=(1, 0, 0),
        #             mo=False,
        #             weight=1.0,
        #             #worldUpVector = (1,0,0),
        #             worldUpType="objectrotation", worldUpObject =f'{prefix}_MainAimUp{num}_{ctrlname}'
        #             #worldUpType = 'None'
        #         )
        #         if guide == mainguides[-1]:
        #             try:
        #                 side = prefix.split("_")[-1]
        #                 print(f'{side}{feathergrps}')
        #                 mc.skinCluster(*def_jnts, f'feathers{side}', toSelectedBones=True)
        #             except Exception as e:
        #                 print(e)
        #     #for subnum in ['01', '02']:



        # bind_joints = [f'arm_{side}_01_JNT', f'arm_{side}_02_JNT', f'arm_{side}_03_JNT', f'arm_{side}_04_JNT', f'arm_{side}_05_JNT', f'arm_{side}_06_JNT', f'arm_{side}_07_JNT', f'arm_{side}_08_JNT']
        # mc.skinCluster(bind_joints, main_surf)


        # for cont in [ f'{prefix}_Shaper', f'{prefix}_Span']:
        #     rot = mc.xform(cont, q=True, ws=True, ro=True)
        #     trans = mc.xform(cont, q=True, ws=True, t=True)
        #     UEface.build_basic_control(name=f'{cont}', shape='ZTarrow', size=10.0, position=trans, rotation=rot)
        # mc.parent(f'{prefix}_Shaper_{grpname}', f'{prefix}_Span_{ctrlname}')
        # mc.parentConstraint(f'hand_{side}_01_switch_JNT', f'{prefix}_Span_{grpname}', mo=True)
        # mc.addAttr(f'{prefix}_Shaper_{ctrlname}', longName="Full_Bend", attributeType="bool", defaultValue=True, keyable=True)
        # mc.addAttr(f'{prefix}_Shaper_{ctrlname}', longName="Full_Twist", attributeType="bool", defaultValue=True, keyable=True)
        # mult_node2 = mc.createNode("multiplyDivide", name=f"{prefix}_Shape_multNode")
        # mc.connectAttr(f'{prefix}_Shaper_{ctrlname}.Full_Bend', f"{mult_node2}.input2X")
        # mc.connectAttr(f'{prefix}_Shaper_{ctrlname}.Full_Twist', f"{mult_node2}.input2Y")
        # mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateX", f"{mult_node2}.input1X")
        # mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateY", f"{mult_node2}.input1Y")
        # for rotoff in rot_offset_list:
        #     parts = rotoff.split('_')
        
        #     # The identifier is the second-to-last element
        #     identifier = parts[-2]
        
        #     if identifier == "base":
        #         mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateX", f"{rotoff}.rotateX")
        #         mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateY", f"{rotoff}.rotateY")
        #     else:
        #         mc.connectAttr(f"{mult_node2}.outputX", f"{rotoff}.rotateX")
        #         mc.connectAttr(f"{mult_node2}.outputY", f"{rotoff}.rotateY")

        # max_val = 20

        # # Objects
        # control = f"{prefix}_Span_{ctrlname}"
        # # 1. Add custom attributes
        # if not mc.objExists(f"{control}.span_mult"):
        #     mc.addAttr(control, longName="span_mult", attributeType="double", defaultValue=1.0, keyable=True)
        # if not mc.objExists(f"{control}.sensitivity"):
        #     mc.addAttr(control, longName="sensitivity", attributeType="double", defaultValue=1.0, keyable=True)

        # # 2. Create MultiplyDivide node
        # mult_node = mc.createNode("multiplyDivide", name=f"{control}_span_multNode")
        # blend = mc.createNode("blendColors", name=f"{control}_span_blend")
        # mc.connectAttr(f"{prefix}_Span_{ctrlname}.translateX", f"{blend}.color1R")

        # # 3. Create remapValue node
        # remap_node = mc.createNode("remapValue", name=f"{control}_span_remapNode")
        # mc.setAttr(f"{remap_node}.inputMin", -max_val)
        # mc.setAttr(f"{remap_node}.inputMax", max_val)
        # mc.setAttr(f"{remap_node}.outputMin", 0)
        # mc.setAttr(f"{remap_node}.outputMax", 2.0)

        # # 4. Connect translateX to multiplyDivide input1X
        # mc.connectAttr(f"{blend}.outputR", f"{mult_node}.input1X")

        # # 5. Connect sensitivity and span_mult to input2 channels
        # mc.connectAttr(f"{control}.sensitivity", f"{mult_node}.input2X")
        # mc.connectAttr(f"{control}.span_mult", f"{mult_node}.input2Y")

        # # 6. Connect multiplyDivide output to remapValue input
        # mc.connectAttr(f"{mult_node}.outputX", f"{remap_node}.inputValue")

        # # 7. Connect remapValue output to multiplyDivide input1Y
        # mc.connectAttr(f"{remap_node}.outValue", f"{mult_node}.input1Y")

        # # 8. Connect multiplyDivide outputY to target joint scaleX
        # for aimjnt in full_aimjnt_list:
        #     mc.connectAttr(f"{mult_node}.outputY", f"{aimjnt}.scaleX")








        # #clean Up
        
        # #Arm Hookin
        # mc.parentConstraint(f'arm_{side}_01_switch_JNT', f'Wing_{side}_Main_Feather_aim_01_{side}_CTRL_CNST_GRP', mo=True)
        # mc.parentConstraint(f'arm_{side}_02_switch_JNT', f'Wing_{side}_Main_Feather_aim_02_{side}_CTRL_CNST_GRP', mo=True)
        # mc.parentConstraint(f'hand_{side}_01_switch_JNT', f'Wing_{side}_Main_Feather_aim_03_{side}_CTRL_CNST_GRP', mo=True)
        # mc.parentConstraint(f'hand_{side}_01_switch_JNT', f'Wing_{side}_Main_Feather_aim_04_{side}_CTRL_CNST_GRP', mo=True) #FIX THIS ONCE WE GET THE IK FINGER CONTROLS SET UP

        # #Put Back in the Heirarchy 
        # mc.parent(f'Wing_{side}_root_jnt',f'arm_{side}_01_JNT')
        # mc.parent(f'Wing_{side}_feather_{side}_CTRL_CNST_GRP', module_grp_name)
        # mc.parent(f'Wing_{side}_handle_{side}_CTRL_CNST_GRP', module_grp_name)
        # mc.parent(f'Wing_{side}_upAim_{side}_CTRL_CNST_GRP', module_grp_name)
        # mc.delete(f'Wing_{side}_curve', f'Wing_{side}_curve1')
        # mc.parent(f'Wing_{side}_Main_loft', module_grp_name)
        # mc.parent(f'Wing_{side}_Span_{side}_CTRL_CNST_GRP', module_grp_name)
        # for num in ['01', '02', '03', '04']:
        #     mc.parent(f'Wing_{side}_Main_Feather_aim_{num}_{side}_CTRL_CNST_GRP', module_grp_name)
        # mc.hide(f'Wing_{side}_handle_{side}_CTRL_CNST_GRP', f'Wing_{side}_MainFeatherAim_01_jnt', f'Wing_{side}_Main_loft') #f'Wing_{side}_Sub01FeatherAim_01_jnt', f'Wing_{side}_Sub02FeatherAim_01_jnt',
        # mc.parent(module_grp_name, 'RIG')
