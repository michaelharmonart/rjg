import math
import re
from importlib import reload

import maya.cmds as mc
import rjg.build.chain as rChain
import rjg.build.guide as rGuide
import rjg.libs.attribute as rAttr
import rjg.libs.control.ctrl as rCtrl
import rjg.libs.transform as rXform
from rjg.build.UEface import UEface
from rjg.libs.spline import generate_knots, get_cvs, get_knots
from rjg.libs.profile import auto_profiler_tag

reload(rAttr)
reload(rChain)
reload(rCtrl)
reload(rGuide)
reload(rXform)


class UEwing(UEface):
    def __init__(self, grp_name=None, ctrl_scale=1,):
        super().__init__(part='Wing', grp_name=grp_name, ctrl_scale=ctrl_scale)
        self.grp_name = grp_name
        #group='Wing_L_guides'
        
        
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

    
    def build_ik_spline_with_controls(self, aim_joints=None, prefix=None, sub=False, FeatherType=None):
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        # Step 1: Create IK spline
        ik_handle, effector, curve = mc.ikHandle(
            sj=aim_joints[0],
            ee=aim_joints[-1],
            sol='ikSplineSolver',
            ccv=True,
            pcv=False
        )
        curve = mc.rename(curve, f'{prefix}_{FeatherType}_curve')
        ik_handle = mc.rename(ik_handle, f'{prefix}_{FeatherType}_ik_handle')
        mc.parent(ik_handle, f'{prefix}_handle_{grpname}')
        mc.parent(curve, f'{prefix}_handle_{grpname}')
        # Step 2: For each CV on the curve, create cluster + control
        cvs = mc.ls(f"{curve}.cv[*]", fl=True)
        
        for i, cv in enumerate(cvs, start=1):
            # Make cluster for the CV
            cluster, cluster_handle = mc.cluster(cv, n=f"{prefix}_{FeatherType}Feather_aimCluster_{i:02}")
            mc.parent(cluster_handle, f'{prefix}_handle_{grpname}')
            # Get cluster position
            pos = mc.pointPosition(cv, w=True)
            if sub == False:
                # Make control
                ctrl_name = f"{prefix}_Main_Feather_aim_{i:02}"
                ctrl, offset = UEface.build_basic_control(
                    name=ctrl_name,
                    shape='ZTsphere',
                    size=10.0,
                    color_rgb=(1, 1, 0),
                    position=pos,
                    rotation=(0, 0, 0)
                )
            
                # Parent cluster to control
                mc.parentConstraint(ctrl, cluster_handle, mo=True)

                close_offset = mc.group(empty=True, name=f'{prefix}Aim_{i:02}_ArmClose_offset')
                mc.xform(close_offset, ws=True, t=pos,)
                mc.parent(close_offset, offset)
                mc.parent(ctrl, close_offset)

            else:
                mc.parentConstraint(f"{prefix}_Main_Feather_aim_{i:02}_{ctrlname}", cluster_handle, mo=True)
        
        return ik_handle, curve


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

    def count_feather_guides(self, prefix, feather):
        ctrlname, grpname = UEwing.get_namestruc(prefix)
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

    def place_joints_on_guide_curve(self, guides=[], feather_count=1, prefix=None, feathertype=None, up_controls=False, even=False):
        if guides == []:
            for i in range(1, feather_count + 1):
                num = f"{i:02d}"
                guides.append(f"{prefix}_MainFeather_{num}_aim")
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        # Get world positions from guides
        positions = [mc.xform(g, q=True, ws=True, t=True) for g in guides]
        
        # Build temp curve from guide positions
        curve = mc.curve(p=positions, degree=3)  # cubic curve
        mc.rebuildCurve(curve, ch=False, rpo=True, spans=len(positions)-1, degree=3)
        
        joints = []
        upgrps = []
        upctrs = []
        if even:
            for i in range(feather_count):
                u = float(i) / (feather_count - 1) if feather_count > 1 else 0.0
                pos = mc.pointOnCurve(curve, pr=u, p=True) #Wing_L_MainFeather01_guide Wing_L_MainFeather_01_guide
                pos2 = mc.xform(f'{prefix}_{feathertype}Feather_{i+1:02}_guide', q=True, ws=True, t=True)
                tangent = UEwing.look_at_rotation(pos,pos2)
                
                jnt = mc.joint(p=pos, name=f"{prefix}_{feathertype}FeatherAim_{i+1:02}_jnt")
                joints.append(jnt)
                if up_controls == True:
                    rot = mc.xform(jnt, q=True, ws=False, rotation=True)
                    size = 1 if feathertype != "Main" else 50
                    upctr, upgrp = UEface.build_basic_control(name=f'{prefix}_{feathertype}AimUp{i+1:02}', shape='ZTpoint', size=size, color_rgb=(1, 1, 0), position=pos, rotation=tangent)
                    mc.parentConstraint(jnt, upgrp, mo=True)
                    upgrps.append(upgrp)
                    upctrs.append(upctr)
                    mc.parent(upgrp, f'{prefix}_upAim_{grpname}')
                    mc.select(jnt)
        else:
            for i in range(1, feather_count + 1):
                num = f"{i:02d}"
                u = float(i) / (feather_count - 1) if feather_count > 1 else 0.0
                #pos = mc.pointOnCurve(curve, pr=u, p=True) #Wing_L_MainFeather01_guide Wing_L_MainFeather_01_guide
                pos = mc.xform(f"{prefix}_MainFeather_{num}_aim", q=True, ws=True, t=True) 
                pos2 = mc.xform(f"{prefix}_MainFeather_{num}_guide", q=True, ws=True, t=True)
                tangent = UEwing.look_at_rotation(pos,pos2)
                
                jnt = mc.joint(p=pos, name=f"{prefix}_{feathertype}FeatherAim_{num}_jnt")
                joints.append(jnt)
                if up_controls == True:
                    rot = mc.xform(jnt, q=True, ws=False, rotation=True)
                    size = 1 if feathertype != "Main" else 50
                    upctr, upgrp = UEface.build_basic_control(name=f'{prefix}_{feathertype}AimUp{num}', shape='ZTpoint', size=size, color_rgb=(1, 1, 0), position=pos, rotation=tangent)
                    mc.parentConstraint(jnt, upgrp, mo=True)
                    upgrps.append(upgrp)
                    upctrs.append(upctr)
                    mc.parent(upgrp, f'{prefix}_upAim_{grpname}')
                    mc.select(jnt)



        # Delete temp curve
        mc.delete(curve)
        mc.parent(f"{prefix}_{feathertype}FeatherAim_01_jnt", f'{prefix}_feather_{grpname}')
        return joints, upgrps, upctrs



    def build_stretchy_spline(self, source='GuideList', offsetname=None, inputlist = [], inputcurve = None, buildControls=True, guidecurve=None, feathernum=None, Stretch=True, StretchControl = None, prefix=None, autoconstrain=True, finalclustertwist=False, featherrot=[0,0,0] ):
        #'GuideList', 'Curve', 'jointlist'
        ctrlname, grpname = UEwing.get_namestruc(prefix)
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
                    rot = UEwing.look_at_rotation(pos, prepos)
                else:
                    posfuture = mc.xform(inputlist[1], q=True, ws=True, t=True)
                    rot = UEwing.look_at_rotation(posfuture, pos)

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


        # Step 2: For each CV on the curve, create cluster + control
        cvs = mc.ls(f"{curve}.cv[*]", fl=True)
        ctrl_list = []
        offset_list = []
        cluster_list = []
        print(jntlist)
        
        for i, cv in enumerate(cvs, start=1):
            # Make cluster for the CV
            cluster, cluster_handle = mc.cluster(cv, n=f"{name}_{i:02}_cluster")
            mc.parent(cluster_handle, f'{prefix}_handle_{grpname}')
            cluster_point = mc.group(empty=True, name=f"{name}_{i:02}_clusterPoint", parent=f'{prefix}_handle_{grpname}') 
            cluster_list.append(cluster_point)
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
                    size=30.0,
                    color_rgb=(1, 1, 0),
                    position=pos,
                    rotation=(0, 0, 0)
                )
                ctrl_list.append(ctrl)
                offset_list.append(offset)
                mc.parent(offset, f'{prefix}_feather_{grpname}')

                # Parent cluster to control
                mc.parentConstraint(ctrl, cluster_handle, mo=True)
                mc.parentConstraint(ctrl, cluster_point, mo=False)

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
                        
                    mc.connectAttr(autoattr, f'{rolladl}.input1')
                    mc.connectAttr(f'{StretchControl}.roll', f'{rolladl}.input2')
                    mc.connectAttr(f'{rolladl}.output', f'{ik_handle}.roll')


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
                mc.connectAttr(f"{postci}.arcLength", f"{frac}.input1X", force=True)
                mc.connectAttr(f"{preci}.arcLength", f"{frac}.input2X", force=True) 

                #md = mc.createNode("multiplyDivide", name=f"{curve}_MD")

                #mc.connectAttr(f"{frac}.outputX", f"{md}.input1X", force=True)
                #mc.connectAttr(Stretch_attr, f"{md}.input2X", force=True)

                for newjnt in jntlist:
                    mc.connectAttr(f"{frac}.outputX", f"{newjnt}.scaleZ")

        mc.parent(jntlist[0], f'{prefix}_net_{grpname}')
        mc.setAttr(f'{jntlist[0]}.template', 1)
        
        return ik_handle, curve, ctrl_list, offset_list, jntlist, cluster_list


    @auto_profiler_tag
    def build_wing(self, buildType='splines'):
        # 'splines' or 'stretchey_splines'
        #group='Wing_L_guides'
        prefix = UEface.get_prefix_from_group(self.grp_name)
        grp = self.grp_name
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        parts = prefix.split("_")   # ["wing", "L"]
        side = parts[-1]

        feather_grp = mc.group(em=True, name=f'{prefix}_feather_{grpname}')
        handle_grp = mc.group(em=True, name=f'{prefix}_handle_{grpname}')
        #upAim_grp = mc.group(em=True, name=f'{prefix}_upAim_{grpname}')
        mc.select(clear=True)
        fk_group = mc.group(em=True, name=f'{prefix}_FK_{grpname}')
        ik_group = mc.group(em=True, name=f'{prefix}_IK_{grpname}')
        netgrp = mc.group(em=True, name=f'{prefix}_net_{grpname}')

        maincount, mainguides = self.count_feather_guides(prefix = prefix, feather='MainFeather')

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

            mainik_handle, maincurve, mainctrl_list, mainoffset_list, mainjntlist, main_cluster_list = (
                self.build_stretchy_spline(
                    source="GuideList",
                    inputlist=root_list,
                    inputcurve=None,
                    buildControls=True,
                    guidecurve="main",
                    Stretch=True,
                    prefix=prefix,
                )
            )
            midik_handle, midcurve, midctrl_list, midoffset_list, midjntlist, mid_cluster_list = (
                self.build_stretchy_spline(
                    source="GuideList",
                    inputlist=mid_list,
                    inputcurve=None,
                    buildControls=True,
                    guidecurve="mid",
                    Stretch=True,
                    prefix=prefix,
                )
            )
            aimik_handle, aimcurve, aimctrl_list, aimoffset_list, aimjntlist, aim_cluster_list = (
                self.build_stretchy_spline(
                    source="GuideList",
                    inputlist=aim_list,
                    inputcurve=None,
                    buildControls=True,
                    guidecurve="aim",
                    Stretch=True,
                    prefix=prefix,
                )
            )
            
            def get_shapes(transform: str) -> list[str]:
                # list the shapes of node
                shape_list: list[str] = mc.listRelatives(
                    transform, shapes=True, noIntermediate=True, children=True
                )
            
                if shape_list:
                    return shape_list
                else:
                    raise RuntimeError(f"{transform} has no child shape nodes")
            main_shape = get_shapes(maincurve)[0]
            mid_shape = get_shapes(midcurve)[0]
            aim_shape = get_shapes(aimcurve)[0]
            positions = get_cvs(main_shape) + get_cvs(mid_shape) + get_cvs(aim_shape)
            knots_v = get_knots(main_shape)[1:-1]
            knots_u = generate_knots(3, degree=2)[1:-1]
            surface = mc.surface(
                name=f"{prefix}_Surface",
                point=[(position.x, position.y, position.z) for position in positions],
                knotU=knots_u,
                knotV=knots_v,
                degreeU=2,
            )
            surface_transform = mc.listRelatives(surface, parent=True)[0]
            for index, cluster in enumerate(main_cluster_list + mid_cluster_list + aim_cluster_list):
                mc.connectAttr(f"{cluster}.translate", f"{surface}.controlPoints[{index}]")
            mc.parent(surface_transform, handle_grp)

            #bind
            bind_joints = []
            pre_jnt = None
            for obj in [f'{prefix}_01_guide', f'{prefix}_02_guide', f'{prefix}_03_guide', f'{prefix}_04_guide']:
                # Get the base name and generate joint name
                base_name = obj.split('|')[-1].replace('_guide', '')
                joint_name = f"{base_name}_bind_jnt"

                # Clear selection before creating the joint to avoid parenting
                mc.select(clear=True)
                joint = mc.joint(name=joint_name)
                bind_joints.append(joint)

                # Match translation and rotation in world space
                pos = mc.xform(obj, q=True, ws=True, t=True)
                rot = mc.xform(obj, q=True, ws=True, ro=True)
                mc.xform(joint, ws=True, t=pos)
                mc.xform(joint, ws=True, ro=rot)
                if pre_jnt != None:
                    mc.parent(joint_name, pre_jnt)
                pre_jnt = joint_name
            #mc.skinCluster(bind_joints, main_surf)
            pre_jnt = None

            parjnts = ['01', '02', '03', '04']

            #Build Feather :)
            rot_offset_list = []
            base_offsets = []
            def_jnts = []
            main_drivers = []
            mid_drivers = []
            aim_drivers = []
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
                root_joint = f"{base_name}_bind_jnt"

                basepos = mc.xform(guide, q=True, ws=True, t=True)
                eepos = mc.xform(ee_guide, q=True, ws=True, t=True)
                mid1pos = [basepos[i] + (eepos[i] - basepos[i]) * (1/3) for i in range(3)]
                mid2pos = [basepos[i] + (eepos[i] - basepos[i]) * (2/3) for i in range(3)]
                rot = mc.xform(guide, q=True, ws=True, ro=True)
                mid1_guide = mc.spaceLocator(p=mid1pos, name=f"interp_locator{guide}")[0]
                mid2_guide = mc.spaceLocator(p=mid2pos, name=f"interp_locator{guide}")[0]
                mc.xform(mid1_guide, ws=True, ro=rot, t=mid1pos)
                mc.xform(mid2_guide, ws=True, ro=rot, t=mid2pos)

                #main
                main_rot = rot #[a + b for a, b in zip(rot, add)]
                main_ctrl, main_group = UEface.build_basic_control( name=f'{prefix}_Feather_{num}', shape='ZTpoint', size=90.0, position=basepos, rotation=main_rot)
                mc.parent(main_group, feather_grp)
                for ax in ["X", "Y", "Z"]:
                    mc.setAttr(f'{main_ctrl}.translate{ax}', lock=True, channelBox=False)
                    mc.setAttr(f'{main_ctrl}.scale{ax}', lock=True, channelBox=False)

                feather_list = [guide, mid1_guide, mid2_guide, ee_guide, aim_guide]
                featherik_handle, feathercurve, featherctrl_list, featheroffset_list, featherjntlist, feathercluster_list = self.build_stretchy_spline(source='GuideList', offsetname=None, inputlist = feather_list, inputcurve = None, buildControls=False, guidecurve=None, feathernum=num, Stretch=True, StretchControl = main_ctrl, prefix=prefix, autoconstrain=True, finalclustertwist=True, featherrot = rot)

                basejnt, basectrl, basectrl_offset =UEface.Simple_joint_and_Control(
                    guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}MainFeather_{num}_base',
                    scale=True,
                    check_side=True,
                    CTRL_Size=50,
                    JNT_Size=0.5
                )
                mid1jnt, mid1ctrl, mid1ctrl_offset =UEface.Simple_joint_and_Control(
                    mid1_guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}MainFeather_{num}_mid1',
                    scale=True,
                    check_side=True,
                    CTRL_Size=50,
                    JNT_Size=0.5
                )
                mid2jnt, mid2ctrl, mid2ctrl_offset =UEface.Simple_joint_and_Control(
                    mid2_guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}MainFeather_{num}_mid2',
                    scale=True,
                    check_side=True,
                    CTRL_Size=50,
                    JNT_Size=0.5
                )
                eejnt, eectrl, eectrl_offset =UEface.Simple_joint_and_Control(
                    ee_guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}MainFeather_{num}_ee',
                    scale=True,
                    check_side=True,
                    CTRL_Size=50,
                    JNT_Size=0.5
                )
                split_joint = basejnt
                split_joints: list[str] = [basejnt,mid1jnt,mid2jnt,eejnt]
                mc.addAttr(split_joint, longName="split_joints", dataType="string")
                mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")

                '''split_joints: list[str] = bind_joints
                    mc.addAttr(split_joint, longName="split_joints", dataType="string")
                    mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")'''

                mc.pointConstraint(basectrl, main_group, mo=True)
                #mc.pointConstraint(basectrl, aim_loc_off, mo=True)
                pre_jnt = None
                pre_ctrl = None
                for part in [guide, mid1_guide, mid2_guide, ee_guide]:
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
                    elif part == mid2_guide:
                        trans = mid2pos
                        nameing = 'mid2'
                        offset = mid2ctrl_offset
                        ctrl = mid2ctrl
                        jnt = mid2jnt
                        mdspot = 'Y'
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

                    #IF i want to do a switch it needs to be a on or off (no blending), then it will pre to post 

                mc.parentConstraint(featherjntlist[0], basectrl_offset, mo=True)
                mc.parentConstraint(featherjntlist[1], mid1ctrl_offset, mo=True)
                mc.parentConstraint(featherjntlist[2], mid2ctrl_offset, mo=True)
                mc.parentConstraint(featherjntlist[3], eectrl_offset, mo=True)





                def_jnts.append(eejnt)
                def_jnts.append(mid1jnt)
                def_jnts.append(mid2jnt)
                def_jnts.append(basejnt)
                mc.delete(mid1_guide, mid2_guide)

                for i, bind_jnt in enumerate(bind_joints):
                    main   = mainoffset_list[i]
                    mid    = midoffset_list[i]
                    aim    = aimoffset_list[i]
                    mc.parentConstraint(bind_jnt, main, mo=True)
                    mc.parentConstraint(bind_jnt, mid, mo=True)
                    mc.parentConstraint(bind_jnt, aim, mo=True)

        pre_jnt = None
        pre_ctrl = None
        armjnts = []
        armoffsets = []
        armctrls = []
        armcloses= []

        #arm Logic
        FKIKSwitch_pos = mc.xform(f'{prefix}_Close', q=True, ws=True, t=True)
        FKIKSwitch_CTL, FKIKSwitch_GRP = UEface.build_basic_control(name=f'{prefix}_FKIKSwitch', shape='ZTgear', size=5.0, color_rgb=(1, 1, 0), position=FKIKSwitch_pos, rotation=(0, 0, 0))
        mc.addAttr(FKIKSwitch_CTL, longName="FK_IK", attributeType="bool", keyable=True)
        rev_node = mc.createNode("reverse", name=f"{prefix}IKReverse")
        mc.connectAttr(f'{FKIKSwitch_CTL}.FK_IK', f'{rev_node}.inputX')


        #fk
        for guide in [f'{prefix}_01_guide', f'{prefix}_02_guide', f'{prefix}_03_guide', f'{prefix}_04_guide']:
            parts = guide.split("_")  # ["wing", "l", "01", "guide"]
            number = parts[-2]        # second to last = "01", "02", etc.
            print(number)
            jnt, ctrl, ctrl_offset = UEface.Simple_joint_and_Control(
                guide,
                orient=True,
                overwrite=True,
                overwrite_name=f'{prefix}_{number}_FK',
                scale=True,
                check_side=False,
                CTRL_Color=(0, 0, 1),
                CTRL_Size=3,
                JNT_Size=0.5,
                bind=False
            )
            #mc.addAttr()
            rot = mc.xform(guide, q=True, ws=True, ro=True)
            trans = mc.xform(guide, q=True, ws=True, t=True)
            close_offset = mc.group(empty=True, name=f'{prefix}_{number}_FK_ArmClose_offset')
            mc.xform(close_offset, ws=True, t=trans, ro=rot)
            mc.parent(close_offset, ctrl_offset)
            mc.parent(ctrl, close_offset)
            if pre_jnt != None:
                mc.parent(jnt, pre_jnt)
                mc.parent(ctrl_offset, pre_ctrl)
                pre_jnt = jnt
                pre_ctrl = ctrl
            else:
                pre_jnt = jnt
                pre_ctrl = ctrl
            armjnts.append(jnt)
            armoffsets.append(ctrl_offset)
            armctrls.append(ctrl)
            armcloses.append(close_offset)
        ########################################################## Come back to this
        for num in ['01', '02', '03', '04']:
            mc.parentConstraint(f'{prefix}_{num}_FK_JNT', f'{prefix}_{num}_bind_jnt', mo=True )

        #ik
        IK_joints = []
        pre_jnt = None
        for obj in [f'{prefix}_01_guide', f'{prefix}_02_guide', f'{prefix}_03_guide', f'{prefix}_04_guide']:
            # Get the base name and generate joint name
            base_name = obj.split('|')[-1].replace('_guide', '')
            joint_name = f"{base_name}_IK_jnt"

            # Clear selection before creating the joint to avoid parenting
            mc.select(clear=True)
            joint = mc.joint(name=joint_name)
            IK_joints.append(joint)

            # Match translation and rotation in world space
            pos = mc.xform(obj, q=True, ws=True, t=True)
            rot = mc.xform(obj, q=True, ws=True, ro=True)
            mc.xform(joint, ws=True, t=pos)
            mc.xform(joint, ws=True, ro=rot)
            if pre_jnt != None:
                mc.parent(joint_name, pre_jnt)
            pre_jnt = joint_name

        pv_pos = mc.xform(f'{prefix}_IK_Aim', q=True, ws=True, t=True)
        ikaimCTL, ikaimGRP = UEface.build_basic_control(name=f'{prefix}_IK_Aim', shape='locator_3D', size=20.0, color_rgb=(1, 1, 0), position=pv_pos, rotation=(0, 0, 0))
        
        ikhandel  = mc.ikHandle(
            name=f"{prefix}_ikHandle",
            sj=f"{prefix}_01_IK_jnt", 
            ee=f"{prefix}_03_IK_jnt", 
            sol="ikRPsolver"
        )[0]

        mc.poleVectorConstraint(ikaimCTL, ikhandel)
        IK_Root_pos = mc.xform(f'{prefix}_01_guide', q=True, ws=True, t=True)
        IK_Root_CTL, IK_Root_GRP = UEface.build_basic_control(name=f'{prefix}_IK_Root', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=IK_Root_pos, rotation=(0, 0, 0))
        mc.parentConstraint(IK_Root_CTL, f"{prefix}_01_IK_jnt", mo=True)

        IK_EE_pos = mc.xform(f'{prefix}_03_guide', q=True, ws=True, t=True)
        IK_EE_rot = mc.xform(f'{prefix}_03_guide', q=True, ws=True, rotation=True)
        IK_EE_CTL, IK_EE_GRP = UEface.build_basic_control(name=f'{prefix}_IK_EE', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=IK_EE_pos, rotation=IK_EE_rot)
        mc.parentConstraint(IK_EE_CTL, ikhandel, mo=True)

        IK_04_pos = mc.xform(f'{prefix}_04_guide', q=True, ws=True, t=True)
        IK_04_CTL, IK_04_GRP = UEface.build_basic_control(name=f'{prefix}_IK_04', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=IK_04_pos, rotation=(0, 0, 0))
        mc.parentConstraint(IK_04_CTL, f"{prefix}_04_IK_jnt", mo=True)
        mc.parent(IK_04_GRP, IK_EE_CTL)
        for num in ['01', '02', '03', '04']:
            mc.parentConstraint(f'{prefix}_{num}_IK_jnt', f'{prefix}_{num}_bind_jnt', mo=True)
            mc.connectAttr(f'{FKIKSwitch_CTL}.FK_IK', f'{prefix}_{num}_bind_jnt_parentConstraint1.{prefix}_{num}_FK_JNTW0')
            mc.connectAttr(f'{rev_node}.outputX', f'{prefix}_{num}_bind_jnt_parentConstraint1.{prefix}_{num}_IK_jntW1')
        mc.pointConstraint( f'{prefix}_01_bind_jnt', FKIKSwitch_GRP, mo=True)
        mc.orientConstraint(f'{prefix}_IK_EE_{ctrlname}', f'{prefix}_03_IK_jnt', mo=True)

        max_val = 20
        
        #Clean Up Wing
        mc.group(f'{prefix}_01_FK_JNT', f'{prefix}_01_IK_jnt', f'{prefix}_ikHandle', name=f'{prefix}_extraOffset_{grpname}') #f'{prefix}_Main_loft'
        mc.parent(f'{prefix}_IK_Aim_{grpname}', f'{prefix}_IK_Root_{ctrlname}' )
        mc.parent(f'{prefix}_IK_EE_{grpname}', f'{prefix}_IK_Root_{ctrlname}' )
        mc.parent(f'{prefix}_IK_Root_{grpname}', ik_group)
        mc.connectAttr(f'{FKIKSwitch_CTL}.FK_IK', f'{prefix}_FK_{grpname}.visibility')
        mc.connectAttr(f'{rev_node}.outputX', f'{prefix}_IK_{grpname}.visibility')
        jnt, ctrl, ctrl_offset =UEface.Simple_joint_and_Control(
                f'{prefix}_Scap',
                orient=True,
                overwrite=False,
                scale=True,
                check_side=True,
                CTRL_Size=10,
                JNT_Size=0.5)
        




        mastergrp = mc.group(em=True, name =f'{prefix}')
        mc.parent( f'{prefix}_FK_{grpname}', f'{prefix}_IK_{grpname}', ctrl)
        mc.parent(f'{prefix}_feather_{grpname}', f'{prefix}_handle_{grpname}',f'{prefix}_FKIKSwitch_{grpname}', f'{prefix}_extraOffset_{grpname}', ctrl_offset, mastergrp)#f'{prefix}_upAim_{grpname}'f'{prefix}_Span_{grpname}'f'{prefix}_aimcurve_{grpname}'
        mc.parent(f'{prefix}_01_bind_jnt', jnt) #f'{prefix}_root_jnt'
        mc.parent(jnt, 'chest_M_JNT')
        mc.parentConstraint('chest_M_02_CTRL', ctrl_offset, mo=True)
        mc.hide(f'{prefix}_handle_{grpname}', f'{prefix}_extraOffset_{grpname}')
        mc.parent(mastergrp, 'RIG')
        mc.parent(netgrp, mastergrp)
        mc.parentConstraint(bind_joints[0], netgrp, mo=True)
