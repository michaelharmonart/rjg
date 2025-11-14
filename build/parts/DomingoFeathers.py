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
    def build_ik_spline_with_controls(aim_joints=None, prefix=None, sub=False, FeatherType=None):
        ctrlname, grpname = DomingoFeathers.get_namestruc(prefix)
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
                    size=1.0,
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
    def build_curve(guide_list, prefix, degree=3):
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
        
        # We'll just loop up to some reasonable high number to check
        for i in range(1, 200):  
            num_str = f"{i:02}" if i < 10 else str(i)  # pad with zero if < 10
            name = f"{prefix}_{feather}_Root_{num_str}"
            
            if mc.objExists(name):
                guides.append(name)
                count += 1
        
        print(f"Found {count} feather guide(s): {guides}")
        return count, guides

    @staticmethod
    def place_joints_on_guide_curve(guides, feather_count, prefix, feathertype, up_controls=False):
        ctrlname, grpname = DomingoFeathers.get_namestruc(prefix)
        # Get world positions from guides
        positions = [mc.xform(g, q=True, ws=True, t=True) for g in guides]
        
        # Build temp curve from guide positions
        curve = mc.curve(p=positions, degree=3)  # cubic curve
        mc.rebuildCurve(curve, ch=False, rpo=True, spans=len(positions)-1, degree=3)
        
        joints = []
        upgrps = []
        upctrs = []
        for i in range(feather_count):
            u = float(i) / (feather_count - 1) if feather_count > 1 else 0.0
            pos = mc.pointOnCurve(curve, pr=u, p=True) #Wing_L_MainFeather01_guide Wing_L_MainFeather_01_guide
            pos2 = mc.xform(f'{prefix}_{feathertype}Feather_Root_{i+1:02}', q=True, ws=True, t=True)
            tangent = DomingoFeathers.look_at_rotation(pos,pos2)
            
            jnt = mc.joint(p=pos, name=f"{prefix}_{feathertype}FeatherAim_{i+1:02}_jnt")
            joints.append(jnt)
            if up_controls == True:
                rot = mc.xform(jnt, q=True, ws=False, rotation=True)
                size = 1 if feathertype != "Main" else 5
                upctr, upgrp = UEface.build_basic_control(name=f'{prefix}_{feathertype}AimUp{i+1:02}', shape='ZTpoint', size=size, color_rgb=(1, 1, 0), position=pos, rotation=tangent)
                mc.parentConstraint(jnt, upgrp, mo=True)
                upgrps.append(upgrp)
                upctrs.append(upctr)
                mc.parent(upgrp, f'{prefix}_upAim_{grpname}')
                mc.select(jnt)

        # Delete temp curve
        mc.delete(curve)
        mc.parent(f"{prefix}_{feathertype}FeatherAim_01_jnt", f'{prefix}_feather_{grpname}')
        return joints, upgrps, upctrs



        #ShaperLogic
        mc.addAttr(f'{prefix}_Shaper_{ctrlname}', longName="Full_Bend", attributeType="bool", defaultValue=True, keyable=True)
        mc.addAttr(f'{prefix}_Shaper_{ctrlname}', longName="Full_Twist", attributeType="bool", defaultValue=True, keyable=True)
        mult_node2 = mc.createNode("multiplyDivide", name=f"{prefix}_Shape_multNode")
        mc.connectAttr(f'{prefix}_Shaper_{ctrlname}.Full_Bend', f"{mult_node2}.input2X")
        mc.connectAttr(f'{prefix}_Shaper_{ctrlname}.Full_Twist', f"{mult_node2}.input2Y")
        mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateX", f"{mult_node2}.input1X")
        mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateY", f"{mult_node2}.input1Y")

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
        mc.select(clear=True)
        if side == 'L':
            pos = mc.xform(f'LeftArm', q=True, ws=True, t=True)
        elif side == 'R':
            pos = mc.xform(f'RightArm', q=True, ws=True, t=True)
        else:
            pos = (0,0,0)
        root_joint = mc.joint(name=f'{prefix}_root_jnt', p=pos)

        #Build Shaper and Span controls
        #build_shaper_and_span_controls(prefix)

        maincount, mainguides = DomingoFeathers.count_feather_guides(prefix = prefix, feather='MainFeather')
        curve_offset = 1
        main_curve = DomingoFeathers.build_curve(mainguides, prefix)
        main_curve2 = mc.duplicate(main_curve)
        mc.move(0, 0,curve_offset, main_curve, r=True)
        mc.move(0, 0,-curve_offset, main_curve2, r=True)
        main_surf = mc.loft(main_curve, main_curve2, name=f'{prefix}_Main_loft')
        
        for feathergrps in ['MainFeather']:
            if feathergrps == 'MainFeather':
                sub = False
            else:
                sub = True
            base = feathergrps.removesuffix("Feather")
            maincount, mainguides = DomingoFeathers.count_feather_guides(prefix = prefix, feather=feathergrps)
            full_aimjnt_list = []
            aim_joints, upgrps, upctrs = DomingoFeathers.place_joints_on_guide_curve([f'{prefix}_CV_01', f'{prefix}_CV_02', f'{prefix}_CV_03', f'{prefix}_CV_04'], maincount, prefix, base, up_controls=True) 
            full_aimjnt_list.extend(aim_joints)
            DomingoFeathers.build_ik_spline_with_controls(aim_joints=aim_joints, prefix=prefix, sub=sub, FeatherType=base)
            
            
            rot_offset_list = []
            base_offsets = []
            def_jnts = []
            for guide in mainguides:
                num = guide.split("_")[-1]
                ee_guide = f'{prefix}_{feathergrps}_EE_{num}'
                basepos = mc.xform(guide, q=True, ws=True, t=True)
                eepos = mc.xform(ee_guide, q=True, ws=True, t=True)
                midpos = [basepos[i] + (eepos[i] - basepos[i]) * .5 for i in range(3)]
                rot = mc.xform(guide, q=True, ws=True, ro=True)
                mid_guide = mc.spaceLocator(p=midpos, name=f"interp_locator{guide}")[0]
                mc.xform(mid_guide, ws=True, ro=rot, t=midpos)
                basejnt, basectrl, basectrl_offset =UEface.Simple_joint_and_Control(
                    guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}{feathergrps}_{num}_base',
                    scale=True,
                    check_side=True,
                    CTRL_Size=2,
                    JNT_Size=0.5
                )
                midjnt, midctrl, midctrl_offset =UEface.Simple_joint_and_Control(
                    mid_guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}{feathergrps}_{num}_mid',
                    scale=True,
                    check_side=True,
                    CTRL_Size=2,
                    JNT_Size=0.5
                )
                eejnt, eectrl, eectrl_offset =UEface.Simple_joint_and_Control(
                    ee_guide,
                    orient=True,
                    overwrite=True,
                    overwrite_name=f'{prefix}{feathergrps}_{num}_ee',
                    scale=True,
                    check_side=True,
                    CTRL_Size=2,
                    JNT_Size=0.5
                )
                pre_jnt = None
                for part in [guide, mid_guide, ee_guide]:
                    if part == guide: 
                        trans = basepos
                        nameing = 'base'
                        offset = basectrl_offset
                        ctrl = basectrl
                        jnt = basejnt
                    elif part == mid_guide:
                        trans = midpos
                        nameing = 'mid'
                        offset = midctrl_offset
                        ctrl = midctrl
                        jnt = midjnt
                    else:
                        trans = eepos 
                        nameing = 'ee'
                        offset = eectrl_offset
                        ctrl = eectrl
                        jnt = eejnt
                    rot_offset = mc.group(empty=True, name=f'{prefix}_{feathergrps}_{num}_{nameing}_rotOffset')
                    mc.xform(rot_offset, ws=True, t=trans, ro=rot)
                    mc.parent(rot_offset, offset)
                    mc.parent(ctrl, rot_offset)
                    rot_offset_list.append(rot_offset)
                    if pre_jnt != None:
                        mc.parent(jnt, pre_jnt)
                        mc.parent(offset, pre_ctrl)
                        pre_jnt = jnt
                        pre_ctrl = ctrl
                    else:
                        pre_jnt = jnt
                        pre_ctrl = ctrl
                        mc.parent(jnt,root_joint)
                        mc.parent(offset,feather_grp)
                def_jnts.append(eejnt)
                def_jnts.append(midjnt)
                def_jnts.append(basejnt)
                mc.delete(mid_guide)
                #mc.skinCluster(eejnt, midjnt, basejnt, f'{prefix}_MainFeather_{num}_GEO', tsb=True)
                #if feathergrps == 'Sub02Feather':
                if guide == mainguides[-1]:
                    try:
                        side = prefix.split("_")[-1]
                        print(f'{side}{feathergrps}')
                        mc.skinCluster(*def_jnts, f'feathers{side}', toSelectedBones=True)
                    except Exception as e:
                        print(e)

                mc.select(clear=True)
                mc.select(main_surf[0])
                mc.select(basectrl_offset, add=True)
                print(main_surf)
                mc.UVPin()
                for attr in ["rotateX", "rotateY", "rotateZ"]:
                    # Find any nodes driving this attribute
                    connections = mc.listConnections(f"{basectrl_offset}.{attr}", s=True, d=False, plugs=True)
                    if connections:
                        for conn in connections:
                            mc.disconnectAttr(conn, f"{basectrl_offset}.{attr}")
                mc.aimConstraint(
                    f'{prefix}_MainFeatherAim_{num}_jnt',
                    basectrl_offset,
                    aimVector=(0, 1, 0),
                    upVector=(1, 0, 0),
                    mo=False,
                    weight=1.0,
                    #worldUpVector = (1,0,0),
                    worldUpType="objectrotation", worldUpObject =f'{prefix}_MainAimUp{num}_{ctrlname}'
                    #worldUpType = 'None'
                )

            #for subnum in ['01', '02']:



        bind_joints = [f'arm_{side}_01_JNT', f'arm_{side}_02_JNT', f'arm_{side}_03_JNT', f'arm_{side}_04_JNT', f'arm_{side}_05_JNT', f'arm_{side}_06_JNT', f'arm_{side}_07_JNT', f'arm_{side}_08_JNT']
        mc.skinCluster(bind_joints, main_surf)


        for cont in [ f'{prefix}_Shaper', f'{prefix}_Span']:
            rot = mc.xform(cont, q=True, ws=True, ro=True)
            trans = mc.xform(cont, q=True, ws=True, t=True)
            UEface.build_basic_control(name=f'{cont}', shape='ZTarrow', size=10.0, position=trans, rotation=rot)
        mc.parent(f'{prefix}_Shaper_{grpname}', f'{prefix}_Span_{ctrlname}')
        mc.parentConstraint(f'hand_{side}_01_switch_JNT', f'{prefix}_Span_{grpname}', mo=True)
        mc.addAttr(f'{prefix}_Shaper_{ctrlname}', longName="Full_Bend", attributeType="bool", defaultValue=True, keyable=True)
        mc.addAttr(f'{prefix}_Shaper_{ctrlname}', longName="Full_Twist", attributeType="bool", defaultValue=True, keyable=True)
        mult_node2 = mc.createNode("multiplyDivide", name=f"{prefix}_Shape_multNode")
        mc.connectAttr(f'{prefix}_Shaper_{ctrlname}.Full_Bend', f"{mult_node2}.input2X")
        mc.connectAttr(f'{prefix}_Shaper_{ctrlname}.Full_Twist', f"{mult_node2}.input2Y")
        mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateX", f"{mult_node2}.input1X")
        mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateY", f"{mult_node2}.input1Y")
        for rotoff in rot_offset_list:
            parts = rotoff.split('_')
        
            # The identifier is the second-to-last element
            identifier = parts[-2]
        
            if identifier == "base":
                mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateX", f"{rotoff}.rotateX")
                mc.connectAttr(f"{prefix}_Shaper_{ctrlname}.rotateY", f"{rotoff}.rotateY")
            else:
                mc.connectAttr(f"{mult_node2}.outputX", f"{rotoff}.rotateX")
                mc.connectAttr(f"{mult_node2}.outputY", f"{rotoff}.rotateY")

        max_val = 20

        # Objects
        control = f"{prefix}_Span_{ctrlname}"
        # 1. Add custom attributes
        if not mc.objExists(f"{control}.span_mult"):
            mc.addAttr(control, longName="span_mult", attributeType="double", defaultValue=1.0, keyable=True)
        if not mc.objExists(f"{control}.sensitivity"):
            mc.addAttr(control, longName="sensitivity", attributeType="double", defaultValue=1.0, keyable=True)

        # 2. Create MultiplyDivide node
        mult_node = mc.createNode("multiplyDivide", name=f"{control}_span_multNode")
        blend = mc.createNode("blendColors", name=f"{control}_span_blend")
        mc.connectAttr(f"{prefix}_Span_{ctrlname}.translateX", f"{blend}.color1R")

        # 3. Create remapValue node
        remap_node = mc.createNode("remapValue", name=f"{control}_span_remapNode")
        mc.setAttr(f"{remap_node}.inputMin", -max_val)
        mc.setAttr(f"{remap_node}.inputMax", max_val)
        mc.setAttr(f"{remap_node}.outputMin", 0)
        mc.setAttr(f"{remap_node}.outputMax", 2.0)

        # 4. Connect translateX to multiplyDivide input1X
        mc.connectAttr(f"{blend}.outputR", f"{mult_node}.input1X")

        # 5. Connect sensitivity and span_mult to input2 channels
        mc.connectAttr(f"{control}.sensitivity", f"{mult_node}.input2X")
        mc.connectAttr(f"{control}.span_mult", f"{mult_node}.input2Y")

        # 6. Connect multiplyDivide output to remapValue input
        mc.connectAttr(f"{mult_node}.outputX", f"{remap_node}.inputValue")

        # 7. Connect remapValue output to multiplyDivide input1Y
        mc.connectAttr(f"{remap_node}.outValue", f"{mult_node}.input1Y")

        # 8. Connect multiplyDivide outputY to target joint scaleX
        for aimjnt in full_aimjnt_list:
            mc.connectAttr(f"{mult_node}.outputY", f"{aimjnt}.scaleX")








        #clean Up
        
        #Arm Hookin
        mc.parentConstraint(f'arm_{side}_01_switch_JNT', f'Wing_{side}_Main_Feather_aim_01_{side}_CTRL_CNST_GRP', mo=True)
        mc.parentConstraint(f'arm_{side}_02_switch_JNT', f'Wing_{side}_Main_Feather_aim_02_{side}_CTRL_CNST_GRP', mo=True)
        mc.parentConstraint(f'hand_{side}_01_switch_JNT', f'Wing_{side}_Main_Feather_aim_03_{side}_CTRL_CNST_GRP', mo=True)
        mc.parentConstraint(f'hand_{side}_01_switch_JNT', f'Wing_{side}_Main_Feather_aim_04_{side}_CTRL_CNST_GRP', mo=True) #FIX THIS ONCE WE GET THE IK FINGER CONTROLS SET UP

        #Put Back in the Heirarchy 
        mc.parent(f'Wing_{side}_root_jnt',f'arm_{side}_01_JNT')
        mc.parent(f'Wing_{side}_feather_{side}_CTRL_CNST_GRP', module_grp_name)
        mc.parent(f'Wing_{side}_handle_{side}_CTRL_CNST_GRP', module_grp_name)
        mc.parent(f'Wing_{side}_upAim_{side}_CTRL_CNST_GRP', module_grp_name)
        mc.delete(f'Wing_{side}_curve', f'Wing_{side}_curve1')
        mc.parent(f'Wing_{side}_Main_loft', module_grp_name)
        mc.parent(f'Wing_{side}_Span_{side}_CTRL_CNST_GRP', module_grp_name)
        for num in ['01', '02', '03', '04']:
            mc.parent(f'Wing_{side}_Main_Feather_aim_{num}_{side}_CTRL_CNST_GRP', module_grp_name)
        mc.hide(f'Wing_{side}_handle_{side}_CTRL_CNST_GRP', f'Wing_{side}_MainFeatherAim_01_jnt', f'Wing_{side}_Main_loft') #f'Wing_{side}_Sub01FeatherAim_01_jnt', f'Wing_{side}_Sub02FeatherAim_01_jnt',
        mc.parent(module_grp_name, 'RIG')
