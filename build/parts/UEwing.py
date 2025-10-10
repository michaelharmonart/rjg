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

    
    @staticmethod
    def build_ik_spline_with_controls(aim_joints=None, prefix=None, sub=False, FeatherType=None):
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
                    shape='circle',
                    size=1.0,
                    color_rgb=(1, 1, 0),
                    position=pos,
                    rotation=(0, 0, 0)
                )
            
                # Parent cluster to control
                mc.parentConstraint(ctrl, cluster_handle,)

                close_offset = mc.group(empty=True, name=f'{prefix}Aim_{i:02}_ArmClose_offset')
                mc.xform(close_offset, ws=True, t=pos,)
                mc.parent(close_offset, offset)
                mc.parent(ctrl, close_offset)

            else:
                mc.parentConstraint(f"{prefix}_Main_Feather_aim_{i:02}_{ctrlname}", cluster_handle, mo=True)
        
        return ik_handle, curve

    @staticmethod
    def build_curve(guide_list, prefix, degree=3):
        ctrlname, grpname = UEwing.get_namestruc(prefix)
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
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        count = 0
        guides = []
        
        # We'll just loop up to some reasonable high number to check
        for i in range(1, 200):  
            num_str = f"{i:02}" if i < 10 else str(i)  # pad with zero if < 10
            name = f"{prefix}_{feather}_{num_str}_guide"
            
            if mc.objExists(name):
                guides.append(name)
                count += 1
        
        print(f"Found {count} feather guide(s): {guides}")
        return count, guides

    @staticmethod
    def place_joints_on_guide_curve(guides, feather_count, prefix, feathertype, up_controls=False):
        ctrlname, grpname = UEwing.get_namestruc(prefix)
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
            pos2 = mc.xform(f'{prefix}_{feathertype}Feather_{i+1:02}_guide', q=True, ws=True, t=True)
            tangent = UEwing.look_at_rotation(pos,pos2)
            
            jnt = mc.joint(p=pos, name=f"{prefix}_{feathertype}FeatherAim_{i+1:02}_jnt")
            joints.append(jnt)
            if up_controls == True:
                rot = mc.xform(jnt, q=True, ws=False, rotation=True)
                size = .2 if feathertype != "Main" else .5
                upctr, upgrp = UEface.build_basic_control(name=f'{prefix}_{feathertype}AimUp{i+1:02}', shape='circle', size=size, color_rgb=(1, 1, 0), position=pos, rotation=tangent)
                mc.parentConstraint(jnt, upgrp, mo=True)
                upgrps.append(upgrp)
                upctrs.append(upctr)
                mc.parent(upgrp, f'{prefix}_upAim_{grpname}')
                mc.select(jnt)

        # Delete temp curve
        mc.delete(curve)
        mc.parent(f"{prefix}_{feathertype}FeatherAim_01_jnt", f'{prefix}_feather_{grpname}')
        return joints, upgrps, upctrs


    @staticmethod
    def get_sub_groups(prefix):
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        """
        Returns a sorted list of sub-groups under the {prefix}_guides group,
        stripped of the prefix.
        """
        group_name = f"{prefix}_guides"
        if not mc.objExists(group_name):
            print(f"[WARN] Group '{group_name}' does not exist.")
            return []

        # List children of the group
        children = mc.listRelatives(group_name, children=True, fullPath=False) or []

        # Filter for sub-groups
        sub_groups = [c for c in children if c.startswith(f"{prefix}_Sub")]

        # Sort numerically
        sub_groups.sort(key=lambda x: int(''.join(filter(str.isdigit, x))))

        # Strip prefix
        stripped_sub_groups = [name.replace(f"{prefix}_", "", 1) for name in sub_groups]

        return stripped_sub_groups

    @staticmethod
    def build_sub_feathers(prefix=None, sub=None, main_surf=None,):
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        subcount, subguides = UEwing.count_feather_guides(prefix = prefix, feather=f'{sub}Feather')
        print(subcount, subguides, sub)
        sub_aimjnt_list = []
        aim_joints, upgrps, upctrs = UEwing.place_joints_on_guide_curve([f'{prefix}_WingAim_01_guide', f'{prefix}_WingAim_02_guide', f'{prefix}_WingAim_03_guide', f'{prefix}_WingAim_04_guide'], subcount, prefix, sub, up_controls=True ) 
        sub_aimjnt_list.extend(aim_joints)
        UEwing.build_ik_spline_with_controls(aim_joints=aim_joints, prefix=prefix, sub=True, FeatherType=sub)
        rot_offset_list = []
        base_offsets = []
        for guide in subguides:
            num = guide.split("_")[-2]
            ee_guide = f'{prefix}_{sub}Feather_{num}_ee_guide'
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
                overwrite_name=f'{prefix}{sub}Feather_{num}_base',
                scale=True,
                check_side=True,
                CTRL_Size=0.2,
                JNT_Size=0.5
            )
            midjnt, midctrl, midctrl_offset =UEface.Simple_joint_and_Control(
                mid_guide,
                orient=True,
                overwrite=True,
                overwrite_name=f'{prefix}{sub}Feather_{num}_mid',
                scale=True,
                check_side=True,
                CTRL_Size=0.2,
                JNT_Size=0.5
            )
            eejnt, eectrl, eectrl_offset =UEface.Simple_joint_and_Control(
                ee_guide,
                orient=True,
                overwrite=True,
                overwrite_name=f'{prefix}{sub}Feather_{num}_ee',
                scale=True,
                check_side=True,
                CTRL_Size=0.2,
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
                rot_offset = mc.group(empty=True, name=f'{prefix}_{sub}Feather_{num}_{nameing}_rotOffset')
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
                    mc.parent(jnt, f'{prefix}_root_jnt')
                    mc.parent(offset, f'{prefix}_feather_{grpname}')
            base_offsets.append(basectrl_offset)
            mc.delete(mid_guide)
            #mc.skinCluster(eejnt, midjnt, basejnt, f'{prefix}_{sub}Feather_{num}_GEO', tsb=True )
            
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
                f'{prefix}_{sub}FeatherAim_{num}_jnt',
                basectrl_offset,
                aimVector=(0, 1, 0),
                upVector=(1, 0, 0),
                mo=False,
                weight=1.0,
                worldUpType="objectrotation", worldUpObject =f'{prefix}_{sub}AimUp{num}_{ctrlname}'
                #worldUpVector = (1,0,0),
                #worldUpType = 'None'
            )
        return rot_offset_list, base_offsets, sub_aimjnt_list


    def build_wing(self):

        #group='Wing_L_guides'
        prefix = UEface.get_prefix_from_group(self.grp_name)
        grp = self.grp_name
        ctrlname, grpname = UEwing.get_namestruc(prefix)

        feather_grp = mc.group(em=True, name=f'{prefix}_feather_{grpname}')
        handle_grp = mc.group(em=True, name=f'{prefix}_handle_{grpname}')
        upAim_grp = mc.group(em=True, name=f'{prefix}_upAim_{grpname}')
        mc.select(clear=True)
        root_joint = mc.joint(name=f'{prefix}_root_jnt')
        fk_group = mc.group(em=True, name=f'{prefix}_FK_{grpname}')
        ik_group = mc.group(em=True, name=f'{prefix}_IK_{grpname}')

        maincount, mainguides = UEwing.count_feather_guides(prefix = prefix, feather='MainFeather')
        curve_offset = 1
        main_curve = UEwing.build_curve(mainguides, prefix)
        main_curve2 = mc.duplicate(main_curve)
        mc.move(0, 0,curve_offset, main_curve, r=True)
        mc.move(0, 0,-curve_offset, main_curve2, r=True)
        main_surf = mc.loft(main_curve, main_curve2, name=f'{prefix}_Main_loft')
        full_aimjnt_list = []
        aim_joints, upgrps, upctrs = UEwing.place_joints_on_guide_curve([f'{prefix}_WingAim_01_guide', f'{prefix}_WingAim_02_guide', f'{prefix}_WingAim_03_guide', f'{prefix}_WingAim_04_guide'], maincount, prefix, 'Main', up_controls=True) 
        full_aimjnt_list.extend(aim_joints)
        UEwing.build_ik_spline_with_controls(aim_joints=aim_joints, prefix=prefix, sub=False, FeatherType='Main')

        rot_offset_list = []
        base_offsets = []
        for guide in mainguides:
            num = guide.split("_")[-2]
            ee_guide = f'{prefix}_MainFeather_{num}_ee_guide'
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
                overwrite_name=f'{prefix}MainFeather_{num}_base',
                scale=True,
                check_side=True,
                CTRL_Size=0.2,
                JNT_Size=0.5
            )
            midjnt, midctrl, midctrl_offset =UEface.Simple_joint_and_Control(
                mid_guide,
                orient=True,
                overwrite=True,
                overwrite_name=f'{prefix}MainFeather_{num}_mid',
                scale=True,
                check_side=True,
                CTRL_Size=0.2,
                JNT_Size=0.5
            )
            eejnt, eectrl, eectrl_offset =UEface.Simple_joint_and_Control(
                ee_guide,
                orient=True,
                overwrite=True,
                overwrite_name=f'{prefix}MainFeather_{num}_ee',
                scale=True,
                check_side=True,
                CTRL_Size=0.2,
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
                rot_offset = mc.group(empty=True, name=f'{prefix}_MainFeather_{num}_{nameing}_rotOffset')
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
            base_offsets.append(basectrl_offset)
            mc.delete(mid_guide)
            #mc.skinCluster(eejnt, midjnt, basejnt, f'{prefix}_MainFeather_{num}_GEO', tsb=True)
            
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

        sub_groups = UEwing.get_sub_groups(prefix)
        print(sub_groups)
        for sub in sub_groups:
            subrot_offset_list, subbase_offsets, sub_aimjnt_list = UEwing.build_sub_feathers(prefix, sub, main_surf,)
            rot_offset_list.extend(subrot_offset_list)
            full_aimjnt_list.extend(sub_aimjnt_list)

        pre_jnt = None
        pre_ctrl = None
        armjnts = []
        armoffsets = []
        armctrls = []
        armcloses= []

        #arm Logic
        FKIKSwitch_pos = mc.xform(f'{prefix}_Close', q=True, ws=True, t=True)
        FKIKSwitch_CTL, FKIKSwitch_GRP = UEface.build_basic_control(name=f'{prefix}_FKIKSwitch', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=FKIKSwitch_pos, rotation=(0, 0, 0))
        mc.addAttr(FKIKSwitch_CTL, longName="FK_IK", attributeType="bool", keyable=True)
        rev_node = mc.createNode("reverse", name=f"{prefix}IKReverse")
        mc.connectAttr(f'{FKIKSwitch_CTL}.FK_IK', f'{rev_node}.inputX')


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
        mc.skinCluster(bind_joints, main_surf)
        pre_jnt = None


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
                CTRL_Size=0.5,
                JNT_Size=0.5,
                #bind=False
            )
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
        for num in ['01', '02', '03', '04']:
            if num != '01':
                mc.parentConstraint(f'{prefix}_{num}_bind_jnt', f'{prefix}_Main_Feather_aim_{num}_{grpname}', mo=True)
            mc.parentConstraint(f'{prefix}_{num}_FK_JNT', f'{prefix}_{num}_bind_jnt' )
        for cont in [f'{prefix}_Close', f'{prefix}_FeatherShaper', f'{prefix}_Span']:
            rot = mc.xform(cont, q=True, ws=True, ro=True)
            trans = mc.xform(cont, q=True, ws=True, t=True)
            UEface.build_basic_control(name=f'{cont}', shape='circle', size=2.0, position=trans, rotation=rot)
        mc.parent(f'{prefix}_FeatherShaper_{grpname}', f'{prefix}_Span_{ctrlname}')
        mc.parent(f'{prefix}_Span_{grpname}', f'{prefix}_04_FK_{ctrlname}')

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
        ikaimCTL, ikaimGRP = UEface.build_basic_control(name=f'{prefix}_IK_Aim', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=pv_pos, rotation=(0, 0, 0))
        
        ikhandel  = mc.ikHandle(
            name=f"{prefix}_ikHandle",
            sj=f"{prefix}_01_IK_jnt", 
            ee=f"{prefix}_03_IK_jnt", 
            sol="ikRPsolver"
        )[0]
        #print(ikhandel)

        mc.poleVectorConstraint(ikaimCTL, ikhandel)
        IK_Root_pos = mc.xform(f'{prefix}_01_guide', q=True, ws=True, t=True)
        IK_Root_CTL, IK_Root_GRP = UEface.build_basic_control(name=f'{prefix}_IK_Root', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=IK_Root_pos, rotation=(0, 0, 0))
        mc.parentConstraint(IK_Root_CTL, f"{prefix}_01_IK_jnt")

        IK_EE_pos = mc.xform(f'{prefix}_03_guide', q=True, ws=True, t=True)
        IK_EE_CTL, IK_EE_GRP = UEface.build_basic_control(name=f'{prefix}_IK_EE', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=IK_EE_pos, rotation=(0, 0, 0))
        mc.parentConstraint(IK_EE_CTL, ikhandel)

        IK_04_pos = mc.xform(f'{prefix}_04_guide', q=True, ws=True, t=True)
        IK_04_CTL, IK_04_GRP = UEface.build_basic_control(name=f'{prefix}_IK_04', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=IK_04_pos, rotation=(0, 0, 0))
        mc.parentConstraint(IK_04_CTL, f"{prefix}_04_IK_jnt")
        mc.parent(IK_04_GRP, IK_EE_CTL)
        for num in ['01', '02', '03', '04']:
            mc.parentConstraint(f'{prefix}_{num}_IK_jnt', f'{prefix}_{num}_bind_jnt', mo=True)
            mc.connectAttr(f'{FKIKSwitch_CTL}.FK_IK', f'{prefix}_{num}_bind_jnt_parentConstraint1.{prefix}_{num}_FK_JNTW0')
            mc.connectAttr(f'{rev_node}.outputX', f'{prefix}_{num}_bind_jnt_parentConstraint1.{prefix}_{num}_IK_jntW1')
        mc.pointConstraint( f'{prefix}_01_bind_jnt', FKIKSwitch_GRP, mo=True)
        mc.orientConstraint(f'{prefix}_IK_EE_{ctrlname}', f'{prefix}_03_IK_jnt')
        ##End


        mc.addAttr(f'{prefix}_FeatherShaper_{ctrlname}', longName="Full_Bend", attributeType="bool", defaultValue=True, keyable=True)
        mc.addAttr(f'{prefix}_FeatherShaper_{ctrlname}', longName="Full_Twist", attributeType="bool", defaultValue=True, keyable=True)
        mult_node2 = mc.createNode("multiplyDivide", name=f"{prefix}_Shape_multNode")
        mc.connectAttr(f'{prefix}_FeatherShaper_{ctrlname}.Full_Bend', f"{mult_node2}.input2X")
        mc.connectAttr(f'{prefix}_FeatherShaper_{ctrlname}.Full_Twist', f"{mult_node2}.input2Y")
        mc.connectAttr(f"{prefix}_FeatherShaper_{ctrlname}.rotateX", f"{mult_node2}.input1X")
        mc.connectAttr(f"{prefix}_FeatherShaper_{ctrlname}.rotateY", f"{mult_node2}.input1Y")
        for rotoff in rot_offset_list:
            parts = rotoff.split('_')
        
            # The identifier is the second-to-last element
            identifier = parts[-2]
        
            if identifier == "base":
                mc.connectAttr(f"{prefix}_FeatherShaper_{ctrlname}.rotateX", f"{rotoff}.rotateX")
                mc.connectAttr(f"{prefix}_FeatherShaper_{ctrlname}.rotateY", f"{rotoff}.rotateY")
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
        #FOLD Logic
        mc.addAttr(f'{prefix}_Close_{ctrlname}', longName="Jnt01_Mult", attributeType="double", defaultValue=0, keyable=True)
        mc.addAttr(f'{prefix}_Close_{ctrlname}', longName="Jnt02_Mult", attributeType="double", defaultValue=1, keyable=True)
        mc.addAttr(f'{prefix}_Close_{ctrlname}', longName="Jnt03_Mult", attributeType="double", defaultValue=1, keyable=True)
        mc.addAttr(f'{prefix}_Close_{ctrlname}', longName="Jnt04_Mult", attributeType="double", defaultValue=1, keyable=True)
        mc.addAttr(f'{prefix}_Close_{ctrlname}', longName="Span_Mult", attributeType="double", defaultValue=1, keyable=True)
        mc.addAttr(f'{prefix}_Close_{ctrlname}', longName="Driver_Mult", attributeType="double", defaultValue=1, keyable=True)

        in_value = -5
        fold_01 = -40
        fold_02 = 100
        fold_03 = -140
        fold_span = -40
        
        remap_node_01 = mc.createNode("remapValue", name=f"{prefix}_01fold_remapNode")
        mc.setAttr(f"{remap_node_01}.inputMin", -in_value)
        mc.setAttr(f"{remap_node_01}.inputMax", in_value)
        mc.setAttr(f"{remap_node_01}.outputMin", -fold_01)
        mc.setAttr(f"{remap_node_01}.outputMax", fold_01)

        remap_node_02 = mc.createNode("remapValue", name=f"{prefix}_02fold_remapNode")
        mc.setAttr(f"{remap_node_02}.inputMin", -in_value)
        mc.setAttr(f"{remap_node_02}.inputMax", in_value)
        mc.setAttr(f"{remap_node_02}.outputMin", -fold_02)
        mc.setAttr(f"{remap_node_02}.outputMax", fold_02)

        remap_node_03 = mc.createNode("remapValue", name=f"{prefix}_03fold_remapNode")
        mc.setAttr(f"{remap_node_03}.inputMin", -in_value)
        mc.setAttr(f"{remap_node_03}.inputMax", in_value)
        mc.setAttr(f"{remap_node_03}.outputMin", -fold_03)
        mc.setAttr(f"{remap_node_03}.outputMax", fold_03)

        remap_node_span = mc.createNode("remapValue", name=f"{prefix}_spanfold_remapNode")
        mc.setAttr(f"{remap_node_span}.inputMin", 0)
        mc.setAttr(f"{remap_node_span}.inputMax", in_value)
        mc.setAttr(f"{remap_node_span}.outputMin", 1)
        mc.setAttr(f"{remap_node_span}.outputMax", fold_span)
        
        mult_node3 = mc.createNode("multiplyDivide", name=f"{prefix}_fold_multNode")
        mult_node4 = mc.createNode("multiplyDivide", name=f"{prefix}_folding_multNode")
        
        mc.connectAttr(f'{prefix}_Close_{ctrlname}.Driver_Mult', f"{mult_node3}.input2X")
        mc.connectAttr(f'{prefix}_Close_{ctrlname}.translateX', f"{mult_node3}.input1X")

        mc.connectAttr(f"{mult_node3}.outputX", f'{remap_node_01}.inputValue')
        mc.connectAttr(f'{remap_node_01}.outValue', f'{mult_node4}.input1X')
        mc.connectAttr(f'{prefix}_Close_{ctrlname}.Jnt01_Mult', f"{mult_node4}.input2X")
        mc.connectAttr(f"{mult_node4}.outputX", f'{prefix}_01_FK_ArmClose_offset.rotateX')

        mc.connectAttr(f"{mult_node3}.outputX", f'{remap_node_02}.inputValue')
        mc.connectAttr(f'{remap_node_02}.outValue', f'{mult_node4}.input1Y')
        mc.connectAttr(f'{prefix}_Close_{ctrlname}.Jnt02_Mult', f"{mult_node4}.input2Y")
        mc.connectAttr(f"{mult_node4}.outputY", f'{prefix}_02_FK_ArmClose_offset.rotateX')
        
        mc.connectAttr(f"{mult_node3}.outputX", f'{remap_node_03}.inputValue')
        mc.connectAttr(f'{remap_node_03}.outValue', f'{mult_node4}.input1Z')
        mc.connectAttr(f'{prefix}_Close_{ctrlname}.Jnt03_Mult', f"{mult_node4}.input2Z")
        mc.connectAttr(f"{mult_node4}.outputZ", f'{prefix}_03_FK_ArmClose_offset.rotateX')
        mc.connectAttr(f"{mult_node3}.outputX", f'{remap_node_span}.inputValue')
        mc.connectAttr(f'{remap_node_span}.outValue', f"{blend}.color2R")

        aimremap_node_01 = mc.createNode("remapValue", name=f"{prefix}_01foldaim_remapNode")
        mc.setAttr(f"{aimremap_node_01}.inputMin", -in_value)
        mc.setAttr(f"{aimremap_node_01}.inputMax", in_value)
        mc.setAttr(f"{aimremap_node_01}.outputMin", -7)
        mc.setAttr(f"{aimremap_node_01}.outputMax", 7)

        aimremap_node_02 = mc.createNode("remapValue", name=f"{prefix}_02foldaim_remapNode")
        mc.setAttr(f"{aimremap_node_02}.inputMin", -in_value)
        mc.setAttr(f"{aimremap_node_02}.inputMax", in_value)
        mc.setAttr(f"{aimremap_node_02}.outputMin", 20)
        mc.setAttr(f"{aimremap_node_02}.outputMax", -20)

        aimremap_node_03 = mc.createNode("remapValue", name=f"{prefix}_03foldaim_remapNode")
        mc.setAttr(f"{aimremap_node_03}.inputMin", -in_value)
        mc.setAttr(f"{aimremap_node_03}.inputMax", in_value)
        mc.setAttr(f"{aimremap_node_03}.outputMin", -13)
        mc.setAttr(f"{aimremap_node_03}.outputMax", 13)

        aimremap_node_04 = mc.createNode("remapValue", name=f"{prefix}_04foldaim_remapNode")
        mc.setAttr(f"{aimremap_node_04}.inputMin", -in_value)
        mc.setAttr(f"{aimremap_node_04}.inputMax", in_value)
        mc.setAttr(f"{aimremap_node_04}.outputMin", -7)
        mc.setAttr(f"{aimremap_node_04}.outputMax", 7)

        aimremap_node_05 = mc.createNode("remapValue", name=f"{prefix}_05foldaim_remapNode")
        mc.setAttr(f"{aimremap_node_05}.inputMin", -in_value)
        mc.setAttr(f"{aimremap_node_05}.inputMax", in_value)
        mc.setAttr(f"{aimremap_node_05}.outputMin", -5)
        mc.setAttr(f"{aimremap_node_05}.outputMax", 5)

        mc.connectAttr(f"{mult_node3}.outputX", f'{aimremap_node_01}.inputValue')
        mc.connectAttr(f'{aimremap_node_01}.outValue', f'{prefix}Aim_01_ArmClose_offset.translateX')
        mc.connectAttr(f"{mult_node3}.outputX", f'{aimremap_node_02}.inputValue')
        mc.connectAttr(f'{aimremap_node_02}.outValue', f'{prefix}Aim_02_ArmClose_offset.translateX')
        mc.connectAttr(f"{mult_node3}.outputX", f'{aimremap_node_03}.inputValue')
        mc.connectAttr(f'{aimremap_node_03}.outValue', f'{prefix}Aim_02_ArmClose_offset.translateY')
        mc.connectAttr(f"{mult_node3}.outputX", f'{aimremap_node_04}.inputValue')
        mc.connectAttr(f'{aimremap_node_04}.outValue', f'{prefix}Aim_03_ArmClose_offset.translateX')
        mc.connectAttr(f"{mult_node3}.outputX", f'{aimremap_node_05}.inputValue')
        mc.connectAttr(f'{aimremap_node_05}.outValue', f'{prefix}Aim_03_ArmClose_offset.translateY')

        #Clean Up Wing

        mc.group(f'{prefix}_01_FK_JNT', f'{prefix}_01_IK_jnt', f'{prefix}_ikHandle', f'{prefix}_Main_loft', name=f'{prefix}_extraOffset_{grpname}')
        mc.parent(f'{prefix}_Close_{grpname}', f'{prefix}_01_FK_{ctrlname}')
        mc.delete(f'{prefix}_curve', f'{prefix}_curve1')
        mc.parent(f'{prefix}_IK_Aim_{grpname}', f'{prefix}_IK_Root_{ctrlname}' )
        mc.parent(f'{prefix}_IK_EE_{grpname}', f'{prefix}_IK_Root_{ctrlname}' )
        mc.parent(f'{prefix}_Span_{grpname}', world=True)
        mc.parentConstraint(f'{prefix}_IK_EE_{ctrlname}', f'{prefix}_Span_{grpname}', mo=True)
        mc.parentConstraint(f'{prefix}_03_FK_{ctrlname}', f'{prefix}_Span_{grpname}', mo=True)
        mc.group(f'{prefix}_Main_Feather_aim_01_{grpname}', f'{prefix}_Main_Feather_aim_02_{grpname}', f'{prefix}_Main_Feather_aim_03_{grpname}', f'{prefix}_Main_Feather_aim_04_{grpname}', name = f'{prefix}_aimcurve_{grpname}')
        mc.parent(f'{prefix}_01_FK_{grpname}', fk_group)
        mc.parent(f'{prefix}_IK_Root_{grpname}', ik_group)
        mc.connectAttr(f'{FKIKSwitch_CTL}.FK_IK', f'{prefix}_FK_{grpname}.visibility')
        mc.connectAttr(f'{rev_node}.outputX', f'{prefix}_IK_{grpname}.visibility')
        for side in ['L', 'R']:
            try:
                mc.connectAttr(f'{FKIKSwitch_CTL}.FK_IK', f'{prefix}_Span_{grpname}_parentConstraint1.Wing_{side}_03_FK_{ctrlname}W1')
            except:
                pass
        mc.connectAttr(f'{rev_node}.outputX', f'{prefix}_Span_{grpname}_parentConstraint1.{prefix}_IK_EE_{ctrlname}W0')

        #Scap
        jnt, ctrl, ctrl_offset =UEface.Simple_joint_and_Control(
                f'{prefix}_Scap',
                orient=True,
                overwrite=False,
                scale=True,
                check_side=True,
                CTRL_Size=0.2,
                JNT_Size=0.5)
        




        mastergrp = mc.group(em=True, name =f'{prefix}')
        mc.parent( f'{prefix}_FK_{grpname}', f'{prefix}_IK_{grpname}', ctrl)
        mc.parent(f'{prefix}_feather_{grpname}', f'{prefix}_handle_{grpname}',f'{prefix}_upAim_{grpname}',f'{prefix}_FKIKSwitch_{grpname}', f'{prefix}_extraOffset_{grpname}', f'{prefix}_Span_{grpname}', f'{prefix}_aimcurve_{grpname}', ctrl_offset, mastergrp)
        mc.parent(f'{prefix}_root_jnt',f'{prefix}_01_bind_jnt', jnt)
        mc.parent(jnt, 'chest_M_JNT')
        mc.parentConstraint('chest_M_02_CTRL', ctrl_offset, mo=True)
        #mc.parentConstraint(ctrl,f'{prefix}_feather_{grpname}', mo=True)
        mc.parentConstraint(ctrl, f'{prefix}_Main_Feather_aim_01_{grpname}', mo=True)
        mc.hide(f'{prefix}_handle_{grpname}', f'{prefix}_extraOffset_{grpname}')
        mc.parent(mastergrp, 'RIG')

