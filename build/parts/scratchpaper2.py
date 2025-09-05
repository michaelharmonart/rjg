import maya.cmds as mc
import math
import re

def build_basic_control(name='Main', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=(0, 0, 0), rotation=(0, 0, 0)):
    """
    Builds a basic control with an offset group. The offset group holds the transform.
    Uses RGB override color instead of color index.

    Args:
        name (str): Control name.
        shape (str): Shape type (currently just 'circle' supported).
        size (float): Size of the control.
        color_rgb (tuple): RGB color override.
        position (tuple): World position (x, y, z).
        rotation (tuple): World rotation (x, y, z).

    Returns:
        ctrl (str): The name of the control.
        offset_grp (str): The name of the offset group.
    """
    # Create the control
    ctrl = mc.circle(name=f'{name}_CTRL', normal=[0, 1, 0], radius=size, ch=False)[0]

    # Create offset group
    offset_grp = mc.group(empty=True, name=f"{name}_GRP")
    mc.parent(ctrl, offset_grp)

    # Apply world-space transform to the group
    mc.xform(offset_grp, ws=True, translation=position, rotation=rotation)

    # Set control color using RGB
    mc.setAttr(f"{ctrl}.overrideEnabled", 1)
    mc.setAttr(f"{ctrl}.overrideRGBColors", 1)
    mc.setAttr(f"{ctrl}.overrideColorRGB", color_rgb[0], color_rgb[1], color_rgb[2], type="double3")
    return ctrl, offset_grp

    #build_basic_control(name='name', size=10, color_rgb=(1,1,0), position=(0,0,0), rotation=(0,0,0))

def add_to_face_bind_set(obj_name):
    """
    Adds the specified object to the selection set 'UE_Face_Bind'.
    If the set doesn't exist, it creates it first.

    Args:
        obj_name (str): The name of the object to add to the set.
    """
    set_name = 'UE_Face_Bind'

    if not mc.objExists(set_name):
        mc.sets(name=set_name)
        print(f"Created set: {set_name}")

    if mc.objExists(obj_name):
        mc.sets(obj_name, add=set_name)
        print(f"Added {obj_name} to {set_name}")
    else:
        print(f"Object '{obj_name}' does not exist.")

def Simple_joint_and_Control(
    guide,
    prefix=None,
    orient=True,
    overwrite=False,
    overwrite_name=None,
    scale=True,
    check_side=False,
    CTRL_Color=(0, 0, 1),
    CTRL_Size=0.2,
    JNT_Size=0.5
):
    """
    Creates a joint and control at the guide's position and orientation.

    Args:
        guide (str): Name of the guide object.
        prefix (str or None): Suffix to prepend (e.g. 'Nose'). If None, no prefix added.
        orient (bool): Use guide's rotation for joint/control.
        overwrite (bool): Use overwrite_name instead of guide name.
        overwrite_name (str): Optional. Custom name if overwrite=True.
        scale (bool): Whether to add a scaleConstraint.
        check_side (bool): Enable color override based on '_L_', '_R_', or '_M_' tags.
        CTRL_Color (tuple): RGB color for control (ignored if check_side=True).
        CTRL_Size (float): Control size.
        JNT_Size (float): Joint radius.

    Returns:
        tuple: (joint_name, control_name, control_offset_group)
    """

    if not mc.objExists(guide):
        mc.error(f"Guide '{guide}' does not exist.")
        return

    # Get world transform
    pos = mc.xform(guide, q=True, ws=True, t=True)
    rot = mc.xform(guide, q=True, ws=True, ro=True) if orient else [0, 0, 0]

    # Derive base name for naming
    if overwrite and overwrite_name:
        base_name = overwrite_name
    else:
        base_name = guide
        if prefix:
            # If prefix is part of the guide name, remove it from base_name
            prefix = f"{prefix}_"
            if base_name.startswith(prefix):
                base_name = base_name[len(prefix):]

    # Build final names
    if prefix:
        joint_name = f"{prefix}_{base_name}_JNT"
        ctrl_name = f"{prefix}_{base_name}"
    else:
        joint_name = f"{base_name}_JNT"
        ctrl_name = f"{base_name}"

    # Determine final control color
    final_color = CTRL_Color
    if check_side:
        if "_L_" in guide:
            final_color = (0, 0, 1)  # Blue
        elif "_R_" in guide:
            final_color = (1, 0, 0)  # Red
        elif "_M_" in guide:
            final_color = (1, 1, 0)  # Yellow

    # Create joint
    mc.select(clear=True)
    jnt = mc.joint(name=joint_name)
    add_to_face_bind_set(jnt)
    mc.xform(jnt, ws=True, t=pos, ro=rot)
    mc.setAttr(f"{jnt}.radius", JNT_Size)

    # Create control
    ctrl, ctrl_offset = build_basic_control(
        name=ctrl_name,
        size=CTRL_Size,
        color_rgb=final_color,
        position=pos,
        rotation=rot
    )

    # Constrain joint to control
    mc.parentConstraint(ctrl, jnt, mo=True)
    if scale:
        mc.scaleConstraint(ctrl, jnt, mo=True)

    return jnt, ctrl, ctrl_offset

def chain_parts(
    chain_names,
    jnt_parent=None,
    ctrl_parent=None,
    joints=True,
    controls=True
):
    """
    Chains together joints and/or controls based on an ordered list of guide base names.

    Args:
        chain_names (list): List of guide names (e.g., ['feather01_01', 'feather01_02', ...])
        jnt_parent (str or None): Optional. Name of the object to parent the first joint under.
        ctrl_parent (str or None): Optional. Name of the object to parent the first offset group under.
        joints (bool): Whether to chain joints (guide + '_JNT').
        controls (bool): Whether to chain controls (guide + '_GRP' under previous guide + '_CTRL').
    """
    if not chain_names or (not joints and not controls):
        return

    prev_jnt = None
    prev_ctrl = None

    for i, guide in enumerate(chain_names):
        jnt_name = f"{guide}_JNT"
        ctrl_name = f"{guide}_CTRL"
        grp_name = f"{guide}_GRP"

        # Chain joints
        if joints and mc.objExists(jnt_name):
            if i == 0:
                if jnt_parent and mc.objExists(jnt_parent):
                    mc.parent(jnt_name, jnt_parent)
            else:
                if prev_jnt and mc.objExists(prev_jnt):
                    mc.parent(jnt_name, prev_jnt)
            prev_jnt = jnt_name

        # Chain controls (via offset groups)
        if controls and mc.objExists(grp_name):
            if i == 0:
                if ctrl_parent and mc.objExists(ctrl_parent):
                    mc.parent(grp_name, ctrl_parent)
            else:
                if prev_ctrl and mc.objExists(prev_ctrl):
                    mc.parent(grp_name, prev_ctrl)
            prev_ctrl = ctrl_name  # we parent GRP, but track CTRL for next

    print(f"[INFO] Chained {'joints' if joints else ''} {'and' if joints and controls else ''} {'controls' if controls else ''} for {len(chain_names)} items.")

def strip_prefix_from_guide(prefix, guide_name):
    """
    Removes the prefix (and underscore) from the start of a guide name,
    and strips the leading underscore from the result (if any).

    Args:
        prefix (str): The prefix/prefix used (e.g. 'Nose').
        guide_name (str): The full guide name (e.g. 'Nose_M_Tip').

    Returns:
        str: The base guide name without the prefix (e.g. 'M_Tip').
    """
    prefix = f"{prefix}_"
    if guide_name.startswith(prefix):
        result = guide_name[len(prefix):]
        return result.lstrip('_')
    return guide_name

def control_rig(guide_list, side):
    ctrlnames = 'CTRL'
    #buildRig
    if side == 'L':
        sidelong = 'Left' 
    elif side == 'R':
        sidelong = 'Right'
    #FK rig  
    FKpre_jnt = None
    IKpre_jnt = None
    pre_ctrl = None
    for guide in guide_list:
        if guide == f'{sidelong}UpLeg':
            ctrlname = f'Leg_{side}_01'
            FKjntname = f'Leg_{side}_01_FK'
            IKjntname = f'Leg_{side}_01_IK'
        elif guide == f'{sidelong}Leg':
            ctrlname = f'Leg_{side}_02'
            FKjntname = f'Leg_{side}_02_FK'
            IKjntname = f'Leg_{side}_02_IK'
        elif guide == f'{sidelong}Knee':
            ctrlname = f'Leg_{side}_03'
            FKjntname = f'Leg_{side}_03_FK'
            IKjntname = f'Leg_{side}_03_IK'
        elif guide == f'{sidelong}Foot':
            ctrlname = f'Leg_{side}_04'
            FKjntname = f'Leg_{side}_04_FK'
            IKjntname = f'Leg_{side}_04_IK'
        elif guide == f'{sidelong}ToeBase':
            ctrlname = f'Foot_{side}'
            FKjntname = f'Leg_{side}_05_FK'
            IKjntname = f'Leg_{side}_05_IK'
        else:
            ctrlname = f'{guide}' 
            FKjntname = f'{guide}_FK'
            IKjntname = f'{guide}_IK'
        pos = mc.xform(guide, q=True, ws=True, t=True)
        rot = mc.xform(guide, q=True, ws=True, ro=True)
        #FK Controls
        ctrl, offset_grp = build_basic_control(name=ctrlname, shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=rot)
        if pre_ctrl != None:
            mc.parent(offset_grp, pre_ctrl)
            if guide in [f'{sidelong}IndexToe_EE', f'{sidelong}ThumbToe_EE', f'{sidelong}MiddleToe_EE', f'{sidelong}RingToe_EE', f'{sidelong}PinkyToe_EE']:
                pre_ctrl = f'Foot_{side}_{ctrlnames}'
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
                FKpre_jnt = f'Leg_{side}_05_FK'
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
                IKpre_jnt = f'Leg_{side}_05_IK'
            else:
                IKpre_jnt = IKjnt
        else:
            IKpre_jnt = IKjnt
    
    #IK rig
    #leg
    ikh1, eff1 = mc.ikHandle(sj=f'Leg_{side}_01_IK', ee=f'Leg_{side}_03_IK', sol=f"ikRPsolver", n=f"{side}_leg_IK1")
    ikh2, eff2 = mc.ikHandle(sj=f'Leg_{side}_02_IK', ee=f'Leg_{side}_04_IK', sol=f"ikRPsolver", n=f"{side}_leg_IK2")
    pos = mc.xform(f'{sidelong}UpLeg', q=True, ws=True, t=True)
    ctrl, offset_grp = build_basic_control(name=f'{side}_Hip', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
    mc.parentConstraint(ctrl, f'Leg_{side}_01_IK', mo=True)

    pos = mc.xform(f'{sidelong}Knee', q=True, ws=True, t=True)
    huckctrl, huckoffset_grp = build_basic_control(name=f'{side}_Huck', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
    mc.parentConstraint(huckctrl, ikh1, mo=True)

    pos = mc.xform(f'{sidelong}Foot', q=True, ws=True, t=True)
    footctrl, footoffset_grp = build_basic_control(name=f'{side}_Foot', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
    mc.parentConstraint(footctrl, ikh2, mo=True)
    mc.orientConstraint(footctrl, f'Leg_{side}_04_IK', mo=True)
    pos = mc.xform(f'{sidelong}ToeBase', q=True, ws=True, t=True)
    rootctrl, rootoffset_grp = build_basic_control(name=f'{side}_FootRoot', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
    mc.parent(footoffset_grp, rootctrl)
    mc.parent(huckoffset_grp, rootctrl)

    pos = mc.xform(f'{sidelong}IKAim01', q=True, ws=True, t=True)
    IKAim01ctrl, IKAim01offset_grp = build_basic_control(name=f'{side}_IKAim01', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
    mc.poleVectorConstraint(IKAim01ctrl, ikh1)

    pos = mc.xform(f'{sidelong}IKAim02', q=True, ws=True, t=True)
    IKAim02ctrl, IKAim02offset_grp = build_basic_control(name=f'{side}_IKAim02', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
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
    mc.joint(name=f'{side}_bankrollparent', position=midpoint)
    mc.joint(name=f'{side}_inner_bank', position=posin)
    mc.joint(name=f'{side}_outer_bank', position=posout)
    mc.joint(name=f'{side}_heel_roll', position=posheel)
    mc.joint(name=f'{side}_Toe_EE_roll', position=postoeee)
    mc.joint(name=f'{side}_Mid_roll', position=postoemid)
    mc.joint(name=f'{side}_child_Piv', position=poschild)

    mc.parentConstraint(f'{side}_child_Piv', footoffset_grp, mo=True)
    mc.parentConstraint(f'{side}_child_Piv', huckoffset_grp, mo=True)
    #mc.parentConstraint(f'{side}_child_Piv', rootoffset_grp, mo=True) #huckoffset_grp
    mc.parentConstraint(rootctrl, f'{side}_bankrollparent', mo=True)
    #mc.parent(f'{side}_inner_bank', rootctrl)

    pos = mc.xform(f'{sidelong}Options', q=True, ws=True, t=True)
    ctrl, offset_grp = build_basic_control(name=f'{side}_FootRoll', shape='circle', size=7.5, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
    mc.addAttr(ctrl, longName="FullRoll_Switch", attributeType="bool", defaultValue=False, keyable=True)
    Roll_Reverse = mc.shadingNode("reverse", asUtility=True, n=f"{side}Rol_rev")
    mc.connectAttr(f'{ctrl}.FullRoll_Switch', f'{Roll_Reverse}.inputX')
    #mc.connectAttr(f'{Roll_Reverse}.outputX', f'{footoffset_grp}_parentConstraint1.{side}_child_PivW0')
    mc.connectAttr(f'{ctrl}.FullRoll_Switch', f'{huckoffset_grp}_parentConstraint1.{side}_child_PivW0')

    bankin = mc.shadingNode('remapValue', asUtility=True, name=f'{side}bankin')
    bankout = mc.shadingNode('remapValue', asUtility=True, name=f'{side}bankout')
    rollheel = mc.shadingNode('remapValue', asUtility=True, name=f'{side}rollheel')
    rollend = mc.shadingNode('remapValue', asUtility=True, name=f'{side}rollend')
    rollmid = mc.shadingNode('remapValue', asUtility=True, name=f'{side}rollmid')
    if side == 'L':
        mod = 1
    if side == 'R':
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
    mc.connectAttr(f'{bankin}.outValue', f'{side}_inner_bank.rotateZ')
    mc.connectAttr(f'{bankout}.outValue', f'{side}_outer_bank.rotateZ')
    mc.connectAttr(f'{rollheel}.outValue', f'{side}_heel_roll.rotateX')
    mc.connectAttr(f'{rollend}.outValue', f'{side}_Toe_EE_roll.rotateX')
    mc.connectAttr(f'{rollmid}.outValue', f'{side}_Mid_roll.rotateX')

    mc.parent(offset_grp, rootctrl)

    #IK Toes
    slide1 = mc.shadingNode('remapValue', asUtility=True, name=f'{side}toeslide1')
    slide2 = mc.shadingNode('remapValue', asUtility=True, name=f'{side}toeslide2')
    mc.setAttr(f'{slide1}.inputMax', 40)
    mc.setAttr(f'{slide1}.outputMax', 10)
    mc.setAttr(f'{slide2}.inputMax', -20)
    mc.setAttr(f'{slide2}.outputMax', -10)
    mc.connectAttr(f'{ctrl}.translateZ', f'{slide1}.inputValue')
    mc.connectAttr(f'{ctrl}.translateZ', f'{slide2}.inputValue')

    for toe in ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']:
        ikhtoe, efftoe = mc.ikHandle(sj=f'{sidelong}{toe}Toe_Root_IK', ee=f'{sidelong}{toe}Toe_EE_IK', sol=f"ikRPsolver", n=f"{side}{toe}_IKHandle")
        pos = mc.xform(f'{sidelong}{toe}Toe_EE_IK', q=True, ws=True, t=True)
        ctrl, offset_grp = build_basic_control(name=f'{sidelong}{toe}Toe_IK', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
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
def output_rig(guide_list, side):
    if side == 'L':
        sidelong = 'Left' 
    elif side == 'R':
        sidelong = 'Right' 
    pos = mc.xform(f'{sidelong}Options', q=True, ws=True, t=True)
    ctrl, offset_grp = build_basic_control(name=f'{side}_Options', shape='circle', size=10.0, color_rgb=(1, 1, 0), position=pos, rotation=(0,0,0))
    mc.addAttr(ctrl, longName="FK_IK_Switch", attributeType="bool", defaultValue=False, keyable=True)
    mc.addAttr(ctrl, longName="IKStretch", attributeType="bool", defaultValue=True, keyable=True)
    mc.addAttr(ctrl, longName="Aim_01", at="enum", en="FootRoot:CTRL:World:Root", keyable=True)
    mc.addAttr(ctrl, longName="Aim_02", at="enum", en="FootRoot:CTRL:World:Root", keyable=True)

    #IK Stretch
    prejnt = None
    restlength = 0
    for joint in [f'Leg_{side}_01_IK', f'Leg_{side}_02_IK', f'Leg_{side}_03_IK', f'Leg_{side}_04_IK' ]:
        if prejnt != None: 
            p1 = mc.xform(prejnt,   q=True, ws=True, t=True)
            p2 = mc.xform(joint, q=True, ws=True, t=True)
            seg_len = ((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2) ** 0.5
            restlength = restlength + seg_len
            prejnt = joint
        else:
            prejnt = joint

    dist = mc.createNode('distanceBetween', name=f"{side}_IKMaths" + "_stretch_DIST")
    mc.connectAttr(f'{side}_FootRoot_CTRL' + '.worldMatrix[0]', dist + '.inMatrix1')
    mc.connectAttr(f'{side}_Hip_CTRL' + '.worldMatrix[0]', dist + '.inMatrix2')
    mdl = mc.createNode('multDoubleLinear', name=f"{side}_IKMaths" + "_stretch_MDL")
    mc.setAttr(mdl + '.input1', 1)              # no global scale, just identity
    #mc.connectAttr(scale_attr, mdl + '.input1')
    mc.setAttr(mdl + '.input2', restlength)

    mdn = mc.createNode('multiplyDivide', name=f"{side}_IKMaths" + "_stretch_MDN")
    mc.connectAttr(dist + '.distance', mdn + '.input1X')
    mc.connectAttr(mdl + '.output',    mdn + '.input2X')
    mc.setAttr(mdn + '.operation', 2)
    stretch_cond = mc.createNode('condition', name=f"{side}_IKMaths" + "_stretch_COND")

    # stretch when distance ≥ rest
    mc.connectAttr(dist + '.distance', stretch_cond + '.firstTerm')
    mc.connectAttr(mdl  + '.output',   stretch_cond + '.secondTerm')
    mc.connectAttr(mdn  + '.outputX',  stretch_cond + '.colorIfTrueR')
    mc.setAttr(stretch_cond + '.operation', 3)   # Greater or Equal
    mc.setAttr(stretch_cond + '.colorIfFalseR', 1)  # neutral value when inactive

    stretch_bta = mc.createNode('blendTwoAttr', name=f"{side}_IKMaths" + "_stretch_switch_BTA")

    mc.setAttr(stretch_bta + '.input[0]', 1)
    mc.connectAttr(stretch_cond + '.outColorR', stretch_bta + '.input[1]')
    mc.connectAttr(f'{ctrl}.IKStretch', stretch_bta + '.attributesBlender')


    # ---------------------------------------------------
    # 5) Combine stretch & squash and drive the joints
    # ---------------------------------------------------
    mult = mc.createNode('multiplyDivide', name=f"{side}_IKMaths" + '_squash_stretch_MDN')
    mc.connectAttr(stretch_bta + '.output', mult + '.input1X')

    # Apply final factor to the length axis (Y here) of each joint except end
    for joint in [f'Leg_{side}_01_IK', f'Leg_{side}_02_IK', f'Leg_{side}_03_IK']:
        mc.connectAttr(mult + '.outputX', joint + '.scaleY')
    

# create chain from joints
def skeleton(guide_list, side):
    # create chain
    # tag any bind joints
    if side == 'L':
        sidelong = 'Left' 
    elif side == 'R':
        sidelong = 'Right' 
    pre_jnt = None

    for guide in guide_list:
        if guide == f'{sidelong}UpLeg':
            jntname = f'Leg_{side}_01_bindJNT'
        elif guide == f'{sidelong}Leg':
            jntname = f'Leg_{side}_02_bindJNT'
        elif guide == f'{sidelong}Knee':
            jntname = f'Leg_{side}_03_bindJNT'
        elif guide == f'{sidelong}Foot':
            jntname = f'Leg_{side}_04_bindJNT'
        elif guide == f'{sidelong}ToeBase':
            jntname = f'Leg_{side}_05_bindJNT'
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
                pre_jnt = f'Leg_{side}_05_bindJNT'
            else:
                pre_jnt = jnt
        else:
            pre_jnt = jnt

    

# create extra attributes to be used during finalization stage (see ../post/finalize.py)
def add_plugs(guide_list, side):
    #addPlugs
    if side == 'L':
        sidelong = 'Left' 
    elif side == 'R':
        sidelong = 'Right'
    grpname = 'GRP'
    ctrlnames = 'CTRL'


    mc.parentConstraint(f'Leg_{side}_03_bindJNT', f'{side}_Options_{grpname}', mo=True)

    bindjnts = []

    #IKFK Switch
    mc.group(f'Leg_{side}_01_{grpname}', name=f'{side}_FK_{grpname}')
    mc.group(f'{side}_Hip_{grpname}', f'{side}_FootRoot_{grpname}', f'{side}_IKAim01_{grpname}', f'{side}_IKAim02_{grpname}', name=f'{side}_IK_{grpname}')
    IK_Reverse = mc.shadingNode("reverse", asUtility=True, n=f"{side}ikFk_rev")
    mc.connectAttr(f'{side}_Options_{ctrlnames}.FK_IK_Switch', f'{IK_Reverse}.inputX')
    mc.connectAttr(f'{side}_Options_{ctrlnames}.FK_IK_Switch', f'{side}_FK_{grpname}.visibility')
    mc.connectAttr(f'{IK_Reverse}.outputX', f'{side}_IK_{grpname}.visibility')

    for guide in guide_list:
        if guide == f'{sidelong}UpLeg':
            basename = f'Leg_{side}_01'
        elif guide == f'{sidelong}Leg':
            basename = f'Leg_{side}_02'
        elif guide == f'{sidelong}Knee':
            basename = f'Leg_{side}_03'
        elif guide == f'{sidelong}Foot':
            basename = f'Leg_{side}_04'
        elif guide == f'{sidelong}ToeBase':
            basename = f'Leg_{side}_05'
        else:
            basename = f'{guide}'
        
        mc.parentConstraint(f'{basename}_FK', f'{basename}_bindJNT', mo=True)
        mc.parentConstraint(f'{basename}_IK', f'{basename}_bindJNT', mo=True)
        mc.connectAttr(f'{side}_Options_{ctrlnames}.FK_IK_Switch', f'{basename}_bindJNT_parentConstraint1.{basename}_FKW0')
        mc.connectAttr(f'{IK_Reverse}.outputX', f'{basename}_bindJNT_parentConstraint1.{basename}_IKW1')
        bindjnts.append(f'{basename}_bindJNT')
    
    #Aim Spaces
    for aim in ['01', '02']:
        pc = mc.parentConstraint(f'{side}_FootRoot_{ctrlnames}', f'{side}_IKAim{aim}_{grpname}', mo=True)
        weights = mc.parentConstraint(pc[0], q=True, wal=True)
        cond1 = mc.createNode("condition", n=f"{side}{aim}foot_cond")
        mc.connectAttr(f'{side}_Options_{ctrlnames}.Aim_{aim}', f"{cond1}.firstTerm")
        mc.setAttr(f"{cond1}.secondTerm", 0)
        mc.setAttr(f"{cond1}.operation", 0)
        mc.setAttr(f"{cond1}.colorIfTrueR", 1)
        mc.setAttr(f"{cond1}.colorIfFalseR", 0)
        mc.connectAttr(f"{cond1}.outColorR", f'{side}_IKAim{aim}_{grpname}_parentConstraint1.{weights[0]}')
        #L_IKAim01_GRP_parentConstraint1.L_FootRoot_CTRLW0

        control = f"{side}_Huck_{ctrlnames}" if aim == "01" else f"{side}_Foot_{ctrlnames}"
        pc = mc.parentConstraint(control, f'{side}_IKAim{aim}_{grpname}', mo=True)
        weights = mc.parentConstraint(pc[0], q=True, wal=True)
        cond2 = mc.createNode("condition", n=f"{side}{aim}{control}_cond")
        mc.connectAttr(f'{side}_Options_{ctrlnames}.Aim_{aim}', f"{cond2}.firstTerm")
        mc.setAttr(f"{cond2}.secondTerm", 1)
        mc.setAttr(f"{cond2}.operation", 0)
        mc.setAttr(f"{cond2}.colorIfTrueR", 1)
        mc.setAttr(f"{cond2}.colorIfFalseR", 0)
        mc.connectAttr(f"{cond2}.outColorR", f'{side}_IKAim{aim}_{grpname}_parentConstraint1.{weights[1]}')

        if mc.objExists('global_M_CTRL'):
            pc = mc.parentConstraint('global_M_CTRL', f'{side}_IKAim{aim}_{grpname}', mo=True)
            weights = mc.parentConstraint(pc[0], q=True, wal=True)
            cond3 = mc.createNode("condition", n=f"{side}{aim}world_cond")
            mc.connectAttr(f'{side}_Options_{ctrlnames}.Aim_{aim}', f"{cond3}.firstTerm")
            mc.setAttr(f"{cond3}.secondTerm", 3)
            mc.setAttr(f"{cond3}.operation", 0)
            mc.setAttr(f"{cond3}.colorIfTrueR", 1)
            mc.setAttr(f"{cond3}.colorIfFalseR", 0)
            mc.connectAttr(f"{cond3}.outColorR", f'{side}_IKAim{aim}_{grpname}_parentConstraint1.{weights[2]}')
        
    mc.group(f'Leg_{side}_01_FK', f'Leg_{side}_01_IK', f'{side}_leg_IK1', f'{side}_leg_IK2', f'{side}_leg_IK1', f'{side}_bankrollparent', f'{side}Thumb_IKHandle', f'{side}Index_IKHandle', f'{side}Middle_IKHandle', f'{side}Ring_IKHandle', f'{side}Pinky_IKHandle', name=f'{side}_extra_GRP')
    mc.hide(f'{side}_extra_{grpname}')
    mc.group(f'{side}_extra_GRP', f'{side}_FK_GRP', f'{side}_IK_GRP', f'{side}_Options_{grpname}', name = f'leg_{side}')
    #Plugs f'Leg_{side}_01_bindJNT' f'{side}_Hip_{grpname}' f'{side}_FootRoot_{grpname}' f'Leg_L_01_{grpname}'
        


    #TEMP skin
    mc.select(bindjnts, 'Luciana_UBM1')
    mc.skinCluster(tsb=True)

def create_module(guide_list, side):

    control_rig(guide_list, side)
    output_rig(guide_list, side)
    skeleton(guide_list, side)
    add_plugs(guide_list, side)


guide_list = ['LeftUpLeg', 'LeftLeg', 'LeftKnee', 'LeftFoot', 'LeftToeBase', 'LeftMiddleToe_Root', 'LeftMiddleToe_Mid', 'LeftMiddleToe_EE', 'LeftIndexToe_Root', 'LeftIndexToe_MId', 'LeftIndexToe_EE', 'LeftRingToe_Root', 'LeftRingToe_Mid', 'LeftRingToe_EE', 'LeftPinkyToe_Root', 'LeftPinkyToe_Mid', 'LeftPinkyToe_EE', 'LeftThumbToe_Root', 'LeftThumbToe_Mid', 'LeftThumbToe_EE']
side = 'L'
create_module(guide_list, side)
