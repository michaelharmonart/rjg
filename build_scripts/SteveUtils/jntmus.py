import maya.cmds as mc

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


def build_simple_muscle_chain(
    mus_root='joint9',
    mus_end='joint10',
    mus_descriptor='pec01',
    tgt_limb='joint4',
    tgt_limb_twist='Y',
    tgt_limb_pop = 'X',
    tgt_limb_stretch = 'Z',
    tgt_extra = None,
    par_jnt = 'joint2',
    tgt_name = 'pec_insert',
    pop_mult=.1,
    slide_mult = -2,
    segments = 1,
    match_index=None,
    buildControl=True,
    Control_parent=None
):
    created_joints = []
    created_groups = []

    # ----------------------------
    # Get guide positions
    # ----------------------------

    root_pos = mc.xform(mus_root, q=True, ws=True, t=True)
    end_pos = mc.xform(mus_end, q=True, ws=True, t=True)

    # ----------------------------
    # Create root and end joints
    # ----------------------------

    mc.select(clear=True)
    root_jnt = mc.joint(n=f'{mus_descriptor}_root_JNT', p=root_pos)
    mc.select(clear=True)
    end_jnt = mc.joint(n=f'{mus_descriptor}_end_JNT', p=end_pos)

    created_joints.extend([root_jnt, end_jnt])

    # ----------------------------
    # Aim root to end (bake into jointOrient)
    # ----------------------------

    aim = mc.aimConstraint(
        end_jnt,
        root_jnt,
        aimVector=(0, 1, 0),
        upVector=(0, 0, 1),
        worldUpType='vector',
        mo=False
    )[0]

    mc.delete(aim)

    # Bake rotations into jointOrient


    mc.select(clear=True)
    end_null = mc.joint(n=f'{mus_descriptor}_endNULL_JNT', p=end_pos)

    rot = mc.getAttr(f'{root_jnt}.rotate')[0]
    for j in [root_jnt, end_jnt, end_null]:
        mc.setAttr(f'{j}.jointOrientX', rot[0])
        mc.setAttr(f'{j}.jointOrientY', rot[1])
        mc.setAttr(f'{j}.jointOrientZ', rot[2])
        mc.setAttr(f'{j}.rotateX', 0)
        mc.setAttr(f'{j}.rotateY', 0)
        mc.setAttr(f'{j}.rotateZ', 0)

    # Parent end under root


    #mc.parent(end_jnt, root_jnt)
    mc.parent(end_jnt, end_null)
    mc.parent(end_null, root_jnt)


    # Store orient for later use if needed
    #root_orient = mc.getAttr(f'{root_jnt}.jointOrient')[0]
    #end_orient = mc.getAttr(f'{end_jnt}.jointOrient')[0]

    root_orient = rot
    end_orient = rot

    # ----------------------------
    # Root NULL joint
    # ----------------------------

    mc.select(clear=True)
    root_null = mc.joint(n=f'{mus_descriptor}_RootNULL_JNT', p=root_pos)

    mc.setAttr(f'{root_null}.jointOrientX', root_orient[0])
    mc.setAttr(f'{root_null}.jointOrientY', root_orient[1])
    mc.setAttr(f'{root_null}.jointOrientZ', root_orient[2])

    mc.parent(root_jnt, root_null)

    created_joints.append(root_null)

    

    # ----------------------------
    # Mid jointa
    # ----------------------------

    mc.select(clear=True)
    mid_jnt = mc.joint(n=f'{mus_descriptor}_mid_JNT', p=root_pos)
    mc.select(clear=True)
    mid_null = mc.joint(n=f'{mus_descriptor}_midNULL_JNT', p=root_pos)


    mc.setAttr(f'{mid_jnt}.jointOrientX', root_orient[0])
    mc.setAttr(f'{mid_jnt}.jointOrientY', root_orient[1])
    mc.setAttr(f'{mid_jnt}.jointOrientZ', root_orient[2])

    mc.setAttr(f'{mid_null}.jointOrientX', root_orient[0])
    mc.setAttr(f'{mid_null}.jointOrientY', root_orient[1])
    mc.setAttr(f'{mid_null}.jointOrientZ', root_orient[2])

    #mc.parent(mid_jnt, root_jnt)
    mc.parent(mid_null, root_jnt)
    mc.parent(mid_jnt, mid_null)


    created_joints.append(mid_jnt)


    # ----------------------------
    # Target locator + reusable groups
    # ----------------------------

    tgt_rot = mc.xform(tgt_limb, q=True, ws=True, ro=True)
    tgt_pos = mc.xform(tgt_limb, q=True, ws=True, t=True)

    top_grp = f'{tgt_limb}_tgt_GRP'
    offset_grp = f'{tgt_limb}_offset_GRP'

    if not mc.objExists(top_grp):
        top_grp = mc.group(em=True, n=top_grp)
        offset_grp = mc.group(em=True, n=offset_grp, p=top_grp)

        mc.xform(top_grp, ws=True, t=tgt_pos, ro=tgt_rot)

        created_groups.extend([top_grp, offset_grp])

        mc.parentConstraint(tgt_limb, top_grp, mo=True)

        


    else:
        offset_grp = mc.listRelatives(top_grp, c=True, type='transform')[0]
        created_groups.extend([top_grp, offset_grp])
        #tgt_loc = f'{tgt_limb}_tgt_loc'
        #loc_grp = f'{tgt_limb}_tgt_loc_GRP'

    # Locator
    if not mc.objExists(f'{tgt_limb}_{tgt_name}_tgt_loc'):

        tgt_loc = mc.spaceLocator(n=f'{tgt_limb}_{tgt_name}_tgt_loc')[0]
        mc.xform(tgt_loc, ws=True, t=end_pos)

        loc_grp = mc.group(tgt_loc, n=f'{tgt_limb}_{tgt_name}_tgt_loc_GRP')
        mc.parent(loc_grp, offset_grp)
    else:
        tgt_loc = f'{tgt_limb}_{tgt_name}_tgt_loc'
        loc_grp = f'{tgt_limb}_{tgt_name}_tgt_loc_GRP'

    


    # ----------------------------
    # Constraints
    # ----------------------------

    mc.aimConstraint(
        tgt_loc,
        root_jnt,
        aimVector=(0, 1, 0),
        upVector=(0, 0, 1),
        worldUpType='vector',
        mo=False
    )

    mc.parentConstraint(tgt_loc, end_null, mo=True)

    # ----------------------------
    # Mid translate driver (simple MD)
    # ----------------------------

    mid_md = mc.createNode('multiplyDivide', n=f'{mus_descriptor}_mid_translate_MD')
    mc.setAttr(f'{mid_md}.input2Y', 0.5)

    mc.connectAttr(f'{end_null}.translateY', f'{mid_md}.input1Y')
    mc.connectAttr(f'{mid_md}.outputY', f'{mid_null}.translateY')

    # ============================================================
    # ===================== FUN SECTION ==========================
    # ============================================================



    # ---- Divide normalize MD ----

    norm_md = mc.createNode('multiplyDivide', n=f'{mus_descriptor}_normalize_MD')
    mc.setAttr(f'{norm_md}.operation', 2)  # divide

    current_len = mc.getAttr(f'{end_null}.translateY')
    mc.setAttr(f'{norm_md}.input2Y', current_len)

    mc.connectAttr(f'{end_null}.translateY', f'{norm_md}.input1Y')

    # ---- Remap for scale up ----

    md = mc.createNode('multiplyDivide', name=f'{mus_descriptor}_MD')
    mdslide = mc.createNode('multiplyDivide', name=f'{mus_descriptor}Slide_MD')

    if tgt_extra:
        adpop = mc.createNode('addDL', name=f'{mus_descriptor}_pop_AD')
        mc.connectAttr(f'{tgt_limb}.rotate{tgt_limb_pop}', f'{adpop}.input1')
        mc.connectAttr(f'{tgt_extra}.rotate{tgt_limb_pop}', f'{adpop}.input2')
        tgt_swingpop = f'{adpop}.output'
        adstretch = mc.createNode('addDL', name=f'{mus_descriptor}_stretch_AD')
        mc.connectAttr(f'{tgt_limb}.rotate{tgt_limb_stretch}', f'{adstretch}.input1')
        mc.connectAttr(f'{tgt_extra}.rotate{tgt_limb_stretch}', f'{adstretch}.input2')
        tgt_swingstretch = f'{adstretch}.output'
    else:
        tgt_swingpop = f'{tgt_limb}.rotate{tgt_limb_pop}'
        tgt_swingstretch = f'{tgt_limb}.rotate{tgt_limb_stretch}'

    mc.connectAttr(tgt_swingpop, f'{md}.input1X')
    mc.connectAttr(tgt_swingpop, f'{md}.input1Y')
    mc.connectAttr(tgt_swingstretch, f'{md}.input1Z')
    mc.connectAttr(tgt_swingpop, f'{mdslide}.input1X')

    mc.addAttr(root_jnt, longName='PopMult', at='double', dv=pop_mult, k=True)
    mc.addAttr(root_jnt, longName='AutoRot', at='double', dv=-.5, k=True)
    mc.addAttr(root_jnt, longName='Slide_mult', at='double', dv=slide_mult, k=True)

    mc.connectAttr(f'{root_jnt}.AutoRot', f'{md}.input2X')
    mc.connectAttr(f'{root_jnt}.PopMult', f'{md}.input2Y')
    mc.connectAttr(f'{root_jnt}.AutoRot', f'{md}.input2Z')
    mc.connectAttr(f'{root_jnt}.Slide_mult', f'{mdslide}.input2X')

    #mc.setAttr(f'{md}.input2X', -.5)
    #mc.setAttr(f'{md}.input2Y', .1)
    #mc.setAttr(f'{md}.input2Z', -.5)

    mc.connectAttr(f'{md}.outputX', f'{end_jnt}.rotate{tgt_limb_stretch}')
    mc.connectAttr(f'{md}.outputY', f'{mid_jnt}.translate{tgt_limb_pop}')
    mc.connectAttr(f'{md}.outputZ', f'{end_jnt}.rotate{tgt_limb_pop}')
    mc.connectAttr(f'{mdslide}.outputX', f'{mid_jnt}.translate{tgt_limb_twist}')


    # ------- Split Jnts ----------
    
    bind_jnts = []
    for i, jnt in enumerate([root_jnt, mid_jnt, end_jnt]):

        pos = mc.xform(jnt, q=True, ws=True, t=True)
        mc.select(clear=True)
        j = mc.joint(n=f'{mus_descriptor}_{i}_JNT', p=pos)

        mc.setAttr(f'{j}.jointOrientX', rot[0])
        mc.setAttr(f'{j}.jointOrientY', rot[1])
        mc.setAttr(f'{j}.jointOrientZ', rot[2])

        if i == 0:
            mc.parent(j, par_jnt)
        else:
            mc.parent(j, f'{mus_descriptor}_0_JNT')

        if i == 1 and buildControl:
            ctrl, offset = build_basic_control(name=f'{mus_descriptor}', shape='circle', size=1.0, color_rgb=(1, 1, 0), position=pos, rotation=rot)
            mc.parentConstraint(ctrl, j)
            mc.parentConstraint(jnt, offset)
        else:
            mc.parentConstraint(jnt, j)
        
        bind_jnts.append(j)
        


    split_joint = bind_jnts[0]
    split_joints: list[str] = [bind_jnts[0], bind_jnts[1], bind_jnts[2],]
    mc.addAttr(split_joint, longName="split_joints", dataType="string")
    mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")
    mc.parentConstraint(par_jnt, root_null, mo=True)

    # ============================================================
    # ============================================================

    return {
        'joints': created_joints,
        'groups': created_groups,
        'locator': tgt_loc,
        'root_orient': root_orient,
        'end_orient': end_orient
    }
        




def pop_corrective(
    pop_root = 'joint1',
    par_jnt = 'COG_M_JNT',
    pop_descriptor = 'PSOAS',
    tgt_limb = 'leg_L_01_JNT',
    blend_par = [],
    pop_mult = .1,
    tgt_limb_pop = 'X',
    pop = 'Y',
    buildControl = True,
    upClamp = 180,
    downClamp = -180,
    ):

    root_pos = mc.xform(pop_root, q=True, ws=True, t=True)
    rot = mc.getAttr(f"{pop_root}.rotate")[0]

    # ----------------------------
    # Create root and end joints
    # ----------------------------

    mc.select(clear=True)
    null_jnt = mc.joint(n=f'{pop_descriptor}_NULL_JNT', p=root_pos)
    root_jnt = mc.joint(n=f'{pop_descriptor}_root_JNT', p=root_pos)
    mc.setAttr(f"{root_jnt}.jointOrient", rot[0], rot[1], rot[2])
    end_jnt = mc.joint(n=f'{pop_descriptor}_end_JNT', p=root_pos)


    if blend_par == []:
        mc.parentConstraint(par_jnt, null_jnt, mo=True)
        mc.orientConstraint(tgt_limb, root_jnt, mo=True)
    else:
        for i, tgt in enumerate(blend_par):
            con = mc.parentConstraint(tgt, root_jnt, mo=True)[0]
            attr_name = f"{tgt}_Influence"
            mc.addAttr(root_jnt, longName=f'{tgt}_Influence', at='double', dv=1)
            mc.connectAttr(f"{root_jnt}.{attr_name}", f"{con}.{tgt}W{i}")

    mc.addAttr(root_jnt, longName = 'Pop_Mult', dv=pop_mult, k=True)
    mc.addAttr(root_jnt, longName = 'Up_Clamp', dv=upClamp, k=True)
    mc.addAttr(root_jnt, longName = 'Down_Clamp', dv=downClamp, k=True)

    clamp = mc.createNode('remapValue', name=f'{pop_descriptor}_clamp_remap')
    mc.connectAttr(f'{root_jnt}.Up_Clamp', f'{clamp}.inputMax')
    mc.connectAttr(f'{root_jnt}.Up_Clamp', f'{clamp}.outputMax')
    mc.connectAttr(f'{root_jnt}.Down_Clamp', f'{clamp}.inputMin')
    mc.connectAttr(f'{root_jnt}.Down_Clamp', f'{clamp}.outputMin')
    mc.connectAttr(f'{tgt_limb}.rotate{tgt_limb_pop}', f'{clamp}.inputValue')

    md = mc.createNode('multiplyDivide', name=f'{pop_descriptor}_MD')
    mc.connectAttr(f'{clamp}.outValue', f'{md}.input1{pop}')
    mc.connectAttr(f'{root_jnt}.Pop_Mult', f'{md}.input2{pop}')

    mc.connectAttr(f'{md}.output{pop}', f'{end_jnt}.translate{pop}')

    



pop_corrective(
    pop_root = 'joint1',
    par_jnt = 'COG_M_JNT',
    pop_descriptor = 'PSOAS',
    tgt_limb = 'leg_L_01_JNT',
    blend_par = [],
    pop_mult = .1,
    tgt_limb_pop = 'X',
    pop = 'Z',
    buildControl = True,
    upClamp = 180,
    downClamp = -180,
    
    )
























                
                                
"""build_simple_muscle_chain(
    mus_root='joint9',
    mus_end='joint10',
    mus_descriptor='pec01',
    tgt_limb='joint4',
    tgt_limb_twist='Y',
    tgt_limb_pop = 'X',
    tgt_limb_stretch = 'Z',
    tgt_extra = 'joint3',
    par_jnt = 'joint2',
    tgt_name = 'pec_insert',
    pop_mult=.05,
    slide_mult = -2,
    segments = 1,
    match_index=None,
    buildControl=True

)
"""