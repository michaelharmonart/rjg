import maya.cmds as mc


def build_simple_muscle_chain(mus_root, mus_end, mus_descriptor, tgt_limb, tgt_limb_twist, tgt_limb_pop,):
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

        # Locator
        tgt_loc = mc.spaceLocator(n=f'{tgt_limb}_tgt_loc')[0]
        mc.xform(tgt_loc, ws=True, t=end_pos)

        loc_grp = mc.group(tgt_loc, n=f'{tgt_limb}_tgt_loc_GRP')
        mc.parent(loc_grp, offset_grp)

        mc.parentConstraint(tgt_limb, top_grp, mo=True)

    else:
        offset_grp = mc.listRelatives(top_grp, c=True, type='transform')[0]
        created_groups.extend([top_grp, offset_grp])
        tgt_loc = f'{tgt_limb}_tgt_loc'
        loc_grp = f'{tgt_limb}_tgt_loc_GRP'

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

    







    # ------- Split Jnts ----------

    split_joint = created_joints[0]
    split_joints: list[str] = [created_joints[0], created_joints[3], created_joints[1],]
    mc.addAttr(split_joint, longName="split_joints", dataType="string")
    mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")

    # ============================================================
    # ============================================================

    return {
        'joints': created_joints,
        'groups': created_groups,
        'locator': tgt_loc,
        'root_orient': root_orient,
        'end_orient': end_orient
    }
        
                
                                
build_simple_muscle_chain(
    mus_root='joint9',
    mus_end='joint10',
    mus_descriptor='pec01',
    tgt_limb='joint4',
    tgt_limb_twist='Y',
    tgt_limb_pop = 'X',
    tgt_extra = None
)
