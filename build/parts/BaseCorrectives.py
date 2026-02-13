import maya.cmds as mc
import rjg.libs.transform as rXform
from importlib import reload
import rjg.build.rigModule as rModule
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
import rjg.libs.attribute as rAttr
reload(rModule)
reload(rChain)
reload(rCtrl)
reload(rAttr)


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
    tgt_influence = .5,
    rig_module=None,
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
        
        mc.addAttr(root_jnt, longName='tgt_rot_influence', k=True, dv=tgt_influence)

        oreintLoc = mc.spaceLocator(name=f'{tgt_limb}_{pop_descriptor}_Orient_LOC')[0]

        
        oreintgrp = mc.group(empty=True, name=f'{tgt_limb}_{pop_descriptor}_Orient_GRP')
        mc.parent(oreintLoc, oreintgrp)

        mc.setAttr(f'{oreintgrp}.translateX', root_pos[0])
        mc.setAttr(f'{oreintgrp}.translateY', root_pos[1])
        mc.setAttr(f'{oreintgrp}.translateZ', root_pos[2])

        mc.setAttr(f'{oreintgrp}.rotateX', rot[0])
        mc.setAttr(f'{oreintgrp}.rotateY', rot[1])
        mc.setAttr(f'{oreintgrp}.rotateZ', rot[2])

        #mc.parent(oreintLoc, oreintgrp)
        mc.parent(oreintgrp, 'CorrectiveRigParts')

        mdOrient = mc.createNode('multiplyDivide', name = f'{tgt_limb}_{pop_descriptor}_Orient_MD')
        for ax in ["X", "Y", "Z"]:
            mc.connectAttr(f'{tgt_limb}.rotate{ax}', f'{mdOrient}.input1{ax}')
            mc.connectAttr(f'{root_jnt}.tgt_rot_influence', f'{mdOrient}.input2{ax}')
            mc.connectAttr(f'{mdOrient}.output{ax}', f'{oreintLoc}.rotate{ax}',)
        
        orc = mc.orientConstraint(oreintLoc, root_jnt, mo=True)[0]
        mc.parentConstraint(par_jnt, oreintgrp, mo=True)
        #mc.connectAttr(f'{root_jnt}.tgt_rot_influence', f'{orc}.{tgt_limb}W0' )
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

    # Bind
    mc.select(par_jnt)
    bindjnt = mc.joint(n=f'{pop_descriptor}_JNT', p=root_pos)
    mc.setAttr(f"{bindjnt}.jointOrient", rot[0], rot[1], rot[2])
    mc.parentConstraint(end_jnt, bindjnt)
    rig_module.tag_bind_joints(bindjnt)

    mc.parent(null_jnt, 'CorrectiveRigParts')
    



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
    Control_parent=None,
    split=False,
    Extra_Twist=None,
    upClamp= 180,
    downClamp=-180,
    flip_pop=False,
    rig_module=None
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
    mc.setAttr(f'{root_jnt}.segmentScaleCompensate', 0)
    mc.select(clear=True)
    end_jnt = mc.joint(n=f'{mus_descriptor}_end_JNT', p=end_pos)
    mc.setAttr(f'{end_jnt}.segmentScaleCompensate', 0)

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
    mc.setAttr(f'{end_null}.segmentScaleCompensate', 0)

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
    mc.setAttr(f'{root_null}.segmentScaleCompensate', 0)

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
    mc.setAttr(f'{mid_jnt}.segmentScaleCompensate', 0)
    mc.select(clear=True)
    mid_null = mc.joint(n=f'{mus_descriptor}_midNULL_JNT', p=root_pos)
    mc.setAttr(f'{mid_null}.segmentScaleCompensate', 0)


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
    if mc.objExists(f'UP_{tgt_limb}_LOC'):
        upvectorloc = f'UP_{tgt_limb}_LOC'
    else:
        upvectorloc = mc.spaceLocator(name = f'UP_{tgt_limb}_LOC', r=False)[0]
        mc.xform(upvectorloc, t=(0,400,0), ws=True)
        mc.parentConstraint(par_jnt, upvectorloc, mo=True)
        mc.parent(upvectorloc, 'CorrectiveRigParts')
    mc.aimConstraint(
        tgt_loc,
        root_jnt,
        aimVector=(0, 1, 0),
        upVector=(0, 0, 1),
        worldUpType='object',
        worldUpObject=upvectorloc,
        mo=True
    )
    mc.parentConstraint(tgt_loc, end_null, mo=True)

    mc.addAttr(root_jnt, longName='PopMult', at='double', dv=pop_mult, k=True)
    mc.addAttr(root_jnt, longName='AutoRot', at='double', dv=-.5, k=True)
    mc.addAttr(root_jnt, longName='Slide_mult', at='double', dv=slide_mult, k=True)
    mc.addAttr(root_jnt, longName='upClamp', at='double', dv=upClamp, k=True)
    mc.addAttr(root_jnt, longName='downClamp', at='double', dv=downClamp, k=True)


    # ----------------------------
    # Mid translate driver (simple MD)
    # ----------------------------
    mid_md = mc.createNode('multiplyDivide', n=f'{mus_descriptor}_mid_translate_MD')
    mc.setAttr(f'{mid_md}.input2Y', 0.5)

    mc.connectAttr(f'{end_null}.translateY', f'{mid_md}.input1Y')
    mc.connectAttr(f'{mid_md}.outputY', f'{mid_null}.translateY')

    # ---- Divide normalize MD ----

    norm_md = mc.createNode('multiplyDivide', n=f'{mus_descriptor}_normalize_MD')
    mc.setAttr(f'{norm_md}.operation', 2)  # divide

    current_len = mc.getAttr(f'{end_null}.translateY')
    mc.setAttr(f'{norm_md}.input2Y', current_len)

    mc.connectAttr(f'{end_null}.translateY', f'{norm_md}.input1Y')

    # ---- Remap for scale up ----

    md = mc.createNode('multiplyDivide', name=f'{mus_descriptor}_MD')
    mdslide = mc.createNode('multiplyDivide', name=f'{mus_descriptor}Slide_MD')

    clampremap = mc.createNode('remapValue', name=f'{mus_descriptor}_clamp_remap')
    mc.connectAttr(f'{root_jnt}.upClamp', f'{clampremap}.inputMax')
    mc.connectAttr(f'{root_jnt}.upClamp', f'{clampremap}.outputMax')
    mc.connectAttr(f'{root_jnt}.downClamp', f'{clampremap}.inputMin')
    mc.connectAttr(f'{root_jnt}.downClamp', f'{clampremap}.outputMin')

    #if tgt_limb.startswith('clavicle_'):
    #    tgt_limb_pop, tgt_limb_stretch = tgt_limb_stretch, tgt_limb_pop

    if tgt_extra:
        #if tgt_extra.startswith('clavicle_'):
        #    exttgt_limb_pop, exttgt_limb_stretch = tgt_limb_stretch, tgt_limb_pop
        #else:
        #    exttgt_limb_pop, exttgt_limb_stretch = tgt_limb_pop, tgt_limb_stretch
        adpop = mc.createNode('addDL', name=f'{mus_descriptor}_pop_AD')
        mc.connectAttr(f'{tgt_limb}.rotate{tgt_limb_pop}', f'{adpop}.input1')
        mc.connectAttr(f'{tgt_extra}.rotate{tgt_limb_pop}', f'{adpop}.input2')
        mc.connectAttr(f'{adpop}.output', f'{clampremap}.inputValue')
        tgt_swingpop = f'{clampremap}.outValue'
        #tgt_swingpop = f'{adpop}.output'
        adstretch = mc.createNode('addDL', name=f'{mus_descriptor}_stretch_AD')
        mc.connectAttr(f'{tgt_limb}.rotate{tgt_limb_stretch}', f'{adstretch}.input1')
        mc.connectAttr(f'{tgt_extra}.rotate{tgt_limb_stretch}', f'{adstretch}.input2')
        tgt_swingstretch = f'{adstretch}.output'
    else:
        #tgt_swingpop = f'{tgt_limb}.rotate{tgt_limb_pop}'
        mc.connectAttr(f'{tgt_limb}.rotate{tgt_limb_pop}', f'{clampremap}.inputValue')
        tgt_swingpop = f'{clampremap}.outValue'
        tgt_swingstretch = f'{tgt_limb}.rotate{tgt_limb_stretch}'

    mc.connectAttr(tgt_swingpop, f'{md}.input1X')
    mc.connectAttr(tgt_swingpop, f'{md}.input1Y')
    mc.connectAttr(tgt_swingstretch, f'{md}.input1Z')
    mc.connectAttr(tgt_swingpop, f'{mdslide}.input1X')

    

    mc.connectAttr(f'{root_jnt}.AutoRot', f'{md}.input2X')
    mc.connectAttr(f'{root_jnt}.PopMult', f'{md}.input2Y')
    mc.connectAttr(f'{root_jnt}.AutoRot', f'{md}.input2Z')
    mc.connectAttr(f'{root_jnt}.Slide_mult', f'{mdslide}.input2X')

    if flip_pop == False:
        mc.connectAttr(f'{md}.outputX', f'{end_jnt}.rotate{tgt_limb_stretch}')
        mc.connectAttr(f'{md}.outputY', f'{mid_jnt}.translate{tgt_limb_pop}')
        mc.connectAttr(f'{md}.outputZ', f'{end_jnt}.rotate{tgt_limb_pop}')
        mc.connectAttr(f'{mdslide}.outputX', f'{mid_jnt}.translate{tgt_limb_twist}')
    elif flip_pop == True:
        mc.connectAttr(f'{md}.outputX', f'{end_jnt}.rotate{tgt_limb_stretch}')
        mc.connectAttr(f'{md}.outputY', f'{mid_jnt}.translate{tgt_limb_stretch}')
        mc.connectAttr(f'{md}.outputZ', f'{end_jnt}.rotate{tgt_limb_pop}')
        mc.connectAttr(f'{mdslide}.outputX', f'{mid_jnt}.translate{tgt_limb_twist}')


    # ------- Bind Jnts ---------- 
    bind_jnts = []
    for i, jnt in enumerate([root_jnt, mid_jnt, end_jnt]):

        pos = mc.xform(jnt, q=True, ws=True, t=True)
        if i == 0:
            mc.select(par_jnt)
            j = mc.joint(n=f'{mus_descriptor}_{i}_JNT', p=pos)
        else:
            mc.select( f'{mus_descriptor}_0_JNT')
            j = mc.joint(n=f'{mus_descriptor}_{i}_JNT', p=pos)

        rig_module.tag_bind_joints(j)

        mc.setAttr(f'{j}.segmentScaleCompensate', 0)
        mc.setAttr(f'{j}.jointOrientX', rot[0])
        mc.setAttr(f'{j}.jointOrientY', rot[1])
        mc.setAttr(f'{j}.jointOrientZ', rot[2])

        '''if i == 0:
            mc.parent(j, par_jnt)
        else:
            mc.parent(j, f'{mus_descriptor}_0_JNT')'''

        if i == 1 and buildControl:
            control = rCtrl.Control(parent=None, shape="hexagon", side=None, suffix='CTRL', name=f'{mus_descriptor}', axis='y', group_type='main', rig_type='primary', translate=mid_jnt, rotate=mid_jnt, ctrl_scale=1)
            mc.parentConstraint(control.ctrl, j)
            mc.parentConstraint(jnt, control.top)
        else:
            mc.parentConstraint(jnt, j)
        
        bind_jnts.append(j)

    try:
        mc.parent(root_null, control.top, top_grp, 'CorrectiveRigParts', )
    except:
        pass    

    if split:
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




def Build_Correctives(side='L'):
    rig_module = rModule.RigModule(side=side, part="correctives")
    if side != 'M':
        if side == 'L':
            SideShort = 'L'
            SideLong = 'Left'
            mod=1
        elif side == 'R':
            SideShort = 'R'
            SideLong = 'Right'
            mod=-1

    cor_root = mc.group(empty=True, name='CorrectiveRigParts')
    mc.parent(cor_root, 'RIG')


    mus_corrective_dict = {
    f"pec_{SideShort}_01": {
        "mus_root": f"{SideLong}_Pec01",
        "mus_end": f"{SideLong}_PecInsert",
        "tgt_limb": f"arm_{SideShort}_01_JNT",
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'X',
        "tgt_limb_stretch":'Z',
        "tgt_extra":f'clavicle_{SideShort}_01_JNT',
        "par_jnt":f'chest_M_JNT',
        "tgt_name":f'pec_{SideShort}_insert',
        "pop_mult":.05* mod,
        "slide_mult":0,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":f'arm_{SideShort}_02_JNT',
        "Control_parent":f'clavicle_{SideShort}_CTRL',
        "upClamp":90,
        "downClamp":-90,
        "flip_pop":False,
        },

    f"pec_{SideShort}_02": {
        "mus_root": f"{SideLong}_Pec02",
        "mus_end": f"{SideLong}_PecInsert",
        "tgt_limb": f"arm_{SideShort}_01_JNT",
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'X',
        "tgt_limb_stretch":'Z',
        "tgt_extra":f'clavicle_{SideShort}_01_JNT',
        "par_jnt":f'chest_M_JNT',
        "tgt_name":f'pec_{SideShort}_insert',
        "pop_mult":.05* mod,
        "slide_mult":0,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":f'arm_{SideShort}_02_JNT',
        "Control_parent":f'clavicle_{SideShort}_CTRL',
        "upClamp":90,
        "downClamp":-90,
        "flip_pop":False,
        },

    f"trap_{SideShort}_01": {
        "mus_root": f"{SideLong}_Trap01",
        "mus_end": f"{SideLong}_TrapInsert01",
        "tgt_limb": f"arm_{SideShort}_01_JNT",
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'X',
        "tgt_limb_stretch":'Z',
        "tgt_extra":f'clavicle_{SideShort}_01_JNT',
        "par_jnt":f'chest_M_JNT',
        "tgt_name":f'trap_{SideShort}_insert01',
        "pop_mult":.1* mod,
        "slide_mult":.1,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":f'arm_{SideShort}_02_JNT',
        "Control_parent":f'clavicle_{SideShort}_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":False,
        },
    f"trap_{SideShort}_02": {
        "mus_root": f"{SideLong}_Trap02",
        "mus_end": f"{SideLong}_TrapInsert01",
        "tgt_limb": f"arm_{SideShort}_01_JNT",
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'X',
        "tgt_limb_stretch":'Z',
        "tgt_extra":f'clavicle_{SideShort}_01_JNT',
        "par_jnt":f'chest_M_JNT',
        "tgt_name":f'trap_{SideShort}_insert01',
        "pop_mult":.09* mod,
        "slide_mult":-.1,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":f'arm_{SideShort}_02_JNT',
        "Control_parent":f'clavicle_{SideShort}_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":False,
        },
    f"trap_{SideShort}_03": {
        "mus_root": f"{SideLong}_Trap03",
        "mus_end": f"{SideLong}_TrapInsert03",
        "tgt_limb": f'clavicle_{SideShort}_01_JNT',
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'Z',
        "tgt_limb_stretch":'X',
        "tgt_extra":None,
        "par_jnt":f'neck_M_01_JNT',
        "tgt_name":f'trap_{SideShort}_insert03',
        "pop_mult":.05,
        "slide_mult":.2,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":None,
        "Control_parent":f'clavicle_{SideShort}_CTRL',
        "upClamp":45,
        "downClamp":-45,
        "flip_pop":False,
        },

    f"bicep_{SideShort}_01": {
        "mus_root": f"{SideLong}_Bicep01",
        "mus_end": f"{SideLong}_BicepInsert",
        "tgt_limb": f'arm_{SideShort}_05_JNT',
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'X',
        "tgt_limb_stretch":'Z',
        "tgt_extra":None,
        "par_jnt":f'arm_{SideShort}_02_JNT',
        "tgt_name":f'bicep_{SideShort}_insert',
        "pop_mult":.003,
        "slide_mult":-.05,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":None,
        "Control_parent":f'clavicle_{SideShort}_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":False,
        },

    f"delt_{SideShort}_01": {
        "mus_root": f"{SideLong}_Delt01",
        "mus_end": f"{SideLong}_DeltInsert",
        "tgt_limb": f'arm_{SideShort}_01_JNT',
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'X',
        "tgt_limb_stretch":'Z',
        "tgt_extra":None,
        "par_jnt":f'clavicle_{SideShort}_01_JNT',
        "tgt_name":f'delt_{SideShort}_insert',
        "pop_mult":.1 * mod,
        "slide_mult":-.1,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":None,
        "Control_parent":f'clavicle_{SideShort}_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":False,
        },

    f"delt_{SideShort}_02": {
        "mus_root": f"{SideLong}_Delt02",
        "mus_end": f"{SideLong}_DeltInsert",
        "tgt_limb": f'arm_{SideShort}_01_JNT',
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'Z',
        "tgt_limb_stretch":'X',
        "tgt_extra":None,
        "par_jnt":f'clavicle_{SideShort}_01_JNT',
        "tgt_name":f'delt_{SideShort}_insert',
        "pop_mult":.05,
        "slide_mult":-.1,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":None,
        "Control_parent":f'clavicle_{SideShort}_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":False,
        },

    f"delt_{SideShort}_03": {
        "mus_root": f"{SideLong}_Delt03",
        "mus_end": f"{SideLong}_DeltInsert",
        "tgt_limb": f'arm_{SideShort}_01_JNT',
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'X',
        "tgt_limb_stretch":'Z',
        "tgt_extra":None,
        "par_jnt":f'clavicle_{SideShort}_01_JNT',
        "tgt_name":f'delt_{SideShort}_insert',
        "pop_mult":-.1* mod,
        "slide_mult":.1,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":None,
        "Control_parent":f'clavicle_{SideShort}_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":False,
        },

    f"SCM_{SideShort}_03": {
        "mus_root": f"{SideLong}_SCM01",
        "mus_end": f"{SideLong}_SCMInsert",
        "tgt_limb": f'neck_M_03_JNT',
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'Z',
        "tgt_limb_stretch":'X',
        "tgt_extra":None,
        "par_jnt":f'neck_M_01_JNT',
        "tgt_name":f'SCM_{SideShort}_insert',
        "pop_mult":-.1 * mod,
        "slide_mult":0,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":True,
        "Extra_Twist":'head_M_JNT',
        "Control_parent":f'chest_M_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":False,
        },

        #HIP CORRECTIVES

    f"GluteMax_{SideShort}_01": {
        "mus_root": f"{SideLong}_GluteMax_01",
        "mus_end": f"{SideLong}_GluteMax_Insert",
        "tgt_limb": f'leg_{SideShort}_01_JNT',
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'X',
        "tgt_limb_stretch":'Z',
        "tgt_extra":None,
        "par_jnt":f'COG_M_JNT',
        "tgt_name":f'GluteMax_{SideShort}_insert',
        "pop_mult":-.15,
        "slide_mult":0,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":f'leg_{SideShort}_01_JNT',
        "Control_parent":f'hip_M_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":True,
        },

    f"TFL_{SideShort}_01": {
        "mus_root": f"{SideLong}_TFL_01",
        "mus_end": f"{SideLong}_TFL_Insert",
        "tgt_limb": f'leg_{SideShort}_01_JNT',
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'Z',
        "tgt_limb_stretch":'X',
        "tgt_extra":None,
        "par_jnt":f'COG_M_JNT',
        "tgt_name":f'TFL_{SideShort}_insert',
        "pop_mult":.2,
        "slide_mult":-.05,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":f'leg_{SideShort}_01_JNT',
        "Control_parent":f'hip_M_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":False,
        },

    f"Graci_{SideShort}_01": {
        "mus_root": f"{SideLong}_Graci_01",
        "mus_end": f"{SideLong}_Graci_02",
        "tgt_limb": f'leg_{SideShort}_01_JNT',
        "tgt_limb_twist":'Y',
        "tgt_limb_pop":'Z',
        "tgt_limb_stretch":'X',
        "tgt_extra":None,
        "par_jnt":f'COG_M_JNT',
        "tgt_name":f'Graci_{SideShort}_insert',
        "pop_mult":.1,
        "slide_mult":-0.05,
        "segments":1,
        "match_index":None,
        "buildControl":True,
        "split":False,
        "Extra_Twist":f'leg_{SideShort}_01_JNT',
        "Control_parent":f'hip_M_CTRL',
        "upClamp":180,
        "downClamp":-180,
        "flip_pop":False,
        },



    }


    pop_corrective_dict = {
    f"PSOAS_{SideShort}_01": {
        "pop_root" : f'{SideLong}UpLeg',
        "par_jnt" : f'COG_M_JNT',
        "pop_descriptor" : f'{SideShort}_PSOAS',
        "tgt_limb" : f'leg_{SideShort}_01_JNT',
        "blend_par" : [],
        "pop_mult" : .3,
        "tgt_limb_pop" : 'X',
        "pop" : 'Z',
        "buildControl" : True,
        "upClamp" : 180,
        "downClamp" : 0,
        "tgt_influence": .5
        },
    
    f"Kneecap_{SideShort}_01": {
        "pop_root" : f'{SideLong}Leg',
        "par_jnt" : f'leg_{SideShort}_04_JNT',
        "pop_descriptor" : f'{SideShort}_Kneecap',
        "tgt_limb" : f'leg_{SideShort}_05_JNT',
        "blend_par" : [],
        "pop_mult" : -.1,
        "tgt_limb_pop" : 'X',
        "pop" : 'Z',
        "buildControl" : True,
        "upClamp" : 0,
        "downClamp" : -90,
        "tgt_influence": .5
        },
    }

    for mus_descriptor, data in mus_corrective_dict.items():
        build_simple_muscle_chain(
            mus_descriptor=mus_descriptor,
            rig_module=rig_module,
            **data
        )

    for pop_descriptor, data in pop_corrective_dict.items():
        pop_corrective(
            rig_module=rig_module,
            **data
        )