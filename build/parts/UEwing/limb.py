from typing import TYPE_CHECKING
import maya.cmds as mc
from rjg.build.UEface import UEface

if TYPE_CHECKING:
    from rjg.build.parts.UEwing.module import UEwing


def build_limb(wing: "UEwing", ctrl_group: str, part_group: str):
    prefix = wing.prefix
    side = prefix.split("_")[-1]
    ctrlname, grpname = (ctrl_group, part_group)
    mc.select(clear=True)
    wing.fk_group = mc.group(em=True, name=f"{prefix}_FK_{grpname}")
    wing.ik_group = mc.group(em=True, name=f"{prefix}_IK_{grpname}")

    # bind
    wing.limb_bind_joints = []
    pre_jnt = None
    for obj in [
        f"{prefix}_01_guide",
        f"{prefix}_02_guide",
        f"{prefix}_03_guide",
        f"{prefix}_04_guide",
    ]:
        # Get the base name and generate joint name
        base_name = obj.split("|")[-1].replace("_guide", "")
        joint_name = f"{base_name}_bind_JNT"

        # Clear selection before creating the joint to avoid parenting
        mc.select(clear=True)
        joint = mc.joint(name=joint_name)
        wing.limb_bind_joints.append(joint)

        # Match translation and rotation in world space
        pos = mc.xform(obj, q=True, ws=True, t=True)
        rot = mc.xform(obj, q=True, ws=True, ro=True)
        mc.xform(joint, ws=True, t=pos)
        # mc.xform(joint, ws=True, ro=rot)

        mc.setAttr(f"{joint}.jointOrientX", rot[0])
        mc.setAttr(f"{joint}.jointOrientY", rot[1])
        mc.setAttr(f"{joint}.jointOrientZ", rot[2])

        if pre_jnt != None:
            mc.parent(joint_name, pre_jnt)
        pre_jnt = joint_name
        # if obj == f"{prefix}_02_guide":
        #    mc.setAttr(f'{joint}.rotateOrder', 2)

    pre_jnt = None

    parjnts = ["01", "02", "03", "04"]

    pre_jnt = None
    pre_ctrl = None
    armjnts = []
    armoffsets = []
    armctrls = []
    armcloses = []

    # arm Logic
    FKIKSwitch_pos = mc.xform(f"{prefix}_Close", q=True, ws=True, t=True)
    FKIKSwitch_CTL, FKIKSwitch_GRP = UEface.build_basic_control(
        name=f"{prefix}_FKIKSwitch",
        shape="ZTgear",
        size=5.0,
        color_rgb=(1, 1, 0),
        position=FKIKSwitch_pos,
        rotation=(0, 0, 0),
    )
    mc.addAttr(FKIKSwitch_CTL, longName="FK_IK", attributeType="bool", keyable=True)
    rev_node = mc.createNode("reverse", name=f"{prefix}IKReverse")
    mc.connectAttr(f"{FKIKSwitch_CTL}.FK_IK", f"{rev_node}.inputX")

    # fk
    for guide in [
        f"{prefix}_01_guide",
        f"{prefix}_02_guide",
        f"{prefix}_03_guide",
        f"{prefix}_04_guide",
    ]:
        parts = guide.split("_")  # ["wing", "l", "01", "guide"]
        number = parts[-2]  # second to last = "01", "02", etc.
        print(number)
        jnt, ctrl, ctrl_offset = UEface.Simple_joint_and_Control(
            guide,
            orient=True,
            overwrite=True,
            overwrite_name=f"{prefix}_{number}_FK",
            scale=True,
            check_side=False,
            CTRL_Color=(0, 0, 1),
            CTRL_Size=3,
            JNT_Size=0.5,
            bind=False,
        )
        # mc.addAttr()
        rot = mc.xform(guide, q=True, ws=True, ro=True)
        trans = mc.xform(guide, q=True, ws=True, t=True)
        close_offset = mc.group(empty=True, name=f"{prefix}_{number}_FK_ArmClose_offset")
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
            mc.parent(ctrl_offset, wing.fk_group)

        armjnts.append(jnt)
        armoffsets.append(ctrl_offset)
        armctrls.append(ctrl)
        armcloses.append(close_offset)
    ########################################################## Come back to this
    for num in ["01", "02", "03", "04"]:
        mc.parentConstraint(f"{prefix}_{num}_FK_JNT", f"{prefix}_{num}_bind_JNT", mo=True)

    # ik
    IK_joints = []
    pre_jnt = None
    for obj in [
        f"{prefix}_01_guide",
        f"{prefix}_02_guide",
        f"{prefix}_03_guide",
        f"{prefix}_04_guide",
    ]:
        # Get the base name and generate joint name
        base_name = obj.split("|")[-1].replace("_guide", "")
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

    pv_pos = mc.xform(f"{prefix}_IK_Aim", q=True, ws=True, t=True)
    ikaimCTL, ikaimGRP = UEface.build_basic_control(
        name=f"{prefix}_IK_Aim",
        shape="locator_3D",
        size=20.0,
        color_rgb=(1, 1, 0),
        position=pv_pos,
        rotation=(0, 0, 0),
    )

    ikhandel = mc.ikHandle(
        name=f"{prefix}_ikHandle",
        sj=f"{prefix}_01_IK_jnt",
        ee=f"{prefix}_03_IK_jnt",
        sol="ikRPsolver",
    )[0]

    mc.poleVectorConstraint(ikaimCTL, ikhandel)
    IK_Root_pos = mc.xform(f"{prefix}_01_guide", q=True, ws=True, t=True)
    IK_Root_CTL, IK_Root_GRP = UEface.build_basic_control(
        name=f"{prefix}_IK_Root",
        shape="circle",
        size=5.0,
        color_rgb=(1, 1, 0),
        position=IK_Root_pos,
        rotation=(0, 0, 0),
    )
    mc.parentConstraint(IK_Root_CTL, f"{prefix}_01_IK_jnt", mo=True)

    IK_EE_pos = mc.xform(f"{prefix}_03_guide", q=True, ws=True, t=True)
    IK_EE_rot = mc.xform(f"{prefix}_03_guide", q=True, ws=True, rotation=True)
    IK_EE_CTL, IK_EE_GRP = UEface.build_basic_control(
        name=f"{prefix}_IK_EE",
        shape="circle",
        size=5.0,
        color_rgb=(1, 1, 0),
        position=IK_EE_pos,
        rotation=IK_EE_rot,
    )
    mc.parentConstraint(IK_EE_CTL, ikhandel, mo=True)

    IK_04_pos = mc.xform(f"{prefix}_04_guide", q=True, ws=True, t=True)
    IK_04_CTL, IK_04_GRP = UEface.build_basic_control(
        name=f"{prefix}_IK_04",
        shape="circle",
        size=5.0,
        color_rgb=(1, 1, 0),
        position=IK_04_pos,
        rotation=(0, 0, 0),
    )
    mc.parentConstraint(IK_04_CTL, f"{prefix}_04_IK_jnt", mo=True)
    mc.parent(IK_04_GRP, IK_EE_CTL)
    for num in ["01", "02", "03", "04"]:
        mc.parentConstraint(f"{prefix}_{num}_IK_jnt", f"{prefix}_{num}_bind_JNT", mo=True)
        mc.connectAttr(
            f"{FKIKSwitch_CTL}.FK_IK",
            f"{prefix}_{num}_bind_JNT_parentConstraint1.{prefix}_{num}_FK_JNTW0",
        )
        mc.connectAttr(
            f"{rev_node}.outputX",
            f"{prefix}_{num}_bind_JNT_parentConstraint1.{prefix}_{num}_IK_jntW1",
        )
    mc.pointConstraint(f"{prefix}_01_bind_JNT", FKIKSwitch_GRP, mo=True)
    mc.orientConstraint(f"{prefix}_IK_EE_{ctrlname}", f"{prefix}_03_IK_jnt", mo=True)

    max_val = 20

    # Clean Up Wing
    mc.group(
        f"{prefix}_01_FK_JNT",
        f"{prefix}_01_IK_jnt",
        f"{prefix}_ikHandle",
        name=f"{prefix}_extraOffset_{grpname}",
    )  # f'{prefix}_Main_loft'
    mc.parent(f"{prefix}_IK_Aim_{grpname}", f"{prefix}_IK_Root_{ctrlname}")
    mc.parent(f"{prefix}_IK_EE_{grpname}", f"{prefix}_IK_Root_{ctrlname}")
    mc.parent(f"{prefix}_IK_Root_{grpname}", wing.ik_group)
    mc.connectAttr(f"{FKIKSwitch_CTL}.FK_IK", f"{prefix}_FK_{grpname}.visibility")
    mc.connectAttr(f"{rev_node}.outputX", f"{prefix}_IK_{grpname}.visibility")
    jnt, ctrl, ctrl_offset = UEface.Simple_joint_and_Control(
        f"{prefix}_Scap",
        orient=True,
        overwrite=False,
        scale=True,
        check_side=True,
        CTRL_Size=10,
        JNT_Size=0.5,
    )

    mc.parent(f"{prefix}_FK_{grpname}", f"{prefix}_IK_{grpname}", ctrl)
    mc.parent(
        f"{prefix}_FKIKSwitch_{grpname}",
        f"{prefix}_extraOffset_{grpname}",
        ctrl_offset,
        wing.mastergrp,
    )  # f'{prefix}_upAim_{grpname}'f'{prefix}_Span_{grpname}'f'{prefix}_aimcurve_{grpname}'
    mc.parent(f"{prefix}_01_bind_JNT", jnt)  # f'{prefix}_root_jnt'
    mc.parent(jnt, "chest_M_JNT")
    mc.parentConstraint("chest_M_02_CTRL", ctrl_offset, mo=True)
    mc.hide(f"{prefix}_extraOffset_{grpname}")
    mc.parent(wing.mastergrp, "RIG")

    # proxy ik / fk switch
    for control in [
        f"Wing_{wing.side}_IK_04_{wing.side}_CTRL",
        f"Wing_{wing.side}_IK_EE_{wing.side}_CTRL",
        f"Wing_{wing.side}_IK_Root_{wing.side}_CTRL",
        f"Wing_{wing.side}_Scap_{wing.side}_CTRL",
        f"Wing_{wing.side}_IK_Aim_{wing.side}_CTRL",
        f"Wing_{wing.side}_01_FK_{wing.side}_CTRL",
        f"Wing_{wing.side}_02_FK_{wing.side}_CTRL",
        f"Wing_{wing.side}_03_FK_{wing.side}_CTRL",
        f"Wing_{wing.side}_04_FK_{wing.side}_CTRL",
    ]:
        mc.addAttr(
            control,
            longName="FK_IK_Switch",
            proxy=f"Wing_{wing.side}_FKIKSwitch_{wing.side}_CTRL.FK_IK",
        )
    mc.setAttr(f"Wing_{wing.side}_FKIKSwitch_{wing.side}_CTRL.FK_IK", 1)
