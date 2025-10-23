import maya.cmds as mc
import re
import random

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

    #build_basic_control(name='name', size=10, color_rgb=(1,1,0), position=(0,0,0), rotation=(0,0,0))
    return ctrl, offset_grp

import maya.cmds as mc
import re
import random

def build_joint_control_chain(base_name='Stem', control_name='SF2_Master_CTRL'):
    """
    Builds control chains for joints following the pattern:
    {base_name}{letter}_{num}_jnt

    Each letter group gets its own chain and is driven by attributes
    on the specified master control.
    Each control now has an added EXTRAOFFSET group used for automated rotation input.
    """

    # Pattern: Stem + letter + _ + number + _jnt
    pattern = re.compile(rf"^{base_name}([A-Za-z])_(\d+)_jnt$")
    all_joints = mc.ls(type="joint") or []

    matched = []
    for jnt in all_joints:
        m = pattern.match(jnt)
        if m:
            letter, num = m.groups()
            matched.append((letter.upper(), int(num), jnt))

    if not matched:
        mc.warning(f"No joints found matching pattern '{base_name}{{letter}}_{{num}}_jnt'.")
        return

    # Sort alphabetically then numerically
    matched.sort(key=lambda x: (x[0], x[1]))

    # Group by letter
    grouped = {}
    for letter, num, jnt in matched:
        grouped.setdefault(letter, []).append((num, jnt))

    created_controls = []

    for letter, items in grouped.items():
        # Random but consistent color per letter
        random.seed(letter)
        color_rgb = (random.random(), random.random(), random.random())

        prev_ctrl = None
        extra_offsets = []  # track new extra offset groups

        # Build the control chain for this letter
        for num, jnt in sorted(items, key=lambda x: x[0]):
            pos = mc.xform(jnt, q=True, ws=True, t=True)
            rot = mc.xform(jnt, q=True, ws=True, ro=True)
            ctrl_name = f"{base_name}{letter}_{str(num).zfill(2)}"

            ctrl, grp = build_basic_control(
                name=ctrl_name,
                position=pos,
                rotation=rot,
                size=.5,
                color_rgb=color_rgb
            )

            # Create extra offset group
            extra_offset = mc.group(empty=True, name=f"{ctrl_name}_EXTRAOFFSET")
            mc.parent(extra_offset, grp)
            mc.parent(ctrl, extra_offset)

            # Match transforms to the parent grp
            mc.xform(extra_offset, ws=True, t=pos, ro=rot)

            # Parent constraints to joint
            mc.parentConstraint(ctrl, jnt, mo=False)
            mc.scaleConstraint(ctrl, jnt, mo=False)

            # Chain parenting
            if prev_ctrl:
                mc.parent(grp, prev_ctrl)
            prev_ctrl = ctrl

            extra_offsets.append(extra_offset)
            created_controls.append(ctrl)

        # === Add master control attributes for this letter ===
        main_attr = f"{base_name}{letter}_Main"
        sec_attr = f"{base_name}{letter}_Secondary"

        if not mc.objExists(f"{control_name}.{main_attr}"):
            mc.addAttr(control_name, ln=main_attr, at="double", dv=10, keyable=True)
        if not mc.objExists(f"{control_name}.{sec_attr}"):
            mc.addAttr(control_name, ln=sec_attr, at="double", dv=1, keyable=True)

        # === Create multiplyDivide node ===
        md_node = mc.createNode("multiplyDivide", n=f"{base_name}{letter}_MD")

        # Connect translateY to input1X and input1Y
        mc.connectAttr(f"{control_name}.translateY", f"{md_node}.input1X", f=True)
        mc.connectAttr(f"{control_name}.translateY", f"{md_node}.input1Y", f=True)

        # Connect master attributes to input2X and input2Y
        mc.connectAttr(f"{control_name}.{main_attr}", f"{md_node}.input2X", f=True)
        mc.connectAttr(f"{control_name}.{sec_attr}", f"{md_node}.input2Y", f=True)

        # Connect outputs to EXTRAOFFSET group rotations
        if extra_offsets:
            # First uses outputX (Main)
            mc.connectAttr(f"{md_node}.outputX", f"{extra_offsets[0]}.rotateZ", f=True)
            # Remaining use outputY (Secondary)
            for grp in extra_offsets[1:]:
                mc.connectAttr(f"{md_node}.outputY", f"{grp}.rotateZ", f=True)

        print(f"✅ Built chain for {base_name}{letter}: "
              f"{len(items)} controls, color={color_rgb}, driven by {control_name}")

    mc.select(created_controls)
    print(f"Created {len(created_controls)} total controls for '{base_name}'.")
    return created_controls


build_joint_control_chain('Stem', control_name='SF2_Master_CTRL')