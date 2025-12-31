import ast
from collections.abc import Collection
from typing import TYPE_CHECKING
from maya.api.OpenMaya import MPoint, MVector
import maya.cmds as mc

from rjg.build.parts.UEwing.spline_system import (
    Spline,
    create_mid_guides,
    create_pin_on_curve,
    create_pin_on_net,
    create_swing_pin_on_curve,
    create_swing_transform,
    get_world_position,
)
from rjg.build.UEface import UEface
from rjg.libs.transform import matrix_constraint

if TYPE_CHECKING:
    from rjg.build.parts.UEwing.module import UEwing


def get_split_joints(joint: str) -> list[str] | None:
    if mc.objExists(f"{joint}.split_joints"):
        value = mc.getAttr(f"{joint}.split_joints")
        evaluated = ast.literal_eval(value)
        if not isinstance(evaluated, list):
            return None
        return evaluated
 
def get_closest_transform(transform: str, target_transforms: Collection[str]) -> str:
    transform_position: MPoint = MPoint(get_world_position(transform))
    closest: str | None = None
    min_dist: float | None = None
    for target_transform in target_transforms:
        target_position: MPoint = MPoint(get_world_position(target_transform))
        distance = transform_position.distanceTo(target_position)
        if min_dist is None:
            min_dist = distance
            closest = target_transform
        elif distance < min_dist:
            min_dist = distance
            closest = target_transform
    return closest            

def get_closest_split_joint(transform: str, joint: str) -> str:
    split_joints = get_split_joints(joint)
    if split_joints is not None:
        return get_closest_transform(transform, split_joints)
    return joint

def build_feathers(wing: "UEwing", keep_spacing: bool = True):
    prefix = wing.prefix
    side = prefix.split("_")[-1]
    wing.feather_grp = mc.group(em=True, name=f"{wing.prefix}_feather", parent=wing.mastergrp)
    wing.spline_grp = mc.group(em=True, name=f"{wing.prefix}_spline", parent=wing.mastergrp)
    wing.net_grp = mc.group(em=True, name=f"{wing.prefix}_net", parent=wing.mastergrp)
    mc.hide(wing.spline_grp)
    feather = "MainFeather"
    guides = wing.get_guides(prefix=prefix, feather=feather)
    root_list = [guide[0] for guide in guides]
    mid_list = [guide[1] for guide in guides]
    aim_list = [guide[2] for guide in guides]
    mainguides = root_list

    bind_joints = wing.limb_bind_joints
    end_joint = bind_joints[2]
    swing_transform = create_swing_transform(
        name=f"{end_joint}_Swing", parent=wing.spline_grp, driver=end_joint
    )
    swing_mapping: dict[str, str] = {end_joint: swing_transform}

    root_guide_curve = f"{prefix}_Root_Curve"
    start_guide_curve = f"{prefix}_Start_Curve"
    mid_guide_curve = f"{prefix}_Mid_Curve"
    end_guide_curve = f"{prefix}_End_Curve"

    # Feathershaping
    root_spline = Spline(
        guides=root_guide_curve,
        name=f"{prefix}_Root_Spline",
        parent=wing.spline_grp,
        create_controls=False,
        create_pins=True,
        ctrl_scale=wing.ctrl_scale,
    )
    mid_spline = Spline(
        guides=mid_guide_curve,
        name=f"{prefix}_Mid_Spline",
        parent=wing.net_grp,
        control_parent=wing.feather_grp,
        ctrl_scale=wing.ctrl_scale,
    )
    tip_spline = Spline(
        guides=end_guide_curve,
        name=f"{prefix}_Tip_Spline",
        parent=wing.net_grp,
        control_parent=wing.feather_grp,
        ctrl_scale=wing.ctrl_scale,
    )
    wing.root_spline = root_spline
    wing.mid_spline = mid_spline
    wing.tip_spline = tip_spline

    # Build Feather :)
    def_jnts = []
    last_index = 3
    for index, (root_guide, mid_guide, tip_guide) in enumerate(
        zip(root_list, mid_list, aim_list), start=1
    ):
        name = root_guide.replace("guide", "Spline")

        joint_parent_segment = wing.spline_grp
        if mc.attributeQuery("parent_joint", node=root_guide, exists=True):
            joint_parent_index = mc.getAttr(f"{root_guide}.parent_joint")
            joint_parent_segment = wing.limb_bind_joints[joint_parent_index]

        root_pin = create_pin_on_curve(
            name=f"{root_guide}_Pin",
            curve=root_spline.spline,
            parent=wing.spline_grp,
            guide=root_guide,
            arc_length=keep_spacing,
        )
        # start_pin = create_pin_on_curve(
        #     name=f"{root_guide}_Start_Pin",
        #     curve=start_spline.spline,
        #     parent=self.spline_grp,
        #     guide=root_guide,
        #     arc_length=keep_spacing,
        # )
        mid_pin = create_pin_on_curve(
            name=f"{mid_guide}_Pin",
            curve=mid_spline.spline,
            parent=wing.spline_grp,
            guide=mid_guide,
            arc_length=keep_spacing,
        )
        tip_pin = create_pin_on_curve(
            name=f"{tip_guide}_Pin",
            curve=tip_spline.spline,
            parent=wing.spline_grp,
            guide=tip_guide,
            arc_length=keep_spacing,
        )
        feather_spline = Spline(
            name=name,
            guides=[root_pin.pin, mid_pin.pin, tip_pin.pin],
            parent=wing.spline_grp,
            control_parent=wing.feather_grp,
            create_controls=False,
            pin_transforms=[root_pin.pin, mid_pin.pin, tip_pin.pin],
            degree=2,
        )
        mc.parent(feather_spline.spline, wing.net_grp)

        orient_driver_base = swing_mapping.get(joint_parent_segment, joint_parent_segment)
        orient_driver = get_closest_split_joint(root_pin.pin, orient_driver_base)
        root_swing_pin = create_swing_pin_on_curve(
            name=f"{root_guide}_Swing_Pin",
            curve=feather_spline.spline,
            parent=wing.spline_grp,
            guide=root_pin.pin,
            orient_guide=root_guide,
            orient_driver=orient_driver,
            arc_length=keep_spacing,
        )

        guide_parent = mc.listRelatives(root_guide, parent=True)[0]
        mid_guides = create_mid_guides(
            root_pin.pin, tip_pin.pin, 2, f"{prefix}_{feather}_mid_guide_", parent=guide_parent
        )

        guide_mapping = {
            root_pin.pin: f"{prefix}{feather}_{index:02d}_base_JNT",
            mid_guides[0]: f"{prefix}{feather}_{index:02d}_mid1_JNT",
            mid_guides[1]: f"{prefix}{feather}_{index:02d}_mid2_JNT",
            tip_guide: f"{prefix}{feather}_{index:02d}_ee_JNT",
        }
        split_joints: list[str] = []
        joint_parent: str | None = None
        for guide in [root_pin.pin] + mid_guides + [tip_guide]:
            if guide in guide_mapping:
                joint_name = guide_mapping[guide]
            else:
                joint_name = f"{guide}_JNT"
            joint = mc.joint(name=joint_name)
            UEface.add_to_face_bind_set(joint)
            split_joints.append(joint)
            pin = create_pin_on_net(
                name=f"{joint}_Pin",
                curve=feather_spline.spline,
                backbone_pins=[mid_pin, mid_pin, tip_pin],
                root_pin=root_swing_pin,
                guide=guide,
                parent=wing.spline_grp,
            )
            if joint_parent is None:
                joint_parent = get_closest_split_joint(pin, joint_parent_segment)
            mc.parent(joint, joint_parent, relative=True)
            matrix_constraint(pin, joint, keep_offset=False)
            joint_parent = joint

        split_joint = split_joints[0]
        mc.addAttr(split_joint, longName="split_joints", dataType="string")
        mc.setAttr(f"{split_joint}.split_joints", repr(split_joints), type="string")
