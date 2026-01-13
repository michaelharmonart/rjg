import maya.cmds as mc
from collections.abc import Collection
from maya.api.OpenMaya import MPoint

def get_world_position(obj):
    return mc.xform(obj, q=True, ws=True, t=True)

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

def parent_to_closest_joint(joint: str, parent_joints: list[str]):
    closest = get_closest_transform(joint, parent_joints)
    if closest:
        mc.parent(joint, closest, relative=True)
    return closest
    
for jnt in ['Wing_LMainFeather_01_base_JNT', 'Wing_LMainFeather_02_base_JNT', 'Wing_LMainFeather_05_base_JNT', 'Wing_LMainFeather_09_base_JNT']:    
    parent_to_closest_joint(jnt, ['arm_L_01_JNT', 'arm_L_02_JNT','arm_L_03_JNT','arm_L_04_JNT',])