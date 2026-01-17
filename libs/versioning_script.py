import maya.cmds as mc

rig_name = "{{RIG_NAME}}"
rig_version = "{{RIG_VERSION}}"
script_node_name = "{{SCRIPT_NODE}}"
metadata_node_name = f"{rig_name}_RIG_METADATA"

def get_namespace() -> str:
    script_nodes = mc.ls(type="script", recursive=True)
    for script_node in script_nodes:
        if script_node_name in script_node:
            if ":" in script_node:
                return f"{script_node.rsplit(':', 1)[0]}:"
            else:
                return ""
    return ""

def version_attribute(attribute: str, value: float):
    if mc.objExists(attribute):
        mc.setAttr(attribute, value)
        print(f"Versioning: Setting {attribute} to {value}")
    else:
        print(f"Versioning: Couldn't find {attribute}")

def update_version(version: float):
    mc.setAttr(f"{metadata_node_name}.RIG_VERSION", version)
    print(f"Versioning: {rig_name} updated to version {version}")

if not mc.objExists(metadata_node_name):
    metadata_node = mc.createNode("network", name=metadata_node_name)
    mc.addAttr(metadata_node, longName="RIG_NAME", dataType="string")
    mc.setAttr(f"{metadata_node}.RIG_NAME", rig_name, type="string")
    mc.addAttr(metadata_node, longName="ORIGINAL_RIG_VERSION", attributeType="double")
    mc.setAttr(f"{metadata_node}.ORIGINAL_RIG_VERSION", float(rig_version))
    mc.addAttr(metadata_node, longName="RIG_VERSION", attributeType="double")
    mc.setAttr(f"{metadata_node}.RIG_VERSION", float(rig_version))
else:
    rig_version= float(rig_version)
    scene_rig_version = mc.getAttr(f"{metadata_node_name}.RIG_VERSION")
    if rig_version > scene_rig_version:
        rig_namespace: str = get_namespace()
        print(f"{rig_name} is now newer than the version first referenced into this file.")
        if scene_rig_version < 3:
            # Auto Clavicle
            for control in ["clavicle_L_CTRL", "clavicle_R_CTRL"]:
                version_attribute(f"{rig_namespace}{control}.autoClavicle", 0)
            update_version(3)
        if scene_rig_version < 4:
            # Shoulder Twist Distribute
            for control in ["arm_L_RIG_GRP", "arm_R_RIG_GRP"]:
                version_attribute(f"{rig_namespace}{control}.shoulderTwistDistribute", 0)
            update_version(4)
        if scene_rig_version < 5:
            # Hip Twist Distribute
            for control in ["leg_L_RIG_GRP", "leg_R_RIG_GRP"]:
                version_attribute(f"{rig_namespace}{control}.hipTwistDistribute", 0)
            update_version(5)
        if scene_rig_version < 6:
            # Gretchen Chest Offset
            if rig_name == "Gretchen":
                for control in ["Chest_Offset_CTRL"]:
                    version_attribute(f"{rig_namespace}{control}.CounterTwist_mult", 0)
            update_version(6)
        if scene_rig_version < 7:
            if rig_name == "Luciana":
                for control in ["tail_M_IK_Blend_GRP"]:
                    version_attribute(f"{rig_namespace}{control}.switch", 1)
