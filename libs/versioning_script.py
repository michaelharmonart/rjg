import maya.cmds as mc

rig_name = "{{RIG_NAME}}"
rig_version = "{{RIG_VERSION}}"
script_node_name = "{{SCRIPT_NODE}}"
metadata_node_name = f"{rig_name}_RIG_METADATA"
def get_namespace() -> str:
    script_nodes = mc.ls(type="script", recursive=True)
    for script_node in script_nodes:
        if script_node_name in script_node:
            return script_node.rsplit(":", 1)[0]

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