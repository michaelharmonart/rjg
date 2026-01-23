import importlib.util
import maya.cmds as mc

spec = importlib.util.find_spec("rjg.libs.versioning_script")
script_string = ""
if spec and spec.origin:
    with open(file=spec.origin, mode="r") as script_file:
        script_string = script_file.read()


def create_versioning_script(rig_name: str, rig_version: float) -> str:
    script_node: str = mc.createNode("script", name=f"{rig_name}_VERSIONING")
    mc.setAttr(f"{script_node}.scriptType", 1)
    mc.setAttr(f"{script_node}.sourceType", 1)

    script = (
        script_string.replace("{{RIG_NAME}}", rig_name)
        .replace("{{RIG_VERSION}}", str(rig_version))
        .replace("{{SCRIPT_NODE}}", script_node)
    )

    mc.setAttr(f"{script_node}.before", script, type="string")