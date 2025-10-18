import maya.cmds as mc

def build_switch_relative(control='honeybear_01', parent01='crate_Main', parent02='crate_Root', ctrlname='_CTRL', grpname='_GRP'):
    grp_node = f'{control}{grpname}'
    ctrl_short_name = f'{control}{ctrlname}'
    
    # Find full path of the control inside the group
    children = mc.listRelatives(grp_node, allDescendents=True, fullPath=True) or []
    ctrl_node = None
    for child in children:
        if child.endswith(ctrl_short_name):
            ctrl_node = child
            break
    
    if not ctrl_node:
        print(f"Control '{ctrl_short_name}' not found under group '{grp_node}'")
        return
    
    print(f"Using control node: {ctrl_node}")
    
    # Create parent constraints (assuming parents have ctrlname suffix)
    parent01_ctrl = f"{parent01}{ctrlname}"
    parent02_ctrl = f"{parent02}{ctrlname}"
    
    # Check parents exist
    if not mc.objExists(parent01_ctrl):
        print(f"Parent control '{parent01_ctrl}' not found.")
        return
    if not mc.objExists(parent02_ctrl):
        print(f"Parent control '{parent02_ctrl}' not found.")
        return
    
    # Create constraints on the group
    mc.parentConstraint(parent01_ctrl, grp_node, mo=True, weight=1)
    mc.parentConstraint(parent02_ctrl, grp_node, mo=True, weight=1)
    
    # Add SpaceSwitch attribute if it doesn't exist
    if not mc.attributeQuery("SpaceSwitch", node=ctrl_node, exists=True):
        mc.addAttr(ctrl_node, ln="SpaceSwitch", at='bool', keyable=True)
        mc.setAttr(f"{ctrl_node}.SpaceSwitch", e=True, keyable=True)
    
    # Find the parentConstraint node on the group
    constraints = mc.listRelatives(grp_node, type='parentConstraint') or []
    if not constraints:
        print(f"No parent constraints found on group '{grp_node}'")
        return
    parent_constraint_node = constraints[0]
    print(f"Using parent constraint node: {parent_constraint_node}")
    
    # Connect SpaceSwitch to parent constraint weights
    mc.connectAttr(f"{ctrl_node}.SpaceSwitch", f"{parent_constraint_node}.{parent01_ctrl}W0", force=True)
    
    # Create reverse node and connect
    reverse_node = mc.shadingNode('reverse', asUtility=True)
    mc.connectAttr(f"{ctrl_node}.SpaceSwitch", f"{reverse_node}.inputX", force=True)
    mc.connectAttr(f"{reverse_node}.outputX", f"{parent_constraint_node}.{parent02_ctrl}W1", force=True)
    
    print("Space switch setup complete.")


# Example call:
build_switch_relative(control='honeybear_02', parent01='crate_Main', parent02='crate_Root')
