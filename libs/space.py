import maya.cmds as mc
from importlib import reload

import rjg.libs.attribute as rAttr

def space_switch(node: str, driver: str, target_list: list[str]=[], name_list: list[str]=[], name: str='space', constraint_type: str ='parent', value: int = 0) -> rAttr.Attribute:
    node_split = node.split('_')
    base_name = node_split[0] + '_' + node_split[1]

    if constraint_type != 'parent':
        parent = base_name + '_MODULE'
        if mc.objExists(parent):
            loc_grp = mc.group(empty=True, parent=base_name + '_MODULE', name=base_name + '_' + name + '_GRP')
        else:
            parent = mc.listRelatives(node, parent=True)
            loc_grp = mc.group(empty=True, name=node + '_' + name + '_GRP')
            if parent:
                mc.parent(loc_grp, parent, relative=True) 
        
        targets = []
        for target in target_list:
            target_name = base_name + '_' + target + '_LOC'
            if mc.objExists(target_name):
                targets.append(target_name)
            else:
                loc = mc.spaceLocator(name=target_name)[0]
                mc.matchTransform(loc, node)
                mc.parent(loc, loc_grp)
                mc.parentConstraint(target, loc, mo=True)
                targets.append(loc)
        if not mc.listRelatives(loc_grp):
            mc.delete(loc_grp)
    else:
        targets = target_list
    
    if constraint_type == 'parent':
        cnst = mc.parentConstraint(targets, node, mo=True)[0]
        wal = mc.parentConstraint(cnst, q=True, wal=True)
        cnst_targets = mc.parentConstraint(cnst, q=True, tl=True)
    elif constraint_type == 'point':
        cnst = mc.pointConstraint(targets, node, mo=True)[0]
        wal = mc.pointConstraint(cnst, q=True, wal=True)
        cnst_targets = mc.pointConstraint(cnst, q=True, tl=True)
    elif constraint_type == 'orient':
        cnst = mc.orientConstraint(targets, node, mo=True)[0]
        wal = mc.orientConstraint(cnst, q=True, wal=True)
        cnst_targets = mc.orientConstraint(cnst, q=True, tl=True)
    else:
        mc.error("constraint_type only supports ['parent', 'point', 'orient']")
        
    target_weight_map = {target: weight_alias for target, weight_alias in zip(cnst_targets, wal)}
    wal_ordered = [target_weight_map[target] for target in targets]

    space = rAttr.Attribute(node=driver, type='enum', value=value, enum_list=name_list, keyable=True, name=name)
    print("making space:", name)
    for i in range(len(targets)):
        if i > 0:
            mc.setDrivenKeyframe(cnst + '.' + wal_ordered[i-1], currentDriver=space.attr, driverValue=i, value=0)
        mc.setDrivenKeyframe(cnst + '.' + wal_ordered[i], currentDriver=space.attr, driverValue=i, value=1)
        if i <= len(target_list) - 2:
            mc.setDrivenKeyframe(cnst + '.' + wal_ordered[i+1], currentDriver=space.attr, driverValue=i, value=0)

    return space
