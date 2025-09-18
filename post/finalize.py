import maya.cmds as mc
from importlib import reload

import rjg.libs.control.ctrl as rCtrl
import rjg.libs.attribute as rAttr
import rjg.libs.space as rSpace
reload(rCtrl)
reload(rAttr)
reload(rSpace)

def get_ctrl_types():
    type_dict = {}

    if mc.objExists('face_M'):
        type_dict['face'] = ['face_M']

    for x in mc.ls('*.ctrlDict'):
        ctrl = rCtrl.Control(ctrl=x.split('.')[0])
        if ctrl.fail:
            continue
        if ctrl.rig_type in type_dict:
            type_dict[ctrl.rig_type].append(ctrl.ctrl)
        else:
            type_dict[ctrl.rig_type] = [ctrl.ctrl]

    return type_dict

def get_ctrl_sides():
    side_dict = {}
    for x in mc.ls('*.ctrlDict'):
        ctrl = rCtrl.Control(ctrl=x.split('.')[0])
        if ctrl.fail:
            continue
        side = ctrl.side
        if not side:
            n = ctrl.ctrl.split('_')[1]
            if n == 'M':
                side = 'M'
            elif n == 'L':
                side = 'L'
            elif n == 'R':
                side = 'R'
            elif n == 'P':
                side = 'P'
            else:
                side = 'Unknown'
                continue
        if side in side_dict:
            side_dict[side].append(ctrl.ctrl)
        else:
            side_dict[side] = [ctrl.ctrl]

    return side_dict

def add_color_attrs(x, y, z, utScale):
    attr_util = rAttr.Attribute(add=False)
    type_dict = get_ctrl_types()
    side_dict = get_ctrl_sides()

    if mc.objExists("global_M_CTRL"):
        par = "global_M_CTRL"
    else:
        par = "RIG"

    c_ctrl = rCtrl.Control(
        parent=par,
        shape="rgb_circles",
        side=None,
        name="color",
        suffix="CTRL",
        axis="y",
        group_type=None,
        rig_type="global",
        ctrl_scale=utScale,
        translate=(x, y, z),
    )
    attr_util.lock_and_hide(node=c_ctrl.ctrl)
    c_ctrl.tag_as_controller()

    c_shapes = mc.listRelatives(c_ctrl.ctrl, shapes=True)
    for shape in c_shapes:
        mc.setAttr(shape + ".overrideEnabled", 1)
        mc.setAttr(shape + ".overrideRGBColors", 1)
    mc.setAttr(c_shapes[1] + ".overrideColorRGB", 1, 0, 0)
    mc.setAttr(c_shapes[2] + ".overrideColorRGB", 0, 1, 0)
    mc.setAttr(c_shapes[0] + ".overrideColorRGB", 0, 0, 1)

    ctrl_side_color_map = {}
    for side, ctrl_list in side_dict.items():
        # TODO: longName
        if side == "M":
            name = "middle"
        elif side == "L":
            name = "left"
        elif side == "R":
            name = "right"
        elif side == "P":
            name = "prop"
        elif side == "F":
            name = "floatBone"
        else:
            name = "unknown"
            print(side, ctrl_list)
            continue
        rAttr.Attribute(node=c_ctrl.ctrl, type="separator", name=name)
        color = rAttr.Attribute(
            node=c_ctrl.ctrl,
            type="double3",
            value=0,
            keyable=False,
            min=0,
            max=1,
            name=name + "Color",
            children_name="RGB",
        )
        mc.setAttr(color.attr, cb=True)
        for ctrl in ctrl_list:
            ctrl_side_color_map[ctrl] = color.attr

    rAttr.Attribute(node=c_ctrl.ctrl, type="separator", name="___")
    for type, ctrl_list in type_dict.items():
        rAttr.Attribute(node=c_ctrl.ctrl, type="separator", name=type)
        color = rAttr.Attribute(
            node=c_ctrl.ctrl,
            type="double3",
            value=0,
            keyable=False,
            min=0,
            max=1,
            name=type + "Color",
            children_name="RGB",
        )
        mc.setAttr(color.attr, cb=True)
        for ctrl in ctrl_list:
            try:
                for shape in mc.listRelatives(ctrl, shapes=True, type="nurbsCurve"):
                    mc.setAttr(shape + ".overrideEnabled", 1)
                    mc.setAttr(shape + ".overrideRGBColors", 1)
                    if type == "primary":
                        mc.connectAttr(ctrl_side_color_map[ctrl], shape + ".overrideColorRGB")
                    else:
                        mc.connectAttr(color.attr, shape + ".overrideColorRGB")
            except Exception as e:
                print(e)

    set_color_defaults(c_ctrl.ctrl)
    mc.setAttr("color_CTRL.v", cb=False)
    return c_ctrl.ctrl

def set_color_defaults(ctrl):
    color_dict = {
        'gimbal'    : (0.00, 0.45, 0.00),
        'root'      : (0.00, 0.30, 0.50),
        'global'    : (0.00, 0.50, 0.50),
        'cog'       : (0.00, 0.30, 0.50),
        'pivot'     : (1.00, 0.25, 0.00),
        'primary'   : (0.882, 0.683, 0.081),
        'bendy'     : (1.00, 0.20, 0.40),
        'tangent'   : (0.85, 0.15, 0.00),
        'offset'    : (0.75, 0.00, 0.00),
        'pv'        : (0.00, 1.00, 1.00),
        'fk'        : (0.00, 0.00, 1.00),
        'secondary' : (1.00, 0.20, 0.20),
        # 'l_eye'     : (0.10, 0.10, 0.70),
        # 'r_eye'     : (0.70, 0.10, 0.10),
        # 'c_eye'     : (0.70, 0.70, 0.10),
        'middle'    : (0.882, 0.683, 0.081),
        'left'      : (0.027, 0.191, 0.846),
        'right'     : (0.953, 0.000, 0.181),
        'prop'      : (1.00, 1.00, 1.00),
    }

    for type, value in color_dict.items():
        color = '{}.{}Color'.format(ctrl, type)
        if mc.objExists(color):
            mc.setAttr(color , *value)

def add_display_type(node, value, name, target):
    dt = rAttr.Attribute(node=node, type='enum', value=value, enum_list=['Normal', 'Template', 'Reference'], keyable=False, name=name)
    mc.setAttr(dt.attr, cb=True)
    mc.setAttr(target + '.overrideEnabled', 1)
    mc.connectAttr(dt.attr, target + '.overrideDisplayType')


def add_vis_ctrl(x, y, z, utScale):
    attr_util = rAttr.Attribute(add=False)
    type_dict = get_ctrl_types()

    if mc.objExists('global_M_CTRL'):
        par = 'global_M_CTRL'
    else:
        par = 'RIG'

    vis_ctrl = rCtrl.Control(parent=par, shape='eye', side=None, name='vis', suffix='CTRL', axis='y', group_type=None, rig_type='global', ctrl_scale=utScale, translate=(x, y, z))
    attr_util.lock_and_hide(node=vis_ctrl.ctrl)
    vis_ctrl.tag_as_controller()

    v_shapes = mc.listRelatives(vis_ctrl.ctrl, shapes=True)
    for shape in v_shapes:
        mc.setAttr(shape + '.overrideEnabled', 1)
        mc.setAttr(shape + '.overrideRGBColors', 1)
    mc.setAttr(v_shapes[0] + '.overrideColorRGB', .35, 0.271, .075)
    mc.setAttr(v_shapes[1] + '.overrideColorRGB', .05, 0.05, .05)

    model_vis = rAttr.Attribute(node=vis_ctrl.ctrl, type='bool', value=1, keyable=False, name='modelVis')
    skel_vis = rAttr.Attribute(node=vis_ctrl.ctrl, type='bool', value=0, keyable=False, name='skelVis')
    rig_vis = rAttr.Attribute(node=vis_ctrl.ctrl, type='bool', value=0, keyable=False, name='rigVis')
    mc.setAttr(model_vis.attr, cb=True)
    mc.setAttr(skel_vis.attr, cb=True)
    mc.setAttr(rig_vis.attr, cb=True)
    rAttr.Attribute(node=vis_ctrl.ctrl, type='separator', value=0, name='displayType')

    mc.connectAttr(model_vis.attr, 'MODEL.visibility')
    mc.connectAttr(skel_vis.attr, 'SKEL.visibility')

    add_display_type(node=vis_ctrl.ctrl, value=2, name='modelDisplay', target='MODEL')
    add_display_type(node=vis_ctrl.ctrl, value=2, name='skelDisplay', target='SKEL')

    for module in mc.ls('*_MODULE'):
        mc.connectAttr(rig_vis.attr, module + '.visibility')

    part_dict = {}
    for c in mc.ls('*_CONTROL'):
        part = c.split('_')[1]
        side = c.split('_')[0]

        if '_F_' in c:
            items = c.split('_')
            pos = items.index('F')
            part = 'F'
            side = '_'.join(items[:pos])

        if part in part_dict:
            part_dict[part].append(side)
        else:
            part_dict[part] = [side]

    for part, sides in part_dict.items():
        try:
            p_vis = rAttr.Attribute(node=vis_ctrl.ctrl, type='bool', value=1, keyable=False, name=part + '_Vis')
        except Exception as e:
            pass
        mc.setAttr(p_vis.attr, cb=True)
        for side in sides:
            try:
                mc.connectAttr(p_vis.attr, '{}_{}_CONTROL.visibility'.format(side, part))
            except Exception as e:
                mc.warning(e)
    try:
        mc.setAttr(vis_ctrl.ctrl + '.F_Vis', 0)
    except:
        pass    
        
    rAttr.Attribute(node=vis_ctrl.ctrl, type='separator', value=0, name='controlType')
    

    for type, ctrl_list in type_dict.items():

        if type in ['global', 'primary', 'fk', 'root', 'pv', 'tangent', 'bendy', 'pivot', 'face']:
            val = 1
        elif type in ['offset', 'gimbal', 'secondary']:
            val = 0
        else:
            val = 1
        t_vis = rAttr.Attribute(node=vis_ctrl.ctrl, type='bool', value=val, keyable=False, name=type + '_Vis')
        mc.setAttr(t_vis.attr, cb=True)
        for ctrl in ctrl_list:
            try:
                shapes = mc.listRelatives(ctrl, shapes=True, path=True)
            except:
                continue
            if shapes:
                for shape in shapes:
                    if mc.nodeType(shape) == 'nurbsCurve':
                        mc.connectAttr(t_vis.attr, shape + '.visibility')
        try:
            mc.connectAttr(vis_ctrl.ctrl + '.face_Vis', 'face_M.visibility')
        except:
            print('no face')

    mc.setAttr('vis_CTRL.v', cb=False)
    return vis_ctrl.ctrl
        


def assemble_skeleton():
    for part in mc.listRelatives('RIG'):
        if mc.objExists(part + '.skeletonPlugs'):
            children = mc.listAttr(part + '.skeletonPlugs')
            for child in children[1:]:
                plug = part + '.' + child
                par = mc.getAttr(plug)
                if 'mc.' in par:
                    par = eval(par)
                    mc.setAttr(plug, par, type='string')
                if mc.objExists(par):
                    mc.parent(child, par)
                else:
                   mc.warning(par + ' does not exist. Skipping...')

def add_global_scale(global_ctrl='global_M_CTRL'):
    gs_list = mc.ls('*.globalScale')
    gs = rAttr.Attribute(node=global_ctrl, type='double', value=1, keyable=True, min=0.001, name='globalScale')
    mc.connectAttr(gs.attr, 'SKEL.sx')
    mc.connectAttr(gs.attr, 'SKEL.sy')
    mc.connectAttr(gs.attr, 'SKEL.sz')
    mc.connectAttr(gs.attr, global_ctrl + '.sx')
    mc.connectAttr(gs.attr, global_ctrl + '.sy')
    mc.connectAttr(gs.attr, global_ctrl + '.sz')
    gs.lock_and_hide(translate=False, rotate=False)
    
    for ps in gs_list:
        try:
            part = ps.split('.')[0]
            mc.connectAttr(gs.attr, ps)
            mc.connectAttr(ps, part + '.sx')
            mc.connectAttr(ps, part + '.sy')
            mc.connectAttr(ps, part + '.sz')
        except Exception as e:
            print(e)

def assemble_rig():
    for part in mc.listRelatives('RIG'):
        try:
            plug_types = ['hideRigPlugs', 'deleteRigPlugs']
            for pt in plug_types:
                if mc.objExists(part + '.' + pt):
                    driven_list = mc.listAttr(part + '.' + pt)[1:]
                    for driven in driven_list:
                        plug = part + '.' + driven
                        node_list = mc.getAttr(plug)
                        node_list = node_list.split(' ')
                        if pt == 'hideRigPlugs':
                            for node in node_list:
                                mc.hide(node)
                        elif pt == 'deleteRigPlugs':
                            #mc.delete(node for node in node_list)
                            for node in node_list:
                                mc.delete(node)
                        else:
                            mc.warning(pt + ' plug type not found. Skipping...')
        except Exception as e:
            print("exception on", part, "hide/delete:", e)

        try:
            plug_types = ['pacRigPlugs', 'pacPocRigPlugs', 'pocRigPlugs', 'orcRigPlugs']
            for pt in plug_types:
                if mc.objExists(part + '.' + pt):
                    driven_list = mc.listAttr(part + '.' + pt)[1:]
                    for driven in driven_list:
                        plug = part + '.' + driven
                        driver = mc.getAttr(plug)
                        if 'mc.' in driver:
                            driver = eval(driver)
                            mc.setAttr(plug, driver, type='string')
                        if mc.objExists(driver):
                            if pt == 'pacRigPlugs':
                                mc.parentConstraint(driver, driven, mo=True)
                            elif pt == 'pacPocRigPlugs':
                                mc.parentConstraint(driver, driven, skipRotate=['x', 'y', 'z'], mo=True)
                            elif pt == 'pocRigPlugs':
                                if '_point' in driven:
                                    driven = driven.replace('_point', '')
                                mc.pointConstraint(driver, driven, mo=True)
                            elif pt == 'orcRigPlugs':
                                if '_orient' in driven:
                                    driven = driven.replace('_orient', '')
                                mc.orientConstraint(driver, driven, mo=True)
                            else:
                                mc.warning(pt + ' plug type does not exist. Skipping...')
                        else:
                            mc.warning(driver + ' driver does not exist. Skipping...')
        except Exception as e:
            print("exception on", part, "constraints:", e)

        try:
            plug_types = ['parent', 'point', 'orient']
            for pt in plug_types:
                for ctrl in mc.ls(part + '*.ctrlDict'):
                    ctrl = ctrl.split('.')[0]
                    attr_name = '{}.{}_{}'.format(part, ctrl, pt)
                    if mc.objExists(attr_name):
                        name_list = mc.listAttr(attr_name)
                        driver = name_list[0].split('.')[0]
                        if mc.objExists(part + '.' + driver):
                            driver = rCtrl.Control(ctrl=driver.replace('_'+pt, ''))
                            target_list = []
                            for name in name_list[1:]:
                                plug = part + '.' + name
                                target = mc.getAttr(plug)
                                if 'mc.' in target:
                                    target = eval(target)
                                    mc.setAttr(plug, target, type='string')
                                target_list.append(target)
                            value = int(target_list[-1])
                            name_list = [name.replace(pt, '').lower() for name in name_list[1:-1]]
                            if all(mc.objExists(obj) for obj in target_list[:-1]):
                                if driver.fail:
                                    rSpace.space_switch(node=part, driver=driver.ctrl, target_list=target_list[:-1], name_list=name_list, name=pt + 'Space', constraint_type=pt, value=value)
                                else:
                                    rSpace.space_switch(node=driver.top, driver=driver.ctrl, target_list=target_list[:-1], name_list=name_list, name=pt + 'Space', constraint_type=pt, value=value)
                            else:
                                print(part, "MISSING SOMETHING")
        except Exception as e:
            print("exception on", part, "space:", e)

        try:
            if mc.objExists(part + '.transferAttributes'):
                driven_list = mc.listAttr(part + '.transferAttributes')[1:]
                for driven in driven_list:
                    if "spine" in driven:
                        spine_switch_transfer(driven)
                        continue
                    plug = part + '.' + driven
                    transfer_node = mc.getAttr(plug)
                    attr_list = mc.listAttr(driven, userDefined=True)
                    for attr in attr_list:
                        if attr != 'ctrlDict':
                            src_attr = rAttr.Attribute(add=False, node=driven, name=attr, transfer_to=transfer_node)
                            src_attr.transfer_attr()

                    
        except Exception as e:
            print("  exception on", part, "transfer:", e)

def spine_switch_transfer(driven):
    stretch = rAttr.Attribute(node='chest_M_01_CTRL', type='double', name='stretch', keyable=False)
    mc.connectAttr('spine_M_SW_REV.outputX', stretch.attr, force=True)
    mc.connectAttr(stretch.attr, driven + '.stretch', force=True)

def add_rig_sets(character=None):
    rig_set = mc.sets(name='rig_SET', empty=True)
    ctrl_set = mc.sets(name='control_SET', empty=True)
    cache_set = mc.sets(name='cache_SET', empty=True)
    prop_set = mc.sets(name='prop_SET', empty=True)
    unreal_set = mc.sets(name='unreal_SET', empty=True)
    mc.sets(ctrl_set, add=rig_set)
    mc.sets(cache_set, add=rig_set)
    mc.sets(prop_set, add=rig_set)
    mc.sets(unreal_set, add=rig_set)

    for part in mc.listRelatives('RIG'):
        part_set = mc.sets(mc.ls(part+'*_CTRL'), name=part + '_SET')
        mc.sets(part_set, add=ctrl_set)

    if not character:
        mc.sets('MODEL', add=cache_set)
    else:
        mc.sets(f'{character}_UBM', add=cache_set)
        mc.sets(f'{character}_EXTRAS', add=cache_set)

    try:
        mc.sets('PROP', add=prop_set)
    except:
        pass

    mc.sets('MODEL', add=unreal_set)
    try:
        mc.sets('root_root_JNT', add=unreal_set)
    except:
        mc.sets('root_M_JNT', add=unreal_set)


def add_switch_ctrl(x, y, z, utScale, quad=False, character=None):
    attr_util = rAttr.Attribute(add=False)

    if mc.objExists('global_M_CTRL'):
        par = 'global_M_CTRL'
    else:
        par = 'RIG'

    s_ctrl = rCtrl.Control(parent=par, shape='gear_3D', side=None, suffix='CTRL', name='switch', axis='y', group_type=None, rig_type='global', ctrl_scale=utScale, translate=(x, y, z))
    attr_util.lock_and_hide(node=s_ctrl.ctrl)
    s_ctrl.tag_as_controller()

    # TODO: add color

    for part in mc.listRelatives('RIG'):
        if mc.objExists(part + '.switchRigPlugs'):
            switch_name = mc.getAttr(part + '.ikFkSwitch')
            if 'arm' in part or 'hand' in part or 'finger' in part or 'spine' in part:
                default_val = 1
            else:
                default_val = 0
            if quad:
                default_val = 0
            if mc.objExists(s_ctrl.ctrl + '.' + switch_name):
                switch_attr = rAttr.Attribute(node=s_ctrl.ctrl, name=switch_name, add=False)
            else:
                switch_attr = rAttr.Attribute(node=s_ctrl.ctrl, type='double', value=default_val, keyable=True, min=0, max=1, name=switch_name)
            mc.connectAttr(switch_attr.attr, part + '.switch')

    if character == 'Rayden':
        rev = mc.createNode('reverse', n='cbow_rev')
        cbow_attr = rAttr.Attribute(node=s_ctrl.ctrl, type='double', value=0, keyable=True, min=0, max=1, name='staticCbowRiggedCbow')
        mc.connectAttr('switch_CTRL.staticCbowRiggedCbow', 'cbow_rev.inputX')

    mc.setAttr('switch_CTRL.v', cb=False)
    return s_ctrl.ctrl


def polish_rig():

    # auto clav
    for s in ['L', 'R']:
        cond = mc.createNode('condition')
        mc.setAttr(cond + ".operation", 3)

        defaultZ = -15.0
        restZ = rAttr.Attribute(node=f'clavicle_{s}_CTRL', type='double', value=defaultZ, name='restRotateZ', keyable=True)

        mc.connectAttr(f"arm_{s}_01_fk_CTRL.rotateZ", cond + ".colorIfTrueB")
        mc.connectAttr(f"arm_{s}_01_fk_CTRL.rotateZ", cond + ".firstTerm")
        mc.connectAttr(cond + ".outColorB", f"clavicle_{s}_CTRL_OFF_GRP.rotateZ")
        mc.connectAttr(restZ.attr, cond + '.secondTerm')
        mc.connectAttr(restZ.attr, cond + '.colorIfFalseB')

        



def final(vis_ctrl=True, color_ctrl=True, switch_ctrl=True, constrain_model=False, utX=0, utY=0, utZ=0, DutX=0, DutY=0, DutZ=0, utScale=1, quad=False, polish=False, character=None):
    if color_ctrl:
        c_ctrl = add_color_attrs(x=utX+DutX, y=utY+DutY, z=utZ+DutZ, utScale=utScale)
    if switch_ctrl:
        s_ctrl = add_switch_ctrl(x=utX, y=utY, z=utZ, utScale=utScale, quad=quad, character=character)
    if vis_ctrl:
        v_ctrl = add_vis_ctrl(x=utX-DutX, y=utY-DutY, z=utZ-DutZ, utScale=utScale)
    assemble_skeleton()
    assemble_rig()
    add_global_scale()
    add_rig_sets(character)
    if polish:
        polish_rig()

    try:
        mc.rename('neck_M_01_fk_CTRL', 'neck_01_FK_M_CTRL')
        mc.rename('neck_M_02_fk_CTRL', 'neck_02_FK_M_CTRL')
        mc.rename('neck_M_03_fk_CTRL', 'neck_03_FK_M_CTRL')
    except:
        print('neck rename didnt work')
        pass

    try:
        mc.delete('*xgm*')
        mc.delete('*:xgm*')
    except:
        pass

    if constrain_model:
        if mc.objExists('root_M_JNT'):
            mc.parentConstraint('root_M_JNT', 'MODEL', mo=True)
            mc.scaleConstraint('root_M_JNT', 'MODEL', mo=True)


