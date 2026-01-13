from importlib import reload

import maya.cmds as mc
import maya.mel as mel
import rjg.libs.control.ctrl as rCtrl
import rjg.libs.file as rFile
from rjg.libs.profile import add_profiler_tag

reload(rFile)
reload(rCtrl)

def get_shapes(transform: str) -> list[str]:
    return mc.listRelatives(transform, children=True, shapes=True)


def copy_mesh_connections(source_shape: str, driven_shape: str, force: bool = True):
    connections_to_copy = [
        (f"{source_shape}.inMesh", f"{driven_shape}.inMesh"),
        (
            f"{source_shape}.instObjGroups[0].objectGroups[0].objectGroupId",
            f"{driven_shape}.instObjGroups[0].objectGroups[0].objectGroupId",
        ),
        (
            f"{source_shape}.instObjGroups[0].objectGroups[0].objectGrpColor",
            f"{driven_shape}.instObjGroups[0].objectGroups[0].objectGrpColor",
        ),
    ]
    
    for source_attr, driven_attr in connections_to_copy:
        source_connections = mc.listConnections(
            source_attr, source=True, destination=False, plugs=True
        )
    
        if source_connections:
            mc.connectAttr(source_connections[0], driven_attr, force=force)
            
def drive_mesh(source_mesh: str, driven_mesh: str, force: bool = True, use_blendshape: bool = False):
    if use_blendshape:
        mc.blendShape(source_mesh, driven_mesh, name=f"{driven_mesh}Projection", w=[(0, 1.0)], foc=True)
    else:
        source_shapes = get_shapes(source_mesh)
        driven_shapes = get_shapes(driven_mesh)
        for source_shape, driven_shape in zip(source_shapes, driven_shapes):
            copy_mesh_connections(source_shape, driven_shape, force)


def project(body=None, char=None, f_model=None, f_rig=None, f_skel=None, extras=None, f_extras=None, rig_par='head_M_02_CTRL_CNST_GRP', tY=0, use_legacy: bool = False):
    before_nodes = set(mc.ls())
    # reparent face sections to main rig
    mc.group(em=True, name='HIDE_FACE')
    mc.parent('HIDE_FACE', char)
    mc.parent(f_model, 'HIDE_FACE')
    mc.parent(f_rig, 'RIG')
    mc.parent(f_skel, 'SKEL')

    # project the face rig as an always-on blendShape
    #mc.xform(f_skel, t=[0, tY, 0])
    mc.blendShape(f_model, body, name='main_blendshapes', w=[(0, 1.0)], foc=True)
    mc.group(em=True, name='HIDE_FACE_EXTRAS', parent='HIDE_FACE')

    if not use_legacy:
        extras_names = mc.listRelatives(f_extras, allDescendents=True, type="transform")
        extras_paths = mc.listRelatives(f_extras, allDescendents=True, type="transform", path=True)
        for extra, path in zip(extras_names, extras_paths):
            extra_rename = mc.rename(path, f"{extra}_clone")
            mc.parent(extra_rename, "HIDE_FACE_EXTRAS")
            drive_mesh(extra_rename, extra, use_blendshape=use_legacy)
    else:
        try:
            mc.select(f_extras, hi=True)
            f_ex_list = mc.ls(selection=True, type='transform')
            for f in f_ex_list[:0:-1]:
                try:
                    f = mc.rename(f, f[len(f_extras):]+'_clone')
                    mc.blendShape(f, f[:-6], name=f[:-6]+'Projection', w=[(0, 1.0)], foc=True)
                    mc.parent(f, "HIDE_FACE_EXTRAS")
                    mc.hyperShade(f, assign='standardSurface1')
                except Exception as e:
                    print(f, ':', e)
        except Exception as e:
            mc.warning('faceProject 42:', e)


    
    # duplcicate the face rig controls and constrain their root to rig_par
    f_rig_name = f_rig
    f_rig = mc.rename(f_rig, f_rig + '_clone')
    f_rig_clone = mc.duplicate(f_rig, renameChildren=True)
    f_rig_clone = mc.rename(f_rig_clone[0], f_rig_name)
    mc.select(f_rig, hierarchy=True)

    f_rig_sel = mc.ls(selection=True, type='transform') #+ mc.ls(selection=True, type='joint') + mc.ls(selection=True, type='follicle')
    mc.select(f_rig_clone, hierarchy=True)
    f_rig_clone_sel = mc.ls(selection=True, type='transform') #+ mc.ls(selection=True, type='joint') + mc.ls(selection=True, type='follicle')
    mc.parentConstraint(rig_par, f_rig_clone_sel[0], mo=True)

    

    # tag incoming controls
    new_controls = mc.listRelatives('face_M', ad=True, type='nurbsCurve')
    for nc in new_controls:
        ct = mc.listRelatives(nc, parent=True)[0]
        rCtrl.tag_as_controller(ct)

    # setup direct connections between the new and original rig and rename 
    for i in range(1, len(f_rig_sel)):
        try:
            mc.connectAttr(f_rig_clone_sel[i] + ".translate", f_rig_sel[i] + ".translate")
            mc.connectAttr(f_rig_clone_sel[i] + ".rotate", f_rig_sel[i] + ".rotate")
            mc.connectAttr(f_rig_clone_sel[i] + ".scale", f_rig_sel[i] + ".scale")

            og_suff = f_rig_sel[i].split('_')[-1]
            clone_suff = f_rig_clone_sel[i].split('_')[-1]
            mc.rename(f_rig_sel[i], f_rig_sel[i].replace(og_suff, og_suff + '_clone'))
            mc.rename(f_rig_clone_sel[i], f_rig_clone_sel[i].replace(clone_suff, clone_suff[:-1]))
        except Exception as e:
            print(e)
            continue

######################################

    if use_legacy:
        pass
    else:
        for side in ['L', 'R']:
            mc.connectAttr(f'{side}_InBrow_ctrl.Crease', f'{side}_InBrow_ctrl_clone.Crease')
            mc.connectAttr(f'{side}_InBrow_ctrl.Forehead_Crease', f'{side}_InBrow_ctrl_clone.Forehead_Crease')
            mc.addAttr(f'{side}_InBrow_ctrl', longName='Legacy_Rig_Switch', at='bool', k=True )
            mc.connectAttr(f'{side}_InBrow_ctrl.Legacy_Rig_Switch', f'{side}_InBrow_ctrl_clone.Legacy_Control')
            #mc.connectAttr(f'{side}_InBrow_ctrl.Legacy_Control', f'{side}_InBrow_ctrl_clone.Legacy_Control')
            mc.createNode('reverse', name=f'{side}_Brow_switch_REV')
            mc.connectAttr(f'{side}_InBrow_ctrl.Legacy_Rig_Switch', f'{side}_Brow_switch_REV.inputX')
            mc.connectAttr(f'{side}_Brow_switch_REV.outputX', f'Modern_{side}_grp.visibility')
            mc.connectAttr(f'{side}_InBrow_ctrl.Legacy_Rig_Switch', f'Legacy_{side}_grp.visibility')
            mc.connectAttr(f'{side}_Eye_mid_05_ctrl.Wrinkle', f'{side}_Eye_mid_05_ctrl_clone.Wrinkle')
            mc.connectAttr(f'{side}_Corner_Main_Mouth_ctrl.Crease', f'{side}_Corner_Main_Mouth_ctrl_clone.Crease')
            mc.addAttr(f'{side}_Eyebrow_main_ctrl', longName='Legacy_Rig_Switch', proxy=f'{side}_InBrow_ctrl.Legacy_Rig_Switch')

    



    try:
        mc.rename('squashAndStretch_clone|squashStretch_Wire', 'squashStretch_Wire_clone')
        mc.rename('squashAndStretch_clone|squashStretch_CRV', 'squashStretch_CRV_clone')
        mc.rename('squashAndStretch|squashStretch_CRV1', 'squashStretch_CRV')
    except Exception as e:
        print("couldn't rename sqst things:", e)

    try:
        mc.select('*_driver')
        drivers = mc.ls(selection=True)

        for d in drivers:
            try:
                mc.connectAttr(d + '.rotate', d + '1.rotate')
            except Exception as e:
                print(e)
                continue
    except Exception as e:
        print(e)

    try:
        mc.select('*Mouth_offset_clone')
        drivers = mc.ls(selection=True)

        for d in drivers:
            try:
                mc.disconnectAttr(d[:-6] + '.translate', d + '.translate')
                mc.disconnectAttr(d[:-6] + '.rotate', d + '.rotate')
                mc.disconnectAttr(d[:-6] + '.scale', d + '.scale') 
                mc.connectAttr(d + '.translateX', d[:-6] + '.translateX')
            except Exception as e:
                print(e)
                continue
    except Exception as e:
        print(e)

    try:
        for s in ['R', 'L']:
            for attr in ['Blink', 'Blink_Height', 'Blink_Influence', 'Eyelid_Follow', s + '_Iris_Scale']:
                mc.connectAttr(f'{s}_eyeCTRL.{attr}', f'{s}_eyeCTRL_clone.{attr}')
    except Exception as e:
        print(e)

    try:
        for s in ['R', 'L']:
            #for attr in [f'{s}_Eye_mid_05_ctrl.Wrinkle']:
            mc.connectAttr(f'{s}_Eye_mid_05_ctrl.Wrinkle', f'{s}_Eye_mid_05_ctrl_clone.Wrinkle')
            mc.connectAttr(f'{s}_InBrow_ctrl.Crease', f'{s}_InBrow_ctrl_clone.Crease')
            mc.connectAttr(f'{s}_Corner_Main_Mouth_ctrl.Crease', f'{s}_Corner_Main_Mouth_ctrl_clone.Crease')
            mc.connectAttr(f'{s}_InBrow_ctrl.Forehead_Crease', f'{s}_InBrow_ctrl_clone.Forehead_Crease')
            for control in ['Mouth_ctrl', 'Mouth_n_Teeth']:
                mc.connectAttr[f'{control}.{s}_Corner_Stretch', f'{control}_clone.{s}_Corner_Stretch']
                mc.connectAttr[f'{control}.{s}_Pucker', f'{control}_clone.{s}_Pucker']

    except Exception as e:
        print(e)

    try:
        for attr in ['LipInfluence', 'LipSquishValue']:
            mc.connectAttr(f'jaw_ctrl.{attr}', f'jaw_ctrl_clone.{attr}')
    except Exception as e:
        print(e)

    try:
        mc.connectAttr('NoseA_CTRL.Nostril_Blend', 'NoseA_CTRL_clone.Nostril_Blend')
    except Exception as e:
        print(e)

    try:
        for attr in ['L_Lip_Corner_Pinch', 'L_NLF_Crease', 'Pucker', 'R_Lip_Corner_Pinch', 'R_NLF_Crease']:
            mc.connectAttr(f'Mouth_Global_ctrl.{attr}', f'Mouth_Global_ctrl_clone.{attr}')
    except Exception as e:
        print(e)
    #Bobo Test
    try:
        for attr in ['TestShape']:
            mc.connectAttr(f'pCube1_ctrl.{attr}', f'pCube1_ctrl_clone.{attr}')
    except Exception as e:
        print(e)
    

    try:
        mc.select('*_parentConstraint1_clone')
        driver_pc = mc.ls(selection=True)

        for d in driver_pc:
            try:
                mc.connectAttr(d + '.constraintRotate', d[:-24] + '.rotate')
                mc.connectAttr(d + '.constraintTranslate', d[:-24] + '.translate')
            except Exception as e:
                print(e)
    except Exception as e:
        print(e)

    try:
        mc.select('*_eyelid_*_Pointer_driver_rotateX')
        driver_rx = mc.ls(selection=True)

        for d in driver_rx:
            try:
                mc.connectAttr(d + '.output', d[:-8] + '.rotateX')
            except Exception as e:
                print(e)
    except Exception as e:
        print(e)

    try:
        mc.select('*_eyelid_*_Pointer_driver_aimConstraint1_clone')
        driver_ac = mc.ls(selection = True)

        try:
            for d in driver_ac:
                mc.connectAttr(d + '.constraintRotateY', d[:-21])
        except Exception as e:
            print(e)
    except Exception as e:
        print(e)

    try:
        mc.select('*_eyelid_??_OFST')
        el_ofst = mc.ls(selection=True)

        for n in el_ofst:
            try:
                mc.disconnectAttr(n + '_parentConstraint1_clone.constraintRotate', n + '.rotate')
            except Exception as e:
                print('100:', e)
                continue
    except Exception as e:
        print('103:', e)


    #Bobo Fixes
    try:
        for attr in ['Elliptical', 'RigScale']:
            mc.connectAttr(f'Left_Master_ctrl.{attr}', f'Left_Master_ctrl_clone.{attr}')
    except:
        pass
    try:
        for attr in ['Elliptical', 'RigScale']:
            mc.connectAttr(f'Right_Master_ctrl.{attr}', f'Right_Master_ctrl_clone.{attr}')
    except:
        pass

    try:
        for side in ['L', 'R']:
            mc.connectAttr(f'{side}_NLF_ctrl.{side}_NLF_Crease', f'{side}_NLF_ctrl_clone.{side}_NLF_Crease')
    except Exception as e:
        print(e)

    mc.select(cl=True)
            
    mc.hide("HIDE_FACE")
    mc.hide(f_rig)
    after_nodes = set(mc.ls())
    added_nodes = after_nodes - before_nodes
    for node in added_nodes:
        if mc.nodeType(node) not in ["mesh"]:
            add_profiler_tag(node, "face")




####Gretchen Fixes
"""
connectAttr -f L_InBrow_ctrl.Crease L_InBrow_ctrl_clone.Crease;
// Result: Connected L_InBrow_ctrl.Crease to L_InBrow_ctrl_clone.Crease.
connectAttr -f L_InBrow_ctrl.Forehead_Crease L_InBrow_ctrl_clone.Forehead_Crease;
// Result: Connected L_InBrow_ctrl.Forehead_Crease to L_InBrow_ctrl_clone.Forehead_Crease.
connectAttr -f L_InBrow_ctrl.Legacy_Control L_InBrow_ctrl_clone.Legacy_Control;
// Result: Connected L_InBrow_ctrl.Legacy_Control to L_InBrow_ctrl_clone.Legacy_Control.
connectAttr -f L_InBrow_ctrl.Legacy_Control Legacy_L_grp.visibility;
// Result: Connected L_InBrow_ctrl.Legacy_Control to Legacy_L_grp.visibility.
shadingNode -asUtility reverse;
// Result: reverse3
select -r Modern_L_grp ;
connectAttr -f L_InBrow_ctrl.Legacy_Control reverse3.inputX;
// Result: Connected L_InBrow_ctrl.Legacy_Control to reverse3.input.inputX.
connectAttr -f reverse3.outputX Modern_L_grp.visibility;
// Result: Connected reverse3.output.outputX to Modern_L_grp.visibility.
connectAttr -f L_Eye_mid_05_ctrl.Wrinkle L_Eye_mid_05_ctrl_clone.Wrinkle;
// Result: Connected L_Eye_mid_05_ctrl.Wrinkle to L_Eye_mid_05_ctrl_clone.Wrinkle.
connectAttr -f L_Corner_Main_Mouth_ctrl.Crease L_Corner_Main_Mouth_ctrl_clone.Crease;
// Result: Connected L_Corner_Main_Mouth_ctrl.Crease to L_Corner_Main_Mouth_ctrl_clone.Crease.

import maya.cmds as mc
mc.addAttr('L_Eyebrow_main_ctrl', longName='Legacy_Switch', proxy='L_InBrow_ctrl.Legacy_Control')

"""
