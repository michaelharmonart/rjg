
from unittest import expectedFailure
import maya.cmds as mc
from importlib import reload
import re

import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
import rjg.build.guide as rGuide
import rjg.libs.transform as rXform
from rjg.build.UEface import UEface
import rjg.build.rigModule as rModule
from rjg.libs.profile import auto_profiler_tag
reload(rAttr)
reload(rChain)
reload(rCtrl)
reload (rGuide)
reload(rXform)


class UEfaceconnect(UEface):
    def __init__(self, grp_name=None, ctrl_scale=1, custom=None, mastermouth=True):
        super().__init__(part='Brow', grp_name=grp_name, ctrl_scale=ctrl_scale)
        self.custom = custom
        self.mastermouth = mastermouth
    
    @auto_profiler_tag
    def build(self, character=None):
        rig_module = rModule.RigModule(side=None, part="UEFace")
        upper_jnt, upper_ctrl, upper_offset = UEface.Simple_joint_and_Control(
            guide='UpperHead_guide',
            overwrite=True,
            overwrite_name='UpperHead',
            orient=True,
            CTRL_Size=9,
            JNT_Size=0.9,
            CTRL_Color=(1, 0.6, 0)
        )
        lower_jnt, lower_ctrl, lower_offset = UEface.Simple_joint_and_Control(
            guide='LowerHead_guide',
            overwrite=True,
            overwrite_name='LowerHead',
            orient=True,
            CTRL_Size=9,
            JNT_Size=0.9,
            CTRL_Color=(1, 0.6, 0)
        )
        mc.parent(lower_jnt, upper_jnt, 'head_M_JNT')
        mc.parent(lower_offset, upper_offset, 'head_M_01_CTRL')

        # Select the set and get its members
        mc.select('UE_Face_Bind')
        all_joints = mc.ls(selection=True)
        if self.custom in ['Domingo', 'Luciana', 'Normal']:
            for joint_name in all_joints:
                rig_module.tag_bind_joints(joint_name)

        # Init groups
        nose_jnts = []
        upper_jnts = []
        lower_jnts = []
        unsorted = []

        # Names to ignore
        ignored_names = ['LowerHead_JNT', 'UpperHead_JNT']

        for jnt in all_joints:
            name = jnt

            # Skip ignored names
            if name in ignored_names:
                continue

            # Remove "Major" joints from set and skip
            if "Major" in name:
                mc.sets(name, remove='UE_Face_Bind')
                continue

            # Sort into categories
            if name == 'Nose_M_NoseRoot_JNT' or 'NLFold' in name:
                nose_jnts.append(name)
            elif any(key in name for key in ['Brow', 'Eye', 'CheekBone']):
                upper_jnts.append(name)
            elif any(key in name for key in ['Mouth', 'Jaw_M_root_JNT', 'Puff', 'uppermouth', 'lowermouth']):
                lower_jnts.append(name)
            else:
                unsorted.append(name)
        try:
            lower_jnts.append('uppermouth_JNT')
            lower_jnts.append('lowermouth_JNT')
        except:
            pass

        # Parent to respective head joints
        if mc.objExists('LowerHead_JNT'):
            for jnt in lower_jnts:
                mc.parent(jnt, 'LowerHead_JNT')

        if mc.objExists('UpperHead_JNT'):
            for jnt in upper_jnts:
                mc.parent(jnt, 'UpperHead_JNT')

        if mc.objExists('head_M_JNT'):
            for jnt in nose_jnts:
                mc.parent(jnt, 'head_M_JNT')

        # Print unsorted list
        print("🟡 Unsorted joints:")
        for j in unsorted:
            print(f" - {j}")

        if self.custom is None:
            mc.parent('Eye_L_look_offset', 'Eye_R_look_offset', 'UpperHead_M_CTRL')
            look_pos = mc.xform('LookNULL_loc', q=True, os=True, t=True)
            ctrl_name, top_group = UEface.build_basic_control( name='Look_M', shape='circle', size=2.0, color_rgb=(1, 1, 0), position=look_pos, rotation=(90, 0, 0))
            mc.parent('Eye_L_Look_L_CTRL_CNST_GRP', 'Eye_R_Look_R_CTRL_CNST_GRP', ctrl_name)
            mc.group('LookNULL_loc', 'Eye_L_eyelid_look_loc', 'Eye_R_eyelid_look_loc', name='Look_Null')
            mc.parent('Look_Null', 'head_M_01_CTRL')
            mc.hide('Look_Null')
            mc.parent('Look_M_M_CTRL_CNST_GRP', 'RIG')
            mc.parent('Brow_L_NULL', 'Brow_R_NULL', 'UpperHead_M_CTRL')
            mc.hide('Brow_L_NULL', 'Brow_R_NULL')
            mc.parent('Brow_L_Master_L_CTRL_CNST_GRP', 'Brow_R_Master_R_CTRL_CNST_GRP', 'Cheek_R_CheekBone_R_CTRL_CNST_GRP', 'Cheek_L_CheekBone_L_CTRL_CNST_GRP', 'UpperHead_M_CTRL' )
            mc.parentConstraint('Nose_M_NoseRoot_M_CTRL', 'Cheek_L_NLFold_02_Major_jnt', mo=True)
            mc.parentConstraint('Nose_M_NoseRoot_M_CTRL', 'Cheek_R_NLFold_02_Major_jnt', mo=True)
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'Cheek_L_NLFold_05_Major_jnt', mo=True)
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'Cheek_R_NLFold_05_Major_jnt', mo=True)
            mc.parent('Cheek_L_Puff_L_CTRL_CNST_GRP', 'Cheek_R_Puff_R_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('Nose_Master_Master_CTRL_CNST_GRP', 'head_M_01_CTRL')
            mc.parent('Jaw_M_root_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('TopTeeth_JNT', 'BotTeeth_JNT', 'Tongue_01_JNT', 'head_M_JNT')
            mc.parent('TopTeeth_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('BotTeeth_M_CTRL_CNST_GRP', 'Tongue_01_01_CTRL_CNST_GRP', 'Jaw_M_root_M_CTRL')
            if self.mastermouth:
                mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'TopTeeth_M_CTRL_CNST_GRP', mo=True)
                mc.addAttr('TopTeeth_M_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Topteeth_spaceswitch', proxy='TopTeeth_M_CTRL.MouthMaster_spaceswitch')
                mc.connectAttr('TopTeeth_M_CTRL.MouthMaster_spaceswitch', 'TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')

                #mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'BotTeeth_M_CTRL_CNST_GRP', mo=True)
                #mc.addAttr('BotTeeth_M_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                #mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Botteeth_spaceswitch', proxy='BotTeeth_M_CTRL.MouthMaster_spaceswitch')
                #mc.connectAttr('BotTeeth_M_CTRL.MouthMaster_spaceswitch', 'BotTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')

                #mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'Tongue_01_01_CTRL_CNST_GRP', mo=True)
                #mc.addAttr('Tongue_01_01_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                #mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Tongue_spaceswitch', proxy='Tongue_01_01_CTRL.MouthMaster_spaceswitch')
                #mc.connectAttr('Tongue_01_01_CTRL.MouthMaster_spaceswitch', 'Tongue_01_01_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')
                for control in ['Tongue_01_01_CTRL', 'BotTeeth_M_CTRL']:
                    mdnodetranslate = mc.createNode('multiplyDivide', name=f'{control}MDtranslate')
                    mdnoderotate = mc.createNode('multiplyDivide', name=f'{control}MDrotate')
                    mc.addAttr(control, longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0)
                    mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName=f'{control}_spaceswitch', proxy=f'{control}.MouthMaster_spaceswitch')
                    for axes in ['X', 'Y','Z']:
                        mc.connectAttr(f"Mouth_M_MasterControl_M_CTRL.translate{axes}", f'{mdnodetranslate}.input1{axes}')
                        mc.connectAttr(f"Mouth_M_MasterControl_M_CTRL.rotate{axes}", f'{mdnoderotate}.input1{axes}')
                        mc.connectAttr(f'{control}.MouthMaster_spaceswitch', f'{mdnodetranslate}.input2{axes}')
                        mc.connectAttr(f'{control}.MouthMaster_spaceswitch', f'{mdnoderotate}.input2{axes}')
                    #for control in ['Tongue_01_01_CTRL', 'BotTeeth_M_CTRL']: 
                    mc.connectAttr(f'{mdnodetranslate}.input1', f'{control}_OFF_GRP.translate')
                    mc.connectAttr(f'{mdnoderotate}.input1', f'{control}_OFF_GRP.rotate')
            
            if self.mastermouth:
                mc.parent('Mouth_M_MasterControl_M_CTRL_CNST_GRP', 'LowerLip_M_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            else:
                mc.parent('LowerLip_M_M_CTRL_CNST_GRP', 'UpperLip_M_M_CTRL_CNST_GRP', 'Major_Mouth_R_CornerLip_Mouth_CTRL_CNST_GRP', 'Major_Mouth_L_CornerLip_Mouth_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'LowerLip_M_M_CTRL_CNST_GRP', mo=True)
            pos = mc.xform('Mouth_M_center', q=True, ws=True, t=True)
            loc = mc.spaceLocator(name='Mouth_NULL_loc')[0]
            # Move it to the desired world position
            mc.xform(loc, worldSpace=True, translation=pos)
            for side in ['L', 'R']:
                mc.pointConstraint('LowerLip_M_M_CTRL_CNST_GRP', f'Major_Mouth_{side}_CornerLip_Mouth_CTRL_CNST_GRP', mo=True)
                mc.pointConstraint(loc, f'Major_Mouth_{side}_CornerLip_Mouth_CTRL_CNST_GRP', mo=True)
                mc.parentConstraint(f'Major_Mouth_{side}_CornerLip_Mouth_CTRL', f'NLFold_{side}_{side}_CTRL_CNST_GRP', mo=True)
                try:
                    mc.parent(f'Ear_{side}_Root_JNT', 'head_M_JNT')
                    mc.parent(f'Ear_{side}_Root_{side}_CTRL_CNST_GRP', 'head_M_01_CTRL')
                except:
                    print('no ear controls')
            if self.mastermouth:
                mc.parent(loc, 'Mouth_M_MasterControl_M_CTRL')
            else:
                mc.parent(loc, 'LowerHead_M_CTRL')

            mc.parent('Eye_L_JNT', 'Eye_R_JNT' ,'UpperHead_JNT' )
            mc.hide('Eye_L_Eyelid_InnerCorner_Major_JNT', 'Eye_L_Eyelid_Lower_Major_JNT', 'Eye_L_Eyelid_OuterCorner_Major_JNT', 'Eye_L_Eyelid_Upper_Major_JNT', 'Eye_R_Eyelid_Upper_Major_JNT', 'Eye_R_Eyelid_OuterCorner_Major_JNT', 'Eye_R_Eyelid_Lower_Major_JNT', 'Eye_R_Eyelid_InnerCorner_Major_JNT', 'Mouth_NULL_loc')
            mc.parentConstraint('LowerHead_M_CTRL', 'Jaw_M_larynx_M_CTRL_CNST_GRP', mo=True)
            mc.parentConstraint('neck_M_02_fk_CTRL', 'Jaw_M_larynx_M_CTRL_CNST_GRP', mo=True)
            mc.parent('Jaw_M_larynx_M_CTRL_CNST_GRP', 'RIG')
            
            # === 1️⃣ Define your control variables ===
            look_ctrl = "Look_M_M_CTRL"          # the one user animates
            look_offset = "Look_M_M_CTRL_SDK_GRP"  # the group above it
            head_ctrl = "head_M_01_CTRL"
            root_ctrl = "global_M_CTRL"

            # === 2️⃣ Add the space switch attribute ===
            attr_name = "spaceSwitch"

            if not mc.attributeQuery(attr_name, node=look_ctrl, exists=True):
                mc.addAttr(look_ctrl, longName=attr_name, attributeType="bool", keyable=True)

            # === 3️⃣ Create the parent constraint (Head + Root → Look Offset) ===
            constraint_name = mc.parentConstraint(head_ctrl, root_ctrl, look_offset, maintainOffset=True)[0]

            # === 4️⃣ Create the reverse node ===
            rev = mc.createNode("reverse", name=f"{look_ctrl}_spaceSwitch_REV")

            # === 5️⃣ Hook up the connections ===
            # Connect the look control's attr to head weight and reverse input
            mc.connectAttr(f"{look_ctrl}.{attr_name}", f"{rev}.inputX", f=True)
            mc.connectAttr(f"{look_ctrl}.{attr_name}", f"{constraint_name}.{head_ctrl}W0", f=True)

            # Connect reverse output to the root weight
            mc.connectAttr(f"{rev}.outputX", f"{constraint_name}.{root_ctrl}W1", f=True)
        
        if self.custom == 'Normal':
            mc.parent('Eye_L_look_offset', 'Eye_R_look_offset', 'UpperHead_M_CTRL')
            look_pos = mc.xform('LookNULL_loc', q=True, os=True, t=True)
            ctrl_name, top_group = UEface.build_basic_control( name='Look_M', shape='circle', size=2.0, color_rgb=(1, 1, 0), position=look_pos, rotation=(90, 0, 0))
            mc.parent('Eye_L_Look_L_CTRL_CNST_GRP', 'Eye_R_Look_R_CTRL_CNST_GRP', ctrl_name)
            mc.group('LookNULL_loc', 'Eye_L_eyelid_look_loc', 'Eye_R_eyelid_look_loc', name='Look_Null')
            mc.parent('Look_Null', 'head_M_01_CTRL')
            mc.hide('Look_Null')
            mc.parent('Look_M_M_CTRL_CNST_GRP', 'RIG')
            mc.parent('Brow_L_NULL', 'Brow_R_NULL', 'UpperHead_M_CTRL')
            mc.hide('Brow_L_NULL', 'Brow_R_NULL')
            mc.parent('Brow_L_Master_L_CTRL_CNST_GRP', 'Brow_R_Master_R_CTRL_CNST_GRP', 'Cheek_R_CheekBone_R_CTRL_CNST_GRP', 'Cheek_L_CheekBone_L_CTRL_CNST_GRP', 'UpperHead_M_CTRL' )
            mc.parentConstraint('Nose_M_NoseRoot_M_CTRL', 'Cheek_L_NLFold_02_Major_jnt', mo=True)
            mc.parentConstraint('Nose_M_NoseRoot_M_CTRL', 'Cheek_R_NLFold_02_Major_jnt', mo=True)
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'Cheek_L_NLFold_05_Major_jnt', mo=True)
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'Cheek_R_NLFold_05_Major_jnt', mo=True)
            mc.parent('Cheek_L_Puff_L_CTRL_CNST_GRP', 'Cheek_R_Puff_R_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('Nose_Master_Master_CTRL_CNST_GRP', 'head_M_01_CTRL')
            mc.parent('Jaw_M_root_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('TopTeeth_JNT', 'BotTeeth_JNT', 'Tongue_01_JNT', 'head_M_JNT')
            mc.parent('TopTeeth_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('BotTeeth_M_CTRL_CNST_GRP', 'Tongue_01_01_CTRL_CNST_GRP', 'Jaw_M_root_M_CTRL')
            if self.mastermouth:
                mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'TopTeeth_M_CTRL_CNST_GRP', mo=True)
                mc.addAttr('TopTeeth_M_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Topteeth_spaceswitch', proxy='TopTeeth_M_CTRL.MouthMaster_spaceswitch')
                mc.connectAttr('TopTeeth_M_CTRL.MouthMaster_spaceswitch', 'TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')

                #mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'BotTeeth_M_CTRL_CNST_GRP', mo=True)
                #mc.addAttr('BotTeeth_M_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                #mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Botteeth_spaceswitch', proxy='BotTeeth_M_CTRL.MouthMaster_spaceswitch')
                #mc.connectAttr('BotTeeth_M_CTRL.MouthMaster_spaceswitch', 'BotTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')

                #mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'Tongue_01_01_CTRL_CNST_GRP', mo=True)
                #mc.addAttr('Tongue_01_01_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                #mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Tongue_spaceswitch', proxy='Tongue_01_01_CTRL.MouthMaster_spaceswitch')
                #mc.connectAttr('Tongue_01_01_CTRL.MouthMaster_spaceswitch', 'Tongue_01_01_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')
                for control in ['Tongue_01_01_CTRL', 'BotTeeth_M_CTRL']:
                    mdnodetranslate = mc.createNode('multiplyDivide', name=f'{control}MDtranslate')
                    mdnoderotate = mc.createNode('multiplyDivide', name=f'{control}MDrotate')
                    mc.addAttr(control, longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0)
                    mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName=f'{control}_spaceswitch', proxy=f'{control}.MouthMaster_spaceswitch')
                    for axes in ['X', 'Y','Z']:
                        mc.connectAttr(f"Mouth_M_MasterControl_M_CTRL.translate{axes}", f'{mdnodetranslate}.input1{axes}')
                        mc.connectAttr(f"Mouth_M_MasterControl_M_CTRL.rotate{axes}", f'{mdnoderotate}.input1{axes}')
                        mc.connectAttr(f'{control}.MouthMaster_spaceswitch', f'{mdnodetranslate}.input2{axes}')
                        mc.connectAttr(f'{control}.MouthMaster_spaceswitch', f'{mdnoderotate}.input2{axes}')
                    #for control in ['Tongue_01_01_CTRL', 'BotTeeth_M_CTRL']: 
                    mc.connectAttr(f'{mdnodetranslate}.input1', f'{control}_OFF_GRP.translate')
                    mc.connectAttr(f'{mdnoderotate}.input1', f'{control}_OFF_GRP.rotate')
            if self.mastermouth:
                mc.parent('Mouth_M_MasterControl_M_CTRL_CNST_GRP', 'LowerLip_M_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            else:
                mc.parent('LowerLip_M_M_CTRL_CNST_GRP', 'UpperLip_M_M_CTRL_CNST_GRP', 'Major_Mouth_R_CornerLip_Mouth_CTRL_CNST_GRP', 'Major_Mouth_L_CornerLip_Mouth_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'LowerLip_M_M_CTRL_CNST_GRP', mo=True)
            pos = mc.xform('Mouth_M_center', q=True, ws=True, t=True)
            loc = mc.spaceLocator(name='Mouth_NULL_loc')[0]
            # Move it to the desired world position
            mc.xform(loc, worldSpace=True, translation=pos)
            for side in ['L', 'R']:
                mc.pointConstraint('LowerLip_M_M_CTRL_CNST_GRP', f'Major_Mouth_{side}_CornerLip_Mouth_CTRL_CNST_GRP', mo=True)
                mc.pointConstraint(loc, f'Major_Mouth_{side}_CornerLip_Mouth_CTRL_CNST_GRP', mo=True)
                mc.parentConstraint(f'Major_Mouth_{side}_CornerLip_Mouth_CTRL', f'NLFold_{side}_{side}_CTRL_CNST_GRP', mo=True)
                try:
                    mc.parent(f'Ear_{side}_Root_JNT', 'head_M_JNT')
                    mc.parent(f'Ear_{side}_Root_{side}_CTRL_CNST_GRP', 'head_M_01_CTRL')
                except:
                    print('no ear controls')
            if self.mastermouth:
                mc.parent(loc, 'Mouth_M_MasterControl_M_CTRL')
            else:
                mc.parent(loc, 'LowerHead_M_CTRL')

            mc.parent('Eye_L_JNT', 'Eye_R_JNT' ,'UpperHead_JNT' )
            mc.hide('Eye_L_Eyelid_InnerCorner_Major_JNT', 'Eye_L_Eyelid_Lower_Major_JNT', 'Eye_L_Eyelid_OuterCorner_Major_JNT', 'Eye_L_Eyelid_Upper_Major_JNT', 'Eye_R_Eyelid_Upper_Major_JNT', 'Eye_R_Eyelid_OuterCorner_Major_JNT', 'Eye_R_Eyelid_Lower_Major_JNT', 'Eye_R_Eyelid_InnerCorner_Major_JNT', 'Mouth_NULL_loc')
            mc.parentConstraint('LowerHead_M_CTRL', 'Jaw_M_larynx_M_CTRL_CNST_GRP', mo=True)
            mc.parentConstraint('neck_M_02_fk_CTRL', 'Jaw_M_larynx_M_CTRL_CNST_GRP', mo=True)
            mc.parent('Jaw_M_larynx_M_CTRL_CNST_GRP', 'RIG')
            
            # === 1️⃣ Define your control variables ===
            look_ctrl = "Look_M_M_CTRL"          # the one user animates
            look_offset = "Look_M_M_CTRL_SDK_GRP"  # the group above it
            head_ctrl = "head_M_01_CTRL"
            root_ctrl = "global_M_CTRL"

            # === 2️⃣ Add the space switch attribute ===
            attr_name = "spaceSwitch"

            if not mc.attributeQuery(attr_name, node=look_ctrl, exists=True):
                mc.addAttr(look_ctrl, longName=attr_name, attributeType="bool", keyable=True)

            # === 3️⃣ Create the parent constraint (Head + Root → Look Offset) ===
            constraint_name = mc.parentConstraint(head_ctrl, root_ctrl, look_offset, maintainOffset=True)[0]

            # === 4️⃣ Create the reverse node ===
            rev = mc.createNode("reverse", name=f"{look_ctrl}_spaceSwitch_REV")

            # === 5️⃣ Hook up the connections ===
            # Connect the look control's attr to head weight and reverse input
            mc.connectAttr(f"{look_ctrl}.{attr_name}", f"{rev}.inputX", f=True)
            mc.connectAttr(f"{look_ctrl}.{attr_name}", f"{constraint_name}.{head_ctrl}W0", f=True)

            # Connect reverse output to the root weight
            mc.connectAttr(f"{rev}.outputX", f"{constraint_name}.{root_ctrl}W1", f=True)


        elif self.custom == 'Domingo':
            mc.parent('Eye_L_look_offset', 'Eye_R_look_offset', 'UpperHead_M_CTRL')
            look_pos = mc.xform('LookNULL_loc', q=True, os=True, t=True)
            ctrl_name, top_group = UEface.build_basic_control( name='Look_M', shape='circle', size=2.0, color_rgb=(1, 1, 0), position=look_pos, rotation=(90, 0, 0))
            mc.parent('Eye_L_Look_L_CTRL_CNST_GRP', 'Eye_R_Look_R_CTRL_CNST_GRP', ctrl_name)
            mc.group('LookNULL_loc', 'Eye_L_eyelid_look_loc', 'Eye_R_eyelid_look_loc', name='Look_Null')
            mc.parent('Look_Null', 'head_M_01_CTRL')
            mc.hide('Look_Null')
            mc.parent('Look_M_M_CTRL_CNST_GRP', 'RIG')
            mc.parent('Brow_L_NULL', 'Brow_R_NULL', 'UpperHead_M_CTRL')
            mc.hide('Brow_L_NULL', 'Brow_R_NULL')
            mc.parent('Brow_L_Master_L_CTRL_CNST_GRP', 'Brow_R_Master_R_CTRL_CNST_GRP', 'Cheek_R_CheekBone_R_CTRL_CNST_GRP', 'Cheek_L_CheekBone_L_CTRL_CNST_GRP', 'UpperHead_M_CTRL' )
            mc.parent('Cheek_L_Puff_L_CTRL_CNST_GRP', 'Cheek_R_Puff_R_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('Jaw_M_root_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('TopTeeth_JNT', 'BotTeeth_JNT', 'Tongue_01_JNT', 'head_M_JNT')
            mc.parent('TopTeeth_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('BotTeeth_M_CTRL_CNST_GRP', 'Tongue_01_01_CTRL_CNST_GRP', 'Jaw_M_root_M_CTRL')
            if self.mastermouth:
                mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'TopTeeth_M_CTRL_CNST_GRP', mo=True)
                mc.addAttr('TopTeeth_M_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Topteeth_spaceswitch', proxy='TopTeeth_M_CTRL.MouthMaster_spaceswitch')
                mc.connectAttr('TopTeeth_M_CTRL.MouthMaster_spaceswitch', 'TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')

                #mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'BotTeeth_M_CTRL_CNST_GRP', mo=True)
                #mc.addAttr('BotTeeth_M_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                #mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Botteeth_spaceswitch', proxy='BotTeeth_M_CTRL.MouthMaster_spaceswitch')
                #mc.connectAttr('BotTeeth_M_CTRL.MouthMaster_spaceswitch', 'BotTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')

                #mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'Tongue_01_01_CTRL_CNST_GRP', mo=True)
                #mc.addAttr('Tongue_01_01_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                #mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Tongue_spaceswitch', proxy='Tongue_01_01_CTRL.MouthMaster_spaceswitch')
                #mc.connectAttr('Tongue_01_01_CTRL.MouthMaster_spaceswitch', 'Tongue_01_01_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')
                for control in ['Tongue_01_01_CTRL', 'BotTeeth_M_CTRL']:
                    mdnodetranslate = mc.createNode('multiplyDivide', name=f'{control}MDtranslate')
                    mdnoderotate = mc.createNode('multiplyDivide', name=f'{control}MDrotate')
                    mc.addAttr(control, longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0)
                    mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName=f'{control}_spaceswitch', proxy=f'{control}.MouthMaster_spaceswitch')
                    for axes in ['X', 'Y','Z']:
                        mc.connectAttr(f"Mouth_M_MasterControl_M_CTRL.translate{axes}", f'{mdnodetranslate}.input1{axes}')
                        mc.connectAttr(f"Mouth_M_MasterControl_M_CTRL.rotate{axes}", f'{mdnoderotate}.input1{axes}')
                        mc.connectAttr(f'{control}.MouthMaster_spaceswitch', f'{mdnodetranslate}.input2{axes}')
                        mc.connectAttr(f'{control}.MouthMaster_spaceswitch', f'{mdnoderotate}.input2{axes}')
                    #for control in ['Tongue_01_01_CTRL', 'BotTeeth_M_CTRL']: 
                    mc.connectAttr(f'{mdnodetranslate}.input1', f'{control}_OFF_GRP.translate')
                    mc.connectAttr(f'{mdnoderotate}.input1', f'{control}_OFF_GRP.rotate')
            #mc.parent('LowerLip_M_M_CTRL_CNST_GRP', 'UpperLip_M_M_CTRL_CNST_GRP', 'Major_Mouth_R_CornerLip_Mouth_CTRL_CNST_GRP', 'Major_Mouth_L_CornerLip_Mouth_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            if self.mastermouth:
                mc.parent('Mouth_M_MasterControl_M_CTRL_CNST_GRP', 'LowerLip_M_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            else:
                mc.parent('LowerLip_M_M_CTRL_CNST_GRP', 'UpperLip_M_M_CTRL_CNST_GRP', 'Major_Mouth_R_CornerLip_Mouth_CTRL_CNST_GRP', 'Major_Mouth_L_CornerLip_Mouth_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'LowerLip_M_M_CTRL_CNST_GRP', mo=True)
            pos = mc.xform('Mouth_M_center', q=True, ws=True, t=True)
            loc = mc.spaceLocator(name='Mouth_NULL_loc')[0]
            # Move it to the desired world position
            mc.xform(loc, worldSpace=True, translation=pos)
            for side in ['L', 'R']:
                mc.pointConstraint('LowerLip_M_M_CTRL_CNST_GRP', f'Major_Mouth_{side}_CornerLip_Mouth_CTRL_CNST_GRP', mo=True)
                mc.pointConstraint(loc, f'Major_Mouth_{side}_CornerLip_Mouth_CTRL_CNST_GRP', mo=True)
                mc.parentConstraint(f'Major_Mouth_{side}_CornerLip_Mouth_CTRL', f'NLFold_{side}_{side}_CTRL_CNST_GRP', mo=True)

            mc.parentConstraint('UpperHead_M_CTRL', 'Cheek_L_NLFold_02_Major_jnt', mo=True)
            mc.parentConstraint('UpperHead_M_CTRL', 'Cheek_R_NLFold_02_Major_jnt', mo=True)
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'Cheek_L_NLFold_05_Major_jnt', mo=True)
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'Cheek_R_NLFold_05_Major_jnt', mo=True)

            if self.mastermouth:
                mc.parent(loc, 'Mouth_M_MasterControl_M_CTRL')
            else:
                mc.parent(loc, 'LowerHead_M_CTRL')
            #mc.parent(loc, 'LowerHead_M_CTRL')

            mc.parent('Eye_L_JNT', 'Eye_R_JNT' ,'UpperHead_JNT' )
            mc.hide('Eye_L_Eyelid_InnerCorner_Major_JNT', 'Eye_L_Eyelid_Lower_Major_JNT', 'Eye_L_Eyelid_OuterCorner_Major_JNT', 'Eye_L_Eyelid_Upper_Major_JNT', 'Eye_R_Eyelid_Upper_Major_JNT', 'Eye_R_Eyelid_OuterCorner_Major_JNT', 'Eye_R_Eyelid_Lower_Major_JNT', 'Eye_R_Eyelid_InnerCorner_Major_JNT', 'Mouth_NULL_loc')
            mc.parentConstraint('LowerHead_M_CTRL', 'Jaw_M_larynx_M_CTRL_CNST_GRP', mo=True)
            mc.parentConstraint('neck_M_02_fk_CTRL', 'Jaw_M_larynx_M_CTRL_CNST_GRP', mo=True)
            mc.parent('Jaw_M_larynx_M_CTRL_CNST_GRP', 'RIG')
            
            # === 1️⃣ Define your control variables ===
            look_ctrl = "Look_M_M_CTRL"          # the one user animates
            look_offset = "Look_M_M_CTRL_SDK_GRP"  # the group above it
            head_ctrl = "head_M_01_CTRL"
            root_ctrl = "global_M_CTRL"

            # === 2️⃣ Add the space switch attribute ===
            attr_name = "spaceSwitch"

            if not mc.attributeQuery(attr_name, node=look_ctrl, exists=True):
                mc.addAttr(look_ctrl, longName=attr_name, attributeType="bool", keyable=True)

            # === 3️⃣ Create the parent constraint (Head + Root → Look Offset) ===
            constraint_name = mc.parentConstraint(head_ctrl, root_ctrl, look_offset, maintainOffset=True)[0]

            # === 4️⃣ Create the reverse node ===
            rev = mc.createNode("reverse", name=f"{look_ctrl}_spaceSwitch_REV")

            # === 5️⃣ Hook up the connections ===
            # Connect the look control's attr to head weight and reverse input
            mc.connectAttr(f"{look_ctrl}.{attr_name}", f"{rev}.inputX", f=True)
            mc.connectAttr(f"{look_ctrl}.{attr_name}", f"{constraint_name}.{head_ctrl}W0", f=True)

            # Connect reverse output to the root weight
            mc.connectAttr(f"{rev}.outputX", f"{constraint_name}.{root_ctrl}W1", f=True)
            #mc.parentConstraint('LowerHead_M_CTRL', 'Stache_M_01_M_CTRL_CNST_GRP', mo=True)
            #mc.parentConstraint('LowerHead_M_CTRL', 'Beard_M_01_M_CTRL_CNST_GRP', mo=True)  
            #mc.parent('Stache_M_01_M_CTRL_CNST_GRP', 'Beard_M_01_M_CTRL_CNST_GRP', 'RIG')
            #mc.parent('Beard_M_01_JNT',  'Stache_M_01_JNT'  ,lower_jnt)     

            mc.parentConstraint('UpperLip_M_M_CTRL', 'Stache_M_01_Major_M_CTRL_CNST_GRP', mo=True)
            mc.parentConstraint('Major_Mouth_L_CornerLip_Mouth_CTRL','Stache_L_03_Major_L_CTRL_CNST_GRP', mo=True)
            mc.parentConstraint('Major_Mouth_R_CornerLip_Mouth_CTRL','Stache_R_03_Major_R_CTRL_CNST_GRP', mo=True)
            mc.parent('Stache_M_01_JNT', upper_jnt)
            mc.parentConstraint('LowerLip_M_M_CTRL', 'Beard_M_01_Major_M_CTRL_CNST_GRP', mo=True, )
            mc.pointConstraint('LowerLip_M_M_CTRL', 'Beard_M_03_Major_M_CTRL_CNST_GRP', mo=True, )
            mc.orientConstraint('LowerHead_M_CTRL', 'Beard_M_03_Major_M_CTRL_CNST_GRP', mo=True, )
            mc.parent('Beard_M_01_JNT', lower_jnt)
            mc.parentConstraint('LowerHead_M_CTRL', 'Beard_M_03_Major_M_CTRL', mo=True)

        elif self.custom == 'Luciana':
            mc.parent('Eye_L_look_offset', 'Eye_R_look_offset', 'UpperHead_M_CTRL')
            look_pos = mc.xform('LookNULL_loc', q=True, os=True, t=True)
            ctrl_name, top_group = UEface.build_basic_control( name='Look_M', shape='circle', size=2.0, color_rgb=(1, 1, 0), position=look_pos, rotation=(90, 0, 0))
            mc.parent('Eye_L_Look_L_CTRL_CNST_GRP', 'Eye_R_Look_R_CTRL_CNST_GRP', ctrl_name)
            mc.group('LookNULL_loc', 'Eye_L_eyelid_look_loc', 'Eye_R_eyelid_look_loc', name='Look_Null')
            mc.parent('Look_Null', 'head_M_01_CTRL')
            mc.hide('Look_Null')
            mc.parent('Look_M_M_CTRL_CNST_GRP', 'RIG')
            mc.parent('Brow_L_NULL', 'Brow_R_NULL', 'UpperHead_M_CTRL')
            mc.hide('Brow_L_NULL', 'Brow_R_NULL')
            mc.parent('Brow_L_Master_L_CTRL_CNST_GRP', 'Brow_R_Master_R_CTRL_CNST_GRP', 'Cheek_R_CheekBone_R_CTRL_CNST_GRP', 'Cheek_L_CheekBone_L_CTRL_CNST_GRP', 'UpperHead_M_CTRL' )
            mc.parent('Cheek_L_Puff_L_CTRL_CNST_GRP', 'Cheek_R_Puff_R_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('Jaw_M_root_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('TopTeeth_JNT', 'BotTeeth_JNT', 'Tongue_01_JNT', 'head_M_JNT')
            mc.parent('TopTeeth_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parent('BotTeeth_M_CTRL_CNST_GRP', 'Tongue_01_01_CTRL_CNST_GRP', 'Jaw_M_root_M_CTRL')

            if self.mastermouth:
                mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'TopTeeth_M_CTRL_CNST_GRP', mo=True)
                mc.addAttr('TopTeeth_M_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Topteeth_spaceswitch', proxy='TopTeeth_M_CTRL.MouthMaster_spaceswitch')
                mc.connectAttr('TopTeeth_M_CTRL.MouthMaster_spaceswitch', 'TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')

                #mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'BotTeeth_M_CTRL_CNST_GRP', mo=True)
                #mc.addAttr('BotTeeth_M_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                #mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Botteeth_spaceswitch', proxy='BotTeeth_M_CTRL.MouthMaster_spaceswitch')
                #mc.connectAttr('BotTeeth_M_CTRL.MouthMaster_spaceswitch', 'BotTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')

                #mc.parentConstraint('Mouth_M_MasterControl_M_CTRL', 'Tongue_01_01_CTRL_CNST_GRP', mo=True)
                #mc.addAttr('Tongue_01_01_CTRL', longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0) #TopTeeth_M_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0
                #mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName='Tongue_spaceswitch', proxy='Tongue_01_01_CTRL.MouthMaster_spaceswitch')
                #mc.connectAttr('Tongue_01_01_CTRL.MouthMaster_spaceswitch', 'Tongue_01_01_CTRL_CNST_GRP_parentConstraint1.Mouth_M_MasterControl_M_CTRLW0')
                for control in ['Tongue_01_01_CTRL', 'BotTeeth_M_CTRL']:
                    mdnodetranslate = mc.createNode('multiplyDivide', name=f'{control}MDtranslate')
                    mdnoderotate = mc.createNode('multiplyDivide', name=f'{control}MDrotate')
                    mc.addAttr(control, longName='MouthMaster_spaceswitch', at='bool', k=True, dv=0)
                    mc.addAttr('Mouth_M_MasterControl_M_CTRL', longName=f'{control}_spaceswitch', proxy=f'{control}.MouthMaster_spaceswitch')
                    for axes in ['X', 'Y','Z']:
                        mc.connectAttr(f"Mouth_M_MasterControl_M_CTRL.translate{axes}", f'{mdnodetranslate}.input1{axes}')
                        mc.connectAttr(f"Mouth_M_MasterControl_M_CTRL.rotate{axes}", f'{mdnoderotate}.input1{axes}')
                        mc.connectAttr(f'{control}.MouthMaster_spaceswitch', f'{mdnodetranslate}.input2{axes}')
                        mc.connectAttr(f'{control}.MouthMaster_spaceswitch', f'{mdnoderotate}.input2{axes}')
                    #for control in ['Tongue_01_01_CTRL', 'BotTeeth_M_CTRL']: 
                    mc.connectAttr(f'{mdnodetranslate}.input1', f'{control}_OFF_GRP.translate')
                    mc.connectAttr(f'{mdnoderotate}.input1', f'{control}_OFF_GRP.rotate')
    
            #mc.parent('LowerLip_M_M_CTRL_CNST_GRP', 'UpperLip_M_M_CTRL_CNST_GRP', 'Major_Mouth_R_CornerLip_Mouth_CTRL_CNST_GRP', 'Major_Mouth_L_CornerLip_Mouth_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            if self.mastermouth:
                mc.parent('Mouth_M_MasterControl_M_CTRL_CNST_GRP', 'LowerLip_M_M_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            else:
                mc.parent('LowerLip_M_M_CTRL_CNST_GRP', 'UpperLip_M_M_CTRL_CNST_GRP', 'Major_Mouth_R_CornerLip_Mouth_CTRL_CNST_GRP', 'Major_Mouth_L_CornerLip_Mouth_CTRL_CNST_GRP', 'LowerHead_M_CTRL')
            mc.parentConstraint('Jaw_M_root_M_CTRL', 'LowerLip_M_M_CTRL_CNST_GRP', mo=True)
            pos = mc.xform('Mouth_M_center', q=True, ws=True, t=True)
            loc = mc.spaceLocator(name='Mouth_NULL_loc')[0]
            # Move it to the desired world position
            mc.xform(loc, worldSpace=True, translation=pos)
            for side in ['L', 'R']:
                mc.pointConstraint('LowerLip_M_M_CTRL_CNST_GRP', f'Major_Mouth_{side}_CornerLip_Mouth_CTRL_CNST_GRP', mo=True)
                mc.pointConstraint(loc, f'Major_Mouth_{side}_CornerLip_Mouth_CTRL_CNST_GRP', mo=True)
            
            if self.mastermouth:
                mc.parent(loc, 'Mouth_M_MasterControl_M_CTRL')
            else:
                mc.parent(loc, 'LowerHead_M_CTRL')
            #mc.parent(loc, 'LowerHead_M_CTRL')

            mc.parent('Eye_L_JNT', 'Eye_R_JNT' ,'UpperHead_JNT' )
            mc.hide('Eye_L_Eyelid_InnerCorner_Major_JNT', 'Eye_L_Eyelid_Lower_Major_JNT', 'Eye_L_Eyelid_OuterCorner_Major_JNT', 'Eye_L_Eyelid_Upper_Major_JNT', 'Eye_R_Eyelid_Upper_Major_JNT', 'Eye_R_Eyelid_OuterCorner_Major_JNT', 'Eye_R_Eyelid_Lower_Major_JNT', 'Eye_R_Eyelid_InnerCorner_Major_JNT', 'Mouth_NULL_loc')
            mc.parentConstraint('LowerHead_M_CTRL', 'Jaw_M_larynx_M_CTRL_CNST_GRP', mo=True)
            mc.parentConstraint('neck_M_02_fk_CTRL', 'Jaw_M_larynx_M_CTRL_CNST_GRP', mo=True)
            mc.parent('Jaw_M_larynx_M_CTRL_CNST_GRP', 'RIG')
            
            # === 1️⃣ Define your control variables ===
            look_ctrl = "Look_M_M_CTRL"          # the one user animates
            look_offset = "Look_M_M_CTRL_SDK_GRP"  # the group above it
            head_ctrl = "head_M_01_CTRL"
            root_ctrl = "global_M_CTRL"

            # === 2️⃣ Add the space switch attribute ===
            attr_name = "spaceSwitch"

            if not mc.attributeQuery(attr_name, node=look_ctrl, exists=True):
                mc.addAttr(look_ctrl, longName=attr_name, attributeType="bool", keyable=True)

            # === 3️⃣ Create the parent constraint (Head + Root → Look Offset) ===
            constraint_name = mc.parentConstraint(head_ctrl, root_ctrl, look_offset, maintainOffset=True)[0]

            # === 4️⃣ Create the reverse node ===
            rev = mc.createNode("reverse", name=f"{look_ctrl}_spaceSwitch_REV")

            # === 5️⃣ Hook up the connections ===
            # Connect the look control's attr to head weight and reverse input
            mc.connectAttr(f"{look_ctrl}.{attr_name}", f"{rev}.inputX", f=True)
            mc.connectAttr(f"{look_ctrl}.{attr_name}", f"{constraint_name}.{head_ctrl}W0", f=True)

            # Connect reverse output to the root weight
            mc.connectAttr(f"{rev}.outputX", f"{constraint_name}.{root_ctrl}W1", f=True)
            #mc.parentConstraint('LowerHead_M_CTRL', 'Stache_M_01_M_CTRL_CNST_GRP', mo=True)
            #mc.parentConstraint('LowerHead_M_CTRL', 'Beard_M_01_M_CTRL_CNST_GRP', mo=True)  
            #mc.parent('Stache_M_01_M_CTRL_CNST_GRP', 'Beard_M_01_M_CTRL_CNST_GRP', 'RIG')
            #mc.parent('Beard_M_01_JNT',  'Stache_M_01_JNT'  ,lower_jnt)     
            facefin_def = ['LowerHead_JNT', 'UpperHead_JNT', 'Jaw_M_root_JNT', 'head_M_JNT', 'Horn_L_guide_JNT', 'Horn_R_guide_JNT', 'Fin_L_01_JNT', 'Fin_L_02_JNT', 'Fin_L_03_JNT', 'Fin_L_04_JNT', 'Fin_L_05_JNT', 'Fin_L_06_JNT', 'Fin_L_08_JNT', 'FinLow_L_09_JNT', 'Fin_R_01_JNT', 'Fin_R_02_JNT', 'Fin_R_03_JNT', 'Fin_R_04_JNT', 'Fin_R_05_JNT', 'Fin_R_06_JNT', 'Fin_R_08_JNT', 'FinLow_R_09_JNT',]
            for side in ['L', 'R']:
                mastercontrol = None
                for guide in [f'Fin_{side}_Master', f'Fin_{side}_01', f'Fin_{side}_02', f'Fin_{side}_03', f'Fin_{side}_04', f'Fin_{side}_05', f'Fin_{side}_06', f'Fin_{side}_07', f'Fin_{side}_08', f'FinLow_{side}_09']:
                    pos = mc.xform(guide, q=True, ws=True, t=True)
                    rot = mc.xform(guide, q=True, ws=True, ro=True)
                    jnt, ctrl, offset = UEface.Simple_joint_and_Control(
                        guide=guide,
                        orient=True,
                        CTRL_Size=10,
                        JNT_Size=0.9,
                    )
                    #facefin_def.append(jnt)
                    mc.parent(offset, 'head_M_01_CTRL')
                    mc.parent(jnt, 'head_M_JNT')
                    if guide == f'FinLow_{side}_09':
                        mc.parentConstraint('Jaw_M_root_M_CTRL', offset, mo=True)
                    elif guide ==  f'Fin_{side}_Master':
                        mastercontrol = ctrl
                    else:
                        if mastercontrol:
                            for axes in ['X', 'Y', 'Z']:
                                mc.connectAttr(f'{mastercontrol}.rotate{axes}', f'{guide}_{side}_CTRL_SDK_GRP.rotate{axes}')
            mastercontrol = None
            #facefin_def = ['LowerHead_JNT', 'UpperHead_JNT', 'Jaw_M_root_JNT', 'Head_M_JNT', 'Horn_L_guide_JNT', 'Horn_R_guide_JNT']
            for side in ['M', 'L', 'R']:
                pos = mc.xform(f'Horn_{side}_guide', q=True, ws=True, t=True)
                rot = mc.xform(f'Horn_{side}_guide', q=True, ws=True, ro=True)
                jnt, ctrl, offset = UEface.Simple_joint_and_Control(
                        guide=f'Horn_{side}_guide',
                        orient=True,
                        CTRL_Size=10,
                        JNT_Size=0.9,
                    )
                if mastercontrol:
                    mc.parent(offset, mastercontrol)
                    mc.parent(jnt, masterjnt)

                else:
                    mc.parent(offset, upper_ctrl)
                    mc.parent(jnt, upper_jnt)
                    mastercontrol = ctrl
                    masterjnt = jnt
            mc.hide('Jaw_M_ee_M_CTRL')
            try:
                mc.skinCluster(*facefin_def, 'facefeathers', toSelectedBones=True)
            except Exception as e:
                print(f"Failed to set: {e}")