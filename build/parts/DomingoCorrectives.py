import maya.cmds as mc
import rjg.build.rigModule as rModule
import rjg.libs.transform as rXform


def Build_Correctives(side='L'):
    rig_module = rModule.RigModule(side=side, part="correctives")
    if side != 'M':
        if side == 'L':
            SideShort = 'L'
            SideLong = 'Left'
        elif side == 'R':
            SideShort = 'R'
            SideLong = 'Right'
        
        
        guidelist = [
        f'{SideLong}AnkleBack',
        f'{SideLong}AnkleFront',
        f'{SideLong}KneeFront',
        f'{SideLong}KneeBack',
        f'{SideLong}Trap',
        f'{SideLong}ArmPit',
        f'{SideLong}Pec',
        f'{SideLong}Scap',
        f'{SideLong}Bicep',
        f'{SideLong}ElbowIn',
        f'{SideLong}WristIn',
        f'{SideLong}WristHigh',
        f'{SideLong}WristLow',
        f'{SideLong}LegIn',
        f'{SideLong}LegOut',
        f'{SideLong}LegFront',
        f'{SideLong}LegBack',
        f'{SideLong}Neck',
        f'{SideLong}Neck1',
        f'{SideLong}Head',

        ]
        
        parlist = [
        f"leg_{SideShort}_08_JNT",   # AnkleFrontBack
        f"leg_{SideShort}_08_JNT",   # LeftAnkleFront 
        f"leg_{SideShort}_05_JNT",    # LeftKneeFront
        f"leg_{SideShort}_05_JNT",    # LeftKneeBack
        f"chest_M_JNT", # LeftTrap
        f"chest_M_JNT", # LeftArmPit
        f"chest_M_JNT", # LeftPec
        f"chest_M_JNT", # LeftScap
        f"clavicle_{SideShort}_01_JNT", # LeftBicep
        f"arm_{SideShort}_04_JNT",    # LeftElbowIn
        f"arm_{SideShort}_08_JNT",    # LeftWristIn
        f"arm_{SideShort}_08_JNT",    # LeftWristHigh
        f"arm_{SideShort}_08_JNT",    # LeftWristLow
        f"leg_{SideShort}_01_JNT",    # LeftLegIn
        "COG_M_JNT",                  # LeftLegOut
        "COG_M_JNT",                  # LeftLegFront
        "COG_M_JNT",                   # LeftLegBack
        'neck_M_01_JNT',
        'neck_M_02_JNT',
        'head_M_JNT',
        ]
            
        for guide, param in zip(guidelist, parlist):
            # Remove SideLong from the start of the string
            base_name = guide.replace(SideLong, "", 1)  # only replace the first occurrence
            # Build the new joint name
            basejoint_name = f"{base_name}_{SideShort}_Base_JNT"
            pos = mc.xform(guide, q=True, ws=True, t=True)
            rot = mc.getAttr(f"{guide}.jointOrient")[0]


            mc.joint(p=pos, name=basejoint_name)
            mc.xform(basejoint_name, ws=True, ro=rot)
            mc.makeIdentity(basejoint_name, apply=True, translate=False, rotate=True)
            mc.setAttr(f'{basejoint_name}.segmentScaleCompensate', 0)
            #mc.parent(basejoint_name, param)
            rXform.clean_parent(basejoint_name, param)

            joint_name = f"{base_name}_{SideShort}_JNT"
            mc.joint(p=pos, name=joint_name)
            mc.xform(joint_name, ws=True, ro=rot)
            mc.makeIdentity(joint_name, apply=True, translate=False, rotate=True)
            mc.setAttr(f'{joint_name}.segmentScaleCompensate', 0)
            #mc.parent(joint_name, basejoint_name)
            rXform.clean_parent(joint_name, basejoint_name)


            rig_module.tag_bind_joints(joint_name)
    else:

        for SideLong in ['Front', 'Back']:
            SideShort = f'{SideLong}_M'

            guidelist = [
                f'{SideLong}Neck1', f'{SideLong}Neck2', f'{SideLong}Head',
            ]
            parlist = [
                'neck_M_01_JNT', 'neck_M_02_JNT', 'head_M_JNT',
            ]

            for guide, param in zip(guidelist, parlist):
                # Remove SideLong from the start of the string
                base_name = guide #guide.replace(SideLong, "", 1)  # only replace the first occurrence
                # Build the new joint name
                basejoint_name = f"{base_name}_{SideShort}_Base_JNT"
                pos = mc.xform(guide, q=True, ws=True, t=True)
                rot = mc.getAttr(f"{guide}.jointOrient")[0]


                mc.joint(p=pos, name=basejoint_name)
                mc.xform(basejoint_name, ws=True, ro=rot)
                mc.makeIdentity(basejoint_name, apply=True, translate=False, rotate=True)
                mc.parent(basejoint_name, param)

                joint_name = f"{base_name}_{SideShort}_JNT"
                mc.joint(p=pos, name=joint_name)
                mc.xform(joint_name, ws=True, ro=rot)
                mc.makeIdentity(joint_name, apply=True, translate=False, rotate=True)
                mc.parent(joint_name, basejoint_name)


                rig_module.tag_bind_joints(joint_name)





'''
    f'{SideLong}HandIndex2Out',
    f'{SideLong}HandIndex3Out',
    f'{SideLong}HandIndex4Out',
    f'{SideLong}HandMiddle2Out',
    f'{SideLong}HandMiddle3Out',
    f'{SideLong}HandMiddle4Out',
    f'{SideLong}HandRing2Out',
    f'{SideLong}HandRing3Out',
    f'{SideLong}HandRing4Out',
    f'{SideLong}HandPinky2Out',
    f'{SideLong}HandPinky3Out',
    f'{SideLong}HandPinky4Out',
    f'{SideLong}HandPinky2In',
    f'{SideLong}HandPinky3In',
    f'{SideLong}HandPinky4In',
    f'{SideLong}HandRing2In',
    f'{SideLong}HandRing3In',
    f'{SideLong}HandRing4In',
    f'{SideLong}HandMiddle2In',
    f'{SideLong}HandMiddle3In',
    f'{SideLong}HandMiddle4In',
    f'{SideLong}HandIndex2In',
    f'{SideLong}HandIndex3In',
    f'{SideLong}HandIndex4In',
    f'{SideLong}HandIndex1In',
    f'{SideLong}HandMiddle1In',
    f'{SideLong}HandRing1In',
    f'{SideLong}HandPinky1In',
    f'{SideLong}HandPinky1Out',
    f'{SideLong}HandRing1Out',
    f'{SideLong}HandMiddle1Out',
    f'{SideLong}HandIndex1Out',
    f'{SideLong}HandThumb1In',
    f'{SideLong}HandThumb2In',
    f'{SideLong}HandThumb3In',
    f'{SideLong}HandThumb2Out',
    f'{SideLong}HandThumb3Out',
    f'{SideLong}HandThumb1Out',

    f'fingerIndex_{SideShort}_02_JNT',
    f'fingerIndex_{SideShort}_03_JNT',
    f'fingerIndex_{SideShort}_04_JNT',
    f'fingerMiddle_{SideShort}_02_JNT',
    f'fingerMiddle_{SideShort}_03_JNT',
    f'fingerMiddle_{SideShort}_04_JNT',
    f'fingerRing_{SideShort}_02_JNT',
    f'fingerRing_{SideShort}_03_JNT',
    f'fingerRing_{SideShort}_04_JNT',
    f'fingerPinky_{SideShort}_02_JNT',
    f'fingerPinky_{SideShort}_03_JNT',
    f'fingerPinky_{SideShort}_04_JNT',
    f'fingerPinky_{SideShort}_02_JNT',
    f'fingerPinky_{SideShort}_03_JNT',
    f'fingerPinky_{SideShort}_04_JNT',
    f'fingerRing_{SideShort}_02_JNT',
    f'fingerRing_{SideShort}_03_JNT',
    f'fingerRing_{SideShort}_04_JNT',
    f'fingerMiddle_{SideShort}_02_JNT',
    f'fingerMiddle_{SideShort}_03_JNT',
    f'fingerMiddle_{SideShort}_04_JNT',
    f'fingerIndex_{SideShort}_02_JNT',
    f'fingerIndex_{SideShort}_03_JNT',
    f'fingerIndex_{SideShort}_04_JNT',
    f'fingerIndex_{SideShort}_01_JNT',
    f'fingerMiddle_{SideShort}_01_JNT',
    f'fingerRing_{SideShort}_01_JNT',
    f'fingerPinky_{SideShort}_01_JNT',
    f'fingerPinky_{SideShort}_01_JNT',
    f'fingerRing_{SideShort}_01_JNT',
    f'fingerMiddle_{SideShort}_01_JNT',
    f'fingerIndex_{SideShort}_01_JNT',
    f'fingerThumb_{SideShort}_01_JNT',
    f'fingerThumb_{SideShort}_02_JNT',
    f'fingerThumb_{SideShort}_03_JNT',
    f'fingerThumb_{SideShort}_02_JNT',
    f'fingerThumb_{SideShort}_03_JNT',
    f'fingerThumb_{SideShort}_01_JNT'

    '''