import maya.cmds as mc
import rjg.build.rigModule as rModule


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
        f'{SideLong}ElbowIn',
        f'{SideLong}ElbowOut',
        f'{SideLong}WristTop',
        f'{SideLong}WristBot',
        f'{SideLong}WristIn',
        f'{SideLong}WristOut',
        f'{SideLong}Pec',
        f'{SideLong}Pit',
        f'{SideLong}ShoulderOut',
        f'{SideLong}Trap',
        f'{SideLong}WingElbowFront', #Start here Leg_L_01_bindJNT  Leg_L_02_bindJNT Leg_L_03_bindJNT
        f'{SideLong}UpKneeFront',
        f'{SideLong}UpKneeBack',
        f'{SideLong}LowKneeFront',
        f'{SideLong}LowKneeBack',
        f'{SideLong}HipBack',
        f'{SideLong}HipFront',
        f'{SideLong}HipUp',
        f'{SideLong}HipIn',
        ]
        
        parlist = [
        f"arm_{SideShort}_04_JNT",   # AnkleFrontBack
        f"arm_{SideShort}_04_JNT",
        f"arm_{SideShort}_08_JNT",
        f"arm_{SideShort}_08_JNT",
        f"arm_{SideShort}_08_JNT",
        f"arm_{SideShort}_08_JNT",
        f"chest_M_JNT",
        f"chest_M_JNT",
        f"arm_{SideShort}_01_JNT",
        f"clavicle_{SideShort}_01_JNT",
        f"Wing_{SideShort}_02_bind_JNT",
        f"Leg_{SideShort}_02_bindJNT",
        f"Leg_{SideShort}_02_bindJNT",
        f"Leg_{SideShort}_03_bindJNT",
        f"Leg_{SideShort}_03_bindJNT",
        f"COG_M_JNT",
        f"COG_M_JNT",
        f"Leg_{SideShort}_01_bindJNT",
        f"Leg_{SideShort}_01_bindJNT",
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
            mc.parent(basejoint_name, param)

            joint_name = f"{base_name}_{SideShort}_JNT"
            mc.joint(p=pos, name=joint_name)
            mc.xform(joint_name, ws=True, ro=rot)
            mc.makeIdentity(joint_name, apply=True, translate=False, rotate=True)
            mc.setAttr(f'{joint_name}.segmentScaleCompensate', 0)
            mc.parent(joint_name, basejoint_name)


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
