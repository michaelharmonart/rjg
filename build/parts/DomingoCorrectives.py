import maya.cmds as mc
import rjg.build.rigModule as rModule


def Build_Correctives(side='L'):
    rig_module = rModule.RigModule(side=side, part="correctives")
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
    f'{SideLong}LegBack'
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
    "COG_M_JNT"                   # LeftLegBack
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
        mc.parent(basejoint_name, param)

        joint_name = f"{base_name}_{SideShort}_JNT"
        mc.joint(p=pos, name=joint_name)
        mc.xform(joint_name, ws=True, ro=rot)
        mc.makeIdentity(joint_name, apply=True, translate=False, rotate=True)
        mc.parent(joint_name, basejoint_name)


        rig_module.tag_bind_joints(joint_name)
