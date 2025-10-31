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
    f"foot_{SideShort}_01_JNT",   # AnkleFrontBack
    f"foot_{SideShort}_01_JNT",   # LeftAnkleFront
    f"leg_{SideShort}_04_JNT",    # LeftKneeFront
    f"leg_{SideShort}_04_JNT",    # LeftKneeBack
    f"clavicle_{SideShort}_01_JNT", # LeftTrap
    f"clavicle_{SideShort}_01_JNT", # LeftArmPit
    f"clavicle_{SideShort}_01_JNT", # LeftPec
    f"clavicle_{SideShort}_01_JNT", # LeftScap
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
        joint_name = f"{base_name}_{SideShort}_JNT"
        pos = mc.xform(guide, q=True, ws=True, t=True)
        rot = mc.xform(guide, q=True, ws=True, ro=True)
        mc.joint(p=pos, o=rot, name=joint_name)
        mc.parent(joint_name, param)
        rig_module.tag_bind_joints(joint_name)
