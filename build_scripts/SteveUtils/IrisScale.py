import maya.cmds as mc

def BuildScalingEyePart(side='L',
                        select=True,
                        Part='iris',
                        pivot='loc',
                        upvector='loc2',
                        controlchannel='look.channel',
                        parentjnt='eye',
                        parent=True):

    if select:
        sel = mc.ls(sl=True, type='joint')
    else:
        sel = []

    if not sel:
        mc.warning("No joints selected.")
        return

    roots = []

    # get pivot world pos
    pivot_pos = mc.xform(pivot, q=True, ws=True, t=True)

    for i, jnt in enumerate(sel, start=1):
        num = str(i).zfill(2)

        root_name = f"{side}_{Part}_{num}_root_JNT"
        end_name  = f"{side}_{Part}_{num}_EE_JNT"
        mc.select(clear=True)

        if mc.objExists(root_name):
            mc.warning(f"{root_name} already exists, skipping.")
            continue

        # create root at pivot
        root = mc.joint(name=root_name, p=pivot_pos)
        mc.select(clear=True)

        # orient root using aimConstraint (Y points at joint, X stabilized by upvector)
        aim = mc.aimConstraint(jnt,
                               root,
                               aimVector=(0, 1, 0),
                               upVector=(1, 0, 0),
                               worldUpType="object",
                               worldUpObject=upvector)
        mc.delete(aim)

        # freeze root rotations
        mc.makeIdentity(root, apply=True, t=0, r=1, s=0, n=0)

        # rename selected joint and parent it
        new_end = mc.rename(jnt, end_name)
        mc.parent(new_end, root)

        # freeze end rotations
        mc.makeIdentity(new_end, apply=True, t=0, r=1, s=0, n=0)

        # connect control channel
        try:
            mc.connectAttr(controlchannel, f"{root}.rotateZ", f=True)
        except:
            mc.warning(f"Could not connect {controlchannel} to {root}.rotateZ")

        roots.append(root)

    # parent all new roots under parentjnt
    if parent and mc.objExists(parentjnt):
        try:
            mc.parent(roots, parentjnt)
        except:
            mc.warning(f"Could not parent roots under {parentjnt}")


# Example call
BuildScalingEyePart(side='L',
                    select=True,
                    Part='iris',
                    pivot='tempPivot_loc_01',
                    upvector='tempPivot_loc_03',
                    controlchannel='L_Aim_ctrl.Iris_Size',
                    parentjnt='L_Eye_jnt',
                    parent=True)
