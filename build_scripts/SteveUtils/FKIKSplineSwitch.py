import maya.cmds as mc

def build_ik_fk_switch(prefix="branch_",
                       suffix="_JNT",
                       fk="FK",
                       ik="IK",
                       bind="BIND",
                       root="branch_world_CTRL",
                       IKRoot="branch_spline_root_OFF",
                       FKRoot="branch_root_FK_grp",
                       MO=True):

    # ----------------------------------------
    # 1. Validate root
    # ----------------------------------------
    if not mc.objExists(root):
        mc.error(f"[IK/FK Switch] Root control does not exist: {root}")

    # ----------------------------------------
    # 2. Add FK__IK_Switch attribute
    # ----------------------------------------
    if not mc.attributeQuery("FK__IK_Switch", node=root, exists=True):
        mc.addAttr(root, ln="FK__IK_Switch", at="double", min=0, max=1, dv=0, k=True)

    switch_attr = f"{root}.FK__IK_Switch"

    # ----------------------------------------
    # 3. Make reverse node
    # ----------------------------------------
    reverse_node = mc.createNode("reverse", n=f"{prefix}FKIK_reverse")
    mc.connectAttr(switch_attr, f"{reverse_node}.inputX")

    print(f"[IK/FK Switch] Reverse node created: {reverse_node}")

    # ----------------------------------------
    # 4. Find all bind joints matching: prefix + Part + "_" + bind + suffix
    # ----------------------------------------
    search_pattern = f"{prefix}*_{bind}{suffix}"
    bind_joints = mc.ls(search_pattern) or []

    if not bind_joints:
        mc.error(f"No joints found matching pattern: {search_pattern}")

    print(f"[IK/FK Switch] Found Bind Joints: {bind_joints}")

    # Extract the Part
    part_list = []
    for j in bind_joints:
        # Example: branch_limb01_BIND_JNT → limb01
        core = j.replace(prefix, "").replace(f"_{bind}{suffix}", "")
        part_list.append(core)

    print(f"[IK/FK Switch] Extracted Parts: {part_list}")

    # ----------------------------------------
    # 5. Loop through parts and connect FK → Bind and IK → Bind
    # ----------------------------------------
    for part in part_list:

        bind_jnt = f"{prefix}{part}_{bind}{suffix}"
        fk_jnt   = f"{prefix}{part}_{fk}{suffix}"
        ik_jnt   = f"{prefix}{part}_{ik}{suffix}"

        if not mc.objExists(fk_jnt):
            mc.error(f"Missing FK joint: {fk_jnt}")
        if not mc.objExists(ik_jnt):
            mc.error(f"Missing IK joint: {ik_jnt}")

        # ---- Parent Constraints ----
        con = mc.parentConstraint(
            fk_jnt, ik_jnt, bind_jnt, mo=MO, n=f"{prefix}{part}_ikfk_parentConstraint"
        )[0]

        # Constraint targets follow this pattern:
        # nodeName.target[0].targetWeight
        fk_w = f"{con}.{fk_jnt}W0"
        ik_w = f"{con}.{ik_jnt}W1"

        # ---- Connect weights ----
        mc.connectAttr(switch_attr, fk_w)                  # FK weight follows switch directly
        mc.connectAttr(f"{reverse_node}.outputX", ik_w)    # IK weight uses reversed

        print(f"[IK/FK Switch] Connected FK/IK for: {part}")

    # ----------------------------------------
    # 6. Visibility hookup
    # ----------------------------------------
    # FK visible = switch
    if mc.objExists(FKRoot):
        try:
            mc.connectAttr(switch_attr, f"{FKRoot}.visibility", force=True)
        except:
            print(f"Warning: FK root visibility already connected.")

    # IK visible = reverse
    if mc.objExists(IKRoot):
        try:
            mc.connectAttr(f"{reverse_node}.outputX", f"{IKRoot}.visibility", force=True)
        except:
            print(f"Warning: IK root visibility already connected.")

    print("\n[IK/FK Switch] Setup Complete!")
    print(f"Reverse Node: {reverse_node}")
    return reverse_node


build_ik_fk_switch(prefix="branch_",
                       suffix="_JNT",
                       fk="FK",
                       ik="IK",
                       bind="BIND",
                       root="branch_world_CTRL",
                       IKRoot="branch_spline_root_OFF",
                       FKRoot="branch_root_FK_grp",
                       MO=True)