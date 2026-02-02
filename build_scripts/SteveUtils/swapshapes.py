import maya.cmds as mc

def swap_shapes_final():
    sel = mc.ls(sl=True, long=True, type='transform')
    if len(sel) != 2:
        mc.error("Select exactly TWO transform objects.")
        return

    obj_a, obj_b = sel

    shapes_a = mc.listRelatives(obj_a, shapes=True, fullPath=True) or []
    shapes_b = mc.listRelatives(obj_b, shapes=True, fullPath=True) or []

    if not shapes_a and not shapes_b:
        mc.warning("No shape nodes found on either object.")
        return

    # Parent shapes to opposite objects
    for s in shapes_a:
        mc.parent(s, obj_b, shape=True, relative=True)

    for s in shapes_b:
        mc.parent(s, obj_a, shape=True, relative=True)

    print("✅ Shape nodes swapped safely!")

# Run
swap_shapes_final()
