import maya.cmds as mc

def build_basic_control(name='Main', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=(0, 0, 0), rotation=(0, 0, 0)):
    """
    Builds a basic control with an offset group. The offset group holds the transform.
    Uses RGB override color instead of color index.

    Args:
        name (str): Control name.
        shape (str): Shape type (currently just 'circle' supported).
        size (float): Size of the control.
        color_rgb (tuple): RGB color override.
        position (tuple): World position (x, y, z).
        rotation (tuple): World rotation (x, y, z).

    Returns:
        ctrl (str): The name of the control.
        offset_grp (str): The name of the offset group.
    """
    # Create the control
    ctrl = mc.circle(name=f'{name}_CTRL', normal=[0, 1, 0], radius=size, ch=False)[0]

    # Create offset group
    offset_grp = mc.group(empty=True, name=f"{name}_GRP")
    mc.parent(ctrl, offset_grp)

    # Apply world-space transform to the group
    mc.xform(offset_grp, ws=True, translation=position, rotation=rotation)

    # Set control color using RGB
    mc.setAttr(f"{ctrl}.overrideEnabled", 1)
    mc.setAttr(f"{ctrl}.overrideRGBColors", 1)
    mc.setAttr(f"{ctrl}.overrideColorRGB", color_rgb[0], color_rgb[1], color_rgb[2], type="double3")
    return ctrl, offset_grp
    #build_basic_control(name='name', size=10, color_rgb=(1,1,0), position=(0,0,0), rotation=(0,0,0))


def get_selected_joint_data(buildcontrol=True, idd='fk', replace='bind'):
    joints = mc.ls(selection=True, type="joint")
    
    if not joints:
        mc.warning("No joints selected.")
        return
    
    lastjnt = None
    lastctrl = None

    for jnt in joints:
        pos = mc.xform(jnt, q=True, ws=True, t=True)
        orient = mc.xform(jnt, q=True, ws=True, rotation=True)

        new_name = jnt.replace(replace, idd)

        mc.select(clear=True)
        newjnt = mc.joint(name=new_name, p=pos)
        mc.setAttr(f"{newjnt}.jointOrient", *orient)

        if lastjnt:
            mc.parent(newjnt, lastjnt)
        lastjnt = newjnt

        if buildcontrol:
            ctrl, offset_grp = build_basic_control(
                name=new_name,
                size=5.0,
                color_rgb=(1, 1, 0),
                position=pos,
                rotation=orient
            )
            mc.parentConstraint(ctrl, newjnt, mo=True)
            mc.scaleConstraint(ctrl, newjnt, mo=True)

            if lastctrl:
                mc.parent(offset_grp, lastctrl)
            lastctrl = ctrl
get_selected_joint_data(buildcontrol = False, idd='ik', replace='bind')        
        
         
                                
        
    
    

                 
                                
                                                                