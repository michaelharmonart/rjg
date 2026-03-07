import rjg.post.dataIO.controls as rCtrlIO
import rjg.libs.control.draw as draw
import maya.cmds as mc




def normalize_ctrl_shapes(ctrl):
    shapes = mc.listRelatives(ctrl, shapes=True, noIntermediate=True) or []

    # sort shapes by name to stabilize order
    shapes = sorted(shapes)

    for i, shp in enumerate(shapes):
        if i == 0:
            new_name = f"{ctrl}Shape"
        else:
            new_name = f"{ctrl}Shape{i}"

        if shp != new_name:
            shp = mc.rename(shp, new_name)


d = draw.Draw(); d.write_curve(name='NAME')

normalize_ctrl_shapes('hips')