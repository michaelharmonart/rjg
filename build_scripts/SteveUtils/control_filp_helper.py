import maya.cmds as cmds

def flip_controls(ctrls, flip=True):
    
    for ctrl in ctrls:
        
        if flip:
            # add suffix
            new_name = f"{ctrl}_DONT"
            
            if cmds.objExists(ctrl) and not cmds.objExists(new_name):
                cmds.rename(ctrl, new_name)

        else:
            # remove suffix
            dont_name = f"{ctrl}_DONT"
            
            if cmds.objExists(dont_name):
                cmds.rename(dont_name, ctrl)
                                
controls = ["COG_M_CTRL", "global_M_CTRL", "foot_L_01_L_CTRL", "foot_R_01_R_CTRL", "hand_L_01_CTRL", "hand_R_01_CTRL", "RJG_M_CTRL"]