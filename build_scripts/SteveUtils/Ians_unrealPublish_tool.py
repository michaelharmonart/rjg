import socket
import maya.cmds as cmds
import maya.mel as mel
import os
import shutil

# Globals to hold UI control names
char_radio = None
rig_anim_radio = None
rig_anim_label = None
anim_label = None
scene_select = None
submit_btn = None
anim_name_field = None
old_anim_path = None

# Animation paths
domingo_groups_rig_path = "G:/dragonkisser/anim/Rig_Exports/Domingo"
domingo_groups_anim_path = "G:/dragonkisser/anim/Anim Export/Domingo"
luciana_groups_rig_path = "G:/dragonkisser/anim/Rig_Exports/Luciana"
luciana_groups_anim_path = "G:/dragonkisser/anim/Anim Export/Luciana"


# ---------------------------------------------------------------------
# EXPORT RIG FUNCTION
# ---------------------------------------------------------------------
def export_rig(character_path):
    print("Exporting rig to:", character_path)

    if "domingo" in character_path.lower():
        export_path = os.path.join(character_path, "domingo.fbx")
    else:
        export_path = os.path.join(character_path, "luciana.fbx")
    # ----------------------------------------------------------
    # 1. Get members of unreal_SET
    # ----------------------------------------------------------
    if not cmds.objExists("unreal_SET"):
        cmds.error("unreal_SET does not exist in the scene.")

    members = cmds.sets("unreal_SET", q=True)
    if not members:
        cmds.error("unreal_SET is empty.")

    if members:
        cmds.select(members, replace=True)

        options = (
            "FBXExportFileVersion=FBX2020;"
            "FBXExportInAscii=False;"
            "FBXExportUpAxis=y;"
            "FBXExportUseSceneName=0;"
            "FBXExportEmbeddedTextures=0;"
            "FBXExportCameras=0;"
            "FBXExportLights=0;"
            "FBXExportAudio=0;"
            
            # REQUIRED CHANGES
            "FBXExportGenerateLog=1;"          # History = TRUE
            "FBXExportChannels=0;"             # Channels = FALSE
            "FBXExportExpressions=0;"          # Expressions = FALSE
            "FBXExportConstraints=0;"          # Constraints already FALSE

            "FBXExportSkeletonDefinitions=0;"
            "FBXExportReferencedAssetsContent=1;"
            "FBXExportSmoothingGroups=1;"
            "FBXExportSmoothMesh=1;"
            "FBXExportAnimationOnly=1;"
            "FBXExportBakeComplexAnimation=0;"
            "FBXExportSkins=0;"
            "FBXExportShapes=1;"
            "FBXExportShapeAttributes=1;"
            "FBXExportIncludeChildren=1;"
            "FBXExportInputConnections=0;"
            "FBXExportApplyConstantKeyReducer=0;"
        )

        cmds.file(
            export_path,
            force=True,
            options=options,
            type="FBX export",
            pr=True,
            es=True
        )
        cmds.file(export_path, force=True, options=options, type="FBX export", pr=True, es=True)

        print(f"FBX exported to: {export_path}")


# ---------------------------------------------------------------------
# EXPORT ANIMATION FUNCTION
# ---------------------------------------------------------------------
def export_animation(export_file):
    print("Exporting animation to:", export_file)

    # ---------------------------------------
    # 1. Find ANY set containing "unreal_SET"
    # ---------------------------------------
    all_sets = cmds.ls(type="objectSet")
    target_set = None

    for s in all_sets:
        if "unreal_SET" in s:
            target_set = s
            break

    if not target_set:
        cmds.error('No object set containing "unreal_SET" found.')

    members = cmds.sets(target_set, q=True)
    if not members:
        cmds.error(f'Set "{target_set}" is empty.')

    cmds.select(members, replace=True)

    # ---------------------------------------
    # 2. Bake animation frame range
    # ---------------------------------------
    start_frame = int(cmds.playbackOptions(q=True, minTime=True))
    end_frame   = int(cmds.playbackOptions(q=True, maxTime=True))

    print(f"Baking animation from {start_frame} to {end_frame}")

    # FBX requires these keys if bake is on
    options = (
        "FBXExportFileVersion=FBX2020;"
        "FBXExportInAscii=False;"
        "FBXExportUpAxis=y;"
        "FBXExportUseSceneName=0;"
        "FBXExportEmbeddedTextures=0;"
        "FBXExportCameras=0;"
        "FBXExportLights=0;"
        "FBXExportAudio=0;"
        "FBXExportGenerateLog=1;"
        "FBXExportChannels=0;"
        "FBXExportExpressions=0;"
        "FBXExportConstraints=0;"
        "FBXExportSkeletonDefinitions=0;"
        "FBXExportReferencedAssetsContent=1;"
        "FBXExportSmoothingGroups=1;"
        "FBXExportSmoothMesh=1;"
        "FBXExportAnimationOnly=1;"        # EXPORT ONLY ANIMATION
        "FBXExportBakeComplexAnimation=1;" # MUST BAKE
        f"FBXExportBakeComplexStart={start_frame};"
        f"FBXExportBakeComplexEnd={end_frame};"
        "FBXExportSkins=1;"                # Export skinning for animation
        "FBXExportShapes=1;"
        "FBXExportShapeAttributes=1;"
        "FBXExportIncludeChildren=1;"
        "FBXExportInputConnections=0;"
        "FBXExportApplyConstantKeyReducer=0;"
    )

    # ---------------------------------------
    # 3. Export animation
    # ---------------------------------------
    cmds.file(
        export_file,
        force=True,
        options=options,
        type="FBX export",
        pr=True,
        es=True
    )

    print(f"Animation exported to: {export_file}")

# ---------------------------------------------------------------------
# UI UPDATE LOGIC
# ---------------------------------------------------------------------
def update_ui(*args):
    global char_radio, rig_anim_radio, rig_anim_label, anim_label
    global scene_select, submit_btn, anim_name_field, old_anim_path

    char_sel = cmds.radioButtonGrp(char_radio, q=True, sl=True)

    # Pick correct anim folder
    if char_sel == 1:
        anim_path = luciana_groups_anim_path
    elif char_sel == 2:
        anim_path = domingo_groups_anim_path
    else:
        anim_path = None

    show_rig_anim = char_sel in [1, 2]

    cmds.control(rig_anim_radio, edit=True, visible=show_rig_anim)
    cmds.control(rig_anim_label, edit=True, visible=show_rig_anim)

    # ----------------------------------------------------------
    # If rig/anim radio visible
    # ----------------------------------------------------------
    if show_rig_anim:

        rig_anim_sel = cmds.radioButtonGrp(rig_anim_radio, q=True, sl=True)
        show_anim_select = (rig_anim_sel == 2)

        cmds.control(anim_label, edit=True, visible=show_anim_select)
        cmds.control(scene_select, edit=True, visible=show_anim_select)
        cmds.control(anim_name_field, edit=True, visible=False)

        # Populate animation list only when folder changes
        if show_anim_select and anim_path and os.path.isdir(anim_path):

            if old_anim_path != anim_path:
                old_anim_path = anim_path
                print("Repopulating animation list for:", anim_path)

                existing_items = cmds.optionMenu(scene_select, q=True, itemListLong=True)
                if existing_items:
                    for item in existing_items:
                        cmds.deleteUI(item)

                # Add files
                for f in os.listdir(anim_path):
                    if f.lower().endswith(".fbx"):
                        cmds.menuItem(label=f)

                cmds.menuItem(label="-- Create New Animation --")

        # If NEW animation selected
        if show_anim_select:
            selected_val = cmds.optionMenu(scene_select, q=True, v=True)
            if selected_val == "-- Create New Animation --":
                cmds.control(anim_name_field, edit=True, visible=True)

        cmds.control(submit_btn, edit=True, visible=True)
        return

    # No rig/anim
    cmds.control(anim_label, edit=True, visible=False)
    cmds.control(scene_select, edit=True, visible=False)
    cmds.control(anim_name_field, edit=True, visible=False)


# ---------------------------------------------------------------------
# SUBMIT ACTION
# ---------------------------------------------------------------------
def submit_action(*args):
    global char_radio, rig_anim_radio, scene_select, anim_name_field

    char_sel = cmds.radioButtonGrp(char_radio, q=True, sl=True)

    if char_sel == 1:
        char_name = "Luciana"
        rig_path = luciana_groups_rig_path
        anim_path = luciana_groups_anim_path
    else:
        char_name = "Domingo"
        rig_path = domingo_groups_rig_path
        anim_path = domingo_groups_anim_path

    rig_anim_sel = cmds.radioButtonGrp(rig_anim_radio, q=True, sl=True)

    # -------------------------------
    # RIG EXPORT
    # -------------------------------
    if rig_anim_sel == 1:
        cmds.confirmDialog(title="Export", message=f"Exporting {char_name} Rig...", button=["OK"])
        export_rig(rig_path)
        return

    # -------------------------------
    # ANIMATION EXPORT
    # -------------------------------
    selected_anim = cmds.optionMenu(scene_select, q=True, v=True)

    if selected_anim == "-- Create New Animation --":
        new_name = cmds.textFieldGrp(anim_name_field, q=True, text=True).strip()

        if not new_name:
            cmds.warning("Please enter a name for the new animation.")
            return

        export_file = os.path.join(anim_path, f"{new_name}.fbx")
        print(f"Creating NEW animation: {export_file}")

    else:
        export_file = os.path.join(anim_path, selected_anim)
        print(f"Updating existing animation: {export_file}")

    cmds.confirmDialog(title="Export", message=f"Exporting Animation:\n{export_file}", button=["OK"])
    export_animation(export_file)


# ---------------------------------------------------------------------
# CLOSE UI
# ---------------------------------------------------------------------
def close_window(*args):
    if cmds.window("exportWin", exists=True):
        cmds.deleteUI("exportWin")


# ---------------------------------------------------------------------
# BUILD UI
# ---------------------------------------------------------------------
def build_ui():
    global char_radio, rig_anim_radio, rig_anim_label, anim_label
    global scene_select, submit_btn, anim_name_field

    if cmds.window("exportWin", exists=True):
        cmds.deleteUI("exportWin")

    window = cmds.window("exportWin", title="Export Rig or Animation", widthHeight=(500, 350))
    cmds.columnLayout(columnAlign='left', rowSpacing=8)

    cmds.text(label="Export Options", align='left', height=30)

    char_radio = cmds.radioButtonGrp(
        label='Character Select',
        labelArray2=['Luciana', 'Domingo'],
        numberOfRadioButtons=2,
        cc=update_ui
    )

    rig_anim_label = cmds.text(label='Rig or Animation?', visible=False)

    rig_anim_radio = cmds.radioButtonGrp(
        labelArray2=['Update Rig', 'Update/Create Anim'],
        numberOfRadioButtons=2,
        visible=False,
        cc=update_ui
    )

    anim_label = cmds.text(label="Animation Select", visible=False)

    scene_select = cmds.optionMenu(label="Choose Animation:", visible=False, changeCommand=update_ui)

    anim_name_field = cmds.textFieldGrp(label="New Animation Name:", visible=False)

    cmds.separator(h=20, style='none')

    cmds.rowLayout(numberOfColumns=2, columnWidth2=(250, 250))
    cmds.button(label="Cancel", command=close_window)
    submit_btn = cmds.button(label="Submit", visible=False, command=submit_action)
    cmds.setParent('..')

    cmds.showWindow(window)


# ---------------------------------------------------------------------
# RUN UI
# ---------------------------------------------------------------------
build_ui()