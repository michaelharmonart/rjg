import platform
import sys
from importlib import reload

from rjg.build_scripts.bettercontrols import apply_control_file
import maya.cmds as mc
import maya.mel as mel

groups = 'G:' if platform.system() == 'Windows' else '/groups'
mc.scriptEditorInfo(suppressWarnings=True,suppressInfo=True)

import rjg.build.buildPart as rBuild
import rjg.build.prop as rProp
import rjg.libs.file as rFile
import rjg.libs.util as rUtil
import rjg.post.dataIO.controls as rCtrlIO
import rjg.post.finalize as rFinal
import rjg.post.usd as rUSD
from rjg.build.parts.bipedLimb import BipedLimb
from rjg.build.parts.clavicle import Clavicle
from rjg.build.parts.hand import Hand
from rjg.libs.skin import auto_split_all_weights
from rjg.libs.profile import add_profiler_tag
import rjg.post.PoseInterpExtras as expi

reload(rUtil)
reload(rProp)
reload(rBuild)
reload(rFinal)
reload(rFile)
reload(rUSD)

import pipe.m.space_switch as spsw
from ngSkinTools2.api import plugin


def ensure_ng_initialized():
    if not plugin.is_plugin_loaded():
        plugin.load_plugin()



### Build Begins ###

def run(character, mp=None, gp=None, ep=None, cp=None, sp=None, pp=None, face=True, previs=False):
    import rjg.build_scripts
    reload(rjg.build_scripts)
    
    import rjg.libs.util as rUtil
    import rjg.post.dataIO.controls as rCtrlIO
    import rjg.post.dataIO.ng_weights as rWeightNgIO
    import rjg.post.dataIO.weights as rWeightIO
    from rjg.build.parts.driverjoints import create_driver_joints
    from rjg.build_scripts import Bobo_Build_Scripts
    from rjg.build_scripts.SteveUtils import CurveNetAtHome
    from rjg.build_scripts.SteveUtils.importskins import import_weights
    from rjg.build_scripts.UnrealCorrectives import BuildCorrectives, build_parents
    from rjg.libs.metadata import create_versioning_script
    reload(rUtil)
    reload(rWeightNgIO)
    reload(rWeightIO)
    reload(rCtrlIO)
    
    ## Setting parameters for individual Characters (splitting off groups)
    not_previs = False if previs or character in ['DungeonMonster', 'Jett', 'Blitz', 'Susaka', 'NPC', 'Fisherman'] else True
    bony = False 

    body_mesh = f'{character}_UBM'

    ensure_ng_initialized()
    mc.file(new=True, f=True)

    #Production
    if character in ['Bobo', 'Gretchen']:
        production = 'HB'
    elif character in ['Domingo', 'Luciana', 'CrowdA', 'CrowdB', 'CrowdC']:
        production = 'DK'
    elif character in ['Robin', 'Rayden', 'DungeonMonster']:
        production = 'LG'
    elif character in ['Jett', 'Blitz']:
        production = 'SG'
    else:
        production = None

    addmusc = True


    ### BUILD SCRIPT
    root = rBuild.build_module(module_type='root', side='M', part='root', model_path=mp, guide_path=gp, base=production, muscle_ctrl=addmusc)
    if ep:
        extras = rFile.import_hierarchy(ep, parent='MODEL')[0]
    #Fun Camera Thing
    mc.viewFit('perspShape', fitFactor=1, all=True, animate=True)

    # Versioning
    create_versioning_script(rig_name=character, rig_version=7.2)
    
    #Fixing Names

    neckList = ['Neck', 'Neck1', 'Neck2', 'Head']

    # Building Parts // setting up the diffrent changes per character

    split_weights = True
    hipshape = 'hips'

    hip = rBuild.build_module(
        module_type="hip",
        side="M",
        part="COG",
        guide_list=["Hips"],
        ctrl_scale=50,
        cog_shape=hipshape,
        waist_shape="circle",
        generate_waist=False,
    )
    spine = rBuild.HybridSpine(
        side="M",
        part="spine",
        base_guide="Hips",
        hip_pivot_guide="HipPivot",
        mid_guide="Spine",
        chest_pivot_guide="Spine",
        upper_chest_pivot_guide="Spine1",
        spine_end_guide="Spine2",
        ctrl_scale=1.5,
        joint_num=7,
        split_weights = split_weights
    )
        

    neck = rBuild.build_module(module_type='autoneck', side='M', part='neck', guide_list=neckList, ctrl_scale=10, segments=3, )
    head = rBuild.build_module(module_type='head', side='M', part='head', guide_list=['Head'], ctrl_scale=50,  autoneckik=True)

    #
    if face:
        for side in ['L', 'R']:
            from rjg.build.parts.UEeye import UEeye
            eye = UEeye(f'Eye_{side}_guides', ctrl_scale=1, skin=['Eyes', 'Corneas'], split=True,)
            eye.build()
            from rjg.build.parts.UEbrow import UEbrow
            brow = UEbrow(f'Brow_{side}_guides', ctrl_scale=1, split=True)
            brow.build()
            from rjg.build.parts.UEcheek import UEcheek
            cheek = UEcheek(f'Cheek_{side}_guides', ctrl_scale=1, NL=True, split=True)
            cheek.build()
            from rjg.build.parts.UEear import UEear
            ear = UEear(f'Ear_{side}_guides', ctrl_scale=1)
            ear.build()

        from rjg.build.parts.UEnose import UEnose
        nose = UEnose('Nose_guides', ctrl_scale=1)
        nose.build()
        from rjg.build.parts.UEjaw import UEjaw
        jaw = UEjaw('Jaw_M_guides', ctrl_scale=1, mentalis=True)
        jaw.build()
        from rjg.build.parts.UEmouth import UEmouth
        mouth = UEmouth('Mouth_guides', ctrl_scale=1, Major_Mouth=4, split=True,)
        mouth.build()
        from rjg.build.parts.UEteeth import UEteeth
        teeth = UEteeth('Tongue_M_guides', ctrl_scale=1, skin=['tongue', 'topteeth', 'botteeth'])
        teeth.build()


        from rjg.build.parts.UEfaceconnect import UEfaceconnect
        faceconnect = UEfaceconnect('UEFace_Guides', ctrl_scale=1, custom='Normal')
        faceconnect.build() 

    #Mirrored Base Rig Parts
    fing_shape = 'circle' 
    curlaxis = 'Z' 
    clavshape = 'Arch'
    Clavmo=True
    Clavaim=False
    FootMus=True
    footshp = "shoe"
    curlshape = "sims"
    chest_control=True
    scap_control=True



    for fs in ["Left", "Right"]:
        clavicle: Clavicle = rBuild.build_module(
            module_type="clavicle",
            side=fs[0],
            part="clavicle",
            guide_list=[fs + piece for piece in ["Shoulder", "Arm"]],
            local_orient=False,
            ctrl_scale=9,
            shape = clavshape,
            mo=Clavmo,
            aim=Clavaim,
            chest_control=chest_control,
            scap_control=scap_control,
        )

        prop_control = True
        arm: BipedLimb = rBuild.build_module(
            module_type="biped_limb",
            side=fs[0],
            part="arm",
            guide_list=[fs + piece for piece in ["Arm", "ForeArm", "Hand"]],
            offset_pv=50,
            ctrl_scale=5,
            bendy=not_previs,
            twisty=not_previs,
            stretchy=not_previs,
            segments=4 if not_previs else 1,
            orient_spaces={
                "world": "ROOT",
                "global": "global_M_CTRL",
                "root": "root_02_M_CTRL",
                "chest01": "chest_M_01_CTRL",
                "chest02": "chest_M_02_CTRL",
            },
            independent_swing_parent="chest_M_02_CTRL",
            independent_swing=True,
            independent_swing_connection_target=clavicle.swing_input,
            swing=True,
            remove_first_joint_twist=True,
            twist_distribute_name="shoulder",
            enable_prop_control=prop_control,
        )
        handroll = False
        bendbo = False
        bendy_switch=None
        ik_fingers = True
        handshape = 'wrist'
            
        # Hand
        hand: Hand = rBuild.build_module(module_type='hand', side=fs[0], part='hand', guide_list=[fs + 'Hand'], ctrl_scale=8, bendy_visibility = bendy_switch, handroll = handroll, handshape=handshape)
        if hand.bendy_vis_attr is not None:
            for control in [arm.fk_ctrls[-1], arm.main_ctrl]:
                mc.addAttr(control.ctrl, longName="handBendyVisibility", proxy=hand.bendy_vis_attr)
        
        Fexpress = False 

        leg = rBuild.build_module(
            module_type="biped_limb",
            side=fs[0],
            part="leg",
            guide_list=[fs + piece for piece in ["UpLeg", "Leg", "Foot"]],
            offset_pv=50,
            ctrl_scale=8,
            bendy=not_previs,
            twisty=not_previs,
            stretchy=True,
            segments=4 if not_previs else 1,
            swing_parent="spine_M_01_JNT",
            swing=True,
            remove_first_joint_twist=True,
            twist_distribute_name="hip"
        )
            
        foot = rBuild.build_module(
            module_type="foot",
            side=fs[0],
            part="foot",
            guide_list=[fs + piece for piece in ["Foot", "ToeBase", "Toe_End"]],
            ctrl_scale=10,
            toe_piv=fs + "ToePiv",
            heel_piv=fs + "HeelPiv",
            in_piv=fs + "In",
            out_piv=fs + "Out",
            express=Fexpress,
            mus_tgt=FootMus,
            foot_shape = footshp
        )
        fingers = []
        
        ffs = ['Index', 'Middle', 'Ring', 'Pinky']
        
        #Fix Bobo's 3 fingered-ness
        if character in ['Bobo', 'Sharkguy']:
            ffs = ffs[:-1]
        for f in ffs:
            finger = rBuild.build_module(
                module_type="finger",
                side=fs[0],
                part="finger" + f,
                guide_list=[
                    fs + "Hand" + f + str(num)
                    for num in range(
                        5
                    )
                ],
                ctrl_scale=1,
                fk_shape=fing_shape,
                bendy=bendbo,
                create_ik=ik_fingers,
                bendy_vis_attr = hand.bendy_vis_attr,
                curlaxis = curlaxis,
                handroll = handroll,
                curlshape = curlshape
            )
            fingers.append(finger)

        thumb = rBuild.build_module(
            module_type="finger",
            side=fs[0],
            part="fingerThumb",
            guide_list=[
                fs + "HandThumb" + str(num + 1) for num in range(4)
            ],
            ctrl_scale=1,
            fk_shape=fing_shape,
            bendy=bendbo,
            create_ik=ik_fingers,
            bendy_vis_attr = hand.bendy_vis_attr,
            curlaxis = curlaxis,
            metacarpal_ik = True,
            curlshape = curlshape
        )
        fingers.append(thumb) 

    """for toe in ['Indextoe', 'Middletoe', 'Ringtoe']:
        toes = rBuild.build_module(module_type="MetaToe",side=side,part=toe, guide_list=[sidelong + "Foot" + toe + str(num)for num in range(5)],
        ctrl_scale=1,
        fk_shape='circle',
        bendy=False,
        create_ik=False,
        curlaxis = 'Z',
        handroll = False,
        curl=True,
        expression_control=True
    )"""


    for side in ['L', 'R']:
        from rjg.build.parts.BaseCorrectives import Build_Correctives
        Build_Correctives(side=side)

    #Clearing Guides grom the scene
    mc.delete('Guides')
    
    #Skinning Proccess Starts here

    

    ### DEFAULT SKIN
    if not bony:
        bind_joints = [jnt.split('.')[0] for jnt in mc.ls('*.bindJoint')]
        geo = mc.ls(body_mesh)
        for g in geo:
            skc = mc.skinCluster(bind_joints, g, tsb=True, skinMethod=1, bindMethod=0)[0]
            mc.setAttr(skc + '.dqsSupportNonRigid', 1)


    ### SKIN/CURVE IO
    try:
        import ngSkinTools2; ngSkinTools2.workspace_control_main_window(); ngSkinTools2.open_ui()
    except Exception as e:
        print(e)

    # read skin data
    if sp:
        if not not_previs and character != 'Jett' and character != 'Blitz' and character !='Susaka' and character !='Drummer' and character !='Luciana' and character != 'Domingo' and character !='Fisherman':
            sp = sp[:-5]
            sp += '_pvis.json'
        sp_div = sp.split('/')
        dir = '/'.join(sp_div[:-1])
        rWeightNgIO.read_skin(body_mesh, dir, sp_div[-1][:-5])

    import rjg.build_scripts.base_misc as rc
    reload(rc)
    if not_previs:
        try: 
            rc.base_extras(body_mesh, extras, character)
        except Exception as e:
            mc.warning(e)

    # initialize skin clusters as ngST layers
    if not bony:
        for s in mc.ls(type='skinCluster'):
            try:
                rWeightNgIO.init_skc(s)
            except Exception as e:
                print(e)

    rFinal.final(utX=90, utY=0, DutZ=15, utScale=3, polish=False, character=character)


    ##### IMPORT POSE INTERPOLATORS
    if pp and not_previs and not bony:
        import rjg.libs.util as rUtil
        rUtil.import_poseInterpolator(pp)

    # clean up scene
    mel.eval('hyperShadePanelMenuCommand("hyperShadePanel1", "deleteUnusedNodes");')
    
    # set up textures
    import rjg.post.textures as rTex
    reload(rTex)
    for item in mc.ls('*_MT1'):
        mc.rename(item, item[:-1])
    rTex.set_textures(character)

    rUSD.connectUSDAttr()


    if face:
            for g in ['Eye_L_Eye_L_Upper_curve_ribbon', 'Eye_R_Eye_R_Lower_curve_ribbon', 'Eye_R_Eye_R_Upper_curve_ribbon', 'Eye_L_Eye_L_Lower_curve_ribbon',]: #'Mouth_LowerLip_surf',
                import_weights(geo=g, path=f'{groups}/bobo/character/Rigs/{character}/SkinFiles')


    if face == False:
        for obj in ['Eyes', 'topteeth', 'botteeth', 'tongue','Corneas']:
            mc.skinCluster('head_M_JNT', obj, tsb=True)



    try:
        skin_clusters = mc.ls(type="skinCluster") or []
        if not skin_clusters:
            print("No skinClusters found in the scene.")
            return
        
        for sc in skin_clusters:
            attr = f"{sc}.dqsSupportNonRigid"
            if mc.objExists(attr):
                try:
                    mc.setAttr(attr, 1)
                except Exception as e:
                    print(f"Failed to set {attr}: {e}")
            else:
                print(f"{attr} does not exist on {sc}")
            mc.setAttr(f'{sc}.skinningMethod', 0)
    except Exception as e:
        print(f"Failed")

    # set up control shapes
    if character in []:
        apply_control_file(cp)
        #fix the root ones
        cp_div = cp.split('/')
        dir = '/'.join(cp_div[:-1])
        rCtrlIO.read_ctrls(dir, curve_file=cp_div[-1][:-5])
    
    else:
        if cp:
            cp_div = cp.split('/')
            dir = '/'.join(cp_div[:-1])
            rCtrlIO.read_ctrls(dir, curve_file=cp_div[-1][:-5]) 

    auto_split_all_weights('MODEL')


    print(f"\n{character} rig build complete.")

