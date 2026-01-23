import platform
import sys
from importlib import reload

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

reload(rUtil)
reload(rProp)
reload(rBuild)
reload(rFinal)
reload(rFile)
reload(rUSD)

import pipe.m.space_switch as spsw
from ngSkinTools2.api import plugin


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



def run_ue_correctives(character, groups, sp=None):
    """
    Applies Unreal Engine style corrective setup for specific characters.

    Parameters:
    - character: str, name of the character ('Susaka', 'NPC', 'Fisherman', 'Drummer', 'Sharkguy')
    - groups: str, base path for skin files
    - mc: maya.cmds module
    - mel: maya.mel module
    - BuildCorrectives: function to build correctives
    - build_parents: function to parent corrective guides
    - import_weights: function to import skin weights
    - create_driver_joints: function to create driver joints
    - rWeightNgIO: optional, object providing read_skin() if using sp
    - sp: optional, path to skin weights
    """

    allowed_chars = ['Susaka', 'NPC', 'Fisherman', 'Drummer', 'Sharkguy']
    if character not in allowed_chars:
        print(f"Character '{character}' not in allowed list. Skipping UE correctives.")
        return



    #New UE Corrective Fixes
    if character in ['Susaka', 'NPC', 'Fisherman']: #Drumkmer
        #Rebuild Skeleton to supoort UE name Scheme and Correctives
        import rjg.post.unrealJntRename as rUEJnt
        rUEJnt.unrealJntRename()
        # Build UE Correctives 
        CorrGuides2 = ["upperarm_twistCor_01", "lowerarm_correctiveRoot", "upperarm_correctiveRoot", "thigh_correctiveRoot", "calf_correctiveRoot"]
        CorrParrent2 = ["upperarm_l", "lowerarm_l", "upperarm_l", "thigh_l", "calf_l"]
        build_parents(CorrGuides2, CorrParrent2)

        CorrGuides = ["lowerarm_out", "lowerarm_fwd", "lowerarm_in", "lowerarm_bck", "lowerarm_twist_01", "lowerarm_twist_02", 
                    "wrist_outer", "wrist_inner",
                    "upperarm_bicep", "upperarm_tricep",
                    "upperarm_out", "upperarm_bck", "upperarm_fwd", "upperarm_in", "upperarm_twist_02", 
                    "clavicle_out", "clavicle_scap",
                    "clavicle_pec", "spine_04_latissimus",
                    "thigh_fwd", "thigh_fwd_lwr", "thigh_in", "thigh_bck", "thigh_out", "thigh_bck_lwr", "thigh_twist_01", "thigh_twist_02", "pelv_in",
                    "calf_knee", "calf_kneeBack", "calf_twist_01", "calf_twist_02",
                    "HandThumb0_Bot", "HandThumb1_Bot", "HandThumb1_TOP", "HandThumb2_Bot", "HandThumb2_TOP", "HandThumb3_Bot", "HandThumb3_TOP",
                    "HandIndex0_BOT", "HandIndex1_BOT", "HandIndex1_TOP","HandIndex2_BOT", "HandIndex2_TOP", "HandIndex3_BOT", "HandIndex3_TOP",
                    "HandMiddle0_BOT", "HandMiddle1_BOT", "HandMiddle1_TOP","HandMiddle2_BOT", "HandMiddle2_TOP", "HandMiddle3_BOT", "HandMiddle3_TOP",
                    "HandRing0_BOT", "HandRing1_BOT", "HandRing1_TOP","HandRing2_BOT", "HandRing2_TOP", "HandRing3_BOT", "HandRing3_TOP",
                    "HandPinky0_BOT", "HandPinky1_BOT", "HandPinky1_TOP","HandPinky2_BOT", "HandPinky2_TOP", "HandPinky3_BOT", "HandPinky3_TOP",
                    "ankle_fwd", "ankle_bck",
                    ]

        CorrParrent = ["lowerarm_correctiveRoot_l", "lowerarm_correctiveRoot_l", "lowerarm_correctiveRoot_l", "lowerarm_correctiveRoot_l", "lowerarm_correctiveRoot_l", "lowerarm_correctiveRoot_l",
                    "hand_l", "hand_l",
                    "upperarm_twistCor_01_l", "upperarm_twistCor_01_l", 
                    "upperarm_correctiveRoot_l", "upperarm_correctiveRoot_l", "upperarm_correctiveRoot_l", "upperarm_correctiveRoot_l", "upperarm_correctiveRoot_l",
                    "clavicle_l", "clavicle_l",
                    "spine_05", "spine_05",
                    "spine_M_01_JNT", "thigh_correctiveRoot_l", "thigh_correctiveRoot_l", "spine_M_01_JNT", "thigh_correctiveRoot_l", "thigh_correctiveRoot_l", "thigh_correctiveRoot_l", "thigh_correctiveRoot_l", "spine_M_01_JNT",
                    "calf_correctiveRoot_l", "calf_correctiveRoot_l", "calf_correctiveRoot_l", "calf_correctiveRoot_l",
                    "thumb_01_l", "thumb_01_l", "thumb_01_l", "thumb_02_l", "thumb_02_l", "thumb_03_l", "thumb_03_l",
                    "index_metacarpal_l", "index_01_l", "index_01_l", "index_02_l", "index_02_l", "index_03_l", "index_03_l",
                    "middle_metacarpal_l", "middle_01_l", "middle_01_l", "middle_02_l", "middle_02_l", "middle_03_l", "middle_03_l", 
                    "ring_metacarpal_l", "ring_01_l", "ring_01_l", "ring_02_l", "ring_02_l", "ring_03_l", "ring_03_l", 
                    "pinky_metacarpal_l", "pinky_01_l", "pinky_01_l", "pinky_02_l", "pinky_02_l", "pinky_03_l", "pinky_03_l",
                    "foot_l", "foot_l", 
                    ]

        BuildCorrectives(CorrGuides, CorrParrent)
        #Re-Skin with new correctives
        from rjg.build_scripts.Susaka_Misc import ribbons
        ribbons()
        

        bindjoints = mc.select(mc.listRelatives("SKEL", ad=True, type="joint"))
        mc.select(f'{character}_UBM')
        mc.skinCluster(f'{character}_UBM', edit=True, unbind=True)
        skc = mc.skinCluster('root', f'{character}_UBM', tsb=False, skinMethod=1, bindMethod=0)[0]
        mc.setAttr(skc + '.dqsSupportNonRigid', 1)
        if sp:
            sp_div = sp.split('/')
            dir = '/'.join(sp_div[:-1])
            rWeightNgIO.read_skin(f'{character}_UBM', dir, sp_div[-1][:-5])
        if character == 'Susaka':
            geo = ['Wrap', 'Glove', 'UnderPantLayer', 'TempHair', 'Nails', 'TempBrows', 'RightEye', 'RightCornea', 'LeftEye', 'LeftCornea', 'Pants2', 'Belt', 'Scarf', 'CowlBase', 'Hood', 'Straps', 'Collar', 'Shirt', 'Pants', 'ArmBand', 'Kneepad', 'RShoe', 'LShoe', 'RoboArm', 'ClothCover' ]
        elif character == 'Drummer':
            geo = ['HeadHigher', 'HandsHigher', 'BotTeeth', 'Cigar', 'Coat', 'CoatPocketLiner', 'Glassess', 'HeadBand', 'LeftCornea', 'LeftEye', 'Lenses', 'Nails', 'Neckless', 'Overalls', 'Pants', 'RightCornea', 'RightEye', 'Scarf', 'Shoes', 'Socks', 'Sweater', 'Hair', 'TopTeeth', 'Tounge']
        elif character == 'NPC':
            geo = ['RightEye', 'RightCornea', 'LeftEye', 'LeftCornea', 'TopTeeth', 'BotTeeth', 'Tounge', 'ClothesGEO', 'Brows', 'Brow', 'Hair']
        elif character == 'Fisherman':
            geo = ['lowShirtGEOlow', 'lowBootGEOlow', 'lowPegLegGEOlow', 'lowHeadGEOlow', 'lowFingerNailsGEOlow', 'lowBeltGEOlow', 'lowPipeGEOlow', 'lowREyeGEOlow', 'lowLEyeGEOlow', 'lowCoatGEOlow', 'lowHandsGEOlowpolySurface2', 'lowFisherManCleanUplow', 'lowSmokeGEOlow', 'lowTempHairGEOlow', 'lowpolySurface1HandsGEOlow', 'HatGEOlowlow']

        
        else:
            geo = []
        
        for g in geo:
            sk = mc.skinCluster('root', g, tsb=False, skinMethod=1, n=f'clothingSkc{g}')[0]
            mc.copySkinWeights(ss='skinCluster11', ds=f'clothingSkc{g}', surfaceAssociation='closestPoint', noMirror=True, )
        if character == 'Susaka':
            for g in ['Eye_L_Eye_L_Upper_curve_ribbon', 'Eye_R_Eye_R_Upper_curve_ribbon', 'Eye_L_Eye_L_Lower_curve_ribbon', 'Eye_R_Eye_R_Lower_curve_ribbon', 'Mouth_LowerLip_surf', 'Mouth_UpperLip_surf', 'Hood', 'Scarf', 'Collar', 'TempHair', 'RoboArm' ]:
                import_weights(geo=g, path=f'{groups}/bobo/character/Rigs/{character}/SkinFiles')
        if character == 'Fisherman':
            for g in ['Eye_L_Eye_L_Upper_curve_ribbon', 'Eye_R_Eye_R_Upper_curve_ribbon', 'Eye_L_Eye_L_Lower_curve_ribbon', 'Eye_R_Eye_R_Lower_curve_ribbon', 'Mouth_LowerLip_surf', 'Mouth_UpperLip_surf', ]:
                import_weights(geo=g, path=f'{groups}/bobo/character/Rigs/Susaka/SkinFiles')


        try:
            driver_controls = ['Major_Mouth_M_LowerLip_01_Mouth_CTRL', 'Major_Mouth_R_LowerLip_03_Mouth_CTRL', 'Major_Mouth_R_UpperLip_03_Mouth_CTRL', 'Major_Mouth_R_CornerLip_Mouth_CTRL', 'Major_Mouth_L_LowerLip_03_Mouth_CTRL', 'Major_Mouth_L_UpperLip_03_Mouth_CTRL', 'Major_Mouth_M_UpperLip_01_Mouth_CTRL', 'Major_Mouth_L_CornerLip_Mouth_CTRL', 'Eye_L_Upper_Major_L_CTRL', 'Eye_L_Lower_Major_L_CTRL', 'Eye_R_Lower_Major_R_CTRL', 'Eye_R_Upper_Major_R_CTRL', 'Brow_R_01_Major_R_CTRL', 'Brow_R_02_Major_R_CTRL', 'Brow_R_Inner_R_CTRL', 'Brow_R_Outer_R_CTRL', 'Brow_R_Master_R_CTRL', 'Brow_L_02_Major_L_CTRL', 'Brow_L_01_Major_L_CTRL', 'Brow_L_Inner_L_CTRL', 'Brow_L_Outer_L_CTRL', 'Brow_L_Master_L_CTRL', 'Jaw_M_root_M_CTRL']
            create_driver_joints(default_mult=10.0, ctrl_suffix="_CTRL", joint_suffix="_Driver", controls=driver_controls, parent='head')
        except:
            print('couldnt create driver joints')


        '''
        # Import UE Pose Interp Poses
        if character == 'Susaka':
            POSE_FILE = r"G:/bobo/character/Rigs/Susaka/Poses/Poses_01.json"
            sys.path.append(f'{groups}/bobo/pipeline/pipeline/software/maya/scripts/rjg/build_scripts')
            from UEPoseImport import CleanImport
            CleanImport(POSE_FILE)
        #mc.delete("UE_Correctives")
        '''

        
        #Fixing/Reskinning Facial Geo 
        print("ExtraSkins")
        try:
            try:
                mc.select('LeftEye', 'RightEye', 'LeftCornea', 'RightCornea', 'BotTeeth', 'TopTeeth', 'Tounge')
                mel.eval('doDetachSkin 3 { "1", "1", "1" };')
                mc.skinCluster('Eye_L_JNT', 'LeftCornea', mi=1, tsb=True)
                mc.skinCluster('Eye_R_JNT', 'RightCornea', mi=1, tsb=True)
                mc.skinCluster('Eye_L_JNT', 'LeftEye', mi=1, tsb=True)
                mc.skinCluster('Eye_R_JNT', 'RightEye', mi=1, tsb=True)
                mc.skinCluster('BotTeeth_JNT', 'BotTeeth', mi=1, tsb=True)
                mc.skinCluster('Tongue_01_JNT', 'Tongue_02_JNT', 'Tongue_03_JNT', 'Tongue_04_JNT', 'Tongue_05_JNT', 'Tounge', mi=1, tsb=True)
                mc.skinCluster('TopTeeth_JNT', 'TopTeeth', mi=1, tsb=True)
                print('Reskinned')
            except Exception as e:
                print(e)
        except Exception as e:
            print(e)
        

    if character == 'Sharkguy':
        import sys

        from rjg.build_scripts.Sharkguy_Build_Scripts import ribbons
        ribbons()
        bindjoints = mc.select(mc.listRelatives("SKEL", ad=True, type="joint"))
        mc.select(f'{character}_UBM')
        mc.skinCluster(f'{character}_UBM', edit=True, unbind=True)
        skc = mc.skinCluster('root_M_JNT', f'{character}_UBM', tsb=False, skinMethod=1, bindMethod=0)[0]
        mc.setAttr(skc + '.dqsSupportNonRigid', 1)
        if sp:
            sp_div = sp.split('/')
            dir = '/'.join(sp_div[:-1])
            rWeightNgIO.read_skin(f'{character}_UBM', dir, sp_div[-1][:-5])
        geo = ['MouthGEO', 'RopesGEO', 'ShirtGEO', 'SwordGEO', 'PauldrenGEO', 'ChestGEO', 'RGautletGEO', 'BeltGEO', 'PantsGEO', 'LGautletGEO', 'OtherEyeBitGEO', 'EyesGEO']
        for g in geo:
            sk = mc.skinCluster('root_M_JNT', g, tsb=False, skinMethod=1, n=f'clothingSkc{g}')[0]
            mc.copySkinWeights(ss='skinCluster11', ds=f'clothingSkc{g}', surfaceAssociation='closestPoint', noMirror=True, )
        mc.delete("Extras_Guides")
        for g in ['Eye_L_Eye_L_Upper_curve_ribbon', 'Eye_R_Eye_R_Upper_curve_ribbon', 'Eye_L_Eye_L_Lower_curve_ribbon', 'Eye_R_Eye_R_Lower_curve_ribbon', 'Mouth_LowerLip_surf', 'Mouth_UpperLip_surf', 'MouthGEO', 'RopesGEO', 'ShirtGEO', 'SwordGEO', 'PauldrenGEO', 'ChestGEO', 'RGautletGEO', 'BeltGEO', 'PantsGEO', 'LGautletGEO', 'OtherEyeBitGEO', 'EyesGEO']:
            import_weights(geo=g, path=f'{groups}/bobo/character/Rigs/{character}/SkinFiles')
        mc.parentConstraint('neck_02_FK_M_CTRL', 'Fin01_M_M_CTRL_CNST_GRP', mo=True)