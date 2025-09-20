import maya.cmds as mc
import rjg.post.dataIO.ng_weights as rWeightNgIO
import rjg.libs.util as rUtil
import sys

from importlib import reload
import platform, time

groups = 'G:' if platform.system() == 'Windows' else '/groups'

reload(rUtil)

manual_skins = [
    #'Eyeball',
    #'Cornea',
    #'Hat',
    #'Shirt',
    #'Boots',
    #'Hair',
    #'Belt',
    #'Tongue',
    #'TopTeeth',
    #'BottomTeeth',
]

def write_clothes():
    for ms in manual_skins:
        rWeightNgIO.write_skin(ms, groups + '/dungeons/character/Rigging/Rigs/Rayden/Skin/Gretchen', name=ms, force=True)
        print("saved:", ms)
        time.sleep(0.5)

def Susaka_extras(skin_src, skin_trg_grp):
    bind_joints = [jnt.split('.')[0] for jnt in mc.ls('*.bindJoint')]
    geo = mc.ls(mc.select(skin_trg_grp, hierarchy=True), selection=True)
    mc.select(skin_trg_grp, hierarchy=True)
    geo = mc.ls(selection=True, type='mesh')

    sk_g = []

    geo = [
        ['Wrap', 'Glove', 'UnderPantLayer', 'TempHair', 'Nails', 'TempBrows', 'RightEye', 'RightCornea', 'LeftEye', 'LeftCornea', 'Pants2', 'Belt', 'Scarf', 'CowlBase', 'Hood', 'Straps', 'Collar', 'Shirt', 'Pants', 'ArmBand', 'Kneepad', 'RShoe', 'LShoe', 'TopTeeth', 'BotTeeth', 'Tounge']
    ]

    #rUtil.create_pxWrap('Shirt', 'Pants', 'Gretchen_UBM')
    #rUtil.create_pxWrap('VestFluff', 'Clothes')
    #mc.parent('Fingernails', 'Rayden_EXTRAS')

    for g in geo:
        sk = mc.skinCluster(bind_joints, g, tsb=True, skinMethod=1, n='clothingSkc')[0]
        sk_g.append(sk)

    mc.skinCluster('head_M_JNT', 'Hair', tsb=True, skinMethod=1, n='hairSkc') #skin the hair to only the head joint in order to avoid weird stretching
        
    for g in sk_g:
        pass
        mc.copySkinWeights(ss='skinCluster1', ds=g, surfaceAssociation='closestPoint', noMirror=True, )
        #rUtil.create_pxWrap([g, 'Rayden_UBM'])

   

def Susaka_misc_pvis(skin_src, skin_trg_grp):
    bind_joints = [jnt.split('.')[0] for jnt in mc.ls('*.bindJoint')]
    geo = mc.ls(mc.select(skin_trg_grp, hierarchy=True), selection=True)
    mc.select(skin_trg_grp, hierarchy=True)
    geo = mc.ls(selection=True, type='mesh')

    sk_g = []
    geo = [
        ['MouthGEO', 'RopesGEO', 'ShirtGEO', 'SwordGEO', 'PauldrenGEO', 'ChestGEO', 'RGautletGEO', 'BeltGEO', 'PantsGEO', 'LGautletGEO', 'OtherEyeBitGEO', 'EyesGEO']
    ]
    for g in geo:
        sk = mc.skinCluster(bind_joints, g, tsb=True, skinMethod=1, n='clothingSkc')[0]
        sk_g.append(sk)

    for g in sk_g:
        mc.copySkinWeights(ss='skinCluster1', ds=g, surfaceAssociation='closestPoint', noMirror=True, )


def ribbons():
    L_Strap = ['Rope1_01', 'Rope1_02', 'Rope1_03', 'Rope1_04']
    R_Strap = ['Rope2_01', 'Rope2_02', 'Rope2_03', 'Rope2_04', 'Rope2_05']

    sys.path.append(f'{groups}/bobo/pipeline/pipeline/software/maya/scripts/rjg/build/parts')
    from ribbon import build_ribbon
    build_ribbon(guide_list=L_Strap,
                fromrig=False,
                Control_List=['Rope1_01', 'Rope1_04'],
                axis='x',
                ribbon_width=0.5,
                prefix='Rope1',
                parent_type='single',
                parent='waist_M_CTRL',
                parent_list=[],
                parent_joint='spine_M_01_JNT')
    build_ribbon(guide_list=R_Strap,
                fromrig=False,
                Control_List=['Rope2_01', 'Rope2_03', 'Rope2_05'],
                axis='x',
                ribbon_width=0.5,
                prefix='Rope2',
                parent_type='single',
                parent='waist_M_CTRL',
                parent_list=[],
                parent_joint='spine_M_01_JNT')
    """build_ribbon(guide_list=Scarf,
                fromrig=False,
                Control_List=['Scarf_Top01_M', 'Scarf_Top03_L', 'Scarf_Top13_M', 'Scarf_Top03_R'],
                axis='x',
                ribbon_width=0.5,
                prefix='Scarf',
                parent_type='single',
                parent='neck_01_FK_M_CTRL',
                parent_list=[],
                parent_joint='spine_05')"""
    
    

