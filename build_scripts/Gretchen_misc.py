import maya.cmds as mc
import rjg.post.dataIO.ng_weights as rWeightNgIO
import rjg.libs.util as rUtil

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

def Gretchen_extras(skin_src, skin_trg_grp):
    bind_joints = [jnt.split('.')[0] for jnt in mc.ls('*.bindJoint')]
    geo = mc.ls(mc.select(skin_trg_grp, hierarchy=True), selection=True)
    mc.select(skin_trg_grp, hierarchy=True)
    geo = mc.ls(selection=True, type='mesh')

    sk_g = []
    
    geo = ['bandanna', 'loopleather1', 'buckle2', 'buckle', 'beltloops', 'honey_pin', 'RightCornea', 'RightEye', 'RightPupil', 'LeftEye', 'LeftCornea', 
            'LeftPupil', 'topeyelashes', 'bottomlash', 'eyebrows', 'hair', 'belt', 'earrings', 'lenses', 'glasses1', 'frame', 'hinge', 'bottomteeth', 
            'topteeth', 'tounge', 'gloves', 'button3','button', 'thread1', 'thread']
    
    buttons = ['button3','button', 'thread1', 'thread']


    classic_sk = [] #'pants', 'shirt1', ]

    #rUtil.create_pxWrap('shirt1', 'pantsCreased1', 'boots', 'Gretchen_UBM')
    rUtil.create_pxWrap('front_pockets', 'backpockets', 'side_pocket', 'pants')
    #rUtil.create_pxWrap('VestFluff', 'Clothes')

    for g in geo:
        sk = mc.skinCluster(bind_joints, g, tsb=True, skinMethod=1, n='clothingSkc')[0]
        sk_g.append(sk)
    
    for g in classic_sk:
        sk_gloves = mc.skinCluster(bind_joints, g, tsb=True, skinMethod=0, n='clothingSkc')[0]

    #mc.skinCluster('head_M_JNT', 'Hair', tsb=True, skinMethod=1, n='hairSkc') #skin the hair to only the head joint in order to avoid weird stretching

    for g in sk_g:
        #pass
        mc.copySkinWeights(ss='skinCluster1', ds=g, surfaceAssociation='closestPoint', noMirror=True, )
        #rUtil.create_pxWrap([g, 'Rayden_UBM'])

    

    #if character == 'Fisherman':
    #        for g in ['Eye_L_Eye_L_Upper_curve_ribbon', 'Eye_R_Eye_R_Upper_curve_ribbon', 'Eye_L_Eye_L_Lower_curve_ribbon', 'Eye_R_Eye_R_Lower_curve_ribbon', 'Mouth_LowerLip_surf', 'Mouth_UpperLip_surf', ]:
    #            import_weights(geo=g, path=f'{groups}/bobo/character/Rigs/Susaka/SkinFiles')



    shirt_skin = mc.skinCluster(bind_joints, 'shirt1', tsb=True, skinMethod=1, n='shirtSkc')[0]
    pants_skin = mc.skinCluster(bind_joints, 'pants', tsb=True, skinMethod=1, n='pantsSkc')[0]
    shoes_skin = mc.skinCluster(bind_joints, 'boots', tsb=True, skinMethod=1, n='shoesSkc')[0]
    gloves_skin = mc.skinCluster(bind_joints, 'gloves', tsb=True, skinMethod=1, n='glovesSkc')[0]
    #WeightNgIO.init_skc(shirt_skin)
    #rWeightNgIO.init_skc(pants_skin)
    #rWeightNgIO.init_skc(shoes_skin)
    #rWeightNgIO.init_skc(gloves_skin)
    rWeightNgIO.read_skin("shirt1", "/groups/bobo/character/Rigs/Gretchen/Weights/", "Gretchen_Shirt_Weights_01")
    rWeightNgIO.read_skin("pants", "/groups/bobo/character/Rigs/Gretchen/Weights/", "Gretchen_Pants_Weights")
    rWeightNgIO.read_skin("boots", "/groups/bobo/character/Rigs/Gretchen/Weights/", "Gretchen_Shoes_Weights")
    rWeightNgIO.read_skin("gloves", "/groups/bobo/character/Rigs/Gretchen/Weights/", "Gretchen_Gloves_Weights")

    for g in buttons:
        mc.copySkinWeights(ss=shirt_skin, ds=g, surfaceAssociation='closestPoint', noMirror=True, )


   

def Gretchen_misc_pvis(skin_src, skin_trg_grp):
    bind_joints = [jnt.split('.')[0] for jnt in mc.ls('*.bindJoint')]
    geo = mc.ls(mc.select(skin_trg_grp, hierarchy=True), selection=True)
    mc.select(skin_trg_grp, hierarchy=True)
    geo = mc.ls(selection=True, type='mesh')

    sk_g = []

    for g in geo:
        sk = mc.skinCluster(bind_joints, g, tsb=True, skinMethod=1, n='clothingSkc')[0]
        sk_g.append(sk)

    for g in sk_g:
        mc.copySkinWeights(ss='skinCluster1', ds=g, surfaceAssociation='closestPoint', noMirror=True, )
