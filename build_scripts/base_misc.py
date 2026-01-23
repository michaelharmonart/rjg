import maya.cmds as mc
import rjg.post.dataIO.ng_weights as rWeightNgIO
import rjg.libs.util as rUtil
from rjg.build_scripts.SteveUtils.importskins import import_weights

from importlib import reload
import platform, time

groups = 'G:' if platform.system() == 'Windows' else '/groups'

reload(rUtil)


def base_extras(skin_src, skin_trg_grp, character):
    bind_joints = [jnt.split('.')[0] for jnt in mc.ls('*.bindJoint')]
    geo = mc.ls(mc.select(skin_trg_grp, hierarchy=True), selection=True)
    mc.select(skin_trg_grp, hierarchy=True)
    geo = mc.ls(selection=True, type='mesh')

    sk_g = []
    group = f'{character}_EXTRAS'
    
    geo = mc.listRelatives(group, children=True, type='transform') or []
    


    classic_sk = [] #'pants', 'shirt1', ]

    #rUtil.create_pxWrap('front_pockets', 'backpockets', 'side_pocket', 'pants')

    for g in geo:
        history = mc.listHistory(g) or []
        skin_clusters = mc.ls(history, type='skinCluster')
        if skin_clusters:
            pass
        else:
            sk = mc.skinCluster(bind_joints, g, tsb=True, skinMethod=1, n='clothingSkc')[0]
            sk_g.append(sk)

    #mc.skinCluster('head_M_JNT', 'Hair', tsb=True, skinMethod=1, n='hairSkc') #skin the hair to only the head joint in order to avoid weird stretching

    for g in sk_g:
        #pass
        mc.copySkinWeights(ss='skinCluster1', ds=g, surfaceAssociation='closestPoint', noMirror=True)
        #rUtil.create_pxWrap([g, 'Rayden_UBM'])

    #read weighted skin maps 
    for g in geo:
        import_weights(geo=g, path=f'{groups}/bobo/character/Rigs/{character}/SkinFiles')




