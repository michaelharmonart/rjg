import maya.cmds as mc
import math



def connect_jnts(segments = 30, inverse = False, ctrl = 'Cut_CTRL' ):
    num1 = 0
    num2 = 100
    if inverse:
        name = 'Bot_Length'
        num1 = 3000
        num2 = 2900
        num3 = 100
    else:
        name = 'Top_Length'
        num1 = 0
        num2 = 100
    if inverse:
        for i in range(0,segments,1): 
            translateremap = mc.createNode('remapValue', name =f'translate{i:02d}{name}_REMAP' ) #{num:02d}
            scaleremap = mc.createNode('remapValue', name =f'scale{i:02d}{name}_REMAP' )

            #translate

            mc.setAttr(f'{translateremap}.inputMax', 0)
            mc.setAttr(f'{translateremap}.inputMin', num1)
            mc.setAttr(f'{translateremap}.outputMax', 0)
            mc.setAttr(f'{translateremap}.outputMin', num1)

            mc.connectAttr(f'{ctrl}.translateY', f'{translateremap}.inputValue')
            mc.connectAttr(f'{translateremap}.outValue', f'{name}_{i:02d}_JNT.translateY')

            #scale
            mc.setAttr(f'{scaleremap}.inputMax', -3000 -num3 + 3000)
            mc.setAttr(f'{scaleremap}.inputMin', num2)
            mc.setAttr(f'{scaleremap}.outputMax', .01)
            mc.setAttr(f'{scaleremap}.outputMin', 1)

            mc.connectAttr(f'{ctrl}.translateY', f'{scaleremap}.inputValue')
            mc.connectAttr(f'{scaleremap}.outValue', f'{name}_{i:02d}_JNT.scaleX')
            mc.connectAttr(f'{scaleremap}.outValue', f'{name}_{i:02d}_JNT.scaleZ')

            num1 = num1 - 100
            num2 = num2 - 100
            num3 = num3 + 100



    else:
        for i in range(0,segments,1): 
            translateremap = mc.createNode('remapValue', name =f'translate{i:02d}{name}_REMAP' ) #{num:02d}
            scaleremap = mc.createNode('remapValue', name =f'scale{i:02d}{name}_REMAP' )

            #translate

            mc.setAttr(f'{translateremap}.inputMax', 3000)
            mc.setAttr(f'{translateremap}.inputMin', num1)
            mc.setAttr(f'{translateremap}.outputMax', 3000)
            mc.setAttr(f'{translateremap}.outputMin', num1)

            mc.connectAttr(f'{ctrl}.translateY', f'{translateremap}.inputValue')
            mc.connectAttr(f'{translateremap}.outValue', f'{name}_{i:02d}_JNT.translateY')

            #scale
            mc.setAttr(f'{scaleremap}.inputMax', 3000 + num2)
            mc.setAttr(f'{scaleremap}.inputMin', num2)
            mc.setAttr(f'{scaleremap}.outputMax', .01)
            mc.setAttr(f'{scaleremap}.outputMin', 1)

            mc.connectAttr(f'{ctrl}.translateY', f'{scaleremap}.inputValue')
            mc.connectAttr(f'{scaleremap}.outValue', f'{name}_{i:02d}_JNT.scaleX')
            mc.connectAttr(f'{scaleremap}.outValue', f'{name}_{i:02d}_JNT.scaleZ')

            num1 = num1 + 100
            num2 = num2 + 100




connect_jnts(segments = 30, inverse = False, ctrl = 'Cut_CTRL' )



