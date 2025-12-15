import maya.cmds as mc
from importlib import reload
import re

import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
import rjg.build.guide as rGuide
import rjg.libs.transform as rXform
reload(rAttr)
reload(rChain)
reload(rCtrl)
reload (rGuide)
reload(rXform)

def build_pin_part(geo_to_pin=None, guide=None, pintype='UVpin', parjnt='root_M_JNT'):
    if mc.objExists(parjnt):
        pass
    else:
        parjnt = 'root_M_JNT'

    pos = mc.xform(guide, q=True, ws=True, t=True)

    mc.select(clear=True)

    jnt=mc.joint(n=f'{guide}_JNT',  p=pos)
    mc.parent(jnt, parjnt)

    pin_ctrl =  rCtrl.Control(shape="square", side=None, suffix='CTRL', name=guide, axis='y', group_type='main', rig_type='primary', translate=guide,)  

    mc.parentConstraint(pin_ctrl.ctrl, jnt, mo=True)
    mc.scaleConstraint(pin_ctrl.ctrl, jnt, mo=True)
    mc.parent(pin_ctrl.top, 'RIG')

    #mc.select(geo_to_pin)
    #mc.select(pin_ctrl.top)
    mc.select(clear=True)
    mc.select(geo_to_pin, f'{guide}_CTRL_CNST_GRP')
    mc.UVPin()

