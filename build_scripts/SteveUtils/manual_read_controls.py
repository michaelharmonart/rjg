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
import rjg.post.PoseInterpExtras as expi

reload(rUtil)
reload(rProp)
reload(rBuild)
reload(rFinal)
reload(rFile)
reload(rUSD)

import pipe.m.space_switch as spsw
from ngSkinTools2.api import plugin




rCtrlIO.read_ctrls(f"{groups}/bobo/character/Rigs/Luciana/Controls/", curve_file="Luciana_control_curves")  