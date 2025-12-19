import maya.cmds as mc
from importlib import reload
import re
import math

import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
import rjg.build.guide as rGuide
import rjg.libs.transform as rXform
from rjg.build.UEface import UEface
reload(rAttr)
reload(rChain)
reload(rCtrl)
reload(rGuide)
reload(rXform)

class comb(UEface):
    def __init__(self, grp_name=None, ctrl_scale=1, parent = 'root_M_JNT', inputlist =[]):
        super().__init__(part='spline', grp_name=grp_name, ctrl_scale=ctrl_scale)
        self.parent = parent
        self.inputlist = inputlist 

    @staticmethod
    def look_at_rotation(pos1, pos2):
        """
        Returns rotation (rx, ry, rz) in degrees so Z+ points from pos1 to pos2.
        """
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        dz = pos2[2] - pos1[2]

        # Yaw (rotation around Y axis)
        ry = math.degrees(math.atan2(dx, dz))  # note order: x/z

        # Pitch (rotation around X axis)
        dist_xz = math.sqrt(dx*dx + dz*dz)
        rx = -math.degrees(math.atan2(dy, dist_xz))

        # Roll is 0 if you don't care
        rz = 0

        return (rx, ry, rz)

    def build(self, source='GuideList', offsetname=None, inputcurve = None, buildControls=False, guidecurve=None, feathernum=None, Stretch=True, StretchControl = None, prefix=None, autoconstrain=True, finalclustertwist=False, featherrot=[0,0,0] ):
        #'GuideList', 'Curve', 'jointlist'
        tempprefix = self.inputlist[0]
        prefix = tempprefix[:-3]

        #Groups
        if mc.objExists("comb_M"):
            mastergroup = "comb_M"
        else:
            mastergroup = mc.group(empty=True, name = "comb_M")
            mc.parent(mastergroup, 'RIG')
        
        fkgrp = mc.group(empty=True, name=f'{prefix}_FK')
        ikgrp = mc.group(empty=True, name=f'{prefix}_IK')
        handlegrp = mc.group(empty=True, name=f'{prefix}_Handle')
        
        mc.addAttr(mastergroup, ln=f'{prefix}_IKFK', at='bool', k=True)
        switchattr = f'{mastergroup}.{prefix}_IKFK'
        revnode = mc.createNode("reverse", name=f"{prefix}fkIk_reverse")
        mc.connectAttr(switchattr, f'{revnode}.inputX')
        revswitchattr = f'{revnode}.outputX'

        mc.connectAttr(switchattr, f'{fkgrp}.visibility')
        mc.connectAttr(revswitchattr, f'{ikgrp}.visibility')

        mc.parent(fkgrp, ikgrp, handlegrp, mastergroup)
        mc.hide(handlegrp)
        controllist = []

        

        #bind
        bindjnts = []
        if source == 'GuideList':
            prejnt = None
            prepos = None
            for i, guide in enumerate(self.inputlist, start=1):
                mc.select(clear=True)
                pos = mc.xform(guide, q=True, ws=True, t=True)
                if offsetname:
                    idx = f"{i:02d}"   # pads to 2 digits → 01, 02, 03...
                    jnt = mc.joint(name=f"{offsetname}_{idx}_JNT")
                else:
                    jnt = mc.joint(name=f'{guide}_JNT')
                    bindjnts.append(jnt)
                    
                
                if prejnt:
                    rot = comb.look_at_rotation(pos, prepos)
                else:
                    posfuture = mc.xform(self.inputlist[1], q=True, ws=True, t=True)
                    rot = comb.look_at_rotation(posfuture, pos)

                mc.xform(jnt, ws=True, t=pos)
                #mc.xform(jnt, ws=True, ro=rot)
                mc.setAttr(f"{jnt}.jointOrientX", rot[0])
                mc.setAttr(f"{jnt}.jointOrientY", rot[1])
                mc.setAttr(f"{jnt}.jointOrientZ", rot[2])

                if prejnt:
                    mc.parent(jnt, prejnt)
                else:
                    mc.parent(jnt, self.parent)
                    split_joint = jnt
                
                prejnt = jnt
                prepos = pos

        split_joints: list[str] = bindjnts
        mc.addAttr(split_joint, longName="split_joints", dataType="string")
        mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")

        #FK

        if source == 'GuideList':
            prejnt = None
            prepos = None
            prectrl = None
            for i, guide in enumerate(self.inputlist, start=1):
                mc.select(clear=True)

                fkjnt, fkctrl, fkctrl_offset =UEface.Simple_joint_and_Control(
                    guide,
                    overwrite=True,
                    overwrite_name = f'{guide}_FK',
                    orient=True,
                    scale=True,
                    check_side=True,
                    CTRL_Size=1,
                    JNT_Size=0.5
                )

                controllist.append(fkctrl)



                if prejnt:
                    mc.parent(fkjnt, prejnt)
                    mc.parent(fkctrl_offset, prectrl)
                else:
                    mc.parent(fkjnt, handlegrp)
                    mc.parent(fkctrl_offset, fkgrp)
                
                prejnt = fkjnt
                prectrl = fkctrl

                switchparent = mc.parentConstraint(fkjnt, f'{guide}_JNT', mo=True)

                fkweight_attrs = mc.parentConstraint(switchparent[0], q=True, wal=True)

                mc.connectAttr(switchattr, f'{switchparent[0]}.{fkweight_attrs[0]}')
                




        #IK

        if source == 'GuideList':
            prejnt = None
            prepos = None
            jntlist = []
            for i, guide in enumerate(self.inputlist, start=1):
                mc.select(clear=True)
                pos = mc.xform(guide, q=True, ws=True, t=True)
                if offsetname:
                    idx = f"{i:02d}"   # pads to 2 digits → 01, 02, 03...
                    jnt = mc.joint(name=f"{offsetname}_{idx}_IK")
                    jntlist.append(jnt)
                else:
                    jnt = mc.joint(name=f'{guide}_IK')

                    #switchparent = mc.parentConstraint(jnt, f'{guide}_JNT', mo=True)

                    #fkweight_attrs = mc.parentConstraint(switchparent[0], q=True, wal=True)

                    #mc.connectAttr(revswitchattr, f'{switchparent[0]}.{fkweight_attrs[1]}')

                    jntlist.append(jnt)
                    
                
                if prejnt:
                    rot = comb.look_at_rotation(pos, prepos)
                else:
                    posfuture = mc.xform(self.inputlist[1], q=True, ws=True, t=True)
                    rot = comb.look_at_rotation(posfuture, pos)

                mc.xform(jnt, ws=True, t=pos)
                #mc.xform(jnt, ws=True, ro=rot)
                mc.setAttr(f"{jnt}.jointOrientX", rot[0])
                mc.setAttr(f"{jnt}.jointOrientY", rot[1])
                mc.setAttr(f"{jnt}.jointOrientZ", rot[2])

                if prejnt:
                    mc.parent(jnt, prejnt)
                else:
                    mc.parent(jnt, handlegrp)
                
                prejnt = jnt
                prepos = pos

            masterikctrl, masterikoffset = UEface.build_basic_control(
            name=f'{self.inputlist[0]}_IK',
            shape='square',
            size=1.0,
            color_rgb=(1, 1, 0),
            position=pos,
            rotation=rot
            )
            mc.parent(masterikoffset, ikgrp)
            mc.parentConstraint(self.parent, masterikoffset, mo=True)
            controllist.append(masterikctrl)
            mc.addAttr(masterikctrl, ln='stretchy', at='double', dv=1, k=True, max=1, min=0 )
            Stretch_attr = f'{masterikctrl}.stretchy'

            #######################################################
        ik_handle, effector, curve = mc.ikHandle(
            sj=jntlist[0],
            ee=jntlist[-1],
            sol='ikSplineSolver',
            ccv=True,
            pcv=False
        )

        name = f"M_{jntlist[0]}_spline"

        curve = mc.rename(curve, f'M_{name}_curve')
        ik_handle = mc.rename(ik_handle, f'M_{name}_handle')
        mc.parent(ik_handle, handlegrp)
        mc.parent(curve, handlegrp)


        # Step 2: For each CV on the curve, create cluster + control
        cvs = mc.ls(f"{curve}.cv[*]", fl=True)
        ctrl_list = []
        offset_list = []
        print(jntlist)


        for i, cv in enumerate(cvs, start=1):
            # Make cluster for the CV
            cluster, cluster_handle = mc.cluster(cv, n=f"{name}_{i:02}_cluster")
            mc.parent(cluster_handle, handlegrp)
            # Get cluster position
            pos = mc.pointPosition(cv, w=True)
            if i == 1:
                mc.parentConstraint(self.parent, cluster_handle, mo=True)
            elif i in [2, 3]:
                mc.parentConstraint(masterikctrl, cluster_handle, mo=True)
                mc.parentConstraint(self.parent, cluster_handle, mo=True)
            else:
                mc.parentConstraint(masterikctrl, cluster_handle, mo=True)

            #StretchControl = masterikctrl
        if Stretch:
            if Stretch_attr:
                postcurve = mc.listRelatives(curve, s=True, ni=True)[0]
                precurve = mc.listRelatives(curve, s=True, ni=False, type='nurbsCurve')
                precurve = [s for s in precurve if mc.getAttr(s + ".intermediateObject")]
                precurve = precurve[0]

                postci = mc.createNode("curveInfo", name=f"{curve}_postci")
                preci = mc.createNode("curveInfo", name=f"{curve}_preci")
                mc.connectAttr(f"{postcurve}.worldSpace[0]", f"{postci}.inputCurve", force=True)
                mc.connectAttr(f"{precurve}.worldSpace[0]", f"{preci}.inputCurve", force=True)

                frac = mc.createNode("multiplyDivide", name=f"{curve}_Frac")

                # Set the operation to DIVIDE (2)
                mc.setAttr(f"{frac}.operation", 2)

                # Connect inputs
                mc.connectAttr(f"{postci}.arcLength", f"{frac}.input1X", force=True)
                mc.connectAttr(f"{preci}.arcLength", f"{frac}.input2X", force=True) 

                md = mc.createNode("multiplyDivide", name=f"{curve}_MD")

                mc.connectAttr(f"{frac}.outputX", f"{md}.input1X", force=True)
                mc.connectAttr(Stretch_attr, f"{md}.input2X", force=True)

                for newjnt in jntlist:
                    mc.connectAttr(f"{md}.outputX", f"{newjnt}.scaleZ")

            else:
                postcurve = mc.listRelatives(curve, s=True, ni=True)[0]
                precurve = mc.listRelatives(curve, s=True, ni=False, type='nurbsCurve')
                precurve = [s for s in precurve if mc.getAttr(s + ".intermediateObject")]
                precurve = precurve[0]

                postci = mc.createNode("curveInfo", name=f"{curve}_postci")
                preci = mc.createNode("curveInfo", name=f"{curve}_preci")
                mc.connectAttr(f"{postcurve}.worldSpace[0]", f"{postci}.inputCurve", force=True)
                mc.connectAttr(f"{precurve}.worldSpace[0]", f"{preci}.inputCurve", force=True)

                frac = mc.createNode("multiplyDivide", name=f"{curve}_Frac")

                # Set the operation to DIVIDE (2)
                mc.setAttr(f"{frac}.operation", 2)

                # Connect inputs
                mc.connectAttr(f"{postci}.arcLength", f"{frac}.input1X", force=True)
                mc.connectAttr(f"{preci}.arcLength", f"{frac}.input2X", force=True) 

                #md = mc.createNode("multiplyDivide", name=f"{curve}_MD")

                #mc.connectAttr(f"{frac}.outputX", f"{md}.input1X", force=True)
                #mc.connectAttr(Stretch_attr, f"{md}.input2X", force=True)

                for newjnt in jntlist:
                    mc.connectAttr(f"{frac}.outputX", f"{newjnt}.scaleZ")

        for i, guide in enumerate(self.inputlist, start=1):
                mc.select(clear=True)
                jnt = f'{guide}_IK'

                switchparent = mc.parentConstraint(jnt, f'{guide}_JNT', mo=True)

                fkweight_attrs = mc.parentConstraint(switchparent[0], q=True, wal=True)

                mc.connectAttr(revswitchattr, f'{switchparent[0]}.{fkweight_attrs[1]}')


        for control in controllist:
            mc.addAttr(
            control,
            ln="IK_Fk",
            proxy=switchattr,
        )

        return bindjnts

        