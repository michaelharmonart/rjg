from importlib import reload

from maya.api.OpenMaya import MPoint, MVector
import maya.cmds as mc
from rjg.libs.transform import create_aim_matrix, get_world_matrix, set_world_matrix
import rjg.build.chain as rChain
import rjg.build.guide as rGuide
import rjg.libs.attribute as rAttr
import rjg.libs.control.ctrl as rCtrl

reload(rAttr)
reload(rChain)
reload(rChain)
reload(rGuide)

class Ik:
    def __init__(self, side=None, part=None, guide_list=None, ctrl_scale=1, sticky=None, solver=None, pv_guide='auto', offset_pv=0, slide_pv=None, stretchy=None):
        self.side = side
        self.part = part
        self.base_name = part + '_' + side

        self.guide_list=guide_list
        self.ctrl_scale = ctrl_scale
        self.sticky = sticky
        self.solver = solver
        self.pv_guide = pv_guide
        self.offset_pv = offset_pv
        self.slide_pv = slide_pv
        self.stretchy = stretchy

        self.check_solvers()
        self.check_pv_guide()

        if self.guide_list:
            if not isinstance(self.guide_list, list):
                self.guide_list = [self.guide_list]

    def build_ik(self):
        self.build_ik_controls()
        self.build_ik_chain()
        self.build_ikh()

    def check_solvers(self):
        if not self.sticky:
            self.sticky = 'sticky'
        if not self.solver:
            self.solver = 'ikRPsolver'

        if self.solver == 'ikRPsolver':
            self.s_name = 'RP'
        elif self.solver == 'ikSCsolver':
            self.s_name = 'SC'
            self.pv_guide = False
        elif self.solver == 'ikSplineSolver':
            self.s_name = 'spline'
        elif self.solver == 'ikSpringSolver':
            self.s_name = 'spring'
        else:
            mc.error("Invalid solver specified.")

    def check_pv_guide(self, guide_list: list[str] | None = None):
        if guide_list is None:
            used_guides = self.guide_list
        else:
            used_guides = guide_list
        if self.pv_guide == "auto":
            self.pv_guide = rGuide.create_pv_guide(
                guide_list=used_guides,
                name=self.base_name,
                slide_pv=self.slide_pv,
                offset_pv=self.offset_pv,
                delete_setup=True,
            )
            # self.pv_guide = rGuide.clean_pv_guide(guide_list=self.guide_list, name=self.base_name, offset_pv=self.offset_pv)
        if self.pv_guide == "smart_auto":
             self.pv_guide = rGuide.create_pv_guide(
                guide_list=used_guides,
                name=self.base_name,
                slide_pv=self.slide_pv,
                offset_pv=self.offset_pv,
                delete_setup=True,
                smart_two_segment=True,
            )

    def build_ik_controls(self, guide_list: list[str] | None = None):
        if guide_list is None:
            used_guides = self.guide_list
        else:
            used_guides = guide_list
        
        self.ik_ctrls: list[rCtrl.Control] = []
        attr_util = rAttr.Attribute(add=False)
        self.ik_ctrl_grp = mc.group(empty=True, name=self.base_name + "_IK_CTRL_GRP")
        self.base_ctrl = rCtrl.Control(parent=self.ik_ctrl_grp, shape='cube', side=None, suffix='CTRL', name=self.base_name +"_IK_BASE", axis='y', group_type='main', rig_type='primary', translate=used_guides[0], ctrl_scale=self.ctrl_scale)
        self.ik_ctrls.append(self.base_ctrl)
        attr_util.lock_and_hide(node=self.base_ctrl.ctrl, translate=False, rotate=False)
        self.base_ctrl.tag_as_controller()

        self.main_ctrl = rCtrl.Control(parent=self.ik_ctrl_grp, shape='cube', side=None, suffix='CTRL', name=self.base_name +"_IK_MAIN", axis='y', group_type='main', rig_type='primary', translate=used_guides[-1], ctrl_scale=self.ctrl_scale)
        self.ik_ctrls.append(self.main_ctrl)
        attr_util.lock_and_hide(node=self.main_ctrl.ctrl, translate=False, rotate=False)
        self.main_ctrl.tag_as_controller()

        if self.pv_guide:
            self.check_pv_guide()
            self.pv_ctrl = rCtrl.Control(parent=self.ik_ctrl_grp, shape='locator_3D', side=None, suffix='CTRL', name=self.base_name +"_IK_PV", axis='y', group_type='main', rig_type='pv', translate=self.pv_guide, ctrl_scale=self.ctrl_scale)
            self.ik_ctrls.append(self.pv_ctrl)
            attr_util.lock_and_hide(node=self.pv_ctrl.ctrl, translate=False)
            self.pv_ctrl.tag_as_controller()

        return self.pv_ctrl.ctrl
    
    def build_auto_pv_driver(self, parent:str):
        ik_start_pos = MVector(
            mc.xform(self.base_ctrl.ctrl, query=True, worldSpace=True, translation=True)
        )
        ik_end_pos = MVector(
            mc.xform(self.main_ctrl.ctrl, query=True, worldSpace=True, translation=True)
        )
        pv_pos = MVector(
            mc.xform(self.pv_ctrl.ctrl, query=True, worldSpace=True, translation=True)
        )
        middle_pos: MVector = (ik_start_pos + ik_end_pos) * 0.5
        
        base_aim_vector: MVector = ik_start_pos - ik_end_pos
        base_up_vector: MVector = pv_pos - middle_pos
        
        aim_up_matrix = create_aim_matrix(base_aim_vector, base_up_vector, position=MPoint(ik_end_pos))
        
        end_auto_pv_group = mc.group(empty=True, name=f"{self.base_name}_End_AutoPV_GRP", parent=parent)
        set_world_matrix(end_auto_pv_group, aim_up_matrix)
        mc.orientConstraint(self.main_ctrl.ctrl, end_auto_pv_group, maintainOffset=True)
        
        end_auto_pv_driver = mc.group(empty=True, name=f"{self.base_name}_End_AutoPV_Driver", parent=end_auto_pv_group)
        aim_const = mc.aimConstraint(self.base_ctrl.ctrl, end_auto_pv_driver)[0]
        
        mc.setAttr(f"{aim_const}.aimVector", 0,1,0)
        mc.setAttr(f"{aim_const}.upVector", 0,0,0)
        mc.setAttr(f"{aim_const}.worldUpType", 0) # No up vector: swing decompositon
        
        self.auto_pv_driver = end_auto_pv_driver
        
        return end_auto_pv_driver
    
    def build_ikspline_controls(self):
        """
        Build controls for IK spline setup.
        Creates start / mid / end controls, no PV.
        """

        self.ikspline_ctrls: list[rCtrl.Control] = []
        attr_util = rAttr.Attribute(add=False)

        # control group
        self.ik_ctrl_grp = mc.group(
            empty=True,
            name=self.base_name + "_IK_CTRL_GRP"
        )

        # ---------- START CTRL ----------
        self.start_ctrl = rCtrl.Control(
            parent=self.ik_ctrl_grp,
            shape='cube',
            side=None,
            suffix='CTRL',
            name=self.base_name + "_IKSPLINE_START",
            axis='y',
            group_type='main',
            rig_type='primary',
            translate=self.guide_list[0],
            ctrl_scale=self.ctrl_scale
        )

        self.ikspline_ctrls.append(self.start_ctrl)
        attr_util.lock_and_hide(
            node=self.start_ctrl.ctrl,
            translate=False,
            rotate=False
        )
        self.start_ctrl.tag_as_controller()

        # ---------- MID CTRL (optional) ----------
        if len(self.guide_list) > 2:
            mid_index = len(self.guide_list) // 2
            self.mid_ctrl = rCtrl.Control(
                parent=self.ik_ctrl_grp,
                shape='cube',
                side=None,
                suffix='CTRL',
                name=self.base_name + "_IKSPLINE_MID",
                axis='y',
                group_type='main',
                rig_type='secondary',
                translate=self.guide_list[mid_index],
                ctrl_scale=self.ctrl_scale
            )

            self.ikspline_ctrls.append(self.mid_ctrl)
            attr_util.lock_and_hide(
                node=self.mid_ctrl.ctrl,
                translate=False,
                rotate=False
            )
            self.mid_ctrl.tag_as_controller()

        # ---------- END CTRL ----------
        self.end_ctrl = rCtrl.Control(
            parent=self.ik_ctrl_grp,
            shape='cube',
            side=None,
            suffix='CTRL',
            name=self.base_name + "_IKSPLINE_END",
            axis='y',
            group_type='main',
            rig_type='primary',
            translate=self.guide_list[-1],
            ctrl_scale=self.ctrl_scale
        )

        self.ikspline_ctrls.append(self.end_ctrl)
        attr_util.lock_and_hide(
            node=self.end_ctrl.ctrl,
            translate=False,
            rotate=False
        )
        self.end_ctrl.tag_as_controller()

        return [ctrl.ctrl for ctrl in self.ikspline_ctrls]


    def build_ik_chain(self, force_planar: bool = False, guide_list: list[str] | None = None):
        if guide_list is None:
            used_guides = self.guide_list
        else:
            used_guides = guide_list
        self.ik_chain = rChain.Chain(transform_list=used_guides, side=self.side, suffix=self.s_name + '_JNT', name=self.part)
        self.ik_chain.create_from_transforms(static=True, force_planar=force_planar)
        self.ik_joints = self.ik_chain.joints

    def build_ikspline_chain(self):
        """
        Builds a joint chain for IK spline deformation.
        """
        self.ikspline_chain = rChain.Chain(
            transform_list=self.guide_list,
            side=self.side,
            suffix=self.s_name + '_JNT',
            name=self.part
        )

        self.ikspline_chain.create_from_transforms(static=True)
        self.ikspline_joints = self.ikspline_chain.joints

    def build_ikh(self, scale_attr=None, constrain=True):
        self.ikh = mc.ikHandle(name=self.base_name + "_IKH", startJoint=self.ik_joints[0], endEffector=self.ik_joints[-1], sticky=self.sticky, solver=self.solver)[0]

        if constrain:
            mc.parentConstraint(self.base_ctrl.ctrl, self.ik_joints[0], mo=True)
            mc.parentConstraint(self.main_ctrl.ctrl, self.ikh, mo=True)
            orient_const = mc.orientConstraint(self.main_ctrl.ctrl, self.ik_joints[-1], maintainOffset=True)[0]
            weight_names = mc.orientConstraint(orient_const, query=True, weightAliasList=True)
            mc.connectAttr(f"{self.ikh}.ikBlend", f"{orient_const}.{weight_names[0]}")

        if self.pv_guide:
            mc.poleVectorConstraint(self.pv_ctrl.ctrl, self.ikh)
            guide = rGuide.create_line_guide(a=self.pv_ctrl.ctrl, b=self.ik_joints[1], name=self.base_name)
            self.guide_group = mc.group(guide['curve'], guide['clusters'], parent=self.ik_ctrl_grp, name=self.base_name+"_GUIDE_GRP")
            mc.setAttr(guide['curve'] + ".inheritsTransform", 0)

        if self.stretchy:
            if not scale_attr:
                scale_attr = rAttr.Attribute(node=self.base_ctrl.ctrl, type='double', value=1, keyable=True, name='globalScale')

            self.squash_switch = rAttr.Attribute(node=self.main_ctrl.ctrl, type='double', value=0, keyable=True, name='squash', max=1, min=0)
            self.stretch_switch = rAttr.Attribute(node=self.main_ctrl.ctrl, type='double', value=0, keyable=True, name='stretch', max=1, min=0)

            dist = mc.createNode('distanceBetween', name=self.base_name + "_stretch_DIST")
            mdn = mc.createNode('multiplyDivide', name=self.base_name + "_stretch_MDN")
            mdl = mc.createNode('multDL', name=self.base_name + "_stretch_MDL")
            stretch_cond = mc.createNode('condition', name=self.base_name + "_stretch_COND")
            squash_cond = mc.createNode('condition', name=self.base_name + "_squash_COND")
            stretch_bta = mc.createNode('blendTwoAttr', name=self.base_name + "_stretch_switch_BTA")
            squash_bta = mc.createNode('blendTwoAttr', name=self.base_name + '_squash_switch_BTA')

            # connect ik controls to drive distance calculation
            mc.connectAttr(self.base_ctrl.ctrl + '.worldMatrix[0]', dist + '.inMatrix1')
            mc.connectAttr(self.main_ctrl.ctrl + '.worldMatrix[0]', dist + '.inMatrix2')

            # connect global scale attr to MDL in order to normalize scale
            mc.connectAttr(scale_attr.attr, mdl + '.input1')
            mc.setAttr(mdl + '.input2', self.ik_chain.chain_length)

            # connect dist and mdn/mdl
            mc.connectAttr(dist + '.distance', mdn + '.input1X')
            mc.connectAttr(mdl + '.output', mdn + '.input2X')
            mc.setAttr(mdn + '.operation', 2)

            # condition: if (start/end len >= total len) stretch
            mc.connectAttr(dist + '.distance', stretch_cond + '.firstTerm')
            mc.connectAttr(mdl + '.output', stretch_cond + '.secondTerm')
            mc.connectAttr(mdn + '.outputX', stretch_cond + '.colorIfTrueR')
            mc.setAttr(stretch_cond + '.operation', 3) #3

            # condition: if (start/end len < total len) squash
            mc.connectAttr(dist + '.distance', squash_cond + '.firstTerm')
            mc.connectAttr(mdl + '.output', squash_cond + '.secondTerm')
            mc.connectAttr(mdn + '.outputX', squash_cond + '.colorIfTrueR')
            mc.setAttr(squash_cond + '.operation', 5)

            # connect stretch condition output to stretch_bta blend value
            mc.setAttr(stretch_bta + '.input[0]', 1)
            mc.connectAttr(stretch_cond + '.outColorR', stretch_bta + '.input[1]')
            mc.connectAttr(self.stretch_switch.attr, stretch_bta + '.attributesBlender')

            # connect squash condition output to squash_bta blend value
            mc.setAttr(squash_bta + '.input[0]', 1)
            mc.connectAttr(squash_cond + '.outColorR', squash_bta + '.input[1]')
            mc.connectAttr(self.squash_switch.attr, squash_bta + '.attributesBlender')

            # multiply both squash and stretch values
            mult = mc.createNode('multiplyDivide', name=self.base_name + '_squash_stretch_MDN')
            mc.connectAttr(stretch_bta + '.output', mult + '.input1X')
            mc.connectAttr(squash_bta + '.output', mult + '.input2X')

            # scale each joint's y accordingly
            # for joint in self.ik_joints[:-1]:
            #     mc.connectAttr(stretch_bta + '.output', joint + '.scaleY')
            for joint in self.ik_joints[:-1]:
                mc.connectAttr(mult + '.outputX', joint + '.scaleY')

    def build_spline_ikh(self):
        """
        Creates IK spline handle and curve.
        """
        self.ikh, self.ikeff, self.spline_curve = mc.ikHandle(
            name=self.base_name + "_IKSPLINE_IKH",
            startJoint=self.ikspline_joints[0],
            endEffector=self.ikspline_joints[-1],
            solver='ikSplineSolver',
            createCurve=True,
            parentCurve=False
        )

        mc.parent(self.ikh, self.spline_curve, 'neck_M_MODULE')

        #mc.rename(self.spline_curve, self.base_name + "_IKSPLINE_CRV")
        buildControls = True
        cvs = mc.ls(f"{self.spline_curve }.cv[*]", fl=True)
        ctrl_list = []
        offset_list = []
        self.iklist = []

        for i, cv in enumerate(cvs, start=1):
            # Make cluster for the CV
            #ctrl_list = []
            cluster_list = []
            cluster, cluster_handle = mc.cluster(cv, n=f"{self.base_name}_{i:02}_cluster")
            mc.parent(cluster_handle, f'neck_M_MODULE') 
            # Get cluster position
            pos = mc.pointPosition(cv, w=True)
            if buildControls:
                    # Make control
                    ctrl_name = f"{self.base_name}_{i:02}"
                    #ctrl_name = f"{prefix}_Main_Feather_aim_{i:02}"
                    self.ik_ctrl = rCtrl.Control(parent=self.control_grp, shape="square", side=None, suffix='CTRL', name=f'{self.base_name}_{i:02}', axis='y', group_type='main', rig_type='primary', translate=pos,)
                    ctrl_list.append(self.ik_ctrl.ctrl)
                    offset_list.append(self.ik_ctrl.top)
                    self.iklist.append(self.ik_ctrl)
                    mc.parent(self.ik_ctrl.top, f'neck_M_IK_CTRL_GRP')

                    # Parent cluster to control
                    mc.parentConstraint(self.ik_ctrl.ctrl, cluster_handle, mo=True)


        mc.addAttr(ctrl_list[-1], longName='Stretchy', at='double', dv=1, k=True, max=1, min=0 )
        Stretch_attr = f'{ctrl_list[-1]}.Stretchy'

        mc.addAttr(ctrl_list[-1], ln='twist', at='double', k=True)
        mc.addAttr(ctrl_list[-1], ln='roll', at='double', k=True)
        mc.connectAttr(f'{ctrl_list[-1]}.roll', f'{self.ikh}.roll')
        mc.connectAttr(f'{ctrl_list[-1]}.twist', f'{self.ikh}.twist')

        if Stretch_attr:
            postcurve = mc.listRelatives(self.spline_curve , s=True, ni=True)[0]
            precurve = mc.listRelatives(self.spline_curve , s=True, ni=False, type='nurbsCurve')
            precurve_shapes = mc.listRelatives(self.spline_curve, s=True, ni=False, type='nurbsCurve')
            if not precurve_shapes:
                raise RuntimeError(f"No curve shapes found under {self.spline_curve}")
            precurve = precurve_shapes[0]

            postci = mc.createNode("curveInfo", name=f"{self.spline_curve }_postci")
            preci = mc.createNode("curveInfo", name=f"{self.spline_curve }_preci")
            mc.connectAttr(f"{postcurve}.worldSpace[0]", f"{postci}.inputCurve", force=True)
            mc.connectAttr(f"{precurve}.worldSpace[0]", f"{preci}.inputCurve", force=True)
            value = mc.getAttr(f"{postci}.arcLength")

            frac = mc.createNode("multiplyDivide", name=f"{self.spline_curve }_Frac")

            # Set the operation to DIVIDE (2)
            mc.setAttr(f"{frac}.operation", 2)

            # Connect inputs
            mc.connectAttr(f"{postci}.arcLength", f"{frac}.input1X", force=True)
            #mc.connectAttr(f"{preci}.arcLength", f"{frac}.input2X", force=True)
            mc.setAttr(f"{frac}.input2X", value) 

            md = mc.createNode("remapValue", name=f"{self.spline_curve }_remap")

            mc.connectAttr(f"{frac}.outputX", f"{md}.outputMax", force=True)
            mc.connectAttr(Stretch_attr, f"{md}.inputValue", force=True)
            mc.setAttr(f"{md}.outputMin", 1)

            for newjnt in self.ikspline_chain.joints:
                mc.connectAttr(f"{md}.outValue", f"{newjnt}.scaleY")

            


    def attach_controls_to_spline(self):
        """
        Creates clusters on spline CVs and parents them under controls.
        """
        cvs = mc.ls(self.spline_curve + '.cv[*]', fl=True)

        self.spline_clusters = []

        for i, ctrl in enumerate(self.ikspline_ctrls):
            clust = mc.cluster(cvs[i], name=f'{ctrl.name}_CLUSTER')[1]
            mc.parentConstraint( ctrl.ctrl, clust, mo=True)
            mc.hide(clust)
            mc.parent(clust, self.limb_grp)
            self.spline_clusters.append(clust)
