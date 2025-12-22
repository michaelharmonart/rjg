import math
import re
from importlib import reload
from tkinter import Scale
from typing import Sequence

import maya.cmds as mc
import rjg.build.chain as rChain
import rjg.build.guide as rGuide
import rjg.libs.attribute as rAttr
import rjg.libs.control.ctrl as rCtrl
import rjg.libs.transform as rXform
from maya.api.OpenMaya import MFnNurbsCurve, MPoint, MSelectionList, MSpace, MVector
from rjg.build.UEface import UEface
from rjg.libs.profile import auto_profiler_tag
from rjg.libs.spline import generate_knots, get_cvs, get_knots
from rjg.libs.spline.math import Vector3

reload(rAttr)
reload(rChain)
reload(rCtrl)
reload(rGuide)
reload(rXform)


def get_curve(node: str) -> str | None:
    if mc.nodeType(node) == "nurbsCurve":
        return node
    else:
        curves = mc.listRelatives(node, children=True, shapes=True, type="nurbsCurve")
        if len(curves) != 0:
            return curves[0]


def get_world_position(transform: str) -> tuple[float, float, float]:
    return mc.xform(transform, query=True, translation=True, worldSpace=True)


def spline_from_guides(
    name: str,
    guides: Sequence[str],
    parent: str | None = None,
    degree: int = 3,
    rebuild_spans: int | None = None,
    edit_point: bool = True,
    display_reference: bool = False,
) -> str:
    positions: list[tuple[float, float, float]] = [get_world_position(guide) for guide in guides]
    if edit_point:
        curve: str = mc.curve(name=name, editPoint=positions, degree=degree)
    else:
        curve: str = mc.curve(name=name, point=positions, degree=degree)
    curve_shape = get_curve(curve)
    curve_shape = mc.rename(curve_shape, f"{curve}Shape")
    if display_reference:
        mc.displaySmoothness(curve_shape, pointsWire=16)
        mc.setAttr(f"{curve_shape}.overrideEnabled", 1)
        mc.setAttr(f"{curve_shape}.overrideDisplayType", 1)
    if rebuild_spans is not None:
        mc.rebuildCurve(spans=rebuild_spans, keepRange=2, degree=degree)
        mc.delete(curve, constructionHistory=True)
    if parent is not None:
        mc.parent(curve, parent)
    return curve


def closest_point_on_curve(curve: str, guide: str, fraction: bool = True) -> float:
    guide_pos = MPoint(get_world_position(guide))

    curve_shape = get_curve(curve)
    sel = MSelectionList()
    sel.add(curve_shape)
    dag_path = sel.getDagPath(0)
    fn_curve: MFnNurbsCurve = MFnNurbsCurve(dag_path)
    parameter: float = fn_curve.closestPoint(guide_pos, space=MSpace.kWorld)[1]
    if fraction:
        length_to_u = fn_curve.findLengthFromParam(parameter)
        total_length = fn_curve.length()
        if total_length == 0.0:
            return 0.0
        return max(min(length_to_u / total_length, 1), 0)
    else:
        return parameter


def create_pin_on_curve(
    name: str, curve: str, guide: str, parent: str, arc_length: bool = True
) -> str:
    curve_shape = get_curve(curve)
    pin: str = mc.spaceLocator(name=name)[0]
    mc.parent(pin, parent, relative=True)

    motion_path = mc.createNode("motionPath", name=f"{name}_motionPathPin")
    mc.setAttr(f"{motion_path}.fractionMode", arc_length)
    mc.connectAttr(f"{curve_shape}.local", f"{motion_path}.geometryPath")
    mc.connectAttr(f"{motion_path}.allCoordinates", f"{pin}.translate")
    mc.connectAttr(f"{motion_path}.rotate", f"{pin}.rotate")

    fraction = closest_point_on_curve(curve_shape, guide, fraction=arc_length)
    mc.setAttr(f"{motion_path}.uValue", fraction)
    return pin


def get_guide_index(guide: str) -> int:
    pattern = r"(?<=_)[0-9]+(?=_)"
    matches = re.findall(pattern, guide)
    if matches:
        guide_id = int(matches[-1])
        return guide_id
    return 0


def lerp_vectors(start_point: MVector, end_point: MVector, alpha: float) -> MVector:
    clamped_alpha = max(min(alpha, 1), 0)
    invert_alpha = 1 - clamped_alpha
    return (start_point * invert_alpha) + (end_point * clamped_alpha)


def create_mid_guides(
    start_guide: str, end_guide: str, mid_num: int, guide_name_prefix: str, parent: str
) -> list[str]:
    start_guide_pos: MVector = MVector(get_world_position(start_guide))
    end_guide_pos: MVector = MVector(get_world_position(end_guide))
    mid_guides: list[str] = []
    for i in range(mid_num):
        num = i + 1
        alpha = num / (mid_num + 1)
        mid_guide_pos = lerp_vectors(start_guide_pos, end_guide_pos, alpha)
        mid_guide = mc.group(name=f"{guide_name_prefix}{num:02d}", empty=True, parent=parent)
        mc.xform(
            mid_guide,
            translation=(mid_guide_pos.x, mid_guide_pos.y, mid_guide_pos.z),
            worldSpace=True,
        )
        mid_guides.append(mid_guide)
    return mid_guides


class Spline:
    def __init__(
        self,
        name: str,
        guides: Sequence[str],
        parent: str,
        build_controls: bool = True,
        control_parent: str | None = None,
        pin_transforms: Sequence[str] | None = None,
        ctrl_scale: float = 1,
        degree: int = 3,
        rebuild: bool = False,
        display_reference=True,
    ) -> None:
        self.name = name
        self.spline = spline_from_guides(
            guides=guides,
            name=f"{name}",
            parent=parent,
            degree=degree,
            rebuild_spans=1 if rebuild else None,
            edit_point=rebuild,
            display_reference=display_reference,
        )
        self.spline_shape = get_curve(self.spline)
        cvs: list[Vector3] = get_cvs(self.spline_shape)
        self.control_list: list[rCtrl.Control] = []
        self.pin_list: list[str] = []
        if pin_transforms:
            self.pin_list = list(pin_transforms)
        for index, cv in enumerate(cvs):
            position = (cv.x, cv.y, cv.z)
            if build_controls:
                ctrl_name = f"{name}_{index:02d}"
                ctrl = rCtrl.Control(
                    name=ctrl_name,
                    shape="ZTsphere",
                    parent=control_parent,
                    side=None,
                    axis="y",
                    group_type="main",
                    rig_type="primary",
                    translate=position,
                    ctrl_scale=ctrl_scale * 0.25,
                )
                if not pin_transforms:
                    ctrl_pin = mc.group(empty=True, name=f"{ctrl_name}_Pin", parent=parent)
                    rXform.matrix_constraint(
                        ctrl.ctrl,
                        ctrl_pin,
                        keep_offset=False,
                        scale=False,
                        rotate=False,
                        shear=False,
                    )
                    self.control_list.append(ctrl)
                    self.pin_list.append(ctrl_pin)

        for index, pin in enumerate(self.pin_list):
            mc.connectAttr(f"{pin}.translate", f"{self.spline_shape}.controlPoints[{index}]")
        pass


class UEwing(UEface):
    def __init__(
        self,
        grp_name: str,
        side: str,
        ctrl_scale=1,
    ):
        super().__init__(part="Wing", grp_name=grp_name, ctrl_scale=ctrl_scale)
        self.grp_name = grp_name
        self.prefix = UEface.get_prefix_from_group(self.grp_name)
        self.side = side
        # group='Wing_L_guides'

    @staticmethod
    def get_namestruc(prefix="Wing_L", rjg=True):
        if rjg == False:
            ctrlname = "CTRL"
            grpname = "GRP"
        else:
            parts = prefix.split("_")  # ["wing", "L"]
            side = parts[-1]  # "L"
            ctrlname = f"{side}_CTRL"
            grpname = f"{side}_CTRL_CNST_GRP"
        return ctrlname, grpname

    def get_guides(self, prefix: str, feather="MainFeather"):
        main_guides = mc.ls(f"{prefix}_{feather}_??_guide")
        valid_guides: list[tuple[str, str, str]] = []
        for guide in main_guides:
            index = get_guide_index(guide)
            if not mc.objExists(f"{prefix}_{feather}_{index:02d}_ee_guide"):
                continue
            mid_guide = f"{prefix}_{feather}_{index:02d}_ee_guide"
            if not mc.objExists(f"{prefix}_{feather}_{index:02d}_aim"):
                continue
            tip_guide = f"{prefix}_{feather}_{index:02d}_aim"
            valid_guides.append((guide, mid_guide, tip_guide))
        return valid_guides

    def build_limb(self):
        prefix = self.prefix
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        mc.select(clear=True)
        self.fk_group = mc.group(em=True, name=f"{prefix}_FK_{grpname}")
        self.ik_group = mc.group(em=True, name=f"{prefix}_IK_{grpname}")

        # bind
        self.limb_bind_joints = []
        pre_jnt = None
        for obj in [
            f"{prefix}_01_guide",
            f"{prefix}_02_guide",
            f"{prefix}_03_guide",
            f"{prefix}_04_guide",
        ]:
            # Get the base name and generate joint name
            base_name = obj.split("|")[-1].replace("_guide", "")
            joint_name = f"{base_name}_bind_jnt"

            # Clear selection before creating the joint to avoid parenting
            mc.select(clear=True)
            joint = mc.joint(name=joint_name)
            self.limb_bind_joints.append(joint)

            # Match translation and rotation in world space
            pos = mc.xform(obj, q=True, ws=True, t=True)
            rot = mc.xform(obj, q=True, ws=True, ro=True)
            mc.xform(joint, ws=True, t=pos)
            mc.xform(joint, ws=True, ro=rot)
            if pre_jnt != None:
                mc.parent(joint_name, pre_jnt)
            pre_jnt = joint_name
        # mc.skinCluster(bind_joints, main_surf)
        pre_jnt = None

        parjnts = ["01", "02", "03", "04"]

        pre_jnt = None
        pre_ctrl = None
        armjnts = []
        armoffsets = []
        armctrls = []
        armcloses = []

        # arm Logic
        FKIKSwitch_pos = mc.xform(f"{prefix}_Close", q=True, ws=True, t=True)
        FKIKSwitch_CTL, FKIKSwitch_GRP = UEface.build_basic_control(
            name=f"{prefix}_FKIKSwitch",
            shape="ZTgear",
            size=5.0,
            color_rgb=(1, 1, 0),
            position=FKIKSwitch_pos,
            rotation=(0, 0, 0),
        )
        mc.addAttr(FKIKSwitch_CTL, longName="FK_IK", attributeType="bool", keyable=True)
        rev_node = mc.createNode("reverse", name=f"{prefix}IKReverse")
        mc.connectAttr(f"{FKIKSwitch_CTL}.FK_IK", f"{rev_node}.inputX")

        # fk
        for guide in [
            f"{prefix}_01_guide",
            f"{prefix}_02_guide",
            f"{prefix}_03_guide",
            f"{prefix}_04_guide",
        ]:
            parts = guide.split("_")  # ["wing", "l", "01", "guide"]
            number = parts[-2]  # second to last = "01", "02", etc.
            print(number)
            jnt, ctrl, ctrl_offset = UEface.Simple_joint_and_Control(
                guide,
                orient=True,
                overwrite=True,
                overwrite_name=f"{prefix}_{number}_FK",
                scale=True,
                check_side=False,
                CTRL_Color=(0, 0, 1),
                CTRL_Size=3,
                JNT_Size=0.5,
                bind=False,
            )
            # mc.addAttr()
            rot = mc.xform(guide, q=True, ws=True, ro=True)
            trans = mc.xform(guide, q=True, ws=True, t=True)
            close_offset = mc.group(empty=True, name=f"{prefix}_{number}_FK_ArmClose_offset")
            mc.xform(close_offset, ws=True, t=trans, ro=rot)
            mc.parent(close_offset, ctrl_offset)
            mc.parent(ctrl, close_offset)
            if pre_jnt != None:
                mc.parent(jnt, pre_jnt)
                mc.parent(ctrl_offset, pre_ctrl)
                pre_jnt = jnt
                pre_ctrl = ctrl
            else:
                pre_jnt = jnt
                pre_ctrl = ctrl
            armjnts.append(jnt)
            armoffsets.append(ctrl_offset)
            armctrls.append(ctrl)
            armcloses.append(close_offset)
        ########################################################## Come back to this
        for num in ["01", "02", "03", "04"]:
            mc.parentConstraint(f"{prefix}_{num}_FK_JNT", f"{prefix}_{num}_bind_jnt", mo=True)

        # ik
        IK_joints = []
        pre_jnt = None
        for obj in [
            f"{prefix}_01_guide",
            f"{prefix}_02_guide",
            f"{prefix}_03_guide",
            f"{prefix}_04_guide",
        ]:
            # Get the base name and generate joint name
            base_name = obj.split("|")[-1].replace("_guide", "")
            joint_name = f"{base_name}_IK_jnt"

            # Clear selection before creating the joint to avoid parenting
            mc.select(clear=True)
            joint = mc.joint(name=joint_name)
            IK_joints.append(joint)

            # Match translation and rotation in world space
            pos = mc.xform(obj, q=True, ws=True, t=True)
            rot = mc.xform(obj, q=True, ws=True, ro=True)
            mc.xform(joint, ws=True, t=pos)
            mc.xform(joint, ws=True, ro=rot)
            if pre_jnt != None:
                mc.parent(joint_name, pre_jnt)
            pre_jnt = joint_name

        pv_pos = mc.xform(f"{prefix}_IK_Aim", q=True, ws=True, t=True)
        ikaimCTL, ikaimGRP = UEface.build_basic_control(
            name=f"{prefix}_IK_Aim",
            shape="locator_3D",
            size=20.0,
            color_rgb=(1, 1, 0),
            position=pv_pos,
            rotation=(0, 0, 0),
        )

        ikhandel = mc.ikHandle(
            name=f"{prefix}_ikHandle",
            sj=f"{prefix}_01_IK_jnt",
            ee=f"{prefix}_03_IK_jnt",
            sol="ikRPsolver",
        )[0]

        mc.poleVectorConstraint(ikaimCTL, ikhandel)
        IK_Root_pos = mc.xform(f"{prefix}_01_guide", q=True, ws=True, t=True)
        IK_Root_CTL, IK_Root_GRP = UEface.build_basic_control(
            name=f"{prefix}_IK_Root",
            shape="circle",
            size=5.0,
            color_rgb=(1, 1, 0),
            position=IK_Root_pos,
            rotation=(0, 0, 0),
        )
        mc.parentConstraint(IK_Root_CTL, f"{prefix}_01_IK_jnt", mo=True)

        IK_EE_pos = mc.xform(f"{prefix}_03_guide", q=True, ws=True, t=True)
        IK_EE_rot = mc.xform(f"{prefix}_03_guide", q=True, ws=True, rotation=True)
        IK_EE_CTL, IK_EE_GRP = UEface.build_basic_control(
            name=f"{prefix}_IK_EE",
            shape="circle",
            size=5.0,
            color_rgb=(1, 1, 0),
            position=IK_EE_pos,
            rotation=IK_EE_rot,
        )
        mc.parentConstraint(IK_EE_CTL, ikhandel, mo=True)

        IK_04_pos = mc.xform(f"{prefix}_04_guide", q=True, ws=True, t=True)
        IK_04_CTL, IK_04_GRP = UEface.build_basic_control(
            name=f"{prefix}_IK_04",
            shape="circle",
            size=5.0,
            color_rgb=(1, 1, 0),
            position=IK_04_pos,
            rotation=(0, 0, 0),
        )
        mc.parentConstraint(IK_04_CTL, f"{prefix}_04_IK_jnt", mo=True)
        mc.parent(IK_04_GRP, IK_EE_CTL)
        for num in ["01", "02", "03", "04"]:
            mc.parentConstraint(f"{prefix}_{num}_IK_jnt", f"{prefix}_{num}_bind_jnt", mo=True)
            mc.connectAttr(
                f"{FKIKSwitch_CTL}.FK_IK",
                f"{prefix}_{num}_bind_jnt_parentConstraint1.{prefix}_{num}_FK_JNTW0",
            )
            mc.connectAttr(
                f"{rev_node}.outputX",
                f"{prefix}_{num}_bind_jnt_parentConstraint1.{prefix}_{num}_IK_jntW1",
            )
        mc.pointConstraint(f"{prefix}_01_bind_jnt", FKIKSwitch_GRP, mo=True)
        mc.orientConstraint(f"{prefix}_IK_EE_{ctrlname}", f"{prefix}_03_IK_jnt", mo=True)

        max_val = 20

        # Clean Up Wing
        mc.group(
            f"{prefix}_01_FK_JNT",
            f"{prefix}_01_IK_jnt",
            f"{prefix}_ikHandle",
            name=f"{prefix}_extraOffset_{grpname}",
        )  # f'{prefix}_Main_loft'
        mc.parent(f"{prefix}_IK_Aim_{grpname}", f"{prefix}_IK_Root_{ctrlname}")
        mc.parent(f"{prefix}_IK_EE_{grpname}", f"{prefix}_IK_Root_{ctrlname}")
        mc.parent(f"{prefix}_IK_Root_{grpname}", self.ik_group)
        mc.connectAttr(f"{FKIKSwitch_CTL}.FK_IK", f"{prefix}_FK_{grpname}.visibility")
        mc.connectAttr(f"{rev_node}.outputX", f"{prefix}_IK_{grpname}.visibility")
        jnt, ctrl, ctrl_offset = UEface.Simple_joint_and_Control(
            f"{prefix}_Scap",
            orient=True,
            overwrite=False,
            scale=True,
            check_side=True,
            CTRL_Size=10,
            JNT_Size=0.5,
        )

        mc.parent(f"{prefix}_FK_{grpname}", f"{prefix}_IK_{grpname}", ctrl)
        mc.parent(
            f"{prefix}_FKIKSwitch_{grpname}",
            f"{prefix}_extraOffset_{grpname}",
            ctrl_offset,
            self.mastergrp,
        )  # f'{prefix}_upAim_{grpname}'f'{prefix}_Span_{grpname}'f'{prefix}_aimcurve_{grpname}'
        mc.parent(f"{prefix}_01_bind_jnt", jnt)  # f'{prefix}_root_jnt'
        mc.parent(jnt, "chest_M_JNT")
        mc.parentConstraint("chest_M_02_CTRL", ctrl_offset, mo=True)
        mc.hide(f"{prefix}_extraOffset_{grpname}")
        mc.parent(self.mastergrp, "RIG")

    def build_feathers(self, keep_spacing: bool = True):
        prefix = self.prefix
        self.feather_grp = mc.group(em=True, name=f"{self.prefix}_feather", parent=self.mastergrp)
        self.spline_grp = mc.group(em=True, name=f"{self.prefix}_spline", parent=self.mastergrp)
        self.net_grp = mc.group(em=True, name=f"{self.prefix}_net", parent=self.mastergrp)
        mc.hide(self.spline_grp)
        feather = "MainFeather"
        guides = self.get_guides(prefix=prefix, feather=feather)
        root_list = [guide[0] for guide in guides]
        mid_list = [guide[1] for guide in guides]
        aim_list = [guide[2] for guide in guides]
        mainguides = root_list

        # Feathershaping
        root_spline = Spline(
            guides=self.limb_bind_joints,
            name=f"{prefix}_Root_Spline",
            parent=self.spline_grp,
            control_parent=self.feather_grp,
            ctrl_scale=self.ctrl_scale,
            rebuild=False,
            degree=1
        )
        mid_spline = Spline(
            guides=mid_list,
            name=f"{prefix}_Mid_Spline",
            parent=self.spline_grp,
            control_parent=self.feather_grp,
            ctrl_scale=self.ctrl_scale,
            rebuild=True,
        )
        tip_spline = Spline(
            guides=aim_list,
            name=f"{prefix}_Tip_Spline",
            parent=self.spline_grp,
            control_parent=self.feather_grp,
            ctrl_scale=self.ctrl_scale,
            rebuild=True,
        )
        mc.parent(root_spline.spline, mid_spline.spline, tip_spline.spline, self.net_grp)

        #main_shape = root_spline.spline_shape
        #mid_shape = mid_spline.spline_shape
        #aim_shape = tip_spline.spline_shape
        #positions = get_cvs(main_shape) + get_cvs(mid_shape) + get_cvs(aim_shape)
        #knots_v = get_knots(mid_shape)[1:-1]
        #knots_u = generate_knots(3, degree=2)[1:-1]
        #surface = mc.surface(
        #    name=f"{prefix}_Surface",
        #    point=[(position.x, position.y, position.z) for position in positions],
        #    knotU=knots_u,
        #    knotV=knots_v,
        #    degreeU=2,
        #)
        #surface_transform = mc.listRelatives(surface, parent=True)[0]
        #for index, cluster in enumerate(
        #    root_spline.pin_list + mid_spline.pin_list + tip_spline.pin_list
        #):
        #    mc.connectAttr(f"{cluster}.translate", f"{surface}.controlPoints[{index}]")
        #mc.parent(surface_transform, self.spline_grp)

        # Build Feather :)
        def_jnts = []
        for index, (root_guide, mid_guide, tip_guide) in enumerate(
            zip(root_list, mid_list, aim_list), start=1
        ):
            name = root_guide.replace("guide", "Spline")
            root_pin = create_pin_on_curve(
                name=f"{root_guide}_Pin",
                curve=root_spline.spline,
                parent=self.spline_grp,
                guide=root_guide,
                arc_length=keep_spacing,
            )
            mid_pin = create_pin_on_curve(
                name=f"{mid_guide}_Pin",
                curve=mid_spline.spline,
                parent=self.spline_grp,
                guide=mid_guide,
                arc_length=keep_spacing,
            )
            tip_pin = create_pin_on_curve(
                name=f"{tip_guide}_Pin",
                curve=tip_spline.spline,
                parent=self.spline_grp,
                guide=tip_guide,
                arc_length=keep_spacing,
            )
            feather_spline = Spline(
                name=name,
                guides=[root_pin, mid_pin, tip_pin],
                parent=self.spline_grp,
                control_parent=self.feather_grp,
                build_controls=False,
                pin_transforms=[root_pin, mid_pin, tip_pin],
                degree=2,
            )
            mc.parent(feather_spline.spline, self.net_grp)
            
            parent = mc.listRelatives(root_guide, parent=True)[0]
            mid_guides = create_mid_guides(
                root_guide, mid_guide, 2, f"{prefix}_{feather}_mid_guide_", parent=parent
            )
            guide_mapping = {
                root_guide: f"{prefix}{feather}_{index:02d}_base_JNT",
                mid_guides[0]: f"{prefix}{feather}_{index:02d}_mid1_JNT",
                mid_guides[1]: f"{prefix}{feather}_{index:02d}_mid2_JNT",
                mid_guide: f"{prefix}{feather}_{index:02d}_ee_JNT",
            }
            
            joint_parent = self.spline_grp
            if mc.attributeQuery('parent_joint', node=root_guide, exists=True):
                joint_parent_index = mc.getAttr(f'{root_guide}.parent_joint')
                joint_parent = self.limb_bind_joints[joint_parent_index]
            split_joints: list[str] = []    
            for guide in [root_guide] + mid_guides + [mid_guide]:
                if guide in guide_mapping:
                    joint_name = guide_mapping[guide]
                else:
                    joint_name = guide
                joint = mc.joint(name=joint_name)
                UEface.add_to_face_bind_set(joint)
                split_joints.append(joint)
                pin = create_pin_on_curve(
                    name=f"{joint}_Pin",
                    curve=feather_spline.spline,
                    guide=guide,
                    parent=self.spline_grp,
                )
                mc.parent(joint, joint_parent, relative=True)
                rXform.matrix_constraint(pin, joint, keep_offset=False)
                joint_parent = joint
            
            split_joint = split_joints[0]
            mc.addAttr(split_joint, longName="split_joints", dataType="string")
            mc.setAttr(
                f'{split_joint}.split_joints',
                repr(split_joints),
                type="string"
            )
        
        for i, bind_jnt in enumerate(self.limb_bind_joints):
            main = root_spline.control_list[i]
            mid = mid_spline.control_list[i]
            aim = tip_spline.control_list[i]
            mc.parentConstraint(bind_jnt, main.top, mo=True)
            mc.parentConstraint(bind_jnt, mid.top, mo=True)
            mc.parentConstraint(bind_jnt, aim.top, mo=True)

    @auto_profiler_tag
    def build_wing(self):
        prefix = self.prefix
        grp = self.grp_name
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        parts = prefix.split("_")  # ["wing", "L"]
        side = parts[-1]
        self.mastergrp = mc.group(em=True, name=f"{prefix}")
        self.build_limb()
        self.build_feathers()
