from collections.abc import Sequence
from importlib import reload

import maya.cmds as mc
import rjg.build.chain as rChain
import rjg.build.fk as rFk
import rjg.build.rigModule as rModule
import rjg.libs.attribute as rAttr
import rjg.libs.control.ctrl as rCtrl

reload(rModule)
reload(rAttr)
reload(rChain)
reload(rFk)


def get_world_position(transform: str) -> tuple[float, float, float]:
    return mc.xform(transform, query=True, translation=True, worldSpace=True)


def spline_from_guides(
    name: str,
    guides: Sequence[str],
    parent: str | None = None,
    degree: int = 3,
    rebuild_spans: int | None = None,
    edit_point: bool = True,
) -> str:
    positions: list[tuple[float, float, float]] = [get_world_position(guide) for guide in guides]
    if edit_point:
        curve: str = mc.curve(name=name, editPoint=positions, degree=degree)
    else:
        curve: str = mc.curve(name=name, point=positions, degree=degree)
    if rebuild_spans is not None:
        mc.rebuildCurve(spans=rebuild_spans, keepRange=2, degree=degree)
        mc.delete(curve, constructionHistory=True)
    if parent is not None:
        mc.parent(curve, parent)
    return curve


class SplineTail(rModule.RigModule, rFk.Fk):
    def __init__(
        self,
        side=None,
        part=None,
        guide_list=None,
        ctrl_scale=1,
        model_path=None,
        guide_path=None,
        pad="auto",
        remove_last=True,
        fk_shape="circle",
        ik_spline: str | None = None,
        segments=12,
    ):
        super().__init__(
            side=side,
            part=part,
            guide_list=guide_list,
            ctrl_scale=ctrl_scale,
            model_path=model_path,
            guide_path=guide_path,
        )

        self.__dict__.update(locals())
        self.gimbal = None
        self.offset = None
        self.ik_spline_guide = ik_spline
        self.segments = segments

        if self.pad == "auto":
            self.pad = len(str(len(self.guide_list))) + 1

        self.create_module()

    def build_ik_spline_with_controls(
        self,
        name: str,
        parent: str,
        guides: Sequence[str],
        start_joint: str,
        end_joint: str,
        ctrl_group: str,
        ctrl_prefix: str | None = None,
        curve: str | None = None,
        spans: int = 1,
    ) -> list[rCtrl.Control]:
        spline_group = mc.group(name=f"{name}_spline_GRP", empty=True, parent=parent)
        # Step 1: Create IK spline
        if curve is None:
            ik_curve = spline_from_guides(
                name=f"{name}_curve", guides=guides, rebuild_spans=spans, parent=spline_group
            )
        else:
            ik_curve = mc.duplicate(curve, name=f"{name}_curve")[0]
            mc.parent(ik_curve, spline_group)
        ik_handle, effector = mc.ikHandle(
            startJoint=start_joint,
            endEffector=end_joint,
            solver="ikSplineSolver",
            parentCurve=False,
            curve=ik_curve,
            createCurve=False,
        )

        ik_handle = mc.rename(ik_handle, f"{name}_ik_handle")
        mc.parent(ik_handle, spline_group)
        # Step 2: For each CV on the curve, create cluster + control
        cvs = mc.ls(f"{ik_curve}.cv[*]", fl=True)

        ik_ctrls: list[rCtrl.Control] = []

        for i, cv in enumerate(cvs, start=1):
            # Make cluster for the CV
            cluster, cluster_handle = mc.cluster(cv, n=f"{name}_Cluster_{i:02}")
            mc.parent(cluster_handle, spline_group)
            # Get cluster position
            pos = mc.pointPosition(cv, w=True)

            # Make control
            prefix = ctrl_prefix if ctrl_prefix is not None else name
            ctrl_name = f"{prefix}_IK_{i:02}"
            ik_ctrl = rCtrl.Control(
                name=ctrl_name,
                parent=ctrl_group,
                shape="ZTsphere",
                ctrl_scale=10 * self.ctrl_scale,
                translate=pos,
            )
            ik_ctrls.append(ik_ctrl)

            # Parent cluster to control
            mc.parentConstraint(ik_ctrl.ctrl, cluster_handle, mo=True)

        return ik_ctrls

    def create_module(self):
        super().create_module()

        self.control_rig()
        self.skeleton()
        self.output_rig()
        self.add_plugs()
    
    def create_fk_control_rig(self):
        fk_group = mc.group(empty=True, name=f"{self.base_name}_FK_GRP", parent=self.module_grp)
        fk_ctrl_group = mc.group(empty=True, name=f"{self.base_name}_FK_CTRL_GRP", parent=self.control_grp)
        self.fk_ctrl_group = fk_ctrl_group
        #fk rig and skel
        precontrol: None | rCtrl.Control = None
        lastjnt = None
        
        fk_chain = rChain.Chain(transform_list=self.guide_list, suffix="FK", name=self.part)
        fk_chain.create_from_transforms(static=True, parent=fk_group)
        self.fk_chain = fk_chain
        fk_joints  = fk_chain.joints
        
        self.fk_controls: list[rCtrl.Control] = []
        control_parent = fk_ctrl_group
        for guide, fk_joint in zip(self.guide_list, fk_joints):
            fk_ctrl = rCtrl.Control(name=guide, shape="circle", ctrl_scale=5 * self.ctrl_scale, translate=guide, rotate=guide)
            mc.parent(fk_ctrl.top, control_parent)
            self.fk_controls.append(fk_ctrl)
            mc.parentConstraint(fk_ctrl.ctrl, fk_joint, mo=True)
            control_parent = fk_ctrl.ctrl
    
    def create_compat_ik_control_rig(self):
        ik_group = mc.group(empty=True, name=f"{self.base_name}_compat_IK_GRP", parent=self.module_grp)
        ik_ctrl_group = mc.group(empty=True, name=f"{self.base_name}_compat_IK_CTRL_GRP", parent=self.control_grp)
        self.ik_ctrl_group = ik_ctrl_group
        
        ik_chain = rChain.Chain(transform_list=self.guide_list, suffix="compat_IK", name=self.part)
        ik_chain.create_from_transforms(static=True, parent=ik_group)
        
        ik_joints = ik_chain.joints
        self.compat_ik_chain = ik_chain
        #ik rig
        self.compat_ik_controls = self.build_ik_spline_with_controls(
            name=f"{self.base_name}_compat",
            parent=ik_group,
            guides=self.guide_list,
            start_joint=ik_joints[0],
            end_joint=ik_joints[-1],
            ctrl_group=ik_ctrl_group,
            ctrl_prefix="Tail",
        )
        
    def create_ik_control_rig(self):
        ik_group = mc.group(empty=True, name=f"{self.base_name}_IK_GRP", parent=self.module_grp)
        ik_ctrl_group = mc.group(empty=True, name=f"{self.base_name}_IK_CTRL_GRP", parent=self.control_grp)
        self.ik_ctrl_group = ik_ctrl_group
        
        ik_chain = rChain.Chain(transform_list=self.guide_list, suffix="IK", name=self.part)
        ik_chain.create_from_transforms(static=True, parent=ik_group)
        
        ik_joints = ik_chain.joints
        self.ik_chain = ik_chain
        #ik rig
        self.ik_controls = self.build_ik_spline_with_controls(
            name=self.base_name,
            parent=ik_group,
            guides=self.guide_list,
            start_joint=ik_joints[0],
            end_joint=ik_joints[-1],
            ctrl_group=ik_ctrl_group,
            curve=self.ik_spline_guide,
        )

    
    def control_rig(self):
        self.create_fk_control_rig()
        self.create_compat_ik_control_rig()
        self.create_ik_control_rig()

    def output_rig(self):
        ik_blend_group = mc.group(
            empty=True, name=f"{self.base_name}_IK_Blend_GRP", parent=self.module_grp
        )
        ik_blend_chain = rChain.Chain(
            transform_list=self.guide_list, name=self.part, suffix="IK_Blend"
        )
        ik_blend_chain.create_blend_chain(
            switch_node=ik_blend_group,
            parent=ik_blend_group,
            chain_a=self.compat_ik_chain.joints,
            chain_b=self.ik_chain.joints,
            handle_offsets=True,
        )
        self.ik_blend_chain = ik_blend_chain
        for guide, fk_joint, ik_joint in zip(
            self.guide_list, self.fk_chain.joints, ik_blend_chain.joints
        ):
            mc.parentConstraint(fk_joint, f"{guide}_jnt", mo=True)
            mc.parentConstraint(ik_joint, f"{guide}_jnt", mo=True)

    def skeleton(self):
        lastjnt = None
        bind_joints = []
        for guide in self.guide_list:
            # World position (translation)
            pos = mc.xform(guide, q=True, ws=True, t=True)   # [x, y, z]
            # World rotation (Euler angles, degrees)
            rot = mc.xform(guide, q=True, ws=True, ro=True)  # [rx, ry, rz]
            mc.select(clear=True)
            bindjnt = mc.joint(p=pos, o=rot, name=f'{guide}_jnt')
            bind_joints.append(bindjnt)
            if lastjnt:
                mc.parent(bindjnt, lastjnt)
                lastjnt = bindjnt
            else:
                lastjnt = bindjnt
        split_joint: str = bind_joints[0]
        split_joints: list[str] = bind_joints
        mc.addAttr(split_joint, longName="split_joints", dataType="string")
        mc.setAttr(f'{split_joint}.split_joints', repr(split_joints), type="string")
        self.joints = bind_joints
        self.tag_bind_joints(bind_joints)

    def add_plugs(self):
        if mc.objExists("switch_CTRL"):
            switch = 'switch_CTRL'
        else:
            switch = self.module_grp
        mc.addAttr(switch, longName="Tail_M_IKFK", attributeType="bool", keyable=True, hidden=False )

        rev = mc.shadingNode("reverse", asUtility=True, name="Tail_Switch_Rev")
        mc.connectAttr(f'{switch}.Tail_M_IKFK', f'{rev}.inputX')
        mc.connectAttr(f'{switch}.Tail_M_IKFK',f'{self.fk_ctrl_group}.visibility')
        mc.connectAttr(f'{rev}.outputX',f'{self.ik_ctrl_group}.visibility')

        for joint, fk_joint, ik_joint in zip(self.joints, self.fk_chain.joints, self.ik_blend_chain.joints):
            mc.connectAttr(f'{switch}.Tail_M_IKFK', f"{joint}_parentConstraint1.{fk_joint}W0")
            mc.connectAttr(f'{rev}.outputX', f"{joint}_parentConstraint1.{ik_joint}W1")

        #mc.parentConstraint('waist_M_CTRL', )
        mc.parent('Tail1_jnt', 'COG_M_JNT')

        mc.parentConstraint('waist_M_CTRL', self.fk_controls[0].ctrl, mo=True)

        mc.addAttr(self.compat_ik_controls[0].ctrl, ln="TailSpace", at="enum", en="Waist:Root:World", k=True)
        
        for control in self.compat_ik_controls:
            grp = control.top

            # make one parentConstraint with all drivers
            pc = mc.parentConstraint("waist_M_CTRL", "global_M_CTRL", grp, mo=True)[0]
            weights = mc.parentConstraint(pc, q=True, wal=True)

            # loop through drivers and make condition per driver
            for idx, driver in enumerate(["waist_M_CTRL", "global_M_CTRL"]):
                cond = mc.createNode("condition", n=f"{control.ctrl}_{driver}_cond")
                mc.connectAttr(f"{self.compat_ik_controls[0].ctrl}.TailSpace", f"{cond}.firstTerm")
                mc.setAttr(f"{cond}.secondTerm", idx)       # match enum index
                mc.setAttr(f"{cond}.operation", 0)          # Equal
                mc.setAttr(f"{cond}.colorIfTrueR", 1)
                mc.setAttr(f"{cond}.colorIfFalseR", 0)
                mc.connectAttr(f"{cond}.outColorR", f"{pc}.{weights[idx]}")

        proxylist = [control.ctrl for control in self.compat_ik_controls] + [control.ctrl for control in self.fk_controls]

        for ctrl in proxylist:
            mc.addAttr(ctrl, longName='FK_IK_Switch', proxy=f'{switch}.Tail_M_IKFK')
