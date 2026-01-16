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
        IK_Spline=True,
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
        self.IK_Spline = IK_Spline
        self.segments = segments

        if self.pad == "auto":
            self.pad = len(str(len(self.guide_list))) + 1

        self.create_module()


    def build_ik_spline_with_controls(self, guides: Sequence[str], aim_joints=None, prefix=None, sub=False):
        ctrlname, grpname = 'M_CTRL', 'M_CTRL_CNST_GRP'
        spline_group = mc.group(name = f'{prefix}_handle_{grpname}', empty=True )
        # Step 1: Create IK spline
        ik_curve = spline_from_guides(name=f'{prefix}_curve', guides=guides, rebuild_spans=1, parent=spline_group)
        ik_handle, effector = mc.ikHandle(
            startJoint=aim_joints[0],
            endEffector=aim_joints[-1],
            solver='ikSplineSolver',
            parentCurve=False,
            curve=ik_curve,
            createCurve=False,
        )
        
        ik_handle = mc.rename(ik_handle, f'{prefix}_ik_handle')
        mc.parent(ik_handle, spline_group)
        # Step 2: For each CV on the curve, create cluster + control
        cvs = mc.ls(f"{ik_curve}.cv[*]", fl=True)
        
        ik_ctrls = []
        ik_offsets = []
        ikgroup = mc.group(name = f'{prefix}_ikcontrl_{grpname}', empty=True )

        for i, cv in enumerate(cvs, start=1):
            # Make cluster for the CV
            cluster, cluster_handle = mc.cluster(cv, n=f"{prefix}_Cluster_{i:02}")
            mc.parent(cluster_handle, f'{prefix}_handle_{grpname}')
            # Get cluster position
            pos = mc.pointPosition(cv, w=True)

            if sub == False:
                # Make control
                ctrl_name = f"{prefix}_IK_{i:02}"
                ik_ctrl = rCtrl.Control(name=ctrl_name, parent=ikgroup, shape="ZTsphere", ctrl_scale=10 * self.ctrl_scale, translate=pos)
                
                ik_ctrls.append(ik_ctrl.ctrl)
                ik_offsets.append(ik_ctrl.top)
                mc.parent(ik_ctrl.top, ikgroup)

                # Parent cluster to control
                mc.parentConstraint(ik_ctrl.ctrl, cluster_handle, mo=True)
                mc.hide(ik_handle,ik_curve,f'{prefix}_handle_{grpname}')
                try:
                    mc.parent(f'{prefix}_handle_{grpname}', ikgroup)
                except:
                    pass

        return ik_handle, ik_curve, ik_ctrls, ik_offsets, ikgroup

    def create_module(self):
        super().create_module()

        self.control_rig()
        self.skeleton()
        self.output_rig()
        self.add_plugs()

    def control_rig(self):
        mc.group(empty=True, name='Tail_FK_GRP')
        mc.group(empty=True, name='Tail_IK_GRP')
    
        #fk rig and skel
        precontrol: None | rCtrl.Control = None
        ik_joints = []
        lastjnt = None
        last_ik_jnt = None
        for guide in self.guide_list:
            # World position (translation)
            pos = mc.xform(guide, q=True, ws=True, t=True)   # [x, y, z]
            # World rotation (Euler angles, degrees)
            rot = mc.xform(guide, q=True, ws=True, ro=True)  # [rx, ry, rz]
            
            fk_ctrl = rCtrl.Control(name=guide, shape="circle", ctrl_scale=5 * self.ctrl_scale, translate=guide, rotate=guide)
            
            if precontrol is not None:
                mc.parent(fk_ctrl.top, precontrol.ctrl)
                precontrol = fk_ctrl
            else:
                precontrol = fk_ctrl
                mc.parent(fk_ctrl.top, 'Tail_FK_GRP')
            mc.select(clear=True)
            fk_joint = mc.joint(p=pos, o=rot, name=f'{guide}_FK')
            if lastjnt:
                mc.parent(fk_joint, lastjnt)
                lastjnt = fk_joint
            else:
                lastjnt = fk_joint
            mc.parentConstraint(fk_ctrl.ctrl, fk_joint, mo=True)
            mc.select(clear=True)
            ik_joint = mc.joint(p=pos, o=rot, name=f'{guide}_IK')
            if last_ik_jnt:
                mc.parent(ik_joint, last_ik_jnt)
                last_ik_jnt = ik_joint
            else:
                last_ik_jnt = ik_joint
            ik_joints.append(ik_joint)
            
        #ik rig
        ikstuffs = self.build_ik_spline_with_controls(guides=self.guide_list, aim_joints=ik_joints, prefix="Tail", sub=False)
        mc.parent(ikstuffs[4],'Tail_IK_GRP')
        mc.parent('Tail1_FK', 'Tail_FK_GRP')
        mc.hide('Tail1_FK', 'Tail1_IK')
        mc.parent('Tail1_IK', 'Tail_IK_GRP')


    def output_rig(self):
        for guide in self.guide_list:
            mc.parentConstraint(f'{guide}_FK', f'{guide}_jnt', mo=True)
            mc.parentConstraint(f'{guide}_IK', f'{guide}_jnt', mo=True)

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

        self.tag_bind_joints(bind_joints)

    def add_plugs(self):
        mc.group(empty=True, name='Tail_M')
        
        if mc.objExists("switch_CTRL"):
            switch = 'switch_CTRL'
        else:
            switch = 'Tail_M'
        mc.addAttr(switch, longName="Tail_M_IKFK", attributeType="bool", keyable=True, hidden=False )

        rev = mc.shadingNode("reverse", asUtility=True, name="Tail_Switch_Rev")
        mc.connectAttr(f'{switch}.Tail_M_IKFK', f'{rev}.inputX')
        mc.connectAttr(f'{switch}.Tail_M_IKFK','Tail_FK_GRP.visibility')
        mc.connectAttr(f'{rev}.outputX','Tail_IK_GRP.visibility')

        for guide in self.guide_list:
            mc.connectAttr(f'{switch}.Tail_M_IKFK', f"{guide}_jnt_parentConstraint1.{guide}_FKW0")
            mc.connectAttr(f'{rev}.outputX', f"{guide}_jnt_parentConstraint1.{guide}_IKW1")

        #mc.parentConstraint('waist_M_CTRL', )
        mc.parent('Tail1_jnt', 'COG_M_JNT')
        mc.parent('Tail_FK_GRP', 'Tail_IK_GRP', 'Tail_M')
        mc.parent('Tail_M', 'RIG')

        mc.parentConstraint('waist_M_CTRL', 'Tail1_M_CTRL_CNST_GRP', mo=True)

        mc.addAttr("Tail_IK_01_M_CTRL", ln="TailSpace", at="enum", en="Waist:Root:World", k=True)

        for num in ["01", "02", "03", "04"]:
            grp = f"Tail_IK_{num}_M_CTRL_CNST_GRP"

            # make one parentConstraint with all drivers
            pc = mc.parentConstraint("waist_M_CTRL", "global_M_CTRL", grp, mo=True)[0]
            weights = mc.parentConstraint(pc, q=True, wal=True)

            # loop through drivers and make condition per driver
            for idx, driver in enumerate(["waist_M_CTRL", "global_M_CTRL"]):
                cond = mc.createNode("condition", n=f"TailCond_{num}_{driver}")
                mc.connectAttr("Tail_IK_01_M_CTRL.TailSpace", f"{cond}.firstTerm")
                mc.setAttr(f"{cond}.secondTerm", idx)       # match enum index
                mc.setAttr(f"{cond}.operation", 0)          # Equal
                mc.setAttr(f"{cond}.colorIfTrueR", 1)
                mc.setAttr(f"{cond}.colorIfFalseR", 0)
                mc.connectAttr(f"{cond}.outColorR", f"{pc}.{weights[idx]}")

        proxylist = ['Tail_IK_01_M_CTRL', 'Tail_IK_02_M_CTRL', 'Tail_IK_03_M_CTRL', 'Tail_IK_04_M_CTRL']
        for i in range(1, self.segments, 1):
            proxylist.append(f'Tail{i}_M_CTRL')
        for ctrl in proxylist:
            mc.addAttr(ctrl, longName='FK_IK_Switch', proxy='Tail_M.Tail_M_IKFK')
