
from turtle import clear
import maya.cmds as mc
from importlib import reload

import rjg.build.rigModule as rModule
import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.build.fk as rFk
import rjg.libs.control.ctrl as rCtrl
reload(rModule)
reload(rAttr)
reload(rChain)
reload(rFk)

class SplineTail(rModule.RigModule, rFk.Fk):
    def __init__(self, side=None, part=None, guide_list=None, ctrl_scale=1, model_path=None, guide_path=None, pad='auto', remove_last=True, fk_shape='circle', IK_Spline = True):
        super().__init__(side=side, part=part, guide_list=guide_list, ctrl_scale=ctrl_scale, model_path=model_path, guide_path=guide_path)

        self.__dict__.update(locals())
        self.gimbal = None
        self.offset = None
        self.IK_Spline = IK_Spline

        if self.pad == 'auto':
            self.pad = len(str(len(self.guide_list))) + 1

        self.create_module()

    @staticmethod
    def build_basic_control( name='Main', shape='circle', size=5.0, color_rgb=(1, 1, 0), position=(0, 0, 0), rotation=(0, 0, 0)):
        rCtrl.ctrl = rCtrl.Control(parent=None, 
                                       shape=shape, 
                                       side='M', 
                                       suffix='CTRL', 
                                       name= name, 
                                       axis='y', 
                                       group_type='main', 
                                       rig_type='primary', 
                                       translate=position, 
                                       rotate=rotation, 
                                       ctrl_scale= size)
        ctrl_name = rCtrl.ctrl.ctrl      # This is the shape transform node (e.g., 'Main_CTRL')
        top_group = rCtrl.ctrl.top       # This is the topmost group (e.g., 'Main_CTRL_OFF')
    
        return ctrl_name, top_group


    @staticmethod
    def build_ik_spline_with_controls(aim_joints=None, prefix=None, sub=False, FeatherType=None):
        ctrlname, grpname = 'M_CTRL', 'M_CTRL_CNST_GRP'
        mc.group(name = f'{prefix}_handle_{grpname}', empty=True )
        # Step 1: Create IK spline
        ik_handle, effector, curve = mc.ikHandle(
            sj=aim_joints[0],
            ee=aim_joints[-1],
            sol='ikSplineSolver',
            ccv=True,
            pcv=False
        )
        curve = mc.rename(curve, f'{prefix}_curve')
        ik_handle = mc.rename(ik_handle, f'{prefix}_ik_handle')
        mc.parent(ik_handle, f'{prefix}_handle_{grpname}')
        mc.parent(curve, f'{prefix}_handle_{grpname}')
        # Step 2: For each CV on the curve, create cluster + control
        cvs = mc.ls(f"{curve}.cv[*]", fl=True)
        
        ikCTRLS = []
        ikOffsets = []
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
                ctrl, offset = SplineTail.build_basic_control(
                    name=ctrl_name,
                    shape='circle',
                    size=1.0,
                    color_rgb=(1, 1, 0),
                    position=pos,
                    rotation=(0, 0, 0)
                )
                
                ikCTRLS.append(ctrl)
                ikOffsets.append(offset)
                mc.parent(offset, ikgroup)

                # Parent cluster to control
                mc.parentConstraint(ctrl, cluster_handle, mo=True)
                mc.hide(ik_handle,curve,f'{prefix}_handle_{grpname}')
                try:
                    mc.parent(f'{prefix}_handle_{grpname}', ikgroup)
                except:
                    pass

        return ik_handle, curve, ikCTRLS, ikOffsets, ikgroup





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
        precontrol = None
        Ikjnts = []
        lastjnt = None
        lastIKjnt = None
        for guide in self.guide_list:
            # World position (translation)
            pos = mc.xform(guide, q=True, ws=True, t=True)   # [x, y, z]
            # World rotation (Euler angles, degrees)
            rot = mc.xform(guide, q=True, ws=True, ro=True)  # [rx, ry, rz]
            ctrl, offset  = SplineTail.build_basic_control( name=guide, shape='circle', size=5.0, color_rgb=(1, 1, 0), position=pos, rotation=rot)
            if precontrol:
                mc.parent(offset, precontrol)
                precontrol = ctrl
            else:
                precontrol = ctrl
                mc.parent(offset, 'Tail_FK_GRP')
            mc.select(clear=True)
            FKjnt = mc.joint(p=pos, o=rot, name=f'{guide}_FK')
            if lastjnt:
                mc.parent(FKjnt, lastjnt)
                lastjnt = FKjnt
            else:
                lastjnt = FKjnt
            mc.parentConstraint(ctrl, FKjnt, mo=True)
            mc.select(clear=True)
            IKjnt = mc.joint(p=pos, o=rot, name=f'{guide}_IK')
            if lastIKjnt:
                mc.parent(IKjnt, lastIKjnt)
                lastIKjnt = IKjnt
            else:
                lastIKjnt = IKjnt
            Ikjnts.append(IKjnt)
        #ik rig
            #curve = SplineTail.build_curve(self.guide_list, prefix='Tail', degree=3)
        ikstuffs = SplineTail.build_ik_spline_with_controls(aim_joints=Ikjnts, prefix="Tail", sub=False, FeatherType=None)
        mc.parent(ikstuffs[4],'Tail_IK_GRP')
        mc.parent('Tail1_FK', 'Tail_FK_GRP')
        mc.hide('Tail1_FK', 'Tail1_IK')
        mc.parent('Tail1_IK', 'Tail_IK_GRP')
        #switch

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





