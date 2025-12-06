from math import radians
from maya.api.OpenMaya import MEulerRotation, MMatrix, MSpace, MTransformationMatrix
import maya.cmds as mc
from importlib import reload

import rjg.build.rigModule as rModule
import rjg.libs.attribute as rAttr
import rjg.build.chain as rChain
import rjg.libs.control.ctrl as rCtrl
from rjg.libs.transform import (
    drive_transform_with_matrix,
    get_parent_inverse_matrix,
    get_world_matrix,
    match_pose,
)
from rjg.libs.pose_interpolator import Pose, PoseDriver, PoseInterpolator
from rjg.libs.maya_api import node
reload(rAttr)
reload(rModule)
reload(rChain)
reload(rCtrl)


class Clavicle(rModule.RigModule):
    def __init__(
        self,
        side=None,
        part=None,
        guide_list=None,
        ctrl_scale=None,
        local_orient=False,
        model_path=None,
        guide_path=None,
        swing_guide: str | None = None,
        auto_clavicle: bool = True,
        auto_clav_up: float = 0.3,
        auto_clav_down: float = 0,
        auto_clav_forward: float = 0.25,
        auto_clav_back: float = 0.25,
    ):
        super().__init__(side=side, part=part, guide_list=guide_list, ctrl_scale=ctrl_scale, model_path=model_path, guide_path=guide_path)
        self.auto_clavicle = auto_clavicle
        self.local_orient = local_orient
        self.mirror = 1 if "L" in self.side else -1
        if swing_guide is None:
            self.swing_guide = self.guide_list[-1]
        else:
            self.swing_guide = swing_guide
        self.auto_clav_up = auto_clav_up
        self.auto_clav_down = auto_clav_down
        self.auto_clav_forward = auto_clav_forward
        self.auto_clav_back = auto_clav_back
        self.create_module()

    def create_module(self):
        super(Clavicle, self).create_module()

        self.control_rig()
        self.output_rig()
        if self.auto_clavicle:
            self.create_auto_clavicle()
        self.skeleton()
        self.add_plugs()

    def create_inputs(self, group: str) -> None:
        self.input_group = mc.group(empty=True, name=f"{self.base_name}_INPUTS", parent=group)
        self.swing_group = mc.group(
            empty=True, name=f"{self.base_name}_Swing_GRP", parent=self.input_group
        )
        self.swing_input = mc.group(
            empty=True, name=f"{self.base_name}_Swing_IN", parent=self.swing_group
        )
        if self.swing_input is not None:
            match_pose(self.swing_group, rotate=self.swing_guide, translate=self.swing_guide)
        else:
            match_pose(self.swing_group, rotate=self.main_ctrl.ctrl, translate=self.main_ctrl.ctrl)

    def control_rig(self):
        if self.local_orient:
            rotate = self.guide_list[0]
        else:
            rotate = (0, 0, 0) if self.side=='L' else (0, 180, 180)

        # create controls
        self.main_ctrl = rCtrl.Control(parent=self.control_grp, shape='cube', side=self.side, suffix='CTRL', name='clavicle', axis='y', group_type='main', rig_type='primary', translate=self.guide_list[0], rotate=rotate, ctrl_scale=self.ctrl_scale)
        self.main_ctrl.tag_as_controller()

        attr_util = rAttr.Attribute(add=False)
        attr_util.lock_and_hide(node=self.main_ctrl.ctrl, translate=False, rotate=False)


    def output_rig(self):
        
        # create clavicle chain
        cnst_grp = mc.group(empty=True, parent=self.module_grp,
                             name=self.base_name + '_CNST_GRP')
        jnt_grp = mc.group(empty=True, name=self.base_name + '_JNT_GRP')
        mc.matchTransform(jnt_grp, self.guide_list[0])
        mc.matchTransform(cnst_grp, self.guide_list[0])
        self.clav_chain = rChain.Chain(transform_list=self.guide_list,
                                        side=self.side,
                                        suffix='driver_JNT',
                                        name=self.part)
        self.clav_chain.create_from_transforms(static=True,
                                               parent=jnt_grp)
        

        # create aim locators
        target_loc = mc.spaceLocator(name=self.base_name + '_target_LOC')[0]
        up_loc = mc.spaceLocator(name=self.base_name + '_upV_LOC')[0]
        up_loc_grp = mc.group(up_loc, name=up_loc + '_GRP')
        mc.parent(jnt_grp, up_loc_grp, target_loc, cnst_grp)

        # position aim locators
        mc.matchTransform(target_loc, self.clav_chain.joints[-1])
        mc.matchTransform(up_loc_grp, self.clav_chain.joints[0])
        mc.xform(up_loc, relative=True, translation=(20 * (1 if 'R' in self.side else -1), 0, 0))
        

        mc.parentConstraint(self.main_ctrl.ctrl, target_loc,
                              maintainOffset=True)
        mc.orientConstraint(self.main_ctrl.ctrl, up_loc_grp,
                              skip=['y', 'z'])
        # if self.side == 'L':
        #     av = (0, 1, 0)
        #     uv = (0, 0, -1)
        # else:
        #     av = (0, 1, 0)
        #     uv = (0, 0, -1)

        # old
        if self.side == 'L':
            av = (0, 1, 0)
            uv = (0, 0, -1)
        else:
            av = (0, -1, 0)
            uv = (0, 0, 1)


        mc.aimConstraint(target_loc, jnt_grp, aimVector=av, upVector=uv,
                           worldUpType='object', worldUpObject=up_loc)

        # add stretch
        rAttr.Attribute(node=self.main_ctrl.ctrl,
                         type='separator', name='______')
        stretch = rAttr.Attribute(node=self.main_ctrl.ctrl,
                                   type='double', value=0,
                                   min=0, max=1, keyable=True,
                                   name='stretch')
        twist = rAttr.Attribute(node=self.main_ctrl.ctrl,
                                 type='double', value=0,
                                 keyable=True, name='twist')

        mc.connectAttr(twist.attr, self.clav_chain.joints[0] + '.rotateY')
        # create nodes for stretch system
        dist = mc.createNode('distanceBetween',
                               name=self.base_name + '_stretch_DST')
        mdn = mc.createNode('multiplyDivide',
                              name=self.base_name + '_stretch_MDN')
        mdl = mc.createNode('multDL',
                              name=self.base_name + '_stretch_MDL')
        bta = mc.createNode('blendTwoAttr',
                              name=self.base_name + '_stretch_BTA')
        # connect start/end to drive stretch
        mc.setAttr(mdn + '.operation', 2)
        mc.connectAttr(self.main_ctrl.top + '.worldMatrix[0]',
                         dist + '.inMatrix1')
        mc.connectAttr(target_loc + '.worldMatrix[0]',
                         dist + '.inMatrix2')
        mc.connectAttr(dist + '.distance', mdn + '.input1X')
        d = mc.getAttr(dist + '.distance')
        # connect attr for global scale
        mc.connectAttr(self.global_scale.attr, mdl + '.input1')
        mc.connectAttr(mdl + '.output', mdn + '.input2X')
        mc.setAttr(mdl + '.input2', d)
        # blend stretch on and off
        mc.setAttr(bta + '.input[0]', 1)
        mc.connectAttr(mdn + '.outputX', bta + '.input[1]')
        mc.connectAttr(stretch.attr,
                         bta + '.attributesBlender')
        # connect final output
        mc.connectAttr(bta + '.output', self.clav_chain.joints[0] + '.scaleY')

    def create_auto_clavicle(self) -> None:
        self.create_inputs(group=self.module_grp)
        swing_driver = PoseDriver(transform=self.swing_input)
        pose_interpolator = PoseInterpolator(
            name=f"{self.base_name}_poseInterpolator",
            gaussian_interpolation=True,
            parent=self.module_grp,
            drivers=[swing_driver],
            allow_negative_weights=False,
        )

        # Define Poses
        mirror_matrix = MMatrix(((self.mirror, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
        rest_matrix: MMatrix = get_world_matrix(self.swing_input)
        parent_inverse_matrix = get_parent_inverse_matrix(self.swing_input)
        def rotate_matrix_with_mirror(rotation_matrix: MTransformationMatrix) -> MMatrix:
            return rest_matrix * (mirror_matrix * rotation_matrix.asMatrix() * mirror_matrix)
        auto_clav_strength_sum = mc.createNode("sum", name=f"{pose_interpolator.name}_Sum")

        def connect_pose(pose: Pose, strength: float):
            index = pose.index
            strength_node = node.MultiplyNode(name=f"{pose.name}_Strength")
            mc.connectAttr(f"{pose_interpolator.pose_interpolator}.output[{index}]", strength_node.input[0])
            mc.setAttr(strength_node.input[1], strength)
            mc.connectAttr(strength_node.output, f"{auto_clav_strength_sum}.input[{index}]")

        def create_pose(name: str, rotation: tuple[float, float, float], strength: float):
            rotation_radians = tuple(radians(a) for a in rotation)
            pose_transform: MTransformationMatrix = MTransformationMatrix()
            pose_transform.setRotation(MEulerRotation(*rotation_radians))
            pose_matrix = rotate_matrix_with_mirror(pose_transform) * parent_inverse_matrix
            pose = Pose(name=f"swing_{name}", matrices=[pose_matrix], gaussian_falloff=0.5)
            pose_interpolator.add_pose(pose)
            connect_pose(pose, strength)

        create_pose(name="up_90", rotation=(0, 0, 90), strength=self.auto_clav_up)
        create_pose(name="down_90", rotation=(0, 0, -90), strength=self.auto_clav_down)
        create_pose(name="forward_90", rotation=(0, -90, 0), strength=self.auto_clav_forward)
        create_pose(name="back_90", rotation=(0, 90, 0), strength=self.auto_clav_back)

        # Set up Auto-Clavicle Attribute and connect it
        auto_clav_attr = rAttr.Attribute(
            node=self.main_ctrl.ctrl,
            type="double",
            value=1,
            min=0,
            max=1,
            keyable=True,
            name="autoClavicle",
        )
        auto_clav_multiplier = node.MultiplyNode(name=f"{self.base_name}_Swing_Multiplier")
        mc.connectAttr(f"{auto_clav_strength_sum}.output", auto_clav_multiplier.input[0])
        mc.connectAttr(auto_clav_attr.attr, auto_clav_multiplier.input[1])
        auto_clav_clamp = node.ClampRangeNode(name=f"{self.base_name}_Swing_Mult_Clamp")
        mc.connectAttr(auto_clav_multiplier.output, auto_clav_clamp.input)

        driven_transform: str = self.main_ctrl.group_list[1]
        # Create constraint matrix to blend with.
        swing_matrix = mc.createNode("multMatrix", name=f"{self.base_name}_Swing_Matrix")
        offset_matrix = (
            get_world_matrix(driven_transform) * get_world_matrix(self.swing_input).inverse()
        )
        mc.setAttr(f"{swing_matrix}.matrixIn[0]", offset_matrix, type="matrix")
        mc.connectAttr(f"{self.swing_input}.worldMatrix", f"{swing_matrix}.matrixIn[1]")
        mc.connectAttr(f"{driven_transform}.parentInverseMatrix[0]", f"{swing_matrix}.matrixIn[2]")

        matrix_blend = mc.createNode("blendMatrix", name=f"{self.base_name}_Swing_Blend")
        mc.connectAttr(f"{swing_matrix}.matrixSum", f"{matrix_blend}.target[0].targetMatrix")

        mc.connectAttr(auto_clav_clamp.output, f"{matrix_blend}.target[0].weight")
        drive_transform_with_matrix(
            f"{matrix_blend}.outputMatrix",
            driven_transform,
            translate=False,
            shear=False,
            scale=False,
        )
        
        pass

    def skeleton(self):
        bind_chain = rChain.Chain(transform_list=self.clav_chain.joints,
                                   side=self.side,
                                   suffix='JNT',
                                   name=self.part)
        bind_chain.create_from_transforms(parent=self.skel)
        self.bind_joints = bind_chain.joints
        self.tag_bind_joints(self.bind_joints[:-1])

    def add_plugs(self):
        rAttr.Attribute(node=self.part_grp, type='plug', value=['chest_M_JNT'], name='skeletonPlugs', children_name=[self.bind_joints[0]])

        driver_list = ['chest_M_02_JNT',
                       'chest_M_02_JNT']
        driven_list = ['clavicle_' + self.side + '_CTRL_CNST_GRP',
                       'clavicle_' + self.side + '_CNST_GRP']
        rAttr.Attribute(node=self.part_grp, type='plug', value=driver_list, name='pacRigPlugs', children_name=driven_list)

        # target_list = ['ROOT', 
        #                'global_M_CTRL',
        #                'root_02_M_CTRL',
        #                'COG_M_CTRL',
        #                'chest_M_01_CTRL',
        #                'chest_M_02_CTRL',
        #                '5']
        # name_list = ['world', 'global', 'root', 'cog', 'chest_01', 'chest_02', 'default_value']
        # point_names = ['point' + name.title() for name in name_list]
        # orient_names = ['orient' + name.title() for name in name_list]
        # rAttr.Attribute(node=self.part_grp, type='plug', value=target_list, name=self.main_ctrl.ctrl +'_point', children_name=point_names)
        # rAttr.Attribute(node=self.part_grp, type='plug', value=target_list, name=self.main_ctrl.ctrl +'_orient', children_name=orient_names)
