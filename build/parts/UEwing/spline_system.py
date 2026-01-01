from dataclasses import dataclass
from importlib import reload
from typing import Sequence

import maya.cmds as mc
import rjg.build.chain as rChain
import rjg.build.guide as rGuide
import rjg.libs.attribute as rAttr
import rjg.libs.control.ctrl as rCtrl
import rjg.libs.transform as rXform
from maya.api.OpenMaya import MFnNurbsCurve, MPoint, MSelectionList, MSpace, MVector
from rjg.libs.maya_api import node
from rjg.libs.spline import get_cvs
from rjg.libs.spline.math import Vector3, point_on_spline_weights
from rjg.libs.spline.maya_query import get_knots
from typing_extensions import Literal

reload(rAttr)
reload(rChain)
reload(rCtrl)
reload(rGuide)
reload(rXform)


def get_curve(node: str) -> str | None:
    """
    Return a nurbsCurve shape from a given node.

    If the node itself is a nurbsCurve shape, it is returned directly.
    Otherwise, the function searches the node's children for a nurbsCurve shape
    and returns the first match.

    Args:
        node: Transform or shape node name.

    Returns:
        The name of the nurbsCurve shape if found, otherwise None.
    """
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
    guides: Sequence[str] | str,
    parent: str | None = None,
    degree: int = 3,
    rebuild_spans: int | None = None,
    edit_point: bool = True,
    display_reference: bool = False,
) -> str:
    """
    Create a nurbs curve from a sequence of guide transforms.

    The curve is built using either edit points or CVs, optionally rebuilt,
    parented, and set to reference display mode for visual clarity.

    Args:
        name: Name of the resulting curve transform.
        guides: Ordered guide transforms defining the curve shape, or a curve.
        parent: Optional parent transform for the curve.
        degree: Degree of the curve.
        rebuild_spans: If provided, rebuilds the curve with this span count.
        edit_point: Whether to construct the curve using edit points.
        display_reference: Whether to display the curve as reference geometry.

    Returns:
        The name of the created curve transform.
    """
    if isinstance(guides, str):
        curve: str = mc.duplicate(guides, name=f"{name}")[0]
        mc.parent(curve, parent)
        mc.makeIdentity(curve, apply=True)
    else:
        positions: list[tuple[float, float, float]] = [
            get_world_position(guide) for guide in guides
        ]
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
    """
    Compute the closest point on a curve to a guide transform.

    The result can be returned as either a normalized arc-length fraction
    or a raw curve parameter value.

    Args:
        curve: Curve transform or shape node.
        guide: Transform used as the query position.
        fraction: If True, return normalized arc-length (0–1).
                    If False, return the curve parameter.

    Returns:
        Closest point value as a fraction or parameter.
    """
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


@dataclass
class MotionPathPin:
    """
    Container for motion path pin data.

    Attributes:
        pin: Transform driven by the motion path.
        motion_path: motionPath node driving the pin.
        orient_attr: Matrix attribute used for orientation output.
    """

    pin: str
    motion_path: str
    orient_attr: str


def create_pin_on_curve(
    name: str,
    curve: str,
    guide: str,
    parent: str,
    arc_length: bool = True,
    normalize_orient: bool = True,
) -> MotionPathPin:
    """
    Create a transform pinned to a curve using a motionPath node.

    The pin is positioned at the closest point to a guide and optionally
    outputs a normalized orientation matrix.

    Args:
        name: Name of the pin transform.
        curve: Curve to attach the pin to.
        guide: Guide transform used to determine initial placement.
        parent: Parent transform for the pin.
        arc_length: Whether to use arc-length parameterization.
        normalize_orient: Whether to normalize orientation via a PickMatrix (remove scale).

    Returns:
        A MotionPathPin instance describing the created pin.
    """
    curve_shape = get_curve(curve)
    pin: str = mc.group(empty=True, name=name, parent=parent)

    motion_path = mc.createNode("motionPath", name=f"{name}_motionPathPin")
    mc.setAttr(f"{motion_path}.fractionMode", arc_length)
    mc.setAttr(f"{motion_path}.follow", True)
    mc.connectAttr(f"{curve_shape}.local", f"{motion_path}.geometryPath")
    mc.connectAttr(f"{motion_path}.allCoordinates", f"{pin}.translate")
    mc.connectAttr(f"{motion_path}.rotate", f"{pin}.rotate")

    if normalize_orient:
        motion_path_orient = node.PickMatrixNode(name=f"{name}_motionPathOrient")
        mc.connectAttr(f"{motion_path}.orientMatrix", motion_path_orient.input_matrix)
        motion_path_orient.use_translate.set(False)
        motion_path_orient.use_scale.set(False)
        motion_path_orient.use_translate.set(False)
        orient_attr = str(motion_path_orient.output_matrix)
    else:
        orient_attr = f"{motion_path}.orientMatrix"

    fraction = closest_point_on_curve(curve_shape, guide, fraction=arc_length)
    mc.setAttr(f"{motion_path}.uValue", fraction)
    return MotionPathPin(pin, motion_path, orient_attr)


def create_swing_pin_on_curve(
    name: str,
    curve: str,
    guide: str,
    orient_guide: str,
    parent: str,
    orient_driver: str | None,
    arc_length: bool = True,
):
    pin_offset = mc.group(empty=True, name=f"{name}_Offset", parent=parent)
    pin: str = mc.spaceLocator(name=f"{name}")[0]
    pin_shape = mc.listRelatives(pin, shapes=True, children=True)[0]
    mc.setAttr(f"{pin_shape}.localScale", 40, 40, 40, type="double3")
    mc.parent(pin, pin_offset, relative=True)
    # pin = mc.group(empty=True, name=f"{name}_Pin", parent=pin_offset)
    guide_pos = mc.xform(guide, query=True, worldSpace=True, translation=True)
    rXform.match_transform(pin_offset, orient_guide)
    mc.xform(pin_offset, worldSpace=True, translation=guide_pos)
    if orient_driver is not None:
        rXform.matrix_constraint(orient_driver, pin_offset)

    curve_pin = create_pin_on_curve(
        name=f"{name}_Curve_Pin",
        curve=curve,
        guide=guide,
        parent=parent,
        arc_length=arc_length,
        normalize_orient=False,
    )
    tangent_vector = node.AxisFromMatrixNode(name=f"{name}_Tangent")
    mc.connectAttr(curve_pin.orient_attr, tangent_vector.input)
    tangent_vector.axis.set(1)

    localize_matrix = node.MultMatrixNode(f"{name}_LocalizeMatrix")
    mc.connectAttr(f"{parent}.worldMatrix[0]", localize_matrix.matrix_in[0])
    mc.connectAttr(f"{pin_offset}.worldInverseMatrix[0]", localize_matrix.matrix_in[1])

    tangent_local = node.MultiplyVectorByMatrixNode(name=f"{name}_TangentLocal")
    mc.connectAttr(tangent_vector.output, tangent_local.input_vector)
    mc.connectAttr(localize_matrix.matrix_sum, tangent_local.input_matrix)

    swing_matrix = node.AimMatrixNode(name=f"{name}_Swing_Matrix")
    mc.connectAttr(tangent_local.output, swing_matrix.primary.target_vector)
    swing_matrix.primary.input_axis.set((0, 1, 0))
    rXform.drive_transform_with_matrix(swing_matrix.output_matrix, pin, translate=False)
    rXform.matrix_constraint(
        curve_pin.pin, pin, keep_offset=False, rotate=False, shear=False, scale=False
    )

    return pin


def create_pin_on_net_matrix(
    name: str,
    curve: str,
    backbone_pins: Sequence[MotionPathPin],
    guide: str,
    arc_length: bool = True,
    normalize_orient: bool = True,
):
    """
    Create a matrix for a pin constrained to a spline network with orientation blending.

    Orientation is computed by blending backbone tangents and constructing
    an orthonormal basis aligned to the local spline direction.

    Args:
        name: Name of the pin locator.
        curve: Spline curve driving the pin position.
        backbone_pins: Backbone motion path pins used for orientation blending.
        guide: Guide transform used for closest-point evaluation.
        parent: Parent transform for the pin.
        arc_length: Whether to use arc-length parameterization.
        normalize_orient: Whether to normalize motion path orientation.

    Returns:
        The path the created pin matrix attribute.
    """
    curve_shape = get_curve(curve)
    curve_knots = get_knots(curve_shape)

    fraction = closest_point_on_curve(curve_shape, guide, fraction=arc_length)
    parameter = closest_point_on_curve(curve_shape, guide, fraction=False)
    weights = point_on_spline_weights(
        cvs=list(backbone_pins), t=parameter, knots=curve_knots, normalize=False, degree=2
    )

    motion_path = mc.createNode("motionPath", name=f"{name}_motionPathPin")
    mc.setAttr(f"{motion_path}.fractionMode", arc_length)
    mc.connectAttr(f"{curve_shape}.local", f"{motion_path}.geometryPath")
    mc.setAttr(f"{motion_path}.uValue", fraction)
    mc.setAttr(f"{motion_path}.follow", True)

    motion_path_orient_attr = f"{motion_path}.orientMatrix"
    if normalize_orient:
        motion_path_orient = node.PickMatrixNode(name=f"{name}_motionPathOrient")
        mc.connectAttr(f"{motion_path}.orientMatrix", motion_path_orient.input_matrix)
        motion_path_orient.use_translate.set(False)
        motion_path_orient.use_scale.set(False)
        motion_path_orient.use_translate.set(False)
        motion_path_orient_attr = motion_path_orient.output_matrix

    tangent_node = node.AxisFromMatrixNode(name=f"{name}_tangent")
    mc.connectAttr(motion_path_orient_attr, tangent_node.input)
    tangent_node.axis.value = 1

    # Pin
    matrix_blend = node.WtAddMatrixNode(name=f"{name}_tangentBlend")
    for index, (backbone_pin, weight) in enumerate(weights):
        mc.connectAttr(backbone_pin.orient_attr, matrix_blend.weight_matrix[index].matrix_in)
        mc.setAttr(matrix_blend.weight_matrix[index].weight_in, weight)

    backbone_tangent_node = node.AxisFromMatrixNode(name=f"{name}_backboneTangent")
    mc.connectAttr(matrix_blend.matrix_sum, backbone_tangent_node.input)
    backbone_tangent_node.axis.value = 1

    cross_product_node = node.CrossProductNode(f"{name}_tangentCross")
    mc.connectAttr(tangent_node.output, cross_product_node.input1)
    mc.connectAttr(backbone_tangent_node.output, cross_product_node.input2)

    backbone_tangent_ortho = node.CrossProductNode(f"{name}_backboneTangentOrtho")
    mc.connectAttr(cross_product_node.output, backbone_tangent_ortho.input1)
    mc.connectAttr(tangent_node.output, backbone_tangent_ortho.input2)

    x_normalize = node.NormalizeNode(f"{name}_xNormalized")
    mc.connectAttr(backbone_tangent_ortho.output, x_normalize.input)
    y_normalize = node.NormalizeNode(f"{name}_yNormalized")
    mc.connectAttr(tangent_node.output, y_normalize.input)
    z_normalize = node.NormalizeNode(f"{name}_zNormalized")
    mc.connectAttr(cross_product_node.output, z_normalize.input)

    basis_matrix_node = node.FourByFourMatrixNode(f"{name}_BasisMatrix")
    mc.connectAttr(x_normalize.output.x, basis_matrix_node.in_00)
    mc.connectAttr(x_normalize.output.y, basis_matrix_node.in_01)
    mc.connectAttr(x_normalize.output.z, basis_matrix_node.in_02)
    mc.connectAttr(y_normalize.output.x, basis_matrix_node.in_10)
    mc.connectAttr(y_normalize.output.y, basis_matrix_node.in_11)
    mc.connectAttr(y_normalize.output.z, basis_matrix_node.in_12)
    mc.connectAttr(z_normalize.output.x, basis_matrix_node.in_20)
    mc.connectAttr(z_normalize.output.y, basis_matrix_node.in_21)
    mc.connectAttr(z_normalize.output.z, basis_matrix_node.in_22)
    mc.connectAttr(f"{motion_path}.allCoordinates.xCoordinate", basis_matrix_node.in_30)
    mc.connectAttr(f"{motion_path}.allCoordinates.yCoordinate", basis_matrix_node.in_31)
    mc.connectAttr(f"{motion_path}.allCoordinates.zCoordinate", basis_matrix_node.in_32)

    return basis_matrix_node.output


def create_pin_on_net(
    name: str,
    curve: str,
    backbone_pins: Sequence[MotionPathPin],
    root_pin: str,
    guide: str,
    parent: str,
    arc_length: bool = True,
    normalize_orient: bool = True,
) -> str:
    """
    Create a pin constrained to a spline network with orientation blending.

    Orientation is computed by blending backbone tangents and constructing
    an orthonormal basis aligned to the local spline direction.

    Args:
        name: Name of the pin locator.
        curve: Spline curve driving the pin position.
        backbone_pins: Backbone motion path pins used for orientation blending.
        guide: Guide transform used for closest-point evaluation.
        parent: Parent transform for the pin.
        arc_length: Whether to use arc-length parameterization.
        normalize_orient: Whether to normalize motion path orientation.

    Returns:
        The name of the created pin transform.
    """
    pin: str = mc.spaceLocator(name=name)[0]
    pin_shape = mc.listRelatives(pin, shapes=True, children=True)[0]
    mc.setAttr(f"{pin_shape}.localScale", 20, 20, 20, type="double3")
    mc.parent(pin, parent, relative=True)

    pin_matrix = create_pin_on_net_matrix(
        name=name,
        curve=curve,
        backbone_pins=backbone_pins,
        guide=guide,
        arc_length=arc_length,
        normalize_orient=normalize_orient,
    )

    curve_shape = get_curve(curve)
    curve_knots = get_knots(curve_shape)

    parameter = closest_point_on_curve(curve_shape, guide, fraction=False)
    weights = point_on_spline_weights(
        cvs=[i for i, _ in enumerate(backbone_pins)],
        t=parameter,
        knots=curve_knots,
        normalize=False,
        degree=2,
        filter_weights=False,
    )
    weight_dict = {i: weight for i, weight in weights}
    root_weight = weight_dict[0]

    driver_matrix = pin_matrix
    if root_weight > 0:
        root_localize = node.MultMatrixNode(name=f"{name}_RootLocalize")
        mc.connectAttr(f"{root_pin}.worldMatrix[0]", root_localize.matrix_in[0])
        mc.connectAttr(f"{pin}.parentInverseMatrix", root_localize.matrix_in[1])

        root_blend = node.BlendMatrixNode(name=f"{name}_RootBlend")
        mc.connectAttr(pin_matrix, root_blend.input_matrix)
        mc.connectAttr(root_localize.matrix_sum, root_blend.target[0].target_matrix)
        root_blend.target[0].weight.set(root_weight)
        root_blend.target[0].translate_weight.set(0)
        root_blend.target[0].scale_weight.set(0)
        root_blend.target[0].shear_weight.set(0)
        driver_matrix = root_blend.output_matrix

    rXform.drive_transform_with_matrix(driver_matrix, pin, scale=True, shear=True)
    return pin


def lerp_vectors(start_point: MVector, end_point: MVector, alpha: float) -> MVector:
    clamped_alpha = max(min(alpha, 1), 0)
    invert_alpha = 1 - clamped_alpha
    return (start_point * invert_alpha) + (end_point * clamped_alpha)


def create_mid_guides(
    start_guide: str, end_guide: str, mid_num: int, guide_name_prefix: str, parent: str
) -> list[str]:
    """
    Create evenly spaced intermediate guide transforms between two guides.
    Guides are positioned via linear interpolation in world space.

    Args:
        start_guide: Starting guide transform.
        end_guide: Ending guide transform.
        mid_num: Number of intermediate guides to create.
        guide_name_prefix: Prefix used to name new guides.
        parent: Parent transform for the new guides.

    Returns:
        List of created mid-guide transform names.
    """

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


def create_swing_transform(
    name: str, driver: str, parent: str, twist_axis: Literal["x", "y", "z"] = "y"
) -> str:
    driver_parents = mc.listRelatives(driver, parent=True)
    if driver_parents:
        driver_parent = driver_parents[0]
    else:
        driver_parent = None

    swing_offset = mc.group(empty=True, name=f"{name}_Offset", parent=parent)
    rXform.match_transform(swing_offset, driver)
    if driver_parent is not None:
        rXform.matrix_constraint(driver_parent, swing_offset)

    driver_local_offset = (
        rXform.get_parent_matrix(driver) * rXform.get_world_matrix(driver).inverse()
    )
    driver_local_matrix = node.MultMatrixNode(name=f"{name}_DriverLocal")
    mc.connectAttr(f"{driver}.matrix", driver_local_matrix.matrix_in[0])
    mc.setAttr(driver_local_matrix.matrix_in[1], driver_local_offset, type="matrix")

    swing_transform = mc.group(empty=True, name=f"{name}", parent=swing_offset)

    quat_node = node.DecomposeMatrixNode(name=f"{name}_Quat")
    mc.connectAttr(driver_local_matrix.matrix_sum, quat_node.input_matrix)

    inverse = node.QuatInvertNode(name=f"{name}_Twist_Inverse")
    if twist_axis == "x":
        mc.connectAttr(quat_node.output_quat.x, inverse.input_quat.x)
    elif twist_axis == "y":
        mc.connectAttr(quat_node.output_quat.y, inverse.input_quat.y)
    elif twist_axis == "z":
        mc.connectAttr(quat_node.output_quat.z, inverse.input_quat.z)
    mc.connectAttr(quat_node.output_quat.w, inverse.input_quat.w)

    swing_quat = node.QuatProdNode(f"{name}_Swing")
    mc.connectAttr(inverse.output_quat, swing_quat.input1_quat)
    mc.connectAttr(quat_node.output_quat, swing_quat.input2_quat)

    swing_euler = node.QuatToEulerNode(f"{name}_Swing_Euler")
    mc.connectAttr(swing_quat.output_quat, swing_euler.input_quat)

    mc.connectAttr(f"{swing_transform}.rotateOrder", swing_euler.input_rotate_order)
    mc.connectAttr(swing_euler.output_rotate, f"{swing_transform}.rotate")

    return swing_transform


class Spline:
    """
    Spline wrapper that builds a curve, optional controls, and pin connections.

    CVs can be driven directly by pinned transforms or generated controls.
    """

    def __init__(
        self,
        name: str,
        guides: Sequence[str] | str,
        parent: str,
        create_controls: bool = True,
        control_parent: str | None = None,
        pin_transforms: Sequence[str] | None = None,
        create_pins: bool = False,
        ctrl_scale: float = 1,
        degree: int = 3,
        rebuild: int | None = None,
        display_reference=True,
    ) -> None:
        """
        Initialize and build a spline system.

        Args:
            name: Base name for the spline.
            guides: Transform(s) defining the spline shape. Can be points along the spline, CVs, or a curve to be copied.
            parent: Parent transform for the spline.
            build_controls: Whether to generate control objects per CV.
            control_parent: Parent transform for generated controls.
            pin_transforms: Optional transforms to drive CVs directly (need to be in same space as the spline).
            ctrl_scale: Scale multiplier for generated controls.
            degree: Curve degree.
            rebuild: Optional rebuild span count.
            display_reference: Whether to show the spline as reference geometry.
        """
        self.name = name
        self.guides = guides
        self.spline: str = spline_from_guides(
            guides=guides,
            name=f"{name}",
            parent=parent,
            degree=degree,
            rebuild_spans=rebuild,
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
            cv_name = f"{name}_{index:02d}"
            if create_pins:
                pin = mc.group(empty=True, name=f"{cv_name}_Pin", parent=parent)
                mc.xform(pin, worldSpace=True, translation=position)
                self.pin_list.append(pin)
            if create_controls:
                ctrl = rCtrl.Control(
                    name=cv_name,
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
                    ctrl_pin = mc.group(empty=True, name=f"{cv_name}_Pin", parent=parent)
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
