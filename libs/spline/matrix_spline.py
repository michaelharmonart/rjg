import maya.cmds as mc
from maya.api.OpenMaya import (
    MDoubleArray,
    MFnNurbsCurve,
    MFnNurbsCurveData,
    MObject,
    MPoint,
    MPointArray,
    MSpace,
)
from rjg.libs.spline.math import (
    Vector3,
    generate_knots,
    point_on_spline_weights,
    resample,
    tangent_on_spline_weights,
)
from rjg.libs.transform import matrix_constraint
from rjg.libs.maya_api import node


class MatrixSpline:
    def __init__(
        self,
        cv_transforms: list[str],
        degree: int = 3,
        knots: list[float] | None = None,
        periodic: bool = False,
        name: str | None = None,
    ) -> None:
        self.periodic: bool = periodic
        self.degree: int = degree
        self.cv_transforms: list[str] = cv_transforms
        number_of_cvs: int = len(cv_transforms) + (periodic * degree)
        if knots:
            self.knots: list[float] = knots
        else:
            self.knots: list[float] = generate_knots(count=number_of_cvs, degree=degree)
        if name:
            self.name: str = name
        else:
            self.name: str = "MatrixSpline"
        cv_matrices: list[str] = []
        for index, cv_transform in enumerate(cv_transforms):
            # Remove scale and shear from matrix since they will interfere with the
            # linear interpolation of the basis vectors (causing flipping)
            pick_matrix = mc.createNode("pickMatrix", name=f"{cv_transform}_PickMatrix")
            mc.connectAttr(f"{cv_transform}.matrix", f"{pick_matrix}.inputMatrix")
            mc.setAttr(f"{pick_matrix}.useShear", 0)
            mc.setAttr(f"{pick_matrix}.useScale", 0)
            # Add nodes to connect individual values from the matrix,
            # I don't know why maya makes us do this instead of just connecting directly
            deconstruct_matrix_attribute = f"{pick_matrix}.outputMatrix"
            row1 = node.RowFromMatrixNode(name=f"{cv_transform}_row1")
            mc.connectAttr(deconstruct_matrix_attribute, row1.matrix)
            mc.setAttr(row1.input, 0)
            row2 = node.RowFromMatrixNode(name=f"{cv_transform}_row2")
            mc.connectAttr(deconstruct_matrix_attribute, row2.matrix)
            mc.setAttr(row2.input, 1)
            row3 = node.RowFromMatrixNode(name=f"{cv_transform}_row3")
            mc.setAttr(row3.input, 2)
            mc.connectAttr(deconstruct_matrix_attribute, row3.matrix)
            row4 = node.RowFromMatrixNode(name=f"{cv_transform}_row4")
            mc.connectAttr(deconstruct_matrix_attribute, row4.matrix)
            mc.setAttr(row4.input, 3)

            # Rebuild the matrix but encode the scale into the empty values in the matrix
            # (this needs to be extracted after the weighted matrix sum)
            cv_matrix = mc.createNode("fourByFourMatrix", name=f"{cv_transform}_CvMatrix")
            mc.connectAttr(f"{row1.output}X", f"{cv_matrix}.in00")
            mc.connectAttr(f"{row1.output}Y", f"{cv_matrix}.in01")
            mc.connectAttr(f"{row1.output}Z", f"{cv_matrix}.in02")
            mc.connectAttr(f"{cv_transform}.scaleX", f"{cv_matrix}.in03")

            mc.connectAttr(f"{row2.output}X", f"{cv_matrix}.in10")
            mc.connectAttr(f"{row2.output}Y", f"{cv_matrix}.in11")
            mc.connectAttr(f"{row2.output}Z", f"{cv_matrix}.in12")
            mc.connectAttr(f"{cv_transform}.scaleY", f"{cv_matrix}.in13")

            mc.connectAttr(f"{row3.output}X", f"{cv_matrix}.in20")
            mc.connectAttr(f"{row3.output}Y", f"{cv_matrix}.in21")
            mc.connectAttr(f"{row3.output}Z", f"{cv_matrix}.in22")
            mc.connectAttr(f"{cv_transform}.scaleZ", f"{cv_matrix}.in23")

            mc.connectAttr(f"{row4.output}X", f"{cv_matrix}.in30")
            mc.connectAttr(f"{row4.output}Y", f"{cv_matrix}.in31")
            mc.connectAttr(f"{row4.output}Z", f"{cv_matrix}.in32")
            mc.connectAttr(f"{row4.output}W", f"{cv_matrix}.in33")

            cv_matrices.append(f"{cv_matrix}.output")

        # If the curve is periodic there are we need to re-add CVs that move together.
        if periodic:
            for i in range(degree):
                cv_matrices.append(cv_matrices[i])

        self.cv_matrices = cv_matrices


def closest_point_on_matrix_spline(
    matrix_spline: MatrixSpline, position: list[float, float, float]
) -> float:
    """
    Finds the closest parameter value on a spline (defined by a MatrixSpline) to a given 3D position.

    Args:
        matrix_spline: Spline definition object.
        position: The world-space point to project onto the spline.

    Returns:
        float: The curve parameter value (in knot space) at the closest point to the input position.
    """
    knots: list[float] = matrix_spline.knots
    degree: int = matrix_spline.degree
    periodic: bool = matrix_spline.periodic
    cv_transforms: list[str] = matrix_spline.cv_transforms
    cv_positions: MPointArray = []
    for transform in cv_transforms:
        cv_position: tuple[float, float, float] = mc.xform(
            transform, query=True, worldSpace=True, translation=True
        )
        position_tuple: tuple[float, float, float] = tuple(cv_position)
        cv_positions.append(MPoint(*position_tuple))
    maya_knots: list[float] = knots[1:-1]

    fn_data: MFnNurbsCurveData = MFnNurbsCurveData()
    data_obj: MObject = fn_data.create()
    fn_curve: MFnNurbsCurve = MFnNurbsCurve()
    curve_obj: MFnNurbsCurve = fn_curve.create(
        cv_positions,
        MDoubleArray(maya_knots),
        degree,
        MFnNurbsCurve.kOpen if not periodic else MFnNurbsCurve.kPeriodic,
        False,  # create2D
        False,  # not rational
        data_obj,
    )

    parameter: float = fn_curve.closestPoint(
        MPoint(position[0], position[1], position[2]), space=MSpace.kObject
    )[1]

    return parameter


def pin_to_matrix_spline(
    matrix_spline: MatrixSpline,
    pinned_transform: str,
    parameter: float,
    normalize_parameter: bool = True,
    stretch: bool = True,
    primary_axis: tuple[int, int, int] | None = (0, 1, 0),
    secondary_axis: tuple[int, int, int] | None = (0, 0, 1),
) -> None:
    """
    Pins a transform to a matrix spline at a given parameter along the curve.

    Args:
        matrix_spline: The matrix spline data object.
        pinned_transform: Transform to pin to the spline.
        parameter: Position along the spline (0–1).
        stretch: Whether to apply automatic scaling along the spline tangent.
        primary_axis (tuple[int, int, int], optional): Local axis of the pinned
            transform that should aim down the spline tangent. Must be one of
            the cardinal axes (±X, ±Y, ±Z). Defaults to (0, 1, 0) (the +Y axis).
        secondary_axis (tuple[int, int, int], optional): Local axis of the pinned
            transform that should be aligned to a secondary reference direction
            from the spline. Used to resolve orientation. Must be one of the
            cardinal axes (±X, ±Y, ±Z) and orthogonal to ``primary_axis``.
            Defaults to (0, 0, 1) (the +Z axis).
    Returns:
        None
    """
    if not primary_axis:
        primary_axis = (0, 1, 0)
    if not secondary_axis:
        secondary_axis = (0, 0, 1)

    CARDINALS = {(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)}
    if tuple(primary_axis) not in CARDINALS or tuple(secondary_axis) not in CARDINALS:
        raise ValueError(
            "primary_axis and secondary_axis must be one of the cardinal axes (±X, ±Y, ±Z)."
        )

    cv_matrices: list[str] = matrix_spline.cv_matrices
    degree: int = matrix_spline.degree
    knots: list[float] = matrix_spline.knots
    segment_name: str = pinned_transform

    # Create node that blends the matrices based on the calculated DeBoor weights.
    blended_matrix = mc.createNode("wtAddMatrix", name=f"{segment_name}_BaseMatrix")
    point_weights = point_on_spline_weights(
        cvs=cv_matrices, t=parameter, degree=degree, knots=knots, normalize=normalize_parameter
    )
    for index, point_weight in enumerate(point_weights):
        mc.setAttr(f"{blended_matrix}.wtMatrix[{index}].weightIn", point_weight[1])
        mc.connectAttr(f"{point_weight[0]}", f"{blended_matrix}.wtMatrix[{index}].matrixIn")

    blended_tangent_matrix = mc.createNode("wtAddMatrix", name=f"{segment_name}_TangentMatrix")
    tangent_weights = tangent_on_spline_weights(
        cvs=cv_matrices, t=parameter, degree=degree, knots=knots, normalize=normalize_parameter
    )
    for index, tangent_weight in enumerate(tangent_weights):
        mc.setAttr(
            f"{blended_tangent_matrix}.wtMatrix[{index}].weightIn",
            tangent_weight[1],
        )
        mc.connectAttr(
            f"{tangent_weight[0]}",
            f"{blended_tangent_matrix}.wtMatrix[{index}].matrixIn",
        )
    tangent_vector_node: node.MultiplyPointByMatrixNode = node.MultiplyPointByMatrixNode(
        name=f"{blended_tangent_matrix}_TangentVector"
    )

    mc.connectAttr(f"{blended_tangent_matrix}.matrixSum", tangent_vector_node.input_matrix)

    # Create nodes to access the values of the blended matrix node.
    deconstruct_matrix_attribute = f"{blended_matrix}.matrixSum"
    blended_matrix_row1 = node.RowFromMatrixNode(name=f"{blended_matrix}_row1")
    mc.setAttr(blended_matrix_row1.input, 0)
    mc.connectAttr(deconstruct_matrix_attribute, blended_matrix_row1.matrix)
    blended_matrix_row2 = node.RowFromMatrixNode(name=f"{blended_matrix}_row2")
    mc.connectAttr(deconstruct_matrix_attribute, blended_matrix_row2.matrix)
    mc.setAttr(blended_matrix_row2.input, 1)
    blended_matrix_row3 = node.RowFromMatrixNode(name=f"{blended_matrix}_row3")
    mc.connectAttr(deconstruct_matrix_attribute, blended_matrix_row3.matrix)
    mc.setAttr(blended_matrix_row3.input, 2)
    blended_matrix_row4 = node.RowFromMatrixNode(name=f"{blended_matrix}_row4")
    mc.connectAttr(deconstruct_matrix_attribute, blended_matrix_row4.matrix)
    mc.setAttr(blended_matrix_row4.input, 3)

    # Create aim matrix node.
    aim_matrix = mc.createNode("aimMatrix", name=f"{segment_name}_AimMatrix")
    mc.setAttr(f"{aim_matrix}.primaryMode", 2)
    mc.setAttr(f"{aim_matrix}.primaryInputAxis", *primary_axis)
    mc.setAttr(f"{aim_matrix}.secondaryMode", 2)
    mc.setAttr(f"{aim_matrix}.secondaryInputAxis", *secondary_axis)
    mc.connectAttr(tangent_vector_node.output, f"{aim_matrix}.primary.primaryTargetVector")

    axis_to_row: dict[tuple[int, int, int], node.RowFromMatrixNode] = {
        (1, 0, 0): blended_matrix_row1,
        (0, 1, 0): blended_matrix_row2,
        (0, 0, 1): blended_matrix_row3,
        (-1, 0, 0): blended_matrix_row1,  # flipped
        (0, -1, 0): blended_matrix_row2,
        (0, 0, -1): blended_matrix_row3,
    }
    secondary_row: node.RowFromMatrixNode | None = axis_to_row.get(tuple(secondary_axis))
    if secondary_row:
        mc.connectAttr(
            f"{secondary_row.output}X", f"{aim_matrix}.secondary.secondaryTargetVectorX"
        )
        mc.connectAttr(
            f"{secondary_row.output}Y", f"{aim_matrix}.secondary.secondaryTargetVectorY"
        )
        mc.connectAttr(
            f"{secondary_row.output}Z", f"{aim_matrix}.secondary.secondaryTargetVectorZ"
        )

    # Create nodes to access the values of the aim matrix node.
    deconstruct_matrix_attribute = f"{aim_matrix}.outputMatrix"
    aim_matrix_row1 = node.RowFromMatrixNode(name=f"{aim_matrix}_row1")
    mc.connectAttr(deconstruct_matrix_attribute, aim_matrix_row1.matrix)
    mc.setAttr(aim_matrix_row1.input, 0)
    aim_matrix_row2 = node.RowFromMatrixNode(name=f"{aim_matrix}_row2")
    mc.connectAttr(deconstruct_matrix_attribute, aim_matrix_row2.matrix)
    mc.setAttr(aim_matrix_row2.input, 1)
    aim_matrix_row3 = node.RowFromMatrixNode(name=f"{aim_matrix}_row3")
    mc.connectAttr(deconstruct_matrix_attribute, aim_matrix_row3.matrix)
    mc.setAttr(aim_matrix_row3.input, 2)

    if stretch:
        # Get tangent vector magnitude
        tangent_vector_length = node.LengthNode(name=f"{segment_name}_tangentVectorLength")
        mc.connectAttr(tangent_vector_node.output, tangent_vector_length.input)
        tangent_vector_length_scaled: node.MultiplyNode = node.MultiplyNode(
            name=f"{segment_name}_tangentVectorLengthScaled"
        )
        mc.connectAttr(tangent_vector_length.output, tangent_vector_length_scaled.input[0])

        tangent_sample = mc.getAttr(tangent_vector_node.output)[0]
        tangent_length = Vector3(tangent_sample[0], tangent_sample[1], tangent_sample[2]).length()
        if tangent_length == 0:
            raise RuntimeError(
                f"{pinned_transform} had a tangent magnitude of 0 and wasn't able to be pinned with stretching enabled."
            )
        mc.setAttr(tangent_vector_length_scaled.input[1], 1 / tangent_length)
        tangent_scale_attr: str = tangent_vector_length_scaled.output

    def is_same_axis(axis1: tuple[int, int, int], axis2: tuple[int, int, int]) -> bool:
        # Compare absolute values to handle flips: (0,1,0) == (0,-1,0)
        return tuple(abs(v) for v in axis1) == tuple(abs(v) for v in axis2)

    def scale_vector(
        vector_attr: str, scale_attr: str, node_name: str, axis: tuple[int, int, int]
    ) -> str:
        scale_node = mc.createNode("multiplyDivide", name=node_name)
        mc.connectAttr(f"{vector_attr}X", f"{scale_node}.input1X")
        mc.connectAttr(f"{vector_attr}Y", f"{scale_node}.input1Y")
        mc.connectAttr(f"{vector_attr}Z", f"{scale_node}.input1Z")
        if stretch and tangent_scale_attr is not None and is_same_axis(axis, primary_axis):
            scalar_to_connect: str = tangent_scale_attr
        else:
            scalar_to_connect: str = scale_attr
        mc.connectAttr(scalar_to_connect, f"{scale_node}.input2X")
        mc.connectAttr(scalar_to_connect, f"{scale_node}.input2Y")
        mc.connectAttr(scalar_to_connect, f"{scale_node}.input2Z")
        return scale_node

    # Create Nodes to re-apply scale
    X_AXIS = (1, 0, 0)
    Y_AXIS = (0, 1, 0)
    Z_AXIS = (0, 0, 1)

    x_scaled: str = scale_vector(
        node_name=f"{segment_name}_xScale",
        vector_attr=aim_matrix_row1.output,
        scale_attr=f"{blended_matrix_row1.output}W",
        axis=X_AXIS,
    )
    y_scaled: str = scale_vector(
        node_name=f"{segment_name}_yScale",
        vector_attr=aim_matrix_row2.output,
        scale_attr=f"{blended_matrix_row2.output}W",
        axis=Y_AXIS,
    )
    z_scaled: str = scale_vector(
        node_name=f"{segment_name}_zScale",
        vector_attr=aim_matrix_row3.output,
        scale_attr=f"{blended_matrix_row3.output}W",
        axis=Z_AXIS,
    )

    # Rebuild the matrix
    output_matrix = mc.createNode("fourByFourMatrix", name=f"{segment_name}_OutputMatrix")
    mc.connectAttr(f"{x_scaled}.outputX", f"{output_matrix}.in00")
    mc.connectAttr(f"{x_scaled}.outputY", f"{output_matrix}.in01")
    mc.connectAttr(f"{x_scaled}.outputZ", f"{output_matrix}.in02")

    mc.connectAttr(f"{y_scaled}.outputX", f"{output_matrix}.in10")
    mc.connectAttr(f"{y_scaled}.outputY", f"{output_matrix}.in11")
    mc.connectAttr(f"{y_scaled}.outputZ", f"{output_matrix}.in12")

    mc.connectAttr(f"{z_scaled}.outputX", f"{output_matrix}.in20")
    mc.connectAttr(f"{z_scaled}.outputY", f"{output_matrix}.in21")
    mc.connectAttr(f"{z_scaled}.outputZ", f"{output_matrix}.in22")

    mc.connectAttr(f"{blended_matrix_row4.output}X", f"{output_matrix}.in30")
    mc.connectAttr(f"{blended_matrix_row4.output}Y", f"{output_matrix}.in31")
    mc.connectAttr(f"{blended_matrix_row4.output}Z", f"{output_matrix}.in32")

    mc.connectAttr(f"{output_matrix}.output", f"{pinned_transform}.offsetParentMatrix")


def matrix_spline_from_transforms(
    transforms: list[str],
    transforms_to_pin: list[str],
    name: str,
    periodic: bool = False,
    degree: int = 3,
    knots: list[str] | None = None,
    padded: bool = True,
    parent: str | None = None,
    stretch: bool = True,
    arc_length: bool = True,
    spline_group: str | None = None,
    primary_axis: tuple[int, int, int] | None = (0, 1, 0),
    secondary_axis: tuple[int, int, int] | None = (0, 0, 1),
) -> MatrixSpline:
    """
    Takes a set of transforms (cvs) and creates a matrix spline with controls and deformation joints.
    Args:
        curve: The curve transform.
        segments: Number of matrices to pin to the curve.
        periodic: Whether the given transforms form a periodic curve or not (no need for repeated CVs)
        degree: Degree of the spline to be created.
        knots: The knot vector for the generated B-Spline.
        padded: When True, segments are sampled such that the end points have half a segment of spacing from the ends of the spline.
        name: Name of the matrix spline group to be created.
        parent: Parent for the newly created matrix spline group.
        stretch: Whether to apply automatic scaling along the spline tangent.
        arc_length: When True, the parameters for the spline will be even according to arc length.
        spline_group: The container group for all the generated subcontrols and joints.
        primary_axis (tuple[int, int, int], optional): Local axis of the pinned
            transform that should aim down the spline tangent. Must be one of
            the cardinal axes (±X, ±Y, ±Z). Defaults to (0, 1, 0) (the +Y axis).
        secondary_axis (tuple[int, int, int], optional): Local axis of the pinned
            transform that should be aligned to a secondary reference direction
            from the spline. Used to resolve orientation. Must be one of the
            cardinal axes (±X, ±Y, ±Z) and orthogonal to ``primary_axis``.
            Defaults to (0, 0, 1) (the +Z axis).
    Returns:
        matrix_spline: The resulting matrix spline.
    """
    segments = len(transforms_to_pin)
    num_cvs: int = len(transforms)
    if not knots:
        if periodic:
            knots = generate_knots(num_cvs + degree, degree=degree, periodic=True)
        else:
            knots = generate_knots(num_cvs, degree=degree)
    cv_positions: list[Vector3] = []

    for transform in transforms:
        position = mc.xform(transform, query=True, worldSpace=True, translation=True)
        cv_positions.append(Vector3(*position))
    if not spline_group:
        if not parent:
            if mc.listRelatives(transforms[0], parent=True):
                curve_parent: str = mc.listRelatives(transforms[0], parent=True)[0]
            else:
                curve_parent: str = None
            if curve_parent:
                container_group: str = mc.group(
                    empty=True, parent=curve_parent, name=f"{name}_MatrixSpline_GRP"
                )
            else:
                container_group: str = mc.group(empty=True, name=f"{name}_MatrixSpline_GRP")
        else:
            container_group: str = mc.group(
                empty=True, parent=parent, name=f"{name}_MatrixSpline_GRP"
            )
    else:
        container_group = spline_group

    mch_group: str = mc.group(empty=True, parent=container_group, name=f"{name}_MCH")

    # Create CV Transforms
    cv_transforms: list[str] = []
    for i, transform in enumerate(transforms):
        cv_transform: str = mc.group(name=f"{name}_CV{i}", empty=True)
        matrix_constraint(transform, cv_transform, keep_offset=False)
        cv_transforms.append(cv_transform)
        mc.parent(cv_transform, mch_group)

    matrix_spline: MatrixSpline = MatrixSpline(
        cv_transforms=cv_transforms, degree=degree, periodic=periodic, name=name, knots=knots
    )

    extended_cv_positions: list[Vector3] = list(cv_positions)

    if periodic:
        for i in range(degree):
            extended_cv_positions.append(cv_positions[i])

    segment_parameters: list[float] = resample(
        cv_positions=extended_cv_positions,
        number_of_points=segments,
        degree=degree,
        knots=knots,
        periodic=periodic,
        padded=padded,
        arc_length=arc_length,
        normalize_parameter=False,
    )

    for index, segment in enumerate(transforms_to_pin):
        segment_pin: str = mc.group(name=f"{segment}_Pin", empty=True, parent=spline_group)
        matrix_constraint(segment_pin, segment, keep_offset=False)
        pin_to_matrix_spline(
            matrix_spline=matrix_spline,
            pinned_transform=segment_pin,
            parameter=segment_parameters[index],
            stretch=stretch,
            primary_axis=primary_axis,
            secondary_axis=secondary_axis,
            normalize_parameter=False,
        )
    return matrix_spline
