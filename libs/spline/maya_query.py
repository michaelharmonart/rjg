import maya.cmds as cmds
from maya.api.OpenMaya import MDoubleArray, MFnNurbsCurve, MPointArray, MSelectionList, MSpace
from rjg.libs.spline.math import Vector3


def maya_to_standard_knots(
    knots: list[float], degree: int = 3, periodic: bool = False
) -> list[float]:
    # Refer to https://openusd.org/dev/api/class_usd_geom_nurbs_curves.html#details
    # The above only works with uniform knots, so this is generalized to higher order and non-uniform knots
    # based on info found here https://developer.rhino3d.com/guides/opennurbs/periodic-curves-and-surfaces/
    # Although there is a typo in the above doc, k[(cv_count)+i] should be k[(cv_count - 1)+i]
    # Don't ask how long it took me to find that out
    """
    Convert Maya-style knot vector to a 'standard' knot vector.

    Args:
        knots (list[float]): Input knot sequence from Maya.
        degree (int, optional): Degree of the curve. Defaults to 3.
        periodic (bool, optional): Whether the curve is periodic. Defaults to False.

    Returns:
        list[float]: Adjusted knot sequence.
    """

    new_knots: list[float] = knots.copy()

    # add placeholders for first/last values
    new_knots.insert(0, 0.0)
    new_knots.append(0.0)

    # A cubic periodic knot vector looks like: [a,b,c,d,e, ...,  p+a,p+b,p+c,p+d,p+e]
    # offset is the length of the repeated indices, in the above case it would be 5
    # (degree is multiplied by 2 since degree is both part of the iterator and the indexing equation):
    # -degree < i < degree (max i here is used to calculate p)
    # k[(degree-1)+i+1] - k[(degree-1)+i] = k[(cv_count-1)+i+1] - k[(cv_count-1)+i]
    # (degree-1)+i and (cv_count-1)+i can now both be substituted with the offset

    offset: int = (degree * 2) - 1

    if periodic:
        new_knots[0] = new_knots[1] - (new_knots[-(offset - 1)] - new_knots[-offset])
        new_knots[-1] = new_knots[-2] + (new_knots[offset] - new_knots[offset - 1])
    else:
        new_knots[0] = new_knots[1]
        new_knots[-1] = new_knots[-2]
    return new_knots


def get_knots(curve_shape: str) -> list[float]:
    """
    Gets the standard knot vector for a given curve shape (not the Maya format).
    Args:
        curve_shape(str): Name of curve shape node.
    Returns:
        list: A list of knot values. (aka knot vector)
    """

    sel: MSelectionList = MSelectionList()
    sel.add(curve_shape)
    curve_obj = sel.getDependNode(0)
    fn_curve: MFnNurbsCurve = MFnNurbsCurve(curve_obj)

    knots_array: MDoubleArray = fn_curve.knots()
    knots: list[float] = [knot for knot in knots_array]

    # Now convert the knot vector from the Maya form to the standard form by filling out the missing values.
    degree: int = cmds.getAttr(f"{curve_shape}.degree")
    if cmds.getAttr(f"{curve_shape}.form") == 2:
        periodic: bool = True
    else:
        periodic: bool = False
    knots: list[float] = maya_to_standard_knots(knots=knots, degree=degree, periodic=periodic)
    return knots


def get_cvs(curve_shape: str) -> list[Vector3]:
    """
    Gets the positions of all CVs for a given curve shape.
    Args:
        curve_shape(str): Name of curve shape node.
    Returns:
        list: A list of CV positions as Vector3s
    """
    sel: MSelectionList = MSelectionList()
    sel.add(curve_shape)
    dag_path = sel.getDagPath(0)
    fn_curve: MFnNurbsCurve = MFnNurbsCurve(dag_path)

    cv_positions: MPointArray = fn_curve.cvPositions(space=MSpace.kWorld)
    positions: list[Vector3] = [Vector3(point.x, point.y, point.z) for point in cv_positions]
    return positions


def get_cv_weights(curve_shape: str) -> list[float]:
    """
    Gets the weights of all CVs for a given curve shape.
    Args:
        curve_shape(str): Name of curve shape node.
    Returns:
        list: A list of CV weight values.
    """
    sel: MSelectionList = MSelectionList()
    sel.add(curve_shape)
    dag_path = sel.getDagPath(0)
    fn_curve: MFnNurbsCurve = MFnNurbsCurve(dag_path)

    cv_positions: MPointArray = fn_curve.cvPositions(space=MSpace.kWorld)
    weights: list[float] = [point.w for point in cv_positions]
    return weights
