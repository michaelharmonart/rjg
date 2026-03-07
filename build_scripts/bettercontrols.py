import json
import os
from enum import Enum
from pathlib import Path
from typing import Any, Literal

import maya.cmds as cmds
from maya.api.OpenMaya import (
    MDoubleArray,
    MFnNurbsCurve,
    MItGeometry,
    MPoint,
    MPointArray,
    MSelectionList,
    MSpace,
)

def get_shapes(transform: str) -> list[str]:
    # list the shapes of node
    shape_list: list[str] = cmds.listRelatives(
        transform, shapes=True, noIntermediate=True, children=True
    )
    return shape_list


def get_cv_positions(curve_shape: str) -> list[tuple[float, float, float]]:
    """
    Gets the positions of all CVs for a given curve shape.
    Args:
        curve_shape(str): Name of curve shape node.
    Returns:
        list: A list of CV positions as tuples
    """
    sel: MSelectionList = MSelectionList()
    sel.add(curve_shape)
    curve_obj = sel.getDependNode(0)
    fn_curve: MFnNurbsCurve = MFnNurbsCurve(curve_obj)

    cv_positions: MPointArray = fn_curve.cvPositions(space=MSpace.kObject)
    positions: list[tuple[float, float, float]] = [
        (point.x, point.y, point.z) for point in cv_positions
    ]
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
    curve_obj = sel.getDependNode(0)
    fn_curve: MFnNurbsCurve = MFnNurbsCurve(curve_obj)

    cv_positions: MPointArray = fn_curve.cvPositions(space=MSpace.kObject)
    weights: list[float] = [point.w for point in cv_positions]
    return weights


def get_cv_data(curve_shape: str) -> tuple[list[tuple[float, float, float]], list[float]]:
    """
    Gets both the positions and weights of all CVs for a given curve shape.
    Args:
        curve_shape (str): Name of curve shape node.
    Returns:
        tuple: (positions, weights)
            positions (list[tuple[float, float, float]]): List of CV positions
            weights (list[float]): List of CV weights
    """
    sel: MSelectionList = MSelectionList()
    sel.add(curve_shape)
    curve_obj = sel.getDependNode(0)
    fn_curve: MFnNurbsCurve = MFnNurbsCurve(curve_obj)

    cv_positions: MPointArray = fn_curve.cvPositions(space=MSpace.kObject)
    positions: list[tuple[float, float, float]] = [
        (point.x, point.y, point.z) for point in cv_positions
    ]
    weights: list[float] = [point.w for point in cv_positions]

    return positions, weights


def get_knots(curve_shape: str) -> list[float]:
    """
    Gets the knot vector for a given curve shape.
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
    return knots


def get_curve_info(curve: str):
    curve_dict = {}
    for curve in get_shapes(transform=curve):
        degree = cmds.getAttr(curve + ".degree")
        form = cmds.getAttr(curve + ".form")
        cv_positions: list[tuple[float, float, float]]
        cv_weights: list[float]
        cv_positions, cv_weights = get_cv_data(curve_shape=curve)
        knots: list[float] = get_knots(curve_shape=curve)
        draw_on_top: bool = cmds.getAttr(f"{curve}.alwaysDrawOnTop")
        curve_info = {
            "degree": degree,
            "form": form,
            "cv_positions": cv_positions,
            "cv_weights": cv_weights,
            "knots": knots,
            "draw_on_top": draw_on_top,
        }
        curve_dict[curve] = curve_info
    return curve_dict

def get_curve_info(curve: str):
    curve_dict = {}
    for curve in get_shapes(transform=curve):
        degree = cmds.getAttr(curve + ".degree")
        form = cmds.getAttr(curve + ".form")
        cv_positions: list[tuple[float, float, float]]
        cv_weights: list[float]
        cv_positions, cv_weights = get_cv_data(curve_shape=curve)
        knots: list[float] = get_knots(curve_shape=curve)
        draw_on_top: bool = cmds.getAttr(f"{curve}.alwaysDrawOnTop")
        curve_info = {
            "degree": degree,
            "form": form,
            "cv_positions": cv_positions,
            "cv_weights": cv_weights,
            "knots": knots,
            "draw_on_top": draw_on_top,
        }
        curve_dict[curve] = curve_info
    return curve_dict


def get_tagged_controls() -> list[str]:
    """
    Returns all transform nodes tagged as controllers via a connected controller node.

    Returns:
        list: A list of transform node names that are tagged as controllers.
    """
    controller_nodes: list[str] = cmds.ls(type="controller")
    tagged_controls: list[str] = []
    for control_node in controller_nodes:
        connected: list[str] = cmds.listConnections(
            f"{control_node}.controllerObject", source=True, destination=False
        )
        if connected[0] in ['hand_L_01_CTRL', 'hand_R_01_CTRL', 'foot_L_01_L_CTRL', 'foot_R_01_R_CTRL', 'COG_M_CTRL', ]:
            tagged_controls.append(connected[0])

    return tagged_controls


def write_control_shapes(filepath: str, force: bool = False) -> None:
    """
    Writes a JSON file representing the control curves of all controls in the scene at the filepath specified.

    Args:
        filepath: The path and filename and extension to save under.
        force: If True, will automatically overwrite any existing file at the filepath specified.

    """

    # If the file exists, only write it if force = True, or after asking for confirmation.
    if os.path.isfile(path=filepath):
        if force:
            pass
        else:
            confirm: str = cmds.confirmDialog(
                title="File Overwrite",
                message=f"{filepath} already exists and will be overwritten, are you sure you want to write the file?",
                button=["Yes", "No"],
                defaultButton="Yes",
                cancelButton="No",
                dismissString="No",
            )
            if confirm == "Yes":
                pass
            else:
                return

    control_dict = {}

    controls: list[str] = get_tagged_controls()
    for control in controls:
        curve_info = get_curve_info(curve=control)
        control_dict[control] = curve_info

    json_path: str = filepath
    json_dump: str = json.dumps(obj=control_dict, indent=4)

    with open(file=json_path, mode="w") as json_file:
        json_file.write(json_dump)
        json_file.close()
    return


def apply_control_file(filepath: str) -> None:
    if not os.path.isfile(path=filepath):
        raise RuntimeError(f"{filepath} is not a valid file. Unable to load controls")
    current_control_dict = {}
    controls = get_tagged_controls()
    for control in controls:
        current_control_dict[control] = True

    control_dict = {}
    with open(filepath, "r") as json_file:
        json_data = json_file.read()
        control_dict = json.loads(json_data)
    for control in control_dict:
        if control in current_control_dict:
            shapes: list[str] = get_shapes(transform=control)
            try:
                for shape in shapes:
                    attr = f"{shape}.alwaysDrawOnTop"
                    #incoming = cmds.listConnections(attr, source=True, destination=False, plugs=True)
                    src = cmds.connectionInfo(attr, sourceFromDestination=True)
                    cmds.delete(shape)
                curve_data = control_dict[control]
                for index, shape in enumerate(curve_data):
                    info = curve_data[shape]
                    positions: list[tuple[float, float, float]] = info["cv_positions"]
                    degree: int = info["degree"]
                    periodic: bool = True if info["form"] == 2 else False
                    knots: list[float] = info["knots"]
                    weights: list[float] = info["cv_weights"]
                    draw_on_top: bool = info["draw_on_top"]
                    position_weights: list[tuple[float, float, float, float]] = [
                        (position[0], position[1], position[2], weights[index])
                        for index, position in enumerate(positions)
                    ]

                    child_curve_transform: str = cmds.curve(
                        pointWeight=position_weights, knot=knots, periodic=periodic, degree=degree
                    )
                    curve_shape_node: str = get_shapes(child_curve_transform)[0]
                    curve_shape_node = cmds.rename(curve_shape_node, shape)
                    xray_attr = f"{curve_shape_node}.alwaysDrawOnTop"
                    cmds.setAttr(f"{curve_shape_node}.alwaysDrawOnTop", 1 if draw_on_top else 0)
                    if src:
                        cmds.connectAttr(src, xray_attr)
                    cmds.parent(curve_shape_node, control, shape=True, relative=True)
                    cmds.delete(child_curve_transform)
            except:
                pass