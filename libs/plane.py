from typing import Literal, Sequence

import maya.cmds as cmds
import numpy as np
from maya.api.OpenMaya import MPoint, MVector


def points_to_array(points: Sequence[MPoint]) -> np.ndarray[tuple[int, Literal[3]], np.float64]:
    array = np.empty((len(points), 3), dtype=np.float64)
    for i, point in enumerate(points):
        array[i] = (point.x, point.y, point.z)
    return array


def project_points_to_plane(
    point_array: np.ndarray[tuple[int, Literal[3]], np.float64],
    plane_normal: np.ndarray[tuple[Literal[3]], np.float64],
    plane_offset: np.ndarray[tuple[Literal[3]], np.float64],
) -> tuple[np.ndarray[tuple[int, Literal[3]], np.float64], np.ndarray[tuple[int], np.float64]]:
    """
    Returns points that have been moved to the plane defined as a normal and position.
    Args:
        point_array: (N, 3) array of [x, y, z]
    Returns:
        array of [x, y, z] points, and array of the distances the points were projected from.
    """
    

    # signed distance of each point to plane
    distances = point_array @ plane_normal + plane_offset

    # subtract the distance along the normal to place the point on the plane.
    projected = point_array - distances[:, np.newaxis] * plane_normal

    return projected, distances


def fit_plane(
    point_array: np.ndarray[tuple[int, Literal[3]], np.float64],
) -> tuple[np.ndarray[tuple[Literal[3]], np.float64], np.ndarray[tuple[Literal[3]], np.float64]]:
    """
    Fit a plane to points minimizing orthogonal distance using SVD.
    Args:
        point_array: (N, 3) array of [x, y, z]
    Returns:
        normal: unit normal vector of the plane
        d: plane offset for implicit equation n·x + d = 0
    """
    centroid: np.ndarray[tuple[Literal[3]], np.float64] = point_array.mean(axis=0)

    centered_points_array: np.ndarray[tuple[int, Literal[3]], np.float64] = point_array - centroid

    _, _, vt = np.linalg.svd(centered_points_array)
    normal: np.ndarray[tuple[Literal[3]], np.float64] = vt[-1]
    normal /= np.linalg.norm(normal)

    d: np.ndarray[tuple[Literal[3]], np.float64] = -normal @ centroid

    return normal, d


def make_points_planar(points: Sequence[MPoint]) -> tuple[list[MPoint], MVector, float]:
    """
    Takes a sequence of MPoints and returns a List of MPoints that are fitted to a plane, the plane normal, and the max error of the fitting.
    """
    point_array = points_to_array(points)
    plane_normal, plane_offset = fit_plane(point_array)
    plane_normal_vector: MVector = MVector(plane_normal[0], plane_normal[1], plane_normal[2])
    projected_points_array, distances = project_points_to_plane(point_array, plane_normal, plane_offset)
    max_distance: float = np.max(distances)
    return (
        [MPoint(point[0], point[1], point[2]) for point in projected_points_array],
        plane_normal_vector,
        max_distance,
    )


def put_transforms_on_plane(transforms: Sequence[str]):
    """
    Test function for checking plane fit math.
    """
    worldspace_points: list[MPoint] = [
        MPoint(cmds.xform(transform, q=True, worldSpace=True, t=True)) for transform in transforms
    ]
    planar_points, normal, error = make_points_planar(worldspace_points)
    for transform, point in zip(transforms, planar_points):
        cmds.xform(
            transform, worldSpace=True, translation=[point.x, point.y, point.z], relative=False
        )
