import maya.cmds as mc
import maya.api.OpenMaya as om
import numpy as np
import json


def build_plane_from_selected_verts():

    verts = mc.filterExpand(mc.ls(sl=True, fl=True), sm=31)
    if not verts:
        mc.warning("Select verts.")
        return

    # -----------------------------
    # Get positions
    # -----------------------------

    points = []

    for v in verts:
        pos = mc.xform(v, q=True, ws=True, t=True)
        points.append(pos)

    points = np.array(points)

    # -----------------------------
    # center
    # -----------------------------

    center = points.mean(axis=0)

    # -----------------------------
    # best-fit plane
    # -----------------------------

    cov = np.cov(points.T)
    eigvals, eigvecs = np.linalg.eig(cov)

    normal = eigvecs[:, eigvals.argmin()]
    normal = om.MVector(*normal).normalize()

    # -----------------------------
    # build axes
    # -----------------------------

    up = om.MVector(0,1,0)

    if abs(normal * up) > 0.99:
        up = om.MVector(1,0,0)

    tangent = normal ^ up
    tangent.normalize()

    bitangent = normal ^ tangent
    bitangent.normalize()

    # -----------------------------
    # plane-space bounds
    # -----------------------------

    u_vals = []
    v_vals = []

    center_vec = om.MVector(*center)

    for p in points:

        vec = om.MVector(*p) - center_vec

        u_vals.append(vec * tangent)
        v_vals.append(vec * bitangent)

    width = max(u_vals) - min(u_vals)
    height = max(v_vals) - min(v_vals)

    # -----------------------------
    # create plane with center loops
    # -----------------------------

    plane = mc.polyPlane(
        w=width,
        h=height,
        sx=2,
        sy=2
    )[0]

    # -----------------------------
    # build transform matrix
    # -----------------------------

    matrix = [
        tangent.x, tangent.y, tangent.z, 0,
        normal.x, normal.y, normal.z, 0,
        bitangent.x, bitangent.y, bitangent.z, 0,
        center[0], center[1], center[2], 1
    ]

    mc.xform(plane, matrix=matrix)

    # -----------------------------
    # store attrs
    # -----------------------------

    orient_data = [
        [tangent.x, tangent.y, tangent.z],
        [normal.x, normal.y, normal.z],
        [bitangent.x, bitangent.y, bitangent.z]
    ]

    size_data = [width, height]

    if not mc.attributeQuery("guideOrient", n=plane, ex=True):
        mc.addAttr(plane, ln="guideOrient", dt="string")

    if not mc.attributeQuery("guideSize", n=plane, ex=True):
        mc.addAttr(plane, ln="guideSize", dt="string")

    mc.setAttr(plane + ".guideOrient", json.dumps(orient_data), type="string")
    mc.setAttr(plane + ".guideSize", json.dumps(size_data), type="string")

    print("Created guide plane:", plane)

    return plane
                
build_plane_from_selected_verts()