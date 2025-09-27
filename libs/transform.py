import maya.cmds as mc
from importlib import reload
from collections import OrderedDict

'''
moves all transform attributes of node to those specified

param node: node to transform
param translate: object, list, or tuple. if list or tuple, must be of length 3 (x, y, z) and translate will be changed to those values in world space
                 if object, node will inherit all worldspace translate values of the object
param rotate: same as translate, but applied to node's rotation
param scale: same as translate, but applied to node's scale
'''
def match_pose(node, translate=None, rotate=None, scale=None):
    #print(type(translate))

    if isinstance(translate, list) or isinstance(translate, tuple):
        if len(translate) == 3:
            mc.setAttr(node + ".translate", *translate)
        else:
            mc.error("Please provide x, y, z translate values.")
    elif not translate:
        pass
    elif mc.objExists(translate):
        src = mc.xform(translate, query=True, worldSpace=True, translation=True)
        mc.xform(node, worldSpace=True, translation=src)
    else:
        mc.error("Input for translate not valid. Please give coordinates or provide a valid object.")

    if isinstance(rotate, list) or isinstance(rotate, tuple):
        if len(rotate) == 3:
            mc.setAttr(node + ".rotate", *rotate)
        else:
            mc.error("Please provide x, y, z rotate values.")
    elif not rotate:
        pass
    elif mc.objExists(rotate):
        src = mc.xform(rotate, query=True, worldSpace=True, rotation=True)
        mc.xform(node, worldSpace=True, rotation=src)
    else:
        mc.error("Input for rotate not valid. Please give coordinates or provide a valid object.")

    if isinstance(scale, list) or isinstance(scale, tuple):
        if len(scale) == 3:
            mc.setAttr(node + ".scale", *scale)
        else:
            mc.error("Please provide x, y, z scale values.")
    elif not scale:
        pass
    elif mc.objExists(scale):
        src = mc.xform(scale, query=True, worldSpace=True, scale=True)
        mc.xform(node, worldSpace=True, scalen=src)
    else:
        mc.error("Input for scale not valid. Please give coordinates or provide a valid object.")

'''
populates an OrderedDict with {node : world space matrix}
'''
def read_pose(nodes):
    if not isinstance(nodes, list):
        nodes = [nodes]
    pose_dict = OrderedDict()

    for node in nodes:
        pose_dict[node] = mc.xform(node, q=True, worldSpace=True, matrix=True)
    return pose_dict

'''
sets worldspace matrix of an object
'''
def set_pose(node, matrix):
    mc.xform(node, worldSpace=True, matrix=matrix)

'''
given a curve and a percentage along the curve, return the worldspace position of that point on the curve
'''
def findPosOnCurve(curve, u_val):
    pci = mc.createNode("pointOnCurveInfo", n='tmp_pci')
    mc.connectAttr(curve + 'Shape.worldSpace[0]', pci + '.inputCurve')
    mc.setAttr(pci + '.turnOnPercentage', 1)
    mc.setAttr(pci + '.parameter', u_val)
    pos = mc.getAttr(pci + '.position')[0]
    mc.delete(pci)
    return pos

def is_identity_matrix(matrix: list[float], epsilon: float = 0.001) -> bool:
    return all(
        abs(value - identity) < epsilon
        for value, identity in zip(matrix, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
    )

def matrix_constraint(
    source_transform: str,
    constrain_transform: str,
    keep_offset: bool = True,
    local_space: bool = True,
    translate: bool = True,
) -> None:
    """
    Constrain a transform to another

    Args:
        source_transform: joint to match.
        constrain_joint: joint to constrain.
        keep_offset: keep the offset of the constrained transform to the source at time of constraint generation.
        local_space: if False the constrained transform will have inheritsTransform turned off.
        translate: whether to constrain translation.
    """
    constraint_name: str = constrain_transform.split("|")[-1]

    # Create node to multiply matrices, as well as a counter to make sure to input into the right slot.
    mult_index: int = 0
    mult_matrix: str = mc.createNode("multMatrix", name=f"{constraint_name}_ConstraintMultMatrix")

    # If we want to keep the offset, we put the position of the constrained transform into
    # the source transform's space and record it.
    if keep_offset:
        # Get the offset matrix
        offset_matrix_node: str = mc.createNode(
            "multMatrix", name=f"{constraint_name}_OffsetMatrix"
        )
        mc.connectAttr(
            f"{constrain_transform}.worldMatrix[0]", f"{offset_matrix_node}.matrixIn[0]"
        )
        mc.connectAttr(
            f"{source_transform}.worldInverseMatrix[0]", f"{offset_matrix_node}.matrixIn[1]"
        )
        offset_matrix = mc.getAttr(f"{offset_matrix_node}.matrixSum")

        # Check the matrix against an identity matrix. If it's the same within a margin of error,
        # the transforms aren't offset, meaning we can skip that extra matrix multiplication.
        if not is_identity_matrix(matrix=offset_matrix):
            # Put the offset into the matrix multiplier
            mc.setAttr(f"{mult_matrix}.matrixIn[{mult_index}]", offset_matrix, type="matrix")
            mult_index += 1
        else:
            keep_offset = False

        mc.delete(offset_matrix_node)

    # Next we multiply by the world matrix of the source transform
    mc.connectAttr(f"{source_transform}.worldMatrix[0]", f"{mult_matrix}.matrixIn[{mult_index}]")
    mult_index += 1

    # If we have a parent transform we then put it into that space by multiplying by it's worldInverseMatrix
    if local_space:
        mc.connectAttr(
            f"{constrain_transform}.parentInverseMatrix[0]", f"{mult_matrix}.matrixIn[{mult_index}]"
        )
        mult_index += 1
    else:
        mc.setAttr(f"{constrain_transform}.inheritsTransform", 0)

    # Create the decomposed matrix and connect it's inputs
    decompose_matrix: str = mc.createNode(
        "decomposeMatrix", name=f"{constraint_name}_ConstrainMatrixDecompose"
    )
    mc.connectAttr(f"{mult_matrix}.matrixSum", f"{decompose_matrix}.inputMatrix")
    mc.connectAttr(f"{constrain_transform}.rotateOrder", f"{decompose_matrix}.inputRotateOrder")

    rotate_attr: str = f"{decompose_matrix}.outputRotate"
    # If it's a joint we have to do a whole bunch of other nonsense to account for joint orient (I was up till 2am because of this)
    if mc.nodeType(constrain_transform) == "joint":
        mc.setAttr(f"{constrain_transform}.jointOrient", 0, 0, 0, type="float3")
    mc.setAttr(f"{constrain_transform}.rotateAxis", 0, 0, 0, type="float3")
    mc.connectAttr(rotate_attr, f"{constrain_transform}.rotate")
    if translate:
        mc.connectAttr(f"{decompose_matrix}.outputTranslate", f"{constrain_transform}.translate")
    mc.connectAttr(f"{decompose_matrix}.outputScale", f"{constrain_transform}.scale")
    mc.connectAttr(f"{decompose_matrix}.outputShear", f"{constrain_transform}.shear")


def freeze_and_zero(transform: str) -> None:
    mc.makeIdentity(transform, apply=True)
    mc.xform(pivots=(0, 0, 0))