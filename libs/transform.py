from cgi import test
from collections import OrderedDict
from collections.abc import Sequence

import maya.cmds as mc
from maya.api.OpenMaya import (
    MAngle,
    MDagPath,
    MEulerRotation,
    MFnTransform,
    MMatrix,
    MPoint,
    MSelectionList,
    MSpace,
    MTransformationMatrix,
    MVector,
)

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
        mc.xform(node, worldSpace=True, scale=src)
    else:
        mc.error("Input for scale not valid. Please give coordinates or provide a valid object.")


def read_pose(nodes: Sequence[str] | str) -> OrderedDict[str, list[float]]:
    '''
    populates an OrderedDict with {node : world space matrix}
    '''    
    
    if isinstance(nodes, str):
        nodes = [nodes]
    pose_dict = OrderedDict()

    for node in nodes:
        pose_dict[node] = mc.xform(node, q=True, worldSpace=True, matrix=True)
    return pose_dict


def set_pose(node: str, matrix: list[float]):
    '''
    sets worldspace matrix of an object
    '''
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

def is_identity_matrix(matrix: list[float] | MMatrix, epsilon: float = 0.001) -> bool:
    if isinstance(matrix, MMatrix):
        return matrix.isEquivalent(MMatrix.kIdentity, epsilon)
    return all(
        abs(value - identity) < epsilon
        for value, identity in zip(matrix, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
    )

def get_local_matrix(transform: str) -> MMatrix:
    """
    Returns the local matrix of a transform.
    """
    selection = MSelectionList()
    selection.add(transform)
    dag_path: MDagPath = selection.getDagPath(0)
    mfn_transform: MFnTransform = MFnTransform(dag_path)
    transformation: MTransformationMatrix = mfn_transform.transformation()
    return transformation.asMatrix()
    
def set_local_matrix(transform: str, matrix: MMatrix, fallback=False) -> None:
    """
    Set the local matrix of a transform by decomposing it into components.

    Args:
        transform: Maya transform node name.
        matrix: Target matrix.
        fallback: If True, use cmds.xform instead of manual decomposition.
    """
    if fallback:
        mc.xform(transform, worldSpace=False, matrix=matrix)
    else:
        # Apply local matrix using transformation matrix
        transform_matrix: MTransformationMatrix = MTransformationMatrix(matrix)
        # Set translation
        translation = transform_matrix.translation(MSpace.kTransform)
        mc.setAttr(f"{transform}.translate", translation.x, translation.y, translation.z)
        node_type = mc.nodeType(transform)

        if node_type == "joint":
            # Zero the rotate channel
            mc.setAttr(f"{transform}.rotate", 0, 0, 0)
            rotation = transform_matrix.rotation()
            mc.setAttr(
                f"{transform}.jointOrient",
                MAngle(rotation.x).asDegrees(),
                MAngle(rotation.y).asDegrees(),
                MAngle(rotation.z).asDegrees(),
            )
        else:
            rotate_order = mc.getAttr(f"{transform}.rotateOrder")
            transform_matrix.reorderRotation(rotate_order + 1)
            rotation = transform_matrix.rotation()
            mc.setAttr(
                f"{transform}.rotate",
                MAngle(rotation.x).asDegrees(),
                MAngle(rotation.y).asDegrees(),
                MAngle(rotation.z).asDegrees(),
            )

        # Set scale
        scale = transform_matrix.scale(MSpace.kTransform)
        mc.setAttr(f"{transform}.scale", scale[0], scale[1], scale[2])

        # Set shear
        shear = transform_matrix.shear(MSpace.kTransform)
        mc.setAttr(f"{transform}.shear", shear[0], shear[1], shear[2])
        

def get_world_matrix(transform: str) -> MMatrix:
    """
    Returns the full world matrix of a transform, including rotateAxis, jointOrient, etc.
    Equivalent to Maya's internal world matrix.
    """
    selection = MSelectionList()
    selection.add(transform)
    dag_path: MDagPath = selection.getDagPath(0)
    return dag_path.inclusiveMatrix()


def get_parent_matrix(transform: str) -> MMatrix:
    """
    Returns the world matrix of a transform's parent, including rotateAxis, jointOrient, etc.
    """
    selection = MSelectionList()
    selection.add(transform)
    dag_path: MDagPath = selection.getDagPath(0)
    return dag_path.exclusiveMatrix()


def get_parent_inverse_matrix(transform: str) -> MMatrix:
    """
    Returns the inverse world matrix of a transform's parent, including rotateAxis, jointOrient, etc.
    """
    selection = MSelectionList()
    selection.add(transform)
    dag_path: MDagPath = selection.getDagPath(0)
    return dag_path.exclusiveMatrixInverse()


def get_matrix_values(matrix: MMatrix) -> list[float]:
    return [matrix[i] for i in range(16)]

def set_world_matrix(transform: str, matrix: MMatrix, fallback=False) -> None:
    """
    Set the world matrix of a transform by decomposing it into local components.

    Args:
        transform: Maya transform node name.
        matrix: Target world space matrix.
        fallback: If True, use cmds.xform instead of manual decomposition.
    """
    if fallback:
        mc.xform(transform, worldSpace=True, matrix=matrix)
    else:

        inverse_matrix: MMatrix = get_parent_inverse_matrix(transform)
        local_matrix: MMatrix = matrix * inverse_matrix
        set_local_matrix(transform=transform, matrix=local_matrix, fallback=False)

def create_aim_matrix(aim_vector: MVector, up_vector: MVector, position: MPoint | None = None) -> MMatrix:
    """
    Aims the y axis of a matrix towards a vector.
    """
    aim_normal: MVector = aim_vector.normal()
    right_normal: MVector = (up_vector ^ aim_vector).normal()
    up_normal: MVector = (aim_vector ^ right_normal).normal()
    
    aim_axis = (aim_normal.x, aim_normal.y, aim_normal.z, 0.0)
    right_axis = (right_normal.x, right_normal.y, right_normal.z, 0.0)
    up_axis = (up_normal.x, up_normal.y, up_normal.z, 0.0)
    if position is not None:
        position_row = (position.x, position.y, position.z, 1.0)
    else:
        position_row = (0.0, 0.0, 0.0, 1.0)
        
    return MMatrix((right_axis, aim_axis, up_axis, position_row))

def match_transform(transform: str, target_transform: str) -> None:
    """
    Match a transform to another in world space.

    Args:
        transform: Object to be moved to the specified transform.
        target_transform: Name of the transform to match to.
    """
    source_matrix: MMatrix = get_world_matrix(transform=target_transform)
    set_world_matrix(transform=transform, matrix=source_matrix)


def matrix_constraint(
    source_transform: str,
    constrain_transform: str,
    keep_offset: bool = True,
    local_space: bool = True,
    translate: bool = True,
    rotate: bool = True,
    scale: bool = True,
    shear: bool = True,
    use_joint_orient: bool = False,
    lock_joint_orient: bool = True,
) -> None:
    """
    Constrain a transform to another

    Args:
        source_transform: joint to match.
        constrain_joint: joint to constrain.
        keep_offset: keep the offset of the constrained transform to the source at time of constraint generation.
        local_space: if False the constrained transform will have inheritsTransform turned off.
        translate: whether to constrain translation.
        lock_joint_orient: When True, if the transform is a joint
            it's joint orient will be locked after being zeroed to keep maya from screwing it up later when re-parenting.
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
    # Drive transform with decomposed values
    # If it's a joint we have to do a whole bunch of other nonsense to account for joint orient
    if mc.nodeType(constrain_transform) == "joint":
        if scale:
            mc.setAttr(f"{constrain_transform}.segmentScaleCompensate", 0)
        if rotate:
            if use_joint_orient:
                # Check if the joint orient isn't about 0
                joint_orient: tuple[float, float, float] = mc.getAttr(
                    f"{constrain_transform}.jointOrient"
                )[0]
                if any(abs(i) > 0.01 for i in joint_orient):
                    # Get our joint orient and turn it into a matrix
                    orient_node: str = mc.createNode(
                        "composeMatrix", name=f"{constraint_name}_OrientMatrix"
                    )
                    mc.connectAttr(
                        f"{constrain_transform}.jointOrient", f"{orient_node}.inputRotate"
                    )
                    orient_matrix_attr = f"{orient_node}.outputMatrix"

                    # We need to compose a different matrix to drive just the rotation due to the joint orient
                    orient_offset_node: str = mc.createNode(
                        "inverseMatrix", name=f"{constraint_name}_OrientOffsetMatrix"
                    )
                    orient_mult_matrix: str = mc.createNode(
                        "multMatrix", name=f"{constraint_name}_ConstraintOrientMatrix"
                    )
                    orient_mult_index: int = 0

                    # If we have an offset it'll be our first matrix in the multiplier (same as above)
                    if keep_offset:
                        mc.setAttr(
                            f"{orient_mult_matrix}.matrixIn[{orient_mult_index}]",
                            offset_matrix,
                            type="matrix",
                        )
                        orient_mult_index += 1

                    # Next we multiply by the world matrix of the source transform
                    mc.connectAttr(
                        f"{source_transform}.worldMatrix[0]",
                        f"{orient_mult_matrix}.matrixIn[{orient_mult_index}]",
                    )
                    orient_mult_index += 1

                    # Depending on if we need to take a parent into account we'll need a few extra nodes
                    # (otherwise just pre-calculate a matrix and plop it in)
                    # Bless Jared Love for figuring this out https://www.youtube.com/watch?v=_LNhZB8jQyo
                    # Essentially we need to take the inverse of the orient * the world matrix of the parent and multiply by that
                    if local_space:
                        # Create a node to multiply the joint orient by the world matrix of the parent
                        orient_parent_mult_matrix: str = mc.createNode(
                            "multMatrix", name=f"{constraint_name}_ConstraintOrientMultMatrix"
                        )
                        mc.connectAttr(orient_matrix_attr, f"{orient_parent_mult_matrix}.matrixIn[0]")
                        mc.connectAttr(
                            f"{constrain_transform}.parentMatrix[0]",
                            f"{orient_parent_mult_matrix}.matrixIn[1]",
                        )

                        # Create an inverse node and connect it to the result of the last step
                        mc.connectAttr(
                            f"{orient_parent_mult_matrix}.matrixSum",
                            f"{orient_offset_node}.inputMatrix",
                        )

                        # Finally add this to a slot on the matrix multiplier node
                        mc.connectAttr(
                            f"{orient_offset_node}.outputMatrix",
                            f"{orient_mult_matrix}.matrixIn[{orient_mult_index}]",
                        )
                        orient_mult_index += 1
                    else:
                        # If we don't care about a parent, just make a temp inverse node and store the inverse of the joint orient
                        mc.connectAttr(
                            f"{orient_node}.outputMatrix", f"{orient_offset_node}.inputMatrix"
                        )
                        inverse_orient_matrix = mc.getAttr(f"{orient_offset_node}.outputMatrix")

                        # And then set it in a slot on the matrix multiplier
                        mc.setAttr(
                            f"{orient_mult_matrix}.matrixIn[{orient_mult_index}]",
                            inverse_orient_matrix,
                            type="matrix",
                        )
                        orient_mult_index += 1
                        # Cleanup temp node
                        mc.delete(orient_offset_node)

                    #  Hook up the matrix multiplier to our decomposeMatrix and feed it into the rotate attribute of the joint
                    orient_decompose_matrix: str = mc.createNode(
                        "decomposeMatrix", name=f"{constraint_name}_ConstrainOrientDecompose"
                    )
                    mc.connectAttr(
                        f"{orient_mult_matrix}.matrixSum", f"{orient_decompose_matrix}.inputMatrix"
                    )
                    mc.connectAttr(
                        f"{constrain_transform}.rotateOrder",
                        f"{orient_decompose_matrix}.inputRotateOrder",
                    )
                    rotate_attr = f"{orient_decompose_matrix}.outputRotate"
            else:
                mc.setAttr(f"{constrain_transform}.jointOrient", 0, 0, 0, type="float3")
                if lock_joint_orient:
                    mc.setAttr(f"{constrain_transform}.jointOrient", lock=True)
        
    if rotate:
        mc.connectAttr(rotate_attr, f"{constrain_transform}.rotate")
        mc.setAttr(f"{constrain_transform}.rotateAxis", 0, 0, 0, type="float3")
    if translate:
        mc.connectAttr(f"{decompose_matrix}.outputTranslate", f"{constrain_transform}.translate")
    if scale:
        mc.connectAttr(f"{decompose_matrix}.outputScale", f"{constrain_transform}.scale")
    if shear:
        mc.connectAttr(f"{decompose_matrix}.outputShear", f"{constrain_transform}.shear")


def freeze_and_zero(transform: str) -> None:
    mc.makeIdentity(transform, apply=True)
    mc.xform(pivots=(0, 0, 0))

def drive_transform_with_matrix(
    matrix_attr: str,
    transform: str,
    translate: bool = True,
    rotate: bool = True,
    scale: bool = True,
    shear: bool = True,
    lock_joint_orient: bool = True,
):
    """
    Drive a transforms translate rotate scale and shear with a matrix attribute.

    Args:
        matrix_attr: The matrix attribute to use as the driver.
        transform: The transform to be driven.
        translate: whether to constrain translation.
        lock_joint_orient: When True, if the transform is a joint
            it's joint orient will be locked after being zeroed to keep maya from screwing it up later when re-parenting.
    """
    constraint_name: str = transform.split("|")[-1]

    # Create the decomposed matrix and connect it's inputs
    decompose_matrix: str = mc.createNode(
        "decomposeMatrix", name=f"{constraint_name}_DriverMatrixDecompose"
    )
    mc.connectAttr(matrix_attr, f"{decompose_matrix}.inputMatrix")
    mc.connectAttr(f"{transform}.rotateOrder", f"{decompose_matrix}.inputRotateOrder")


    # Drive transform with decomposed values
    # If it's a joint we have to do a whole bunch of other nonsense to account for joint orient
    if mc.nodeType(transform) == "joint":
        if scale:
            mc.setAttr(f"{transform}.segmentScaleCompensate", 0)
        if rotate:
            print(f"reset orient on {transform}")
            mc.setAttr(f"{transform}.jointOrient", lock=False)
            mc.setAttr(f"{transform}.jointOrient", 0, 0, 0, type="float3")
            if lock_joint_orient:
                mc.setAttr(f"{transform}.jointOrient", lock=True)
    if rotate:
        mc.connectAttr(f"{decompose_matrix}.outputRotate", f"{transform}.rotate")
        mc.setAttr(f"{transform}.rotateAxis", 0, 0, 0, type="float3")
    if translate:
        mc.connectAttr(f"{decompose_matrix}.outputTranslate", f"{transform}.translate")
    if scale:
        mc.connectAttr(f"{decompose_matrix}.outputScale", f"{transform}.scale")
    if shear:
        mc.connectAttr(f"{decompose_matrix}.outputShear", f"{transform}.shear")


def clean_parent(transform: str, parent: str, joint_orient: bool = True) -> None:
    """
    Parent a node while preserving its world transform without creating
    Maya's intermediate "compensation" transforms.

    - For transforms: world matrix is preserved.
    - For joints (if joint_orient=True): rotation is baked into jointOrient
      and rotate is zeroed, keeping the joint clean for IK/FK.

    Args:
        transform: Node to reparent.
        parent: New parent node.
        joint_orient: If True, bake rotation into jointOrient for joints.
    """
    object_world_matrix: MMatrix = get_world_matrix(transform)
    node_type = mc.nodeType(transform)
    mc.parent(transform, parent, relative=True)

    if node_type == "joint" and joint_orient:
        mc.setAttr(f"{transform}.jointOrient", 0, 0, 0)
        set_world_matrix(transform, object_world_matrix)
        # Get current rotation info
        rotate_order = mc.getAttr(f"{transform}.rotateOrder")
        rotation = mc.getAttr(f"{transform}.rotate")[0]
        # Convert rotation XYZ rotate order for replacing the joint orient
        euler: MEulerRotation = MEulerRotation(
            MAngle(rotation[0], MAngle.kDegrees).asRadians(),
            MAngle(rotation[1], MAngle.kDegrees).asRadians(),
            MAngle(rotation[2], MAngle.kDegrees).asRadians(),
            rotate_order,
        )
        euler.reorderIt(MEulerRotation.kXYZ)
        # Apply to jointOrient (convert back to degrees)
        mc.setAttr(
            f"{transform}.jointOrient",
            MAngle(euler.x).asDegrees(),
            MAngle(euler.y).asDegrees(),
            MAngle(euler.z).asDegrees(),
        )
        # Zero the rotate channel
        mc.setAttr(f"{transform}.rotate", 0, 0, 0)
    else:
        set_world_matrix(transform, object_world_matrix)
