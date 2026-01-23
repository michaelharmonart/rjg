from importlib import reload
from typing import Sequence

import maya.api.OpenMaya as om
import maya.cmds as mc
import rjg.libs.attribute as rAttr
import rjg.libs.math as rMath
from maya.api.OpenMaya import MPoint, MVector
from rjg.libs.plane import make_points_planar

reload(rAttr)
reload(rMath)

def clean_pv_guide(guide_list=None, name=None, suffix=None, slide_pv=None, offset_pv=5):
    if not suffix:
        suffix = 'guide'
    if not name:
        name = guide_list[0] + '_' + suffix

    vecA = om.MVector(mc.xform(guide_list[0], t=True, q=True, ws=True))
    vecC = om.MVector(mc.xform(guide_list[-1], t=True, q=True, ws=True))
    if len(guide_list)%2 == 0:
        vecB = rMath.vec_midpoint(guide_list[len(guide_list)//2], guide_list[(len(guide_list)//2)-1])
    else:
        vecB = om.MVector(mc.xform(guide_list[len(guide_list)//2], t=True, q=True, ws=True))
    
    AC_mid = rMath.vec_midpoint(vecA, vecC)
    pv_pos = AC_mid + (vecB - AC_mid)*offset_pv

    # pv_loc = mc.spaceLocator(name=name + '_LOC')[0]
    # mc.xform(pv_loc, t=pv_pos, ws=True)

    return (pv_pos[0], pv_pos[1], pv_pos[2])


def get_pv_position(guide_list: Sequence[str], distance: float = 1.2) -> MPoint:
    """
    Automatic creation of pole vector position pole vector position.
    
    Args:
        distance: distance of the pole vector from the mid point
    Returns:
        MPoint: The pole vector position in world space.
    """
    # This idea for this pole vector positioning math/technique for a 2 segment limb is from mGear's calculatePoleVector function.
    def get_best_mid_point(positions: Sequence[MPoint]) -> MPoint:
        # Pick the point that is farthest from the baseline vector to build a traingle to define the guide plane.
        # Equation for distance between a point P and a vector line V is: ||P cross V|| / ||V||
        # Assuming that P and V both start at origin. https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation
        first_point = MVector(positions[0])
        last_point = MVector(positions[-1])
        base_vector: MVector =  last_point - first_point
        
        best_distance: float = 0
        best_point: MPoint = positions[1]
        for i in range(1, len(positions)-1):
            vector1: MVector = MVector(positions[i]) - first_point
            cross: MVector = (vector1 ^ base_vector)
            distance: float = cross.length() / base_vector.length()
            if distance > best_distance:
                best_distance = distance
                best_point = positions[i]
        return best_point    
        
    def project_vector(vector_a: MVector, vector_b: MVector) -> MVector:
        return ((vector_a * vector_b) / (vector_b * vector_b)) * vector_b
    
    first_guide = guide_list[0]
    last_guide = guide_list[-1]

    planar_points, *_ = make_points_planar(
        [
            MPoint(mc.xform(transform, query=True, worldSpace=True, translation=True))
            for transform in guide_list
        ]
    )

    first_position = MVector(
        mc.xform(first_guide, query=True, worldSpace=True, translation=True)
    )
    middle_position = MVector(get_best_mid_point(planar_points))
    last_position = MVector(mc.xform(last_guide, query=True, worldSpace=True, translation=True))
    base_mid: MVector = (last_position + first_position) * 0.5

    # Calculate a scale reference for how far the pole vector should be based on the two "segment" lengths.
    first_segment_length = (middle_position - first_position).length()
    second_segment_length = (last_position - middle_position).length()
    pole_distance = (first_segment_length + second_segment_length) * 0.5 * distance

    # Project the first segment onto the vector from the first to last point
    base_vector: MVector = last_position - first_position # Vector from first to last point (base of the traingle)
    projected = first_position + project_vector((middle_position - first_position), base_vector)
    mid_pointer: MVector = middle_position - projected # projected point to the midpoint points directly out from the base
    pole_vector_direction: MVector = mid_pointer.normal()
    
    # Construct the pole vector position. 
    # We take the normalized direction and scale it by the scale reference we calculated earlier, 
    # then add the vector that goes from the base out to the midpoint, 
    # this makes sure the pole vector is pushed out far enough when the chain is very bent.
    # Finally we take that and add it to the midpoint betwen the first and last points.
    pole_vector = (pole_vector_direction * pole_distance) + mid_pointer + base_mid

    return MPoint(pole_vector)


def create_pv_guide(
    guide_list=None, name=None, suffix=None, slide_pv=None, offset_pv=0, delete_setup=None, smart_two_segment: bool = False
):
    if not guide_list:
        guide_list = mc.ls(sl=True)

    if len(guide_list) != 3:
        if len(guide_list) > 3:
            point = get_pv_position(guide_list)
            return (point.x, point.y, point.z)
        else:
            mc.error('Must select or define three transforms to use as guides.')
    elif smart_two_segment:
        point = get_pv_position(guide_list)
        return (point.x, point.y, point.z)
        
    if not suffix:
        suffix = 'guide'

    if not name:
        name = guide_list[0] + '_' + suffix

    # build pv guide groups
    guide_grp = mc.group(empty=True, name=name + '_GRP')
    middle_grp = mc.group(parent=guide_grp, empty=True,
                            name=name + '_middle_GRP')
    dnt_grp = mc.group(parent=guide_grp, empty=True, name=name + '_DNT_GRP')
    cls_grp = mc.group(parent=dnt_grp, empty=True, name=name + '_CLS_GRP')
    mc.hide(dnt_grp)

    # define points of polygon from guides
    point_list = []
    for guide in guide_list:
        pos = mc.xform(guide, query=True, worldSpace=True, translation=True)
        point_list.append(pos)

    # create polygon plane
    poly = mc.polyCreateFacet(p=point_list, name=name + '_MSH',
                                constructionHistory=False)[0]

    # create clusters of the plane and constrain them to guides
    for i, vtx in enumerate(mc.ls(poly + '.vtx[*]', flatten=True)):
        cls, handle = mc.cluster(vtx, name='{}_{:02d}_CLS'.format(name, i))
        mc.pointConstraint(guide_list[i], handle, maintainOffset=False)
        mc.parent(handle, cls_grp)

    # create up vector locator
    upv_loc = mc.spaceLocator(name=name + '_upV_LOC')[0]

    # constrain upV_LOC between the first and last guide
    upv_cnst = mc.pointConstraint(guide_list[0], guide_list[-1], upv_loc,
                                    maintainOffset=False)[0]
    wal = mc.pointConstraint(upv_cnst, query=True, weightAliasList=True)

    # constrain middle group to middle guide
    mc.parentConstraint(guide_list[1], middle_grp, maintainOffset=False)

    # create nurbs plane
    nrb = mc.nurbsPlane(pivot=(0, 0, 0),
                          axis=(0, 1, 0),
                          width=0.25,
                          lengthRatio=1,
                          degree=1,
                          patchesU=1,
                          patchesV=1,
                          constructionHistory=False,
                          name=name + '_NRB')[0]

    # hide the nurb shape
    surf = mc.listRelatives(nrb, shapes=True)[0]
    mc.hide(surf)

    mc.matchTransform(nrb, guide_list[1])
    mc.parent(nrb, middle_grp)

    # create normal constraint from poly to nurb
    mc.normalConstraint(poly, nrb, weight=1, aimVector=(0, 0, 1),
                          upVector=(1, 0, 0), worldUpType='object',
                          worldUpObject=upv_loc)

    # create poleVector locator
    pv_loc = mc.spaceLocator(name=name + '_LOC')[0]
    mc.matchTransform(pv_loc, nrb)
    mc.parent(pv_loc, nrb)

    # find slide value and give attributes to pv locator
    if slide_pv:
        slide_ratio = slide_pv
    else:
        a_len = rMath.distance_between(point_a=guide_list[0],
                                        point_b=guide_list[1])
        b_len = rMath.distance_between(point_a=guide_list[1],
                                        point_b=guide_list[2])
        total_len = a_len + b_len
        slide_ratio = float(b_len) / total_len

    offset = rAttr.Attribute(node=pv_loc, type='double', value=offset_pv,
                              keyable=True, name='offset')
    slide = rAttr.Attribute(node=pv_loc, type='double', min=0, max=1,
                             value=slide_ratio,
                             keyable=True, name='slide')

    # calculate distance between mid joint and upV
    dist = mc.createNode('distanceBetween', name=name + '_DST')
    adl = mc.createNode('addDL', name=name + '_ADL')
    mdl = mc.createNode('multDL', name=name + '_MDL')
    rev = mc.createNode('reverse', name=name + '_REV')

    mc.connectAttr(upv_loc + '.worldMatrix[0]', dist + '.inMatrix1')
    mc.connectAttr(guide_list[1] + '.worldMatrix[0]', dist + '.inMatrix2')
    mc.connectAttr(dist + '.distance', adl + '.input1')
    mc.connectAttr(offset.attr, adl + '.input2')
    mc.connectAttr(adl + '.output', mdl + '.input1')
    mc.setAttr(mdl + '.input2', -1)

    mc.connectAttr(slide.attr, rev + '.inputX')
    mc.connectAttr(slide.attr, upv_cnst + '.' + wal[0])
    mc.connectAttr(rev + '.outputX', upv_cnst + '.' + wal[1])
    mc.connectAttr(mdl + '.output', pv_loc + '.translateX')

    # create and organize guides lines
    ik_gde = create_line_guide(a=guide_list[0], b=guide_list[-1],
                               name=name + '_ik')
    pv_gde = create_line_guide(a=pv_loc, b=upv_loc, name=name + '_pv')

    mc.parent(ik_gde['curve'], pv_gde['curve'], pv_loc)
    mc.setAttr(ik_gde['curve'] + '.inheritsTransform', 0)
    mc.setAttr(ik_gde['curve'] + '.translate', 0, 0, 0)
    mc.setAttr(ik_gde['curve'] + '.rotate', 0, 0, 0)
    mc.setAttr(pv_gde['curve'] + '.inheritsTransform', 0)
    mc.setAttr(pv_gde['curve'] + '.translate', 0, 0, 0)
    mc.setAttr(pv_gde['curve'] + '.rotate', 0, 0, 0)

    # cleanup
    mc.parent(poly, upv_loc, ik_gde['clusters'], pv_gde['clusters'], dnt_grp)
    offset.lock_and_hide(node=pv_loc)

    if delete_setup:
        pv_guide = mc.xform(pv_loc, query=True, worldSpace=True,
                              translation=True)
        mc.delete(guide_grp)
        return pv_guide
    else:
        return pv_loc


def create_line_guide(a=None, b=None, name=None, suffix=None):
    if not a and not b:
        a, b = mc.ls(sl=True)[0:2]

    if not suffix:
        suffix = 'GDE'
    if name:
        name = name + '_' + suffix
    else:
        name = '{}_to_{}_{}'.format(a, b, suffix)

    # start and end positions
    pos_a = mc.xform(a, query=True, worldSpace=True, translation=True)
    pos_b = mc.xform(b, query=True, worldSpace=True, translation=True)

    # create guide curve and rename shape
    crv = mc.curve(ep=[pos_a, pos_b], degree=1, name=name)
    shp = mc.listRelatives(crv, shapes=True)[0]
    shp = mc.rename(shp, crv + 'Shape')

    # drawing options for curve
    mc.setAttr(shp + '.overrideEnabled', 1)
    mc.setAttr(shp + '.overrideDisplayType', 1)

    # create and constrain clusters to drive guide
    cls_a, handle_a = mc.cluster(crv + '.cv[0]', name=crv + '_start_CLS')
    cls_b, handle_b = mc.cluster(crv + '.cv[1]', name=crv + '_end_CLS')
    mc.pointConstraint(a, handle_a, maintainOffset=False)
    mc.pointConstraint(b, handle_b, maintainOffset=False)
    mc.hide(handle_a, handle_b)

    return {'clusters': [handle_a, handle_b],
            'curve': crv}
