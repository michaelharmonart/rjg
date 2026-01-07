from dataclasses import dataclass
import maya.cmds as mc
import maya.mel as mel
import rjg.libs.control.ctrl as rCtrl

import os

def create_pxWrap(*argv):
    a = argv if len(argv) > 1 else argv[0]

    driven = a[:-1]
    driver = a[-1]

    #print(f"adding driver: {driver} to driven: {driven}")
    mc.select(driven)
    pxWrap = mc.proximityWrap()
    mc.proximityWrap(pxWrap, e=True, addDrivers=driver)
    mc.setAttr(pxWrap[0] + '.falloffScale', 15.0)


def create_shirt_pxWrap(driven_mesh, driver_mesh):
    """
    Create a proximityWrap specifically for the shirt.
    Ensures only the *non-Orig* shapes are connected as driver and driven,
    avoiding the common Orig-shape wiring bug.

    Args:
        driven_mesh (str): The name of the driven mesh transform (e.g., 'shirt1').
        driver_mesh (str): The name of the driver mesh transform (e.g., 'shirtlow').

    Returns:
        str: The name of the created proximityWrap node.
    """

    def get_non_orig_shape(mesh_transform):
        """Return the first shape under mesh_transform that does NOT contain 'Orig' in its name."""
        shapes = mc.listRelatives(mesh_transform, shapes=True, noIntermediate=True) or []
        for shape in shapes:
            if 'Orig' not in shape:
                return shape
        raise RuntimeError(f"No valid non-Orig shape found under {mesh_transform}")

    driven_shape = get_non_orig_shape(driven_mesh)
    driver_shape = get_non_orig_shape(driver_mesh)

    # Select driven shape explicitly
    mc.select(driven_mesh, r=True)

    # Create proximityWrap node with only driven selected
    pxWrap = mc.proximityWrap()

    # Explicitly connect the correct driver shape to drivers[0]
    # Force connection to override any wrong auto-connections
    mc.connectAttr(f"{driver_shape}.worldMesh[0]", f"{pxWrap[0]}.drivers[0].driverGeometry", force=True)

    # Explicitly connect the driven Orig shape to originalGeometry for safety
    driven_orig_shapes = [s for s in mc.listRelatives(driven_mesh, shapes=True) if 'Orig' in s]
    if driven_orig_shapes:
        mc.connectAttr(f"{driven_orig_shapes[0]}.worldMesh[0]", f"{pxWrap[0]}.originalGeometry[0]", force=True)

    # Optionally set falloff scale as your original function did
    mc.setAttr(f"{pxWrap[0]}.falloffScale", 15.0)

    return pxWrap[0]


def create_pxPin(x, y, z, target_vtx, n='default', ctrl=False, prop=None):
    pin = mc.spaceLocator(n=n)
    mc.move(x, y, z, r=True, os=True, wd=True)

    mc.select(target_vtx, pin)
    pxPin = mc.ProximityPin()

    grp=None

    if ctrl:
        rig_grp = mc.group(empty=True, n=n + '_M', parent='RIG')
        pin_ctrl = rCtrl.Control(parent=rig_grp, name=n, shape='lollipop', side='M', suffix='CTRL', axis='y', group_type='main', rig_type='primary', translate=pin[0], rotate=(0, 0, 0), ctrl_scale=5)
        pin_ctrl.tag_as_controller()

        #mc.parentConstraint(pin_ctrl.ctrl, pin[0])
        mc.parentConstraint(pin[0], pin_ctrl.top, mo=True)
        mc.parentConstraint(pin_ctrl.ctrl, prop, mo=True)
        # grp = mc.group(empty=True, n=n + '_PIN')
        # mc.select(grp, n)
        # mel.eval("MatchTransform")
        # mc.parentConstraint(pin_ctrl.ctrl, grp, mo=True)

    mc.delete(pxPin)

    return pin, grp


def pvis_blink(jnt, blink, dist):
    mc.setDrivenKeyframe(jnt + '.rotateX', cd=blink+'.blink')
    mc.setAttr(blink + '.blink', 1)
    mc.setAttr(jnt + '.rotateX', dist)
    mc.setDrivenKeyframe(jnt + '.rotateX', cd=blink + '.blink')
    mc.setAttr(blink + '.blink', 0)
    



'''
main_mesh: (str) name of mesh with the blendshape deformer
blandshape: name of the blendshape
driver: control attribute to drive shape
driven: corrective blend shape
driver_range: attribute values for key driver
driven_range: blend shape envelope values for key driven 
index: index to insert
'''
def connect_corrective(main_mesh, blendshape, driver, driven, driver_range, driven_range, curve='linear', index=0):
    mc.select(driven)
    mc.blendShape(blendshape, e=True, target=[main_mesh, index, driven, 1], w=[index, 0])

    org = mc.getAttr(driver)

    bst = blendshape + '.' + driven

    for a, b in zip(driver_range, driven_range):
        mc.setAttr(driver, a)
        mc.setAttr(bst, b)
        mc.select(blendshape)
        mc.setDrivenKeyframe(at=driven, cd=driver, itt=curve, ott=curve)

    mc.setAttr(driver, org)
    mc.select(clear=True)

    mc.delete(driven)


def corrective_setup(mesh, input=None):
    if not mc.objExists('main_blendshapes'):
        mc.select(mesh)
        bs = mc.blendShape(n='main_blendshapes', automatic=True)
        mc.select(clear=True)
    
    imp = mc.file(input[0], i=True)

    for id, s in enumerate(input[1:]):
       connect_corrective(s[0], 'main_blendshapes', s[1], s[2], s[3], s[4], s[5], id+49)

    mc.delete("*_DEFAULT")


def import_poseInterpolator(path):
    try:
        from maya import mel
        mel.eval(f'poseInterpolatorImportPoses "{path}" 1;')
        mc.select('*_poseInterpolator')
        pi = mc.ls(selection=True)
        mc.parent(pi, 'RIG')
    except Exception as e:
        mc.warning(e)

def inplace_symlink(path):
    temp = f"{path}\\tmp"
    os.symlink(path, temp)

def get_last_deformer(shape: str) -> str | None:
    """Return the last deformer in the geometry’s deformation chain."""
    deformers = mc.deformableShape(shape, chain=True) or []
    if deformers:
        return deformers[-1]
    return None


@dataclass
class BypassNodes():
    pre_node: str
    def_node: str | None
    post_node: str

def create_mesh_connection_bypass(source: str, destination: str, name: str) -> BypassNodes:
    # Disconnect
    mc.disconnectAttr(source, destination)

    # Create bypass nodes
    pre_node: str = mc.createNode("groupParts", name=f"{name}_PRE")
    post_node: str = mc.createNode("groupParts", name=f"{name}_POST")

    # Rebuild connections
    mc.connectAttr(source, f"{pre_node}.inputGeometry")
    mc.connectAttr(f"{pre_node}.outputGeometry", f"{post_node}.inputGeometry")
    mc.connectAttr(f"{post_node}.outputGeometry", destination)

    # Add toggle attribute
    mc.addAttr(pre_node, longName="bypassed", attributeType="bool")

    return BypassNodes(pre_node=pre_node, def_node=None, post_node=post_node)



def create_deformer_bypass_nodes(deformer: str, name: str) -> BypassNodes:
    """
    Wraps a deformer with groupPart nodes to make it easy to bypass it at any time 
    by connecting the PRE outputGeometry to the POST inputGeometry.
    Re-enabling is done by connecting the DEF outputGeometry to the POST inputGeometry.

    Args:
        deformer (str): Name of the deformer node to wrap with bypass nodes.
        name (str): Base name used when naming the created groupParts nodes.

    Returns:
        BypassNodes: An object containing the names of the created bypass nodes.

    Visualization:    
    ┌───────┐   ┌────────────┐  ┌────────┐
    │  PRE  ┼──►│  Deformer  ┼──►  POST  │
    └───────┘   └─────┬──────┘  └────────┘
                  ┌───▼───┐               
                  │  DEF  │               
                  └───────┘               
    """
    deformer_input_attr: str = f"{deformer}.input[0].inputGeometry"
    deformer_output_attr: str = f"{deformer}.outputGeometry[0]"

    # Get existing connections
    source_mesh_attr: str = mc.listConnections(deformer_input_attr, source=True, destination=False, plugs=True)[0]
    destination_mesh_attr: str = mc.listConnections(deformer_output_attr, source=False, destination=True, plugs=True)[0]
    
    # Create bypass nodes
    pre_node: str = mc.createNode("groupParts", name=f"{name}_PRE")
    def_node: str = mc.createNode("groupParts", name=f"{name}_DEF")
    post_node: str = mc.createNode("groupParts", name=f"{name}_POST")
    
    # Rebuild connections
    mc.connectAttr(source_mesh_attr, f"{pre_node}.inputGeometry")
    mc.connectAttr(f"{pre_node}.outputGeometry", deformer_input_attr, force=True)
    mc.connectAttr(deformer_output_attr, f"{post_node}.inputGeometry")
    mc.connectAttr(f"{post_node}.outputGeometry", destination_mesh_attr, force=True)
    mc.connectAttr(deformer_output_attr, f"{def_node}.inputGeometry")

    # Add toggle attribute
    mc.addAttr(def_node, longName="bypassed", attributeType="bool")

    return BypassNodes(pre_node=pre_node, def_node=def_node, post_node=post_node)