import math
import json
import uuid
import os
import maya.cmds as cmds
import dwpicker
from dwpicker.templates import BUTTON


# ─── IMAGE DIMENSIONS ─────────────────────────────────────────────────────────

def _get_image_dimensions(image_path):
    """Read image dimensions from file using PySide6 or PySide2."""
    try:
        from PySide6.QtGui import QImage
        img = QImage(image_path)
        if not img.isNull():
            return img.width(), img.height()
    except ImportError:
        pass
    try:
        from PySide2.QtGui import QImage
        img = QImage(image_path)
        if not img.isNull():
            return img.width(), img.height()
    except ImportError:
        pass
    raise RuntimeError(f"Could not read image dimensions from: {image_path}")


# ─── MATH HELPERS ─────────────────────────────────────────────────────────────

def _deg_to_rad(deg):
    return deg * math.pi / 180.0


def _multiply_matrix_vector(m, v):
    return [
        m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
        m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
        m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2],
    ]


def _rotation_matrix_y(deg):
    r = _deg_to_rad(deg)
    c, s = math.cos(r), math.sin(r)
    return [[ c, 0, s], [ 0, 1, 0], [-s, 0, c]]


def _rotation_matrix_x(deg):
    r = _deg_to_rad(deg)
    c, s = math.cos(r), math.sin(r)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _multiply_matrices(a, b):
    result = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                result[i][j] += a[i][k] * b[k][j]
    return result


# ─── CORE PROJECTION ──────────────────────────────────────────────────────────

def get_2d_position(control, camera_data, image_width, image_height,
                    btn_width, btn_height):
    """
    Project a Maya control's world position onto 2D picker space.
    Returns position centered on the control (top-left corner for dwpicker).

    Args:
        control      (str):   Maya control node name
        camera_data  (dict):  Loaded camera JSON
        image_width  (int):   Snapshot image width in pixels
        image_height (int):   Snapshot image height in pixels
        btn_width    (float): Button width  (used to center the button on the point)
        btn_height   (float): Button height (used to center the button on the point)

    Returns:
        (float, float): (x, y) top-left corner position in picker pixel space
    """
    world_pos = cmds.xform(control, query=True, worldSpace=True, translation=True)
    pivot     = camera_data['group_translate']

    relative = [
        world_pos[0] - pivot[0],
        world_pos[1] - pivot[1],
        world_pos[2] - pivot[2],
    ]

    swivel_mat = _rotation_matrix_y(camera_data['swivel'])
    tilt_mat   = _rotation_matrix_x(camera_data['tilt'])
    view_mat   = _multiply_matrices(tilt_mat, swivel_mat)

    cam_space = _multiply_matrix_vector(view_mat, relative)

    ortho_width  = camera_data['orthographicWidth']
    ortho_height = ortho_width * (image_height / image_width)

    norm_x =  cam_space[0] / ortho_width
    norm_y = -cam_space[1] / ortho_height   # flip Y (image Y goes downward)

    pixel_x = (norm_x + 0.5) * image_width
    pixel_y = (norm_y + 0.5) * image_height

    # Offset by half button size so the button is centered on the control point
    centered_x = pixel_x - (btn_width  / 2.0)
    centered_y = pixel_y - (btn_height / 2.0)

    return centered_x, centered_y


# ─── CONTROL COLOR ────────────────────────────────────────────────────────────

def _get_control_color(control):
    """Get the wireframe override color of a Maya control as a hex string."""
    maya_index_colors = {
        0:  '#808080', 1:  '#000000', 2:  '#404040', 3:  '#808080',
        4:  '#800000', 5:  '#000040', 6:  '#000080', 7:  '#002020',
        8:  '#200020', 9:  '#800040', 10: '#804020', 11: '#404000',
        12: '#802000', 13: '#FF0000', 14: '#00FF00', 15: '#0040FF',
        16: '#FFFFFF', 17: '#FFFF00', 18: '#40C0FF', 19: '#40FF40',
        20: '#FF8080', 21: '#FF8040', 22: '#FFFF80', 23: '#008040',
        24: '#804040', 25: '#808040', 26: '#408040', 27: '#408080',
        28: '#4040FF', 29: '#FF40FF', 30: '#FF8040', 31: '#FF4040',
    }

    shapes = cmds.listRelatives(control, shapes=True) or []
    node   = shapes[0] if shapes else control

    try:
        if cmds.getAttr(node + '.overrideRGBColors'):
            r = cmds.getAttr(node + '.overrideColorR')
            g = cmds.getAttr(node + '.overrideColorG')
            b = cmds.getAttr(node + '.overrideColorB')
            return '#{:02X}{:02X}{:02X}'.format(
                int(r * 255), int(g * 255), int(b * 255)
            )
    except Exception:
        pass

    try:
        if cmds.getAttr(node + '.overrideEnabled'):
            index = cmds.getAttr(node + '.overrideColor')
            return maya_index_colors.get(index, '#888888')
    except Exception:
        pass

    return '#888888'


# ─── BUTTON PLACEMENT ─────────────────────────────────────────────────────────

def place_button_on_picker(picker, control, position_2d,
                            color=None,
                            shape='square',
                            btn_width=120.0,
                            btn_height=25.0):
    """
    Add a button to the picker at the given 2D position.

    Args:
        picker       : Active dwpicker instance from dwpicker.current()
        control (str): Maya control node name
        position_2d  : (x, y) top-left corner in picker pixel space
        color   (str): Hex color string e.g. '#4080FF'. 
                       Defaults to None (uses control wireframe color)
        shape   (str): 'square' or 'round'. Defaults to 'square'
        btn_width (float):  Button width in pixels.  Defaults to 120.0
        btn_height (float): Button height in pixels. Defaults to 25.0
    """
    resolved_color = color if color else _get_control_color(control)
    x, y = position_2d

    btn = BUTTON.copy()
    btn.update({
        'id':                     str(uuid.uuid4()),
        'text.content':           control.split(':')[-1],   # strip namespace
        'shape.left':             x,
        'shape.top':              y,
        'shape.width':            float(btn_width),
        'shape.height':           float(btn_height),
        'shape':                  shape,                    # 'square' or 'round'
        'bgcolor.normal':         resolved_color,
        'bgcolor.hovered':        resolved_color,
        'bgcolor.clicked':        resolved_color,
        'action.targets':         [control],
        'panel':                  0,
        'shape.space':            'world',
        'shape.anchor':           'top_left',
        'shape.ignored_by_focus': False,
    })

    picker.document.add_shapes([btn])


# ─── MAIN HELPER ──────────────────────────────────────────────────────────────

def add_selected_controls_to_picker(camera_json_path, image_path,
                                     color=None,
                                     shape='square',
                                     btn_width=120.0,
                                     btn_height=25.0):
    """
    For each selected Maya control, project its 3D position onto the 2D
    picker image and place a button at that position.

    Args:
        camera_json_path (str):   Path to the exported camera JSON
        image_path       (str):   Path to the snapshot image (dimensions read automatically)
        color      (str):         Hex color e.g. '#4080FF'. None = use control color.
                                  Defaults to '#4080FF' (blue)
        shape      (str):         'square' or 'round'. Defaults to 'square'
        btn_width  (float):       Button width in pixels.  Defaults to 120.0
        btn_height (float):       Button height in pixels. Defaults to 25.0
    """
    # Default color is blue
    resolved_color = color if color else '#4080FF'

    picker = dwpicker.current()
    if picker is None:
        cmds.warning("No active picker found. Open a picker first.")
        return

    selection = cmds.ls(selection=True)
    if not selection:
        cmds.warning("Nothing selected.")
        return

    # Read image dimensions from the actual file
    image_width, image_height = _get_image_dimensions(image_path)
    print(f"Image dimensions: {image_width} x {image_height}px")

    with open(camera_json_path, 'r') as f:
        camera_data = json.load(f)

    for control in selection:
        pos_2d = get_2d_position(
            control, camera_data,
            image_width, image_height,
            btn_width, btn_height
        )
        print(f"  {control}  ->  ({pos_2d[0]:.1f}, {pos_2d[1]:.1f})")
        place_button_on_picker(
            picker     = picker,
            control    = control,
            position_2d= pos_2d,
            color      = resolved_color,
            shape      = shape,
            btn_width  = btn_width,
            btn_height = btn_height,
        )

    picker.document.changed.emit()
    print(f"Added {len(selection)} buttons to picker.")
add_selected_controls_to_picker(
    camera_json_path = r'G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\picker_files\CrowdB_camera.json',
    image_path       = r'G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\picker_files\CrowdB.0.png',
    btn_width        = 4.0,
    btn_height       = 4.0,
)