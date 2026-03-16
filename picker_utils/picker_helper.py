import json
import os
import uuid
import dwpicker
from dwpicker.templates import BUTTON


def _get_image_dimensions(image_path):
    """Read image dimensions - tries PySide6 then PySide2 (depends on Maya version)."""
    try:
        from PySide6.QtGui import QImage
        img = QImage(image_path)
        if img.isNull():
            raise ValueError("QImage returned null.")
        return img.width(), img.height()
    except ImportError:
        pass
    try:
        from PySide2.QtGui import QImage
        img = QImage(image_path)
        if img.isNull():
            raise ValueError("QImage returned null.")
        return img.width(), img.height()
    except ImportError:
        pass
    print("Warning: could not read image dimensions, defaulting to 1920x1080")
    return 1920, 1080


def _make_background_image_shape(image_path):
    """
    Build a locked, full-size background image shape using the real BUTTON template.
    """
    width, height = _get_image_dimensions(image_path)

    shape = BUTTON.copy()
    shape.update({
        # Required unique ID (needed by document.py)
        'id':                     str(uuid.uuid4()),

        # Position and size
        'shape.left':             0.0,
        'shape.top':              0.0,
        'shape.width':            float(width),
        'shape.height':           float(height),

        # Image settings
        'image.path':             image_path.replace("\\", "/"),
        'image.width':            width,
        'image.height':           height,
        'image.fit':              True,
        'image.ratio':            True,

        # Background - no text, no visible border
        'background':             True,
        'text.content':           '',
        'border':                 False,
        'bgcolor.normal':         '#000000',
        'bgcolor.hovered':        '#000000',
        'bgcolor.clicked':        '#000000',
        'bgcolor.transparency':   255,

        # No selection behaviour
        'action.targets':         [],
        'action.commands':        [],
        'action.menu_commands':   [],

        # Locked - can't be accidentally moved in editor
        'shape.ignored_by_focus': True,
    })

    return shape, width, height


def initialize_picker_from_shot(
        character_name,
        path,
        image,
        json_path,
        picker_des):
    """
    Create a blank dwpicker document with a locked background image and open it.

    Args:
        character_name (str): Name of the character e.g. 'Drago'
        path          (str): Directory where the image lives
        image         (str): Image filename e.g. 'drago_bg.png'
        json_path     (str): Full path to save the .json file to
        picker_des    (str): Short descriptor e.g. 'body'
    """
    picker_name = f"{character_name}_Picker_{picker_des}"
    image_path  = os.path.join(path, image)

    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Image not found: {image_path}")

    bg_shape, img_w, img_h = _make_background_image_shape(image_path)

    data = {
        'general': {
            'version':            [1, 0, 0],
            'name':               picker_name,
            'panels':             [[1.0, [1.0]]],
            'panels.orientation': 'vertical',
            'panels.zoom_locked': [False],
            'panels.as_sub_tab':  False,
            'panels.colors':      [None],
            'panels.names':       ['Panel 1'],
            'hidden_layers':      [],
            'menu_commands':      [],
        },
        'shapes': [bg_shape]
    }

    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"Picker '{picker_name}' saved to: {json_path}")
    print(f"Background: {image_path} ({img_w} x {img_h}px)")

    dwpicker.open_picker_file(json_path)
    return data
    
initialize_picker_from_shot(
    character_name = 'CrowdB',
    path           = r'G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\picker_files',
    image          = 'CrowdB.0.png',
    json_path      = r'G:\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\picker_files\crowdB_Picker.json',
    picker_des     = 'body'
)