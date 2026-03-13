import os
import json
import platform
import maya.cmds as mc


# -------------------------------------------------
# PIPELINE PATH
# -------------------------------------------------

groups = "G:" if platform.system() == "Windows" else "/groups"

DEFAULT_DIR = rf"{groups}\dragonkisser\pipeline\pipeline\software\maya\scripts\rjg\build_scripts\character_defaults"

class ControlDefaultsTool:

    def __init__(self):

        self.character = None
        self.json_path = None

        self.window = "controlDefaultsTool"

        if mc.window(self.window, exists=True):
            mc.deleteUI(self.window)

        self.window = mc.window(self.window, title="Control Defaults Tool", widthHeight=(250,150))

        mc.columnLayout(adj=True, rs=10)

        self.init_btn = mc.button(
            label="Initialize Character",
            height=40,
            command=self.initialize_character
        )

        mc.button(
            label="Save Out Control",
            height=35,
            command=self.save_controls
        )

        mc.button(
            label="Read Control Defaults",
            height=35,
            command=self.read_selected
        )

        mc.button(
            label="Read All",
            height=35,
            command=self.read_all
        )

        mc.showWindow(self.window)

    # -------------------------------------------------
    # INITIALIZE CHARACTER
    # -------------------------------------------------

    def initialize_character(self, *_):

        try:

            ubm_nodes = mc.ls("*_UBM")

            if not ubm_nodes:
                raise RuntimeError("No *_UBM node found")

            ubm = ubm_nodes[0]

            self.character = ubm.replace("_UBM", "")

            if not os.path.exists(DEFAULT_DIR):
                os.makedirs(DEFAULT_DIR)

            self.json_path = os.path.join(
                DEFAULT_DIR,
                f"{self.character}_defaults.json"
            )

            if os.path.exists(self.json_path):

                mc.button(self.init_btn, e=True, bgc=(0.2,0.6,0.2))
                print(f"{self.character}_defaults.json already exists")

            else:

                data = {"controls": {}}

                with open(self.json_path, "w") as f:
                    json.dump(data, f, indent=4)

                mc.button(self.init_btn, e=True, bgc=(0.2,0.6,0.2))
                print(f"{self.character}_defaults.json created")

        except Exception as e:

            mc.button(self.init_btn, e=True, bgc=(0.8,0.2,0.2))
            print("Initialization failed:", e)

    # -------------------------------------------------
    # LOAD JSON
    # -------------------------------------------------

    def load_json(self):

        if not self.json_path:
            print("Initialize character first")
            return None

        with open(self.json_path, "r") as f:
            return json.load(f)

    # -------------------------------------------------
    # SAVE JSON
    # -------------------------------------------------

    def save_json(self, data):

        with open(self.json_path, "w") as f:
            json.dump(data, f, indent=4)

    # -------------------------------------------------
    # SAVE CONTROL DEFAULTS
    # -------------------------------------------------

    def save_controls(self, *_):

        sel = mc.ls(sl=True)

        if not sel:
            print("No controls selected")
            return

        data = self.load_json()

        if not data:
            return

        for ctrl in sel:

            if not mc.objExists(ctrl):
                continue

            keyable = mc.listAttr(ctrl, keyable=True) or []

            attrs = []

            for attr in keyable:

                if attr in [
                    "translateX","translateY","translateZ",
                    "rotateX","rotateY","rotateZ",
                    "scaleX","scaleY","scaleZ",
                    "visibility"
                ]:
                    continue

                attrs.append(attr)

            if not attrs:
                print(f"No custom attributes on {ctrl}")
                continue

            if ctrl in data["controls"]:

                result = mc.confirmDialog(
                    title="Control Exists",
                    message=f"{ctrl} already exists in JSON",
                    button=["Skip","Overwrite"],
                    defaultButton="Skip"
                )

                if result == "Skip":
                    continue

                if result == "Overwrite":
                    data["controls"].pop(ctrl)

            data["controls"][ctrl] = {}

            for attr in attrs:

                plug = f"{ctrl}.{attr}"

                if mc.objExists(plug):

                    val = mc.getAttr(plug)
                    data["controls"][ctrl][attr] = val

            print(f"Saved defaults for {ctrl}")

        self.save_json(data)

    # -------------------------------------------------
    # READ SELECTED CONTROLS
    # -------------------------------------------------

    def read_selected(self, *_):

        sel = mc.ls(sl=True)

        if not sel:
            print("No controls selected")
            return

        data = self.load_json()

        if not data:
            return

        for ctrl in sel:

            if ctrl not in data["controls"]:
                print(f"No defaults found for {ctrl}")
                continue

            for attr, val in data["controls"][ctrl].items():

                plug = f"{ctrl}.{attr}"

                if mc.objExists(plug):

                    try:
                        mc.setAttr(plug, val)
                    except:
                        pass

            print(f"Defaults applied to {ctrl}")

    # -------------------------------------------------
    # READ ALL CONTROLS
    # -------------------------------------------------

    def read_all(self, *_):

        data = self.load_json()

        if not data:
            return

        for ctrl in data["controls"]:

            if not mc.objExists(ctrl):

                print(f"{ctrl} does not exist in scene")
                continue

            for attr, val in data["controls"][ctrl].items():

                plug = f"{ctrl}.{attr}"

                if mc.objExists(plug):

                    try:
                        mc.setAttr(plug, val)
                    except:
                        pass

            print(f"Defaults applied to {ctrl}")


# -------------------------------------------------
# RUN TOOL
# -------------------------------------------------

def run():
    ControlDefaultsTool()