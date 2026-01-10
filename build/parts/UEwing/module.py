import re
from importlib import reload

import maya.cmds as mc
import rjg.build.chain as rChain
import rjg.build.guide as rGuide
import rjg.libs.attribute as rAttr
import rjg.libs.control.ctrl as rCtrl
import rjg.libs.transform as rXform
from rjg.build.parts.UEwing import feathers, limb
from rjg.build.parts.UEwing.spline_system import Spline
from rjg.build.UEface import UEface
from rjg.libs.profile import auto_profiler_tag

reload(rAttr)
reload(rChain)
reload(rCtrl)
reload(rGuide)
reload(rXform)


def get_guide_index(guide: str) -> int:
    pattern = r"(?<=_)[0-9]+(?=_)"
    matches = re.findall(pattern, guide)
    if matches:
        guide_id = int(matches[-1])
        return guide_id
    return 0


class UEwing(UEface):
    def __init__(self, grp_name: str, side: str, ctrl_scale=1, twisty=True, buildlimb=True, limb_descriptor=None):
        super().__init__(part="Wing", grp_name=grp_name, ctrl_scale=ctrl_scale)
        self.grp_name = grp_name
        self.prefix = UEface.get_prefix_from_group(self.grp_name)
        self.side = side
        self.twisty = twisty
        self.buildlimb = buildlimb
        self.limb_descriptor = limb_descriptor
        # group='Wing_L_guides'

    @staticmethod
    def get_namestruc(prefix="Wing_L", rjg=True):
        if rjg == False:
            ctrlname = "CTRL"
            grpname = "GRP"
        else:
            parts = prefix.split("_")  # ["wing", "L"]
            side = parts[-1]  # "L"
            ctrlname = f"{side}_CTRL"
            grpname = f"{side}_CTRL_CNST_GRP"
        return ctrlname, grpname

    def get_guides(self, prefix: str, feather="MainFeather"):
        main_guides = mc.ls(f"{prefix}_{feather}_??_guide")
        valid_guides: list[tuple[str, str, str]] = []
        for guide in main_guides:
            index = get_guide_index(guide)
            if not mc.objExists(f"{prefix}_{feather}_{index:02d}_ee_guide"):
                continue
            mid_guide = f"{prefix}_{feather}_{index:02d}_ee_guide"
            if not mc.objExists(f"{prefix}_{feather}_{index:02d}_aim"):
                continue
            tip_guide = f"{prefix}_{feather}_{index:02d}_aim"
            valid_guides.append((guide, mid_guide, tip_guide))
        return valid_guides

    def build_limb(self, bendy: bool = True):
        prefix = self.prefix
        side = prefix.split("_")[-1]
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        limb.build_limb(self, ctrlname, grpname, bendy)


    def add_global_twist(self, main_ctrl=None):
        """
        Adds a global twist attribute to control twist along the whole bendy chain.
        main_ctrl : str, the main control driving the leg (e.g., FootRoot_CTRL)
        """
        if not main_ctrl:
            main_ctrl = f"Wing_{self.side}"  # fallback to your main leg control

        # Add the twist attribute
        if not mc.objExists(f"{main_ctrl}.TwistDistribute"):
            mc.addAttr(
                main_ctrl,
                longName="TwistDistribute",
                attributeType="double",
                min=0,
                max=1,
                defaultValue=1,
                keyable=True,
            )

        # Create a multiplyDivide node
        twist_mdn = mc.createNode("multiplyDivide", n=f"{self.side}_bendyTwist_MDN")
        mc.setAttr(twist_mdn + ".operation", 2)  # divide
        mc.connectAttr(f"{main_ctrl}.TwistDistribute", twist_mdn + ".input1X")

        # Connect to all bendy joints’ rotateX
        # for jnt in self.bendy_chain.joints:
        #    mc.connectAttr(twist_mdn + '.outputX', f'{jnt}.rotateX')
    def apply_limb_descriptor(self, data):
        self.limb_bind_joints = data["bind_joints"]

        class Dummy: pass
        self.bendy_chain = Dummy()
        self.bendy_chain.bendy_controls = data["bendy_controls"]
        self.bendy_chain.joints = data["bendy_joints"]


    def connectlimb(
        self,
    ):  # bind_joints = [f'arm_{side}_01_JNT', f'arm_{side}_02_JNT', f'arm_{side}_03_JNT', f'arm_{side}_04_JNT', f'arm_{side}_05_JNT', f'arm_{side}_06_JNT', f'arm_{side}_07_JNT', f'arm_{side}_08_JNT']
        self.limb_bind_joints = [
            f"arm_{self.side}_01_JNT",
            f"arm_{self.side}_05_JNT",
            f"arm_{self.side}_09_JNT",
        ]

    def build_feathers(self, keep_spacing: bool = True):
        feathers.build_feathers(self, keep_spacing)

    def connect_feathers(self, root_spline: Spline, mid_spline: Spline, tip_spline: Spline):
        if self.buildlimb:
            bind_joints = self.limb_bind_joints
            
            chain_bendy_controls: list[rCtrl.Control] = self.bendy_chain.bendy_controls
            root_bind = [ctrl.ctrl for ctrl in chain_bendy_controls] + bind_joints[-2:]
            root_mapping = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7)]
            
            wing_mapping = [(0, 0), (1, 0), (2, 1), (3, 2), (4, 3)]

            for ctrl_index, joint_index in root_mapping:
                root_pin = root_spline.pin_list[ctrl_index]
                bind_joint = root_bind[joint_index]
                mc.parentConstraint(bind_joint, root_pin, maintainOffset=True)

            for ctrl_index, joint_index in wing_mapping:
                ctrls = (
                    mid_spline.control_list[ctrl_index],
                    tip_spline.control_list[ctrl_index],
                )
                bind_joint = bind_joints[joint_index]
                for ctrl in ctrls:
                    mc.parentConstraint(bind_joint, ctrl.top, maintainOffset=True)

            # 50% blend for elbow
            mc.parentConstraint(bind_joints[0], root_spline.pin_list[2], mo=True)
        else:
            root_spline = self.root_spline
            mid_spline = self.mid_spline
            tip_spline = self.tip_spline

            root_bind = self.limb_bind_joints

            # mapping tables (your existing logic)
            root_mapping = [
                (0, 0),
                (1, 1),
                (2, 2),
                (3, 3),
            ]

            wing_mapping = [
                (0, 1),
                (1, 2),
                (2, 3),
            ]

            # ---------------- ROOT ----------------
            for ctrl_index, joint_index in root_mapping:

                if not self.buildlimb:
                    if ctrl_index >= len(root_spline.pin_list):
                        continue
                    if joint_index >= len(root_bind):
                        continue

                root_pin = root_spline.pin_list[ctrl_index]
                bind_joint = root_bind[joint_index]

                mc.parentConstraint(bind_joint, root_pin, mo=True)

            # ---------------- MID / TIP ----------------
            for ctrl_index, joint_index in wing_mapping:

                if not self.buildlimb:
                    if ctrl_index >= len(mid_spline.control_list):
                        continue
                    if ctrl_index >= len(tip_spline.control_list):
                        continue
                    if joint_index >= len(root_bind):
                        continue

                mid_ctrl = mid_spline.control_list[ctrl_index]
                tip_ctrl = tip_spline.control_list[ctrl_index]

                bind_joint = root_bind[joint_index]

                mc.parentConstraint(bind_joint, mid_ctrl.top, mo=True)
                mc.parentConstraint(bind_joint, tip_ctrl.top, mo=True)

            root_pin = root_spline.pin_list[ctrl_index]
            bind_joint = root_bind[joint_index]
            mc.parentConstraint(bind_joint, root_pin, maintainOffset=True)


    @auto_profiler_tag
    def build_wing(self):
        prefix = self.prefix
        grp = self.grp_name
        ctrlname, grpname = UEwing.get_namestruc(prefix)
        parts = prefix.split("_")  # ["wing", "L"]
        side = parts[-1]
        self.mastergrp = mc.group(em=True, name=f"{prefix}")
        if self.buildlimb:
            self.build_limb(self.twisty)
        else:
            #self.connectlimb()
            if self.limb_descriptor:
                self.apply_limb_descriptor(self.limb_descriptor)
        self.build_feathers()
        self.connect_feathers(
            root_spline=self.root_spline, mid_spline=self.mid_spline, tip_spline=self.tip_spline
        )
