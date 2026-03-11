import maya.cmds as mc

class SplitJointTool:
    def __init__(self):
        self.root_joint = None
        self.chain_joints = []

        self.build_ui()

    def build_ui(self):
        if mc.window("splitJointToolWin", exists=True):
            mc.deleteUI("splitJointToolWin")

        self.window = mc.window("splitJointToolWin", title="Split Joint Setup", widthHeight=(250,120))
        mc.columnLayout(adjustableColumn=True, rowSpacing=10)

        mc.button(label="Set Root", height=30, command=self.set_root)
        mc.button(label="Set Chain", height=30, command=self.set_chain)
        mc.button(label="Apply", height=30, command=self.apply)

        mc.showWindow(self.window)

    # -------------------------
    # Button Functions
    # -------------------------

    def set_root(self, *args):
        sel = mc.ls(selection=True)

        if not sel:
            mc.warning("Please select a root joint.")
            return

        joint = sel[0]

        if mc.nodeType(joint) != "joint":
            mc.warning("Selected object is not a joint.")
            return

        self.root_joint = joint
        print(f"Root joint set to: {self.root_joint}")


    def set_chain(self, *args):
        # ensure ordered selection works
        mc.selectPref(trackSelectionOrder=True)

        sel = mc.ls(orderedSelection=True)

        if not sel:
            mc.warning("Please select joints for the chain.")
            return

        for obj in sel:
            if mc.nodeType(obj) != "joint":
                mc.warning(f"{obj} is not a joint.")
                return

        self.chain_joints = sel
        print("Chain joints set to:")
        for j in self.chain_joints:
            print(f"  {j}")


    def apply(self, *args):
        if not self.root_joint:
            mc.warning("Root joint not set.")
            return

        if not self.chain_joints:
            mc.warning("Chain joints not set.")
            return

        # create attribute
        if not mc.attributeQuery("split_joints", node=self.root_joint, exists=True):
            mc.addAttr(self.root_joint, longName="split_joints", dataType="string")

        mc.setAttr(
            f"{self.root_joint}.split_joints",
            repr(self.chain_joints),
            type="string"
        )

        print("Split joints attribute applied.")
        print(f"Root: {self.root_joint}")
        print(f"Chain: {self.chain_joints}")


def run():
    SplitJointTool()


run()