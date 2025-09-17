import maya.cmds as mc


def is_maya2026_or_newer() -> bool:
    return mc.about(apiVersion=True) >= 20260000


class PointMatrixMultiplyNode:
    def __init__(self, name: str = "multiplyPointByMatrix") -> None:
        if is_maya2026_or_newer:
            mult_node: str = mc.createNode("multiplyPointByMatrix", name=name)
            self.input_point: str = f"{mult_node}.input"
        else:
            mult_node: str = mc.createNode("pointMatrixMult", name=name)
            self.input_point: str = f"{mult_node}.inPoint"
        self.name: str = mult_node
        self.input_matrix: str = f"{mult_node}.matrix"
        self.output: str = f"{mult_node}.output"

class AddNode:
    def __init__(self, name: str = "addDoubleLinear") -> None:
        if is_maya2026_or_newer:
            mult_node: str = mc.createNode("multiplyPointByMatrix", name=name)
            self.input_point: str = f"{mult_node}.input"
        else:
            mult_node: str = mc.createNode("pointMatrixMult", name=name)
            self.input_point: str = f"{mult_node}.inPoint"
        self.name: str = mult_node
        self.input_matrix: str = f"{mult_node}.matrix"
        self.output: str = f"{mult_node}.output"