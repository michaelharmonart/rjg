import maya.cmds as mc
import maya.api.OpenMaya as om2
from importlib import reload

class Attribute:
    def __init__(self, add=True, type=None, node=None, name=None, value=None, enum_list=None, lock=False, min=None, max=None, keyable=None, children_name=None, transfer_to=None):
        self.type = type
        self.node = node
        self.name = name
        self.value = value
        self.enum_list = enum_list
        self.min = min
        self.max = max
        self.keyable = keyable
        self.lock = lock
        self.children_name = children_name
        self.transfer_to = transfer_to

        self.hasMinValue = True if min is not None else False
        self.hasMaxValue = True if max is not None else False

        if self.name and self.node:
            self.attr = self.node + '.' + self.name
        
        if add:
            if not self.type:
                mc.error("Must define type when adding attributes.")
            self.add_attr()

    def is_proxy(self) -> bool:
        sel: om2.MSelectionList = om2.MSelectionList()
        sel.add(self.attr)
        plug: om2.MPlug = sel.getPlug(0)
        return plug.isProxy

    def add_attr(self):
        self.attr = self.node + '.' + self.name
        type_dict = {'string' : self.add_string,
                     'double' : self.add_double,
                     'bool' : self.add_bool,
                     'enum' : self.add_enum,
                     'separator' : self.add_separator,
                     'double3' : self.add_double3,
                     'color' : self.add_color,
                     'plug' : self.add_plug}
        type_dict[self.type]()

        if self.type == 'plug':
            self.value = self.children_name
        else:
            self.value = mc.getAttr(self.attr)

    def add_string(self):
        mc.addAttr(self.node, longName=self.name, dataType='string')
        mc.setAttr(self.attr, self.value, type='string')

    def add_bool(self):
        mc.addAttr(
            self.node,
            attributeType="bool",
            defaultValue=self.value if self.value else 0,
            keyable=self.keyable if self.keyable else True,
            longName=self.name,
        )

    def add_double(self):
        if not self.value:
            self.value=0
            
        if self.hasMaxValue and self.hasMinValue:
            mc.addAttr(self.node, attributeType='double', maxValue=self.max, minValue=self.min, hasMinValue=self.hasMinValue, hasMaxValue=self.hasMaxValue, defaultValue=self.value, keyable=self.keyable, longName=self.name)
        elif self.hasMaxValue:
            mc.addAttr(self.node, attributeType='double', maxValue=self.max, hasMinValue=self.hasMinValue, hasMaxValue=self.hasMaxValue, defaultValue=self.value, keyable=self.keyable, longName=self.name)
        elif self.hasMinValue:
            mc.addAttr(self.node, attributeType='double', minValue=self.min, hasMinValue=self.hasMinValue, hasMaxValue=self.hasMaxValue, defaultValue=self.value, keyable=self.keyable, longName=self.name)
        else:
            mc.addAttr(self.node, attributeType='double', hasMinValue=self.hasMinValue, hasMaxValue=self.hasMaxValue, defaultValue=self.value, keyable=self.keyable, longName=self.name)


    def add_double3(self):
        mc.addAttr(self.node, attributeType='double3', hasMinValue=self.hasMinValue, hasMaxValue=self.hasMaxValue, keyable=self.keyable, longName=self.name)
        for child in self.children_name:
            mc.addAttr(self.node, parent=self.name, attributeType='double', hasMinValue=self.hasMinValue, hasMaxValue=self.hasMaxValue, defaultValue=self.value, keyable=self.keyable, longName=self.name + child)
            
        for child in self.children_name:    
            child_attr = self.attr + child
            if self.hasMinValue:
                mc.addAttr(child_attr, edit=True, min=self.min)
            if self.hasMaxValue:
                mc.addAttr(child_attr, edit=True, max=self.max)
            mc.setAttr(child_attr, cb=True)

    def add_color(self):
        mc.addAttr(
            self.node,
            attributeType="float3",
            hasMinValue=self.hasMinValue,
            hasMaxValue=self.hasMaxValue,
            keyable=self.keyable,
            longName=self.name,
            usedAsColor=True,
        )
        for child in self.children_name:
            mc.addAttr(
                self.node,
                parent=self.name,
                attributeType="float",
                hasMinValue=self.hasMinValue,
                hasMaxValue=self.hasMaxValue,
                defaultValue=self.value,
                keyable=self.keyable,
                longName=self.name + child,
            )

        for child in self.children_name:
            child_attr = self.attr + child
            if self.hasMinValue:
                mc.addAttr(child_attr, edit=True, min=self.min)
            if self.hasMaxValue:
                mc.addAttr(child_attr, edit=True, max=self.max)
            mc.setAttr(child_attr, cb=True)

    def add_enum(self):
        if self.enum_list:
            enum_name = ':'.join(self.enum_list) + ':'
        mc.addAttr(self.node, attributeType='enum', defaultValue=self.value, enumName=enum_name, keyable=self.keyable, longName=self.name)

    def add_separator(self):
        mc.addAttr(self.node, attributeType='enum', enumName='________', keyable=False, longName=self.name)
        mc.setAttr(self.attr, cb=True)

    def add_plug(self):
        mc.addAttr(self.node, numberOfChildren=len(self.children_name), attributeType='compound', longName=self.name)
        for child in self.children_name:
            mc.addAttr(self.node, longName=child, dt='string', parent=self.name)
        for plug, val in zip(mc.listAttr(self.attr)[1:], self.value):
            mc.setAttr(self.node + '.' + plug, val, type='string')

    def lock_and_hide(self, node=None, translate=True, rotate=True, scale=True,
                      visibility=True, attribute_list=None):
        if not node:
            node = self.node

        for axis in 'XYZ':
            if translate:
                if isinstance(translate, str) and axis not in translate:
                    continue
                else:
                    pass
                mc.setAttr('{}.translate{}'.format(node, axis), lock=True)
                mc.setAttr('{}.translate{}'.format(node, axis), keyable=False)
            if rotate:
                if isinstance(rotate, str) and axis not in rotate:
                    continue
                else:
                    pass
                mc.setAttr('{}.rotate{}'.format(node, axis), lock=True)
                mc.setAttr('{}.rotate{}'.format(node, axis), keyable=False)
            if scale:
                if isinstance(scale, str) and axis not in scale:
                    continue
                else:
                    pass
                mc.setAttr('{}.scale{}'.format(node, axis), lock=True)
                mc.setAttr('{}.scale{}'.format(node, axis), keyable=False)

        if visibility:
            mc.setAttr(node + '.visibility', lock=True)
            mc.setAttr(node + '.visibility', keyable=False)

        if attribute_list:
            for attr in attribute_list:
                mc.setAttr('{}.{}'.format(node, attr), lock=True)
                mc.setAttr('{}.{}'.format(node, attr), keyable=False)

    def transfer_attr(self, connect=True):
        old_attr = self.attr
        self.get_attr()
        self.node = self.transfer_to
        if self.is_proxy() and connect:
            sel: om2.MSelectionList = om2.MSelectionList()
            sel.add(old_attr)
            plug: om2.MPlug = sel.getPlug(0)
            source: om2.MPlug = plug.source()
            source_node_name = om2.MFnDependencyNode(source.node()).name()
            source_attr_name = source.partialName()
            mc.addAttr(self.transfer_to, longName=self.name, proxy=f"{source_node_name}.{source_attr_name}")
        else:
            self.add_attr()
            if connect:
                mc.connectAttr(self.attr, old_attr)

    def get_attr(self):
        if 'stretch' in self.attr or 'squash' in self.attr:
            self.min, self.max = 0, 1
        if self.min == None:
            self.hasMinValue = False
        else:
            self.hasMinValue = True

        if self.max == None:
            self.hasMaxValue = False
        else:
            self.hasMaxValue = True

        if not self.type:
            self.type = mc.getAttr(self.attr, type=True)
        if self.hasMinValue:
            self.min = mc.attributeQuery(self.name, node=self.node, minimum=True)[0]
        if self.hasMaxValue:
            self.max = mc.attributeQuery(self.name, node=self.node, maximum=True)[0]
        if not self.value:
            self.value = mc.getAttr(self.attr)
        if not self.keyable:
            self.keyable = mc.getAttr(self.attr, keyable=True)


        
        

