from functools import wraps
import maya.cmds as cmds
from rjg.libs.color import color_from_name, float_to_byte_color

def auto_profiler_tag(build_method):
    @wraps(build_method)
    def wrapper(self, *args, **kwargs):
        # Capture the state of the scene before anything happens
        before_nodes = set(cmds.ls())
        
        # Run the actual build logic
        result = build_method(self, *args, **kwargs)
        
        # Check if the object is a rig module.
        if hasattr(self, "tag_created_nodes"):
            self.tag_created_nodes(before_nodes)
        return result
    return wrapper
        
def add_profiler_tag(node: str | list[str], tag_name: str, tag_color: tuple[float, float, float] | None = None, allow_mesh: bool = False):
    """
    Add a profiler tag to a node for rig speed profiling based on part/name.

    Args:
        node: Node(s) to be tagged.
        tag_name: Name for the tag (for example a rig part name).
        tag_color: RGB color for the tag, if None it will be generated automatically from the tag_name.
    """

    
    # Create the data structure. See documentation here https://help.autodesk.com/view/MAYAUL/2024/ENU/?guid=GUID-8D5FFC12-608C-45EA-B035-1AB56F3C42F1
    if "NodeProfileStruct" not in (cmds.dataStructure(q=True) or []):
        cmds.dataStructure(format="raw", asString="name=NodeProfileStruct:string=NodeProfileTag:int32=NodeProfileTagColor")

    if isinstance(node, list):
        nodes = node
    else:
        nodes = [node]
    
    for node in nodes:
        try:
            if not allow_mesh and cmds.nodeType(node) == "mesh":
                continue
                
            # Add metadata channels only if they don't yet exist
            extant_metadata: list[str] = (cmds.addMetadata(node, q=True, channelName=True) or [])
            if "ProfileTag" in extant_metadata and "ProfileTagColor" in extant_metadata:
                continue
            if "ProfileTag" not in extant_metadata:
                cmds.addMetadata(node, streamName="ProfileTagStream", channelName="ProfileTag", structure="NodeProfileStruct")
            if "ProfileTagColor" not in extant_metadata:
                cmds.addMetadata(node, streamName="ProfileTagColorStream", channelName="ProfileTagColor", structure="NodeProfileStruct")
        
            # Set the actual metadata
            cmds.editMetadata(node, streamName="ProfileTagStream", memberName="NodeProfileTag", channelName="ProfileTag", stringValue=tag_name, index=0)
            
            # Get the tag_color value
            if tag_color is not None:
                color: tuple[int, int, int] = float_to_byte_color(tag_color)
            else:
                color: tuple[int, int, int] = float_to_byte_color(color_from_name(tag_name))
            cmds.editMetadata(node, streamName="ProfileTagColorStream", memberName="NodeProfileTagColor", channelName="ProfileTagColor", value=color[0], index=0)
            cmds.editMetadata(node, streamName="ProfileTagColorStream", memberName="NodeProfileTagColor", channelName="ProfileTagColor", value=color[1], index=1)
            cmds.editMetadata(node, streamName="ProfileTagColorStream", memberName="NodeProfileTagColor", channelName="ProfileTagColor", value=color[2], index=2)
        except Exception as e:
            print(f"{node} -- {e}")
