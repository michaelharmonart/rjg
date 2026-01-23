import maya.cmds as cmds

# -----------------------------
# Global storage
# -----------------------------

ORDERED_VERTS = []
_script_job_id = None
_last_selection = set()


# -----------------------------
# Script job callback
# -----------------------------

def _on_selection_changed():
    global _last_selection, ORDERED_VERTS

    current = set(cmds.ls(sl=True, fl=True) or [])

    # Newly added components
    added = current - _last_selection

    for item in added:
        if '.vtx[' in item:
            ORDERED_VERTS.append(item)

    _last_selection = current


# -----------------------------
# Public API
# -----------------------------

def start_recording():
    """Begin recording vertex selection order"""
    global _script_job_id, ORDERED_VERTS, _last_selection

    stop_recording()

    ORDERED_VERTS.clear()
    _last_selection = set(cmds.ls(sl=True, fl=True) or [])

    _script_job_id = cmds.scriptJob(
        event=["SelectionChanged", _on_selection_changed],
        protected=True
    )

    print("▶ Vertex order recording started")


def stop_recording():
    """Stop recording"""
    global _script_job_id

    if _script_job_id and cmds.scriptJob(exists=_script_job_id):
        cmds.scriptJob(kill=_script_job_id, force=True)

    _script_job_id = None
    print("■ Vertex order recording stopped")


def get_ordered_vertices():
    """Return recorded vertices"""
    return list(ORDERED_VERTS)
        
start_recording()


stop_recording()

print(get_ordered_vertices())