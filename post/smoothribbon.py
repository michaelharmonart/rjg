import maya.cmds as mc

def smooth_nurbs_skin(surface, strength=0.5, iterations=1):
    skin = mc.ls(mc.listHistory(surface), type="skinCluster")
    if not skin:
        raise RuntimeError("No skinCluster found")
    skin = skin[0]

    infs = mc.skinCluster(skin, q=True, inf=True)

    # Get CV ranges
    spansU = mc.getAttr(surface + ".spansU")
    spansV = mc.getAttr(surface + ".spansV")
    degU = mc.getAttr(surface + ".degreeU")
    degV = mc.getAttr(surface + ".degreeV")

    maxU = spansU + degU
    maxV = spansV + degV

    for _ in range(iterations):
        new_weights = {}

        for u in range(maxU):
            for v in range(maxV):
                cv = f"{surface}.cv[{u}][{v}]"

                # Current weights
                w = mc.skinPercent(skin, cv, q=True, v=True)

                neighbors = []

                for du, dv in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nu, nv = u + du, v + dv
                    if 0 <= nu < maxU and 0 <= nv < maxV:
                        ncv = f"{surface}.cv[{nu}][{nv}]"
                        nw = mc.skinPercent(skin, ncv, q=True, v=True)
                        neighbors.append(nw)

                if not neighbors:
                    continue

                # Average neighbor weights
                avg = [sum(n[i] for n in neighbors) / len(neighbors) for i in range(len(infs))]

                # Blend with original
                blended = [
                    (1 - strength) * w[i] + strength * avg[i]
                    for i in range(len(infs))
                ]

                new_weights[cv] = blended

        # Apply
        for cv, weights in new_weights.items():
            mc.skinPercent(
                skin,
                cv,
                transformValue=list(zip(infs, weights))
            )
            