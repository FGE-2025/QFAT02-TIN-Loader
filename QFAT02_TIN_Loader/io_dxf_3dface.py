# -*- coding: utf-8 -*-


def has_dxf_3dface_entity(path):
    """Return True when an ASCII/text DXF contains at least one real 3DFACE entity.

    This is intentionally a lightweight sniff used for drag/drop routing.
    It looks for a DXF entity marker pair:
        0
        3DFACE
    Normal DXF files without 3DFACE are left for QGIS native DXF loading.
    """
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            prev = None
            for line in f:
                cur = line.strip()
                if prev == "0" and cur.upper() == "3DFACE":
                    return True
                prev = cur
    except Exception:
        return False
    return False


def parse_dxf_3dface(path):
    """Parse ASCII DXF 3DFACE entities into vertices and triangle faces.

    DXF 3DFACE vertex codes are grouped by coordinate axis:
      v1 = 10,20,30
      v2 = 11,21,31
      v3 = 12,22,32
      v4 = 13,23,33
    A 3DFACE may store a triangle by repeating vertex 3 as vertex 4.
    """
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = [ln.strip() for ln in f]

    verts = []
    tris = []
    vmap = {}

    def add_vertex(pt):
        key = (float(pt[0]), float(pt[1]), float(pt[2]))
        if key not in vmap:
            vmap[key] = len(verts) + 1
            verts.append(key)
        return vmap[key]

    def get_point(coords, x_code, y_code, z_code):
        x = coords.get(x_code)
        y = coords.get(y_code)
        z = coords.get(z_code, 0.0)
        if x is None or y is None:
            return None
        return (x, y, z)

    i = 0
    while i + 1 < len(lines):
        if lines[i] == "0" and lines[i + 1].upper() == "3DFACE":
            i += 2
            coords = {}
            while i + 1 < len(lines):
                code = lines[i]
                val = lines[i + 1]
                if code == "0":
                    break
                if code in ("10", "20", "30", "11", "21", "31", "12", "22", "32", "13", "23", "33"):
                    try:
                        coords[code] = float(val)
                    except Exception:
                        pass
                i += 2

            pts = []
            for codes in (("10", "20", "30"), ("11", "21", "31"), ("12", "22", "32"), ("13", "23", "33")):
                pt = get_point(coords, *codes)
                if pt is not None:
                    pts.append(pt)

            if len(pts) >= 3:
                a = add_vertex(pts[0])
                b = add_vertex(pts[1])
                c = add_vertex(pts[2])
                if len({a, b, c}) == 3:
                    tris.append((a, b, c))

                # If vertex 4 is different from vertex 3, the 3DFACE is a quad.
                # Split it consistently into two triangles.
                if len(pts) >= 4 and pts[3] != pts[2]:
                    d = add_vertex(pts[3])
                    if len({a, c, d}) == 3:
                        tris.append((a, c, d))
            continue
        i += 1

    return verts, tris
