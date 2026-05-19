# -*- coding: utf-8 -*-


def write_2dm(out_path, verts, tris):
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("MESH2D\n")
        for i, (x, y, z) in enumerate(verts, start=1):
            f.write(f"ND {i} {x:.10g} {y:.10g} {z:.10g}\n")
        for i, face in enumerate(tris, start=1):
            if len(face) == 4:
                a, b, c, d = face
                f.write(f"E4Q {i} {a} {b} {c} {d}\n")
            else:
                a, b, c = face[:3]
                f.write(f"E3T {i} {a} {b} {c}\n")


def parse_2dm(path):
    verts = []
    tris = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for ln in f:
            s = ln.strip()
            if not s:
                continue
            if s.startswith("ND"):
                parts = s.split()
                if len(parts) >= 5:
                    _, nid, x, y, z = parts[:5]
                else:
                    _, nid, x, y = parts[:4]
                    z = "0"
                verts.append((float(x), float(y), float(z)))
            elif s.startswith("E3T"):
                parts = s.split()
                if len(parts) >= 5:
                    _, eid, a, b, c = parts[:5]
                    tris.append((int(a), int(b), int(c)))
            elif s.startswith("E4Q"):
                parts = s.split()
                if len(parts) >= 6:
                    _, eid, a, b, c, d = parts[:6]
                    tris.append((int(a), int(b), int(c), int(d)))
    if not verts or not tris:
        raise ValueError("No vertices or triangles parsed from 2DM")
    return verts, tris


def parse_mdal_tin(path):
    verts = []
    tris = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        lines = [ln.strip() for ln in f if ln.strip()]
    i = 0
    if not lines or not lines[i].startswith("TIN"):
        raise ValueError("Not a TIN file")
    while i < len(lines) and not lines[i].startswith("BEGT"):
        i += 1
    if i == len(lines):
        raise ValueError("Missing BEGT")
    i += 1
    while i < len(lines) and (lines[i].startswith("TNAM") or lines[i].startswith("MAT")):
        i += 1
    if i == len(lines) or not lines[i].startswith("VERT"):
        raise ValueError("Missing VERT")
    n_vert = int(lines[i].split()[1])
    i += 1
    for _ in range(n_vert):
        parts = lines[i].split()
        i += 1
        verts.append((float(parts[0]), float(parts[1]), float(parts[2])))
    while i < len(lines) and not lines[i].startswith("TRI"):
        i += 1
    if i == len(lines):
        raise ValueError("Missing TRI")
    n_tri = int(lines[i].split()[1])
    i += 1
    for _ in range(n_tri):
        a, b, c = [int(v) for v in lines[i].split()[:3]]
        i += 1
        tris.append((a, b, c))
    return verts, tris
