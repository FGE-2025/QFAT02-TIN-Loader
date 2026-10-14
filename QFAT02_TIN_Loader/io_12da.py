# -*- coding: utf-8 -*-
import gzip
import os
import zipfile


def _text_score(text):
    """Score decoded text so UTF-16 without BOM can beat ANSI mis-decoding."""
    if not text:
        return -9999
    sample = text[:200000]
    low = sample.lower()
    score = 0
    for token in ("points", "triangles", "nulling", "tin"):
        if token in low:
            score += 20
    # Penalise text that still looks like raw UTF-16 decoded as single-byte text.
    nul_count = sample.count("\x00")
    if nul_count:
        score -= min(80, nul_count * 2)
    printable = sum(1 for ch in sample if ch in "\r\n\t" or ord(ch) >= 32)
    score += int(20.0 * printable / max(1, len(sample)))
    return score


def _decode_12d_text(data):
    """Auto-detect 12d text encoding and return UTF-8-safe text + metadata.

    QFAT02 parses Unicode text internally. This function detects common 12d
    export encodings, decodes them, and normalises the text to Python Unicode
    (equivalent to UTF-8 text for the parser/writers).
    """
    bom_map = (
        (b"\xef\xbb\xbf", "utf-8-sig", "UTF-8 BOM"),
        (b"\xff\xfe", "utf-16", "UTF-16 LE BOM"),
        (b"\xfe\xff", "utf-16", "UTF-16 BE BOM"),
    )
    for bom, codec, label in bom_map:
        if data.startswith(bom):
            text = data.decode(codec, errors="replace")
            return text, {
                "encoding": label,
                "codec": codec,
                "converted_to_utf8": True,
                "warn_encoding": label not in ("UTF-8 BOM",),
            }

    candidates = []
    for codec, label in (
        ("utf-8", "UTF-8"),
        ("utf-16-le", "UTF-16 LE"),
        ("utf-16-be", "UTF-16 BE"),
        ("cp1252", "ANSI / CP1252"),
        ("latin-1", "Latin-1 fallback"),
    ):
        try:
            decoded = data.decode(codec)
        except UnicodeDecodeError:
            continue
        candidates.append((_text_score(decoded), decoded, codec, label))

    if candidates:
        candidates.sort(key=lambda item: item[0], reverse=True)
        score, text, codec, label = candidates[0]
        return text, {
            "encoding": label,
            "codec": codec,
            "converted_to_utf8": True,
            "warn_encoding": label not in ("UTF-8", "ANSI / CP1252"),
        }

    # Last resort should rarely happen, but keeps the importer from crashing on
    # odd text exports. Replacement characters may still cause parse failure.
    text = data.decode("utf-8", errors="replace")
    return text, {
        "encoding": "UTF-8 replacement fallback",
        "codec": "utf-8",
        "converted_to_utf8": True,
        "warn_encoding": True,
    }


def _read_raw_12d_bytes(path):
    """Read .12da text, including common .12daz compressed/archive forms."""
    low = path.lower()
    if low.endswith(".12daz"):
        if zipfile.is_zipfile(path):
            with zipfile.ZipFile(path) as zf:
                names = [n for n in zf.namelist() if not n.endswith("/")]
                if not names:
                    raise ValueError("The .12daz archive is empty.")
                preferred_12da = [n for n in names if n.lower().endswith(".12da")]
                preferred_other = [n for n in names if n.lower().endswith((".4da", ".txt"))]
                preferred = preferred_12da or preferred_other
                # Expected case: one main .12da in the .12daz archive. If there
                # are multiple candidates, use the largest as the practical main file.
                name = max(preferred, key=lambda n: zf.getinfo(n).file_size) if preferred else max(names, key=lambda n: zf.getinfo(n).file_size)
                return zf.read(name), name
        try:
            with gzip.open(path, "rb") as f:
                return f.read(), os.path.basename(path)
        except Exception:
            pass
    with open(path, "rb") as f:
        return f.read(), os.path.basename(path)


def read_12da_text(path):
    data, member_name = _read_raw_12d_bytes(path)
    text, meta = _decode_12d_text(data)
    meta["member_name"] = member_name
    return text, meta, member_name


def strip_comment(s: str) -> str:
    return s.split('//', 1)[0].strip()


def parse_12d_float(value: str) -> float:
    """Parse decimal or hexadecimal floating-point values used by 12d exports."""
    value = value.strip()
    if value.lower().lstrip("+-").startswith("0x"):
        return float.fromhex(value)
    return float(value)


def parse_12da(path):
    """Parse a 12d .12da TIN: reads points, triangles, nulling (visibility)."""
    points, triangles, nulling = [], [], []
    in_points = in_tris = in_null = False
    seen = {"points": False, "triangles": False, "nulling": False}
    text, encoding_meta, member_name = read_12da_text(path)
    for raw in text.splitlines():
        line = strip_comment(raw)
        if not line:
            continue
        low = line.lower()
        if low.startswith('points'):
            in_points, in_tris, in_null = True, False, False
            seen["points"] = False
            continue
        if low.startswith('triangles'):
            in_points, in_tris, in_null = False, True, False
            seen["triangles"] = False
            continue
        if low.startswith('nulling'):
            in_points, in_tris, in_null = False, False, True
            seen["nulling"] = False
            continue
        if line == '}':
            in_points = in_tris = in_null = False
            continue

        if in_points:
            if not seen["points"]:
                seen["points"] = True
                toks = line.split()
                if len(toks) == 1 and toks[0].lstrip('+-').isdigit():
                    continue
            toks = line.replace(',', ' ').split()
            if len(toks) >= 3:
                try:
                    points.append((parse_12d_float(toks[0]), parse_12d_float(toks[1]), parse_12d_float(toks[2])))
                except Exception:
                    pass
        elif in_tris:
            if not seen["triangles"]:
                seen["triangles"] = True
                toks = line.split()
                if len(toks) == 1 and toks[0].lstrip('+-').isdigit():
                    continue
            toks = line.replace(',', ' ').split()
            if len(toks) >= 3:
                try:
                    triangles.append((int(toks[0]), int(toks[1]), int(toks[2])))
                except Exception:
                    pass
        elif in_null:
            if not seen["nulling"]:
                seen["nulling"] = True
                toks = line.split()
                if len(toks) == 1 and toks[0].lstrip('+-').isdigit():
                    continue
            toks = line.replace(',', ' ').split()
            for t in toks:
                try:
                    nulling.append(int(t))
                except Exception:
                    pass
    visible = [True] * len(triangles)
    if nulling and len(nulling) >= len(triangles):
        visible = [(v == 2) for v in nulling[:len(triangles)]]
    return {
        "points": points,
        "triangles": triangles,
        "visible": visible,
        "encoding": encoding_meta.get("encoding", "unknown"),
        "encoding_codec": encoding_meta.get("codec", ""),
        "encoding_warn": bool(encoding_meta.get("warn_encoding", False)),
        "converted_to_utf8": bool(encoding_meta.get("converted_to_utf8", True)),
        "member_name": member_name,
    }


def write_tin(points, triangles, visible, out_path):
    """Write MDAL ASCII .TIN from arrays. Assumes triangles already filtered/reindexed as desired."""
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write("TIN\nBEGT\n")
        f.write(f"VERT {len(points)}\n")
        for (x, y, z) in points:
            f.write(f"{x} {y} {z} 0\n")
        f.write(f"TRI {len(triangles)}\n")
        for (i, j, k) in triangles:
            f.write(f"{i} {j} {k}\n")
        f.write("ENDT\n")


def tri_area_xy(a, b, c):
    return abs(0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])))


def apply_modifications(points, triangles, visible, opts):
    """Apply filters/transforms; returns (points, triangles, visible) ready to write."""
    pts = [list(p) for p in points]
    tris = list(triangles)
    vis = list(visible)

    dx = opts.get("dx", 0.0)
    dy = opts.get("dy", 0.0)
    z_add = opts.get("z_add", 0.0)
    z_scale = opts.get("z_scale", 1.0)
    if dx or dy or z_add or (z_scale != 1.0):
        for p in pts:
            p[0] = p[0] + dx
            p[1] = p[1] + dy
            p[2] = p[2] * z_scale + z_add

    if opts.get("visible_only", True):
        keep = [i for i, v in enumerate(vis) if v]
        tris = [tris[i] for i in keep]
        vis = [vis[i] for i in keep]

    if opts.get("drop_construction", False):
        new_tris, new_vis = [], []
        for t, v in zip(tris, vis):
            if (t[0] <= 4) or (t[1] <= 4) or (t[2] <= 4):
                continue
            new_tris.append(t)
            new_vis.append(v)
        tris, vis = new_tris, new_vis

    if opts.get("use_bbox", False):
        xmin, ymin, xmax, ymax = opts.get("bbox", (0, 0, 0, 0))
        if xmin > xmax:
            xmin, xmax = xmax, xmin
        if ymin > ymax:
            ymin, ymax = ymax, ymin
        new_tris, new_vis = [], []
        for t, v in zip(tris, vis):
            a = pts[t[0] - 1]
            b = pts[t[1] - 1]
            c = pts[t[2] - 1]
            inside = (xmin <= a[0] <= xmax and ymin <= a[1] <= ymax and
                      xmin <= b[0] <= xmax and ymin <= b[1] <= ymax and
                      xmin <= c[0] <= xmax and ymin <= c[1] <= ymax)
            if inside:
                new_tris.append(t)
                new_vis.append(v)
        tris, vis = new_tris, new_vis

    min_area = max(0.0, float(opts.get("min_area", 0.0)))
    if min_area > 0.0:
        new_tris, new_vis = [], []
        for t, v in zip(tris, vis):
            a = pts[t[0] - 1]
            b = pts[t[1] - 1]
            c = pts[t[2] - 1]
            if tri_area_xy(a, b, c) >= min_area:
                new_tris.append(t)
                new_vis.append(v)
        tris, vis = new_tris, new_vis

    if opts.get("reindex", True):
        used = set()
        for (i, j, k) in tris:
            used.update([i, j, k])
        used_sorted = sorted(used)
        mapping = {old: i + 1 for i, old in enumerate(used_sorted)}
        new_pts = [pts[old - 1] for old in used_sorted]
        new_tris = [(mapping[i], mapping[j], mapping[k]) for (i, j, k) in tris]
        pts, tris = new_pts, new_tris

    return pts, tris, vis
