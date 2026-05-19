# -*- coding: utf-8 -*-
import io
import re


def _read_tin_lines(path, max_lines=None):
    """Read text lines from a .tin using practical encoding fallbacks."""
    data = open(path, "rb").read()
    if data.startswith(b"\xef\xbb\xbf"):
        data = data[3:]
    for enc in ("utf-8", "cp1252", "latin-1"):
        try:
            text = data.decode(enc)
            break
        except UnicodeDecodeError:
            text = None
    if text is None:
        text = data.decode("cp1252", errors="replace")
    lines = [ln.rstrip("\n\r") for ln in text.splitlines()]
    if max_lines is not None:
        return lines[:max_lines]
    return lines


def _is_int_text(s):
    try:
        int(s)
        return True
    except Exception:
        return False


def _find_keyword(lines, keyword, start=0):
    key = keyword.upper()
    for i in range(start, len(lines)):
        parts = lines[i].strip().split()
        if parts and parts[0].upper() == key:
            return i
    return -1


def _first_data_row_col_count(lines, start, stop_keywords):
    stops = tuple(k.upper() for k in stop_keywords)
    for i in range(start, len(lines)):
        row = lines[i].strip()
        if not row:
            continue
        up = row.upper()
        if up.startswith(stops):
            break
        return len(row.split())
    return None


def detect_tin_format(path):
    """Return a practical format label for ambiguous .tin files.

    Important: do not only scan the first few hundred lines. Large TIN files may
    have thousands of VERT rows before the TRI header, so the TRI header can be
    far below the top of the file.

    Labels:
    - sms_xms: SMS/XMS-style TIN needing normalisation before MDAL/QGIS load.
    - mdal_tin: appears already MDAL/QGIS-readable ASCII TIN.
    - ascii_tin: recognised ASCII TIN, but not enough detail to choose.
    - unknown: not a recognised supported ASCII TIN.
    """
    lines = _read_tin_lines(path)
    useful = [ln.strip() for ln in lines if ln.strip() and not ln.strip().startswith(("#", "//"))]
    if not useful:
        return "unknown"

    first_parts = useful[0].split()
    if not first_parts or first_parts[0].upper() != "TIN":
        return "unknown"

    has_begt = any(ln.upper().split()[0] == "BEGT" for ln in useful if ln.split())
    has_endt = any(ln.upper().split()[0] == "ENDT" for ln in useful if ln.split())
    has_sms_clues = any(ln.upper().split()[0] in ("TNAM", "MAT") for ln in useful if ln.split())

    vert_i = _find_keyword(useful, "VERT", 0)
    if vert_i < 0:
        return "unknown"
    vert_parts = useful[vert_i].split()
    n_vert = int(vert_parts[1]) if len(vert_parts) > 1 and _is_int_text(vert_parts[1]) else None
    vert_cols = _first_data_row_col_count(useful, vert_i + 1, ("TRI", "ENDT", "BEGT", "TIN"))

    # Prefer a direct jump over the vertex block when VERT count is available.
    tri_i = -1
    if n_vert is not None:
        tri_i = _find_keyword(useful, "TRI", vert_i + 1 + n_vert)
    if tri_i < 0:
        tri_i = _find_keyword(useful, "TRI", vert_i + 1)
    if tri_i < 0:
        return "unknown"

    # SMS/XMS exports normally use 3-column vertex rows and may contain TNAM/MAT.
    # QGIS/MDAL-friendly TIN output commonly has 4-column vertex rows.
    if has_sms_clues or vert_cols == 3:
        return "sms_xms"
    if vert_cols and vert_cols >= 4:
        return "mdal_tin"
    if has_begt or has_endt:
        return "ascii_tin"
    return "ascii_tin"


def parse_tin_auto(path):
    """Parse a supported ASCII .tin to vertices/faces using detected format."""
    fmt = detect_tin_format(path)
    if fmt in ("sms_xms", "mdal_tin", "ascii_tin"):
        return parse_sms_tin(path)
    raise ValueError("Unsupported or unrecognised .tin format")

def normalize_sms_tin(src, dst):
    def push(buf, s):
        buf.write(s + "\n")

    lines = _read_tin_lines(src)
    useful = [ln.strip() for ln in lines if ln.strip()]
    if not useful or not useful[0].upper().startswith("TIN"):
        raise ValueError("Not an ASCII TIN (missing 'TIN').")

    out = io.StringIO()
    push(out, "TIN")
    push(out, "BEGT")

    i_vert = _find_keyword(useful, "VERT", 0)
    if i_vert < 0:
        raise ValueError("Missing VERT section.")
    m = re.match(r"\s*VERT\s+(\d+)", useful[i_vert], re.IGNORECASE)
    if not m:
        raise ValueError("Malformed VERT header.")
    n_vert = int(m.group(1))
    push(out, f"VERT {n_vert}")

    i = i_vert + 1
    verts = 0
    while verts < n_vert and i < len(useful):
        raw = useful[i].strip()
        i += 1
        up = raw.upper()
        if up.startswith(("TRI", "ENDT", "BEGT")):
            break
        parts = raw.split()
        if len(parts) < 3:
            continue
        x, y, z = parts[0:3]
        fourth = parts[3] if len(parts) >= 4 else "0"
        push(out, f" {x} {y} {z} {fourth}")
        verts += 1
    if verts != n_vert:
        raise ValueError("VERT count does not match vertex rows found.")

    i_tri = _find_keyword(useful, "TRI", i_vert + 1 + n_vert)
    if i_tri < 0:
        i_tri = _find_keyword(useful, "TRI", i_vert + 1)
    if i_tri < 0:
        raise ValueError("Missing TRI section.")
    m = re.match(r"\s*TRI\s+(\d+)", useful[i_tri], re.IGNORECASE)
    if not m:
        raise ValueError("Malformed TRI header.")
    n_tri = int(m.group(1))
    push(out, f"TRI {n_tri}")

    i = i_tri + 1
    tris = 0
    while tris < n_tri and i < len(useful):
        raw = useful[i].strip()
        i += 1
        up = raw.upper()
        if up.startswith(("ENDT", "BEGT", "TIN")):
            break
        parts = raw.split()
        if len(parts) < 3:
            continue
        a, b, c = parts[0:3]
        push(out, f"  {a}   {b}   {c}")
        tris += 1
    if tris != n_tri:
        raise ValueError("TRI count does not match triangle rows found.")

    push(out, "ENDT")
    with open(dst, "w", encoding="utf-8") as f:
        f.write(out.getvalue())

def parse_sms_tin(path):
    """Parse SMS/XMS-style or MDAL/QGIS-style ASCII TIN into vertices/faces.

    This parser is intentionally tolerant because .tin files share the same
    extension but vary slightly. It accepts optional BEGT/ENDT wrappers and
    3-column or 4-column vertex rows.
    """
    verts = []
    tris = []
    lines = _read_tin_lines(path)
    useful = [ln.strip() for ln in lines if ln.strip()]
    if not useful or not useful[0].upper().startswith("TIN"):
        raise ValueError("Missing TIN header")

    i_vert = _find_keyword(useful, "VERT", 0)
    if i_vert < 0:
        raise ValueError("Missing VERT")
    m = re.match(r"\s*VERT\s+(\d+)", useful[i_vert], re.IGNORECASE)
    if not m:
        raise ValueError("Bad VERT header")
    n_vert = int(m.group(1))

    i = i_vert + 1
    while len(verts) < n_vert and i < len(useful):
        raw = useful[i].strip()
        i += 1
        up = raw.upper()
        if up.startswith(("TRI", "ENDT", "BEGT", "TIN")):
            break
        parts = raw.split()
        if len(parts) < 3:
            continue
        verts.append((float(parts[0]), float(parts[1]), float(parts[2])))
    if len(verts) != n_vert:
        raise ValueError("VERT count does not match vertex rows found")

    i_tri = _find_keyword(useful, "TRI", i_vert + 1 + n_vert)
    if i_tri < 0:
        i_tri = _find_keyword(useful, "TRI", i_vert + 1)
    if i_tri < 0:
        raise ValueError("Missing TRI")
    m = re.match(r"\s*TRI\s+(\d+)", useful[i_tri], re.IGNORECASE)
    if not m:
        raise ValueError("Bad TRI header")
    n_tri = int(m.group(1))

    i = i_tri + 1
    while len(tris) < n_tri and i < len(useful):
        raw = useful[i].strip()
        i += 1
        up = raw.upper()
        if up.startswith(("ENDT", "BEGT", "TIN")):
            break
        parts = raw.split()
        if len(parts) < 3:
            continue
        tris.append((int(parts[0]), int(parts[1]), int(parts[2])))
    if len(tris) != n_tri:
        raise ValueError("TRI count does not match triangle rows found")
    if not verts or not tris:
        raise ValueError("No vertices or triangles found")
    return verts, tris
