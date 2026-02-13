# -*- coding: utf-8 -*-
# QFAT02 TIN Loader v2.1.0 — quick open to .TIN Mesh (auto-saved), plus options dialog; proper unload()
from qgis.PyQt.QtWidgets import (
    QAction, QFileDialog, QMessageBox, QDialog, QVBoxLayout, QHBoxLayout,
    QLabel, QCheckBox, QDoubleSpinBox, QGroupBox, QPushButton, QGridLayout
)
from qgis.PyQt.QtGui import QIcon
from qgis.core import QgsProject, QgsMeshLayer
import os, tempfile, re, urllib.parse

def _strip_comment(s: str) -> str:
    return s.split('//', 1)[0].strip()

def parse_12da(path):
    """Parse a 12d .12da TIN: reads points, triangles, nulling (visibility)."""
    points, triangles, nulling = [], [], []
    in_points = in_tris = in_null = False
    seen = {"points": False, "triangles": False, "nulling": False}
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw in f:
            line = _strip_comment(raw)
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
                        continue  # count line
                toks = line.replace(',', ' ').split()
                if len(toks) >= 3:
                    try:
                        x, y, z = float(toks[0]), float(toks[1]), float(toks[2])
                        points.append((x, y, z))
                    except Exception:
                        pass
            elif in_tris:
                if not seen["triangles"]:
                    seen["triangles"] = True
                    toks = line.split()
                    if len(toks) == 1 and toks[0].lstrip('+-').isdigit():
                        continue  # count line
                toks = line.replace(',', ' ').split()
                if len(toks) >= 3:
                    try:
                        i, j, k = int(toks[0]), int(toks[1]), int(toks[2])
                        triangles.append((i, j, k))
                    except Exception:
                        pass
            elif in_null:
                if not seen["nulling"]:
                    seen["nulling"] = True
                    toks = line.split()
                    if len(toks) == 1 and toks[0].lstrip('+-').isdigit():
                        continue  # count line
                toks = line.replace(',', ' ').split()
                for t in toks:
                    try:
                        nulling.append(int(t))
                    except Exception:
                        pass
    visible = [True]*len(triangles)
    if nulling and len(nulling) >= len(triangles):
        visible = [ (v == 2) for v in nulling[:len(triangles)] ]
    return {"points": points, "triangles": triangles, "visible": visible}

def write_tin(points, triangles, visible, out_path):
    """Write MDAL ASCII .TIN from arrays. Assumes triangles already filtered/reindexed as desired."""
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write("TIN\nBEGT\n")
        f.write(f"VERT {len(points)}\n")
        for (x,y,z) in points:
            f.write(f"{x} {y} {z} 0\n")
        f.write(f"TRI {len(triangles)}\n")
        for (i,j,k) in triangles:
            f.write(f"{i} {j} {k}\n")
        f.write("ENDT\n")

def _unique_sibling_path(src_path, suffix="_mdal.tin"):
    base_dir = os.path.dirname(src_path)
    base_name = os.path.splitext(os.path.basename(src_path))[0]
    candidate = os.path.join(base_dir, base_name + suffix)
    if not os.path.exists(candidate):
        return candidate
    n = 1
    while True:
        cand = os.path.join(base_dir, f"{base_name}{suffix[:-4]}_{n}.tin")
        if not os.path.exists(cand):
            return cand
        n += 1

def _writable_dir(path):
    try:
        testfile = tempfile.TemporaryFile(dir=path)
        testfile.close()
        return True
    except Exception:
        return False

def tri_area_xy(a, b, c):
    return abs(0.5 * ((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1])))

def apply_modifications(points, triangles, visible, opts):
    """Apply filters/transforms; returns (points, triangles, visible) ready to write."""
    pts = [list(p) for p in points]
    tris = list(triangles)
    vis = list(visible)

    # XY/Z transform
    dx = opts.get("dx", 0.0); dy = opts.get("dy", 0.0)
    z_add = opts.get("z_add", 0.0); z_scale = opts.get("z_scale", 1.0)
    if dx or dy or z_add or (z_scale != 1.0):
        for p in pts:
            p[0] = p[0] + dx
            p[1] = p[1] + dy
            p[2] = p[2] * z_scale + z_add

    # visible-only
    if opts.get("visible_only", True):
        keep = [i for i,v in enumerate(vis) if v]
        tris = [tris[i] for i in keep]
        vis  = [vis[i]  for i in keep]

    # drop construction tris touching pid 1..4
    if opts.get("drop_construction", False):
        new_tris, new_vis = [], []
        for t,v in zip(tris, vis):
            if (t[0] <= 4) or (t[1] <= 4) or (t[2] <= 4):
                continue
            new_tris.append(t); new_vis.append(v)
        tris, vis = new_tris, new_vis

    # bbox clip: keep only triangles whose all vertices inside bbox
    if opts.get("use_bbox", False):
        xmin,ymin,xmax,ymax = opts.get("bbox", (0,0,0,0))
        if xmin > xmax: xmin, xmax = xmax, xmin
        if ymin > ymax: ymin, ymax = ymax, ymin
        new_tris, new_vis = [], []
        for t,v in zip(tris, vis):
            a = pts[t[0]-1]; b = pts[t[1]-1]; c = pts[t[2]-1]
            inside = (xmin <= a[0] <= xmax and ymin <= a[1] <= ymax and
                      xmin <= b[0] <= xmax and ymin <= b[1] <= ymax and
                      xmin <= c[0] <= xmax and ymin <= c[1] <= ymax)
            if inside:
                new_tris.append(t); new_vis.append(v)
        tris, vis = new_tris, new_vis

    # min area filter
    min_area = max(0.0, float(opts.get("min_area", 0.0)))
    if min_area > 0.0:
        new_tris, new_vis = [], []
        for t,v in zip(tris, vis):
            a = pts[t[0]-1]; b = pts[t[1]-1]; c = pts[t[2]-1]
            if tri_area_xy(a,b,c) >= min_area:
                new_tris.append(t); new_vis.append(v)
        tris, vis = new_tris, new_vis

    # reindex vertices to used-only
    if opts.get("reindex", True):
        used = set()
        for (i,j,k) in tris:
            used.update([i,j,k])
        used_sorted = sorted(used)
        mapping = {old:i+1 for i,old in enumerate(used_sorted)}
        new_pts = [pts[old-1] for old in used_sorted]
        new_tris = [(mapping[i], mapping[j], mapping[k]) for (i,j,k) in tris]
        pts, tris = new_pts, new_tris

    return pts, tris, vis

class ModDialog(QDialog):
    def __init__(self, parent=None, bounds=None):
        super().__init__(parent)
        self.setWindowTitle("12da → .TIN (MDAL) – Options")
        lay = QVBoxLayout(self)

        # Basics
        self.chk_visible = QCheckBox("Use visible triangles only"); self.chk_visible.setChecked(True)
        self.chk_construction = QCheckBox("Drop triangles that touch construction vertices (pid 1–4)")
        self.chk_reindex = QCheckBox("Reindex vertices to used-only"); self.chk_reindex.setChecked(True)

        g_basic = QGroupBox("Basic filters")
        gbl = QVBoxLayout(); gbl.addWidget(self.chk_visible); gbl.addWidget(self.chk_construction); gbl.addWidget(self.chk_reindex)
        g_basic.setLayout(gbl); lay.addWidget(g_basic)

        # XY/Z
        g_tf = QGroupBox("Transforms")
        grid = QGridLayout()
        self.dx = QDoubleSpinBox(); self.dx.setDecimals(6); self.dx.setRange(-1e9,1e9); self.dx.setValue(0.0)
        self.dy = QDoubleSpinBox(); self.dy.setDecimals(6); self.dy.setRange(-1e9,1e9); self.dy.setValue(0.0)
        self.z_add = QDoubleSpinBox(); self.z_add.setDecimals(6); self.z_add.setRange(-1e9,1e9); self.z_add.setValue(0.0)
        self.z_scale = QDoubleSpinBox(); self.z_scale.setDecimals(6); self.z_scale.setRange(-1e6,1e6); self.z_scale.setValue(1.0)
        grid.addWidget(QLabel("ΔX"),0,0); grid.addWidget(self.dx,0,1)
        grid.addWidget(QLabel("ΔY"),0,2); grid.addWidget(self.dy,0,3)
        grid.addWidget(QLabel("Z add"),1,0); grid.addWidget(self.z_add,1,1)
        grid.addWidget(QLabel("Z scale"),1,2); grid.addWidget(self.z_scale,1,3)
        g_tf.setLayout(grid); lay.addWidget(g_tf)

        # Area filter
        g_area = QGroupBox("Triangle area filter (XY units)")
        h = QHBoxLayout()
        self.min_area = QDoubleSpinBox(); self.min_area.setDecimals(6); self.min_area.setRange(0.0, 1e12); self.min_area.setValue(0.0)
        h.addWidget(QLabel("Min area")); h.addWidget(self.min_area)
        g_area.setLayout(h); lay.addWidget(g_area)

        # BBox
        g_bb = QGroupBox("Clip by bounding box (all vertices inside)")
        grid_bb = QGridLayout()
        self.use_bbox = QCheckBox("Enable")
        self.xmin = QDoubleSpinBox(); self.xmin.setDecimals(6); self.xmin.setRange(-1e12,1e12)
        self.ymin = QDoubleSpinBox(); self.ymin.setDecimals(6); self.ymin.setRange(-1e12,1e12)
        self.xmax = QDoubleSpinBox(); self.xmax.setDecimals(6); self.xmax.setRange(-1e12,1e12)
        self.ymax = QDoubleSpinBox(); self.ymax.setDecimals(6); self.ymax.setRange(-1e12,1e12)
        grid_bb.addWidget(self.use_bbox,0,0,1,4)
        grid_bb.addWidget(QLabel("Xmin"),1,0); grid_bb.addWidget(self.xmin,1,1)
        grid_bb.addWidget(QLabel("Ymin"),1,2); grid_bb.addWidget(self.ymin,1,3)
        grid_bb.addWidget(QLabel("Xmax"),2,0); grid_bb.addWidget(self.xmax,2,1)
        grid_bb.addWidget(QLabel("Ymax"),2,2); grid_bb.addWidget(self.ymax,2,3)
        g_bb.setLayout(grid_bb); lay.addWidget(g_bb)

        # Buttons
        btns = QHBoxLayout()
        self.btn_ok = QPushButton("OK"); self.btn_cancel = QPushButton("Cancel")
        btns.addStretch(1); btns.addWidget(self.btn_ok); btns.addWidget(self.btn_cancel)
        lay.addLayout(btns)
        self.btn_ok.clicked.connect(self.accept); self.btn_cancel.clicked.connect(self.reject)

        # Prefill bbox
        if bounds:
            xmin,xmax,ymin,ymax = bounds
            self.xmin.setValue(xmin); self.xmax.setValue(xmax)
            self.ymin.setValue(ymin); self.ymax.setValue(ymax)

    def options(self):
        return {
            "visible_only": self.chk_visible.isChecked(),
            "drop_construction": self.chk_construction.isChecked(),
            "reindex": self.chk_reindex.isChecked(),
            "dx": float(self.dx.value()),
            "dy": float(self.dy.value()),
            "z_add": float(self.z_add.value()),
            "z_scale": float(self.z_scale.value()),
            "min_area": float(self.min_area.value()),
            "use_bbox": self.use_bbox.isChecked(),
            "bbox": (float(self.xmin.value()), float(self.ymin.value()), float(self.xmax.value()), float(self.ymax.value())),
        }

class Qfat02LoaderPlugin:
    def __init__(self, iface):
        self.iface = iface
        self.actions = []
        self.menu = "&QFAT02 TIN Loader"
        self.toolbar = self.iface.addToolBar("QFAT02 TIN Loader")
        self.toolbar.setObjectName("QFAT02_TIN_Loader_toolbar")

    def initGui(self):
        icon = QIcon(os.path.join(os.path.dirname(__file__), "icons", "qfat_logo.png"))

        # Quick one-click loader
        act_quick = QAction(icon, "Open .12da as Mesh (auto .TIN)…", self.iface.mainWindow())
        act_quick.triggered.connect(self.quick_open_12da_as_mesh_auto)
        self.iface.addPluginToMenu(self.menu, act_quick)
        self.toolbar.addAction(act_quick)
        self.actions.append(act_quick)

        # Options variant
        act_opts = QAction(icon, "Open .12da as Mesh (options)…", self.iface.mainWindow())
        act_opts.triggered.connect(self.open_12da_as_mesh_with_options)
        self.iface.addPluginToMenu(self.menu, act_opts)
        self.toolbar.addAction(act_opts)
        self.actions.append(act_opts)

        # --- QFAT02 additions ---
        # SMS TIN variant (normalize then load)
        act_sms = QAction(icon, "Open .tin (SMS)…", self.iface.mainWindow())
        act_sms.triggered.connect(self.quick_open_tin_sms)
        self.iface.addPluginToMenu(self.menu, act_sms)
        self.toolbar.addAction(act_sms)
        self.actions.append(act_sms)

        # Save current mesh as 2DM (editable)
        act_save2dm = QAction(icon, "Save current mesh as 2DM (editable)…", self.iface.mainWindow())
        act_save2dm.triggered.connect(self.save_active_mesh_as_2dm)
        self.iface.addPluginToMenu(self.menu, act_save2dm)
        self.toolbar.addAction(act_save2dm)
        self.actions.append(act_save2dm)

        # Converters: direct to 2DM (editable)
        act_conv12 = QAction(icon, "Convert .12da → 2DM (editable)…", self.iface.mainWindow())
        act_conv12.triggered.connect(self.convert_12da_to_2dm_quick)
        self.iface.addPluginToMenu(self.menu, act_conv12)
        self.toolbar.addAction(act_conv12)
        self.actions.append(act_conv12)

        act_convSms = QAction(icon, "Convert .tin (SMS) → 2DM (editable)…", self.iface.mainWindow())
        act_convSms.triggered.connect(self.convert_sms_tin_to_2dm_quick)
        self.iface.addPluginToMenu(self.menu, act_convSms)
        self.toolbar.addAction(act_convSms)
        self.actions.append(act_convSms)

    def unload(self):
        try:
            for a in self.actions:
                try:
                    self.iface.removePluginMenu(self.menu, a)
                except Exception:
                    pass
                try:
                    self.toolbar.removeAction(a)
                except Exception:
                    pass
        finally:
            try:
                self.iface.mainWindow().removeToolBar(self.toolbar)
            except Exception:
                pass

    # --- internals ---
    def _auto_out_path(self, src):
        folder = os.path.dirname(src)
        if _writable_dir(folder):
            return _unique_sibling_path(src, suffix="_mdal.tin")
        tmp = tempfile.gettempdir()
        base = os.path.splitext(os.path.basename(src))[0]
        return os.path.join(tmp, base + "_mdal.tin")

    def _write_and_load(self, pts, tris, vis, out_tin):
        write_tin(pts, tris, vis, out_tin)
        mesh_layer = QgsMeshLayer(out_tin, os.path.basename(out_tin), "mdal")
        if not mesh_layer.isValid():
            raise RuntimeError("MDAL could not load the .TIN mesh.")
        QgsProject.instance().addMapLayer(mesh_layer)
        return out_tin

    # --- Quick path: defaults visible-only + reindex ---
    def quick_open_12da_as_mesh_auto(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .12da (quick mesh)", "", "12d TIN (*.12da);;All files (*)")
        if not src:
            return
        try:
            data = parse_12da(src)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Parse failed", f"Could not parse .12da:\n{e}")
            return
        if not data["points"] or not data["triangles"]:
            QMessageBox.warning(self.iface.mainWindow(), "No data", "No points/triangles in file.")
            return

        opts = {"visible_only": True, "reindex": True, "dx":0.0, "dy":0.0, "z_add":0.0, "z_scale":1.0}
        pts, tris, vis = apply_modifications(data["points"], data["triangles"], data["visible"], opts)

        out_tin = self._auto_out_path(src)
        try:
            path = self._write_and_load(pts, tris, vis, out_tin)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Load failed", f"{e}")
            return
        QMessageBox.information(self.iface.mainWindow(), "Mesh loaded",
                                f"Created and loaded mesh.\nTIN saved to:\n{path}")

    # --- Options path: opens dialog then writes next to source ---
    def open_12da_as_mesh_with_options(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .12da (options)", "", "12d TIN (*.12da);;All files (*)")
        if not src:
            return
        try:
            data = parse_12da(src)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Parse failed", f"Could not parse .12da:\n{e}")
            return
        if not data["points"] or not data["triangles"]:
            QMessageBox.warning(self.iface.mainWindow(), "No data", "No points/triangles in file.")
            return

        xs = [p[0] for p in data["points"]]; ys = [p[1] for p in data["points"]]
        bounds = (min(xs), max(xs), min(ys), max(ys)) if xs and ys else None
        dlg = ModDialog(self.iface.mainWindow(), bounds=bounds)
        if dlg.exec_() != QDialog.Accepted:
            return
        opts = dlg.options()
        pts, tris, vis = apply_modifications(data["points"], data["triangles"], data["visible"], opts)

        out_tin = self._auto_out_path(src)
        try:
            path = self._write_and_load(pts, tris, vis, out_tin)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Load failed", f"{e}")
            return
        QMessageBox.information(self.iface.mainWindow(), "Mesh loaded",
                                f"Created and loaded mesh.\nTIN saved to:\n{path}")

    # --- Helper: decode underlying file path from a mesh layer ---
    def _get_mesh_source_path(self, mesh_layer: QgsMeshLayer):
        try:
            p = mesh_layer.customProperty("QFAT02.srcFile", "")
            if p and os.path.exists(p):
                return p
        except Exception:
            pass
        uri = ""
        try:
            uri = mesh_layer.dataProvider().dataSourceUri()
        except Exception:
            try:
                uri = mesh_layer.source()
            except Exception:
                uri = ""
        if not uri:
            return ""
        if uri.lower().startswith("mdal:"):
            uri = uri.split(":", 1)[1]
        uri = uri.split("?", 1)[0].split("|", 1)[0]
        if uri.lower().startswith("file://"):
            from urllib.parse import urlparse, unquote
            pth = unquote(urlparse(uri).path)
            if len(pth) > 3 and pth[0] == "/" and pth[2] == ":":
                pth = pth[1:]
            uri = pth
        return uri.strip("\"'")

    # --- SMS TIN path (no sniff): normalize then load ---
    def quick_open_tin_sms(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .tin (SMS)", "", "TIN (*.tin);;All files (*)")
        if not src:
            return
        try:
            out_tin = self._auto_out_path(src)
            self._normalize_sms_tin(src, out_tin)
            mesh_layer = QgsMeshLayer(out_tin, os.path.basename(out_tin), "mdal")
            if not mesh_layer.isValid():
                raise RuntimeError("MDAL could not load the normalized .TIN.")
            try:
                mesh_layer.setCrs(QgsProject.instance().crs())
            except Exception:
                pass
            try:
                mesh_layer.setCustomProperty("QFAT02.srcFile", out_tin)
            except Exception:
                pass
            QgsProject.instance().addMapLayer(mesh_layer)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Load failed", f"{e}")
            return
        QMessageBox.information(self.iface.mainWindow(), "Mesh loaded",
                                f"Normalized and loaded SMS .TIN.\nSaved as:\n{out_tin}")

    def _normalize_sms_tin(self, src, dst):
        import io
        def push(buf, s): buf.write(s + "\n")
        with open(src, "r", encoding="utf-8", errors="replace") as f:
            lines = [ln.rstrip("\n") for ln in f]

        i = 0
        if not lines or not lines[0].strip().startswith("TIN"):
            raise ValueError("Not an ASCII TIN (missing 'TIN').")
        out = io.StringIO()
        push(out, "TIN")
        i += 1

        # seek BEGT
        while i < len(lines) and not lines[i].strip().startswith("BEGT"):
            i += 1
        if i == len(lines):
            raise ValueError("Missing BEGT section.")
        push(out, "BEGT")
        i += 1

        # Skip any TNAM lines (we don't want TNAM)
        while i < len(lines) and lines[i].strip().startswith("TNAM"):
            i += 1

        # Optional MAT line - pass through
        if i < len(lines) and lines[i].strip().startswith("MAT"):
            push(out, lines[i].strip())
            i += 1

        # Expect VERT
        while i < len(lines) and not lines[i].strip().startswith("VERT"):
            i += 1
        if i == len(lines):
            raise ValueError("Missing VERT section.")
        m = re.match(r"\s*VERT\s+(\d+)", lines[i])
        if not m:
            raise ValueError("Malformed VERT header.")
        n_vert = int(m.group(1))
        push(out, f"VERT {n_vert}")
        i += 1

        verts = 0
        while verts < n_vert and i < len(lines):
            raw = lines[i].strip(); i += 1
            if not raw:
                continue
            parts = raw.split()
            if len(parts) < 3:
                raise ValueError(f"Bad vertex line: '{raw}'")
            x, y, z = parts[0:3]
            fourth = parts[3] if len(parts) >= 4 else "0"
            push(out, f" {x} {y} {z} {fourth}")
            verts += 1

        # TRI
        while i < len(lines) and not lines[i].strip().startswith("TRI"):
            i += 1
        if i == len(lines):
            raise ValueError("Missing TRI section.")
        m = re.match(r"\s*TRI\s+(\d+)", lines[i])
        if not m:
            raise ValueError("Malformed TRI header.")
        n_tri = int(m.group(1))
        push(out, f"TRI {n_tri}")
        i += 1

        tris = 0
        while tris < n_tri and i < len(lines):
            raw = lines[i].strip(); i += 1
            if not raw:
                continue
            parts = raw.split()
            if len(parts) < 3:
                raise ValueError(f"Bad triangle line: '{raw}'")
            a, b, c = parts[0:3]
            push(out, f"  {a}   {b}   {c}")
            tris += 1

        push(out, "ENDT")
        with open(dst, "w", encoding="utf-8") as f:
            f.write(out.getvalue())

    # --- Export active mesh (typically a TIN) to 2DM for editing ---
    def save_active_mesh_as_2dm(self):
        lyr = self.iface.activeLayer()
        if not isinstance(lyr, QgsMeshLayer) or not lyr.isValid():
            QMessageBox.warning(self.iface.mainWindow(), "No mesh layer",
                                "Select a mesh layer first.")
            return
        src_uri = lyr.source()
        base = os.path.splitext(os.path.basename(src_uri))[0]
        default = os.path.join(os.path.dirname(src_uri), base + "_editable.2dm")
        out_path, _ = QFileDialog.getSaveFileName(self.iface.mainWindow(), "Save as 2DM", default, "2DM Mesh (*.2dm);;All files (*)")
        if not out_path:
            return
        try:
            verts, tris = self._extract_mesh_vertices_faces(lyr)
        except Exception:
            src_guess = self._get_mesh_source_path(lyr) or src_uri
            src_guess = src_guess.split("?",1)[0].split("|",1)[0]
            low = src_guess.lower()
            if ".tin" in low:
                verts, tris = self._parse_mdal_tin(src_guess)
            elif ".2dm" in low:
                verts, tris = self._parse_2dm(src_guess)
            else:
                QMessageBox.critical(self.iface.mainWindow(), "Export failed",
                                     f"Could not access mesh geometry. Source was:\n{src_uri}\nTip: try on a TIN or 2DM-backed layer.")
                return
        try:
            self._write_2dm(out_path, verts, tris)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Write failed", f"{e}")
            return
        try:
            new_layer = QgsMeshLayer(out_path, os.path.basename(out_path), "mdal")
            if new_layer.isValid():
                QgsProject.instance().addMapLayer(new_layer)
        except Exception:
            pass
        QMessageBox.information(self.iface.mainWindow(), "Saved",
                                f"Wrote editable 2DM and added to project:\n{out_path}")

    def _extract_mesh_vertices_faces(self, mesh_layer: QgsMeshLayer):
        prov = mesh_layer.dataProvider()
        vcount = prov.vertexCount()
        fcount = prov.faceCount()
        verts = []
        for i in range(vcount):
            v = prov.vertex(i)
            try:
                x = v.x(); y = v.y(); z = v.z()
            except Exception:
                x = v.x(); y = v.y(); z = 0.0
            verts.append((x, y, z))
        tris = []
        for i in range(fcount):
            face = prov.face(i)
            try:
                n = face.size()
                if n != 3:
                    continue
                a = face.vertexIndex(0) + 1
                b = face.vertexIndex(1) + 1
                c = face.vertexIndex(2) + 1
            except Exception:
                continue
            tris.append((a, b, c))
        if not verts or not tris:
            raise RuntimeError("No vertices or triangles found in mesh")
        return verts, tris

    def _parse_mdal_tin(self, path):
        verts = []; tris = []
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            lines = [ln.strip() for ln in f if ln.strip()]
        i = 0
        if not lines[i].startswith("TIN"):
            raise ValueError("Not a TIN file")
        while i < len(lines) and not lines[i].startswith("BEGT"): i += 1
        if i == len(lines): raise ValueError("Missing BEGT")
        i += 1
        while i < len(lines) and (lines[i].startswith("TNAM") or lines[i].startswith("MAT")): i += 1
        if i == len(lines) or not lines[i].startswith("VERT"): raise ValueError("Missing VERT")
        n_vert = int(lines[i].split()[1]); i += 1
        for _ in range(n_vert):
            parts = lines[i].split(); i += 1
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            verts.append((x, y, z))
        while i < len(lines) and not lines[i].startswith("TRI"): i += 1
        if i == len(lines): raise ValueError("Missing TRI")
        n_tri = int(lines[i].split()[1]); i += 1
        for _ in range(n_tri):
            a, b, c = [int(v) for v in lines[i].split()[:3]]; i += 1
            tris.append((a, b, c))
        return verts, tris

    def _parse_2dm(self, path):
        verts = []; tris = []
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for ln in f:
                s = ln.strip()
                if not s: continue
                if s.startswith("ND"):
                    parts = s.split()
                    if len(parts) >= 5:
                        _, nid, x, y, z = parts[:5]
                    else:
                        _, nid, x, y = parts[:4]; z = "0"
                    verts.append((float(x), float(y), float(z)))
                elif s.startswith("E3T"):
                    parts = s.split()
                    if len(parts) >= 5:
                        _, eid, a, b, c = parts[:5]
                        tris.append((int(a), int(b), int(c)))
        if not verts or not tris:
            raise ValueError("No vertices or triangles parsed from 2DM")
        return verts, tris

    # --- Direct conversion: .12da → 2DM (editable) ---
    def convert_12da_to_2dm_quick(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .12da (convert to 2DM)", "", "12d TIN (*.12da);;All files (*)")
        if not src:
            return
        try:
            data = parse_12da(src)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Parse failed", f"Could not parse .12da:\n{e}")
            return
        if not data["points"] or not data["triangles"]:
            QMessageBox.warning(self.iface.mainWindow(), "No data", "No points/triangles in file.")
            return
        opts = {"visible_only": True, "reindex": True, "dx":0.0, "dy":0.0, "z_add":0.0, "z_scale":1.0}
        pts, tris, vis = apply_modifications(data["points"], data["triangles"], data["visible"], opts)
        base = os.path.splitext(os.path.basename(src))[0]
        out_2dm = os.path.join(os.path.dirname(src), base + "_editable.2dm")
        try:
            self._write_2dm(out_2dm, pts, tris)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Write failed", f"{e}")
            return
        try:
            lyr = QgsMeshLayer(out_2dm, os.path.basename(out_2dm), "mdal")
            if lyr.isValid():
                QgsProject.instance().addMapLayer(lyr)
        except Exception:
            pass
        QMessageBox.information(self.iface.mainWindow(), "Converted", f"Wrote 2DM and added to project:\n{out_2dm}")

    # --- Direct conversion: .tin (SMS) → 2DM (editable) ---
    def convert_sms_tin_to_2dm_quick(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .tin (SMS) (convert to 2DM)", "", "TIN (*.tin);;All files (*)")
        if not src:
            return
        try:
            verts, tris = self._parse_sms_tin(src)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Parse failed", f"Could not read SMS .tin:\n{e}")
            return
        base = os.path.splitext(os.path.basename(src))[0]
        out_2dm = os.path.join(os.path.dirname(src), base + "_editable.2dm")
        try:
            self._write_2dm(out_2dm, verts, tris)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Write failed", f"{e}")
            return
        try:
            lyr = QgsMeshLayer(out_2dm, os.path.basename(out_2dm), "mdal")
            if lyr.isValid():
                QgsProject.instance().addMapLayer(lyr)
        except Exception:
            pass
        QMessageBox.information(self.iface.mainWindow(), "Converted", f"Wrote 2DM and added to project:\n{out_2dm}")

    def _parse_sms_tin(self, path):
        verts = []; tris = []
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            lines = [ln.rstrip("\n") for ln in f]
        if not lines or not lines[0].strip().startswith("TIN"):
            raise ValueError("Missing TIN header")
        i = 1
        while i < len(lines) and not lines[i].strip().startswith("BEGT"):
            i += 1
        if i == len(lines):
            raise ValueError("Missing BEGT")
        i += 1
        while i < len(lines) and lines[i].strip().startswith("TNAM"):
            i += 1
        if i < len(lines) and lines[i].strip().startswith("MAT"):
            i += 1
        while i < len(lines) and not lines[i].strip().startswith("VERT"):
            i += 1
        if i == len(lines): raise ValueError("Missing VERT")
        m = re.match(r"\s*VERT\s+(\d+)", lines[i])
        if not m: raise ValueError("Bad VERT header")
        n_vert = int(m.group(1)); i += 1
        for _ in range(n_vert):
            if i >= len(lines): raise ValueError("Unexpected EOF in VERT block")
            parts = lines[i].split(); i += 1
            if len(parts) < 3: continue
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            verts.append((x, y, z))
        while i < len(lines) and not lines[i].strip().startswith("TRI"):
            i += 1
        if i == len(lines): raise ValueError("Missing TRI")
        m = re.match(r"\s*TRI\s+(\d+)", lines[i])
        if not m: raise ValueError("Bad TRI header")
        n_tri = int(m.group(1)); i += 1
        for _ in range(n_tri):
            if i >= len(lines): raise ValueError("Unexpected EOF in TRI block")
            parts = lines[i].split(); i += 1
            if len(parts) < 3: continue
            a, b, c = int(parts[0]), int(parts[1]), int(parts[2])
            tris.append((a, b, c))
        if not verts or not tris:
            raise ValueError("No vertices or triangles found")
        return verts, tris

    def _write_2dm(self, out_path, verts, tris):
        with open(out_path, "w", encoding="utf-8") as f:
            f.write("MESH2D\n")
            for i, (x, y, z) in enumerate(verts, start=1):
                f.write(f"ND {i} {x:.10g} {y:.10g} {z:.10g}\n")
            for i, (a, b, c) in enumerate(tris, start=1):
                f.write(f"E3T {i} {a} {b} {c}\n")
