# -*- coding: utf-8 -*-
from qgis.PyQt.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QDoubleSpinBox,
    QGroupBox, QPushButton, QGridLayout, QComboBox
)


class ModDialog(QDialog):
    def __init__(self, parent=None, bounds=None, visible_only_default=True):
        super().__init__(parent)
        self.setWindowTitle("12da -> .TIN (MDAL) - Options")
        lay = QVBoxLayout(self)

        self.chk_visible = QCheckBox("Skip hidden 12d triangles in full TIN")
        self.chk_visible.setChecked(bool(visible_only_default))
        self.chk_construction = QCheckBox("Drop triangles that touch construction vertices (pid 1-4)")
        self.chk_reindex = QCheckBox("Reindex vertices to used-only")
        self.chk_reindex.setChecked(True)

        g_basic = QGroupBox("Basic filters")
        gbl = QVBoxLayout()
        gbl.addWidget(self.chk_visible)
        gbl.addWidget(self.chk_construction)
        gbl.addWidget(self.chk_reindex)
        g_basic.setLayout(gbl)
        lay.addWidget(g_basic)

        g_tf = QGroupBox("Transforms")
        grid = QGridLayout()
        self.dx = QDoubleSpinBox(); self.dx.setDecimals(6); self.dx.setRange(-1e9, 1e9); self.dx.setValue(0.0)
        self.dy = QDoubleSpinBox(); self.dy.setDecimals(6); self.dy.setRange(-1e9, 1e9); self.dy.setValue(0.0)
        self.z_add = QDoubleSpinBox(); self.z_add.setDecimals(6); self.z_add.setRange(-1e9, 1e9); self.z_add.setValue(0.0)
        self.z_scale = QDoubleSpinBox(); self.z_scale.setDecimals(6); self.z_scale.setRange(-1e6, 1e6); self.z_scale.setValue(1.0)
        grid.addWidget(QLabel("dX"), 0, 0); grid.addWidget(self.dx, 0, 1)
        grid.addWidget(QLabel("dY"), 0, 2); grid.addWidget(self.dy, 0, 3)
        grid.addWidget(QLabel("Z add"), 1, 0); grid.addWidget(self.z_add, 1, 1)
        grid.addWidget(QLabel("Z scale"), 1, 2); grid.addWidget(self.z_scale, 1, 3)
        g_tf.setLayout(grid)
        lay.addWidget(g_tf)

        g_area = QGroupBox("Triangle area filter (XY units)")
        h = QHBoxLayout()
        self.min_area = QDoubleSpinBox(); self.min_area.setDecimals(6); self.min_area.setRange(0.0, 1e12); self.min_area.setValue(0.0)
        h.addWidget(QLabel("Min area")); h.addWidget(self.min_area)
        g_area.setLayout(h)
        lay.addWidget(g_area)

        g_bb = QGroupBox("Clip by bounding box (all vertices inside)")
        grid_bb = QGridLayout()
        self.use_bbox = QCheckBox("Enable")
        self.xmin = QDoubleSpinBox(); self.xmin.setDecimals(6); self.xmin.setRange(-1e12, 1e12)
        self.ymin = QDoubleSpinBox(); self.ymin.setDecimals(6); self.ymin.setRange(-1e12, 1e12)
        self.xmax = QDoubleSpinBox(); self.xmax.setDecimals(6); self.xmax.setRange(-1e12, 1e12)
        self.ymax = QDoubleSpinBox(); self.ymax.setDecimals(6); self.ymax.setRange(-1e12, 1e12)
        grid_bb.addWidget(self.use_bbox, 0, 0, 1, 4)
        grid_bb.addWidget(QLabel("Xmin"), 1, 0); grid_bb.addWidget(self.xmin, 1, 1)
        grid_bb.addWidget(QLabel("Ymin"), 1, 2); grid_bb.addWidget(self.ymin, 1, 3)
        grid_bb.addWidget(QLabel("Xmax"), 2, 0); grid_bb.addWidget(self.xmax, 2, 1)
        grid_bb.addWidget(QLabel("Ymax"), 2, 2); grid_bb.addWidget(self.ymax, 2, 3)
        g_bb.setLayout(grid_bb)
        lay.addWidget(g_bb)

        btns = QHBoxLayout()
        self.btn_ok = QPushButton("OK")
        self.btn_cancel = QPushButton("Cancel")
        btns.addStretch(1)
        btns.addWidget(self.btn_ok)
        btns.addWidget(self.btn_cancel)
        lay.addLayout(btns)
        self.btn_ok.clicked.connect(self.accept)
        self.btn_cancel.clicked.connect(self.reject)

        if bounds:
            xmin, xmax, ymin, ymax = bounds
            self.xmin.setValue(xmin)
            self.xmax.setValue(xmax)
            self.ymin.setValue(ymin)
            self.ymax.setValue(ymax)

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



class SettingsDialog(QDialog):
    def __init__(self, parent=None, values=None):
        super().__init__(parent)
        values = values or {}
        self.setWindowTitle("QFAT02 TIN Loader - Settings")
        lay = QVBoxLayout(self)

        info = QLabel("Configure default behaviour for common QFAT02 import workflows.")
        info.setWordWrap(True)
        lay.addWidget(info)

        grid = QGridLayout()
        row = 0

        self.cbo_drop = QComboBox()
        self.cbo_drop.addItem("Ask every time", "ask")
        self.cbo_drop.addItem("Load as mesh / TIN where possible", "load")
        self.cbo_drop.addItem("Convert to 2DM and load", "convert_2dm")
        grid.addWidget(QLabel("Default drag-and-drop action"), row, 0)
        grid.addWidget(self.cbo_drop, row, 1)
        row += 1

        self.chk_visible_default = QCheckBox("Skip hidden 12d triangles in full TIN")
        self.chk_visible_default.setToolTip("For full TIN exports, 12d may include hidden/nulled triangles. When this is on, QFAT02 skips those triangles and loads only the visible TIN.")
        grid.addWidget(QLabel("12d full TIN handling"), row, 0)
        grid.addWidget(self.chk_visible_default, row, 1)
        row += 1

        self.cbo_tin = QComboBox()
        self.cbo_tin.addItem("Auto-detect", "auto")
        self.cbo_tin.addItem("Force SMS/XMS", "force_sms")
        self.cbo_tin.addItem("Force direct MDAL/QGIS load", "force_mdal")
        grid.addWidget(QLabel("TIN handling"), row, 0)
        grid.addWidget(self.cbo_tin, row, 1)
        row += 1

        self.cbo_overwrite = QComboBox()
        self.cbo_overwrite.addItem("Ask before overwrite", "ask")
        self.cbo_overwrite.addItem("Auto-rename", "auto_rename")
        self.cbo_overwrite.addItem("Overwrite", "overwrite")
        grid.addWidget(QLabel("Output overwrite behaviour"), row, 0)
        grid.addWidget(self.cbo_overwrite, row, 1)
        row += 1

        self.chk_summary = QCheckBox("Show import/conversion summary messages")
        grid.addWidget(QLabel("Import summary message"), row, 0)
        grid.addWidget(self.chk_summary, row, 1)
        row += 1


        lay.addLayout(grid)

        enc = QLabel("12d encoding conversion: QFAT02 auto-detects UTF-8, UTF-8 BOM, UTF-16 LE/BE BOM, ANSI/CP1252, and Latin-1 fallback, then converts internally to UTF-8 for parsing. A warning is shown when the source is not UTF-8 or ANSI/CP1252.")
        enc.setWordWrap(True)
        lay.addWidget(enc)

        dz = QLabel("12daz handling: QFAT02 assumes one main .12da inside the archive. If multiple .12da files are found, the largest file is used.")
        dz.setWordWrap(True)
        lay.addWidget(dz)

        about_box = QGroupBox("About")
        about_lay = QVBoxLayout()
        about_title = QLabel("QFAT02 TIN Loader")
        about_title.setStyleSheet("font-weight: bold;")
        about_lay.addWidget(about_title)
        about_text = QLabel(
            "This tool is part of QFAT (QGIS Flood Analysis Toolkit) — a set of focused QGIS utilities designed to support flood modelling workflows (e.g., TUFLOW, HEC-RAS), hydraulic QA, and rapid engineering checks.\n\n"
            "Credits:\n"
            "- Initial idea inspired by Lee Canavan.\n"
            "- LandXML support is independently implemented in QFAT02, inspired by and written with reference to Lutra Consulting\'s open-source LandXML mesh conversion work."
        )
        about_text.setWordWrap(True)
        about_lay.addWidget(about_text)
        about_box.setLayout(about_lay)
        lay.addWidget(about_box)

        btns = QHBoxLayout()
        self.btn_ok = QPushButton("OK")
        self.btn_cancel = QPushButton("Cancel")
        btns.addStretch(1)
        btns.addWidget(self.btn_ok)
        btns.addWidget(self.btn_cancel)
        lay.addLayout(btns)
        self.btn_ok.clicked.connect(self.accept)
        self.btn_cancel.clicked.connect(self.reject)

        self._set_combo(self.cbo_drop, values.get("drag_drop_action", "ask"))
        self.chk_visible_default.setChecked(bool(values.get("visible_only_default", True)))
        self._set_combo(self.cbo_tin, values.get("tin_handling", "auto"))
        self._set_combo(self.cbo_overwrite, values.get("overwrite_behaviour", "ask"))
        self.chk_summary.setChecked(bool(values.get("show_import_summary", True)))

    def _set_combo(self, combo, value):
        idx = combo.findData(value)
        if idx >= 0:
            combo.setCurrentIndex(idx)

    def values(self):
        return {
            "drag_drop_action": self.cbo_drop.currentData(),
            "visible_only_default": self.chk_visible_default.isChecked(),
            "tin_handling": self.cbo_tin.currentData(),
            "overwrite_behaviour": self.cbo_overwrite.currentData(),
            "show_import_summary": self.chk_summary.isChecked(),
        }
