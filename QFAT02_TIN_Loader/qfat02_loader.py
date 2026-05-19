# -*- coding: utf-8 -*-
# QFAT02 TIN Loader v2.2.27 - plugin wiring only; parsers are split into modules.
import os
import tempfile
from urllib.parse import urlparse, unquote

from qgis.PyQt.QtWidgets import QAction, QFileDialog, QMessageBox, QDialog, QMenu, QToolButton
from qgis.PyQt.QtGui import QIcon, QDesktopServices
from qgis.PyQt.QtCore import QObject, QEvent, Qt, QUrl
from qgis.core import QgsProject, QgsMeshLayer, QgsVectorLayer, QgsSettings, QgsProcessingUtils

from .dialogs import ModDialog, SettingsDialog
from .io_12da import parse_12da, write_tin, apply_modifications
from .io_sms_tin import normalize_sms_tin, parse_sms_tin, detect_tin_format, parse_tin_auto
from .io_2dm import write_2dm, parse_2dm, parse_mdal_tin
from .io_dxf_3dface import parse_dxf_3dface, has_dxf_3dface_entity
from .io_landxml import parse_landxml_surface
from .utils import writable_dir, unique_path


class Qfat02LoaderPlugin(QObject):
    def __init__(self, iface):
        super().__init__(iface.mainWindow())
        self.iface = iface
        self.actions = []
        self.menu = "&QFAT02 TIN Loader"
        self.toolbar = self.iface.addToolBar("QFAT02 TIN Loader")
        self.toolbar.setObjectName("QFAT02_TIN_Loader_toolbar")
        self._toolbar_menu = None
        self._toolbar_button = None
        self._plugin_menus = []
        self._plugin_menu_actions = []
        self._settings_group_menus = []
        self._old_accept_drops = {}
        self._drop_targets = []
        self.settings = QgsSettings()

    def initGui(self):
        icon = QIcon(os.path.join(os.path.dirname(__file__), "icons", "qfat02.svg"))
        self._add_action(self._icon("act_12d_mesh.svg"), "Open .12da/.12daz as Mesh (auto .TIN)...", self.quick_open_12da_as_mesh_auto)
        self._add_action(self._icon("act_12d_opt.svg"), "Open .12da/.12daz as Mesh (options)...", self.open_12da_as_mesh_with_options)
        self._add_action(self._icon("act_tin_mesh.svg"), "Open .tin (auto-detect)...", self.quick_open_tin_auto)
        self._add_action(self._icon("act_dxf_mesh.svg"), "Open .dxf (3DFACE) as Mesh/TIN...", self.open_dxf_3dface_as_tin_quick)
        self._add_action(self._icon("act_mesh_save.svg"), "Save current mesh as 2DM (editable)...", self.save_active_mesh_as_2dm)
        self._add_action(self._icon("act_12d_2dm.svg"), "Convert .12da/.12daz -> 2DM (editable)...", self.convert_12da_to_2dm_quick)
        self._add_action(self._icon("act_tin_2dm.svg"), "Convert .tin (auto-detect) -> 2DM (editable)...", self.convert_tin_to_2dm_quick)
        self._add_action(self._icon("act_dxf_2dm.svg"), "Open .dxf (3DFACE) -> 2DM (editable)...", self.convert_dxf_3dface_to_2dm_quick)
        self._add_action(self._icon("act_dxf_qgis.svg"), "Open .dxf normally in QGIS...", self.open_dxf_normally_quick)
        self._add_action(self._icon("act_xml_2dm.svg"), "Open LandXML surface -> 2DM (editable)...", self.convert_landxml_to_2dm_quick)
        self._add_action(self._icon("act_set_set.svg"), "Settings...", self.open_settings_dialog)
        self._build_plugin_menu_groups()
        self._build_toolbar_dropdown(icon)
        self._install_drop_filter()

    def _key(self, name):
        return "QFAT02_TIN_Loader/" + name

    def _setting(self, name, default):
        try:
            val = self.settings.value(self._key(name), default)
            if val is None:
                return default
            return val
        except Exception:
            return default

    def _bool_setting(self, name, default=True):
        val = self._setting(name, default)
        if isinstance(val, bool):
            return val
        return str(val).strip().lower() in ("1", "true", "yes", "on")

    def _all_settings(self):
        return {
            "drag_drop_action": str(self._setting("drag_drop_action", "ask")),
            "visible_only_default": self._bool_setting("visible_only_default", True),
            "tin_handling": str(self._setting("tin_handling", "auto")),
            "overwrite_behaviour": str(self._setting("overwrite_behaviour", "ask")),
            "show_import_summary": self._bool_setting("show_import_summary", True),
        }

    def _save_settings(self, values):
        for key, value in values.items():
            self.settings.setValue(self._key(key), value)

    def open_settings_dialog(self):
        dlg = SettingsDialog(self.iface.mainWindow(), self._all_settings())
        if dlg.exec_() != QDialog.Accepted:
            return
        self._save_settings(dlg.values())
        QMessageBox.information(self.iface.mainWindow(), "Settings saved", "QFAT02 settings have been saved.")

    def _show_summary(self):
        return self._bool_setting("show_import_summary", True)

    def _visible_only_default(self):
        return self._bool_setting("visible_only_default", True)

    def _tin_handling(self):
        mode = str(self._setting("tin_handling", "auto"))
        if mode not in ("auto", "force_sms", "force_mdal"):
            mode = "auto"
        return mode

    def _overwrite_behaviour(self):
        mode = str(self._setting("overwrite_behaviour", "ask"))
        if mode not in ("ask", "auto_rename", "overwrite"):
            mode = "ask"
        return mode

    def _resolve_existing_path(self, path, label="output"):
        if not os.path.exists(path):
            return path
        mode = self._overwrite_behaviour()
        if mode == "overwrite":
            return path
        if mode == "auto_rename":
            return unique_path(path)
        box = QMessageBox(self.iface.mainWindow())
        box.setWindowTitle("Output exists")
        box.setText("The {} already exists:\n{}".format(label, path))
        box.setInformativeText("Overwrite it? Choose No to auto-rename instead.")
        box.setStandardButtons(QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
        box.setDefaultButton(QMessageBox.No)
        res = box.exec_()
        if res == QMessageBox.Yes:
            return path
        if res == QMessageBox.No:
            return unique_path(path)
        raise ValueError("Cancelled by user")

    def _base_2dm_path(self, src):
        base = os.path.splitext(os.path.basename(src))[0]
        return os.path.join(os.path.dirname(src), base + "_editable.2dm")

    def _output_2dm_path(self, src):
        return self._resolve_existing_path(self._base_2dm_path(src), "2DM output")

    def _add_action(self, icon, text, callback):
        action = QAction(icon, text, self.iface.mainWindow())
        action.triggered.connect(callback)
        self.actions.append(action)
        return action

    def _icon(self, filename):
        return QIcon(os.path.join(os.path.dirname(__file__), "icons", filename))

    def _find_action(self, text):
        for action in self.actions:
            if action.text() == text:
                return action
        return None

    def _add_group_menu(self, parent_menu, title, icon_file, action_texts):
        group_menu = QMenu(title, parent_menu)
        group_menu.setIcon(self._icon(icon_file))
        for text in action_texts:
            action = self._find_action(text)
            if action is not None:
                group_menu.addAction(action)
        parent_menu.addMenu(group_menu)
        if title == "Settings":
            group_menu.installEventFilter(self)
            self._settings_group_menus.append(group_menu)
        return group_menu

    def _open_plugin_folder(self):
        folder = os.path.dirname(__file__)
        QDesktopServices.openUrl(QUrl.fromLocalFile(folder))

    def _handle_settings_secret_click(self, obj, event):
        if event.type() != QEvent.MouseButtonDblClick:
            return False
        try:
            if event.button() != Qt.LeftButton:
                return False
            if not (event.modifiers() & Qt.ShiftModifier):
                return False
        except Exception:
            return False

        # Secret shortcut: Shift + left double-click on the Settings group
        # in the toolbar/menu opens the installed plugin folder.
        if isinstance(obj, QMenu):
            action = obj.actionAt(event.pos())
            if action is None or action.text().startswith("Settings"):
                self._open_plugin_folder()
                return True

        return False


    def _menu_groups(self):
        # Single source of truth: plugin menu and toolbar dropdown must stay identical.
        # Group by source file type rather than by action type.
        return [
            ("12DA / 12DAZ", "group_12d.svg", [
                "Open .12da/.12daz as Mesh (auto .TIN)...",
                "Open .12da/.12daz as Mesh (options)...",
                "Convert .12da/.12daz -> 2DM (editable)...",
            ]),
            ("TIN", "group_tin_file.svg", [
                "Open .tin (auto-detect)...",
                "Convert .tin (auto-detect) -> 2DM (editable)...",
            ]),
            ("DXF", "group_dxf.svg", [
                "Open .dxf (3DFACE) as Mesh/TIN...",
                "Open .dxf (3DFACE) -> 2DM (editable)...",
                "Open .dxf normally in QGIS...",
            ]),
            ("LandXML", "group_landxml.svg", [
                "Open LandXML surface -> 2DM (editable)...",
            ]),
            ("Current Mesh", "group_mesh.svg", [
                "Save current mesh as 2DM (editable)...",
            ]),
            ("Settings", "group_set.svg", [
                "Settings...",
            ]),
        ]

    def _build_plugin_menu_groups(self):
        # Main plugin menu mirrors the toolbar dropdown exactly.
        for title, icon_file, action_texts in self._menu_groups():
            group_menu = QMenu(title, self.iface.mainWindow())
            group_menu.setIcon(self._icon(icon_file))
            for text in action_texts:
                action = self._find_action(text)
                if action is not None:
                    group_menu.addAction(action)
            if title == "Settings":
                group_menu.installEventFilter(self)
                self._settings_group_menus.append(group_menu)
            self._plugin_menus.append(group_menu)
            menu_action = group_menu.menuAction()
            self.iface.addPluginToMenu(self.menu, menu_action)
            self._plugin_menu_actions.append(menu_action)

    def _build_toolbar_dropdown(self, icon):
        self._toolbar_menu = QMenu(self.iface.mainWindow())
        self._toolbar_menu.setTitle("QFAT02 TIN Loader")
        self._toolbar_menu.installEventFilter(self)

        for title, icon_file, action_texts in self._menu_groups():
            self._add_group_menu(self._toolbar_menu, title, icon_file, action_texts)

        self._toolbar_button = QToolButton(self.iface.mainWindow())
        self._toolbar_button.setIcon(icon)
        self._toolbar_button.setText("QFAT02")
        self._toolbar_button.setToolTip("QFAT02 TIN Loader actions")
        self._toolbar_button.setPopupMode(QToolButton.InstantPopup)
        self._toolbar_button.setMenu(self._toolbar_menu)
        self.toolbar.addWidget(self._toolbar_button)

    def _install_drop_filter(self):
        # Accept drops on both the map canvas and the layer panel / TOC.
        # Keep mainWindow as a fallback for older QGIS builds.
        targets = []
        for getter in (
            lambda: self.iface.mapCanvas(),
            lambda: self.iface.layerTreeView(),
            lambda: self.iface.mainWindow(),
        ):
            try:
                obj = getter()
                if obj is not None and obj not in targets:
                    targets.append(obj)
                # Some Qt views receive drag/drop events on their viewport, not the view itself.
                if obj is not None and hasattr(obj, "viewport"):
                    vp = obj.viewport()
                    if vp is not None and vp not in targets:
                        targets.append(vp)
            except Exception:
                pass
        self._drop_targets = []
        self._old_accept_drops = {}
        for obj in targets:
            try:
                self._old_accept_drops[obj] = obj.acceptDrops()
                obj.setAcceptDrops(True)
                obj.installEventFilter(self)
                self._drop_targets.append(obj)
            except Exception:
                pass

    def unload(self):
        try:
            for obj in getattr(self, "_drop_targets", []):
                try:
                    obj.removeEventFilter(self)
                    if obj in getattr(self, "_old_accept_drops", {}):
                        obj.setAcceptDrops(self._old_accept_drops[obj])
                except Exception:
                    pass
            for menu_action in getattr(self, "_plugin_menu_actions", []):
                try:
                    self.iface.removePluginMenu(self.menu, menu_action)
                except Exception:
                    pass
            for action in self.actions:
                try:
                    self.iface.removePluginMenu(self.menu, action)
                except Exception:
                    pass
                try:
                    self.toolbar.removeAction(action)
                except Exception:
                    pass
        finally:
            try:
                self.iface.mainWindow().removeToolBar(self.toolbar)
            except Exception:
                pass

    def _qgis_temp_folder(self):
        try:
            folder = QgsProcessingUtils.tempFolder()
            if folder:
                os.makedirs(folder, exist_ok=True)
                return folder
        except Exception:
            pass
        folder = os.path.join(tempfile.gettempdir(), "QFAT02_TIN_Loader")
        try:
            os.makedirs(folder, exist_ok=True)
        except Exception:
            folder = tempfile.gettempdir()
        return folder

    def _temp_output_path(self, requested_path):
        folder = self._qgis_temp_folder()
        base = os.path.basename(requested_path)
        candidate = os.path.join(folder, base)
        return unique_path(candidate)

    def _offer_temp_output_path(self, requested_path, label, error):
        box = QMessageBox(self.iface.mainWindow())
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle("Cannot write output")
        box.setText("QFAT02 could not create the {}.".format(label))
        box.setInformativeText("Requested path:\n{}\n\nReason:\n{}\n\nYou can save this output to the QGIS temporary folder instead.".format(requested_path, error))
        btn_tmp = box.addButton("Save to QGIS temp folder", QMessageBox.AcceptRole)
        box.addButton(QMessageBox.Cancel)
        box.setDefaultButton(btn_tmp)
        box.exec_()
        if box.clickedButton() == btn_tmp:
            return self._temp_output_path(requested_path)
        raise ValueError("Cancelled by user")

    def _ensure_parent_writable_or_temp(self, requested_path, label):
        folder = os.path.dirname(requested_path) or os.getcwd()
        if os.path.isdir(folder) and writable_dir(folder):
            return requested_path
        reason = "Directory is not writable or does not exist: {}".format(folder)
        return self._offer_temp_output_path(requested_path, label, reason)

    def _write_tin_safe(self, pts, tris, vis, requested_path):
        out_path = self._ensure_parent_writable_or_temp(requested_path, "TIN output")
        try:
            write_tin(pts, tris, vis, out_path)
            return out_path
        except Exception as e:
            fallback = self._offer_temp_output_path(out_path, "TIN output", e)
            write_tin(pts, tris, vis, fallback)
            return fallback

    def _normalize_sms_tin_safe(self, src, requested_path):
        out_path = self._ensure_parent_writable_or_temp(requested_path, "normalised TIN output")
        try:
            normalize_sms_tin(src, out_path)
            return out_path
        except Exception as e:
            fallback = self._offer_temp_output_path(out_path, "normalised TIN output", e)
            normalize_sms_tin(src, fallback)
            return fallback

    def _write_2dm_safe(self, requested_path, verts, faces):
        out_path = self._ensure_parent_writable_or_temp(requested_path, "2DM output")
        try:
            write_2dm(out_path, verts, faces)
            return out_path
        except Exception as e:
            fallback = self._offer_temp_output_path(out_path, "2DM output", e)
            write_2dm(fallback, verts, faces)
            return fallback

    def _auto_out_path(self, src):
        folder = os.path.dirname(src)
        base = os.path.splitext(os.path.basename(src))[0]
        candidate = os.path.join(folder, base + "_mdal.tin")
        candidate = self._resolve_existing_path(candidate, "normalised TIN output")
        return candidate

    def _write_and_load_tin(self, pts, tris, vis, out_tin):
        out_tin = self._write_tin_safe(pts, tris, vis, out_tin)
        mesh_layer = QgsMeshLayer(out_tin, os.path.basename(out_tin), "mdal")
        if not mesh_layer.isValid():
            raise RuntimeError("MDAL could not load the .TIN mesh.")
        QgsProject.instance().addMapLayer(mesh_layer)
        return out_tin

    def _add_2dm_layer(self, out_2dm):
        lyr = QgsMeshLayer(out_2dm, os.path.basename(out_2dm), "mdal")
        if lyr.isValid():
            try:
                lyr.setCrs(QgsProject.instance().crs())
            except Exception:
                pass
            QgsProject.instance().addMapLayer(lyr)
        return lyr

    def quick_open_12da_as_mesh_auto(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .12da/.12daz (quick mesh)", "", "12d TIN (*.12da *.12daz);;All files (*)")
        if src:
            self._open_12da_as_tin(src, show_message=True)

    def open_12da_as_mesh_with_options(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .12da/.12daz (options)", "", "12d TIN (*.12da *.12daz);;All files (*)")
        if not src:
            return
        try:
            data = parse_12da(src)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Parse failed", f"Could not parse .12da/.12daz:\n{e}")
            return
        if not data["points"] or not data["triangles"]:
            QMessageBox.warning(self.iface.mainWindow(), "No data", "No points/triangles in file.")
            return
        self._show_encoding_warning_if_needed(data)
        xs = [p[0] for p in data["points"]]
        ys = [p[1] for p in data["points"]]
        bounds = (min(xs), max(xs), min(ys), max(ys)) if xs and ys else None
        dlg = ModDialog(self.iface.mainWindow(), bounds=bounds, visible_only_default=self._visible_only_default())
        if dlg.exec_() != QDialog.Accepted:
            return
        pts, tris, vis = apply_modifications(data["points"], data["triangles"], data["visible"], dlg.options())
        out_tin = self._auto_out_path(src)
        try:
            path = self._write_and_load_tin(pts, tris, vis, out_tin)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Load failed", f"{e}")
            return
        if self._show_summary():
            QMessageBox.information(self.iface.mainWindow(), "Mesh loaded", "Created and loaded mesh.\nTIN saved to:\n{}{}".format(path, self._encoding_note(data)))

    def _open_12da_as_tin(self, src, show_message=False):
        data = parse_12da(src)
        if not data["points"] or not data["triangles"]:
            raise ValueError("No points/triangles in file")
        self._show_encoding_warning_if_needed(data)
        opts = {"visible_only": self._visible_only_default(), "reindex": True, "dx": 0.0, "dy": 0.0, "z_add": 0.0, "z_scale": 1.0}
        pts, tris, vis = apply_modifications(data["points"], data["triangles"], data["visible"], opts)
        out_tin = self._auto_out_path(src)
        path = self._write_and_load_tin(pts, tris, vis, out_tin)
        if show_message and self._show_summary():
            QMessageBox.information(self.iface.mainWindow(), "Mesh loaded", "Created and loaded mesh.\nTIN saved to:\n{}{}".format(path, self._encoding_note(data)))
        return path

    def quick_open_tin_auto(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .tin (auto-detect)", "", "TIN (*.tin);;All files (*)")
        if not src:
            return
        try:
            loaded_path, fmt = self._open_tin_auto(src)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Load failed", f"{e}")
            return
        label = self._tin_format_label(fmt)
        if self._show_summary():
            QMessageBox.information(self.iface.mainWindow(), "Mesh loaded", "Loaded .TIN ({})\nSource/working file:\n{}".format(label, loaded_path))

    def _tin_format_label(self, fmt):
        labels = {
            "sms_xms": "SMS/XMS style, normalised",
            "mdal_tin": "MDAL/QGIS-readable style",
            "ascii_tin": "ASCII TIN, normalised",
            "unknown": "unknown"
        }
        return labels.get(fmt, fmt)

    def _load_mdal_tin_layer(self, tin_path, source_path=None):
        mesh_layer = QgsMeshLayer(tin_path, os.path.basename(tin_path), "mdal")
        if not mesh_layer.isValid():
            raise RuntimeError("MDAL could not load the .TIN mesh.")
        try:
            mesh_layer.setCrs(QgsProject.instance().crs())
            mesh_layer.setCustomProperty("QFAT02.srcFile", source_path or tin_path)
        except Exception:
            pass
        QgsProject.instance().addMapLayer(mesh_layer)
        return tin_path

    def _open_tin_auto(self, src):
        mode = self._tin_handling()
        if mode == "force_mdal":
            return self._load_mdal_tin_layer(src, source_path=src), "mdal_tin"
        if mode == "force_sms":
            out_tin = self._auto_out_path(src)
            out_tin = self._normalize_sms_tin_safe(src, out_tin)
            self._load_mdal_tin_layer(out_tin, source_path=out_tin)
            return out_tin, "sms_xms"

        fmt = detect_tin_format(src)
        if fmt == "unknown":
            raise ValueError("Unsupported or unrecognised .tin format. Expected SMS/XMS or MDAL/QGIS-readable ASCII TIN.")

        if fmt == "mdal_tin":
            try:
                return self._load_mdal_tin_layer(src, source_path=src), fmt
            except Exception:
                # Some files look MDAL-ready but still need normalising. Fall through.
                pass

        out_tin = self._auto_out_path(src)
        out_tin = self._normalize_sms_tin_safe(src, out_tin)
        self._load_mdal_tin_layer(out_tin, source_path=out_tin)
        return out_tin, fmt

    def _prepare_2dm_mesh(self, verts, faces):
        if not faces:
            raise ValueError("No valid mesh elements to write")
        return verts, faces, {}

    def _clean_note(self, stats):
        return ""

    def _show_encoding_warning_if_needed(self, data):
        if not isinstance(data, dict) or not data.get("encoding_warn", False):
            return
        enc = data.get("encoding", "unknown")
        member = data.get("member_name", "")
        msg = "This 12da file appears to be encoded as {}.\n\nQFAT02 converted it internally to UTF-8 for parsing.\n\nFor best compatibility with other tools, consider exporting/saving 12da as ANSI/CP1252 or UTF-8.".format(enc)
        if member and member != os.path.basename(member):
            msg += "\n\n12daz internal file:\n{}".format(member)
        QMessageBox.warning(self.iface.mainWindow(), "12d Encoding Warning", msg)

    def _encoding_note(self, data):
        enc = data.get("encoding", "unknown") if isinstance(data, dict) else "unknown"
        member = data.get("member_name", "") if isinstance(data, dict) else ""
        note = "\n12d encoding detected: {}. Converted internally to UTF-8 for parsing.".format(enc)
        if member and member != os.path.basename(member):
            note += "\n12daz internal file: {}".format(member)
        if data.get("encoding_warn", False):
            note += "\nWarning: this 12da is not ANSI/CP1252 or UTF-8. For best compatibility with other tools, consider exporting/saving 12da as ANSI or UTF-8."
        return note


    def save_active_mesh_as_2dm(self):
        lyr = self.iface.activeLayer()
        if not isinstance(lyr, QgsMeshLayer) or not lyr.isValid():
            QMessageBox.warning(self.iface.mainWindow(), "No mesh layer", "Select a mesh layer first.")
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
            try:
                verts, tris = self._fallback_parse_mesh_source(lyr, src_uri)
            except Exception:
                QMessageBox.critical(self.iface.mainWindow(), "Export failed", f"Could not access mesh geometry. Source was:\n{src_uri}\nTip: try on a TIN or 2DM-backed layer.")
                return
        try:
            verts, tris, clean_stats = self._prepare_2dm_mesh(verts, tris)
            out_path = self._write_2dm_safe(out_path, verts, tris)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Write failed", f"{e}")
            return
        self._add_2dm_layer(out_path)
        if self._show_summary():
            QMessageBox.information(self.iface.mainWindow(), "Saved", "Wrote editable 2DM and added to project:\n{}{}".format(out_path, self._clean_note(clean_stats)))

    def _fallback_parse_mesh_source(self, lyr, src_uri):
        src_guess = self._get_mesh_source_path(lyr) or src_uri
        src_guess = src_guess.split("?", 1)[0].split("|", 1)[0]
        low = src_guess.lower()
        if ".tin" in low:
            return parse_mdal_tin(src_guess)
        if ".2dm" in low:
            return parse_2dm(src_guess)
        raise ValueError("Unsupported mesh source")

    def _extract_mesh_vertices_faces(self, mesh_layer):
        prov = mesh_layer.dataProvider()
        verts = []
        for i in range(prov.vertexCount()):
            v = prov.vertex(i)
            try:
                verts.append((v.x(), v.y(), v.z()))
            except Exception:
                verts.append((v.x(), v.y(), 0.0))
        tris = []
        for i in range(prov.faceCount()):
            face = prov.face(i)
            try:
                if face.size() != 3:
                    continue
                tris.append((face.vertexIndex(0) + 1, face.vertexIndex(1) + 1, face.vertexIndex(2) + 1))
            except Exception:
                continue
        if not verts or not tris:
            raise RuntimeError("No vertices or triangles found in mesh")
        return verts, tris

    def _get_mesh_source_path(self, mesh_layer):
        try:
            p = mesh_layer.customProperty("QFAT02.srcFile", "")
            if p and os.path.exists(p):
                return p
        except Exception:
            pass
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
            pth = unquote(urlparse(uri).path)
            if len(pth) > 3 and pth[0] == "/" and pth[2] == ":":
                pth = pth[1:]
            uri = pth
        return uri.strip("\"'")

    def convert_12da_to_2dm_quick(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .12da/.12daz (convert to 2DM)", "", "12d TIN (*.12da *.12daz);;All files (*)")
        if not src:
            return
        try:
            out_2dm, vcount, tcount, clean_stats, data = self._convert_12da_to_2dm_path(src)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Conversion failed", str(e))
            return
        if self._show_summary():
            QMessageBox.information(self.iface.mainWindow(), "Converted", "Wrote 2DM and added to project:\n{}\n\nVertices: {}\nTriangles: {}{}{}".format(out_2dm, vcount, tcount, self._clean_note(clean_stats), self._encoding_note(data)))

    def _convert_12da_to_2dm_path(self, src):
        data = parse_12da(src)
        if not data["points"] or not data["triangles"]:
            raise ValueError("No points/triangles in file")
        self._show_encoding_warning_if_needed(data)
        opts = {"visible_only": self._visible_only_default(), "reindex": True, "dx": 0.0, "dy": 0.0, "z_add": 0.0, "z_scale": 1.0}
        pts, tris, vis = apply_modifications(data["points"], data["triangles"], data["visible"], opts)
        pts, tris, clean_stats = self._prepare_2dm_mesh(pts, tris)
        out_2dm = self._output_2dm_path(src)
        out_2dm = self._write_2dm_safe(out_2dm, pts, tris)
        self._add_2dm_layer(out_2dm)
        return out_2dm, len(pts), len(tris), clean_stats, data

    def convert_tin_to_2dm_quick(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .tin (auto-detect, convert to 2DM)", "", "TIN (*.tin);;All files (*)")
        if not src:
            return
        try:
            out_2dm, vcount, tcount, fmt, clean_stats = self._convert_tin_to_2dm_path(src)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Parse failed", f"Could not read .tin:\n{e}")
            return
        if self._show_summary():
            QMessageBox.information(self.iface.mainWindow(), "Converted", "Wrote 2DM and added to project:\n{}\n\nDetected: {}\nVertices: {}\nTriangles: {}{}".format(out_2dm, self._tin_format_label(fmt), vcount, tcount, self._clean_note(clean_stats)))

    def _convert_tin_to_2dm_path(self, src):
        mode = self._tin_handling()
        if mode == "force_sms":
            fmt = "sms_xms"
            verts, tris = parse_sms_tin(src)
        elif mode == "force_mdal":
            fmt = "mdal_tin"
            verts, tris = parse_mdal_tin(src)
        else:
            fmt = detect_tin_format(src)
            verts, tris = parse_tin_auto(src)
        verts, tris, clean_stats = self._prepare_2dm_mesh(verts, tris)
        out_2dm = self._output_2dm_path(src)
        out_2dm = self._write_2dm_safe(out_2dm, verts, tris)
        self._add_2dm_layer(out_2dm)
        return out_2dm, len(verts), len(tris), fmt, clean_stats

    def convert_dxf_3dface_to_2dm_quick(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .dxf (3DFACE) (convert to 2DM)", "", "DXF (*.dxf);;All files (*)")
        if not src:
            return
        try:
            self._convert_dxf_3dface_path(src, show_message=True)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "DXF conversion failed", str(e))

    def _convert_dxf_3dface_path(self, src, show_message=True):
        verts, tris = parse_dxf_3dface(src)
        if not verts or not tris:
            raise ValueError("No DXF 3DFACE geometry found. This importer supports ASCII DXF 3DFACE entities only.")
        verts, tris, clean_stats = self._prepare_2dm_mesh(verts, tris)
        out_2dm = self._output_2dm_path(src)
        out_2dm = self._write_2dm_safe(out_2dm, verts, tris)
        self._add_2dm_layer(out_2dm)
        if show_message and self._show_summary():
            QMessageBox.information(self.iface.mainWindow(), "Converted", "Wrote 2DM and added to project:\n{}\n\nVertices: {}\nTriangles: {}{}".format(out_2dm, len(verts), len(tris), self._clean_note(clean_stats)))
        return out_2dm

    def open_dxf_3dface_as_tin_quick(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .dxf (3DFACE) as Mesh/TIN", "", "DXF (*.dxf);;All files (*)")
        if not src:
            return
        try:
            self._open_dxf_3dface_as_tin_path(src, show_message=True)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "DXF mesh load failed", str(e))

    def _open_dxf_3dface_as_tin_path(self, src, show_message=True):
        verts, tris = parse_dxf_3dface(src)
        if not verts or not tris:
            raise ValueError("No DXF 3DFACE geometry found. This importer supports ASCII DXF 3DFACE entities only.")
        verts, tris, clean_stats = self._prepare_2dm_mesh(verts, tris)
        out_tin = self._auto_out_path(src)
        path = self._write_and_load_tin(verts, tris, [True] * len(tris), out_tin)
        if show_message and self._show_summary():
            QMessageBox.information(self.iface.mainWindow(), "Mesh loaded", "Created and loaded mesh from DXF 3DFACE.\nTIN saved to:\n{}\n\nVertices: {}\nTriangles: {}{}".format(path, len(verts), len(tris), self._clean_note(clean_stats)))
        return path

    def open_dxf_normally_quick(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open .dxf normally in QGIS", "", "DXF (*.dxf);;All files (*)")
        if not src:
            return
        try:
            self._open_path_normally_in_qgis(src)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "DXF open failed", str(e))

    def convert_landxml_to_2dm_quick(self):
        src, _ = QFileDialog.getOpenFileName(self.iface.mainWindow(), "Open LandXML surface (convert to 2DM)", "", "LandXML (*.xml *.landxml);;All files (*)")
        if not src:
            return
        try:
            self._convert_landxml_path(src, show_message=True)
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "LandXML conversion failed", str(e))

    def _convert_landxml_path(self, src, show_message=True):
        verts, tris = parse_landxml_surface(src)
        if not verts or not tris:
            raise ValueError("No recognised LandXML TIN surface found. Expected Surface/Definition with Pnts and Faces.")
        verts, tris, clean_stats = self._prepare_2dm_mesh(verts, tris)
        out_2dm = self._output_2dm_path(src)
        out_2dm = self._write_2dm_safe(out_2dm, verts, tris)
        self._add_2dm_layer(out_2dm)
        if show_message and self._show_summary():
            QMessageBox.information(self.iface.mainWindow(), "Converted", "Wrote 2DM and added to project:\n{}\n\nVertices: {}\nFaces: {}{}".format(out_2dm, len(verts), len(tris), self._clean_note(clean_stats)))
        return out_2dm

    def eventFilter(self, obj, event):
        if self._handle_settings_secret_click(obj, event):
            return True
        try:
            if obj in getattr(self, "_drop_targets", []) and event.type() in (QEvent.DragEnter, QEvent.DragMove, QEvent.Drop):
                md = event.mimeData()
                if not md or not md.hasUrls():
                    return False
                paths = []
                for url in md.urls():
                    p = url.toLocalFile()
                    if p and self._is_supported_drop_path(p):
                        paths.append(p)
                if not paths:
                    return False
                if event.type() in (QEvent.DragEnter, QEvent.DragMove):
                    event.acceptProposedAction()
                    return True

                drop_action = self._get_drop_action_for_paths(paths)
                if not drop_action:
                    event.acceptProposedAction()
                    return True
                loaded = 0
                errors = []
                for path in paths:
                    try:
                        if drop_action == "open_qgis":
                            self._open_path_normally_in_qgis(path)
                        else:
                            self._open_supported_path(path, from_drop=True, drop_action=drop_action)
                        loaded += 1
                    except Exception as e:
                        errors.append(os.path.basename(path) + ": " + str(e))
                event.acceptProposedAction()
                if errors:
                    QMessageBox.warning(self.iface.mainWindow(), "Drop finished with warnings", "Loaded: {}\n\n{}".format(loaded, "\n".join(errors[:8])))
                elif loaded and self._show_summary():
                    QMessageBox.information(self.iface.mainWindow(), "Drop finished", "Loaded {} file(s).".format(loaded))
                return True
        except Exception:
            return False
        return super().eventFilter(obj, event)

    def _is_supported_drop_path(self, path):
        if not os.path.isfile(path):
            return False
        ext = os.path.splitext(path)[1].lower()
        if ext == ".dxf":
            # Only intercept DXF drag/drop when it contains 3DFACE terrain geometry.
            # Normal CAD DXF files are left for QGIS native DXF loading.
            return has_dxf_3dface_entity(path)
        return ext in (".12da", ".12daz", ".tin", ".xml", ".landxml")

    def _get_drop_action_for_paths(self, paths):
        has_dxf = any(os.path.splitext(p)[1].lower() == ".dxf" for p in paths)
        # DXF 3DFACE is ambiguous: it can be terrain or normal CAD.
        # Always ask for DXF, regardless of the global drag/drop default.
        box = QMessageBox(self.iface.mainWindow())
        box.setWindowTitle("QFAT02 drag and drop")
        box.setText("Choose how to handle the dropped file(s).")
        if has_dxf:
            box.setInformativeText("QFAT02 can load 3DFACE terrain DXF as a mesh/TIN, convert it to 2DM, or open the DXF normally in QGIS.")
            btn_tin = box.addButton("Open as TIN / Mesh", QMessageBox.AcceptRole)
            btn_convert = box.addButton("Open as 2DM", QMessageBox.ActionRole)
            btn_native = box.addButton("Open normally in QGIS", QMessageBox.ActionRole)
            box.addButton(QMessageBox.Cancel)
            box.exec_()
            clicked = box.clickedButton()
            if clicked == btn_tin:
                return "load"
            if clicked == btn_convert:
                return "convert_2dm"
            if clicked == btn_native:
                return "open_qgis"
            return None

        mode = str(self._setting("drag_drop_action", "ask"))
        if mode in ("load", "convert_2dm"):
            return mode

        box.setInformativeText("Load as mesh/TIN is faster. Convert to 2DM is better for editing.")
        btn_load = box.addButton("Load as mesh / TIN", QMessageBox.AcceptRole)
        btn_convert = box.addButton("Convert to 2DM", QMessageBox.ActionRole)
        box.addButton(QMessageBox.Cancel)
        box.exec_()
        clicked = box.clickedButton()
        if clicked == btn_load:
            return "load"
        if clicked == btn_convert:
            return "convert_2dm"
        return None


    def _open_path_normally_in_qgis(self, src):
        ext = os.path.splitext(src)[1].lower()
        name = os.path.splitext(os.path.basename(src))[0]
        if ext == ".dxf":
            layer = QgsVectorLayer(src, name, "ogr")
            if not layer.isValid():
                # Fallback to the iface helper, which uses the normal OGR vector path.
                layer = self.iface.addVectorLayer(src, name, "ogr")
                if layer is None or not layer.isValid():
                    raise ValueError("QGIS could not open this DXF as a normal vector layer.")
                return layer
            QgsProject.instance().addMapLayer(layer)
            return layer
        # For non-DXF paths, use the existing supported-file router.
        return self._open_supported_path(src, from_drop=True, drop_action="load")

    def _open_supported_path(self, src, from_drop=False, drop_action=None):
        ext = os.path.splitext(src)[1].lower()
        action = drop_action or str(self._setting("drag_drop_action", "ask"))
        if action == "ask":
            action = "load"

        if action == "convert_2dm":
            if ext in (".12da", ".12daz"):
                return self._convert_12da_to_2dm_path(src)
            if ext == ".tin":
                return self._convert_tin_to_2dm_path(src)
            if ext == ".dxf":
                return self._convert_dxf_3dface_path(src, show_message=not from_drop)
            if ext in (".xml", ".landxml"):
                return self._convert_landxml_path(src, show_message=not from_drop)

        if ext in (".12da", ".12daz"):
            return self._open_12da_as_tin(src, show_message=False)
        if ext == ".tin":
            return self._open_tin_auto(src)
        if ext == ".dxf":
            return self._open_dxf_3dface_as_tin_path(src, show_message=not from_drop)
        if ext in (".xml", ".landxml"):
            return self._convert_landxml_path(src, show_message=not from_drop)
        raise ValueError("Unsupported file type")
