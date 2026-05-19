# -*- coding: utf-8 -*-
"""LandXML surface loading helpers.

LandXML support is an independent QFAT02 implementation inspired by, and
written with reference to, Lutra Consulting's open-source LandXML mesh
conversion work:
https://github.com/lutraconsulting/qgis-landxml-mesh-plugin

Only the LandXML import/loading side is used here. QFAT02 converts recognised
LandXML TIN surfaces to 2DM for QGIS mesh workflows.
"""

import copy
import xml.etree.ElementTree as ET


SURFACE_VERTEX_ID_OFFSET = 10000


class LandXMLVertex(object):
    def __init__(self, vertex_id, x, y, z=0.0):
        self.id = int(vertex_id)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    @classmethod
    def from_xml_element(cls, element):
        vertex_id = element.attrib.get("id") or element.attrib.get("ID") or element.attrib.get("name")
        if vertex_id is None:
            raise ValueError("LandXML point is missing an id")

        text = (element.text or "").strip().replace(",", " ")
        parts = [p for p in text.split() if p]
        if len(parts) < 2:
            raise ValueError("LandXML point has fewer than two coordinates")

        # LandXML surface points are commonly stored as: northing easting elevation.
        y = float(parts[0])
        x = float(parts[1])
        z = float(parts[2]) if len(parts) >= 3 else 0.0
        return cls(vertex_id, x, y, z)

    def copy_with_offset(self, offset):
        return LandXMLVertex(self.id + int(offset), self.x, self.y, self.z)


class LandXMLFace(object):
    def __init__(self, face_id, point_ids):
        self.id = int(face_id)
        self.point_ids = [int(v) for v in point_ids]

    @classmethod
    def from_xml_element(cls, face_id, element):
        text = (element.text or "").strip().replace(",", " ")
        point_ids = [int(v) for v in text.split() if v]
        if len(point_ids) < 3:
            raise ValueError("LandXML face has fewer than three vertices")
        return cls(face_id, point_ids[:4])

    def copy_with_offset(self, offset):
        offset = int(offset)
        return LandXMLFace(self.id + offset, [v + offset for v in self.point_ids])


class LandXMLSurface(object):
    def __init__(self, surface_element, surface_number, namespace_prefix="", namespace=None):
        self.surface = surface_element
        self.surface_number = int(surface_number)
        self.namespace_prefix = namespace_prefix
        self.namespace = namespace or {"": ""}
        self.name = self.surface.attrib.get("name") or "Surface_{}".format(surface_number + 1)
        self.id_offset = self.surface_number * SURFACE_VERTEX_ID_OFFSET
        self._points = []
        self._faces = []
        self._read_points()
        self._read_faces()

    def _find_child(self, parent, name):
        if parent is None:
            return None
        found = parent.find("{}{}".format(self.namespace_prefix, name), namespaces=self.namespace)
        if found is not None:
            return found
        for child in list(parent):
            if _local_name(child.tag).lower() == name.lower():
                return child
        return None

    @property
    def definition(self):
        return self._find_child(self.surface, "Definition")

    def _read_points(self):
        points_parent = self._find_child(self.definition, "Pnts")
        if points_parent is None:
            return
        for point_element in list(points_parent):
            if _local_name(point_element.tag).lower() != "p":
                continue
            try:
                self._points.append(LandXMLVertex.from_xml_element(point_element))
            except Exception:
                continue

    def _read_faces(self):
        faces_parent = self._find_child(self.definition, "Faces")
        if faces_parent is None:
            return
        next_face_id = 1
        for face_element in list(faces_parent):
            if _local_name(face_element.tag).lower() != "f":
                continue

            # Some public LandXML mesh examples skip faces marked with i="1".
            try:
                if int(face_element.attrib.get("i", "0")) == 1:
                    continue
            except ValueError:
                pass

            try:
                self._faces.append(LandXMLFace.from_xml_element(next_face_id, face_element))
                next_face_id += 1
            except Exception:
                continue

    def points(self, add_id_offset=False):
        points = copy.deepcopy(self._points)
        if add_id_offset:
            points = [p.copy_with_offset(self.id_offset) for p in points]
        return points

    def faces(self, add_id_offset=False):
        faces = copy.deepcopy(self._faces)
        if add_id_offset:
            faces = [f.copy_with_offset(self.id_offset) for f in faces]
        return faces

    def empty(self):
        return not self._points or not self._faces


class LandXMLReader(object):
    """Read LandXML surfaces and expose point/face lists.

    This is a small, QGIS-independent import-only reader for QFAT02,
    inspired by and written with reference to Lutra Consulting's open-source
    LandXML mesh converter.
    """

    def __init__(self, path):
        self.path = path
        self.xml_tree = ET.parse(path)
        self.xml_root = self.xml_tree.getroot()
        if "landxml" not in _local_name(self.xml_root.tag).lower():
            raise ValueError("Not a valid LandXML file")

        self.namespace_prefix, self.namespace = _namespace_info(self.xml_root)
        self.surfaces = []
        self._read_surfaces()

    @property
    def surface_count(self):
        return len(self.surfaces)

    def _read_surfaces(self):
        surfaces_parent = self.xml_root.find("{}Surfaces".format(self.namespace_prefix), namespaces=self.namespace)
        if surfaces_parent is None:
            # Fallback for files without a standard namespace or with unusual nesting.
            for elem in self.xml_root.iter():
                if _local_name(elem.tag).lower() == "surfaces":
                    surfaces_parent = elem
                    break

        if surfaces_parent is None:
            return

        for i, surface_element in enumerate(list(surfaces_parent)):
            if _local_name(surface_element.tag).lower() != "surface":
                continue
            surface = LandXMLSurface(surface_element, i, self.namespace_prefix, self.namespace)
            if not surface.empty():
                self.surfaces.append(surface)

    def crs_text(self):
        coord = self.xml_root.find("{}CoordinateSystem".format(self.namespace_prefix), namespaces=self.namespace)
        if coord is None:
            for elem in self.xml_root.iter():
                if _local_name(elem.tag).lower() == "coordinatesystem":
                    coord = elem
                    break
        if coord is None:
            return ""
        if "epsgCode" in coord.attrib:
            return "EPSG:{}".format(coord.attrib.get("epsgCode"))
        if "ogcWktCode" in coord.attrib:
            return coord.attrib.get("ogcWktCode", "")
        return ""

    @property
    def all_points(self):
        points = []
        for surface in self.surfaces:
            points.extend(surface.points(add_id_offset=True))
        return points

    @property
    def all_faces(self):
        faces = []
        for surface in self.surfaces:
            faces.extend(surface.faces(add_id_offset=True))
        return faces


def parse_landxml_surface(path, surface_index=None):
    """Return vertices and faces from a LandXML TIN surface.

    By default all recognised surfaces are merged. Faces may be triangles or
    quads; write_2dm() handles both as E3T/E4Q.
    """
    reader = LandXMLReader(path)
    if not reader.surfaces:
        return [], []

    if surface_index is None:
        points = reader.all_points
        faces = reader.all_faces
    else:
        surface = reader.surfaces[int(surface_index)]
        points = surface.points(add_id_offset=False)
        faces = surface.faces(add_id_offset=False)

    id_to_new = {}
    verts = []
    for point in points:
        if point.id in id_to_new:
            continue
        id_to_new[point.id] = len(verts) + 1
        verts.append((point.x, point.y, point.z))

    out_faces = []
    for face in faces:
        mapped = [id_to_new[v] for v in face.point_ids if v in id_to_new]
        if len(mapped) >= 3 and len(set(mapped[:3])) == 3:
            if len(mapped) >= 4 and len(set(mapped[:4])) == 4:
                out_faces.append((mapped[0], mapped[1], mapped[2], mapped[3]))
            else:
                out_faces.append((mapped[0], mapped[1], mapped[2]))

    return verts, out_faces


def _local_name(tag):
    return str(tag).split("}", 1)[-1] if "}" in str(tag) else str(tag)


def _namespace_info(root):
    tag = str(root.tag)
    if tag.startswith("{") and "}" in tag:
        uri = tag[1:].split("}", 1)[0]
        return "ns:", {"ns": uri}
    return "", {"": ""}
