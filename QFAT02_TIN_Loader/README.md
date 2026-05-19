# QFAT02 TIN Loader

QFAT02 TIN Loader imports common engineering terrain surfaces into QGIS for fast review and QA. Supported inputs include 12d (.12da/.12daz), SMS/XMS or MDAL/QGIS-style (.tin), DXF 3DFACE, and LandXML surfaces. It can convert supported surfaces to 2DM for downstream editing or model inputs, including TUFLOW workflows.

This tool is part of QFAT (QGIS Flood Analysis Toolkit) — a set of focused QGIS utilities designed to support flood modelling workflows (e.g., TUFLOW, HEC-RAS), hydraulic QA, and rapid engineering checks.

- Open **12d (.12da/.12daz)** and **SMS/XMS or MDAL/QGIS-style (.tin)** files as Mesh layers.  
- Convert **DXF 3DFACE** terrain to editable **2DM**.  
- Convert **LandXML** TIN surfaces to editable **2DM**.  
- Drag and drop supported files onto the QGIS map canvas or layer panel / TOC, including `.12da`, `.12daz`, `.tin`, `.dxf`, `.xml`, and `.landxml`.  
- Convert directly to **2DM** for full editing with **QGIS Mesh Digitizing tools**.  
- TUFLOW should be able to read **2DM** format directly.

**Tested with QGIS 3.34 and 3.40.**


**Current version:** v2.2.27

---

**Future improvements:**  
- Add 12d full TIN conversion tools.
- Add 12d to HEC conversion tools.
- Add 12d / SMS TIN export options.
- Add SHP / GeoPackage import using TUFLOW utilities.
- Hook into the planned CodePad fuzzy loader.
- Add GDA conversion support.

---

**Status:** 2nd release — feedback welcome!


## TIN format detection

QFAT02 treats `.tin` as an ambiguous extension. It detects likely SMS/XMS-style TIN files and normalises them before loading, while likely MDAL/QGIS-readable ASCII TIN files are loaded directly where possible.


## Settings

QFAT02 includes a small settings dialog for common import behaviour:

- **Default drag-and-drop action**: ask every time, load as mesh/TIN where possible, or convert to 2DM and load.
- **Skip hidden 12d triangles in full TIN**: skip hidden/nulled triangles and load only the visible TIN.
- **TIN handling**: auto-detect, force SMS/XMS, or force direct MDAL/QGIS load.
- **Output overwrite behaviour**: ask before overwrite, auto-rename, or overwrite.
- **Import summary message**: show or hide import/conversion summary messages.
- **About**: shows the QFAT02 description and credits.

12d encoding conversion is automatic: QFAT02 detects UTF-8, UTF-8 BOM, UTF-16 LE/BE BOM, ANSI/CP1252, and Latin-1 fallback, then converts internally to UTF-8 for parsing. It warns when the source 12da is not UTF-8 or ANSI/CP1252.

If QFAT02 cannot write a generated `.tin` or `.2dm` beside the source file because the folder is read-only or write-protected, it shows a warning and offers to save the output to the QGIS temporary folder instead.

12daz handling assumes one main `.12da` inside the archive. If multiple `.12da` files are found, QFAT02 uses the largest file.


## Toolbar and menu

QFAT02 uses one toolbar dropdown button to keep the QGIS toolbar tidy. The toolbar dropdown and plugin menu use the same file-type grouping:

- 12DA / 12DAZ
- TIN
- DXF
- LandXML
- Current Mesh
- Settings



Note: For drag-and-drop, QFAT02 only intercepts DXF files that contain 3DFACE terrain geometry. Normal DXF files without 3DFACE are left for QGIS native loading. For 3DFACE DXF files, the prompt includes **Open normally in QGIS**.


Note: DXF 3DFACE can be opened as TIN/Mesh, converted to 2DM, or opened normally in QGIS when dropped onto the canvas or layer panel.

## Disclaimer

This is a personal open-source project developed independently using public QGIS APIs and general programming methods.

This project is not affiliated with, endorsed by, or supported by any employer or client organisation.

No company confidential information, client data, project files, internal templates, or proprietary code are intentionally included.

Users are responsible for reviewing and testing the tool before use in any professional workflow.


## Licence and third-party credits

QFAT02 TIN Loader is released under the licence included in this repository.

Credit to Lee Canavan, whose initial discussion and idea helped inspire QFAT02 TIN Loader.

LandXML support is an independent QFAT02 implementation inspired by, and written with reference to, Lutra Consulting's open-source LandXML mesh conversion work. The relevant credit is also noted in `io_landxml.py`.

QGIS, TUFLOW, HEC-RAS, 12d, SMS/XMS, LandXML, DXF, and other named formats or software remain the property of their respective owners.

