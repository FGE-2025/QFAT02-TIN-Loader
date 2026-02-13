# QFAT02 TIN Loader

QFAT02 TIN Loader imports 12d (.12da) and SMS/XMS (.tin) terrain surfaces into QGIS as mesh layers for fast review and QA. It can then convert the surface to 2DM for downstream editing or model inputs (including TUFLOW, which can read 2DM directly).

QFAT02 is part of QFAT (QGIS Flood Analysis Toolkit): a set of focused QGIS utilities supporting flood modelling workflows (e.g. TUFLOW, HEC-RAS), hydraulic QA, and rapid engineering checks.

- Open **12d (.12da)** and **SMS/XMS (.tin)** files as Mesh layers.  
- Convert directly to **2DM** for full editing with **QGIS Mesh Digitizing tools**.  
- TUFLOW should be able to read **2DM** format directly.

**Tested with QGIS 3.34 and 3.40.**

**Current version:** 2.1.0

---

**Future improvements:**  
- Handle different encoding (for now, modellers please do your own conversion to ANSI, untick *Full TIN* when exporting from 12d, etc.).  
- Support for more formats (e.g. `.12daz`, 3D faces in DWG/DXF).  
- Other interface improvements.  

---

**Status:** First release — feedback welcome!
