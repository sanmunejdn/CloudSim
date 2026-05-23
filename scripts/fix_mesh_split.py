from pathlib import Path

d = Path(r"D:\Project\VSprogram\CGAL5.5.2\CloudSim\src\Data\Data\source")
orig = (d / "MeshBackendData.cpp").read_text(encoding="utf-8").splitlines(keepends=True)

core_h = (
    '#include "pch.h"\n'
    '#include "MeshBackendData.h"\n'
    '#include "BackendObjectAttribute.h"\n'
    '#include "BackendPropertyRow.h"\n'
    '#include "RunLogger.h"\n'
    '#include "geometry_base64.h"\n'
    '#include "../../PropertyCore/inc/PropertyAttribute.h"\n\n'
)
(d / "MeshBackendData_core.cpp").write_text(core_h + "".join(orig[42:307]), encoding="utf-8")

common = (d / "MeshBackendData_load_common.cpp").read_text(encoding="utf-8")
common = common.replace("namespace {", "namespace mesh_backend_load {", 1)
if not common.rstrip().endswith("}"):
    common = common.rstrip() + "\n\n} // namespace mesh_backend_load\n"
(d / "MeshBackendData_load_common.cpp").write_text(common, encoding="utf-8")

for name in ("MeshBackendData_step.cpp", "MeshBackendData_dxf.cpp"):
    text = (d / name).read_text(encoding="utf-8")
    if "namespace mesh_backend_load" not in text:
        insert = '#include "MeshBackendData_loaders.h"\n\nnamespace mesh_backend_load {\n\n'
        # already has loaders include
        lines = text.split("\n", 7)
        head = "\n".join(lines[:7]) + "\n\nnamespace mesh_backend_load {\n\n"
        body = "\n".join(lines[7:])
        body = body.replace("static void meshAppendShapeTriangles", "void meshAppendShapeTriangles")
        body = body.replace("static void meshCollectStepHierarchyRecursive", "void meshCollectStepHierarchyRecursive")
        text = head + body
    if not text.rstrip().endswith("}"):
        text = text.rstrip() + "\n\n} // namespace mesh_backend_load\n"
    (d / name).write_text(text, encoding="utf-8")

# step: MeshBackendData methods outside namespace
step = (d / "MeshBackendData_step.cpp").read_text(encoding="utf-8")
if "bool MeshBackendData::loadStepHierarchyFromFile" in step and "mesh_backend_load::meshLoadErr" not in step:
    step = step.replace("meshLoadErr(", "mesh_backend_load::meshLoadErr(")
    (d / "MeshBackendData_step.cpp").write_text(step, encoding="utf-8")

cgal = (d / "MeshBackendData_cgal_io.cpp").read_text(encoding="utf-8")
if "using namespace mesh_backend_load" not in cgal:
    cgal = cgal.replace(
        '#include "RunLogger.h"\n\n',
        '#include "RunLogger.h"\n\nusing namespace mesh_backend_load;\n\n',
        1,
    )
dxf_block = """\tif (ext == "dxf")
\t{
\t\tstd::vector<float> soup;
\t\tif (!meshLoadDxfSingleFile(path, soup, errMsg))
\t\t{
\t\t\treturn false;
\t\t}
\t\tsetTriangleSoup(std::move(soup));
\t\tRunLogger::info("[MeshBackendData] DXF mesh loaded successfully.");
\t\treturn true;
\t}
"""
start = cgal.find('\tif (ext == "dxf")')
end = cgal.find('\t// OBJ', start)
if start != -1 and end != -1:
    cgal = cgal[:start] + dxf_block + "\n" + cgal[end:]
(d / "MeshBackendData_cgal_io.cpp").write_text(cgal, encoding="utf-8")

dxf = (d / "MeshBackendData_dxf.cpp").read_text(encoding="utf-8")
if "meshLoadDxfSingleFile" not in dxf:
    insert_fn = '''
bool meshLoadDxfSingleFile(const std::string& path, std::vector<float>& soup, std::string* errMsg)
{
\tsoup.clear();
\tDL_Dxf dxf;
\tMeshDxfCollector collector;
\tif (!dxf.in(path, &collector))
\t{
\t\tmeshLoadErr(errMsg, "Could not open DXF file.");
\t\treturn false;
\t}
\tif (collector.soup.empty())
\t{
\t\tmeshLoadErr(errMsg,
\t\t\t"DXF contained no triangulatable geometry (3DFACE/TRACE/SOLID or closed polyline / polygon mesh).");
\t\treturn false;
\t}
\tsoup = std::move(collector.soup);
\treturn true;
}

'''
    marker = "bool MeshBackendData::loadDxfHierarchyFromFile"
    dxf = dxf.replace(marker, insert_fn + marker)
    dxf = dxf.replace("meshLoadErr(", "mesh_backend_load::meshLoadErr(")
    dxf = dxf.replace("meshPushTri(", "mesh_backend_load::meshPushTri(")
    (d / "MeshBackendData_dxf.cpp").write_text(dxf, encoding="utf-8")

print("fixed")
