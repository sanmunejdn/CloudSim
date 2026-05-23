from pathlib import Path

src = Path(r"D:\Project\VSprogram\CGAL5.5.2\CloudSim\src\Data\Data\source\MeshBackendData.cpp")
lines = src.read_text(encoding="utf-8").splitlines(keepends=True)
d = src.parent


def write_file(name: str, parts: list[str]) -> None:
    (d / name).write_text("".join(parts), encoding="utf-8")


core_h = [
    '#include "pch.h"\n',
    '#include "MeshBackendData.h"\n',
    '#include "BackendObjectAttribute.h"\n',
    '#include "BackendPropertyRow.h"\n',
    '#include "RunLogger.h"\n',
    '#include "geometry_base64.h"\n',
    '#include "../../PropertyCore/inc/PropertyAttribute.h"\n\n',
]
write_file("MeshBackendData_core.cpp", core_h + lines[0:307])

load_h = [
    '#include "pch.h"\n',
    '#include "MeshBackendData_loaders.h"\n',
    '#include "MeshBackendData.h"\n',
    '#include "RunLogger.h"\n\n',
]
write_file(
    "MeshBackendData_load_common.cpp",
    load_h + lines[308:625] + lines[642:668],
)

step_h = [
    '#include "pch.h"\n',
    '#include "MeshBackendData_loaders.h"\n',
    '#include "MeshBackendData.h"\n',
    '#include "RunLogger.h"\n\n',
]
step_extra = """
bool meshLoadStepSingleFile(const std::string& path, std::vector<float>& soup, std::string* errMsg)
{
\tSTEPControl_Reader reader;
\tconst IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
\tif (status != IFSelect_RetDone)
\t{
\t\tmeshLoadErr(errMsg, "OCCT STEP read failed.");
\t\treturn false;
\t}
\tconst bool ok = reader.TransferRoots();
\tif (!ok)
\t{
\t\tmeshLoadErr(errMsg, "OCCT STEP transfer failed.");
\t\treturn false;
\t}
\tconst TopoDS_Shape shape = reader.OneShape();
\tif (shape.IsNull())
\t{
\t\tmeshLoadErr(errMsg, "OCCT STEP produced an empty shape.");
\t\treturn false;
\t}
\tconst Standard_Real linDeflectionRel = 0.01;
\tconst Standard_Boolean isRelative = Standard_True;
\tconst Standard_Real angDeflection = 0.5;
\tBRepMesh_IncrementalMesh mesher(shape, linDeflectionRel, isRelative, angDeflection, Standard_False);
\t(void)mesher;
\tmeshAppendShapeTriangles(shape, soup);
\tif (soup.empty())
\t{
\t\tmeshLoadErr(errMsg, "OCCT STEP triangulation produced an empty triangle soup.");
\t\treturn false;
\t}
\treturn true;
}
"""
write_file(
    "MeshBackendData_step.cpp",
    step_h + lines[626:641] + lines[691:771] + lines[1216:1254] + [step_extra],
)

dxf_h = [
    '#include "pch.h"\n',
    '#include "MeshBackendData_loaders.h"\n',
    '#include "MeshBackendData.h"\n',
    '#include "RunLogger.h"\n\n',
    '#include "dl_creationadapter.h"\n',
    '#include "dl_dxf.h"\n\n',
]
write_file(
    "MeshBackendData_dxf.cpp",
    dxf_h + lines[669:690] + lines[772:1214] + lines[1255:1350],
)

cgal_h = [
    '#include "pch.h"\n',
    '#include "MeshBackendData_loaders.h"\n',
    '#include "MeshBackendData.h"\n',
    '#include "RunLogger.h"\n\n',
]
load_from_file = lines[1351:1485]
# Replace inline STEP block with helper call
text = "".join(load_from_file)
step_block_start = text.find('\tif (ext == "step" || ext == "stp")')
step_block_end = text.find('\tif (ext == "dxf")')
if step_block_start != -1 and step_block_end != -1:
    replacement = (
        '\tif (ext == "step" || ext == "stp")\n'
        '\t{\n'
        '\t\tstd::vector<float> soup;\n'
        '\t\tif (!meshLoadStepSingleFile(path, soup, errMsg))\n'
        '\t\t{\n'
        '\t\t\treturn false;\n'
        '\t\t}\n'
        '\t\tsetTriangleSoup(std::move(soup));\n'
        '\t\tRunLogger::info("[MeshBackendData] STEP mesh loaded successfully.");\n'
        '\t\treturn !m_triangleSoup.empty();\n'
        '\t}\n\n'
    )
    text = text[:step_block_start] + replacement + text[step_block_end:]

write_file("MeshBackendData_cgal_io.cpp", cgal_h + [text])
print("split ok")
