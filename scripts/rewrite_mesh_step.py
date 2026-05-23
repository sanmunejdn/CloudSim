from pathlib import Path

orig = Path(r"D:\Project\VSprogram\CGAL5.5.2\CloudSim\src\Data\Data\source\MeshBackendData.cpp").read_text(encoding="utf-8").splitlines(keepends=True)
d = Path(r"D:\Project\VSprogram\CGAL5.5.2\CloudSim\src\Data\Data\source")

head = (
    '#include "pch.h"\n'
    '#include "MeshBackendData_loaders.h"\n'
    '#include "MeshBackendData.h"\n'
    '#include "RunLogger.h"\n\n'
    'namespace mesh_backend_load {\n\n'
)

body = orig[626:771] + orig[1216:1254]
body_text = "".join(body)
body_text = body_text.replace("constexpr bool kMeshStepFlipReversedFaceWinding = true;\n\n", "")
body_text = body_text.replace("static void meshPushTransformedTri", "void meshPushTransformedTri")
body_text = body_text.replace("static void meshAppendShapeTriangles", "void meshAppendShapeTriangles")
body_text = body_text.replace("static bool meshHasChildren", "bool meshHasChildren")
body_text = body_text.replace("static std::string meshShapeTypeName", "std::string meshShapeTypeName")
body_text = body_text.replace("static void meshCollectStepHierarchyRecursive", "void meshCollectStepHierarchyRecursive")

extra = """
bool meshLoadStepSingleFile(const std::string& path, std::vector<float>& soup, std::string* errMsg)
{
\tstd::vector<float> tmp;
\tif (!meshLoadStepSingleFile(path, tmp, errMsg))
\t\treturn false;
\tsoup = std::move(tmp);
\treturn true;
}
"""
# fix duplicate name in extra - use inline from original 1362-1411
extra = """
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

} // namespace mesh_backend_load

"""

hierarchy = "".join(orig[1216:1254])
hierarchy = hierarchy.replace("meshLoadErr(", "mesh_backend_load::meshLoadErr(")
hierarchy = hierarchy.replace("meshCollectStepHierarchyRecursive(", "mesh_backend_load::meshCollectStepHierarchyRecursive(")

(d / "MeshBackendData_step.cpp").write_text(head + body_text + extra + hierarchy, encoding="utf-8")
print("step rewritten")
