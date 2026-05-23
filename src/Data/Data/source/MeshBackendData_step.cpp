#include "pch.h"
#include "MeshBackendData_loaders.h"
#include "MeshBackendData.h"
#include "RunLogger.h"

namespace mesh_backend_load {

void meshPushTransformedTri(std::vector<float>& soup, const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3,
	const bool reverseWinding)
{
	const gp_Pnt& pb = reverseWinding ? p3 : p2;
	const gp_Pnt& pc = reverseWinding ? p2 : p3;
	soup.push_back(static_cast<float>(p1.X()));
	soup.push_back(static_cast<float>(p1.Y()));
	soup.push_back(static_cast<float>(p1.Z()));
	soup.push_back(static_cast<float>(pb.X()));
	soup.push_back(static_cast<float>(pb.Y()));
	soup.push_back(static_cast<float>(pb.Z()));
	soup.push_back(static_cast<float>(pc.X()));
	soup.push_back(static_cast<float>(pc.Y()));
	soup.push_back(static_cast<float>(pc.Z()));
}

void meshAppendShapeTriangles(const TopoDS_Shape& shape, std::vector<float>& soup)
{
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
	{
		const TopoDS_Face face = TopoDS::Face(exp.Current());
		const bool reverseWinding = kMeshStepFlipReversedFaceWinding && (face.Orientation() == TopAbs_REVERSED);
		TopLoc_Location loc;
		Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		if (tri.IsNull() || !tri->HasGeometry() || tri->NbTriangles() <= 0)
		{
			continue;
		}
		const gp_Trsf xf = loc.Transformation();
		for (Standard_Integer ti = 1; ti <= tri->NbTriangles(); ++ti)
		{
			const Poly_Triangle& t = tri->Triangle(ti);
			Standard_Integer n1 = 0, n2 = 0, n3 = 0;
			t.Get(n1, n2, n3);
			gp_Pnt p1 = tri->Node(n1);
			gp_Pnt p2 = tri->Node(n2);
			gp_Pnt p3 = tri->Node(n3);
			p1.Transform(xf);
			p2.Transform(xf);
			p3.Transform(xf);
			meshPushTransformedTri(soup, p1, p2, p3, reverseWinding);
		}
	}
}

bool meshHasChildren(const TopoDS_Shape& shape)
{
	TopoDS_Iterator it(shape);
	return it.More();
}

std::string meshShapeTypeName(const TopAbs_ShapeEnum t)
{
	switch (t)
	{
	case TopAbs_COMPOUND: return "Compound";
	case TopAbs_COMPSOLID: return "CompSolid";
	case TopAbs_SOLID: return "Solid";
	case TopAbs_SHELL: return "Shell";
	case TopAbs_FACE: return "Face";
	case TopAbs_WIRE: return "Wire";
	case TopAbs_EDGE: return "Edge";
	case TopAbs_VERTEX: return "Vertex";
	default: return "Shape";
	}
}

void meshCollectStepHierarchyRecursive(
	const TopoDS_Shape& shape,
	const std::string& path,
	const std::string& parentPath,
	std::vector<MeshHierarchyPart>& outParts)
{
	const bool hasChildren = meshHasChildren(shape);
	if (!hasChildren)
	{
		MeshHierarchyPart part;
		part.partPath = path;
		part.parentPartPath = parentPath;
		part.displayName = meshShapeTypeName(shape.ShapeType()) + "_" + path;
		meshAppendShapeTriangles(shape, part.triangleSoup);
		if (!part.triangleSoup.empty())
		{
			outParts.push_back(std::move(part));
		}
		return;
	}

	int childIndex = 0;
	for (TopoDS_Iterator it(shape); it.More(); it.Next(), ++childIndex)
	{
		const TopoDS_Shape child = it.Value();
		const std::string childPath = path.empty() ? std::to_string(childIndex) : (path + "/" + std::to_string(childIndex));
		meshCollectStepHierarchyRecursive(child, childPath, path, outParts);
	}
}

bool meshLoadStepSingleFile(const std::string& path, std::vector<float>& soup, std::string* errMsg)
{
	STEPControl_Reader reader;
	const IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
	if (status != IFSelect_RetDone)
	{
		meshLoadErr(errMsg, "OCCT STEP read failed.");
		return false;
	}
	const bool ok = reader.TransferRoots();
	if (!ok)
	{
		meshLoadErr(errMsg, "OCCT STEP transfer failed.");
		return false;
	}
	const TopoDS_Shape shape = reader.OneShape();
	if (shape.IsNull())
	{
		meshLoadErr(errMsg, "OCCT STEP produced an empty shape.");
		return false;
	}
	const Standard_Real linDeflectionRel = 0.01;
	const Standard_Boolean isRelative = Standard_True;
	const Standard_Real angDeflection = 0.5;
	BRepMesh_IncrementalMesh mesher(shape, linDeflectionRel, isRelative, angDeflection, Standard_False);
	(void)mesher;
	meshAppendShapeTriangles(shape, soup);
	if (soup.empty())
	{
		meshLoadErr(errMsg, "OCCT STEP triangulation produced an empty triangle soup.");
		return false;
	}
	return true;
}

} // namespace mesh_backend_load

bool MeshBackendData::loadStepHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts, std::string* errMsg)
{
	outParts.clear();
	STEPControl_Reader reader;
	const IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
	if (status != IFSelect_RetDone)
	{
		mesh_backend_load::meshLoadErr(errMsg, "OCCT STEP read failed.");
		return false;
	}
	const bool ok = reader.TransferRoots();
	if (!ok)
	{
		mesh_backend_load::meshLoadErr(errMsg, "OCCT STEP transfer failed.");
		return false;
	}
	const TopoDS_Shape shape = reader.OneShape();
	if (shape.IsNull())
	{
		mesh_backend_load::meshLoadErr(errMsg, "OCCT STEP produced an empty shape.");
		return false;
	}

	const Standard_Real linDeflectionRel = 0.01;
	const Standard_Boolean isRelative = Standard_True;
	const Standard_Real angDeflection = 0.5;
	BRepMesh_IncrementalMesh mesher(shape, linDeflectionRel, isRelative, angDeflection, Standard_False);
	(void)mesher;

	mesh_backend_load::meshCollectStepHierarchyRecursive(shape, "0", std::string(), outParts);
	if (outParts.empty())
	{
		mesh_backend_load::meshLoadErr(errMsg, "OCCT STEP hierarchy triangulation produced no mesh parts.");
		return false;
	}
	RunLogger::info("[MeshBackendData] STEP hierarchy loaded successfully.");
	return true;
}
