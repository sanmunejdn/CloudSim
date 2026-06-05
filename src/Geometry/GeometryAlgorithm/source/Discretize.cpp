#include "detail/OccIncludes.h"

#include "Discretize.h"
#include "ShapeHandle.h"
#include "ShapeQuery.h"
#include "ShapeIo.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace geoalgo
{
namespace detail
{

constexpr bool kFlipReversedFaceWinding = true;

void setErr(std::string* errMsg, const char* text)
{
	if (errMsg)
	{
		*errMsg = text;
	}
}

void pushTri(
	std::vector<float>& soup,
	const gp_Pnt& p1,
	const gp_Pnt& p2,
	const gp_Pnt& p3,
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

void appendShapeTriangles(const TopoDS_Shape& shape, const TessellateParams& params, std::vector<float>& soup)
{
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
	{
		const TopoDS_Face face = TopoDS::Face(exp.Current());
		const bool reverseWinding = params.flipReversedFaces && kFlipReversedFaceWinding
			&& (face.Orientation() == TopAbs_REVERSED);
		TopLoc_Location loc;
		const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		if (tri.IsNull() || !tri->HasGeometry() || tri->NbTriangles() <= 0)
		{
			continue;
		}
		const gp_Trsf xf = loc.Transformation();
		for (Standard_Integer ti = 1; ti <= tri->NbTriangles(); ++ti)
		{
			const Poly_Triangle& t = tri->Triangle(ti);
			Standard_Integer n1 = 0;
			Standard_Integer n2 = 0;
			Standard_Integer n3 = 0;
			t.Get(n1, n2, n3);
			gp_Pnt p1 = tri->Node(n1);
			gp_Pnt p2 = tri->Node(n2);
			gp_Pnt p3 = tri->Node(n3);
			p1.Transform(xf);
			p2.Transform(xf);
			p3.Transform(xf);
			pushTri(soup, p1, p2, p3, reverseWinding);
		}
	}
}

bool shapeHasChildren(const TopoDS_Shape& shape)
{
	TopoDS_Iterator it(shape);
	return it.More();
}

std::string shapeTypeName(const TopAbs_ShapeEnum t)
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

void collectHierarchyRecursive(
	const TopoDS_Shape& shape,
	const std::string& path,
	const std::string& parentPath,
	const TessellateParams& params,
	std::vector<MeshHierarchyPart>& outParts)
{
	if (!shapeHasChildren(shape))
	{
		MeshHierarchyPart part;
		part.partPath = path;
		part.parentPartPath = parentPath;
		part.displayName = shapeTypeName(shape.ShapeType()) + "_" + path;
		appendShapeTriangles(shape, params, part.triangleSoup);
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
		collectHierarchyRecursive(child, childPath, path, params, outParts);
	}
}

void collectHierarchyTopologyRecursive(
	const TopoDS_Shape& shape,
	const std::string& path,
	const std::string& parentPath,
	std::vector<MeshHierarchyPart>& outParts)
{
	if (!shapeHasChildren(shape))
	{
		MeshHierarchyPart part;
		part.partPath = path;
		part.parentPartPath = parentPath;
		part.displayName = shapeTypeName(shape.ShapeType()) + "_" + path;
		outParts.push_back(std::move(part));
		return;
	}

	int childIndex = 0;
	for (TopoDS_Iterator it(shape); it.More(); it.Next(), ++childIndex)
	{
		const TopoDS_Shape child = it.Value();
		const std::string childPath = path.empty() ? std::to_string(childIndex) : (path + "/" + std::to_string(childIndex));
		collectHierarchyTopologyRecursive(child, childPath, path, outParts);
	}
}

} // namespace detail

void applyQualityPreset(MeshDiscretizeParams& params)
{
	if (params.quality == MeshQualityPreset::Custom)
	{
		return;
	}
	switch (params.quality)
	{
	case MeshQualityPreset::Coarse:
		params.tessellate.linearDeflectionMm = 0.05;
		params.tessellate.angularDeflectionDeg = 1.0;
		break;
	case MeshQualityPreset::Fine:
		params.tessellate.linearDeflectionMm = 0.002;
		params.tessellate.angularDeflectionDeg = 0.25;
		break;
	case MeshQualityPreset::Medium:
	default:
		params.tessellate.linearDeflectionMm = 0.01;
		params.tessellate.angularDeflectionDeg = 0.5;
		break;
	}
}

bool meshShapeIncremental(const TopoDS_Shape& shape, const TessellateParams& params, std::string* errMsg)
{
	if (shape.IsNull())
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	const Standard_Real linDef = params.linearDeflectionMm;
	const Standard_Boolean isRelative = params.linearDeflectionRelative ? Standard_True : Standard_False;
	const Standard_Real angDef = params.angularDeflectionDeg;
	BRepMesh_IncrementalMesh mesher(shape, linDef, isRelative, angDef, Standard_False);
	(void)mesher;
	return true;
}

bool discretizeFaceToSoup(
	const TopoDS_Face& face,
	const TessellateParams& params,
	std::vector<float>& soup,
	std::string* errMsg)
{
	TopoDS_Shape shape = face;
	if (!meshShapeIncremental(shape, params, errMsg))
	{
		return false;
	}
	const std::size_t before = soup.size();
	detail::appendShapeTriangles(shape, params, soup);
	if (soup.size() == before)
	{
		detail::setErr(errMsg, "face triangulation empty");
		return false;
	}
	return true;
}

bool discretizeShapeToSoup(
	const TopoDS_Shape& shape,
	const TessellateParams& params,
	std::vector<float>& soup,
	std::string* errMsg)
{
	TopoDS_Shape copy = shape;
	if (!meshShapeIncremental(copy, params, errMsg))
	{
		return false;
	}
	detail::appendShapeTriangles(copy, params, soup);
	if (soup.empty())
	{
		detail::setErr(errMsg, "shape triangulation empty");
		return false;
	}
	return true;
}

bool tessellateStepFile(
	const std::string& pathLocal,
	const TessellateParams& params,
	std::vector<float>& soup,
	std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!readStepShape(pathLocal, shape, errMsg))
	{
		return false;
	}
	return discretizeShapeToSoup(shape, params, soup, errMsg);
}

bool tessellateStepHierarchy(
	const std::string& pathLocal,
	const TessellateParams& params,
	std::vector<MeshHierarchyPart>& outParts,
	std::string* errMsg)
{
	outParts.clear();
	TopoDS_Shape shape;
	if (!readStepShape(pathLocal, shape, errMsg))
	{
		return false;
	}
	return collectShapeHierarchy(shape, params, outParts, errMsg);
}

bool collectShapeHierarchy(
	const TopoDS_Shape& shape,
	const TessellateParams& params,
	std::vector<MeshHierarchyPart>& outParts,
	std::string* errMsg)
{
	outParts.clear();
	TopoDS_Shape copy = shape;
	if (!meshShapeIncremental(copy, params, errMsg))
	{
		return false;
	}
	detail::collectHierarchyRecursive(shape, "0", std::string(), params, outParts);
	if (outParts.empty())
	{
		detail::setErr(errMsg, "shape hierarchy triangulation produced no mesh parts");
		return false;
	}
	return true;
}

bool collectShapeHierarchy(
	const ShapeHandle& shape,
	const TessellateParams& params,
	std::vector<MeshHierarchyPart>& outParts,
	std::string* errMsg)
{
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	return collectShapeHierarchy(native, params, outParts, errMsg);
}

bool collectShapeHierarchyTopology(
	const TopoDS_Shape& shape,
	std::vector<MeshHierarchyPart>& outParts,
	std::string* errMsg)
{
	outParts.clear();
	if (shape.IsNull())
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	detail::collectHierarchyTopologyRecursive(shape, "0", std::string(), outParts);
	if (outParts.empty())
	{
		MeshHierarchyPart root;
		root.partPath = "0";
		root.parentPartPath.clear();
		root.displayName = detail::shapeTypeName(shape.ShapeType()) + "_0";
		outParts.push_back(std::move(root));
	}
	return true;
}

bool collectShapeHierarchyTopology(
	const ShapeHandle& shape,
	std::vector<MeshHierarchyPart>& outParts,
	std::string* errMsg)
{
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	return collectShapeHierarchyTopology(native, outParts, errMsg);
}

bool discretizeShapeFaceByIndex(
	const ShapeHandle& shapeHandle,
	const int faceIndex,
	const TessellateParams& params,
	std::vector<float>& soup,
	std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	TopoDS_Face face;
	if (!shapeFaceAtIndex(shape, faceIndex, face, errMsg))
	{
		return false;
	}
	return discretizeFaceToSoup(face, params, soup, errMsg);
}

} // namespace geoalgo
