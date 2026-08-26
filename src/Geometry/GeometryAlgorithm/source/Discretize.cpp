/// @file Discretize.cpp
/// @brief Discretize 实现

#include "Discretize.h"

#include "CsgkBackend.h"
#include "ShapeHandle.h"
#include "ShapeIo.h"
#include "ShapeQuery.h"
#include "detail/OccIncludes.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

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

void pushTri(std::vector<float>& soup, const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3, const bool reverseWinding)
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

void appendFaceTriangles(const TopoDS_Face& face, const TessellateParams& params, std::vector<float>& soup)
{
	const bool reverseWinding =
		params.flipReversedFaces && kFlipReversedFaceWinding && (face.Orientation() == TopAbs_REVERSED);
	TopLoc_Location loc;
	const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
	if (tri.IsNull() || !tri->HasGeometry() || tri->NbTriangles() <= 0)
	{
		return;
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

void appendShapeTriangles(const TopoDS_Shape& shape, const TessellateParams& params, std::vector<float>& soup)
{
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
	{
		appendFaceTriangles(TopoDS::Face(exp.Current()), params, soup);
	}
}

bool shapeHasChildren(const TopoDS_Shape& shape)
{
	TopoDS_Iterator it(shape);
	return it.More();
}

bool isAssemblyGroup(const TopoDS_Shape& shape)
{
	const TopAbs_ShapeEnum t = shape.ShapeType();
	return (t == TopAbs_COMPOUND || t == TopAbs_COMPSOLID) && shapeHasChildren(shape);
}

std::string shapeTypeName(const TopAbs_ShapeEnum t)
{
	switch (t)
	{
	case TopAbs_COMPOUND:
		return "Compound";
	case TopAbs_COMPSOLID:
		return "CompSolid";
	case TopAbs_SOLID:
		return "Solid";
	case TopAbs_SHELL:
		return "Shell";
	case TopAbs_FACE:
		return "Face";
	case TopAbs_WIRE:
		return "Wire";
	case TopAbs_EDGE:
		return "Edge";
	case TopAbs_VERTEX:
		return "Vertex";
	default:
		return "Shape";
	}
}

void collectHierarchyRecursive(const TopoDS_Shape& shape, const std::string& path, const std::string& parentPath,
							   const TessellateParams& params, std::vector<MeshHierarchyPart>& outParts)
{
	// Compound/CompSolid 才往下拆；Solid 起当叶子，避免拆到 Face
	if (!isAssemblyGroup(shape))
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
		const std::string childPath =
			path.empty() ? std::to_string(childIndex) : (path + "/" + std::to_string(childIndex));
		collectHierarchyRecursive(child, childPath, path, params, outParts);
	}
}

void collectHierarchyTopologyRecursive(const TopoDS_Shape& shape, const std::string& path,
									   const std::string& parentPath, std::vector<MeshHierarchyPart>& outParts)
{
	if (!isAssemblyGroup(shape))
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
		const std::string childPath =
			path.empty() ? std::to_string(childIndex) : (path + "/" + std::to_string(childIndex));
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
	// 复合体上开并行网格会空转，装配 STEP 表现为假死
	BRepMesh_IncrementalMesh mesher(shape, linDef, isRelative, angDef, Standard_False);
	(void)mesher;
	return true;
}

bool discretizeFaceToSoup(const TopoDS_Face& face, const TessellateParams& params, std::vector<float>& soup,
						  std::string* errMsg)
{
	TopoDS_Shape shape = face;
	// 清旧三角，避免 IncrementalMesh 复用残缺/过粗缓存导致面高亮不全。
	// 注意：TopoDS 句柄浅拷贝共享 TShape，Clean 实际清的是调用方 shape 的三角化缓存（OCC 约定）
	BRepTools::Clean(shape);
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

bool discretizeShapeToSoup(const TopoDS_Shape& shape, const TessellateParams& params, std::vector<float>& soup,
						   std::string* errMsg)
{
	TopoDS_Shape copy = shape;
	// 清除旧三角化，避免改 deflection 时 OCC 复用缓存。
	// 句柄浅拷贝共享 TShape：Clean 原地改调用方 shape 的缓存；对同一 shape 并发调用本族 API 不安全
	BRepTools::Clean(copy);
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

bool discretizeShapeToSoupPerFace(const TopoDS_Shape& shape, const TessellateParams& params,
								  std::vector<float>& outSoup, std::vector<int>& outTriangleFaceIndex,
								  std::vector<std::vector<float>>* outFaceSoups, std::string* errMsg)
{
	outSoup.clear();
	outTriangleFaceIndex.clear();
	if (outFaceSoups)
	{
		outFaceSoups->clear();
	}
	if (shape.IsNull())
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	TopoDS_Shape copy = shape;
	// Clean 浅拷贝句柄 = 原地清调用方 shape 的三角化缓存（共享 TShape）；同一 shape 勿并发离散
	BRepTools::Clean(copy);
	if (!meshShapeIncremental(copy, params, errMsg))
	{
		return false;
	}
	int faceIdx = 0;
	for (TopExp_Explorer exp(copy, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx)
	{
		std::vector<float> faceSoup;
		detail::appendFaceTriangles(TopoDS::Face(exp.Current()), params, faceSoup);
		if (outFaceSoups)
		{
			outFaceSoups->push_back(faceSoup);
		}
		if (faceSoup.size() < 9U || (faceSoup.size() % 9U) != 0U)
		{
			continue;
		}
		const std::size_t triCount = faceSoup.size() / 9U;
		outTriangleFaceIndex.insert(outTriangleFaceIndex.end(), triCount, faceIdx);
		outSoup.insert(outSoup.end(), faceSoup.begin(), faceSoup.end());
	}
	if (outSoup.empty())
	{
		detail::setErr(errMsg, "per-face tessellation produced empty mesh");
		return false;
	}
	return true;
}

bool discretizeShapeToSoupPerFace(const ShapeHandle& shape, const TessellateParams& params, std::vector<float>& outSoup,
								  std::vector<int>& outTriangleFaceIndex, std::vector<std::vector<float>>* outFaceSoups,
								  std::string* errMsg)
{
#ifdef CLOUDSIM_USE_CSGK
	if(ShapeHandleAccess::isCsgkBackend(shape))
	{
		// CSGK 后端降级：faceSoups 返回空、triangleFaceIndex 全 0，CSGK shape 的面级拾取/高亮
		// 与 sliceBrepImportArtifactsForShape（要求 faceSoups.size()==面数）静默不可用
		if(!discretizeCsgkShapeToSoup(shape, params, outSoup, errMsg))
			return false;
		outTriangleFaceIndex.assign(outSoup.size() / 9U, 0);
		if(outFaceSoups)
			outFaceSoups->clear();
		return true;
	}
#endif
	TopoDS_Shape native;
	if(!ShapeHandleAccess::nativeShape(shape, &native))
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	return discretizeShapeToSoupPerFace(native, params, outSoup, outTriangleFaceIndex, outFaceSoups, errMsg);
}

bool tessellateStepFile(const std::string& pathLocal, const TessellateParams& params, std::vector<float>& soup,
						std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!readStepShape(pathLocal, shape, errMsg))
	{
		return false;
	}
	return discretizeShapeToSoup(shape, params, soup, errMsg);
}

bool tessellateStepHierarchy(const std::string& pathLocal, const TessellateParams& params,
							 std::vector<MeshHierarchyPart>& outParts, std::string* errMsg)
{
	outParts.clear();
	TopoDS_Shape shape;
	if (!readStepShape(pathLocal, shape, errMsg))
	{
		return false;
	}
	return collectShapeHierarchy(shape, params, outParts, errMsg);
}

bool collectShapeHierarchy(const TopoDS_Shape& shape, const TessellateParams& params,
						   std::vector<MeshHierarchyPart>& outParts, std::string* errMsg)
{
	outParts.clear();
	// mesh 的是 copy、收集的是 shape：二者共享 TShape，三角化经浅拷贝写回同一缓存后才可被 shape 侧读到（隐性契约）
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

bool collectShapeHierarchy(const ShapeHandle& shape, const TessellateParams& params,
						   std::vector<MeshHierarchyPart>& outParts, std::string* errMsg)
{
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	return collectShapeHierarchy(native, params, outParts, errMsg);
}

bool collectShapeHierarchyTopology(const TopoDS_Shape& shape, std::vector<MeshHierarchyPart>& outParts,
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

bool collectShapeHierarchyTopology(const ShapeHandle& shape, std::vector<MeshHierarchyPart>& outParts,
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

bool collectBrepSolidParts(const ShapeHandle& shape, std::vector<ShapeHierarchyPart>& outParts, std::string* errMsg)
{
	outParts.clear();
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	int solidIndex = 0;
	for (TopExp_Explorer exp(native, TopAbs_SOLID); exp.More(); exp.Next(), ++solidIndex)
	{
		const TopoDS_Shape solid = exp.Current();
		ShapeHierarchyPart part;
		part.partPath = "solid/" + std::to_string(solidIndex);
		part.parentPartPath.clear();
		part.displayName = "Solid_" + std::to_string(solidIndex);
		part.shape = ShapeHandleAccess::fromNativeShape(&solid);
		outParts.push_back(std::move(part));
	}
	if (outParts.empty())
	{
		ShapeHierarchyPart part;
		part.partPath = "0";
		part.parentPartPath.clear();
		part.displayName = detail::shapeTypeName(native.ShapeType()) + "_0";
		part.shape = shape;
		outParts.push_back(std::move(part));
	}
	return true;
}

bool collectBrepTopLevelShapeParts(const ShapeHandle& shape, std::vector<ShapeHierarchyPart>& outParts,
								   std::string* errMsg)
{
	outParts.clear();
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		detail::setErr(errMsg, "null shape");
		return false;
	}
	// 只拆一层：子装配内多个 Solid 仍绑在同一子 Shape 上，供自定义设备作单 Link
	if (!detail::isAssemblyGroup(native))
	{
		ShapeHierarchyPart part;
		part.partPath = "0";
		part.parentPartPath.clear();
		part.displayName = detail::shapeTypeName(native.ShapeType()) + "_0";
		part.shape = shape;
		outParts.push_back(std::move(part));
		return true;
	}
	int childIndex = 0;
	for (TopoDS_Iterator it(native); it.More(); it.Next(), ++childIndex)
	{
		const TopoDS_Shape child = it.Value();
		if (child.IsNull())
		{
			continue;
		}
		ShapeHierarchyPart part;
		part.partPath = std::to_string(childIndex);
		part.parentPartPath.clear();
		part.displayName = detail::shapeTypeName(child.ShapeType()) + "_" + part.partPath;
		part.shape = ShapeHandleAccess::fromNativeShape(&child);
		outParts.push_back(std::move(part));
	}
	if (outParts.empty())
	{
		ShapeHierarchyPart part;
		part.partPath = "0";
		part.parentPartPath.clear();
		part.displayName = detail::shapeTypeName(native.ShapeType()) + "_0";
		part.shape = shape;
		outParts.push_back(std::move(part));
	}
	return true;
}

bool discretizeShapeFaceByIndex(const ShapeHandle& shapeHandle, const int faceIndex, const TessellateParams& params,
								std::vector<float>& soup, std::string* errMsg)
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
