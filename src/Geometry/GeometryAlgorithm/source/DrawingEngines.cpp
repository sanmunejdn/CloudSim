/// @file DrawingEngines.cpp
/// @brief Exact(HLRBRep_Algo) / Mesh(PolyAlgo) 投影引擎

#include "DrawingEngines.h"

#include <BRepLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <HLRBRep_PolyAlgo.hxx>
#include <HLRBRep_PolyHLRToShape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>

#include <algorithm>
#include <cmath>

namespace geoalgo
{
namespace drawing_engines
{
namespace
{

void buildCurves3dIfNeeded(TopoDS_Shape& shape)
{
	if (!shape.IsNull())
		BRepLib::BuildCurves3d(shape);
}

void appendCompoundEntities(TopoDS_Shape compound, DrawingEdgeClass cls, bool hidden,
							const TessellateParams& params, std::vector<DrawingEntity>& ents)
{
	if (compound.IsNull())
		return;
	buildCurves3dIfNeeded(compound);
	for (TopExp_Explorer exp(compound, TopAbs_EDGE); exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		DrawingEntity ent;
		if (!drawingEntityFromEdge(edge, cls, hidden, params, ent))
			continue;
		ents.push_back(std::move(ent));
	}
}

template <typename ToShapeT>
void extractEdgeSlots(ToShapeT& toShape, const TessellateParams& params, std::vector<DrawingEntity>& ents)
{
	// 锐边+轮廓；光滑/接缝默认不取（与轮廓重叠致多线）
	// 不用 OutLineVCompound3d：那是世界系边，XY 混入会飘线
	appendCompoundEntities(toShape.VCompound(), DrawingEdgeClass::Sharp, false, params, ents);
	appendCompoundEntities(toShape.OutLineVCompound(), DrawingEdgeClass::Outline, false, params, ents);
	appendCompoundEntities(toShape.HCompound(), DrawingEdgeClass::Sharp, true, params, ents);
	appendCompoundEntities(toShape.OutLineHCompound(), DrawingEdgeClass::Outline, true, params, ents);
}

} // namespace

bool extractExactHlrEntities(const TopoDS_Shape& shape, const gp_Ax2& viewAx, int nbIso,
							 const TessellateParams& params, std::vector<DrawingEntity>& out, std::string* errMsg)
{
	out.clear();
	if (shape.IsNull())
	{
		if (errMsg)
			*errMsg = "HLR exact: null shape";
		return false;
	}
	try
	{
		Handle(HLRBRep_Algo) hlr = new HLRBRep_Algo();
		hlr->Add(shape, (std::max)(0, nbIso));
		hlr->Projector(HLRAlgo_Projector(viewAx));
		hlr->Update();
		hlr->Hide();
		HLRBRep_HLRToShape toShape(hlr);
		out.reserve(256);
		extractEdgeSlots(toShape, params, out);
	}
	catch (const Standard_Failure& e)
	{
		if (errMsg)
			*errMsg = std::string("HLR exact: ") + (e.GetMessageString() ? e.GetMessageString() : "OCC failure");
		return false;
	}
	catch (...)
	{
		if (errMsg)
			*errMsg = "HLR exact: unknown failure";
		return false;
	}
	return !out.empty();
}

bool extractMeshHlrEntities(const TopoDS_Shape& shape, const gp_Ax2& viewAx, const TessellateParams& params,
							std::vector<DrawingEntity>& out, std::string* errMsg)
{
	out.clear();
	if (shape.IsNull())
	{
		if (errMsg)
			*errMsg = "HLR mesh: null shape";
		return false;
	}
	try
	{
		// PolyAlgo 要求整形体已剖分；用较粗弦高换预览速度
		TopoDS_Shape meshed = shape;
		const double defl =
			params.linearDeflectionRelative
				? (std::max)(0.1, params.linearDeflectionMm * 50.0)
				: (std::max)(0.1, params.linearDeflectionMm * 2.0);
		BRepMesh_IncrementalMesh(meshed, defl);

		Handle(HLRBRep_PolyAlgo) hlr = new HLRBRep_PolyAlgo();
		hlr->Load(meshed);
		hlr->Projector(HLRAlgo_Projector(viewAx));
		hlr->Update();

		HLRBRep_PolyHLRToShape toShape;
		toShape.Update(hlr);
		out.reserve(256);
		extractEdgeSlots(toShape, params, out);
	}
	catch (const Standard_Failure& e)
	{
		if (errMsg)
			*errMsg = std::string("HLR mesh: ") + (e.GetMessageString() ? e.GetMessageString() : "OCC failure");
		return false;
	}
	catch (...)
	{
		if (errMsg)
			*errMsg = "HLR mesh: unknown failure";
		return false;
	}
	return !out.empty();
}

} // namespace drawing_engines
} // namespace geoalgo
