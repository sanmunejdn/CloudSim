/// @file FaceOffsetCurveDiscretizer.cpp
/// @brief FaceOffsetCurveDiscretizer 实现

#include "FaceOffsetCurveDiscretizer.h"

#include "Discretize.h"
#include "FeatureDiscretizeParamUtils.h"
#include "FeatureDiscretizerRegistry.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeCommon.h"
#include "detail/OccIncludes.h"

#include <BRepAdaptor_Surface.hxx>

namespace geoalgo
{
REGISTER_FEATURE_DISCRETIZER(FaceOffsetCurveDiscretizer);

std::vector<FeatureDiscretizerParamField> FaceOffsetCurveDiscretizer::paramFields() const
{
	std::vector<FeatureDiscretizerParamField> fields = featureDiscretizerCommonParamFields();
	fields.push_back(doubleFeatureParamField("offsetMm", "Offset", "偏置距离", "mm", -1000.0, 1000.0, 0.01, 0.0, 10));
	return fields;
}

bool FaceOffsetCurveDiscretizer::validate(const FeatureDiscretizeInput& input, std::string* errMsg) const
{
	if (input.strategyId != strategyId())
	{
		if (errMsg)
		{
			*errMsg = "strategyId mismatch";
		}
		return false;
	}
	if (input.geometry.faceIndices.empty() || input.geometry.edgeIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "FaceOffsetCurve requires faceIndices and edgeIndices";
		}
		return false;
	}
	return true;
}

bool FaceOffsetCurveDiscretizer::discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input,
											RawPath& out, std::string* errMsg) const
{
	if (!validate(input, errMsg))
	{
		return false;
	}

	TopoDS_Face face;
	TopoDS_Edge edge;
	if (!shapeFaceAtIndex(shape, input.geometry.faceIndices[0], face, errMsg) ||
		!shapeEdgeAtIndex(shape, input.geometry.edgeIndices[0], edge, errMsg))
	{
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(input);
	Polyline3d poly;
	if (!discretizeEdge(edge, detail::toTessellate(disc), poly, errMsg))
	{
		return false;
	}

	const BRepAdaptor_Surface surf(face);
	const double offset = paramDouble(input.params, "offsetMm", 0.0);
	Polyline3d offsetPoly;
	const std::size_t n = poly.xyz.size() / 3U;
	offsetPoly.xyz.reserve(poly.xyz.size());
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		gp_Pnt p(poly.xyz[b], poly.xyz[b + 1U], poly.xyz[b + 2U]);
		GeomAPI_ProjectPointOnSurf proj(p, BRep_Tool::Surface(face));
		if (proj.NbPoints() < 1)
		{
			continue;
		}
		Standard_Real u = 0.0;
		Standard_Real v = 0.0;
		proj.LowerDistanceParameters(u, v);
		gp_Pnt onS;
		gp_Vec du;
		gp_Vec dv;
		surf.D1(u, v, onS, du, dv);
		gp_Vec nrm = du.Crossed(dv);
		if (face.Orientation() == TopAbs_REVERSED)
		{
			nrm.Reverse();
		}
		if (nrm.Magnitude() < 1e-12)
		{
			continue;
		}
		nrm.Normalize();
		p.Translate(nrm.Multiplied(offset));
		offsetPoly.xyz.push_back(static_cast<float>(p.X()));
		offsetPoly.xyz.push_back(static_cast<float>(p.Y()));
		offsetPoly.xyz.push_back(static_cast<float>(p.Z()));
	}

	const std::vector<TopoDS_Face> contextFaces{face};
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = &contextFaces;
	frameCtx.normalConvention = detail::FaceNormalConvention::LineReverseFace;
	detail::appendPolylineToRawPath(offsetPoly, out, disc, true, frameCtx);

	detail::applyPostDiscretizeResample(strategyId(), input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
