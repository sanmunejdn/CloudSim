#include "FaceParamSurfaceDiscretizer.h"

#include "FeatureDiscretizerRegistry.h"
#include "detail/ParamSurfaceDiscretize.h"

namespace geoalgo
{

REGISTER_FEATURE_DISCRETIZER(FaceParamSurfaceDiscretizer);

std::vector<FeatureDiscretizerParamField> FaceParamSurfaceDiscretizer::paramFields() const
{
	std::vector<FeatureDiscretizerParamField> fields = featureDiscretizerCommonParamFields();
	fields.push_back(doubleFeatureParamField(
		"colSpacingMm", "Column spacing", "列间距", "mm", 0.1, 1000.0, 0.1, 1.0, 10, "scan"));
	fields.push_back(doubleFeatureParamField(
		"gridAngleDeg", "Grid angle", "扫描偏角", "deg", -360.0, 360.0, 1.0, 0.0, 11, "scan"));
	fields.push_back(doubleFeatureParamField(
		"trackStartPct", "Track start", "轨迹起点%", "%", 0.0, 100.0, 1.0, 0.0, 12, "scan"));
	fields.push_back(doubleFeatureParamField(
		"trackEndPct", "Track end", "轨迹终点%", "%", 0.0, 100.0, 1.0, 100.0, 13, "scan"));
	fields.push_back(boolFeatureParamField("unifySameBasis", "Unify same basis", "同母面统一域", true, 14, "scan"));
	fields.push_back(enumFeatureParamField(
		"heteroCombineMode",
		"Hetero combine",
		"异母拼接",
		{"Auto", "RowStitch", "PerFace"},
		{"自动", "逐行拼接", "逐面"},
		{"Auto", "RowStitch", "PerFace"},
		0,
		15,
		"scan"));
	fields.push_back(enumFeatureParamField(
		"edgeDiscretizeMode",
		"Edge mode",
		"边离散模式",
		{"Uniform", "ChordHeight"},
		{"均匀", "等弦高"},
		{"Uniform", "ChordHeight"},
		0,
		16,
		"scan"));
	fields.push_back(enumFeatureParamField(
		"trajConnectMode",
		"Connect mode",
		"行间连接",
		{"Bow", "Z"},
		{"弓", "Z"},
		{"Bow", "Z"},
		0,
		17,
		"scan"));
	fields.push_back(boolFeatureParamField("reverseLayer", "Reverse row", "反转行", false, 18, "scan"));
	return fields;
}

bool FaceParamSurfaceDiscretizer::discretize(
	const TopoDS_Shape& shape,
	const FeatureDiscretizeInput& input,
	RawPath& out,
	std::string* errMsg) const
{
	if (!validate(input, errMsg))
	{
		return false;
	}
	return discretizeFaceParamSurface(shape, input, out, errMsg);
}

} // namespace geoalgo
