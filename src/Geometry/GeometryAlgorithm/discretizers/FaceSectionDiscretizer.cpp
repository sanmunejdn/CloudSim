#include "FaceSectionDiscretizer.h"

#include "FeatureDiscretizerRegistry.h"
#include "detail/FaceSectionDiscretize.h"

namespace geoalgo
{

REGISTER_FEATURE_DISCRETIZER(FaceSectionDiscretizer);

std::vector<FeatureDiscretizerParamField> FaceSectionDiscretizer::paramFields() const
{
	std::vector<FeatureDiscretizerParamField> fields = featureDiscretizerCommonParamFields();
	fields.push_back(doubleFeatureParamField(
		"sectionOriginX", "Section origin X", "截面原点 X", "mm", -1e6, 1e6, 0.1, 0.0, 10, "section"));
	fields.push_back(doubleFeatureParamField(
		"sectionOriginY", "Section origin Y", "截面原点 Y", "mm", -1e6, 1e6, 0.1, 0.0, 11, "section"));
	fields.push_back(doubleFeatureParamField(
		"sectionOriginZ", "Section origin Z", "截面原点 Z", "mm", -1e6, 1e6, 0.1, 0.0, 12, "section"));
	fields.push_back(doubleFeatureParamField(
		"sectionRxDeg", "Section RX", "截面绕 X", "deg", -360.0, 360.0, 1.0, 0.0, 13, "section"));
	fields.push_back(doubleFeatureParamField(
		"sectionRyDeg", "Section RY", "截面绕 Y", "deg", -360.0, 360.0, 1.0, 0.0, 14, "section"));
	fields.push_back(intFeatureParamField("uvCountU", "Uniform segments", "均匀段数", 2, 9999, 3, 15, "section"));
	fields.push_back(enumFeatureParamField(
		"edgeDiscretizeMode",
		"Edge mode",
		"边离散模式",
		{"Uniform", "ChordHeight"},
		{"均匀", "等弦高"},
		{"Uniform", "ChordHeight"},
		0,
		16,
		"section"));
	fields.push_back(enumFeatureParamField(
		"trajConnectMode",
		"Connect mode",
		"层间连接",
		{"Bow", "Z", "Zhi"},
		{"弓", "Z", "之"},
		{"Bow", "Z", "Zhi"},
		0,
		17,
		"section"));
	fields.push_back(boolFeatureParamField("reverseLayer", "Reverse layer", "反转层", false, 18, "section"));
	fields.push_back(intFeatureParamField("layerKeepStart", "Layer keep start", "层保留起点", 0, 9999, 0, 19, "section"));
	fields.push_back(intFeatureParamField("layerKeepEnd", "Layer keep end", "层保留终点", 0, 9999, 0, 20, "section"));
	return fields;
}

bool FaceSectionDiscretizer::discretize(
	const TopoDS_Shape& shape,
	const FeatureDiscretizeInput& input,
	RawPath& out,
	std::string* errMsg) const
{
	if (!validate(input, errMsg))
	{
		return false;
	}
	return discretizeFaceSectionGrid(shape, input, out, errMsg);
}

} // namespace geoalgo
