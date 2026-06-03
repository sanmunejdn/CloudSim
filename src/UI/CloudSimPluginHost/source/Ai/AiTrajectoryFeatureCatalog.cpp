#include "Ai/AiTrajectoryFeatureCatalog.h"

#include "AiDomainTypes.h"

#include <FeatureSpec.h>
#include <GeometryRef.h>

#include <json.hpp>

#include <algorithm>
#include <set>

namespace
{

bool containsAny(const QString& text, const QStringList& keys)
{
	for (const QString& k : keys)
	{
		if (text.contains(k))
		{
			return true;
		}
	}
	return false;
}

void writeRefsJson(nlohmann::json& refs, const geoalgo::FeatureRefs& r)
{
	if (!r.edgeIndices.empty())
	{
		refs["edgeIndices"] = r.edgeIndices;
	}
	if (!r.faceIndices.empty())
	{
		refs["faceIndices"] = r.faceIndices;
	}
	if (r.uvCountU > 0)
	{
		refs["uvCountU"] = r.uvCountU;
	}
	if (r.uvCountV > 0)
	{
		refs["uvCountV"] = r.uvCountV;
	}
	if (r.gridAngleDeg != 0.0)
	{
		refs["gridAngleDeg"] = r.gridAngleDeg;
	}
}

nlohmann::json featureSpecToJsonObj(const geoalgo::FeatureSpec& spec)
{
	nlohmann::json j;
	j["schemaVersion"] = spec.schemaVersion;
	j["featureId"] = spec.featureId;
	j["kind"] = geoalgo::featureKindToString(spec.kind);
	nlohmann::json wp;
	wp["backendIdUtf8"] = spec.workpiece.backendIdUtf8;
	wp["stepPathUtf8"] = spec.workpiece.stepPathUtf8;
	j["workpiece"] = wp;
	nlohmann::json refs;
	writeRefsJson(refs, spec.refs);
	j["refs"] = refs;
	nlohmann::json disc;
	disc["stepMm"] = spec.discretize.stepMm;
	disc["linearDeflectionMm"] = spec.discretize.linearDeflectionMm;
	disc["outputTangent"] = spec.discretize.outputTangent;
	disc["outputNormal"] = spec.discretize.outputNormal;
	j["discretize"] = disc;
	return j;
}

bool parseCatalogJson(const QByteArray& jsonUtf8, geoalgo::FeatureCatalog& out)
{
	try
	{
		const nlohmann::json j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
		out = geoalgo::FeatureCatalog{};
		out.backendIdUtf8 = j.value("backendIdUtf8", std::string());
		out.stepPathUtf8 = j.value("stepPathUtf8", std::string());
		if (!j.contains("candidates") || !j["candidates"].is_array())
		{
			return false;
		}
		for (const auto& item : j["candidates"])
		{
			geoalgo::FeatureCandidate c;
			c.candidateId = item.value("candidateId", std::string());
			c.summary = item.value("summary", std::string());
			const std::string kindStr = item.value("suggestedKind", std::string());
			geoalgo::featureKindFromString(kindStr, c.suggestedKind);
			if (item.contains("refs") && item["refs"].is_object())
			{
				const auto& refs = item["refs"];
				if (refs.contains("edgeIndices") && refs["edgeIndices"].is_array())
				{
					for (const auto& ei : refs["edgeIndices"])
					{
						if (ei.is_number_integer())
						{
							c.refs.edgeIndices.push_back(ei.get<int>());
						}
					}
				}
				if (refs.contains("faceIndices") && refs["faceIndices"].is_array())
				{
					for (const auto& fi : refs["faceIndices"])
					{
						if (fi.is_number_integer())
						{
							c.refs.faceIndices.push_back(fi.get<int>());
						}
					}
				}
			}
			c.lengthMm = item.value("lengthMm", 0.0);
			c.areaMm2 = item.value("areaMm2", 0.0);
			c.dihedralDeg = item.value("dihedralDeg", 0.0);
			out.candidates.push_back(std::move(c));
		}
		return true;
	}
	catch (...)
	{
		return false;
	}
}

const geoalgo::FeatureCandidate* findCandidateById(const geoalgo::FeatureCatalog& catalog, const std::string& id)
{
	for (const geoalgo::FeatureCandidate& c : catalog.candidates)
	{
		if (c.candidateId == id)
		{
			return &c;
		}
	}
	return nullptr;
}

} // namespace

namespace AiTrajectoryFeatureCatalog
{

bool isLineKind(const geoalgo::FeatureKind kind)
{
	switch (kind)
	{
	case geoalgo::FeatureKind::EdgeChain:
	case geoalgo::FeatureKind::FaceIntersection:
	case geoalgo::FeatureKind::FaceBoundary:
		return true;
	default:
		return false;
	}
}

bool isSurfaceKind(const geoalgo::FeatureKind kind)
{
	switch (kind)
	{
	case geoalgo::FeatureKind::FaceUVGrid:
	case geoalgo::FeatureKind::FaceOffsetCurve:
	case geoalgo::FeatureKind::FaceBoundary:
		return true;
	default:
		return false;
	}
}

AiFeatureAxis inferFeatureAxisFromText(const QString& userText)
{
	const QString t = userText.trimmed();
	if (containsAny(t, {QStringLiteral("面特征"), QStringLiteral("大平面"), QStringLiteral("栅格"),
			QStringLiteral("打磨"), QStringLiteral("grind"), QStringLiteral("UV")})
		|| (containsAny(t, {QStringLiteral("面"), QStringLiteral("surface"), QStringLiteral("face")})
			&& !containsAny(t, {QStringLiteral("面特征")})))
	{
		return AiFeatureAxis::Surface;
	}
	if (containsAny(t, {QStringLiteral("线特征"), QStringLiteral("边"), QStringLiteral("焊缝"),
			QStringLiteral("交线"), QStringLiteral("轮廓"), QStringLiteral("涂胶"), QStringLiteral("weld"),
			QStringLiteral("glue"), QStringLiteral("seam")}))
	{
		return AiFeatureAxis::Line;
	}
	if (containsAny(t, {QStringLiteral("轨迹"), QStringLiteral("特征"), QStringLiteral("识别"),
			QStringLiteral("trajectory"), QStringLiteral("feature")}))
	{
		return AiFeatureAxis::Ambiguous;
	}
	return AiFeatureAxis::Ambiguous;
}

bool isSelectionFollowUpText(const QString& userText)
{
	const QString t = userText.trimmed();
	static const QStringList keys = {
		QStringLiteral("选"), QStringLiteral("选择"), QStringLiteral("确认"), QStringLiteral("第"),
		QStringLiteral("编号"), QStringLiteral("select"), QStringLiteral("pick"),
	};
	if (containsAny(t, keys))
	{
		return true;
	}
	for (const QChar& ch : t)
	{
		if (ch.isDigit())
		{
			return true;
		}
	}
	return false;
}

bool isAxisClarificationText(const QString& userText, AiFeatureAxis* outAxis)
{
	const QString t = userText.trimmed();
	if (containsAny(t, {QStringLiteral("线"), QStringLiteral("边"), QStringLiteral("line")}))
	{
		if (outAxis)
		{
			*outAxis = AiFeatureAxis::Line;
		}
		return true;
	}
	if (containsAny(t, {QStringLiteral("面"), QStringLiteral("surface")}))
	{
		if (outAxis)
		{
			*outAxis = AiFeatureAxis::Surface;
		}
		return true;
	}
	return false;
}

QByteArray buildCatalogSliceJson(const geoalgo::FeatureCatalog& catalog, const AiFeatureAxis axis, const int maxItems)
{
	nlohmann::json root;
	root["backendIdUtf8"] = catalog.backendIdUtf8;
	root["stepPathUtf8"] = catalog.stepPathUtf8;
	root["featureAxis"] = aiFeatureAxisToString(axis).toStdString();
	nlohmann::json arr = nlohmann::json::array();
	int displayIndex = 1;
	for (const geoalgo::FeatureCandidate& c : catalog.candidates)
	{
		if (displayIndex > maxItems)
		{
			break;
		}
		const bool lineCandidate = !c.refs.edgeIndices.empty();
		const bool surfaceCandidate = !c.refs.faceIndices.empty() && c.refs.edgeIndices.empty();
		if (axis == AiFeatureAxis::Line && !lineCandidate)
		{
			continue;
		}
		if (axis == AiFeatureAxis::Surface && !surfaceCandidate)
		{
			continue;
		}
		nlohmann::json item;
		item["displayIndex"] = displayIndex;
		item["candidateId"] = c.candidateId;
		item["suggestedKind"] = geoalgo::featureKindToString(c.suggestedKind);
		item["summary"] = c.summary;
		nlohmann::json refs;
		writeRefsJson(refs, c.refs);
		item["refs"] = refs;
		item["lengthMm"] = c.lengthMm;
		item["areaMm2"] = c.areaMm2;
		item["dihedralDeg"] = c.dihedralDeg;
		arr.push_back(item);
		++displayIndex;
	}
	root["candidates"] = arr;
	return QByteArray::fromStdString(root.dump(2));
}

QString suggestedPipelineTemplateForAxis(const AiFeatureAxis axis, const QString& userText)
{
	if (containsAny(userText, {QStringLiteral("胶"), QStringLiteral("glue")}))
	{
		return QStringLiteral("glue_default");
	}
	if (containsAny(userText, {QStringLiteral("磨"), QStringLiteral("grind")}))
	{
		return QStringLiteral("grind_default");
	}
	if (axis == AiFeatureAxis::Surface)
	{
		return QStringLiteral("grind_default");
	}
	if (containsAny(userText, {QStringLiteral("涂胶")}))
	{
		return QStringLiteral("glue_default");
	}
	return QStringLiteral("weld_default");
}

bool candidateToFeatureSpec(const geoalgo::FeatureCandidate& candidate, const std::string& backendId,
	const std::string& stepPath, geoalgo::FeatureSpec& out)
{
	out = geoalgo::FeatureSpec{};
	out.workpiece.backendIdUtf8 = backendId;
	out.workpiece.stepPathUtf8 = stepPath;
	out.featureId = candidate.candidateId;
	out.kind = candidate.suggestedKind;
	out.refs = candidate.refs;
	if (out.kind == geoalgo::FeatureKind::FaceUVGrid)
	{
		out.refs.uvCountU = 16;
		out.refs.uvCountV = 16;
		out.discretize.stepMm = 0.0;
	}
	else if (out.kind == geoalgo::FeatureKind::FaceBoundary)
	{
		out.discretize.stepMm = 2.0;
	}
	else
	{
		out.discretize.stepMm = 5.0;
	}
	out.discretize.linearDeflectionMm = 0.01;
	out.discretize.outputTangent = true;
	out.discretize.outputNormal = true;
	return true;
}

AiParseResult tryParseTrajectoryFeatureRules(const QString& userText, const AiFeatureAxis axis,
	const QByteArray& catalogSliceUtf8, const QString& backendId, const QString& stepPath)
{
	AiParseResult r;
	r.domainId = AiDomainIds::trajectoryFeature();
	r.outputKind = AiDomainOutputKind::StructuredJson;

	if (axis == AiFeatureAxis::Ambiguous)
	{
		r.ok = true;
		nlohmann::json j;
		j["version"] = 1;
		j["featureAxis"] = "ambiguous";
		j["clarifyMessage"] = "请说明需要线特征（边/焊缝/轮廓）还是面特征（平面/打磨栅格）。";
		j["features"] = nlohmann::json::array();
		j["suggestedPipelineTemplate"] = "weld_default";
		r.outputJsonUtf8 = QByteArray::fromStdString(j.dump());
		r.parserVia = QStringLiteral("Rules");
		return r;
	}

	geoalgo::FeatureCatalog sliceCatalog;
	if (!parseCatalogJson(catalogSliceUtf8, sliceCatalog))
	{
		r.ok = false;
		r.errorMessage = QStringLiteral("Catalog slice invalid.");
		r.parserVia = QStringLiteral("Rules");
		return r;
	}

	std::vector<geoalgo::FeatureSpec> specs;
	const std::string backend = backendId.toStdString();
	const std::string step = stepPath.toStdString();
	const std::string intent = userText.toUtf8().constData();

	std::vector<geoalgo::FeatureSpec> suggested;
	std::string suggestErr;
	if (geometry_backend_ops::suggestFeaturesFromCatalog(sliceCatalog, intent, suggested, &suggestErr)
		&& !suggested.empty())
	{
		specs = std::move(suggested);
	}
	else
	{
		for (const geoalgo::FeatureCandidate& c : sliceCatalog.candidates)
		{
			geoalgo::FeatureSpec spec;
			candidateToFeatureSpec(c, backend, step, spec);
			specs.push_back(std::move(spec));
			if (static_cast<int>(specs.size()) >= 8)
			{
				break;
			}
		}
	}

	if (specs.empty())
	{
		r.ok = false;
		r.errorMessage = QStringLiteral("未找到匹配的特征候选。");
		r.parserVia = QStringLiteral("Rules");
		return r;
	}

	std::vector<std::string> selectedIds;
	for (const geoalgo::FeatureSpec& s : specs)
	{
		selectedIds.push_back(s.featureId);
	}

	nlohmann::json j;
	j["version"] = 1;
	j["featureAxis"] = aiFeatureAxisToString(axis).toStdString();
	j["selectedCandidateIds"] = selectedIds;
	j["suggestedPipelineTemplate"] = suggestedPipelineTemplateForAxis(axis, userText).toStdString();
	nlohmann::json feats = nlohmann::json::array();
	for (const geoalgo::FeatureSpec& s : specs)
	{
		feats.push_back(featureSpecToJsonObj(s));
	}
	j["features"] = feats;
	r.ok = true;
	r.outputJsonUtf8 = QByteArray::fromStdString(j.dump());
	r.parserVia = QStringLiteral("Rules");
	return r;
}

bool parseDisplayIndexSelection(const QString& userText, const QByteArray& catalogSliceUtf8,
	std::vector<std::string>& outCandidateIds, QString* err)
{
	outCandidateIds.clear();

	std::set<int> indices;
	const QString t = userText;
	for (int i = 0; i < t.size(); ++i)
	{
		if (t[i].isDigit())
		{
			int val = 0;
			while (i < t.size() && t[i].isDigit())
			{
				val = val * 10 + t[i].digitValue();
				++i;
			}
			if (val > 0)
			{
				indices.insert(val);
			}
			--i;
		}
	}
	if (indices.empty())
	{
		if (err)
		{
			*err = QStringLiteral("未解析到编号，请使用如「选 1 和 3」。");
		}
		return false;
	}

	try
	{
		const nlohmann::json j = nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
		if (!j.contains("candidates") || !j["candidates"].is_array())
		{
			if (err)
			{
				*err = QStringLiteral("Catalog slice missing candidates.");
			}
			return false;
		}
		for (const auto& item : j["candidates"])
		{
			const int displayIndex = item.value("displayIndex", 0);
			if (indices.count(displayIndex) > 0)
			{
				outCandidateIds.push_back(item.value("candidateId", std::string()));
			}
		}
	}
	catch (...)
	{
		if (err)
		{
			*err = QStringLiteral("Catalog slice parse failed.");
		}
		return false;
	}

	if (outCandidateIds.empty())
	{
		if (err)
		{
			*err = QStringLiteral("编号超出当前候选范围。");
		}
		return false;
	}
	return true;
}

AiParseResult buildFeaturePlanFromCandidateIds(const std::vector<std::string>& candidateIds,
	const QByteArray& catalogFullUtf8, const QString& backendId, const QString& stepPath,
	const QString& pipelineTemplate)
{
	AiParseResult r;
	r.domainId = AiDomainIds::trajectoryFeature();
	r.outputKind = AiDomainOutputKind::StructuredJson;

	geoalgo::FeatureCatalog catalog;
	if (!parseCatalogJson(catalogFullUtf8, catalog))
	{
		r.ok = false;
		r.errorMessage = QStringLiteral("Full catalog invalid.");
		return r;
	}

	const std::string backend = backendId.toStdString();
	const std::string step = stepPath.toStdString();
	nlohmann::json j;
	j["version"] = 1;
	j["selectedCandidateIds"] = candidateIds;
	j["suggestedPipelineTemplate"] = pipelineTemplate.toStdString();
	nlohmann::json feats = nlohmann::json::array();
	for (const std::string& id : candidateIds)
	{
		const geoalgo::FeatureCandidate* c = findCandidateById(catalog, id);
		if (!c)
		{
			continue;
		}
		geoalgo::FeatureSpec spec;
		candidateToFeatureSpec(*c, backend, step, spec);
		feats.push_back(featureSpecToJsonObj(spec));
	}
	if (feats.empty())
	{
		r.ok = false;
		r.errorMessage = QStringLiteral("无法将编号映射到特征。");
		return r;
	}
	j["features"] = feats;
	r.ok = true;
	r.outputJsonUtf8 = QByteArray::fromStdString(j.dump());
	r.parserVia = QStringLiteral("Selection");
	return r;
}

} // namespace AiTrajectoryFeatureCatalog
