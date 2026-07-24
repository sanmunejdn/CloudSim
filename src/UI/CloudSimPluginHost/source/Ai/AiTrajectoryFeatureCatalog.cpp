/// @file AiTrajectoryFeatureCatalog.cpp
/// @brief AiTrajectoryFeatureCatalog 实现

#include "Ai/AiTrajectoryFeatureCatalog.h"

#include "AiDomainTypes.h"

#include <algorithm>
#include <set>

#include <FeatureListDocument.h>
#include <GeometryRef.h>
#include <json.hpp>

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

void writeGeometryJson(nlohmann::json& geometry, const geoalgo::FeatureGeometry& g)
{
	if (!g.edgeIndices.empty())
	{
		geometry["edgeIndices"] = g.edgeIndices;
	}
	if (!g.faceIndices.empty())
	{
		geometry["faceIndices"] = g.faceIndices;
	}
}

nlohmann::json featureEntryToJsonObj(const geoalgo::FeatureEntry& entry)
{
	nlohmann::json j;
	j["featureId"] = entry.featureId;
	j["strategyId"] = entry.strategyId;
	nlohmann::json geometry;
	writeGeometryJson(geometry, entry.geometry);
	j["geometry"] = geometry;
	j["params"] = entry.params;
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
			c.suggestedStrategyId = item.value("suggestedStrategyId", std::string("EdgeChain"));
			if (item.contains("geometry") && item["geometry"].is_object())
			{
				const auto& geometry = item["geometry"];
				if (geometry.contains("edgeIndices") && geometry["edgeIndices"].is_array())
				{
					for (const auto& ei : geometry["edgeIndices"])
					{
						if (ei.is_number_integer())
						{
							c.geometry.edgeIndices.push_back(ei.get<int>());
						}
					}
				}
				if (geometry.contains("faceIndices") && geometry["faceIndices"].is_array())
				{
					for (const auto& fi : geometry["faceIndices"])
					{
						if (fi.is_number_integer())
						{
							c.geometry.faceIndices.push_back(fi.get<int>());
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
bool isLineStrategy(const std::string& strategyId)
{
	return strategyId == "EdgeChain" || strategyId == "SyntheticPolyline";
}

bool isSurfaceStrategy(const std::string& strategyId)
{
	return strategyId == "FaceSection" || strategyId == "FaceParamSurface" || strategyId == "FaceOffsetCurve" ||
		   strategyId == "FaceBoundary" || strategyId == "FaceIntersection";
}

AiFeatureAxis inferFeatureAxisFromText(const QString& userText)
{
	const QString t = userText.trimmed();
	if (containsAny(t, {QStringLiteral("面特征"), QStringLiteral("大平面"), QStringLiteral("栅格"),
						QStringLiteral("打磨"), QStringLiteral("grind"), QStringLiteral("UV")}) ||
		(containsAny(t, {QStringLiteral("面"), QStringLiteral("surface"), QStringLiteral("face")}) &&
		 !containsAny(t, {QStringLiteral("面特征")})))
	{
		return AiFeatureAxis::Surface;
	}
	if (containsAny(t, {QStringLiteral("线特征"), QStringLiteral("边"), QStringLiteral("焊缝"), QStringLiteral("交线"),
						QStringLiteral("轮廓"), QStringLiteral("涂胶"), QStringLiteral("weld"), QStringLiteral("glue"),
						QStringLiteral("seam")}))
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
		QStringLiteral("选"),	QStringLiteral("选择"),	  QStringLiteral("确认"), QStringLiteral("第"),
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
		const bool lineCandidate = !c.geometry.edgeIndices.empty();
		const bool surfaceCandidate = !c.geometry.faceIndices.empty() && c.geometry.edgeIndices.empty();
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
		item["suggestedStrategyId"] = c.suggestedStrategyId;
		item["summary"] = c.summary;
		nlohmann::json geometry;
		writeGeometryJson(geometry, c.geometry);
		item["geometry"] = geometry;
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

bool candidateToFeatureEntry(const geoalgo::FeatureCandidate& candidate, const std::string& backendId,
							 const std::string& stepPath, geoalgo::FeatureEntry& out)
{
	(void)backendId;
	(void)stepPath;
	out = geoalgo::FeatureEntry{};
	out.featureId = candidate.candidateId;
	out.strategyId = candidate.suggestedStrategyId;
	out.geometry = candidate.geometry;
	out.params = geometry_backend_ops::featureDiscretizerDefaultParams(out.strategyId);
	if (out.strategyId == "FaceSection")
	{
		out.params["stepMm"] = 10.0;
		out.params["linearDeflectionMm"] = 1.0;
		out.params["uvCountU"] = 3;
	}
	else if (out.strategyId == "FaceParamSurface")
	{
		out.params["stepMm"] = 10.0;
		out.params["colSpacingMm"] = 1.0;
		out.params["linearDeflectionMm"] = 1.0;
	}
	else if (out.strategyId == "FaceBoundary")
	{
		out.params["stepMm"] = 2.0;
	}
	else
	{
		out.params["stepMm"] = 5.0;
	}
	out.params["linearDeflectionMm"] = 0.01;
	out.params["outputTangent"] = true;
	out.params["outputNormal"] = true;
	return true;
}

AiParseResult tryParseTrajectoryFeatureRules(const QString& userText, const AiFeatureAxis axis,
											 const QByteArray& catalogSliceUtf8, const QString& backendId,
											 const QString& stepPath)
{
	AiParseResult r;
	r.domainId = AiDomainIds::trajectoryFeature();
	r.outputKind = AiDomainOutputKind::StructuredJson;

	if (axis == AiFeatureAxis::Ambiguous)
	{
		r.ok = true;
		nlohmann::json j;
		j["version"] = 2;
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

	const std::string backend = backendId.toStdString();
	const std::string step = stepPath.toStdString();
	const std::string intent = userText.toUtf8().constData();

	geoalgo::FeatureListDocument suggestedDoc;
	std::string suggestErr;
	std::vector<geoalgo::FeatureEntry> entries;
	if (geometry_backend_ops::suggestFeaturesFromCatalog(sliceCatalog, intent, suggestedDoc, &suggestErr) &&
		!suggestedDoc.features.empty())
	{
		entries = suggestedDoc.features;
	}
	else
	{
		for (const geoalgo::FeatureCandidate& c : sliceCatalog.candidates)
		{
			geoalgo::FeatureEntry entry;
			candidateToFeatureEntry(c, backend, step, entry);
			entries.push_back(std::move(entry));
			if (static_cast<int>(entries.size()) >= 8)
			{
				break;
			}
		}
	}

	if (entries.empty())
	{
		r.ok = false;
		r.errorMessage = QStringLiteral("未找到匹配的特征候选。");
		r.parserVia = QStringLiteral("Rules");
		return r;
	}

	std::vector<std::string> selectedIds;
	for (const geoalgo::FeatureEntry& e : entries)
	{
		selectedIds.push_back(e.featureId);
	}

	nlohmann::json j;
	j["version"] = 2;
	j["schemaVersion"] = 2;
	j["featureAxis"] = aiFeatureAxisToString(axis).toStdString();
	j["selectedCandidateIds"] = selectedIds;
	j["suggestedPipelineTemplate"] = suggestedPipelineTemplateForAxis(axis, userText).toStdString();
	nlohmann::json wp;
	wp["backendIdUtf8"] = backend;
	wp["stepPathUtf8"] = step;
	j["workpiece"] = wp;
	nlohmann::json feats = nlohmann::json::array();
	for (const geoalgo::FeatureEntry& e : entries)
	{
		feats.push_back(featureEntryToJsonObj(e));
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
											   const QByteArray& catalogFullUtf8, const QString& backendId,
											   const QString& stepPath, const QString& pipelineTemplate)
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
	j["version"] = 2;
	j["schemaVersion"] = 2;
	j["selectedCandidateIds"] = candidateIds;
	j["suggestedPipelineTemplate"] = pipelineTemplate.toStdString();
	nlohmann::json wp;
	wp["backendIdUtf8"] = backend;
	wp["stepPathUtf8"] = step;
	j["workpiece"] = wp;
	nlohmann::json feats = nlohmann::json::array();
	for (const std::string& id : candidateIds)
	{
		const geoalgo::FeatureCandidate* c = findCandidateById(catalog, id);
		if (!c)
		{
			continue;
		}
		geoalgo::FeatureEntry entry;
		candidateToFeatureEntry(*c, backend, step, entry);
		feats.push_back(featureEntryToJsonObj(entry));
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
