#ifndef CLOUDSIMPLUGINHOST_AITRAJECTORYFEATURECATALOG_H
#define CLOUDSIMPLUGINHOST_AITRAJECTORYFEATURECATALOG_H

/// @file AiTrajectoryFeatureCatalog.h
/// @brief trajectory.feature 规则解析、catalog 切片与编号映射

#include "AiParseTypes.h"
#include "AiTrajectoryFeatureTypes.h"

#include <QByteArray>
#include <QString>
#include <string>
#include <vector>

namespace geoalgo
{
struct FeatureCatalog;
struct FeatureCandidate;
struct FeatureEntry;
struct FeatureListDocument;
} // namespace geoalgo

/// trajectory.feature 规则解析、catalog 切片与编号映射
namespace AiTrajectoryFeatureCatalog
{
AiFeatureAxis inferFeatureAxisFromText(const QString& userText);

bool isSelectionFollowUpText(const QString& userText);

bool isAxisClarificationText(const QString& userText, AiFeatureAxis* outAxis);

/// 从全量 catalog 按轴切片，写入 displayIndex 1..N；maxItems<=0 表示不截断
QByteArray buildCatalogSliceJson(const geoalgo::FeatureCatalog& catalog, AiFeatureAxis axis, int maxItems = 0);

QString suggestedPipelineTemplateForAxis(AiFeatureAxis axis, const QString& userText);

/// rules 路径：基于 catalog 切片 + 用户意图生成 trajectory.feature JSON
AiParseResult tryParseTrajectoryFeatureRules(const QString& userText, AiFeatureAxis axis,
											 const QByteArray& catalogSliceUtf8, const QString& backendId,
											 const QString& stepPath);

/// 解析「选 1、3」等编号为 candidateId 列表（displayIndex 1-based）
bool parseDisplayIndexSelection(const QString& userText, const QByteArray& catalogSliceUtf8,
								std::vector<std::string>& outCandidateIds, QString* err);

AiParseResult buildFeaturePlanFromCandidateIds(const std::vector<std::string>& candidateIds,
											   const QByteArray& catalogFullUtf8, const QString& backendId,
											   const QString& stepPath, const QString& pipelineTemplate);

bool candidateToFeatureEntry(const geoalgo::FeatureCandidate& candidate, const std::string& backendId,
							 const std::string& stepPath, geoalgo::FeatureEntry& out);

bool isLineStrategy(const std::string& strategyId);
bool isSurfaceStrategy(const std::string& strategyId);

} // namespace AiTrajectoryFeatureCatalog

#endif // CLOUDSIMPLUGINHOST_AITRAJECTORYFEATURECATALOG_H
