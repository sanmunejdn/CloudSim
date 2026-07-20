#ifndef LABELINGPLUGIN_LABELINGCONFIGIO_H
#define LABELINGPLUGIN_LABELINGCONFIGIO_H

/// @file LabelingConfigIO.h
/// @brief 解析 plugins/com.cloudsim.labeling/labeling_config.json（不存在则返回默认写入路径）

#include <QString>

struct LabelingPluginConfig
{
	QString configFilePath;
	QString pythonExecutable;
	QString datasetRoot;
	QString trainingRoot;
	QString defaultSegConfig;
	QString deployOnnxRel;
	QString deployConfigRel;
};

/// 解析 plugins/com.cloudsim.labeling/labeling_config.json（不存在则返回默认写入路径）
QString resolveLabelingConfigFilePath();

LabelingPluginConfig loadLabelingPluginConfig();

bool saveLabelingPluginConfig(const LabelingPluginConfig& cfg);

#endif // LABELINGPLUGIN_LABELINGCONFIGIO_H
