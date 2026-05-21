#pragma once

#include <QString>

class OsgWidget;

/// 从文件导入模型或点云到 OSG 暂存/预览场景（staging），与后端数据绑定前的预览加载。
class OsgWidgetImportController
{
public:
	bool importModelFile(OsgWidget& self, const QString& filePath, QString* errorMessage);
	bool importPointCloudFile(OsgWidget& self, const QString& filePath, QString* errorMessage);
};

