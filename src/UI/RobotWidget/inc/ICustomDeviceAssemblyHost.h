#ifndef ROBOTWIDGET_ICUSTOMDEVICEASSEMBLYHOST_H
#define ROBOTWIDGET_ICUSTOMDEVICEASSEMBLYHOST_H

/// @file ICustomDeviceAssemblyHost.h
/// @brief 组装对话框所需宿主能力（Widget 实现，避免 Dialog 依赖 CloudSimHost）

#include "robotwidget_global.h"

#include <functional>
#include <memory>

#include <QString>
#include <QStringList>
#include <QWidget>

class CustomDeviceBackendData;
class IRobotDocumentHost;

class ROBOTWIDGET_EXPORT ICustomDeviceAssemblyHost
{
public:
	virtual ~ICustomDeviceAssemblyHost() = default;

	virtual bool useChinese() const = 0;
	virtual QString i18n(const QString& en, const QString& zh) const = 0;
	virtual IRobotDocumentHost* document() = 0;

	virtual bool registerCustomDevice(const std::shared_ptr<CustomDeviceBackendData>& device, QString* outError) = 0;
	virtual bool attachChildToCustomDevice(const std::string& deviceId, const std::string& childId,
										   QString* outError) = 0;
	/// 导入模型并返回各文件根 backendId；失败项写入 outErrors
	virtual QStringList importModelsForAssembly(QWidget* parent, const QStringList& paths, QStringList* outErrors) = 0;
	/// 进入视口点面抽 Solid；可连点多次，关闭对话框须 end
	virtual void beginPickSolidInView(std::function<void(const QString& partId)> onPartPicked) = 0;
	virtual void endPickSolidInView() = 0;
	virtual bool exportCustomDeviceUrdfInteractive(const QString& deviceBackendId) = 0;

	virtual void markFollowAttachmentDirty(const QString& deviceBackendId) = 0;
	virtual void refreshBackendTree() = 0;
	virtual void focusBackendInTree(const QString& backendId) = 0;
	virtual void runFollowSolveAndSync() = 0;
	virtual void appendRunInfo(const QString& message) = 0;
	/// Apply 成功后：刷新轴目标并选中该设备
	virtual void onCustomDeviceAssemblyCommitted(const QString& deviceBackendId) = 0;
};

#endif // ROBOTWIDGET_ICUSTOMDEVICEASSEMBLYHOST_H
