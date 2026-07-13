#include "BackendVisualSync.h"

#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "DocumentHostEvents.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"

namespace cloudsim::host {

namespace {

using ::BackendColor;
using ::BackendDataBase;
using ::MeshBackendData;
using ::OsgWidget;
using ::PointCloudBackendData;

bool isUiOnlyKey(const QString& key)
{
	return key.isEmpty() || key.startsWith(QStringLiteral("ui."));
}

} // namespace

bool propertyKeyNeedsVisualSync(const QString& key)
{
	if (isUiOnlyKey(key))
	{
		return false;
	}
	if (key.startsWith(QStringLiteral("follow.")))
	{
		return true;
	}
	if (key.startsWith(QStringLiteral("pose.")) || key.startsWith(QStringLiteral("rotation.")))
	{
		return true;
	}
	if (key.contains(QStringLiteral("color"), Qt::CaseInsensitive)
		|| key.contains(QStringLiteral("visible"), Qt::CaseInsensitive))
	{
		return true;
	}
	return true;  // 未知 key 默认同步
}

bool propertyKeyCommitsPose(const QString& key)
{
	if (key.startsWith(QStringLiteral("follow.")))
	{
		return true;
	}
	if (key.startsWith(QStringLiteral("pose.")) || key.startsWith(QStringLiteral("rotation.")))
	{
		return true;
	}
	return key.contains(QStringLiteral("pose"), Qt::CaseInsensitive)
		|| key.contains(QStringLiteral("rotation"), Qt::CaseInsensitive);
}

void syncVisualAfterPropertyChange(DocumentHost& host, const BackendDataBase& data, const bool applyColor)
{
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		return;
	}
	const std::string backendId = data.id();
	osg->syncSelectionForBackendId(backendId);
	if (applyColor)
	{
		const BackendColor color = data.color();
		osg->applyColorToBackendObject(backendId, osg::Vec4(color.r, color.g, color.b, color.a));
		osg->requestRedraw();
		return;
	}
	if (!osg->hasBackendObjectBranch(backendId))
	{
		return;
	}
	(void)host.sceneBridge().syncOuterPatFromBackend(data);
	osg->requestRedraw();
}

void syncVisualAfterPropertyChangeById(DocumentHost& host, const QString& objectId, const bool applyColor)
{
	if (objectId.isEmpty())
	{
		return;
	}
	const auto obj = host.backend().getData(objectId.toStdString());
	if (!obj)
	{
		return;
	}
	syncVisualAfterPropertyChange(host, *obj, applyColor);
}

void afterDataServicePropertyChange(DocumentHost& host, const BackendDataBase& data, const QString& key)
{
	// 属性面板连续 spin：数据已写入，全量 OSG/Follow/PoseCommitted 延至失焦（见 MainWindow flushPropertyPanelVisualCommit）
	if (host.deferPropertyPanelVisualFullSync()
		&& (key.startsWith(QStringLiteral("pose.")) || key.startsWith(QStringLiteral("rotation."))))
	{
		return;
	}
	if (!propertyKeyNeedsVisualSync(key))
	{
		if (propertyKeyCommitsPose(key))
		{
			publishPoseCommittedFromBackend(host, data);
		}
		return;
	}
	const bool applyColor = key.contains(QStringLiteral("color"), Qt::CaseInsensitive);
	if (applyColor)
	{
		syncVisualAfterPropertyChange(host, data, true);
		return;
	}
	if (dynamic_cast<const PointCloudBackendData*>(&data) || dynamic_cast<const MeshBackendData*>(&data))
	{
		syncVisualAfterPropertyChange(host, data, false);
	}
	if (propertyKeyCommitsPose(key))
	{
		publishPoseCommittedFromBackend(host, data);
	}
}

} // namespace cloudsim::host
