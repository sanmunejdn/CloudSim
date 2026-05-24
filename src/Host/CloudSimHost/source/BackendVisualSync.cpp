#include "BackendVisualSync.h"

#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "DocumentHostEvents.h"

#include "BackendDataBase.h"
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
	return true;
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
	if (!osg->hasBackendObjectBranch(backendId))
	{
		return;
	}
	(void)host.sceneBridge().syncOuterPatFromBackend(data);
	if (applyColor)
	{
		const BackendColor color = data.color();
		osg->setSelectedColor(color.r, color.g, color.b, color.a);
	}
	osg->requestRedraw();
}

void afterDataServicePropertyChange(DocumentHost& host, const BackendDataBase& data, const QString& key)
{
	if (!propertyKeyNeedsVisualSync(key))
	{
		if (propertyKeyCommitsPose(key))
		{
			publishPoseCommittedFromBackend(host, data);
		}
		return;
	}
	const bool applyColor = key.contains(QStringLiteral("color"), Qt::CaseInsensitive);
	if (dynamic_cast<const PointCloudBackendData*>(&data) || dynamic_cast<const MeshBackendData*>(&data))
	{
		syncVisualAfterPropertyChange(host, data, applyColor);
	}
	if (propertyKeyCommitsPose(key))
	{
		publishPoseCommittedFromBackend(host, data);
	}
}

} // namespace cloudsim::host
