/// @file BackendVisualSync.cpp
/// @brief 后端到 OSG 视觉同步

#include "BackendVisualSync.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendPropertyVisualAspect.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "DocumentHostEvents.h"
#include "OsgWidget.h"
#include "visual/VisualAspect.h"

namespace cloudsim::host
{
namespace
{
using ::BackendDataBase;
using ::OsgWidget;

VisualAspect aspectsFromSchemaBits(const std::uint32_t bits)
{
	return static_cast<VisualAspect>(bits);
}

} // namespace

bool propertyKeyNeedsVisualSync(const QString& key)
{
	if (key.isEmpty() || key.startsWith(QStringLiteral("ui.")))
	{
		return false;
	}
	if (key.startsWith(QStringLiteral("follow.")))
	{
		return false;
	}
	return true;
}

bool propertyKeyCommitsPose(const QString& key)
{
	return backend_property_schema::propertyCommitsPoseFromSchema(std::string(), key.toStdString()) ||
		   key.startsWith(QStringLiteral("follow."));
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
		host.markVisualDirty(backendId, VisualAspect::Appearance);
		(void)host.flushVisualSync();
		return;
	}
	if (!osg->hasBackendObjectBranch(backendId))
	{
		return;
	}
	host.markVisualDirty(backendId, VisualAspect::Transform);
	(void)host.flushVisualSync();
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
	if (host.deferPropertyPanelVisualFullSync() &&
		(key.startsWith(QStringLiteral("pose.")) || key.startsWith(QStringLiteral("rotation."))))
	{
		return;
	}
	const std::uint32_t aspectBits =
		backend_property_schema::visualAspectsForPropertyKey(data.className(), key.toStdString());
	const VisualAspect aspects = aspectsFromSchemaBits(aspectBits);
	if (aspects == VisualAspect::None && !key.startsWith(QStringLiteral("follow.")))
	{
		if (propertyKeyCommitsPose(key))
		{
			publishPoseCommittedFromBackend(host, data);
		}
		return;
	}
	if (hasVisualAspect(aspects, VisualAspect::Appearance))
	{
		host.markVisualDirty(data.id(), VisualAspect::Appearance);
	}
	if (hasVisualAspect(aspects, VisualAspect::Visibility))
	{
		host.markVisualDirty(data.id(), VisualAspect::Visibility);
	}
	if (hasVisualAspect(aspects, VisualAspect::Transform))
	{
		host.markVisualDirty(data.id(), VisualAspect::Transform);
	}
	if (hasVisualAspect(aspects, VisualAspect::Geometry))
	{
		host.markVisualDirty(data.id(), VisualAspect::Geometry);
	}
	if (!host.deferPropertyPanelVisualFullSync())
	{
		(void)host.flushVisualSync();
	}
	if (propertyKeyCommitsPose(key))
	{
		publishPoseCommittedFromBackend(host, data);
	}
}

} // namespace cloudsim::host
