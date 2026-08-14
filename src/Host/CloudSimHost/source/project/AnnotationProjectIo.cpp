/// @file AnnotationProjectIo.cpp
/// @brief AnnotationProjectIo 实现

#include "AnnotationProjectIo.h"

#include "OsgWidget.h"

#include <osg/Vec3f>

namespace cloudsim::host
{
using ::OsgWidget;

QJsonArray buildAnnotationsJsonFromOsg(OsgWidget& osgWidget, QJsonObject& inOutRootExtras)
{
	QJsonArray annArray;
	const auto snapshots = osgWidget.annotationSnapshots();
	for (const auto& s : snapshots)
	{
		QJsonObject a;
		a.insert(QStringLiteral("id"), s.id);
		a.insert(QStringLiteral("displayText"), s.displayText);
		a.insert(QStringLiteral("backendId"), s.backendId);
		QJsonObject local;
		local.insert(QStringLiteral("x"), s.localCentered.x());
		local.insert(QStringLiteral("y"), s.localCentered.y());
		local.insert(QStringLiteral("z"), s.localCentered.z());
		a.insert(QStringLiteral("localCentered"), local);
		if (s.hasWorldAnchor)
		{
			QJsonObject w;
			w.insert(QStringLiteral("x"), s.worldAnchor.x());
			w.insert(QStringLiteral("y"), s.worldAnchor.y());
			w.insert(QStringLiteral("z"), s.worldAnchor.z());
			a.insert(QStringLiteral("worldAnchor"), w);
		}
		a.insert(QStringLiteral("visible"), s.visible);
		annArray.push_back(a);
	}
	const QString camFollow = QString::fromStdString(osgWidget.cameraFollowBackendId());
	if (!camFollow.isEmpty())
	{
		inOutRootExtras.insert(QStringLiteral("cameraFollowBackendId"), camFollow);
	}
	return annArray;
}

void applyAnnotationsFromProjectJson(OsgWidget& osgWidget, const QJsonObject& root)
{
	QList<OsgWidget::AnnotationSnapshot> snapshots;
	const QJsonArray annArray = root.value(QStringLiteral("annotations")).toArray();
	for (const QJsonValue& v : annArray)
	{
		if (!v.isObject())
		{
			continue;
		}
		const QJsonObject a = v.toObject();
		OsgWidget::AnnotationSnapshot s;
		s.id = a.value(QStringLiteral("id")).toString();
		s.displayText = a.value(QStringLiteral("displayText")).toString();
		s.backendId = a.value(QStringLiteral("backendId")).toString();
		const QJsonObject local = a.value(QStringLiteral("localCentered")).toObject();
		s.localCentered = osg::Vec3f(static_cast<float>(local.value(QStringLiteral("x")).toDouble()),
									 static_cast<float>(local.value(QStringLiteral("y")).toDouble()),
									 static_cast<float>(local.value(QStringLiteral("z")).toDouble()));
		const QJsonObject world = a.value(QStringLiteral("worldAnchor")).toObject();
		if (!world.isEmpty())
		{
			s.worldAnchor = osg::Vec3f(static_cast<float>(world.value(QStringLiteral("x")).toDouble()),
									   static_cast<float>(world.value(QStringLiteral("y")).toDouble()),
									   static_cast<float>(world.value(QStringLiteral("z")).toDouble()));
			s.hasWorldAnchor = true;
		}
		s.visible = a.value(QStringLiteral("visible")).toBool(true);
		snapshots.push_back(s);
	}
	osgWidget.restoreAnnotations(snapshots);
	osgWidget.setCameraFollowBackendId(root.value(QStringLiteral("cameraFollowBackendId")).toString().toStdString());
}

} // namespace cloudsim::host
