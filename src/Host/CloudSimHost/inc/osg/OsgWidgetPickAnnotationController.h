#ifndef CLOUDSIMHOST_OSGWIDGETPICKANNOTATIONCONTROLLER_H
#define CLOUDSIMHOST_OSGWIDGETPICKANNOTATIONCONTROLLER_H

/// @file OsgWidgetPickAnnotationController.h
/// @brief 点选标记与三维文字标注的增删改、刷新与按后端恢复，从 OsgWidget 中抽出的标注逻辑。

#include "OsgWidget.h"

#include <QList>
#include <QString>

/// 点选标记与三维文字标注的增删改、刷新与按后端恢复，从 OsgWidget 中抽出的标注逻辑。
class OsgWidgetPickAnnotationController
{
public:
	void updatePointPickMarker(OsgWidget& self, const osg::Vec3f& pointWorld, bool hit);
	void clearPointPickMarker(OsgWidget& self);

	void addPointAnnotation(OsgWidget& self, const osg::Vec3f& pointWorld);
	void addPointAnnotationForBackend(OsgWidget& self, const osg::Vec3f& pointWorld, const QString& backendId);
	void refreshAnnotationTexts(OsgWidget& self);
	/// Keeps annotation marker/text scale in sync with m_activeModelDiagonal (call from frame timer).
	void updateAnnotationScales(OsgWidget& self);
	bool setAnnotationVisible(OsgWidget& self, const QString& annotationId, bool visible);
	bool removeAnnotation(OsgWidget& self, const QString& annotationId);
	void clearAllAnnotations(OsgWidget& self);

	QList<typename OsgWidget::AnnotationSnapshot> annotationSnapshots(const OsgWidget& self) const;
	void restoreAnnotations(OsgWidget& self, const QList<typename OsgWidget::AnnotationSnapshot>& snapshots);
};

#endif // CLOUDSIMHOST_OSGWIDGETPICKANNOTATIONCONTROLLER_H
