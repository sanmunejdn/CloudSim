#ifndef WIDGET_OSGWIDGETPICKANNOTATIONCONTROLLER_H
#define WIDGET_OSGWIDGETPICKANNOTATIONCONTROLLER_H

/// @file OsgWidgetPickAnnotationController.h
/// @brief 点选标记与三维文字标注的增删改、刷新与按后端恢复，自 OsgWidget 拆出

#include "OsgWidget.h"

#include <QList>
#include <QString>

/// 点选标记与三维文字标注的增删改、刷新与按后端恢复，自 OsgWidget 拆出
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

#endif // WIDGET_OSGWIDGETPICKANNOTATIONCONTROLLER_H
