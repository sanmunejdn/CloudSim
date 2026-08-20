#ifndef ENGINEERINGDRAWINGPLUGIN_DRAWINGSHEETMODEL_H
#define ENGINEERINGDRAWINGPLUGIN_DRAWINGSHEETMODEL_H

/// @file DrawingSheetModel.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 图纸几何与场景坐标 QPainterPath 缓存（画布只读渲染）

#include "DrawingSheetCanvasWidget.h"

#include <QHash>
#include <QPainterPath>
#include <QString>
#include <QVector>

class DrawingSheetModel
{
public:
	void clear();
	void setViews(const QVector<DrawingSheetCanvasWidget::DrawingView>& views);
	void updateViewGeometry(const DrawingSheetCanvasWidget::DrawingView& view);
	void invalidatePaths();
	void invalidatePath(const QString& viewId);
	/// 视图拖动时同步场景 path，避免整包重建
	void translatePath(const QString& viewId, const QPointF& delta);

	/// 场景坐标 path；内容变更后重建，缩放/平移只变换绘制
	const QPainterPath& visiblePath(const QString& viewId) const;
	const QPainterPath& hiddenPath(const QString& viewId) const;

	bool hasPath(const QString& viewId) const;

private:
	struct ViewPaths
	{
		QPainterPath visible;
		QPainterPath hidden;
		bool valid = false;
	};

	void rebuildPath(const DrawingSheetCanvasWidget::DrawingView& view) const;

	QVector<DrawingSheetCanvasWidget::DrawingView> m_views;
	mutable QHash<QString, ViewPaths> m_paths;
	static const QPainterPath s_empty;
};

#endif
