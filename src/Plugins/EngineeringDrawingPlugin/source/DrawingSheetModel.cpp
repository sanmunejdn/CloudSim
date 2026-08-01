/// @file DrawingSheetModel.cpp
/// @brief 视图折线 → 场景 QPainterPath 缓存

#include "DrawingSheetModel.h"

const QPainterPath DrawingSheetModel::s_empty;

void DrawingSheetModel::clear()
{
	m_views.clear();
	m_paths.clear();
}

void DrawingSheetModel::setViews(const QVector<DrawingSheetCanvasWidget::DrawingView>& views)
{
	m_views = views;
	m_paths.clear();
	for (const DrawingSheetCanvasWidget::DrawingView& v : m_views)
		rebuildPath(v);
}

void DrawingSheetModel::updateViewGeometry(const DrawingSheetCanvasWidget::DrawingView& view)
{
	for (DrawingSheetCanvasWidget::DrawingView& v : m_views)
	{
		if (v.id != view.id)
			continue;
		v.visible = view.visible;
		v.hidden = view.hidden;
		rebuildPath(v);
		return;
	}
	m_views.push_back(view);
	rebuildPath(view);
}

void DrawingSheetModel::invalidatePaths()
{
	m_paths.clear();
	for (const DrawingSheetCanvasWidget::DrawingView& v : m_views)
		rebuildPath(v);
}

void DrawingSheetModel::invalidatePath(const QString& viewId)
{
	m_paths.remove(viewId);
	for (const DrawingSheetCanvasWidget::DrawingView& v : m_views)
	{
		if (v.id == viewId)
		{
			rebuildPath(v);
			return;
		}
	}
}

void DrawingSheetModel::translatePath(const QString& viewId, const QPointF& delta)
{
	auto it = m_paths.find(viewId);
	if (it == m_paths.end() || !it->valid)
		return;
	it->visible.translate(delta);
	it->hidden.translate(delta);
}

const QPainterPath& DrawingSheetModel::visiblePath(const QString& viewId) const
{
	const auto it = m_paths.constFind(viewId);
	if (it == m_paths.cend() || !it->valid)
		return s_empty;
	return it->visible;
}

const QPainterPath& DrawingSheetModel::hiddenPath(const QString& viewId) const
{
	const auto it = m_paths.constFind(viewId);
	if (it == m_paths.cend() || !it->valid)
		return s_empty;
	return it->hidden;
}

bool DrawingSheetModel::hasPath(const QString& viewId) const
{
	const auto it = m_paths.constFind(viewId);
	return it != m_paths.cend() && it->valid;
}

void DrawingSheetModel::rebuildPath(const DrawingSheetCanvasWidget::DrawingView& view) const
{
	ViewPaths paths;
	auto appendPolys = [](QPainterPath& path, const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys) {
		for (const DrawingSheetCanvasWidget::Polyline2d& poly : polys)
		{
			if (poly.points.size() < 2)
				continue;
			path.moveTo(poly.points.front());
			for (int i = 1; i < poly.points.size(); ++i)
				path.lineTo(poly.points[i]);
		}
	};
	appendPolys(paths.visible, view.visible);
	appendPolys(paths.hidden, view.hidden);
	paths.valid = true;
	m_paths.insert(view.id, paths);
}
