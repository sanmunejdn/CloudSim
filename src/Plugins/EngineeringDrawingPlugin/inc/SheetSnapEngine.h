#ifndef ENGINEERINGDRAWINGPLUGIN_SHEETSNAPENGINE_H
#define ENGINEERINGDRAWINGPLUGIN_SHEETSNAPENGINE_H

/// @file SheetSnapEngine.h
/// @brief 图纸对象捕捉：端点/中点/交点/圆心/垂足/最近点 + 正交/极轴

#include <QLineF>
#include <QPointF>
#include <QString>
#include <QVector>

struct SheetSnapFlags
{
	bool endpoint = true;
	bool midpoint = true;
	bool intersection = true;
	bool center = true;
	bool perpendicular = false;
	bool nearest = false;
	bool ortho = false;
	bool polar = false;
	double polarStepDeg = 45.0;
};

struct SheetSnapResult
{
	QPointF pos;
	bool snapped = false;
	QString kind; ///< end|mid|int|cen|perp|near|ortho|polar
};

class SheetSnapEngine
{
public:
	void setFlags(const SheetSnapFlags& f) { m_flags = f; }
	SheetSnapFlags flags() const { return m_flags; }

	void setSegments(const QVector<QLineF>& segs) { m_segs = segs; }
	void setCenters(const QVector<QPointF>& centers) { m_centers = centers; }

	SheetSnapResult snap(const QPointF& raw, double tolMm, const QPointF* orthoRef) const;

	static bool segmentIntersection(const QLineF& a, const QLineF& b, QPointF& out);

private:
	SheetSnapFlags m_flags;
	QVector<QLineF> m_segs;
	QVector<QPointF> m_centers;
};

#endif
