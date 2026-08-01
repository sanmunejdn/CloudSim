/// @file SketchGeom.h
/// @brief 草图平面 UV 几何（对齐 OneCAD 语义，无 OCC 依赖）

#ifndef GEOMETRICMODELINGPLUGIN_SKETCHGEOM_H
#define GEOMETRICMODELINGPLUGIN_SKETCHGEOM_H

#include "PluginGeometryTypes.h"

#include <QByteArray>
#include <QString>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

struct SkVec2
{
	double u = 0.0;
	double v = 0.0;
};

inline double skDist(const SkVec2& a, const SkVec2& b)
{
	const double du = a.u - b.u;
	const double dv = a.v - b.v;
	return std::sqrt(du * du + dv * dv);
}

inline SkVec2 skLerp(const SkVec2& a, const SkVec2& b, double t)
{
	return {a.u + (b.u - a.u) * t, a.v + (b.v - a.v) * t};
}

enum class SkEntityKind
{
	Line = 0,
	Arc,
	Circle
};

enum class SkSnapKind
{
	None = 0,
	Endpoint,
	Midpoint,
	Center,
	OnCurve,
	Grid,
	Horizontal,
	Vertical
};

struct SkSnapResult
{
	bool snapped = false;
	SkSnapKind kind = SkSnapKind::None;
	SkVec2 pos{};
};

struct SkPoint
{
	int id = 0;
	SkVec2 p{};
	bool fixed = false;
};

struct SkLine
{
	int id = 0;
	int p1 = -1;
	int p2 = -1;
	bool construction = false;
};

struct SkArc
{
	int id = 0;
	int pStart = -1;
	int pMid = -1;
	int pEnd = -1;
	bool construction = false;
};

struct SkCircle
{
	int id = 0;
	int center = -1;
	double radius = 0.0;
	bool construction = false;
};

struct SkEllipse
{
	int id = 0;
	int center = -1;
	double majorR = 0.0;
	double minorR = 0.0;
	double angleRad = 0.0;
	bool construction = false;
};

enum class SkSplineMode
{
	ThroughPoints = 0,
	ControlPoints = 1
};

struct SkSpline
{
	int id = 0;
	std::vector<int> throughPts;
	/// ControlPoints 模式的极点；旧文档可空
	std::vector<int> controlPts;
	SkSplineMode mode = SkSplineMode::ThroughPoints;
	bool construction = false;
};

enum class SkConstraintKind
{
	Coincident = 0,
	Horizontal,
	Vertical,
	EqualLength,
	Distance,
	Parallel,
	Perpendicular,
	Radius,
	Angle,
	ArcRadius,
	Tangent,
	Symmetric,
	Midpoint,
	/// 椭圆长/短半轴（a=椭圆 id）
	MajorRadius,
	MinorRadius
};

struct SkConstraint
{
	SkConstraintKind kind = SkConstraintKind::Coincident;
	int a = -1;
	int b = -1;
	double value = 0.0;
	/// Symmetric：对称轴直线 id
	int c = -1;
};

enum class SkConstraintDiag
{
	Normal = 0,
	Conflicting,
	Redundant
};

struct SkOverlayStyle
{
	std::unordered_set<int> conflictEntityIds;
	std::unordered_set<int> redundantEntityIds;
	/// 尺寸工具悬停/已选高亮（点、线、圆、弧 id）
	std::unordered_set<int> highlightEntityIds;
	double normalBiasMm = 0.05;
};

class SketchDocument2d
{
public:
	int addPoint(double u, double v, bool fixed = false);
	int addLine(int p1, int p2, bool construction = false);
	int addArc(int pStart, int pMid, int pEnd, bool construction = false);
	int addCircle(int center, double radius, bool construction = false);
	int addEllipse(int center, double majorR, double minorR, double angleRad = 0.0, bool construction = false);
	int addSpline(const std::vector<int>& throughPts, bool construction = false);
	/// 过点模式首次进入控制点时生成 poles（默认同过点）
	bool ensureSplineControlPoints(int splineId);
	bool setSplineMode(int splineId, SkSplineMode mode);
	void addConstraint(const SkConstraint& c);
	bool toggleConstruction(int entityId);
	bool removeLine(int id);
	bool removeArc(int id);
	bool removeCircle(int id);
	bool removeEllipse(int id);
	bool removeSpline(int id);
	/// 删线/弧/圆/样条并清理关联约束与孤点
	bool removeEntity(int id);
	/// 直线裁剪：在交点处分裂；返回是否修改
	bool trimLineAt(const SkVec2& uv, double tolMm);
	/// 在最近点处劈开样条为两段（端点侧过短则删除该侧）
	bool trimSplineAt(const SkVec2& uv, double tolMm);
	/// 相对直线镜像选中图元（线/圆/弧/样条）
	bool mirrorEntities(int mirrorLineId, const std::vector<int>& entityIds);
	void clear();

	SkPoint* findPoint(int id);
	const SkPoint* findPoint(int id) const;
	SkLine* findLine(int id);
	const SkLine* findLine(int id) const;
	SkArc* findArc(int id);
	const SkArc* findArc(int id) const;
	SkCircle* findCircle(int id);
	const SkCircle* findCircle(int id) const;
	SkEllipse* findEllipse(int id);
	const SkEllipse* findEllipse(int id) const;
	SkSpline* findSpline(int id);
	const SkSpline* findSpline(int id) const;

	const std::vector<SkPoint>& points() const { return m_points; }
	std::vector<SkPoint>& pointsMut() { return m_points; }
	const std::vector<SkLine>& lines() const { return m_lines; }
	std::vector<SkLine>& linesMut() { return m_lines; }
	const std::vector<SkArc>& arcs() const { return m_arcs; }
	std::vector<SkArc>& arcsMut() { return m_arcs; }
	const std::vector<SkCircle>& circles() const { return m_circles; }
	std::vector<SkCircle>& circlesMut() { return m_circles; }
	const std::vector<SkEllipse>& ellipses() const { return m_ellipses; }
	std::vector<SkEllipse>& ellipsesMut() { return m_ellipses; }
	const std::vector<SkSpline>& splines() const { return m_splines; }
	std::vector<SkSpline>& splinesMut() { return m_splines; }
	const std::vector<SkConstraint>& constraints() const { return m_constraints; }
	std::vector<SkConstraint>& constraintsMut() { return m_constraints; }

	SkVec2 worldToUv(const PluginSketchPlane& plane, const PluginPoint3d& w) const;
	PluginPoint3d uvToWorld(const PluginSketchPlane& plane, const SkVec2& uv, double normalBiasMm = 0.0) const;

	bool exportClosedProfileXyz(const PluginSketchPlane& plane, std::vector<float>& outXyzMm,
								std::string* err = nullptr) const;
	/// 所有闭合环；最大面积者为外轮廓，其余为孔
	bool exportClosedProfilesXyz(const PluginSketchPlane& plane, std::vector<std::vector<float>>& outLoops,
								 std::string* err = nullptr) const;
	bool exportClosedProfilesUv(std::vector<std::vector<SkVec2>>& outLoops, std::string* err = nullptr) const;
	/// 开放路径：连续 Line/Arc/样条折线（拒绝分叉/闭环）
	bool exportOpenPathXyz(const PluginSketchPlane& plane, std::vector<float>& outXyzMm,
						   std::string* err = nullptr) const;
	/// 开放路径段：Line/Arc 真段 + 样条离散为折线段
	bool exportOpenPathSegments(const PluginSketchPlane& plane, std::vector<PluginSketchSweepPathSegment>& outSegs,
								std::string* err = nullptr) const;
	/// 闭合外轮廓真曲线段（单圆/椭圆或线弧闭环）；失败时回退折线导出
	bool exportClosedProfileSegments(const PluginSketchPlane& plane, std::vector<PluginSketchSweepPathSegment>& outSegs,
									 std::string* err = nullptr) const;

	void tessellateOverlay(const PluginSketchPlane& plane, std::vector<PluginSketchOverlaySegment>& out,
						   const SkVec2* previewA = nullptr, const SkVec2* previewB = nullptr,
						   const SkSnapResult* snap = nullptr, const SkOverlayStyle* style = nullptr,
						   const std::vector<SkVec2>* previewPoly = nullptr) const;

	QByteArray toJsonUtf8() const;
	bool fromJsonUtf8(const QByteArray& utf8);

	QString constraintLabel(const SkConstraint& c) const;

	/// 拾取：返回点/线/圆/弧/样条文档 id，未命中 -1
	int hitTestPoint(const SkVec2& uv, double tolMm) const;
	int hitTestLine(const SkVec2& uv, double tolMm) const;
	int hitTestCircle(const SkVec2& uv, double tolMm) const;
	int hitTestEllipse(const SkVec2& uv, double tolMm) const;
	int hitTestArc(const SkVec2& uv, double tolMm) const;
	int hitTestSpline(const SkVec2& uv, double tolMm) const;

	bool sampleSplineUv(const SkSpline& sp, std::vector<SkVec2>& out, int segsPerSpan = 12) const;
	bool isSplineThroughPoint(int pointId) const;

private:
	int nextId();
	int m_seq = 1;
	std::vector<SkPoint> m_points;
	std::vector<SkLine> m_lines;
	std::vector<SkArc> m_arcs;
	std::vector<SkCircle> m_circles;
	std::vector<SkEllipse> m_ellipses;
	std::vector<SkSpline> m_splines;
	std::vector<SkConstraint> m_constraints;
};

SkSnapResult sketchSnap(const SketchDocument2d& doc, const SkVec2& raw, double tolMm, double gridMm,
						const SkVec2* refForOrtho);

bool sketchCircumcenter(const SkVec2& a, const SkVec2& b, const SkVec2& c, SkVec2& out, double& radius);

/// 过点 Catmull-Rom 均匀采样（端点重复）
void sketchSampleCatmullRom(const std::vector<SkVec2>& through, std::vector<SkVec2>& out, int segsPerSpan = 12);

void sketchSampleEllipse(const SkVec2& center, double majorR, double minorR, double angleRad,
						 std::vector<SkVec2>& out, int segs = 48);

bool offsetClosedUv(const std::vector<SkVec2>& poly, double dist, std::vector<SkVec2>& out, std::string* err = nullptr);

/// 闭合折线是否自交（端点共享不算）
bool closedPolylineSelfIntersectsUv(const std::vector<SkVec2>& poly, double eps = 1e-9);

#endif
