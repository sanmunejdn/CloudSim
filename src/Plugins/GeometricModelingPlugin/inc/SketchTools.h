/// @file SketchTools.h
/// @brief 直线/弧/圆/矩形/尺寸/几何约束/构造线/裁剪镜像

#ifndef GEOMETRICMODELINGPLUGIN_SKETCHTOOLS_H
#define GEOMETRICMODELINGPLUGIN_SKETCHTOOLS_H

#include "SketchGeom.h"

#include <optional>
#include <string>

enum class SketchToolKind
{
	Line = 0,
	Arc,
	Circle,
	Rectangle,
	Spline,
	DimLength,
	DimDistance,
	DimRadius,
	DimAngle,
	DimArcRadius,
	ToggleConstruction,
	GeomHorizontal,
	GeomVertical,
	GeomCoincident,
	GeomParallel,
	GeomPerpendicular,
	GeomEqualLength,
	GeomFix,
	Trim,
	Mirror,
	Delete
};

inline bool sketchToolIsDimension(SketchToolKind k)
{
	return k == SketchToolKind::DimLength || k == SketchToolKind::DimDistance || k == SketchToolKind::DimRadius ||
		   k == SketchToolKind::DimAngle || k == SketchToolKind::DimArcRadius;
}

inline bool sketchToolIsGeomConstraint(SketchToolKind k)
{
	return k == SketchToolKind::GeomHorizontal || k == SketchToolKind::GeomVertical ||
		   k == SketchToolKind::GeomCoincident || k == SketchToolKind::GeomParallel ||
		   k == SketchToolKind::GeomPerpendicular || k == SketchToolKind::GeomEqualLength ||
		   k == SketchToolKind::GeomFix;
}

inline bool sketchToolIsPickSession(SketchToolKind k)
{
	return sketchToolIsDimension(k) || sketchToolIsGeomConstraint(k) || k == SketchToolKind::ToggleConstruction ||
		   k == SketchToolKind::Trim || k == SketchToolKind::Mirror || k == SketchToolKind::Delete;
}

class ISketchTool
{
public:
	virtual ~ISketchTool() = default;
	virtual SketchToolKind kind() const = 0;
	virtual std::string name() const = 0;
	virtual void cancel() = 0;
	virtual void onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc) = 0;
	virtual void onMove(const SkVec2& pos) = 0;
	virtual bool hasPreview(SkVec2& outA, SkVec2& outB) const = 0;
	virtual bool previewPolyline(std::vector<SkVec2>& out) const
	{
		(void)out;
		return false;
	}
	virtual std::optional<SkVec2> referencePoint() const = 0;
};

class LineSketchTool : public ISketchTool
{
public:
	SketchToolKind kind() const override { return SketchToolKind::Line; }
	std::string name() const override { return "Line"; }
	void cancel() override;
	void onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc) override;
	void onMove(const SkVec2& pos) override;
	bool hasPreview(SkVec2& outA, SkVec2& outB) const override;
	std::optional<SkVec2> referencePoint() const override;

private:
	bool m_active = false;
	int m_lastPointId = -1;
	SkVec2 m_start{};
	SkVec2 m_curr{};
};

class ArcSketchTool : public ISketchTool
{
public:
	SketchToolKind kind() const override { return SketchToolKind::Arc; }
	std::string name() const override { return "Arc"; }
	void cancel() override;
	void onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc) override;
	void onMove(const SkVec2& pos) override;
	bool hasPreview(SkVec2& outA, SkVec2& outB) const override;
	std::optional<SkVec2> referencePoint() const override;

private:
	int m_step = 0;
	int m_idStart = -1;
	int m_idMid = -1;
	SkVec2 m_start{};
	SkVec2 m_mid{};
	SkVec2 m_curr{};
};

class CircleSketchTool : public ISketchTool
{
public:
	SketchToolKind kind() const override { return SketchToolKind::Circle; }
	std::string name() const override { return "Circle"; }
	void cancel() override;
	void onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc) override;
	void onMove(const SkVec2& pos) override;
	bool hasPreview(SkVec2& outA, SkVec2& outB) const override;
	std::optional<SkVec2> referencePoint() const override;

private:
	bool m_haveCenter = false;
	int m_centerId = -1;
	SkVec2 m_center{};
	SkVec2 m_curr{};
};

class RectSketchTool : public ISketchTool
{
public:
	SketchToolKind kind() const override { return SketchToolKind::Rectangle; }
	std::string name() const override { return "Rectangle"; }
	void cancel() override;
	void onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc) override;
	void onMove(const SkVec2& pos) override;
	bool hasPreview(SkVec2& outA, SkVec2& outB) const override;
	std::optional<SkVec2> referencePoint() const override;

private:
	bool m_haveFirst = false;
	SkVec2 m_a{};
	SkVec2 m_curr{};
};

class SplineSketchTool : public ISketchTool
{
public:
	SketchToolKind kind() const override { return SketchToolKind::Spline; }
	std::string name() const override { return "Spline"; }
	void cancel() override;
	void onPress(const SkVec2& pos, bool rightButton, SketchDocument2d& doc) override;
	void onMove(const SkVec2& pos) override;
	bool hasPreview(SkVec2& outA, SkVec2& outB) const override;
	bool previewPolyline(std::vector<SkVec2>& out) const override;
	std::optional<SkVec2> referencePoint() const override;

private:
	std::vector<SkVec2> m_pts;
	SkVec2 m_curr{};
	bool m_haveCursor = false;
};

#endif
