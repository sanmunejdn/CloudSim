/// @file SketchConstraintSolver.h
/// @brief PlaneGCS 薄封装（尺寸/几何约束 + 诊断）

#ifndef GEOMETRICMODELINGPLUGIN_SKETCHCONSTRAINTSOLVER_H
#define GEOMETRICMODELINGPLUGIN_SKETCHCONSTRAINTSOLVER_H

#include <string>
#include <vector>

struct SketchPoint2d
{
	double x = 0.0;
	double y = 0.0;
	bool fixed = false;
};

struct SketchLine2d
{
	int p1 = -1;
	int p2 = -1;
};

struct SketchArc2d
{
	int center = -1;
	int start = -1;
	int end = -1;
	double radius = 0.0;
};

enum class SketchConstraintKind
{
	Distance = 0,
	Horizontal,
	Vertical,
	EqualLength,
	Coincident,
	Parallel,
	Perpendicular,
	Radius,
	Angle,	   ///< a/b=线索引，value=度
	ArcRadius ///< a=弧索引，value=半径 mm
};

struct SketchConstraint2d
{
	SketchConstraintKind kind = SketchConstraintKind::Distance;
	int a = -1;
	int b = -1;
	double value = 0.0;
	int tagId = 0; ///< 0=自动分配；否则用文档约束序号映射诊断
};

class SketchConstraintSolver
{
public:
	void clear();
	int addPoint(double x, double y, bool fixed = false);
	int addLine(int p1, int p2);
	int addArc(int center, int start, int end, double radius);
	void addConstraint(const SketchConstraint2d& c);

	/// @return 0 成功
	int solve(std::string* errMsg = nullptr);
	int dof() const { return m_dof; }
	bool hasConflicting() const { return m_hasConflicting; }
	bool hasRedundant() const { return m_hasRedundant; }
	const std::vector<int>& conflictingTags() const { return m_conflictingTags; }
	const std::vector<int>& redundantTags() const { return m_redundantTags; }

	const std::vector<SketchPoint2d>& points() const { return m_points; }
	const std::vector<SketchArc2d>& arcs() const { return m_arcs; }

	static bool runEquilateralTriangleSelfTest(std::string* errMsg = nullptr);

private:
	std::vector<SketchPoint2d> m_points;
	std::vector<SketchLine2d> m_lines;
	std::vector<SketchArc2d> m_arcs;
	std::vector<SketchConstraint2d> m_constraints;
	int m_dof = 0;
	bool m_hasConflicting = false;
	bool m_hasRedundant = false;
	std::vector<int> m_conflictingTags;
	std::vector<int> m_redundantTags;
};

#endif
