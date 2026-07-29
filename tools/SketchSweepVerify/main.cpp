/// @file main.cpp
/// @brief Rectangle + path sweep offline verify（共面 / 反向 / 对齐校验）

#include "SketchSweep.h"
#include "ShapeHandle.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{
double volumeOf(const geoalgo::ShapeHandle& h)
{
	TopoDS_Shape native;
	if (!geoalgo::ShapeHandleAccess::nativeShape(h, &native) || native.IsNull())
		return -1.0;
	GProp_GProps props;
	BRepGProp::VolumeProperties(native, props);
	return std::abs(props.Mass());
}

void countFaces(const geoalgo::ShapeHandle& h, int& faces, int& planarFaces)
{
	faces = 0;
	planarFaces = 0;
	TopoDS_Shape native;
	if (!geoalgo::ShapeHandleAccess::nativeShape(h, &native) || native.IsNull())
		return;
	for (TopExp_Explorer ex(native, TopAbs_FACE); ex.More(); ex.Next())
	{
		++faces;
		BRepAdaptor_Surface surf(TopoDS::Face(ex.Current()), Standard_True);
		if (surf.GetType() == GeomAbs_Plane)
			++planarFaces;
	}
}

std::vector<geoalgo::SketchSweepPathSegment> splineThrough(const std::vector<gp_Pnt>& through)
{
	std::vector<geoalgo::SketchSweepPathSegment> segs;
	for (std::size_t i = 0; i + 1 < through.size(); ++i)
	{
		geoalgo::SketchSweepPathSegment s;
		s.kind = geoalgo::SketchSweepPathSegKind::SplineThrough;
		s.ax = static_cast<float>(through[i].X());
		s.ay = static_cast<float>(through[i].Y());
		s.az = static_cast<float>(through[i].Z());
		s.bx = static_cast<float>(through[i + 1].X());
		s.by = static_cast<float>(through[i + 1].Y());
		s.bz = static_cast<float>(through[i + 1].Z());
		segs.push_back(s);
	}
	return segs;
}

bool runCase(const char* name, const std::vector<float>& profile,
			 const std::vector<geoalgo::SketchSweepPathSegment>& segs, bool expectCurvedSides = false,
			 double minVol = 1.0)
{
	std::string err;
	geoalgo::SketchSweepParams sp;
	sp.mode = geoalgo::SketchSweepMode::Boss;
	geoalgo::ShapeHandle shape;
	if (!geoalgo::sketchSweepSegmentsToHandle(profile, segs, sp, nullptr, shape, &err) || shape.isNull())
	{
		std::cerr << "FAIL " << name << ": " << err << '\n';
		return false;
	}
	const double vol = volumeOf(shape);
	int faces = 0, planar = 0;
	countFaces(shape, faces, planar);
	std::cout << name << " vol=" << vol << " faces=" << faces << " planar=" << planar << '\n';
	if (vol < minVol)
	{
		std::cerr << "FAIL " << name << ": volume too small\n";
		return false;
	}
	if (expectCurvedSides && planar == faces && faces > 4)
	{
		std::cerr << "FAIL " << name << ": curved sweep became all-planar\n";
		return false;
	}
	if (expectCurvedSides && faces >= 6 && planar >= faces - 1)
	{
		std::cerr << "FAIL " << name << ": faceted planar fan\n";
		return false;
	}
	return true;
}
} // namespace

int main()
{
	// 30x20 矩形，位于 XY，中心在原点
	const std::vector<float> rect = {-15, -10, 0, 15, -10, 0, 15, 10, 0, -15, 10, 0, -15, -10, 0};

	{
		geoalgo::SketchSweepPathSegment s;
		s.kind = geoalgo::SketchSweepPathSegKind::Line;
		s.ax = 0;
		s.ay = 0;
		s.az = 0;
		s.bx = 0;
		s.by = 0;
		s.bz = 80;
		if (!runCase("straight", rect, {s}, false, 40000))
			return 1;
	}

	if (!runCase("splineS", rect,
				 splineThrough({gp_Pnt(0, 0, 0), gp_Pnt(0, 20, 30), gp_Pnt(0, -10, 60), gp_Pnt(0, 0, 90)}), true,
				 50000))
		return 1;

	// 截图同类：轮廓与路径共面
	if (!runCase("coplanarS", rect,
				 splineThrough({gp_Pnt(0, 0, 0), gp_Pnt(20, 15, 0), gp_Pnt(-10, 40, 0), gp_Pnt(0, 70, 0)}), true,
				 40000))
		return 1;

	if (!runCase("coplanarS_reversed", rect,
				 splineThrough({gp_Pnt(0, 70, 0), gp_Pnt(-10, 40, 0), gp_Pnt(20, 15, 0), gp_Pnt(0, 0, 0)}), true,
				 40000))
		return 1;

	// 窄截面 + 长 S（更接近用户截图）
	const std::vector<float> thin = {-8, -1, 0, 8, -1, 0, 8, 1, 0, -8, 1, 0, -8, -1, 0};
	if (!runCase("thin_coplanarS", thin,
				 splineThrough({gp_Pnt(0, 0, 0), gp_Pnt(25, 20, 0), gp_Pnt(-15, 50, 0), gp_Pnt(5, 90, 0)}), true, 500))
		return 1;

	std::cout << "OK all\n";
	return 0;
}
