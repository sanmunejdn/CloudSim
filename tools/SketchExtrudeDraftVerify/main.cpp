/// @file main.cpp
/// @brief 对称/定长拉伸 + 拔模离线点验

#include "SketchExtrude.h"
#include "ShapeHandle.h"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

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
	return props.Mass();
}

bool expectCenteredMidPlane(const geoalgo::ShapeHandle& shape, double halfLen, std::string& err)
{
	const auto bb = shape.boundingBoxMm();
	if (!bb.valid)
	{
		err = "bbox invalid";
		return false;
	}
	if (std::abs(bb.minZ + halfLen) > 0.5 || std::abs(bb.maxZ - halfLen) > 0.5)
	{
		err = "not centered about sketch plane";
		return false;
	}
	return true;
}
} // namespace

int main()
{
	const std::vector<float> square = {0, 0, 0, 40, 0, 0, 40, 40, 0, 0, 40, 0, 0, 0, 0};
	std::string err;

	geoalgo::SketchExtrudeParams blind;
	blind.mode = geoalgo::SketchExtrudeMode::Pad;
	blind.lengthMm = 20.0;
	blind.draftAngleDeg = 5.0;
	geoalgo::ShapeHandle blindShape;
	if (!geoalgo::sketchExtrudePolylineToHandle(square, blind, nullptr, blindShape, &err) || blindShape.isNull())
	{
		std::cerr << "FAIL Blind+draft: " << err << '\n';
		return 1;
	}
	std::cout << "OK Blind+draft5 vol=" << volumeOf(blindShape) << '\n';

	geoalgo::SketchExtrudeParams mid;
	mid.mode = geoalgo::SketchExtrudeMode::Pad;
	mid.lengthMm = 20.0;
	mid.endCondition = geoalgo::SketchExtrudeEndCondition::MidPlane;
	geoalgo::ShapeHandle plain;
	if (!geoalgo::sketchExtrudePolylineToHandle(square, mid, nullptr, plain, &err) || plain.isNull())
	{
		std::cerr << "FAIL MidPlane: " << err << '\n';
		return 1;
	}
	if (!expectCenteredMidPlane(plain, 10.0, err))
	{
		std::cerr << "FAIL MidPlane: " << err << '\n';
		return 1;
	}
	const double v0 = volumeOf(plain);
	std::cout << "OK MidPlane vol=" << v0 << '\n';

	mid.draftAngleDeg = 5.0;
	geoalgo::ShapeHandle drafted;
	if (!geoalgo::sketchExtrudePolylineToHandle(square, mid, nullptr, drafted, &err) || drafted.isNull())
	{
		std::cerr << "FAIL MidPlane+draft: " << err << '\n';
		return 1;
	}
	if (!expectCenteredMidPlane(drafted, 10.0, err))
	{
		std::cerr << "FAIL MidPlane+draft center: " << err << '\n';
		return 1;
	}
	const double v1 = volumeOf(drafted);
	if (std::abs(v1 - v0) < 1.0)
	{
		std::cerr << "FAIL MidPlane+draft volume unchanged (v0=" << v0 << " v1=" << v1 << ")\n";
		return 1;
	}
	std::cout << "OK MidPlane+draft5 vol=" << v1 << " delta=" << (v1 - v0) << '\n';
	return 0;
}
