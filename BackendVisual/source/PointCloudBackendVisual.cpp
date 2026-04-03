#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "PointCloudBackendVisual.h"

#include "BackendDataBase.h"
#include "BackendGeometryMetrics.h"
#include "BackendIdUserData.h"
#include "BackendVisualMath.h"
#include "PointCloudBackendData.h"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include <osg/PrimitiveSet>
#include <osg/StateSet>
#include <osg/Vec3>
#include <osg/Vec4>

namespace {

osg::ref_ptr<osg::Geode> buildGeodeImpl(const PointCloudBackendData& data, std::string* errorMessage)
{
	const std::vector<float>& xyz = data.pointPositionsXyz();
	if (xyz.size() < 3U || (xyz.size() % 3U) != 0U)
	{
		if (errorMessage)
		{
			*errorMessage = "Invalid point buffer in backend data.";
		}
		return nullptr;
	}
	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	points->reserve(xyz.size() / 3U);
	for (std::size_t i = 0; i + 2 < xyz.size(); i += 3)
	{
		points->push_back(osg::Vec3(xyz[i], xyz[i + 1], xyz[i + 2]));
	}
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setVertexArray(points.get());
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points->size())));
	const std::vector<float>& rgba = data.pointVertexRgba();
	if (data.hasPerVertexColors() && rgba.size() == xyz.size() / 3U * 4U)
	{
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		colors->reserve(rgba.size() / 4U);
		for (std::size_t i = 0; i + 3 < rgba.size(); i += 4)
		{
			colors->push_back(osg::Vec4(rgba[i], rgba[i + 1], rgba[i + 2], rgba[i + 3]));
		}
		geometry->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);
	}
	else
	{
		const BackendColor c = data.color();
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(c.r, c.g, c.b, c.a));
		geometry->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
	}
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geometry.get());
	geode->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f));
	return geode;
}

} // namespace

osg::ref_ptr<osg::Geode> PointCloudBackendVisual::makeStagingGeode(const PointCloudBackendData& data,
	std::string* errorMessage) const
{
	return buildGeodeImpl(data, errorMessage);
}

std::string PointCloudBackendVisual::typeKey() const
{
	return "PointCloudBackendData";
}

bool PointCloudBackendVisual::buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions&, BranchBuildResult& out,
	std::string* errorMessage)
{
	const auto* pc = dynamic_cast<const PointCloudBackendData*>(&data);
	if (!pc)
	{
		if (errorMessage)
		{
			*errorMessage = "Backend type mismatch (expected PointCloudBackendData).";
		}
		return false;
	}
	osg::ref_ptr<osg::Geode> geode = buildGeodeImpl(*pc, errorMessage);
	if (!geode)
	{
		return false;
	}
	const osg::Vec3f center = backend_geometry_metrics::pointCloudCenterFromXyz(pc->pointPositionsXyz());
	const float diagonal = backend_geometry_metrics::pointCloudDiagonalFromXyz(pc->pointPositionsXyz());
	osg::ref_ptr<osg::PositionAttitudeTransform> inner = new osg::PositionAttitudeTransform;
	inner->setPosition(-center);
	inner->addChild(geode.get());
	const BackendVec3 p = pc->pose();
	const BackendVec3 r = pc->rotation();
	osg::ref_ptr<osg::PositionAttitudeTransform> outer = new osg::PositionAttitudeTransform;
	const osg::Vec3f pose(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
	outer->setPosition(center + pose);
	outer->setAttitude(backendvisual_math::eulerDegToQuat(
		osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z))));
	outer->addChild(inner.get());
	osg::StateSet* oss = outer->getOrCreateStateSet();
	oss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	BackendIdUserData::attach(outer.get(), pc->id());
	out.outer = outer;
	out.modelCenter = center;
	out.diagonal = diagonal;
	return true;
}

void PointCloudBackendVisual::computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
	float& outDiagonal) const
{
	const auto* pc = dynamic_cast<const PointCloudBackendData*>(&data);
	if (!pc)
	{
		outCenter = osg::Vec3f(0.0f, 0.0f, 0.0f);
		outDiagonal = 1.0f;
		return;
	}
	outCenter = backend_geometry_metrics::pointCloudCenterFromXyz(pc->pointPositionsXyz());
	outDiagonal = backend_geometry_metrics::pointCloudDiagonalFromXyz(pc->pointPositionsXyz());
}
