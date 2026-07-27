/// @file FrameBackendVisual.cpp
/// @brief FrameBackendVisual 实现

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendIdUserData.h"
#include "BackendPoseOsg.h"
#include "FrameBackendData.h"
#include "FrameBackendVisual.h"
#include "BackendTypeIds.h"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/PrimitiveSet>
#include <osg/StateSet>
#include <osg/Vec3>
#include <osg/Vec4>

namespace
{
osg::ref_ptr<osg::Geode> createRgbAxisGeode(const float axisLengthMm)
{
	osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	const osg::Vec4 xColor(1.0f, 0.4f, 0.4f, 1.0f);
	const osg::Vec4 yColor(0.4f, 1.0f, 0.4f, 1.0f);
	const osg::Vec4 zColor(0.4f, 0.6f, 1.0f, 1.0f);

	verts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	verts->push_back(osg::Vec3(axisLengthMm, 0.0f, 0.0f));
	colors->push_back(xColor);
	colors->push_back(xColor);

	verts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	verts->push_back(osg::Vec3(0.0f, axisLengthMm, 0.0f));
	colors->push_back(yColor);
	colors->push_back(yColor);

	verts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	verts->push_back(osg::Vec3(0.0f, 0.0f, axisLengthMm));
	colors->push_back(zColor);
	colors->push_back(zColor);

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(verts.get());
	geom->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, static_cast<GLsizei>(verts->size())));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->setName("frameAxes");
	geode->addDrawable(geom.get());
	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	ss->setMode(GL_BLEND, osg::StateAttribute::ON);
	ss->setAttributeAndModes(new osg::LineWidth(2.5f), osg::StateAttribute::ON);
	return geode;
}
} // namespace

std::string FrameBackendVisual::typeKey() const
{
	return backend_type::kClassFrame;
}

bool FrameBackendVisual::buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions&,
										  BranchBuildResult& out, std::string* errorMessage)
{
	const auto* frame = dynamic_cast<const FrameBackendData*>(&data);
	if (!frame)
	{
		if (errorMessage)
		{
			*errorMessage = "Backend type mismatch (expected FrameBackendData).";
		}
		return false;
	}

	const float axisLen = frame->axisLengthMm();
	osg::ref_ptr<osg::Geode> axes = createRgbAxisGeode(axisLen);
	osg::ref_ptr<osg::MatrixTransform> outer = new osg::MatrixTransform;
	outer->setMatrix(backend_pose_osg::osgMatrixFromBackendWorldMatrix(data.worldMatrix()));
	outer->addChild(axes.get());
	BackendIdUserData::attach(outer.get(), frame->id());

	out.outer = outer;
	out.modelCenter = osg::Vec3f(0.0f, 0.0f, 0.0f);
	out.diagonal = axisLen * 1.732f;
	return true;
}

void FrameBackendVisual::computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
													   float& outDiagonal) const
{
	outCenter = osg::Vec3f(0.0f, 0.0f, 0.0f);
	outDiagonal = FrameBackendData::kDefaultAxisLengthMm * 1.732f;
	if (const auto* frame = dynamic_cast<const FrameBackendData*>(&data))
	{
		outDiagonal = frame->axisLengthMm() * 1.732f;
	}
}
