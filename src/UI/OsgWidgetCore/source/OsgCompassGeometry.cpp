/// @file OsgCompassGeometry.cpp
/// @brief OsgCompassGeometry 实现

#include "OsgCompassGeometry.h"

#include "OsgCompassRender.h"

#include <osg/Array>
#include <osg/GL>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/Quat>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Vec3>
#include <osg/Vec4>

namespace osg_compass
{
osg::ref_ptr<osg::Node> buildTransformCompassNode(TransformCompassBranches* outBranches)
{
	if (outBranches)
	{
		for (int i = 0; i < 3; ++i)
		{
			outBranches->axis[i] = nullptr;
			outBranches->ring[i] = nullptr;
		}
	}

	const float axisLen = kCompassAxisLength;
	const float coneH = 20.0f * kCompassGeomScale;
	const float coneR = 7.0f * kCompassGeomScale;
	const float shaftEnd = axisLen - 12.0f * kCompassGeomScale;
	const float tipExtension = 6.0f * kCompassGeomScale;
	const float ringRadius = 65.0f * kCompassGeomScale;
	const float tubeR = 2.85f * kCompassGeomScale;

	const auto applyCompassStateSet = [](osg::StateSet* ss) { applyUnlitHighlitStateSet(ss); };

	auto addSolidTorusRing = [&](int plane, const osg::Vec4& color) -> osg::ref_ptr<osg::Geode>
	{
		const int slices = 48;
		const int stacks = 12;
		const int rowVerts = slices + 1;

		osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
		verts->reserve(static_cast<unsigned>((stacks + 1) * rowVerts));

		auto mapTorusPoint = [&](float cu, float su, float cv, float sv) -> osg::Vec3
		{
			const float major = ringRadius + tubeR * cv;
			if (plane == 0)
			{
				return osg::Vec3(tubeR * sv, major * cu, major * su);
			}
			if (plane == 1)
			{
				return osg::Vec3(major * cu, tubeR * sv, major * su);
			}
			return osg::Vec3(major * cu, major * su, tubeR * sv);
		};

		for (int stack = 0; stack <= stacks; ++stack)
		{
			const float v = osg::PI * 2.0f * static_cast<float>(stack) / static_cast<float>(stacks);
			const float cv = std::cos(v);
			const float sv = std::sin(v);
			for (int slice = 0; slice <= slices; ++slice)
			{
				const float u = osg::PI * 2.0f * static_cast<float>(slice) / static_cast<float>(slices);
				verts->push_back(mapTorusPoint(std::cos(u), std::sin(u), cv, sv));
			}
		}

		osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLES);
		indices->reserve(static_cast<unsigned>(stacks * slices * 6));
		for (int stack = 0; stack < stacks; ++stack)
		{
			for (int slice = 0; slice < slices; ++slice)
			{
				const unsigned i0 = static_cast<unsigned>(stack * rowVerts + slice);
				const unsigned i1 = i0 + 1U;
				const unsigned i2 = i0 + static_cast<unsigned>(rowVerts);
				const unsigned i3 = i2 + 1U;
				indices->push_back(i0);
				indices->push_back(i2);
				indices->push_back(i1);
				indices->push_back(i1);
				indices->push_back(i2);
				indices->push_back(i3);
			}
		}

		osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
		ca->push_back(color);
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		geom->setVertexArray(verts.get());
		geom->setColorArray(ca.get(), osg::Array::BIND_OVERALL);
		geom->setNormalBinding(osg::Geometry::BIND_OFF);
		geom->addPrimitiveSet(indices.get());
		applyCompassStateSet(geom->getOrCreateStateSet());

		osg::ref_ptr<osg::Geode> g = new osg::Geode;
		g->addDrawable(geom.get());
		applyCompassStateSet(g->getOrCreateStateSet());
		return g;
	};

	auto addPositiveAxis = [&](const osg::Vec3& p1, const osg::Vec3& coneDir,
							   const osg::Vec4& col) -> osg::ref_ptr<osg::Geode>
	{
		osg::ref_ptr<osg::Geode> g = new osg::Geode;
		osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
		v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
		v->push_back(p1);
		osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
		ca->push_back(col);
		osg::ref_ptr<osg::Geometry> lineGeom = new osg::Geometry;
		lineGeom->setVertexArray(v.get());
		lineGeom->setColorArray(ca.get(), osg::Array::BIND_OVERALL);
		lineGeom->setNormalBinding(osg::Geometry::BIND_OFF);
		lineGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
		g->addDrawable(lineGeom.get());
		applyCompassStateSet(lineGeom->getOrCreateStateSet());
		lineGeom->getOrCreateStateSet()->setAttribute(new osg::LineWidth(6.0f));

		const osg::Vec3 tip = coneDir * (axisLen + tipExtension);
		const osg::Vec3 coneCenter = tip - coneDir * (coneH * 0.5f);
		osg::ref_ptr<osg::Cone> cone = new osg::Cone(osg::Vec3(0.0f, 0.0f, 0.0f), coneR, coneH);
		osg::Quat rot;
		rot.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), coneDir);
		cone->setRotation(rot);
		cone->setCenter(coneCenter);
		osg::ref_ptr<osg::ShapeDrawable> coneDraw = new osg::ShapeDrawable(cone.get());
		coneDraw->setUseDisplayList(false);
		coneDraw->setColor(col);
		g->addDrawable(coneDraw.get());
		applyCompassStateSet(coneDraw->getOrCreateStateSet());
		applyCompassStateSet(g->getOrCreateStateSet());
		return g;
	};

	osg::ref_ptr<osg::Group> root = new osg::Group;
	applyCompassStateSet(root->getOrCreateStateSet());

	const osg::Vec4 red(1.0f, 0.15f, 0.15f, 1.0f);
	const osg::Vec4 green(0.15f, 1.0f, 0.15f, 1.0f);
	const osg::Vec4 blue(0.15f, 0.45f, 1.0f, 1.0f);

	auto wrapAxisBranch = [&](osg::Node* child) -> osg::ref_ptr<osg::MatrixTransform>
	{
		osg::ref_ptr<osg::MatrixTransform> br = new osg::MatrixTransform;
		br->addChild(child);
		applyCompassStateSet(br->getOrCreateStateSet());
		return br;
	};

	osg::ref_ptr<osg::MatrixTransform> axisBranches[3];
	osg::ref_ptr<osg::MatrixTransform> ringBranches[3];

	axisBranches[0] =
		wrapAxisBranch(addPositiveAxis(osg::Vec3(shaftEnd, 0.0f, 0.0f), osg::Vec3(1.0f, 0.0f, 0.0f), red).get());
	root->addChild(axisBranches[0].get());

	axisBranches[1] =
		wrapAxisBranch(addPositiveAxis(osg::Vec3(0.0f, shaftEnd, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f), green).get());
	root->addChild(axisBranches[1].get());

	axisBranches[2] =
		wrapAxisBranch(addPositiveAxis(osg::Vec3(0.0f, 0.0f, shaftEnd), osg::Vec3(0.0f, 0.0f, 1.0f), blue).get());
	root->addChild(axisBranches[2].get());

	ringBranches[0] = wrapAxisBranch(addSolidTorusRing(0, osg::Vec4(1.0f, 0.35f, 0.35f, 1.0f)).get());
	root->addChild(ringBranches[0].get());

	ringBranches[1] = wrapAxisBranch(addSolidTorusRing(1, osg::Vec4(0.35f, 1.0f, 0.35f, 1.0f)).get());
	root->addChild(ringBranches[1].get());

	ringBranches[2] = wrapAxisBranch(addSolidTorusRing(2, osg::Vec4(0.35f, 0.55f, 1.0f, 1.0f)).get());
	root->addChild(ringBranches[2].get());

	if (outBranches)
	{
		for (int i = 0; i < 3; ++i)
		{
			outBranches->axis[i] = axisBranches[i];
			outBranches->ring[i] = ringBranches[i];
		}
	}

	return root;
}

} // namespace osg_compass
