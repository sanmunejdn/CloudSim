#include "OsgSectionPlaneGeometry.h"

#include "OsgCompassGeometry.h"
#include "OsgCompassRender.h"

#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/PolygonOffset>
#include <osg/Vec4>

namespace osg_section_plane
{

osg::ref_ptr<osg::Node> buildSectionPlaneQuadNode(const float planeHalfSizeMm)
{
	auto* root = new osg::Group;
	root->setName("SectionPlaneQuadRoot");

	osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
	verts->push_back(osg::Vec3f(-planeHalfSizeMm, -planeHalfSizeMm, 0.f));
	verts->push_back(osg::Vec3f(planeHalfSizeMm, -planeHalfSizeMm, 0.f));
	verts->push_back(osg::Vec3f(planeHalfSizeMm, planeHalfSizeMm, 0.f));
	verts->push_back(osg::Vec3f(-planeHalfSizeMm, planeHalfSizeMm, 0.f));

	osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLES);
	indices->push_back(0);
	indices->push_back(1);
	indices->push_back(2);
	indices->push_back(0);
	indices->push_back(2);
	indices->push_back(3);

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4f(0.2f, 0.75f, 1.0f, 0.28f));

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(verts.get());
	geom->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
	geom->addPrimitiveSet(indices.get());
	geom->setNormalBinding(osg::Geometry::BIND_OFF);

	osg::ref_ptr<osg::Geode> planeGeode = new osg::Geode;
	planeGeode->addDrawable(geom.get());
	planeGeode->setName("SectionPlaneQuad");

	const auto modeOn = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;
	osg::StateSet* ss = planeGeode->getOrCreateStateSet();
	ss->setMode(GL_BLEND, modeOn);
	ss->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), modeOn);
	ss->setAttributeAndModes(new osg::PolygonOffset(1.f, 1.f), modeOn);
	ss->setMode(GL_DEPTH_TEST, modeOn);
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::LEQUAL);
	depth->setWriteMask(false);
	ss->setAttributeAndModes(depth.get(), modeOn);

	root->addChild(planeGeode.get());
	return root;
}

osg::ref_ptr<osg::Node> buildSectionPlaneNode(
	const float planeHalfSizeMm,
	osg_compass::TransformCompassBranches* outBranches)
{
	auto* root = new osg::Group;
	root->setName("SectionPlaneRoot");
	root->addChild(buildSectionPlaneQuadNode(planeHalfSizeMm).get());
	root->addChild(osg_compass::buildTransformCompassNode(outBranches));
	return root;
}

} // namespace osg_section_plane
