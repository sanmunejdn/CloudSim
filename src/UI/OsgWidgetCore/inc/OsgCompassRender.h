#pragma once

#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/StateAttribute>
#include <osg/StateSet>

namespace osg_compass {

inline void applyUnlitHighlitStateSet(osg::StateSet* ss)
{
	if (!ss)
	{
		return;
	}
	const auto modeOn = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE;
	const auto modeOff = osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE;
	ss->setAttributeAndModes(new osg::PolygonOffset(-1.0f, -1.0f), modeOn);
	ss->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	ss->setMode(GL_BLEND, modeOff);
	ss->setMode(GL_LIGHTING, modeOff);
	ss->setMode(GL_COLOR_MATERIAL, modeOff);
	ss->setMode(GL_FOG, modeOff);
	ss->setMode(GL_CULL_FACE, modeOff);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setWriteMask(false);
	ss->setAttributeAndModes(depth.get(), modeOn);
	ss->setMode(GL_DEPTH_TEST, modeOff);
}

} // namespace osg_compass
