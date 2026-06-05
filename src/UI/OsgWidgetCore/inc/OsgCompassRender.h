#pragma once

#include <osg/Depth>
#include <osg/GL>
#include <osg/Material>
#include <osg/StateAttribute>
#include <osg/StateSet>

/// 罗盘渲染：对象变换与 TCP 示教共用固定色、无光照状态，避免头灯/受光网格导致明暗闪烁
namespace osg_compass {

/// 套用罗盘 StateSet：关闭光照与所有光源，顶点/面色直接输出，深度始终通过且不写深度缓冲
inline void applyUnlitHighlitStateSet(osg::StateSet* ss)
{
	if (!ss)
	{
		return;
	}
	const auto modeOn = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;
	const auto modeOff = osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;

	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	ss->setRenderBinDetails(100, "RenderBin");

	ss->setMode(GL_LIGHTING, modeOff);
	ss->setMode(GL_COLOR_MATERIAL, modeOff);
	ss->setMode(GL_NORMALIZE, modeOff);
	ss->setMode(GL_FOG, modeOff);
	ss->setMode(GL_CULL_FACE, modeOff);
	ss->setMode(GL_BLEND, modeOff);
	for (GLenum light = GL_LIGHT0; light <= GL_LIGHT7; ++light)
	{
		ss->setMode(light, modeOff);
	}

	osg::ref_ptr<osg::Material> mat = new osg::Material;
	mat->setColorMode(osg::Material::OFF);
	mat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	mat->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	ss->setAttributeAndModes(mat.get(), modeOn);

	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setWriteMask(false);
	ss->setAttributeAndModes(depth.get(), modeOn);
	ss->setMode(GL_DEPTH_TEST, modeOff);
}

} // namespace osg_compass
