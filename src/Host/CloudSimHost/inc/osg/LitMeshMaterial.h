#ifndef CLOUDSIMHOST_LITMESHMATERIAL_H
#define CLOUDSIMHOST_LITMESHMATERIAL_H

/// @file LitMeshMaterial.h
/// @brief Fixed-pipeline plastic / painted-metal look for lit triangle meshes (aligned with scene light 0 in OsgScene).

#include <osg/Material>
#include <osg/Vec4>

namespace LitMeshMaterial
{
/// Fixed-pipeline plastic / painted-metal look for lit triangle meshes (aligned with scene light 0 in OsgScene).
inline void applyPlastic(osg::Material& mat, const osg::Vec4& baseColor)
{
	const float amb = 0.22f;
	mat.setAmbient(osg::Material::FRONT_AND_BACK,
				   osg::Vec4(baseColor.r() * amb, baseColor.g() * amb, baseColor.b() * amb, baseColor.a()));
	mat.setDiffuse(osg::Material::FRONT_AND_BACK, baseColor);
	mat.setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.62f, 0.62f, 0.58f, 1.0f));
	mat.setShininess(osg::Material::FRONT_AND_BACK, 64.0f);
	const float em = 0.014f;
	mat.setEmission(osg::Material::FRONT_AND_BACK,
					osg::Vec4(baseColor.r() * em, baseColor.g() * em, baseColor.b() * em, baseColor.a()));
}

} // namespace LitMeshMaterial

#endif // CLOUDSIMHOST_LITMESHMATERIAL_H
