/// @file MeshBackendVisual.cpp
/// @brief MeshBackendVisual 实现

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendDataBase.h"
#include "BackendGeometryMetrics.h"
#include "BackendIdUserData.h"
#include "BackendPoseOsg.h"
#include "BackendVisualMath.h"
#include "MeshBackendData.h"
#include "MeshBackendVisual.h"
#include "BackendTypeIds.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>

#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/PrimitiveSet>
#include <osg/Quat>
#include <osg/StateSet>
#include <osg/Vec3>
#include <osg/Vec4>

namespace
{
static void applyLitPlastic(osg::Material& mat, const osg::Vec4& baseColor)
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

#include "MeshWireGeometry.inc"

void applyAlwaysOnTopStateSet(osg::StateSet* ss)
{
	if (!ss)
	{
		return;
	}
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
							 osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setWriteMask(false);
	ss->setAttributeAndModes(depth.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
}

osg::ref_ptr<osg::Geode> buildOverlayLineGeode(const std::vector<float>& lineSegments, const osg::Vec4& lineColor,
											   const float lineWidth, const bool alwaysOnTop)
{
	if (lineSegments.size() < 6U || (lineSegments.size() % 6U) != 0U)
	{
		return nullptr;
	}
	osg::ref_ptr<osg::Vec3Array> lineVerts = new osg::Vec3Array;
	lineVerts->reserve(lineSegments.size() / 3U);
	for (std::size_t i = 0; i + 2 < lineSegments.size(); i += 3U)
	{
		lineVerts->push_back(osg::Vec3(lineSegments[i], lineSegments[i + 1U], lineSegments[i + 2U]));
	}
	osg::ref_ptr<osg::Geometry> geometryWire = new osg::Geometry;
	geometryWire->setVertexArray(lineVerts.get());
	geometryWire->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineVerts->size())));
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(lineColor);
	geometryWire->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
	osg::ref_ptr<osg::Geode> geodeWire = new osg::Geode;
	geodeWire->setName("meshOverlayLines");
	geodeWire->addDrawable(geometryWire.get());
	osg::StateSet* ssWire = geodeWire->getOrCreateStateSet();
	ssWire->setAttributeAndModes(new osg::LineWidth(lineWidth));
	ssWire->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	if (alwaysOnTop)
	{
		applyAlwaysOnTopStateSet(ssWire);
	}
	return geodeWire;
}

osg::ref_ptr<osg::Node> buildMeshDisplayNodeImpl(const MeshBackendData& data, const MeshVisualOptions& opt,
												 std::string* errorMessage)
{
	const std::vector<float>& soup = data.triangleSoup();
	const bool hasSoup = soup.size() >= 9U && (soup.size() % 9U) == 0U;
	if (!hasSoup && !data.hasOverlayLineSegments())
	{
		// URDF 机器人空壳根等：无三角也要能建枝，否则工程重开后根节点丢失
		osg::ref_ptr<osg::Group> emptyShell = new osg::Group;
		emptyShell->setName("EmptyMeshShell");
		return emptyShell;
	}
	osg::ref_ptr<osg::Group> grp = new osg::Group;
	if (hasSoup)
	{
		osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
		va->reserve(soup.size() / 3U);
		for (std::size_t i = 0; i + 2 < soup.size(); i += 3)
		{
			va->push_back(osg::Vec3(soup[i], soup[i + 1], soup[i + 2]));
		}
		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
		geometry->setVertexArray(va.get());
		geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(va->size())));
		if (opt.useSceneLighting)
		{
			osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array;
			na->reserve(va->size());
			const std::vector<float>& fileNormals = data.triangleVertexNormals();
			if (data.hasTriangleVertexNormals())
			{
				for (std::size_t i = 0; i + 2 < fileNormals.size(); i += 3)
				{
					na->push_back(osg::Vec3(fileNormals[i], fileNormals[i + 1], fileNormals[i + 2]));
				}
			}
			else
			{
				for (std::size_t i = 0; i + 2 < va->size(); i += 3)
				{
					const osg::Vec3& p0 = (*va)[i];
					const osg::Vec3& p1 = (*va)[i + 1];
					const osg::Vec3& p2 = (*va)[i + 2];
					osg::Vec3 e1 = p1 - p0;
					osg::Vec3 e2 = p2 - p0;
					osg::Vec3 n = e1 ^ e2;
					const float len2 = n.length2();
					if (len2 > 1e-20f)
					{
						n.normalize();
					}
					else
					{
						n.set(0.0f, 0.0f, 1.0f);
					}
					const osg::Vec3f nf(static_cast<float>(n.x()), static_cast<float>(n.y()),
										static_cast<float>(n.z()));
					na->push_back(nf);
					na->push_back(nf);
					na->push_back(nf);
				}
			}
			geometry->setNormalArray(na.get(), osg::Array::BIND_PER_VERTEX);
		}
		const BackendColor c = data.color();
		const osg::Vec4 fillColor(c.r, c.g, c.b, c.a);
		const bool useVertexColors = data.hasTriangleVertexColors();
		if (useVertexColors)
		{
			osg::ref_ptr<osg::Vec4Array> vertexColors = new osg::Vec4Array;
			vertexColors->reserve(va->size());
			const std::vector<float>& rgb = data.triangleVertexColors();
			for (std::size_t i = 0; i + 2 < rgb.size(); i += 3U)
			{
				vertexColors->push_back(osg::Vec4(rgb[i], rgb[i + 1U], rgb[i + 2U], 1.0f));
			}
			geometry->setColorArray(vertexColors.get(), osg::Array::BIND_PER_VERTEX);
		}
		else
		{
			osg::ref_ptr<osg::Vec4Array> mc = new osg::Vec4Array;
			mc->push_back(fillColor);
			geometry->setColorArray(mc.get(), osg::Array::BIND_OVERALL);
		}
		osg::ref_ptr<osg::Geode> geodeFill = new osg::Geode;
		geodeFill->addDrawable(geometry.get());
		osg::StateSet* ssFill = geodeFill->getOrCreateStateSet();
		if (useVertexColors)
		{
			ssFill->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		}
		else if (opt.useSceneLighting)
		{
			osg::ref_ptr<osg::Material> mat = new osg::Material;
			applyLitPlastic(*mat, fillColor);
			ssFill->setAttributeAndModes(mat.get(), osg::StateAttribute::ON);
			ssFill->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
			ssFill->setMode(GL_LIGHT0, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
			ssFill->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
		}

		grp->addChild(geodeFill.get());
		if (opt.showWireOutline)
		{
			osg::ref_ptr<osg::Geometry> geometryWire;
			if (opt.useSceneLighting)
			{
				geometryWire = buildMeshFeatureEdgeGeometry(soup, fillColor, 28.0f);
				if (!geometryWire.valid())
				{
					geometryWire = buildMeshOutlineWireGeometry(soup, fillColor);
				}
			}
			else
			{
				geometryWire = buildMeshOutlineWireGeometry(soup, fillColor);
			}
			if (geometryWire.valid())
			{
				osg::ref_ptr<osg::Geode> geodeWire = new osg::Geode;
				geodeWire->setName("meshWireOverlay");
				geodeWire->addDrawable(geometryWire.get());
				osg::StateSet* ssWire = geodeWire->getOrCreateStateSet();
				ssWire->setAttributeAndModes(new osg::PolygonOffset(-1.0f, -1.0f),
											 osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
				ssWire->setAttributeAndModes(new osg::LineWidth(1.0f));
				ssWire->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
				grp->addChild(geodeWire.get());
			}
		}
	}
	if (data.hasOverlayLineSegments())
	{
		const BackendColor c = data.color();
		const osg::Vec4 lineColor(c.r, c.g, c.b, c.a);
		osg::ref_ptr<osg::Geode> overlayGeode =
			buildOverlayLineGeode(data.overlayLineSegments(), lineColor, 3.0f, data.overlayLinesAlwaysOnTop());
		if (overlayGeode.valid())
		{
			grp->addChild(overlayGeode.get());
		}
	}
	return grp;
}

} // namespace

std::string MeshBackendVisual::typeKey() const
{
	return backend_type::kClassModel;
}

osg::ref_ptr<osg::Node> MeshBackendVisual::makeDisplayNode(const MeshBackendData& data,
														   const MeshVisualOptions& options,
														   std::string* errorMessage) const
{
	return buildMeshDisplayNodeImpl(data, options, errorMessage);
}

bool MeshBackendVisual::buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions,
										 BranchBuildResult& out, std::string* errorMessage)
{
	const auto* mesh = dynamic_cast<const MeshBackendData*>(&data);
	if (!mesh)
	{
		if (errorMessage)
		{
			*errorMessage = "Backend type mismatch (expected MeshBackendData).";
		}
		return false;
	}
	osg::ref_ptr<osg::Node> meshRoot = buildMeshDisplayNodeImpl(*mesh, meshOptions, errorMessage);
	if (!meshRoot)
	{
		return false;
	}
	const osg::Vec3f center = backend_geometry_metrics::meshCenterFromSoup(mesh->triangleSoup());
	const float diagonal = backend_geometry_metrics::meshDiagonalFromSoup(mesh->triangleSoup());
	osg::ref_ptr<osg::PositionAttitudeTransform> inner = new osg::PositionAttitudeTransform;
	inner->setPosition(osg::Vec3f(0.0f, 0.0f, 0.0f));
	inner->addChild(meshRoot.get());
	osg::ref_ptr<osg::MatrixTransform> outer = new osg::MatrixTransform;
	outer->setMatrix(backend_pose_osg::osgMatrixFromBackendWorldMatrix(data.worldMatrix()));
	outer->addChild(inner.get());
	osg::StateSet* oss = outer->getOrCreateStateSet();
	if (meshOptions.useSceneLighting)
	{
		oss->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	}
	else
	{
		oss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	}
	BackendIdUserData::attach(outer.get(), mesh->id());
	out.outer = outer;
	out.modelCenter = center;
	out.diagonal = diagonal;
	return true;
}

void MeshBackendVisual::computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
													  float& outDiagonal) const
{
	const auto* mesh = dynamic_cast<const MeshBackendData*>(&data);
	if (!mesh)
	{
		outCenter = osg::Vec3f(0.0f, 0.0f, 0.0f);
		outDiagonal = 1.0f;
		return;
	}
	outCenter = backend_geometry_metrics::meshCenterFromSoup(mesh->triangleSoup());
	outDiagonal = backend_geometry_metrics::meshDiagonalFromSoup(mesh->triangleSoup());
}
