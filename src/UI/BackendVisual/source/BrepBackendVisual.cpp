/// @file BrepBackendVisual.cpp
/// @brief BrepBackendVisual 实现

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
#include "BrepBackendData.h"
#include "BrepBackendVisual.h"
#include "BackendTypeIds.h"

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <BrepImportArtifacts.h>
#include <Discretize.h>
#include <ShapeHandle.h>
#include <ViewTessellate.h>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/StateSet>

namespace
{
osg::Vec3f centerFromBounds(const BackendBoundingBox& b)
{
	if (!b.valid)
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	return osg::Vec3f(static_cast<float>((b.min.x + b.max.x) * 0.5), static_cast<float>((b.min.y + b.max.y) * 0.5),
					  static_cast<float>((b.min.z + b.max.z) * 0.5));
}

float diagonalFromBounds(const BackendBoundingBox& b)
{
	if (!b.valid)
	{
		return 1.0f;
	}
	const double dx = b.max.x - b.min.x;
	const double dy = b.max.y - b.min.y;
	const double dz = b.max.z - b.min.z;
	return static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
}

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

void fillVec3ArrayFromSoup(const std::vector<float>& soup, osg::Vec3Array& outVerts)
{
	outVerts.clear();
	outVerts.reserve(soup.size() / 3U);
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3U)
	{
		outVerts.push_back(osg::Vec3(soup[i], soup[i + 1], soup[i + 2]));
	}
}

void applySoupToFillGeometry(const std::vector<float>& soup, osg::Geometry& geometry, osg::Vec3Array& vertices,
							 osg::Vec3Array* normalArray, bool useSceneLighting,
							 const std::vector<float>* precomputedNormals = nullptr)
{
	fillVec3ArrayFromSoup(soup, vertices);
	geometry.setVertexArray(&vertices);

	const GLsizei vertCount = static_cast<GLsizei>(vertices.size());
	if (geometry.getNumPrimitiveSets() > 0)
	{
		osg::DrawArrays* draw = dynamic_cast<osg::DrawArrays*>(geometry.getPrimitiveSet(0));
		if (draw)
		{
			draw->setFirst(0);
			draw->setCount(vertCount);
		}
		else
		{
			geometry.removePrimitiveSet(0, geometry.getNumPrimitiveSets());
			geometry.addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertCount));
		}
	}
	else
	{
		geometry.addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertCount));
	}

	if (useSceneLighting && normalArray)
	{
		if (precomputedNormals && precomputedNormals->size() == soup.size())
		{
			fillVec3ArrayFromSoup(*precomputedNormals, *normalArray);
		}
		else
		{
			std::vector<float> computed;
			geoalgo::computeTriangleSoupNormals(soup, computed);
			fillVec3ArrayFromSoup(computed, *normalArray);
		}
		geometry.setNormalArray(normalArray, osg::Array::BIND_PER_VERTEX);
		normalArray->dirty();
	}

	vertices.dirty();
	geometry.dirtyBound();
}

osg::ref_ptr<osg::Geode> buildBrepEdgeWireGeode(const std::vector<std::vector<float>>& edgePolylines,
												const osg::Vec4& fillColor, const MeshVisualOptions& opt)
{
	if (!opt.showWireOutline || edgePolylines.empty())
	{
		return nullptr;
	}

	osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
	for (const std::vector<float>& pl : edgePolylines)
	{
		const std::size_t nPts = pl.size() / 3U;
		if (nPts < 2U)
		{
			continue;
		}
		for (std::size_t i = 0; i + 2 < pl.size(); i += 3U)
		{
			verts->push_back(osg::Vec3(pl[i], pl[i + 1], pl[i + 2]));
		}
	}
	if (verts->size() < 2U)
	{
		return nullptr;
	}

	osg::ref_ptr<osg::Geometry> geometryWire = new osg::Geometry;
	geometryWire->setUseDisplayList(false);
	geometryWire->setUseVertexBufferObjects(true);
	geometryWire->setVertexArray(verts.get());

	GLsizei drawFirst = 0;
	for (const std::vector<float>& pl : edgePolylines)
	{
		const std::size_t nPts = pl.size() / 3U;
		if (nPts < 2U)
		{
			continue;
		}
		geometryWire->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, drawFirst, static_cast<GLsizei>(nPts)));
		drawFirst += static_cast<GLsizei>(nPts);
	}
	if (geometryWire->getNumPrimitiveSets() == 0)
	{
		return nullptr;
	}

	osg::ref_ptr<osg::Vec4Array> wc = new osg::Vec4Array;
	const float dim = 0.35f;
	wc->push_back(osg::Vec4(fillColor.r() * dim, fillColor.g() * dim, fillColor.b() * dim, fillColor.a()));
	geometryWire->setColorArray(wc.get(), osg::Array::BIND_OVERALL);

	osg::ref_ptr<osg::Geode> geodeWire = new osg::Geode;
	geodeWire->setName("brepWireOverlay");
	geodeWire->setNodeMask(0x2u);
	geodeWire->addDrawable(geometryWire.get());
	osg::StateSet* ssWire = geodeWire->getOrCreateStateSet();
	ssWire->setAttributeAndModes(new osg::PolygonOffset(-1.0f, -1.0f),
								 osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ssWire->setAttributeAndModes(new osg::LineWidth(1.0f));
	ssWire->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	return geodeWire;
}

osg::ref_ptr<osg::Node> buildBrepDisplayNode(const BrepBackendData& data, const MeshVisualOptions& opt,
											 geoalgo::BrepImportArtifacts& artifacts, std::string* errorMessage)
{
	const geoalgo::ShapeHandle& shape = data.shapeRef();
	if (shape.isNull())
	{
		if (errorMessage)
		{
			*errorMessage = "Empty B-rep shape.";
		}
		return nullptr;
	}

	const std::vector<float>& soup = artifacts.displaySoup;
	if (soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		if (errorMessage)
		{
			*errorMessage = "B-rep tessellation produced empty mesh.";
		}
		return nullptr;
	}

	const BackendColor c = data.color();
	const osg::Vec4 fillColor(c.r, c.g, c.b, c.a);

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> normals = opt.useSceneLighting ? new osg::Vec3Array : nullptr;
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setUseDisplayList(false);
	geometry->setUseVertexBufferObjects(true);
	const std::vector<float>* preNormals =
		artifacts.displayNormals.size() == soup.size() ? &artifacts.displayNormals : nullptr;
	applySoupToFillGeometry(soup, *geometry, *vertices, normals.get(), opt.useSceneLighting, preNormals);

	const std::unordered_map<int, BackendColor>& faceHighlights = data.faceHighlightColors();
	const bool useFaceHighlights = !faceHighlights.empty() && !artifacts.triangleFaceIndex.empty();
	if (useFaceHighlights)
	{
		const std::size_t triCount = soup.size() / 9U;
		osg::ref_ptr<osg::Vec4Array> vertexColors = new osg::Vec4Array;
		vertexColors->reserve(triCount * 3U);
		for (std::size_t ti = 0; ti < triCount; ++ti)
		{
			osg::Vec4 c(fillColor);
			if (ti < artifacts.triangleFaceIndex.size())
			{
				const auto it = faceHighlights.find(artifacts.triangleFaceIndex[ti]);
				if (it != faceHighlights.end())
				{
					c = osg::Vec4(it->second.r, it->second.g, it->second.b, it->second.a);
				}
			}
			vertexColors->push_back(c);
			vertexColors->push_back(c);
			vertexColors->push_back(c);
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
	if (useFaceHighlights)
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

	osg::ref_ptr<osg::Group> grp = new osg::Group;
	grp->addChild(geodeFill.get());

	if (opt.showWireOutline)
	{
		std::string pickErr;
		(void)geoalgo::ensureBrepImportPickArtifacts(shape, artifacts, errorMessage ? &pickErr : nullptr);
		osg::ref_ptr<osg::Geode> wireGeode = buildBrepEdgeWireGeode(artifacts.edgePolylines, fillColor, opt);
		if (wireGeode.valid())
		{
			grp->addChild(wireGeode.get());
		}
	}

	return grp;
}

} // namespace

std::string BrepBackendVisual::typeKey() const
{
	return backend_type::kClassBrepModel;
}

bool BrepBackendVisual::buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions,
										 BranchBuildResult& out, std::string* errorMessage)
{
	const auto* brep = dynamic_cast<const BrepBackendData*>(&data);
	if (!brep || !brep->hasGeometry())
	{
		if (errorMessage)
		{
			*errorMessage = "Backend type mismatch or empty B-rep (expected BrepBackendData).";
		}
		return false;
	}
	std::string artifactErr;
	const std::shared_ptr<geoalgo::BrepImportArtifacts> artifacts =
		geoalgo::getOrBuildBrepImportArtifacts(brep->shapeRef(), errorMessage ? &artifactErr : nullptr);
	if (!artifacts)
	{
		if (errorMessage)
		{
			*errorMessage = artifactErr.empty() ? "B-rep import artifacts failed." : artifactErr;
		}
		return false;
	}
	osg::ref_ptr<osg::Node> root = buildBrepDisplayNode(*brep, meshOptions, *artifacts, errorMessage);
	if (!root)
	{
		return false;
	}
	const BackendBoundingBox bb = brep->geometryBounds();
	const osg::Vec3f center = centerFromBounds(bb);
	const float diagonal = diagonalFromBounds(bb);
	osg::ref_ptr<osg::PositionAttitudeTransform> inner = new osg::PositionAttitudeTransform;
	inner->setPosition(osg::Vec3f(0.0f, 0.0f, 0.0f));
	inner->addChild(root.get());
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
	BackendIdUserData::attachBrep(outer.get(), brep->id(), brep->shapeRef());
	out.outer = outer;
	out.modelCenter = center;
	out.diagonal = diagonal;
	out.brepArtifacts = artifacts;
	return true;
}

void BrepBackendVisual::computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
													  float& outDiagonal) const
{
	const auto* brep = dynamic_cast<const BrepBackendData*>(&data);
	if (!brep)
	{
		outCenter = osg::Vec3f(0.0f, 0.0f, 0.0f);
		outDiagonal = 1.0f;
		return;
	}
	outCenter = centerFromBounds(brep->geometryBounds());
	outDiagonal = diagonalFromBounds(brep->geometryBounds());
}
