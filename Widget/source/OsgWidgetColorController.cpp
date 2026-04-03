#include "OsgWidgetColorController.h"

#include "OsgWidget.h"
#include "LitMeshMaterial.h"

#include <algorithm>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/NodeVisitor>
#include <osg/StateSet>
#include <osg/Vec4>

namespace {

void paintOverallColorOnNode(osg::Node* root, const osg::Vec4& color, bool useSceneLighting)
{
	if (!root)
	{
		return;
	}
	struct ColorVisitor : public osg::NodeVisitor
	{
		explicit ColorVisitor(const osg::Vec4& c)
			: osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), clr(c) {}

		void apply(osg::Geode& geode) override
		{
			const bool wireOverlay = (geode.getName() == "meshWireOverlay");
			const osg::Vec4 c = wireOverlay
				? osg::Vec4(
					std::max(0.12f, clr.r() * 0.38f),
					std::max(0.12f, clr.g() * 0.38f),
					std::max(0.12f, clr.b() * 0.38f),
					clr.a())
				: clr;
			for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
			{
				osg::Drawable* drawable = geode.getDrawable(i);
				osg::Geometry* geom = drawable ? drawable->asGeometry() : nullptr;
				if (!geom)
				{
					continue;
				}
				osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
				colors->push_back(c);
				geom->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
				geom->dirtyDisplayList();
				geom->dirtyBound();
			}
			traverse(geode);
		}

		osg::Vec4 clr;
	};
	ColorVisitor visitor(color);
	root->accept(visitor);
	osg::StateSet* ss = root->getOrCreateStateSet();
	osg::ref_ptr<osg::Material> material = new osg::Material;
	if (useSceneLighting)
	{
		LitMeshMaterial::applyPlastic(*material, color);
		ss->setAttributeAndModes(material.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
		ss->setMode(GL_COLOR_MATERIAL, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		ss->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
		ss->setMode(GL_LIGHT0, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
		ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	}
	else
	{
		material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
		material->setAmbient(osg::Material::FRONT_AND_BACK, color * 0.3f);
		ss->setAttributeAndModes(material.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
		ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	}
}

} // namespace

void OsgWidgetColorController::applyColorToStagingGeometry(OsgWidget& self, const osg::Vec4& color)
{
	osg::Node* n = self.stagingGeometryRoot();
	if (n)
	{
		paintOverallColorOnNode(n, color, false);
	}
}

void OsgWidgetColorController::applyColorToBackendObject(OsgWidget& self, const std::string& backendId, const osg::Vec4& color)
{
	if (backendId.empty())
	{
		return;
	}
	for (const auto& kv : self.m_backendObjectRoots)
	{
		if (!kv.second.valid())
		{
			continue;
		}
		if (!self.isBackendDescendantOf(kv.first, backendId))
		{
			continue;
		}
		osg::PositionAttitudeTransform* outer = kv.second.get();
		if (!outer || outer->getNumChildren() < 1)
		{
			continue;
		}
		auto* inner = dynamic_cast<osg::PositionAttitudeTransform*>(outer->getChild(0));
		if (!inner || inner->getNumChildren() < 1)
		{
			continue;
		}
		paintOverallColorOnNode(inner->getChild(0), color, self.isBackendMeshLit(kv.first));
	}
}

void OsgWidgetColorController::applyColorToActiveBackendObject(OsgWidget& self, const osg::Vec4& color)
{
	if (self.m_activeBackendId.empty())
	{
		applyColorToStagingGeometry(self, color);
		return;
	}
	applyColorToBackendObject(self, self.m_activeBackendId, color);
}
