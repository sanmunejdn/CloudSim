/// @file OsgWidgetPickAnnotationController.cpp
/// @brief Smaller than compass gizmo. Pure linear-in-diagonal matches large scenes; the old

#include "OsgWidgetPickAnnotationController.h"

#include "ObjectGizmoFrame.h"
#include "OsgWidget.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

#include <osg/Array>
#include <osg/AutoTransform>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osg/PrimitiveSet>
#include <osg/ShapeDrawable>
#include <osgText/Text>

namespace
{
static void backendOuterLocalPosQuat(const osg::MatrixTransform* mt, osg::Vec3f& pos, osg::Quat& q)
{
	osg::Vec3d t;
	osg::Vec3d s;
	osg::Quat so;
	mt->getMatrix().decompose(t, q, s, so);
	pos.set(static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z()));
}

/// Smaller than compass gizmo. Pure linear-in-diagonal matches large scenes; the old
/// max(12, diagonal*k)/220 collapsed almost all small diagonals to the same tiny min clamp.
float annotationScaleForDiagonal(float diagonal)
{
	if (!(diagonal > 0.0f) || diagonal != diagonal)
	{
		diagonal = 1.0f;
	}
	// Linear in diagonal; floor is for tiny scenes where linear term is still unreadably small.
	const float s = diagonal * 0.00014f;
	return std::max(0.38f, std::min(2.5f, s));
}

} // namespace

void OsgWidgetPickAnnotationController::updatePointPickMarker(OsgWidget& self, const osg::Vec3f& pointWorld, bool hit)
{
	if (!self.m_pickFeedbackTransform.valid())
	{
		return;
	}
	if (!self.m_pickFeedbackNode.valid())
	{
		osg::ref_ptr<osg::Geode> geode = new osg::Geode;

		const float radius = 5.0f;
		const int seg = 40;
		osg::ref_ptr<osg::Vec3Array> circle = new osg::Vec3Array;
		for (int i = 0; i <= seg; ++i)
		{
			const float a = osg::PI * 2.0f * static_cast<float>(i) / static_cast<float>(seg);
			circle->push_back(osg::Vec3(std::cos(a) * radius, std::sin(a) * radius, 0.0f));
		}
		osg::ref_ptr<osg::Geometry> ring = new osg::Geometry;
		ring->setVertexArray(circle.get());
		osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
		color->push_back(osg::Vec4(0.2f, 1.0f, 0.2f, 1.0f));
		ring->setColorArray(color.get(), osg::Array::BIND_OVERALL);
		ring->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(circle->size())));
		geode->addDrawable(ring.get());
		geode->getOrCreateStateSet()->setAttribute(new osg::LineWidth(3.0f));
		geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		self.m_pickFeedbackNode = geode.get();
		self.m_pickFeedbackTransform->addChild(self.m_pickFeedbackNode.get());
	}

	osg::Geode* geode = dynamic_cast<osg::Geode*>(self.m_pickFeedbackNode.get());
	if (geode && geode->getNumDrawables() > 0)
	{
		osg::Geometry* ring = geode->getDrawable(0) ? geode->getDrawable(0)->asGeometry() : nullptr;
		if (ring)
		{
			osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(ring->getColorArray());
			if (colors && !colors->empty())
			{
				(*colors)[0] = hit ? osg::Vec4(0.2f, 1.0f, 0.2f, 1.0f) : osg::Vec4(1.0f, 0.3f, 0.3f, 1.0f);
				colors->dirty();
			}
		}
	}

	osg::Vec3f pivotW(0.0f, 0.0f, 0.0f);
	self.computeGizmoPivotWorld(pivotW);
	ObjectGizmoFrame gf;
	const osg::Quat invAtt = self.readActiveObjectGizmoFrame(gf) ? gf.attitude().inverse() : osg::Quat();
	const osg::Vec3f localPos = invAtt * (pointWorld - pivotW);
	self.m_pickFeedbackTransform->setPosition(localPos);
	self.m_pickFeedbackTransform->setNodeMask(0xffffffffu);
}

void OsgWidgetPickAnnotationController::clearPointPickMarker(OsgWidget& self)
{
	if (self.m_pickFeedbackTransform.valid())
	{
		self.m_pickFeedbackTransform->setNodeMask(0u);
	}
}

void OsgWidgetPickAnnotationController::addPointAnnotation(OsgWidget& self, const osg::Vec3f& pointWorld)
{
	addPointAnnotationForBackend(self, pointWorld, QString::fromStdString(self.m_activeBackendId));
}

void OsgWidgetPickAnnotationController::addPointAnnotationForBackend(OsgWidget& self, const osg::Vec3f& pointWorld,
																	 const QString& backendId)
{
	const auto resolveTopVisualBackendId = [&](const QString& startBackendId) -> QString
	{
		if (startBackendId.isEmpty())
		{
			return QString();
		}
		std::string cur = startBackendId.toStdString();
		std::string bestVisual;
		std::unordered_set<std::string> visited;
		for (int depth = 0; depth < 1024 && !cur.empty(); ++depth)
		{
			if (!visited.insert(cur).second)
			{
				break;
			}
			auto visIt = self.m_backendObjectRoots.find(cur);
			if (visIt != self.m_backendObjectRoots.end() && visIt->second.valid())
			{
				bestVisual = cur;
			}
			auto pIt = self.m_backendParentIds.find(cur);
			if (pIt == self.m_backendParentIds.end() || pIt->second.empty())
			{
				break;
			}
			cur = pIt->second;
		}
		if (!bestVisual.empty())
		{
			return QString::fromStdString(bestVisual);
		}

		// Fallback for assembly/group parents without direct geometry:
		// choose a visible descendant branch so annotation remains attached
		// to the transformed hierarchy instead of freezing in world space.
		const std::string start = startBackendId.toStdString();
		std::string fallbackVisual;
		std::vector<std::string> queue;
		queue.push_back(start);
		std::size_t qi = 0;
		while (qi < queue.size())
		{
			const std::string parent = queue[qi++];
			for (const auto& rel : self.m_backendParentIds)
			{
				if (rel.second != parent)
				{
					continue;
				}
				const std::string& child = rel.first;
				if (!visited.insert(child).second)
				{
					continue;
				}
				auto visIt = self.m_backendObjectRoots.find(child);
				if (visIt != self.m_backendObjectRoots.end() && visIt->second.valid())
				{
					fallbackVisual = child;
					break;
				}
				queue.push_back(child);
			}
			if (!fallbackVisual.empty())
			{
				break;
			}
		}
		return fallbackVisual.empty() ? QString() : QString::fromStdString(fallbackVisual);
	};

	const QString trackedBackendId = resolveTopVisualBackendId(backendId);
	osg::MatrixTransform* trackedOuter = nullptr;
	if (!trackedBackendId.isEmpty())
	{
		auto it = self.m_backendObjectRoots.find(trackedBackendId.toStdString());
		if (it != self.m_backendObjectRoots.end() && it->second.valid())
		{
			trackedOuter = it->second.get();
		}
	}

	// Marker visuals (independent of where we attach in the scene graph).
	osg::ref_ptr<osg::Geode> markerGeode = new osg::Geode;
	markerGeode->setCullingActive(false);
	osg::ref_ptr<osg::Sphere> markerSphere = new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 4.5f);
	osg::ref_ptr<osg::ShapeDrawable> markerDrawable = new osg::ShapeDrawable(markerSphere.get());
	markerDrawable->setColor(osg::Vec4(1.0f, 0.85f, 0.2f, 1.0f));
	markerGeode->addDrawable(markerDrawable.get());

	osg::ref_ptr<osg::Vec3Array> lineVerts = new osg::Vec3Array;
	lineVerts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	lineVerts->push_back(osg::Vec3(16.0f, 18.0f, 0.0f));
	osg::ref_ptr<osg::Geometry> lineGeom = new osg::Geometry;
	lineGeom->setVertexArray(lineVerts.get());
	osg::ref_ptr<osg::Vec4Array> lineColor = new osg::Vec4Array;
	lineColor->push_back(osg::Vec4(1.0f, 0.9f, 0.3f, 1.0f));
	lineGeom->setColorArray(lineColor.get(), osg::Array::BIND_OVERALL);
	lineGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
	markerGeode->addDrawable(lineGeom.get());

	const QString annotationId = QStringLiteral("P%1").arg(++self.m_annotationCounter);
	const QString text = QStringLiteral("%1 (%2, %3, %4)")
							 .arg(annotationId)
							 .arg(pointWorld.x(), 0, 'f', 3)
							 .arg(pointWorld.y(), 0, 'f', 3)
							 .arg(pointWorld.z(), 0, 'f', 3);
	osg::ref_ptr<osgText::Text> label = new osgText::Text;
	label->setFont("C:/Windows/Fonts/msyh.ttc");
	label->setCharacterSize(20.0f);
	label->setFontResolution(48, 48);
	label->setColor(self.m_darkUiTheme ? osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) : osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	label->setBackdropType(osgText::Text::NONE);
	label->setDataVariance(osg::Object::DYNAMIC);
	label->setAlignment(osgText::TextBase::LEFT_BOTTOM);
	label->setPosition(osg::Vec3(18.0f, 18.0f, 0.0f));
	label->setText(text.toStdString());
	markerGeode->addDrawable(label.get());

	osg::StateSet* ss = markerGeode->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setWriteMask(false);
	ss->setAttributeAndModes(depth.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setAttributeAndModes(new osg::LineWidth(2.0f), osg::StateAttribute::ON);
	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	osg::Vec3f localAnchor = pointWorld;
	if (trackedOuter)
	{
		osg::Vec3f backendPos;
		osg::Quat backendAtt;
		backendOuterLocalPosQuat(trackedOuter, backendPos, backendAtt);
		const osg::Quat invAtt = backendAtt.inverse();
		localAnchor = invAtt * (pointWorld - backendPos);
	}

	const float annScale = annotationScaleForDiagonal(self.m_activeModelDiagonal);
	osg::ref_ptr<osg::MatrixTransform> scaleMt = new osg::MatrixTransform;
	scaleMt->setMatrix(osg::Matrix::scale(static_cast<double>(annScale), static_cast<double>(annScale),
										  static_cast<double>(annScale)));
	scaleMt->addChild(markerGeode.get());

	osg::ref_ptr<osg::AutoTransform> at = new osg::AutoTransform;
	// Annotation nodes are attached to the top-level annotation group (world space),
	// so initial placement must use world coordinates.
	at->setPosition(pointWorld);
	at->setNodeMask(0xffffffffu);
	at->setCullingActive(false);
	at->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
	at->setAutoScaleToScreen(true);
	// Screen-space auto scale; cap multiplier so large models do not inflate too much.
	const float autoClamp = std::max(0.2f, std::min(1.0f, annScale));
	at->setMinimumScale(1.2f * autoClamp);
	at->setMaximumScale(22.0f * autoClamp);
	at->addChild(scaleMt.get());

	if (self.m_annotationGroup.valid())
	{
		self.m_annotationGroup->addChild(at.get());
		self.m_annotationGroup->dirtyBound();
	}
	else
	{
		emit self.pointPickFeedback(QStringLiteral("[ANNOT] create failed: annotation group is null"));
	}

	auto entry = OsgWidget::AnnotationEntry{};
	entry.id = annotationId.toStdString();
	entry.displayText = text.toStdString();
	entry.transform = at;
	entry.scaleBranch = scaleMt;
	entry.textDrawable = label;
	entry.backendId = trackedBackendId.toStdString();
	entry.worldAnchor = pointWorld;
	entry.hasWorldAnchor = true;
	entry.localCentered = localAnchor;
	entry.visible = true;
	self.m_annotations.push_back(entry);
	emit self.pointPickFeedback(
		QStringLiteral("[ANNOT] created id=%1 world=(%2,%3,%4) tracked=%5 local=(%6,%7,%8) count=%9 groupMask=0x%10 "
					   "nodeMask=0x%11")
			.arg(QString::fromStdString(entry.id))
			.arg(pointWorld.x(), 0, 'f', 3)
			.arg(pointWorld.y(), 0, 'f', 3)
			.arg(pointWorld.z(), 0, 'f', 3)
			.arg(entry.backendId.empty() ? QStringLiteral("<none>") : QString::fromStdString(entry.backendId))
			.arg(localAnchor.x(), 0, 'f', 3)
			.arg(localAnchor.y(), 0, 'f', 3)
			.arg(localAnchor.z(), 0, 'f', 3)
			.arg(static_cast<int>(self.m_annotations.size()))
			.arg(self.m_annotationGroup.valid() ? QString::number(self.m_annotationGroup->getNodeMask(), 16)
												: QStringLiteral("0"))
			.arg(QString::number(at->getNodeMask(), 16)));
	emit self.annotationCreated(QString::fromStdString(entry.id), QString::fromStdString(entry.displayText));
}

void OsgWidgetPickAnnotationController::refreshAnnotationTexts(OsgWidget& self)
{
	for (auto& a : self.m_annotations)
	{
		if (!a.transform.valid() || !a.textDrawable.valid())
		{
			emit self.pointPickFeedback(QStringLiteral("[ANNOT] refresh skip id=%1 reason=invalid transform/text")
											.arg(QString::fromStdString(a.id)));
			continue;
		}
		osg::Vec3f world = osg::Vec3f(0.0f, 0.0f, 0.0f);
		if (!a.backendId.empty())
		{
			const std::string& backendStd = a.backendId;
			auto it = self.m_backendObjectRoots.find(backendStd);
			if (it != self.m_backendObjectRoots.end() && it->second.valid())
			{
				osg::Vec3f backendPos;
				osg::Quat backendAtt;
				backendOuterLocalPosQuat(it->second.get(), backendPos, backendAtt);
				world = backendPos + (backendAtt * a.localCentered);
			}
			else if (a.hasWorldAnchor)
			{
				world = a.worldAnchor;
			}
		}
		else
		{
			if (a.hasWorldAnchor)
			{
				world = a.worldAnchor;
			}
			// Legacy fallback: localCentered was stored in gizmo (center+pose) space at save time.
			else if (ObjectGizmoFrame gf; self.readActiveObjectGizmoFrame(gf))
			{
				world = gf.centerPlusPose() + (gf.attitude() * a.localCentered);
			}
		}

		const QString text = QStringLiteral("%1 (%2, %3, %4)")
								 .arg(QString::fromStdString(a.id))
								 .arg(world.x(), 0, 'f', 3)
								 .arg(world.y(), 0, 'f', 3)
								 .arg(world.z(), 0, 'f', 3);
		a.transform->setPosition(world);
		a.transform->dirtyBound();
		a.displayText = text.toStdString();
		a.textDrawable->setText(text.toStdString());
		emit self.pointPickFeedback(QStringLiteral("[ANNOT] refresh id=%1 world=(%2,%3,%4) visible=%5")
										.arg(QString::fromStdString(a.id))
										.arg(world.x(), 0, 'f', 3)
										.arg(world.y(), 0, 'f', 3)
										.arg(world.z(), 0, 'f', 3)
										.arg(a.visible ? QStringLiteral("true") : QStringLiteral("false")));
	}
}

void OsgWidgetPickAnnotationController::updateAnnotationScales(OsgWidget& self)
{
	const float s = annotationScaleForDiagonal(self.m_activeModelDiagonal);
	const float autoClamp = std::max(0.2f, std::min(1.0f, s));
	for (auto& a : self.m_annotations)
	{
		if (a.scaleBranch.valid())
		{
			a.scaleBranch->setMatrix(
				osg::Matrix::scale(static_cast<double>(s), static_cast<double>(s), static_cast<double>(s)));
		}
		if (a.transform.valid())
		{
			a.transform->setMinimumScale(1.2f * autoClamp);
			a.transform->setMaximumScale(22.0f * autoClamp);
		}
	}
}

bool OsgWidgetPickAnnotationController::setAnnotationVisible(OsgWidget& self, const QString& annotationId, bool visible)
{
	for (auto& a : self.m_annotations)
	{
		if (a.id == annotationId.toStdString() && a.transform.valid())
		{
			a.visible = visible;
			a.transform->setNodeMask(visible ? 0xffffffffu : 0u);
			emit self.annotationVisibilityChanged(annotationId, visible);
			return true;
		}
	}
	return false;
}

bool OsgWidgetPickAnnotationController::removeAnnotation(OsgWidget& self, const QString& annotationId)
{
	for (std::size_t i = 0; i < self.m_annotations.size(); ++i)
	{
		auto& a = self.m_annotations[i];
		if (a.id == annotationId.toStdString())
		{
			if (a.transform.valid())
			{
				if (self.m_annotationGroup.valid())
				{
					self.m_annotationGroup->removeChild(a.transform.get());
				}
			}
			emit self.annotationRemoved(annotationId);
			self.m_annotations.erase(self.m_annotations.begin() + static_cast<std::ptrdiff_t>(i));
			return true;
		}
	}
	return false;
}

void OsgWidgetPickAnnotationController::clearAllAnnotations(OsgWidget& self)
{
	self.clearPointAnnotations();
}

QList<OsgWidget::AnnotationSnapshot> OsgWidgetPickAnnotationController::annotationSnapshots(const OsgWidget& self) const
{
	QList<OsgWidget::AnnotationSnapshot> snapshots;
	snapshots.reserve(static_cast<int>(self.m_annotations.size()));
	for (const auto& a : self.m_annotations)
	{
		OsgWidget::AnnotationSnapshot s;
		s.id = QString::fromStdString(a.id);
		s.displayText = QString::fromStdString(a.displayText);
		s.backendId = QString::fromStdString(a.backendId);
		s.localCentered = a.localCentered;
		s.worldAnchor = a.worldAnchor;
		s.hasWorldAnchor = a.hasWorldAnchor;
		s.visible = a.visible;
		snapshots.push_back(s);
	}
	return snapshots;
}

void OsgWidgetPickAnnotationController::restoreAnnotations(OsgWidget& self,
														   const QList<OsgWidget::AnnotationSnapshot>& snapshots)
{
	clearAllAnnotations(self);
	for (const auto& s : snapshots)
	{
		osg::Vec3f world = osg::Vec3f(0.0f, 0.0f, 0.0f);
		bool worldResolved = false;

		if (!s.backendId.isEmpty())
		{
			const std::string backendStd = s.backendId.toStdString();
			auto it = self.m_backendObjectRoots.find(backendStd);
			if (it != self.m_backendObjectRoots.end() && it->second.valid())
			{
				osg::Vec3f backendPos;
				osg::Quat backendAtt;
				backendOuterLocalPosQuat(it->second.get(), backendPos, backendAtt);
				if (s.hasWorldAnchor)
				{
					world = s.worldAnchor;
				}
				else
				{
					world = backendPos + (backendAtt * s.localCentered);
				}
				worldResolved = true;
			}
		}

		if (!worldResolved)
		{
			if (s.hasWorldAnchor)
			{
				world = s.worldAnchor;
			}
			else if (ObjectGizmoFrame gf; self.readActiveObjectGizmoFrame(gf))
			{
				world = gf.centerPlusPose() + (gf.attitude() * s.localCentered);
			}
			else
			{
				continue;
			}
		}

		// Always attach legacy annotations to some backend branch.
		// If backendId is missing (old project), we fall back to current active backend.
		const QString backendIdUsed =
			!s.backendId.isEmpty() ? s.backendId : QString::fromStdString(self.m_activeBackendId);
		if (!backendIdUsed.isEmpty())
		{
			addPointAnnotationForBackend(self, world, backendIdUsed);
		}
		else
		{
			addPointAnnotation(self, world);
		}

		if (!self.m_annotations.empty())
		{
			auto& last = self.m_annotations.back();

			if (!s.id.isEmpty())
				last.id = s.id.toStdString();
			last.worldAnchor = s.worldAnchor;
			last.hasWorldAnchor = s.hasWorldAnchor;

			// Let refreshAnnotationTexts() rewrite coordinate string.
			refreshAnnotationTexts(self);
			setAnnotationVisible(self, QString::fromStdString(last.id), s.visible);
		}
	}
}
