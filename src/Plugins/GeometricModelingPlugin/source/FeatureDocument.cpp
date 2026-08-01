/// @file FeatureDocument.cpp

#include "FeatureDocument.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>

#include <algorithm>

QString FeatureDocument::nextId(const char* prefix)
{
	return QStringLiteral("%1_%2").arg(QLatin1String(prefix)).arg(m_seq++);
}

QString FeatureDocument::addSketch(const PluginSketchPlane& plane, const QString& name, const QString& datumPlaneId)
{
	GeomodelingFeature f;
	f.id = nextId("Sketch");
	f.name = name.isEmpty() ? f.id : name;
	f.kind = GeomodelingFeatureKind::Sketch;
	f.plane = plane;
	f.datumPlaneId = datumPlaneId;
	m_features.push_back(f);
	return f.id;
}

QString FeatureDocument::addDatumPlane(const PluginSketchPlane& plane, const QString& name)
{
	GeomodelingFeature f;
	f.id = nextId("DatumPlane");
	f.name = name.isEmpty() ? f.id : name;
	f.kind = GeomodelingFeatureKind::DatumPlane;
	f.plane = plane;
	m_features.push_back(f);
	return f.id;
}

QString FeatureDocument::addDatumPlaneOffset(const PluginSketchPlane& plane, GeomodelingDatumSourceKind sourceKind,
											 double offsetMm, int originPlaneIndex, const QString& faceBackendId,
											 int faceIndex, const QString& name)
{
	GeomodelingFeature f;
	f.id = nextId("DatumPlane");
	f.name = name.isEmpty() ? f.id : name;
	f.kind = GeomodelingFeatureKind::DatumPlane;
	f.plane = plane;
	f.datumSourceKind = sourceKind;
	f.datumOffsetMm = offsetMm;
	f.datumOriginPlaneIndex = originPlaneIndex;
	f.datumFaceBackendId = faceBackendId;
	f.datumFaceIndex = faceIndex;
	m_features.push_back(f);
	return f.id;
}

QString FeatureDocument::addDatumPlaneAngle(const PluginSketchPlane& plane, double angleDeg,
										   const PluginPoint3d& hingeOrigin, const PluginPoint3d& hingeDir,
										   const QString& name)
{
	GeomodelingFeature f;
	f.id = nextId("DatumPlane");
	f.name = name.isEmpty() ? f.id : name;
	f.kind = GeomodelingFeatureKind::DatumPlaneAngle;
	f.plane = plane;
	f.datumAngleDeg = angleDeg;
	f.datumHingeOrigin = hingeOrigin;
	f.datumHingeDir = hingeDir;
	m_features.push_back(f);
	return f.id;
}

QString FeatureDocument::addPad(const QString& sketchId, double lengthMm, bool reversed)
{
	GeomodelingFeature f;
	f.id = nextId("Pad");
	f.name = f.id;
	f.kind = GeomodelingFeatureKind::Pad;
	f.sketchRefId = sketchId;
	f.lengthMm = lengthMm;
	f.reversed = reversed;
	m_features.push_back(f);
	return f.id;
}

QString FeatureDocument::addPocket(const QString& sketchId, double lengthMm, bool reversed)
{
	GeomodelingFeature f;
	f.id = nextId("Pocket");
	f.name = f.id;
	f.kind = GeomodelingFeatureKind::Pocket;
	f.sketchRefId = sketchId;
	f.lengthMm = lengthMm;
	f.reversed = reversed;
	m_features.push_back(f);
	return f.id;
}

bool FeatureDocument::setProfile(const QString& sketchId, const std::vector<float>& xyz)
{
	if (auto* f = find(sketchId))
	{
		f->profileXyzMm = xyz;
		return true;
	}
	return false;
}

bool FeatureDocument::setSketchDocument(const QString& sketchId, const QByteArray& utf8)
{
	if (auto* f = find(sketchId))
	{
		f->sketchDocumentUtf8 = utf8;
		return true;
	}
	return false;
}

bool FeatureDocument::setVisible(const QString& featureId, bool visible)
{
	if (auto* f = find(featureId))
	{
		f->visible = visible;
		return true;
	}
	return false;
}

bool FeatureDocument::setLength(const QString& featureId, double lengthMm)
{
	if (auto* f = find(featureId))
	{
		f->lengthMm = lengthMm;
		return true;
	}
	return false;
}

bool FeatureDocument::setDraftAngleDeg(const QString& featureId, double draftAngleDeg)
{
	if (auto* f = find(featureId))
	{
		f->draftAngleDeg = draftAngleDeg;
		return true;
	}
	return false;
}

bool FeatureDocument::setReversed(const QString& featureId, bool reversed)
{
	if (auto* f = find(featureId))
	{
		f->reversed = reversed;
		return true;
	}
	return false;
}

bool FeatureDocument::setExtrudeEnd(const QString& featureId, GeomodelingExtrudeEnd end,
								   const PluginSketchPlane* upToFace, const QString& upToFaceBackendId,
								   int upToFaceIndex)
{
	if (auto* f = find(featureId))
	{
		f->endCondition = end;
		if (end == GeomodelingExtrudeEnd::UpToFace && upToFace && upToFace->isPlanar)
		{
			f->upToFacePlane = *upToFace;
			f->hasUpToFacePlane = true;
			f->upToFaceBackendId = upToFaceBackendId;
			f->upToFaceIndex = upToFaceIndex;
		}
		else
		{
			f->hasUpToFacePlane = false;
			f->upToFaceBackendId.clear();
			f->upToFaceIndex = -1;
		}
		return true;
	}
	return false;
}

bool FeatureDocument::removeFeature(const QString& featureId)
{
	const GeomodelingFeature* target = find(featureId);
	if (!target)
		return false;

	std::vector<QString> removeIds;
	removeIds.push_back(featureId);
	if (target->kind == GeomodelingFeatureKind::Sketch)
	{
		for (const auto& f : m_features)
		{
			if (f.id == featureId)
				continue;
			if (f.sketchRefId == featureId || f.pathSketchRefId == featureId || f.loftSketchRefId == featureId)
				removeIds.push_back(f.id);
		}
	}

	m_features.erase(std::remove_if(m_features.begin(), m_features.end(),
									[&](const GeomodelingFeature& f)
									{
										for (const QString& id : removeIds)
										{
											if (f.id == id)
												return true;
										}
										return false;
									}),
					 m_features.end());

	for (const QString& id : removeIds)
	{
		if (m_rollbackAfterFeatureId == id)
		{
			clearRollback();
			break;
		}
	}
	return true;
}

bool FeatureDocument::applyRollbackTo(const QString& featureId)
{
	int idx = -1;
	for (int i = 0; i < static_cast<int>(m_features.size()); ++i)
	{
		if (m_features[static_cast<std::size_t>(i)].id == featureId)
		{
			idx = i;
			break;
		}
	}
	if (idx < 0)
		return false;
	for (int i = 0; i < static_cast<int>(m_features.size()); ++i)
		m_features[static_cast<std::size_t>(i)].suppressed = (i > idx);
	m_rollbackAfterFeatureId = featureId;
	return true;
}

void FeatureDocument::clearRollback()
{
	for (auto& f : m_features)
		f.suppressed = false;
	m_rollbackAfterFeatureId.clear();
}

GeomodelingFeature* FeatureDocument::find(const QString& id)
{
	for (auto& f : m_features)
		if (f.id == id)
			return &f;
	return nullptr;
}

const GeomodelingFeature* FeatureDocument::find(const QString& id) const
{
	for (const auto& f : m_features)
		if (f.id == id)
			return &f;
	return nullptr;
}

QJsonObject FeatureDocument::toJson() const
{
	QJsonArray arr;
	for (const auto& f : m_features)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), f.id);
		o.insert(QStringLiteral("name"), f.name);
		o.insert(QStringLiteral("kind"), static_cast<int>(f.kind));
		o.insert(QStringLiteral("lengthMm"), f.lengthMm);
		if (std::abs(f.length2Mm) > 1e-9)
			o.insert(QStringLiteral("length2Mm"), f.length2Mm);
		if (std::abs(f.startOffsetMm) > 1e-9)
			o.insert(QStringLiteral("startOffsetMm"), f.startOffsetMm);
		o.insert(QStringLiteral("draftAngleDeg"), f.draftAngleDeg);
		o.insert(QStringLiteral("reversed"), f.reversed);
		{
			QString endStr = QStringLiteral("Blind");
			if (f.endCondition == GeomodelingExtrudeEnd::UpToFace)
				endStr = QStringLiteral("UpToFace");
			else if (f.endCondition == GeomodelingExtrudeEnd::MidPlane)
				endStr = QStringLiteral("MidPlane");
			else if (f.endCondition == GeomodelingExtrudeEnd::ThroughAll)
				endStr = QStringLiteral("ThroughAll");
			else if (f.endCondition == GeomodelingExtrudeEnd::UpToVertex)
				endStr = QStringLiteral("UpToVertex");
			else if (f.endCondition == GeomodelingExtrudeEnd::OffsetFromFace)
				endStr = QStringLiteral("OffsetFromFace");
			else if (f.endCondition == GeomodelingExtrudeEnd::TwoDirections)
				endStr = QStringLiteral("TwoDirections");
			o.insert(QStringLiteral("endCondition"), endStr);
		}
		if (f.hasUpToFacePlane)
		{
			QJsonObject up;
			up.insert(QStringLiteral("ox"), f.upToFacePlane.origin.x);
			up.insert(QStringLiteral("oy"), f.upToFacePlane.origin.y);
			up.insert(QStringLiteral("oz"), f.upToFacePlane.origin.z);
			up.insert(QStringLiteral("nx"), f.upToFacePlane.normal.x);
			up.insert(QStringLiteral("ny"), f.upToFacePlane.normal.y);
			up.insert(QStringLiteral("nz"), f.upToFacePlane.normal.z);
			up.insert(QStringLiteral("planar"), f.upToFacePlane.isPlanar);
			o.insert(QStringLiteral("upToFacePlane"), up);
		}
		if (!f.upToFaceBackendId.isEmpty())
			o.insert(QStringLiteral("upToFaceBackendId"), f.upToFaceBackendId);
		if (f.upToFaceIndex >= 0)
			o.insert(QStringLiteral("upToFaceIndex"), f.upToFaceIndex);
		if (f.hasUpToVertex)
		{
			QJsonObject vtx;
			vtx.insert(QStringLiteral("x"), f.upToVertex.x);
			vtx.insert(QStringLiteral("y"), f.upToVertex.y);
			vtx.insert(QStringLiteral("z"), f.upToVertex.z);
			o.insert(QStringLiteral("upToVertex"), vtx);
		}
		if (std::abs(f.offsetFromFaceMm) > 1e-9)
			o.insert(QStringLiteral("offsetFromFaceMm"), f.offsetFromFaceMm);
		if (std::abs(f.twistDeg) > 1e-9)
			o.insert(QStringLiteral("twistDeg"), f.twistDeg);
		o.insert(QStringLiteral("sketchRefId"), f.sketchRefId);
		if (!f.pathSketchRefId.isEmpty())
			o.insert(QStringLiteral("pathSketchRefId"), f.pathSketchRefId);
		o.insert(QStringLiteral("resultBackendId"), f.resultBackendId);
		o.insert(QStringLiteral("suppressed"), f.suppressed);
		o.insert(QStringLiteral("visible"), f.visible);
		QJsonObject pl;
		pl.insert(QStringLiteral("ox"), f.plane.origin.x);
		pl.insert(QStringLiteral("oy"), f.plane.origin.y);
		pl.insert(QStringLiteral("oz"), f.plane.origin.z);
		pl.insert(QStringLiteral("nx"), f.plane.normal.x);
		pl.insert(QStringLiteral("ny"), f.plane.normal.y);
		pl.insert(QStringLiteral("nz"), f.plane.normal.z);
		pl.insert(QStringLiteral("xx"), f.plane.axisX.x);
		pl.insert(QStringLiteral("xy"), f.plane.axisX.y);
		pl.insert(QStringLiteral("xz"), f.plane.axisX.z);
		pl.insert(QStringLiteral("yx"), f.plane.axisY.x);
		pl.insert(QStringLiteral("yy"), f.plane.axisY.y);
		pl.insert(QStringLiteral("yz"), f.plane.axisY.z);
		pl.insert(QStringLiteral("planar"), f.plane.isPlanar);
		o.insert(QStringLiteral("plane"), pl);
		QJsonArray xyz;
		for (float v : f.profileXyzMm)
			xyz.append(v);
		o.insert(QStringLiteral("profile"), xyz);
		if (!f.profileHolesXyzMm.empty())
		{
			QJsonArray holes;
			for (const auto& h : f.profileHolesXyzMm)
			{
				QJsonArray ha;
				for (float hv : h)
					ha.append(hv);
				holes.append(ha);
			}
			o.insert(QStringLiteral("profileHoles"), holes);
		}
		if (!f.pathXyzMm.empty())
		{
			QJsonArray path;
			for (float v : f.pathXyzMm)
				path.append(v);
			o.insert(QStringLiteral("path"), path);
		}
		if (!f.pathSegments.empty())
		{
			QJsonArray segs;
			for (const auto& s : f.pathSegments)
			{
				QJsonObject so;
				so.insert(QStringLiteral("kind"), s.kind);
				so.insert(QStringLiteral("a"), QJsonArray{s.ax, s.ay, s.az});
				so.insert(QStringLiteral("b"), QJsonArray{s.bx, s.by, s.bz});
				if (s.kind == 1 || s.kind == 3 || s.kind == 4)
					so.insert(QStringLiteral("m"), QJsonArray{s.mx, s.my, s.mz});
				segs.append(so);
			}
			o.insert(QStringLiteral("pathSegments"), segs);
		}
		if (!f.profileSegments.empty())
		{
			QJsonArray segs;
			for (const auto& s : f.profileSegments)
			{
				QJsonObject so;
				so.insert(QStringLiteral("kind"), s.kind);
				so.insert(QStringLiteral("a"), QJsonArray{s.ax, s.ay, s.az});
				so.insert(QStringLiteral("b"), QJsonArray{s.bx, s.by, s.bz});
				if (s.kind == 1 || s.kind == 3 || s.kind == 4)
					so.insert(QStringLiteral("m"), QJsonArray{s.mx, s.my, s.mz});
				segs.append(so);
			}
			o.insert(QStringLiteral("profileSegments"), segs);
		}
		if (!f.sketchDocumentUtf8.isEmpty())
			o.insert(QStringLiteral("sketchDocument"), QString::fromUtf8(f.sketchDocumentUtf8));
		if (f.datumSourceKind != GeomodelingDatumSourceKind::None)
			o.insert(QStringLiteral("datumSourceKind"), static_cast<int>(f.datumSourceKind));
		if (f.datumSourceKind == GeomodelingDatumSourceKind::OriginPlane)
			o.insert(QStringLiteral("datumOriginPlaneIndex"), f.datumOriginPlaneIndex);
		if (f.datumSourceKind == GeomodelingDatumSourceKind::Face)
		{
			o.insert(QStringLiteral("datumFaceBackendId"), f.datumFaceBackendId);
			o.insert(QStringLiteral("datumFaceIndex"), f.datumFaceIndex);
		}
		if (std::abs(f.datumOffsetMm) > 1e-12 || f.datumSourceKind != GeomodelingDatumSourceKind::None)
			o.insert(QStringLiteral("datumOffsetMm"), f.datumOffsetMm);
		if (f.kind == GeomodelingFeatureKind::DatumPlaneAngle || std::abs(f.datumAngleDeg) > 1e-9)
		{
			o.insert(QStringLiteral("datumAngleDeg"), f.datumAngleDeg);
			QJsonObject hinge;
			hinge.insert(QStringLiteral("ox"), f.datumHingeOrigin.x);
			hinge.insert(QStringLiteral("oy"), f.datumHingeOrigin.y);
			hinge.insert(QStringLiteral("oz"), f.datumHingeOrigin.z);
			hinge.insert(QStringLiteral("dx"), f.datumHingeDir.x);
			hinge.insert(QStringLiteral("dy"), f.datumHingeDir.y);
			hinge.insert(QStringLiteral("dz"), f.datumHingeDir.z);
			o.insert(QStringLiteral("datumHinge"), hinge);
		}
		if (!f.datumPlaneId.isEmpty())
			o.insert(QStringLiteral("datumPlaneId"), f.datumPlaneId);
		arr.append(o);
	}
	QJsonObject root;
	root.insert(QStringLiteral("features"), arr);
	root.insert(QStringLiteral("seq"), m_seq);
	root.insert(QStringLiteral("rollbackAfterFeatureId"), m_rollbackAfterFeatureId);
	return root;
}

void FeatureDocument::fromJson(const QJsonObject& obj)
{
	clear();
	m_seq = obj.value(QStringLiteral("seq")).toInt(1);
	m_rollbackAfterFeatureId = obj.value(QStringLiteral("rollbackAfterFeatureId")).toString();
	const QJsonArray arr = obj.value(QStringLiteral("features")).toArray();
	for (const QJsonValue& v : arr)
	{
		const QJsonObject o = v.toObject();
		GeomodelingFeature f;
		f.id = o.value(QStringLiteral("id")).toString();
		f.name = o.value(QStringLiteral("name")).toString();
		f.kind = static_cast<GeomodelingFeatureKind>(o.value(QStringLiteral("kind")).toInt());
		f.lengthMm = o.value(QStringLiteral("lengthMm")).toDouble(10.0);
		f.length2Mm = o.value(QStringLiteral("length2Mm")).toDouble(0.0);
		f.startOffsetMm = o.value(QStringLiteral("startOffsetMm")).toDouble(0.0);
		f.draftAngleDeg = o.value(QStringLiteral("draftAngleDeg")).toDouble(0.0);
		f.reversed = o.value(QStringLiteral("reversed")).toBool();
		{
			const QString endStr = o.value(QStringLiteral("endCondition")).toString();
			if (endStr == QLatin1String("UpToFace"))
				f.endCondition = GeomodelingExtrudeEnd::UpToFace;
			else if (endStr == QLatin1String("MidPlane"))
				f.endCondition = GeomodelingExtrudeEnd::MidPlane;
			else if (endStr == QLatin1String("ThroughAll"))
				f.endCondition = GeomodelingExtrudeEnd::ThroughAll;
			else if (endStr == QLatin1String("UpToVertex"))
				f.endCondition = GeomodelingExtrudeEnd::UpToVertex;
			else if (endStr == QLatin1String("OffsetFromFace"))
				f.endCondition = GeomodelingExtrudeEnd::OffsetFromFace;
			else if (endStr == QLatin1String("TwoDirections"))
				f.endCondition = GeomodelingExtrudeEnd::TwoDirections;
			else
				f.endCondition = GeomodelingExtrudeEnd::Blind;
		}
		if (o.contains(QStringLiteral("upToFacePlane")))
		{
			const QJsonObject up = o.value(QStringLiteral("upToFacePlane")).toObject();
			f.upToFacePlane.origin = {up.value(QStringLiteral("ox")).toDouble(), up.value(QStringLiteral("oy")).toDouble(),
									 up.value(QStringLiteral("oz")).toDouble()};
			f.upToFacePlane.normal = {up.value(QStringLiteral("nx")).toDouble(), up.value(QStringLiteral("ny")).toDouble(),
									 up.value(QStringLiteral("nz")).toDouble()};
			f.upToFacePlane.isPlanar = up.value(QStringLiteral("planar")).toBool(true);
			f.hasUpToFacePlane = true;
		}
		f.upToFaceBackendId = o.value(QStringLiteral("upToFaceBackendId")).toString();
		f.upToFaceIndex = o.value(QStringLiteral("upToFaceIndex")).toInt(-1);
		f.hasUpToVertex = false;
		if (o.contains(QStringLiteral("upToVertex")))
		{
			const QJsonObject vtx = o.value(QStringLiteral("upToVertex")).toObject();
			f.upToVertex.x = vtx.value(QStringLiteral("x")).toDouble();
			f.upToVertex.y = vtx.value(QStringLiteral("y")).toDouble();
			f.upToVertex.z = vtx.value(QStringLiteral("z")).toDouble();
			f.hasUpToVertex = true;
		}
		f.offsetFromFaceMm = o.value(QStringLiteral("offsetFromFaceMm")).toDouble(0.0);
		f.twistDeg = o.value(QStringLiteral("twistDeg")).toDouble(0.0);
		f.sketchRefId = o.value(QStringLiteral("sketchRefId")).toString();
		f.pathSketchRefId = o.value(QStringLiteral("pathSketchRefId")).toString();
		f.resultBackendId = o.value(QStringLiteral("resultBackendId")).toString();
		f.suppressed = o.value(QStringLiteral("suppressed")).toBool(false);
		f.visible = o.value(QStringLiteral("visible")).toBool(true);
		const QJsonObject pl = o.value(QStringLiteral("plane")).toObject();
		f.plane.origin = {pl.value(QStringLiteral("ox")).toDouble(), pl.value(QStringLiteral("oy")).toDouble(),
						  pl.value(QStringLiteral("oz")).toDouble()};
		f.plane.normal = {pl.value(QStringLiteral("nx")).toDouble(), pl.value(QStringLiteral("ny")).toDouble(),
						  pl.value(QStringLiteral("nz")).toDouble()};
		f.plane.axisX = {pl.value(QStringLiteral("xx")).toDouble(), pl.value(QStringLiteral("xy")).toDouble(),
						 pl.value(QStringLiteral("xz")).toDouble()};
		f.plane.axisY = {pl.value(QStringLiteral("yx")).toDouble(), pl.value(QStringLiteral("yy")).toDouble(),
						 pl.value(QStringLiteral("yz")).toDouble()};
		f.plane.isPlanar = pl.value(QStringLiteral("planar")).toBool(true);
		for (const QJsonValue& p : o.value(QStringLiteral("profile")).toArray())
			f.profileXyzMm.push_back(static_cast<float>(p.toDouble()));
		for (const QJsonValue& hv : o.value(QStringLiteral("profileHoles")).toArray())
		{
			std::vector<float> hole;
			for (const QJsonValue& p : hv.toArray())
				hole.push_back(static_cast<float>(p.toDouble()));
			if (hole.size() >= 12)
				f.profileHolesXyzMm.push_back(std::move(hole));
		}
		for (const QJsonValue& p : o.value(QStringLiteral("path")).toArray())
			f.pathXyzMm.push_back(static_cast<float>(p.toDouble()));
		for (const QJsonValue& sv : o.value(QStringLiteral("pathSegments")).toArray())
		{
			const QJsonObject so = sv.toObject();
			GeomodelingFeature::PathSegment s;
			s.kind = so.value(QStringLiteral("kind")).toInt(0);
			const QJsonArray a = so.value(QStringLiteral("a")).toArray();
			const QJsonArray b = so.value(QStringLiteral("b")).toArray();
			const QJsonArray m = so.value(QStringLiteral("m")).toArray();
			if (a.size() >= 3)
			{
				s.ax = static_cast<float>(a[0].toDouble());
				s.ay = static_cast<float>(a[1].toDouble());
				s.az = static_cast<float>(a[2].toDouble());
			}
			if (b.size() >= 3)
			{
				s.bx = static_cast<float>(b[0].toDouble());
				s.by = static_cast<float>(b[1].toDouble());
				s.bz = static_cast<float>(b[2].toDouble());
			}
			if (m.size() >= 3)
			{
				s.mx = static_cast<float>(m[0].toDouble());
				s.my = static_cast<float>(m[1].toDouble());
				s.mz = static_cast<float>(m[2].toDouble());
			}
			f.pathSegments.push_back(s);
		}
		for (const QJsonValue& sv : o.value(QStringLiteral("profileSegments")).toArray())
		{
			const QJsonObject so = sv.toObject();
			GeomodelingFeature::PathSegment s;
			s.kind = so.value(QStringLiteral("kind")).toInt(0);
			const QJsonArray a = so.value(QStringLiteral("a")).toArray();
			const QJsonArray b = so.value(QStringLiteral("b")).toArray();
			const QJsonArray m = so.value(QStringLiteral("m")).toArray();
			if (a.size() >= 3)
			{
				s.ax = static_cast<float>(a[0].toDouble());
				s.ay = static_cast<float>(a[1].toDouble());
				s.az = static_cast<float>(a[2].toDouble());
			}
			if (b.size() >= 3)
			{
				s.bx = static_cast<float>(b[0].toDouble());
				s.by = static_cast<float>(b[1].toDouble());
				s.bz = static_cast<float>(b[2].toDouble());
			}
			if (m.size() >= 3)
			{
				s.mx = static_cast<float>(m[0].toDouble());
				s.my = static_cast<float>(m[1].toDouble());
				s.mz = static_cast<float>(m[2].toDouble());
			}
			f.profileSegments.push_back(s);
		}
		const QString skDoc = o.value(QStringLiteral("sketchDocument")).toString();
		if (!skDoc.isEmpty())
			f.sketchDocumentUtf8 = skDoc.toUtf8();
		f.datumSourceKind = static_cast<GeomodelingDatumSourceKind>(o.value(QStringLiteral("datumSourceKind")).toInt(0));
		f.datumOriginPlaneIndex = o.value(QStringLiteral("datumOriginPlaneIndex")).toInt(0);
		f.datumFaceBackendId = o.value(QStringLiteral("datumFaceBackendId")).toString();
		f.datumFaceIndex = o.value(QStringLiteral("datumFaceIndex")).toInt(-1);
		f.datumOffsetMm = o.value(QStringLiteral("datumOffsetMm")).toDouble(0.0);
		f.datumAngleDeg = o.value(QStringLiteral("datumAngleDeg")).toDouble(0.0);
		if (o.contains(QStringLiteral("datumHinge")))
		{
			const QJsonObject hinge = o.value(QStringLiteral("datumHinge")).toObject();
			f.datumHingeOrigin = {static_cast<float>(hinge.value(QStringLiteral("ox")).toDouble()),
								 static_cast<float>(hinge.value(QStringLiteral("oy")).toDouble()),
								 static_cast<float>(hinge.value(QStringLiteral("oz")).toDouble())};
			f.datumHingeDir = {static_cast<float>(hinge.value(QStringLiteral("dx")).toDouble()),
							   static_cast<float>(hinge.value(QStringLiteral("dy")).toDouble()),
							   static_cast<float>(hinge.value(QStringLiteral("dz")).toDouble())};
		}
		f.datumPlaneId = o.value(QStringLiteral("datumPlaneId")).toString();
		m_features.push_back(std::move(f));
	}
}

namespace
{
QJsonArray vec3(double x, double y, double z)
{
	return QJsonArray{x, y, z};
}

PluginSketchPlane planeFromHost(const QJsonObject& pl)
{
	PluginSketchPlane plane;
	const QJsonArray o = pl.value(QStringLiteral("origin")).toArray();
	const QJsonArray ax = pl.value(QStringLiteral("axisX")).toArray();
	const QJsonArray ay = pl.value(QStringLiteral("axisY")).toArray();
	const QJsonArray n = pl.value(QStringLiteral("normal")).toArray();
	if (o.size() >= 3)
		plane.origin = {o[0].toDouble(), o[1].toDouble(), o[2].toDouble()};
	if (ax.size() >= 3)
		plane.axisX = {ax[0].toDouble(), ax[1].toDouble(), ax[2].toDouble()};
	if (ay.size() >= 3)
		plane.axisY = {ay[0].toDouble(), ay[1].toDouble(), ay[2].toDouble()};
	if (n.size() >= 3)
		plane.normal = {n[0].toDouble(), n[1].toDouble(), n[2].toDouble()};
	plane.isPlanar = pl.value(QStringLiteral("isPlanar")).toBool();
	return plane;
}

GeomodelingFeatureKind kindFromHost(const QString& s)
{
	if (s == QLatin1String("Pad"))
		return GeomodelingFeatureKind::Pad;
	if (s == QLatin1String("Pocket"))
		return GeomodelingFeatureKind::Pocket;
	if (s == QLatin1String("Sweep"))
		return GeomodelingFeatureKind::Sweep;
	if (s == QLatin1String("SweepCut"))
		return GeomodelingFeatureKind::SweepCut;
	if (s == QLatin1String("Fillet"))
		return GeomodelingFeatureKind::Fillet;
	if (s == QLatin1String("Chamfer"))
		return GeomodelingFeatureKind::Chamfer;
	if (s == QLatin1String("Revolve"))
		return GeomodelingFeatureKind::Revolve;
	if (s == QLatin1String("RevolveCut"))
		return GeomodelingFeatureKind::RevolveCut;
	if (s == QLatin1String("LinearPattern"))
		return GeomodelingFeatureKind::LinearPattern;
	if (s == QLatin1String("CircularPattern"))
		return GeomodelingFeatureKind::CircularPattern;
	if (s == QLatin1String("Mirror3D"))
		return GeomodelingFeatureKind::Mirror3D;
	if (s == QLatin1String("Loft"))
		return GeomodelingFeatureKind::Loft;
	if (s == QLatin1String("LoftCut"))
		return GeomodelingFeatureKind::LoftCut;
	if (s == QLatin1String("Shell"))
		return GeomodelingFeatureKind::Shell;
	if (s == QLatin1String("Draft"))
		return GeomodelingFeatureKind::Draft;
	if (s == QLatin1String("DatumPlane"))
		return GeomodelingFeatureKind::DatumPlane;
	if (s == QLatin1String("DatumPlaneAngle"))
		return GeomodelingFeatureKind::DatumPlaneAngle;
	return GeomodelingFeatureKind::Sketch;
}

QString kindToHost(GeomodelingFeatureKind k)
{
	if (k == GeomodelingFeatureKind::Pad)
		return QStringLiteral("Pad");
	if (k == GeomodelingFeatureKind::Pocket)
		return QStringLiteral("Pocket");
	if (k == GeomodelingFeatureKind::Sweep)
		return QStringLiteral("Sweep");
	if (k == GeomodelingFeatureKind::SweepCut)
		return QStringLiteral("SweepCut");
	if (k == GeomodelingFeatureKind::Fillet)
		return QStringLiteral("Fillet");
	if (k == GeomodelingFeatureKind::Chamfer)
		return QStringLiteral("Chamfer");
	if (k == GeomodelingFeatureKind::Revolve)
		return QStringLiteral("Revolve");
	if (k == GeomodelingFeatureKind::RevolveCut)
		return QStringLiteral("RevolveCut");
	if (k == GeomodelingFeatureKind::LinearPattern)
		return QStringLiteral("LinearPattern");
	if (k == GeomodelingFeatureKind::CircularPattern)
		return QStringLiteral("CircularPattern");
	if (k == GeomodelingFeatureKind::Mirror3D)
		return QStringLiteral("Mirror3D");
	if (k == GeomodelingFeatureKind::Loft)
		return QStringLiteral("Loft");
	if (k == GeomodelingFeatureKind::LoftCut)
		return QStringLiteral("LoftCut");
	if (k == GeomodelingFeatureKind::Shell)
		return QStringLiteral("Shell");
	if (k == GeomodelingFeatureKind::Draft)
		return QStringLiteral("Draft");
	if (k == GeomodelingFeatureKind::DatumPlane)
		return QStringLiteral("DatumPlane");
	if (k == GeomodelingFeatureKind::DatumPlaneAngle)
		return QStringLiteral("DatumPlaneAngle");
	return QStringLiteral("Sketch");
}

QString endToHost(GeomodelingExtrudeEnd e)
{
	if (e == GeomodelingExtrudeEnd::UpToFace)
		return QStringLiteral("UpToFace");
	if (e == GeomodelingExtrudeEnd::MidPlane)
		return QStringLiteral("MidPlane");
	if (e == GeomodelingExtrudeEnd::ThroughAll)
		return QStringLiteral("ThroughAll");
	if (e == GeomodelingExtrudeEnd::UpToVertex)
		return QStringLiteral("UpToVertex");
	if (e == GeomodelingExtrudeEnd::OffsetFromFace)
		return QStringLiteral("OffsetFromFace");
	if (e == GeomodelingExtrudeEnd::TwoDirections)
		return QStringLiteral("TwoDirections");
	return QStringLiteral("Blind");
}

GeomodelingExtrudeEnd endFromHost(const QString& s)
{
	if (s == QLatin1String("UpToFace"))
		return GeomodelingExtrudeEnd::UpToFace;
	if (s == QLatin1String("MidPlane"))
		return GeomodelingExtrudeEnd::MidPlane;
	if (s == QLatin1String("ThroughAll"))
		return GeomodelingExtrudeEnd::ThroughAll;
	if (s == QLatin1String("UpToVertex"))
		return GeomodelingExtrudeEnd::UpToVertex;
	if (s == QLatin1String("OffsetFromFace"))
		return GeomodelingExtrudeEnd::OffsetFromFace;
	if (s == QLatin1String("TwoDirections"))
		return GeomodelingExtrudeEnd::TwoDirections;
	return GeomodelingExtrudeEnd::Blind;
}
} // namespace

QByteArray FeatureDocument::toParametricHistoryJson() const
{
	QJsonArray arr;
	for (const auto& f : m_features)
	{
		if (f.kind == GeomodelingFeatureKind::DatumPlane || f.kind == GeomodelingFeatureKind::DatumPlaneAngle)
			continue;
		QJsonObject o;
		o.insert(QStringLiteral("id"), f.id);
		o.insert(QStringLiteral("name"), f.name);
		o.insert(QStringLiteral("kind"), kindToHost(f.kind));
		QJsonObject pl;
		pl.insert(QStringLiteral("origin"), vec3(f.plane.origin.x, f.plane.origin.y, f.plane.origin.z));
		pl.insert(QStringLiteral("axisX"), vec3(f.plane.axisX.x, f.plane.axisX.y, f.plane.axisX.z));
		pl.insert(QStringLiteral("axisY"), vec3(f.plane.axisY.x, f.plane.axisY.y, f.plane.axisY.z));
		pl.insert(QStringLiteral("normal"), vec3(f.plane.normal.x, f.plane.normal.y, f.plane.normal.z));
		pl.insert(QStringLiteral("isPlanar"), f.plane.isPlanar);
		o.insert(QStringLiteral("plane"), pl);
		QJsonArray xyz;
		for (float v : f.profileXyzMm)
			xyz.append(v);
		o.insert(QStringLiteral("profile"), xyz);
		if (!f.profileHolesXyzMm.empty())
		{
			QJsonArray holes;
			for (const auto& h : f.profileHolesXyzMm)
			{
				QJsonArray ha;
				for (float hv : h)
					ha.append(hv);
				holes.append(ha);
			}
			o.insert(QStringLiteral("profileHoles"), holes);
		}
		if (!f.pathXyzMm.empty())
		{
			QJsonArray path;
			for (float v : f.pathXyzMm)
				path.append(v);
			o.insert(QStringLiteral("path"), path);
		}
		if (!f.pathSegments.empty())
		{
			QJsonArray segs;
			for (const auto& s : f.pathSegments)
			{
				QJsonObject so;
				so.insert(QStringLiteral("kind"), s.kind);
				so.insert(QStringLiteral("a"), QJsonArray{s.ax, s.ay, s.az});
				so.insert(QStringLiteral("b"), QJsonArray{s.bx, s.by, s.bz});
				if (s.kind == 1 || s.kind == 3 || s.kind == 4)
					so.insert(QStringLiteral("m"), QJsonArray{s.mx, s.my, s.mz});
				segs.append(so);
			}
			o.insert(QStringLiteral("pathSegments"), segs);
		}
		if (!f.profileSegments.empty())
		{
			QJsonArray segs;
			for (const auto& s : f.profileSegments)
			{
				QJsonObject so;
				so.insert(QStringLiteral("kind"), s.kind);
				so.insert(QStringLiteral("a"), QJsonArray{s.ax, s.ay, s.az});
				so.insert(QStringLiteral("b"), QJsonArray{s.bx, s.by, s.bz});
				if (s.kind == 1 || s.kind == 3 || s.kind == 4)
					so.insert(QStringLiteral("m"), QJsonArray{s.mx, s.my, s.mz});
				segs.append(so);
			}
			o.insert(QStringLiteral("profileSegments"), segs);
		}
		if (std::abs(f.twistDeg) > 1e-9)
			o.insert(QStringLiteral("twistDeg"), f.twistDeg);
		o.insert(QStringLiteral("lengthMm"), f.lengthMm);
		if (std::abs(f.length2Mm) > 1e-9)
			o.insert(QStringLiteral("length2Mm"), f.length2Mm);
		if (std::abs(f.startOffsetMm) > 1e-9)
			o.insert(QStringLiteral("startOffsetMm"), f.startOffsetMm);
		o.insert(QStringLiteral("draftAngleDeg"), f.draftAngleDeg);
		o.insert(QStringLiteral("reversed"), f.reversed);
		o.insert(QStringLiteral("endCondition"), endToHost(f.endCondition));
		if (f.hasUpToFacePlane)
		{
			QJsonObject up;
			up.insert(QStringLiteral("origin"),
					  vec3(f.upToFacePlane.origin.x, f.upToFacePlane.origin.y, f.upToFacePlane.origin.z));
			up.insert(QStringLiteral("normal"),
					  vec3(f.upToFacePlane.normal.x, f.upToFacePlane.normal.y, f.upToFacePlane.normal.z));
			up.insert(QStringLiteral("isPlanar"), f.upToFacePlane.isPlanar);
			o.insert(QStringLiteral("upToFacePlane"), up);
		}
		if (!f.upToFaceBackendId.isEmpty())
			o.insert(QStringLiteral("upToFaceBackendId"), f.upToFaceBackendId);
		if (f.upToFaceIndex >= 0)
			o.insert(QStringLiteral("upToFaceIndex"), f.upToFaceIndex);
		if (f.hasUpToVertex)
			o.insert(QStringLiteral("upToVertex"),
					  vec3(f.upToVertex.x, f.upToVertex.y, f.upToVertex.z));
		if (std::abs(f.offsetFromFaceMm) > 1e-9)
			o.insert(QStringLiteral("offsetFromFaceMm"), f.offsetFromFaceMm);
		o.insert(QStringLiteral("sketchRefId"), f.sketchRefId);
		if (!f.pathSketchRefId.isEmpty())
			o.insert(QStringLiteral("pathSketchRefId"), f.pathSketchRefId);
		if (!f.loftSketchRefId.isEmpty())
			o.insert(QStringLiteral("loftSketchRefId"), f.loftSketchRefId);
		o.insert(QStringLiteral("suppressed"), f.suppressed);
		o.insert(QStringLiteral("visible"), f.visible);
		if (!f.sketchDocumentUtf8.isEmpty())
			o.insert(QStringLiteral("sketchDocument"), QString::fromUtf8(f.sketchDocumentUtf8));
		if (!f.edgeIndices.empty())
		{
			QJsonArray arr;
			for (int v : f.edgeIndices)
				arr.append(v);
			o.insert(QStringLiteral("edgeIndices"), arr);
		}
		if (!f.faceIndices.empty())
		{
			QJsonArray arr;
			for (int v : f.faceIndices)
				arr.append(v);
			o.insert(QStringLiteral("faceIndices"), arr);
		}
		o.insert(QStringLiteral("radiusMm"), f.radiusMm);
		o.insert(QStringLiteral("chamferDistMm"), f.chamferDistMm);
		o.insert(QStringLiteral("shellThicknessMm"), f.shellThicknessMm);
		o.insert(QStringLiteral("revolveAngleDeg"), f.revolveAngleDeg);
		o.insert(QStringLiteral("axisO"), vec3(f.axisOx, f.axisOy, f.axisOz));
		o.insert(QStringLiteral("axisD"), vec3(f.axisDx, f.axisDy, f.axisDz));
		o.insert(QStringLiteral("patternCount"), f.patternCount);
		o.insert(QStringLiteral("patternD"), vec3(f.patternDx, f.patternDy, f.patternDz));
		o.insert(QStringLiteral("patternAngleDeg"), f.patternAngleDeg);
		if (!f.patternSourceFeatureId.isEmpty())
			o.insert(QStringLiteral("patternSourceFeatureId"), f.patternSourceFeatureId);
		{
			QJsonObject mp;
			mp.insert(QStringLiteral("origin"), vec3(f.mirrorPlane.origin.x, f.mirrorPlane.origin.y, f.mirrorPlane.origin.z));
			mp.insert(QStringLiteral("axisX"),
					  vec3(f.mirrorPlane.axisX.x, f.mirrorPlane.axisX.y, f.mirrorPlane.axisX.z));
			mp.insert(QStringLiteral("axisY"),
					  vec3(f.mirrorPlane.axisY.x, f.mirrorPlane.axisY.y, f.mirrorPlane.axisY.z));
			mp.insert(QStringLiteral("normal"),
					  vec3(f.mirrorPlane.normal.x, f.mirrorPlane.normal.y, f.mirrorPlane.normal.z));
			mp.insert(QStringLiteral("isPlanar"), f.mirrorPlane.isPlanar);
			o.insert(QStringLiteral("mirrorPlane"), mp);
		}
		o.insert(QStringLiteral("mirrorKeepOriginal"), f.mirrorKeepOriginal);
		if (f.kind == GeomodelingFeatureKind::DatumPlaneAngle || std::abs(f.datumAngleDeg) > 1e-9)
		{
			o.insert(QStringLiteral("datumAngleDeg"), f.datumAngleDeg);
			QJsonObject hinge;
			hinge.insert(QStringLiteral("ox"), f.datumHingeOrigin.x);
			hinge.insert(QStringLiteral("oy"), f.datumHingeOrigin.y);
			hinge.insert(QStringLiteral("oz"), f.datumHingeOrigin.z);
			hinge.insert(QStringLiteral("dx"), f.datumHingeDir.x);
			hinge.insert(QStringLiteral("dy"), f.datumHingeDir.y);
			hinge.insert(QStringLiteral("dz"), f.datumHingeDir.z);
			o.insert(QStringLiteral("datumHinge"), hinge);
		}
		if (f.datumSourceKind != GeomodelingDatumSourceKind::None)
			o.insert(QStringLiteral("datumSourceKind"), static_cast<int>(f.datumSourceKind));
		if (f.datumSourceKind == GeomodelingDatumSourceKind::OriginPlane)
			o.insert(QStringLiteral("datumOriginPlaneIndex"), f.datumOriginPlaneIndex);
		if (f.datumSourceKind == GeomodelingDatumSourceKind::Face)
		{
			o.insert(QStringLiteral("datumFaceBackendId"), f.datumFaceBackendId);
			o.insert(QStringLiteral("datumFaceIndex"), f.datumFaceIndex);
		}
		if (std::abs(f.datumOffsetMm) > 1e-12 || f.datumSourceKind != GeomodelingDatumSourceKind::None)
			o.insert(QStringLiteral("datumOffsetMm"), f.datumOffsetMm);
		if (!f.datumPlaneId.isEmpty())
			o.insert(QStringLiteral("datumPlaneId"), f.datumPlaneId);
		arr.append(o);
	}
	QJsonObject root;
	root.insert(QStringLiteral("features"), arr);
	root.insert(QStringLiteral("seq"), m_seq);
	root.insert(QStringLiteral("rollbackAfterFeatureId"), m_rollbackAfterFeatureId);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool FeatureDocument::fromParametricHistoryJson(const QByteArray& utf8)
{
	QJsonParseError pe;
	const QJsonDocument doc = QJsonDocument::fromJson(utf8, &pe);
	if (pe.error != QJsonParseError::NoError || !doc.isObject())
		return false;
	const QJsonObject obj = doc.object();
	clear();
	m_seq = obj.value(QStringLiteral("seq")).toInt(1);
	m_rollbackAfterFeatureId = obj.value(QStringLiteral("rollbackAfterFeatureId")).toString();
	for (const QJsonValue& v : obj.value(QStringLiteral("features")).toArray())
	{
		const QJsonObject o = v.toObject();
		GeomodelingFeature f;
		f.id = o.value(QStringLiteral("id")).toString();
		f.name = o.value(QStringLiteral("name")).toString(f.id);
		f.kind = kindFromHost(o.value(QStringLiteral("kind")).toString());
		f.plane = planeFromHost(o.value(QStringLiteral("plane")).toObject());
		f.lengthMm = o.value(QStringLiteral("lengthMm")).toDouble(10.0);
		f.length2Mm = o.value(QStringLiteral("length2Mm")).toDouble(0.0);
		f.startOffsetMm = o.value(QStringLiteral("startOffsetMm")).toDouble(0.0);
		f.draftAngleDeg = o.value(QStringLiteral("draftAngleDeg")).toDouble(0.0);
		f.reversed = o.value(QStringLiteral("reversed")).toBool();
		f.endCondition = endFromHost(o.value(QStringLiteral("endCondition")).toString());
		if (o.contains(QStringLiteral("upToFacePlane")))
		{
			f.upToFacePlane = planeFromHost(o.value(QStringLiteral("upToFacePlane")).toObject());
			f.hasUpToFacePlane = true;
			if (!f.upToFacePlane.isPlanar)
				f.upToFacePlane.isPlanar = true;
		}
		f.upToFaceBackendId = o.value(QStringLiteral("upToFaceBackendId")).toString();
		f.upToFaceIndex = o.value(QStringLiteral("upToFaceIndex")).toInt(-1);
		f.hasUpToVertex = false;
		if (o.contains(QStringLiteral("upToVertex")))
		{
			const QJsonArray vtx = o.value(QStringLiteral("upToVertex")).toArray();
			if (vtx.size() >= 3)
			{
				f.upToVertex.x = vtx[0].toDouble();
				f.upToVertex.y = vtx[1].toDouble();
				f.upToVertex.z = vtx[2].toDouble();
				f.hasUpToVertex = true;
			}
		}
		f.offsetFromFaceMm = o.value(QStringLiteral("offsetFromFaceMm")).toDouble(0.0);
		f.twistDeg = o.value(QStringLiteral("twistDeg")).toDouble(0.0);
		f.sketchRefId = o.value(QStringLiteral("sketchRefId")).toString();
		f.pathSketchRefId = o.value(QStringLiteral("pathSketchRefId")).toString();
		f.loftSketchRefId = o.value(QStringLiteral("loftSketchRefId")).toString();
		f.suppressed = o.value(QStringLiteral("suppressed")).toBool(false);
		f.visible = o.value(QStringLiteral("visible")).toBool(true);
		for (const QJsonValue& p : o.value(QStringLiteral("profile")).toArray())
			f.profileXyzMm.push_back(static_cast<float>(p.toDouble()));
		for (const QJsonValue& hv : o.value(QStringLiteral("profileHoles")).toArray())
		{
			std::vector<float> hole;
			for (const QJsonValue& p : hv.toArray())
				hole.push_back(static_cast<float>(p.toDouble()));
			if (hole.size() >= 12)
				f.profileHolesXyzMm.push_back(std::move(hole));
		}
		for (const QJsonValue& p : o.value(QStringLiteral("path")).toArray())
			f.pathXyzMm.push_back(static_cast<float>(p.toDouble()));
		for (const QJsonValue& ev : o.value(QStringLiteral("edgeIndices")).toArray())
			f.edgeIndices.push_back(ev.toInt());
		for (const QJsonValue& fv : o.value(QStringLiteral("faceIndices")).toArray())
			f.faceIndices.push_back(fv.toInt());
		f.radiusMm = o.value(QStringLiteral("radiusMm")).toDouble(1.0);
		f.chamferDistMm = o.value(QStringLiteral("chamferDistMm")).toDouble(1.0);
		f.shellThicknessMm = o.value(QStringLiteral("shellThicknessMm")).toDouble(1.0);
		f.revolveAngleDeg = o.value(QStringLiteral("revolveAngleDeg")).toDouble(360.0);
		{
			const QJsonArray axisO = o.value(QStringLiteral("axisO")).toArray();
			const QJsonArray axisD = o.value(QStringLiteral("axisD")).toArray();
			if (axisO.size() >= 3)
			{
				f.axisOx = axisO[0].toDouble();
				f.axisOy = axisO[1].toDouble();
				f.axisOz = axisO[2].toDouble();
			}
			if (axisD.size() >= 3)
			{
				f.axisDx = axisD[0].toDouble();
				f.axisDy = axisD[1].toDouble();
				f.axisDz = axisD[2].toDouble();
			}
		}
		f.patternCount = o.value(QStringLiteral("patternCount")).toInt(2);
		{
			const QJsonArray patternD = o.value(QStringLiteral("patternD")).toArray();
			if (patternD.size() >= 3)
			{
				f.patternDx = patternD[0].toDouble();
				f.patternDy = patternD[1].toDouble();
				f.patternDz = patternD[2].toDouble();
			}
		}
		f.patternAngleDeg = o.value(QStringLiteral("patternAngleDeg")).toDouble(360.0);
		f.patternSourceFeatureId = o.value(QStringLiteral("patternSourceFeatureId")).toString();
		if (o.contains(QStringLiteral("mirrorPlane")))
			f.mirrorPlane = planeFromHost(o.value(QStringLiteral("mirrorPlane")).toObject());
		f.mirrorKeepOriginal = o.value(QStringLiteral("mirrorKeepOriginal")).toBool(true);
		f.datumAngleDeg = o.value(QStringLiteral("datumAngleDeg")).toDouble(0.0);
		if (o.contains(QStringLiteral("datumHinge")))
		{
			const QJsonObject hinge = o.value(QStringLiteral("datumHinge")).toObject();
			f.datumHingeOrigin = {static_cast<float>(hinge.value(QStringLiteral("ox")).toDouble()),
								 static_cast<float>(hinge.value(QStringLiteral("oy")).toDouble()),
								 static_cast<float>(hinge.value(QStringLiteral("oz")).toDouble())};
			f.datumHingeDir = {static_cast<float>(hinge.value(QStringLiteral("dx")).toDouble()),
							   static_cast<float>(hinge.value(QStringLiteral("dy")).toDouble()),
							   static_cast<float>(hinge.value(QStringLiteral("dz")).toDouble(1.0))};
		}
		f.datumSourceKind = static_cast<GeomodelingDatumSourceKind>(o.value(QStringLiteral("datumSourceKind")).toInt(0));
		f.datumOriginPlaneIndex = o.value(QStringLiteral("datumOriginPlaneIndex")).toInt(0);
		f.datumFaceBackendId = o.value(QStringLiteral("datumFaceBackendId")).toString();
		f.datumFaceIndex = o.value(QStringLiteral("datumFaceIndex")).toInt(-1);
		f.datumOffsetMm = o.value(QStringLiteral("datumOffsetMm")).toDouble(0.0);
		f.datumPlaneId = o.value(QStringLiteral("datumPlaneId")).toString();
		for (const QJsonValue& sv : o.value(QStringLiteral("pathSegments")).toArray())
		{
			const QJsonObject so = sv.toObject();
			GeomodelingFeature::PathSegment s;
			s.kind = so.value(QStringLiteral("kind")).toInt(0);
			const QJsonArray a = so.value(QStringLiteral("a")).toArray();
			const QJsonArray b = so.value(QStringLiteral("b")).toArray();
			const QJsonArray m = so.value(QStringLiteral("m")).toArray();
			if (a.size() >= 3)
			{
				s.ax = static_cast<float>(a[0].toDouble());
				s.ay = static_cast<float>(a[1].toDouble());
				s.az = static_cast<float>(a[2].toDouble());
			}
			if (b.size() >= 3)
			{
				s.bx = static_cast<float>(b[0].toDouble());
				s.by = static_cast<float>(b[1].toDouble());
				s.bz = static_cast<float>(b[2].toDouble());
			}
			if (m.size() >= 3)
			{
				s.mx = static_cast<float>(m[0].toDouble());
				s.my = static_cast<float>(m[1].toDouble());
				s.mz = static_cast<float>(m[2].toDouble());
			}
			f.pathSegments.push_back(s);
		}
		for (const QJsonValue& sv : o.value(QStringLiteral("profileSegments")).toArray())
		{
			const QJsonObject so = sv.toObject();
			GeomodelingFeature::PathSegment s;
			s.kind = so.value(QStringLiteral("kind")).toInt(0);
			const QJsonArray a = so.value(QStringLiteral("a")).toArray();
			const QJsonArray b = so.value(QStringLiteral("b")).toArray();
			const QJsonArray m = so.value(QStringLiteral("m")).toArray();
			if (a.size() >= 3)
			{
				s.ax = static_cast<float>(a[0].toDouble());
				s.ay = static_cast<float>(a[1].toDouble());
				s.az = static_cast<float>(a[2].toDouble());
			}
			if (b.size() >= 3)
			{
				s.bx = static_cast<float>(b[0].toDouble());
				s.by = static_cast<float>(b[1].toDouble());
				s.bz = static_cast<float>(b[2].toDouble());
			}
			if (m.size() >= 3)
			{
				s.mx = static_cast<float>(m[0].toDouble());
				s.my = static_cast<float>(m[1].toDouble());
				s.mz = static_cast<float>(m[2].toDouble());
			}
			f.profileSegments.push_back(s);
		}
		const QJsonValue skDoc = o.value(QStringLiteral("sketchDocument"));
		if (skDoc.isString())
			f.sketchDocumentUtf8 = skDoc.toString().toUtf8();
		else if (skDoc.isObject())
			f.sketchDocumentUtf8 = QJsonDocument(skDoc.toObject()).toJson(QJsonDocument::Compact);
		if (f.id.isEmpty())
			continue;
		m_features.push_back(std::move(f));
	}
	// Host 可能只带 suppressed 位：从中恢复回退标记
	if (m_rollbackAfterFeatureId.isEmpty())
	{
		for (int i = 0; i < static_cast<int>(m_features.size()); ++i)
		{
			if (!m_features[static_cast<std::size_t>(i)].suppressed)
				continue;
			if (i > 0)
				m_rollbackAfterFeatureId = m_features[static_cast<std::size_t>(i - 1)].id;
			break;
		}
	}
	return true;
}

void FeatureDocument::clear()
{
	m_features.clear();
	m_seq = 1;
	m_rollbackAfterFeatureId.clear();
}

std::vector<GeomodelingFeature> FeatureDocument::extractDatumPlanes()
{
	std::vector<GeomodelingFeature> out;
	std::vector<GeomodelingFeature> rest;
	out.reserve(m_features.size());
	rest.reserve(m_features.size());
	for (const GeomodelingFeature& f : m_features)
	{
		if (f.kind == GeomodelingFeatureKind::DatumPlane || f.kind == GeomodelingFeatureKind::DatumPlaneAngle)
			out.push_back(f);
		else
			rest.push_back(f);
	}
	m_features = std::move(rest);
	return out;
}

void FeatureDocument::appendPreserved(const GeomodelingFeature& f)
{
	m_features.push_back(f);
	const QString num = f.id.section(QLatin1Char('_'), -1);
	bool ok = false;
	const int n = num.toInt(&ok);
	if (ok && n >= m_seq)
		m_seq = n + 1;
}
