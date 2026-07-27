#ifndef GEOMETRICMODELINGPLUGIN_FEATUREDOCUMENT_H
#define GEOMETRICMODELINGPLUGIN_FEATUREDOCUMENT_H

/// @file FeatureDocument.h
/// @brief Body 特征树（对齐 OneCAD OperationRecord / FreeCAD PartDesign 语义）

#include "PluginGeometryTypes.h"

#include <QByteArray>
#include <QJsonObject>
#include <QString>
#include <vector>

enum class GeomodelingFeatureKind
{
	Sketch = 0,
	Pad,
	Pocket,
	Sweep,
	SweepCut
};

enum class GeomodelingExtrudeEnd
{
	Blind = 0,
	UpToFace,
	MidPlane,
	ThroughAll
};

struct GeomodelingFeature
{
	QString id;
	QString name;
	GeomodelingFeatureKind kind = GeomodelingFeatureKind::Sketch;
	PluginSketchPlane plane{};
	std::vector<float> profileXyzMm;
	std::vector<float> pathXyzMm;
	struct PathSegment
	{
		int kind = 0;
		float ax = 0, ay = 0, az = 0;
		float bx = 0, by = 0, bz = 0;
		float mx = 0, my = 0, mz = 0;
	};
	std::vector<PathSegment> pathSegments;
	double lengthMm = 10.0;
	double draftAngleDeg = 0.0;
	bool reversed = false;
	GeomodelingExtrudeEnd endCondition = GeomodelingExtrudeEnd::Blind;
	/// UpToFace：烤平面回退；活引用见 upToFaceBackendId / upToFaceIndex
	PluginSketchPlane upToFacePlane{};
	bool hasUpToFacePlane = false;
	QString upToFaceBackendId;
	int upToFaceIndex = -1;
	QString sketchRefId;
	QString pathSketchRefId;
	QString resultBackendId;
	QByteArray sketchDocumentUtf8;
	bool suppressed = false;
	/// 草图视口 overlay；默认显示
	bool visible = true;
};

class FeatureDocument
{
public:
	QString addSketch(const PluginSketchPlane& plane, const QString& name = QString());
	QString addPad(const QString& sketchId, double lengthMm, bool reversed = false);
	QString addPocket(const QString& sketchId, double lengthMm, bool reversed = false);
	bool setProfile(const QString& sketchId, const std::vector<float>& xyz);
	bool setSketchDocument(const QString& sketchId, const QByteArray& utf8);
	bool setVisible(const QString& featureId, bool visible);
	bool setLength(const QString& featureId, double lengthMm);
	bool setDraftAngleDeg(const QString& featureId, double draftAngleDeg);
	bool setReversed(const QString& featureId, bool reversed);
	bool setExtrudeEnd(const QString& featureId, GeomodelingExtrudeEnd end, const PluginSketchPlane* upToFace = nullptr,
					   const QString& upToFaceBackendId = QString(), int upToFaceIndex = -1);
	/// 删除特征；草图会级联删除引用它的 Pad/Pocket/Sweep
	bool removeFeature(const QString& featureId);
	GeomodelingFeature* find(const QString& id);
	const GeomodelingFeature* find(const QString& id) const;
	const std::vector<GeomodelingFeature>& features() const { return m_features; }
	QString rollbackAfterFeatureId() const { return m_rollbackAfterFeatureId; }
	bool applyRollbackTo(const QString& featureId);
	void clearRollback();
	QJsonObject toJson() const;
	void fromJson(const QJsonObject& obj);
	QByteArray toParametricHistoryJson() const;
	bool fromParametricHistoryJson(const QByteArray& utf8);
	void clear();

private:
	QString nextId(const char* prefix);
	std::vector<GeomodelingFeature> m_features;
	int m_seq = 1;
	QString m_rollbackAfterFeatureId;
};

#endif
