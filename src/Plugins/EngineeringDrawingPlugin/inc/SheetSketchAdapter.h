#ifndef ENGINEERINGDRAWINGPLUGIN_SHEETSKETCHADAPTER_H
#define ENGINEERINGDRAWINGPLUGIN_SHEETSKETCHADAPTER_H

/// @file SheetSketchAdapter.h
/// @brief 图幅场景坐标驱动草图内核（UV=mm，无 OCC/OSG）

#include "SketchGeom.h"
#include "SketchTools.h"

#include <QByteArray>
#include <QHash>
#include <QPointF>
#include <QString>
#include <QVector>
#include <functional>
#include <memory>
#include <vector>

struct SheetSketchPolyline
{
	QVector<QPointF> points;
	bool construction = false;
	int entityId = -1;
};

class SheetSketchAdapter
{
public:
	SketchDocument2d& document() { return m_doc; }
	const SketchDocument2d& document() const { return m_doc; }

	void clear();
	void setTool(SketchToolKind kind);
	void clearTool();
	ISketchTool* tool() const { return m_tool.get(); }
	bool hasTool() const { return static_cast<bool>(m_tool); }

	static SkVec2 toUv(const QPointF& scene);
	static QPointF toScene(const SkVec2& uv);

	/// tolMm：场景毫米容差；extra：视图边线端点等
	SkVec2 snapScene(const QPointF& scene, double tolMm, const QVector<QPointF>& extra,
					 const SkVec2* refForOrtho) const;

	bool press(const QPointF& scene, bool rightButton, double snapTolMm, const QVector<QPointF>& extraSnap,
			   const QString& layerId = QStringLiteral("L0"));
	void move(const QPointF& scene, double snapTolMm, const QVector<QPointF>& extraSnap);
	void cancelTool();

	QVector<SheetSketchPolyline> tessellate() const;
	QVector<QPointF> previewPolyline() const;
	SkSnapResult lastSnap() const { return m_lastSnap; }

	int hitTestEntity(const QPointF& scene, double tolMm) const;
	int hitTestEntity(const QPointF& scene, double tolMm, const std::function<bool(int)>& accept) const;
	bool removeEntity(int id);

	QString layerOf(int entityId) const;
	void setLayerOf(int entityId, const QString& layerId);
	void setEntityLayers(const QHash<int, QString>& map);
	QHash<int, QString> entityLayers() const { return m_entityLayer; }
	void remapLayer(const QString& fromId, const QString& toId);

	QByteArray toJsonUtf8() const { return m_doc.toJsonUtf8(); }
	bool fromJsonUtf8(const QByteArray& utf8);

private:
	SkVec2 applySnap(const QPointF& scene, double tolMm, const QVector<QPointF>& extra) const;
	int maxEntityId() const;
	void collectEntityIds(QVector<int>& out) const;

	SketchDocument2d m_doc;
	std::unique_ptr<ISketchTool> m_tool;
	mutable SkSnapResult m_lastSnap;
	QHash<int, QString> m_entityLayer;
};

#endif
