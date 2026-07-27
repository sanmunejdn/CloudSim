/// @file SketchEditSession.h
/// @brief 草图编辑会话：平面、工具、尺寸、overlay、求解诊断

#ifndef GEOMETRICMODELINGPLUGIN_SKETCHEDITSESSION_H
#define GEOMETRICMODELINGPLUGIN_SKETCHEDITSESSION_H

#include "PluginGeometryTypes.h"
#include "SketchGeom.h"
#include "SketchTools.h"

#include <QString>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class IPluginDocument;
class IPluginGeometryHost;

class SketchEditSession
{
public:
	bool active() const { return m_active; }
	const PluginSketchPlane& plane() const { return m_plane; }
	SketchDocument2d& document() { return m_doc; }
	const SketchDocument2d& document() const { return m_doc; }
	int lastDof() const { return m_lastDof; }
	bool lastHasConflict() const { return m_lastConflict; }
	bool lastHasRedundant() const { return m_lastRedundant; }
	QString statusText() const;
	QString dofStatusText() const;

	bool begin(IPluginGeometryHost* geo, IPluginDocument* doc, const PluginSketchPlane& plane, QString* err = nullptr);
	bool beginWithDocument(IPluginGeometryHost* geo, IPluginDocument* doc, const PluginSketchPlane& plane,
						   const QByteArray& sketchJson, QString* err = nullptr);
	void end();
	void setTool(SketchToolKind kind);
	SketchToolKind toolKind() const { return m_toolKind; }
	void setUseChinese(bool useChinese);
	bool useChinese() const { return m_useChinese; }

	bool handleInput(const PluginSketchInputEvent& ev);
	bool solveNow(std::string* err = nullptr);
	bool exportClosedProfile(std::vector<float>& outXyzMm, std::string* err = nullptr) const;
	void refreshOverlay();
	QByteArray sketchDocumentJson() const { return m_doc.toJsonUtf8(); }

	/// 尺寸/求解后通知宿主刷新状态栏
	void setChangeNotifier(std::function<void()> fn) { m_onChanged = std::move(fn); }
	/// 追加其它可见草图（当前编辑草图由会话自身绘制）
	void setBackgroundOverlayProvider(std::function<void(std::vector<PluginSketchOverlaySegment>&)> fn)
	{
		m_backgroundOverlay = std::move(fn);
	}

	/// 尺寸工具：成功返回 true
	bool tryAddDimensionAt(const SkVec2& uv, QString* err = nullptr);

	/// 镜像参数面板
	int mirrorAxisId() const { return m_toolKind == SketchToolKind::Mirror ? m_dimPickA : -1; }
	const std::vector<int>& mirrorTargetIds() const { return m_mirrorTargets; }
	bool mirrorPickingAxis() const { return m_mirrorPickAxis; }
	void setMirrorPickingAxis(bool axis);
	void clearMirrorTargets();
	bool removeMirrorTarget(int entityId);
	bool confirmMirror(QString* err = nullptr);
	void resetMirrorSelection();
	QString entityDisplayName(int id) const;

private:
	void applySnap(SkVec2& uv);
	void syncConstraintsToSolver(class SketchConstraintSolver& solver,
								 std::unordered_map<int, int>& pointIdToIdx,
								 std::unordered_map<int, int>& lineIdToIdx,
								 std::unordered_map<int, int>& arcIdToIdx,
								 std::unordered_map<int, int>& constraintTagToDocIndex);
	void rebuildDiagEntitySets();
	bool promptAndAddConstraint(SkConstraintKind kind, int a, int b, double defaultValue, QString* err);
	bool addGeomConstraintNoPrompt(SkConstraintKind kind, int a, int b, QString* err);
	void updateDimHover(const SkVec2& uv);
	void updatePickHover(const SkVec2& uv);
	int hitDimTarget(const SkVec2& uv, double* outMeasure = nullptr) const;
	int hitAnyCurve(const SkVec2& uv) const;
	bool tryPickSessionAt(const SkVec2& uv, bool rightButton, QString* err);
	bool deleteEntityAt(const SkVec2& uv, QString* err);
	void resetPickState();
	bool beginPointDrag(const SkVec2& uv);
	void updatePointDrag(const SkVec2& uv);
	void endPointDrag();
	QString tr(const QString& en, const QString& zh) const;

	SkVec2 m_lastCursorUv{};
	bool m_hasCursorUv = false;
	bool m_useChinese = true;

	bool m_active = false;
	IPluginGeometryHost* m_geo = nullptr;
	IPluginDocument* m_docPtr = nullptr;
	PluginSketchPlane m_plane{};
	SketchDocument2d m_doc;
	SketchToolKind m_toolKind = SketchToolKind::Line;
	std::unique_ptr<ISketchTool> m_tool;
	SkSnapResult m_lastSnap{};
	double m_snapTolMm = 3.0;
	double m_gridMm = 5.0;
	bool m_gridOn = true;
	int m_lastDof = 0;
	bool m_lastConflict = false;
	bool m_lastRedundant = false;
	std::unordered_set<int> m_conflictEntities;
	std::unordered_set<int> m_redundantEntities;
	/// 拾取暂存（尺寸/几何约束/镜像共用）
	int m_dimPickA = -1;
	int m_dimHoverId = -1;
	QString m_dimHint;
	std::vector<int> m_mirrorTargets;
	bool m_mirrorPickAxis = true;
	int m_dragPointId = -1;
	std::function<void()> m_onChanged;
	std::function<void(std::vector<PluginSketchOverlaySegment>&)> m_backgroundOverlay;
};

#endif
