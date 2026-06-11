#pragma once

#include <memory>
#include <string>
#include <vector>

#include "BackendDataBase.h"
#include <ShapeHandle.h>

#include <unordered_map>

struct DATA_EXPORT BrepHierarchyPart
{
	std::string partPath;
	std::string parentPartPath;
	std::string displayName;
	geoalgo::ShapeHandle shapeRef;
};

/// STEP B-rep 工件：场景显示与轨迹特征共用同一 ShapeHandle
class DATA_EXPORT BrepBackendData : public BackendDataBase
{
public:
	BrepBackendData();
	~BrepBackendData() override = default;

	std::string className() const override;
	bool hasGeometry() const override;
	BackendBoundingBox geometryBounds() const override;
	std::size_t geometryElementCount() const override;
	void clearGeometry() override;

	void setShape(geoalgo::ShapeHandle shape);
	const geoalgo::ShapeHandle& shapeRef() const { return m_shape; }
	void shareShapeFrom(const BrepBackendData& other);

	void setBrepSidecarRelativePath(std::string relativePath) { m_brepSidecarRel = std::move(relativePath); }
	const std::string& brepSidecarRelativePath() const { return m_brepSidecarRel; }

	void setPose(const BackendVec3& position) override;
	BackendVec3 pose() const override;
	void setRotation(const BackendVec3& eulerDeg) override;
	BackendVec3 rotation() const override;
	void setColor(const BackendColor& color) override;
	BackendColor color() const override;

	void setFaceHighlightColors(std::unordered_map<int, BackendColor> colorsByFaceIndex);
	const std::unordered_map<int, BackendColor>& faceHighlightColors() const;
	void clearFaceHighlightColors();

	bool hasPoseProperty() const override { return true; }
	bool hasRotationProperty() const override { return true; }
	bool hasColorProperty() const override { return true; }

	bool loadFromStepFile(const std::string& path, std::string* errMsg = nullptr);
	bool loadFromBrepFile(const std::string& path, std::string* errMsg = nullptr);
	bool writeBrepFile(const std::string& path, std::string* errMsg = nullptr) const;
	bool writeStepFile(const std::string& path, std::string* errMsg = nullptr) const;

	static bool loadStepHierarchyFromFile(
		const std::string& path,
		std::vector<BrepHierarchyPart>& outParts,
		std::string* errMsg = nullptr);

	nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const override;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
		const BackendDataManager* mgr = nullptr) override;

private:
	void recomputeBounds();
	void saveDerivedJson(nlohmann::json& out) const override;
	bool loadDerivedJson(const nlohmann::json& in, std::string* errMsg) override;

	geoalgo::ShapeHandle m_shape;
	std::string m_brepSidecarRel;
	BackendBoundingBox m_bounds;
	BackendVec3 m_position;
	BackendVec3 m_rotation;
	BackendColor m_color;
	std::unordered_map<int, BackendColor> m_faceHighlightColors;
};
