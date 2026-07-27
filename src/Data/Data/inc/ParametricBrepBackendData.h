#ifndef DATA_PARAMETRICBREPBACKENDDATA_H
#define DATA_PARAMETRICBREPBACKENDDATA_H

/// @file ParametricBrepBackendData.h
/// @brief 参数化工件：B-rep + 草图/Pad/Pocket 创建过程

#include "BrepBackendData.h"
#include "ParametricBrepFeature.h"

#include <string>
#include <unordered_map>
#include <vector>

class DATA_EXPORT ParametricBrepBackendData : public BrepBackendData
{
public:
	ParametricBrepBackendData();
	~ParametricBrepBackendData() override = default;

	std::string className() const override;

	const std::vector<ParametricFeature>& features() const { return m_features; }
	std::string addSketch(const ParametricSketchPlane& plane, const std::string& name = {});
	std::string addPad(const std::string& sketchId, double lengthMm, bool reversed = false);
	std::string addPocket(const std::string& sketchId, double lengthMm, bool reversed = false);
	std::string addSweep(const std::string& profileSketchId, const std::string& pathSketchId, bool cut = false);
	bool setProfile(const std::string& sketchId, const std::vector<float>& xyz);
	bool setLength(const std::string& featureId, double lengthMm);
	ParametricFeature* findFeature(const std::string& id);
	const ParametricFeature* findFeature(const std::string& id) const;
	void setFeatures(std::vector<ParametricFeature> features);
	void clearFeatures();

	/// 按特征链重放 → 更新 ShapeHandle；并刷新面归属表
	bool rebuild(std::string* errMsg = nullptr);

	/// 当前 tip 上 faceIndex → 产生该面的特征 id（进程内，跨会话不保证）
	std::string featureIdForFace(int faceIndex) const;
	const std::unordered_map<int, std::string>& faceOwnerByIndex() const { return m_faceOwnerByIndex; }

	nlohmann::json historyToJson() const;
	bool historyFromJson(const nlohmann::json& in, std::string* errMsg = nullptr);

	/// Pad→Pocket→改长 rebuild→JSON 往返
	static bool runParametricHistorySelfTest(std::string* errMsg = nullptr);

protected:
	void saveDerivedJson(nlohmann::json& out) const override;
	bool loadDerivedJson(const nlohmann::json& in, std::string* errMsg) override;

private:
	std::string nextId(const char* prefix);
	const ParametricFeature* findSketchFor(const ParametricFeature& feat) const;

	std::vector<ParametricFeature> m_features;
	int m_seq = 1;
	std::unordered_map<int, std::string> m_faceOwnerByIndex;
};

#endif
