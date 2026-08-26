#ifndef DATA_MESHBACKENDDATA_LOADERS_H
#define DATA_MESHBACKENDDATA_LOADERS_H

/// @file MeshBackendData_loaders.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 网格文件加载（MeshBackendData 内部）

#include <string>
#include <vector>

class MeshBackendData;
struct MeshHierarchyPart;

/// 网格文件加载（MeshBackendData 内部）
namespace mesh_backend_load
{
/// STEP 反向面绕序翻转，与 OSG 法线一致
constexpr bool kMeshStepFlipReversedFaceWinding = true;

void meshLoadErr(std::string* errMsg, const char* text);
std::string meshLowerExtension(const std::string& path);

/// 退化三角（叉积过小）返回 false，不入 soup
bool meshPushTri(std::vector<float>& soup, double ax, double ay, double az, double bx, double by, double bz, double cx,
				 double cy, double cz);

bool meshTryLoadObjWithVertexNormals(const std::string& path, std::vector<float>& soup, std::vector<float>& normalSoup);

bool meshLoadStepSingleFile(const std::string& path, std::vector<float>& soup, std::string* errMsg);
bool meshLoadDxfSingleFile(const std::string& path, std::vector<float>& soup, std::string* errMsg);
bool meshLoadCgalMeshFile(MeshBackendData& mesh, const std::string& path, const std::string& ext, std::string* errMsg,
						  int meshImportQuality = 1);

/// 兼容旧签名；真源层不再做有损抽稀（quality 0/1 均不删三角）
void meshApplyImportQualityToSoup(std::vector<float>& soup, int meshImportQuality);

} // namespace mesh_backend_load

#endif // DATA_MESHBACKENDDATA_LOADERS_H
