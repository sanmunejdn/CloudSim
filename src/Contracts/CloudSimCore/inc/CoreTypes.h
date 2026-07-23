#ifndef CLOUDSIMCORE_CORETYPES_H
#define CLOUDSIMCORE_CORETYPES_H

/// @file CoreTypes.h
/// @brief 三维向量

#include "cloudsim_core_global.h"

#include <QHash>
#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <QVector>
#include <array>
#include <functional>

namespace cloudsim::core
{
using ObjectId = QString;
using Mat4 = std::array<double, 16>;

/// 三维向量
struct Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

/// 位姿 DTO
struct PoseDto
{
	Vec3 positionMm;
	Vec3 eulerDeg;
};

/// 颜色 DTO
struct ColorDto
{
	float r = 1.f;
	float g = 1.f;
	float b = 1.f;
	float a = 1.f;
};

/// 包围盒 DTO
struct BBoxDto
{
	Vec3 min;
	Vec3 max;
	bool valid = false;
};

/// 属性行 DTO
struct PropertyRowDto
{
	QString key;
	QString labelEn;
	bool editable = true;
	QString value;
};

/// 注册对象元数据
struct RegisterObjectDto
{
	QString className;
	QString name;
	ObjectId parentId;
};

/// 导入选项
struct ImportOptionsDto
{
	bool quietUi = false;
	bool resetViewToHome = true;
	/// 后端目录类型
	QString catalogTypeName = QStringLiteral("Model");
	bool isPointCloud = false;
	ObjectId parentId;
	/// 工程稳定 id
	ObjectId persistedId;
	/// 网格导入精度：0=Coarse，1=Medium，2=Fine（STL/OBJ/PLY/OFF）
	int meshImportQuality = 1;
};

/// 规划结果
struct PlanResultDto
{
	bool ok = false;
	QString error;
	QVector<double> jointTargetsRad;
	bool hasExternalAxisQ = false;
	double externalAxisQ = 0.0;
};

/// 运动指令
struct MotionInstructionDto
{
	QString instructionType;
	PoseDto targetPose;
	QString jointRadCsv;
	QJsonObject axisConfiguration;
	QJsonObject extensions;
};

/// 规划上下文
struct PlanContextDto
{
	QVector<double> seedJointRad;
	QString urdfPath;
	QString tcpLinkName;
	Mat4 toolFrame = identityMat4();
	QJsonObject extensions;

	static Mat4 identityMat4()
	{
		Mat4 m{};
		m[0] = m[5] = m[10] = m[15] = 1.0;
		return m;
	}
};

/// URDF 注册结果
struct RobotRegistrationDto
{
	bool ok = false;
	QString error;
	ObjectId sceneRootBackendId;
	QVector<QString> warnings;
	int linkCount = 0;
	int jointCount = 0;
	QString sourceDisplayName;
};

/// 几何类型（Widget 侧分支，避免 dynamic_cast Data 类型）
enum class GeometryKind
{
	None,
	Points,
	Mesh
};

/// 后端对象快照（Widget 不持有 BackendDataBase）
struct BackendObjectDto
{
	ObjectId id;
	QString name;
	QString className;
	QVector<ObjectId> parentIds;
	QVector<ObjectId> childIds;
	bool hasGeometry = false;
	GeometryKind geometryKind = GeometryKind::None;
	BBoxDto bbox;
	bool visible = true;
};

/// 物体变换 gizmo：世界轴 / 局部轴
enum class TransformGizmoFrameDto
{
	World,
	Local,
};

using RobotBaseWorldResolver = std::function<bool(Mat4& outRobotBaseWorldColumnMajor)>;

/// Follow 求解上下文（UI 策略经 DTO 传入 Host）
struct FollowSolveContextDto
{
	bool skipAll = false;
	ObjectId gizmoSelectedBackendId;
	ObjectId manualPoseAuthorityBackendId;
};

/// 场景标注快照（树/UI 用，无 OSG 类型）
struct AnnotationSnapshotDto
{
	QString id;
	QString displayText;
	bool visible = true;
};

/// 指令路点叠加轴（仿真预览）
struct InstructionPoseAxisDto
{
	Vec3 positionMm;
	Vec3 eulerDeg;
	bool lineMotion = false;
	bool reachable = true;
	QString robotBackendId;
	QString backendId;
	bool mountTcpOnPatRoot = false;
	bool hasLocalMatrix = false;
	Mat4 localMatrix{};
	QString urdfTcpAttachLinkName;
};

struct RawTrajectoryOverlayVertexDto
{
	Vec3 positionMm;
	bool reachable = true;
};

struct RawTrajectoryOverlayFrameDto
{
	Vec3 positionMm;
	Vec3 eulerDeg;
	bool reachable = true;
};

struct FeatureCatalogOverlayItemDto
{
	int displayIndex = 0;
	Vec3 anchorWorldMm;
	Vec3 labelWorldMm;
	bool hasEdgeSegment = false;
	Vec3 edgeAWorldMm;
	Vec3 edgeBWorldMm;
};

struct RobotFrameOverlayUpdateDto
{
	QString robotRootBackendId;
	bool showToolFrames = false;
	struct ToolEntryDto
	{
		QString name;
		QString mountBackendId;
		Mat4 localMatrix{};
		bool active = false;
	};
	QVector<ToolEntryDto> toolFrames;
	bool showUserFrames = false;
	struct UserEntryDto
	{
		QString name;
		QString mountBackendId;
		Mat4 localMatrix{};
	};
	QVector<UserEntryDto> userFrames;
};

/// 运动指令可行轴配置枚举（Widget 不依赖 RobotScene 类型）
struct FeasibleMotionAxisOptionsDto
{
	QStringList presetTokens;
	QStringList elbowTokens;
	QStringList wristTokens;
	QStringList armTokens;
	QStringList turnJ1Tokens;
	QStringList turnJ4Tokens;
	QStringList turnJ6Tokens;
};

/// 机器人 per-link FK 切片 DTO（osg::Matrixd → Mat4，供 Widget 不依赖 RobotScene）
struct RobotPerLinkKinematicsSliceDto
{
	QString urdfAbsolutePath;
	ObjectId sceneRootBackendId;
	QHash<QString, QString> linkNameToBackendId;
	QHash<QString, Mat4> fkMeshWorldT0;
	QHash<QString, Mat4> outerWorldAtBindByBackendId;
	Mat4 robotBasePlacementWorld = PlanContextDto::identityMat4();
	bool meshVerticesInLinkFrame = false;
};

} // namespace cloudsim::core

#endif // CLOUDSIMCORE_CORETYPES_H
