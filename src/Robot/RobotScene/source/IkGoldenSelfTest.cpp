/// @file IkGoldenSelfTest.cpp
/// @brief 读取 testdata/ik_golden/*.json，校验肘符号与 branch 约定

#include "IkGoldenSelfTest.h"

#include "IkSolveTypes.h"
#include "RobotInstructionAxisConfiguration.h"
#include "RunLogger.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

namespace RobotInstruction
{
namespace
{
void fail(std::vector<std::string>* failures, const std::string& msg)
{
	if (failures)
	{
		failures->push_back(msg);
	}
	RunLogger::warn("[IkGoldenSelfTest] " + msg);
}

QString defaultSyntheticPath()
{
	// source/ -> ../testdata/ik_golden/synthetic.json
	const QFileInfo here(QString::fromUtf8(__FILE__));
	return here.absoluteDir().absoluteFilePath(QStringLiteral("../testdata/ik_golden/synthetic.json"));
}

bool checkOneCase(const QJsonObject& o, std::vector<std::string>* failures)
{
	const std::string name = o.value(QStringLiteral("name")).toString(QStringLiteral("?")).toStdString();
	const QJsonArray qArr = o.value(QStringLiteral("qRad")).toArray();
	const QJsonArray nArr = o.value(QStringLiteral("jointNames")).toArray();
	if (qArr.size() < 6 || nArr.size() != qArr.size())
	{
		fail(failures, name + ": qRad/jointNames size");
		return false;
	}
	std::vector<double> q;
	std::vector<std::string> names;
	q.reserve(static_cast<size_t>(qArr.size()));
	names.reserve(static_cast<size_t>(nArr.size()));
	for (int i = 0; i < qArr.size(); ++i)
	{
		q.push_back(qArr.at(i).toDouble());
		names.push_back(nArr.at(i).toString().toStdString());
	}
	const JointConfigurationClass cls = classifyJointConfiguration(q, names, &q, 2, 4);
	const QString expectElbow = o.value(QStringLiteral("expectElbow")).toString().toUpper();
	if (expectElbow == QStringLiteral("UP") && cls.elbow != ElbowPosture::Up)
	{
		fail(failures, name + ": expectElbow UP");
		return false;
	}
	if (expectElbow == QStringLiteral("DOWN") && cls.elbow != ElbowPosture::Down)
	{
		fail(failures, name + ": expectElbow DOWN");
		return false;
	}
	const int expectSign = o.value(QStringLiteral("expectBranchElbowSign")).toInt(-1);
	const int expectWrist = o.value(QStringLiteral("expectWristBranch")).toInt(-1);
	const int elbowSign = q[2] >= 0.0 ? 0 : 1;
	if (expectSign >= 0 && elbowSign != expectSign)
	{
		fail(failures, name + ": elbowSign mismatch");
		return false;
	}
	if (expectWrist >= 0)
	{
		const int bid = encodeIkBranchId(elbowSign, expectWrist);
		if (ikBranchElbowSign(bid) != elbowSign || ikBranchWrist(bid) != expectWrist)
		{
			fail(failures, name + ": branch encode/decode");
			return false;
		}
	}
	return true;
}
} // namespace

bool runIkGoldenSelfTest(std::vector<std::string>* failures, const std::string& jsonPathUtf8)
{
	const QString path =
		jsonPathUtf8.empty() ? defaultSyntheticPath() : QString::fromUtf8(jsonPathUtf8.c_str());
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
	{
		// 默认路径取自源码树，脱离源码的部署环境跳过而非报错
		if (jsonPathUtf8.empty())
		{
			RunLogger::info("[IkGoldenSelfTest] SKIP (no golden file: " + path.toStdString() + ")");
			return true;
		}
		fail(failures, std::string("open golden failed: ") + path.toStdString());
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
	if (!doc.isObject())
	{
		fail(failures, "golden JSON root not object");
		return false;
	}
	const QJsonArray cases = doc.object().value(QStringLiteral("cases")).toArray();
	if (cases.isEmpty())
	{
		fail(failures, "golden cases empty");
		return false;
	}
	bool ok = true;
	for (const QJsonValue& v : cases)
	{
		if (!v.isObject() || !checkOneCase(v.toObject(), failures))
		{
			ok = false;
		}
	}
	if (ok)
	{
		RunLogger::info("[IkGoldenSelfTest] PASS (" + std::to_string(cases.size()) + " cases, " +
						path.toStdString() + ")");
	}
	return ok;
}

} // namespace RobotInstruction
