/// @file CameraResourceStore.cpp
/// @brief 抓图/内参/标定结果写入 resource/industrial_camera

#include "CameraResourceStore.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>

namespace industrial_camera_ui
{
namespace
{

QJsonObject mat4ToJson(const industrial_camera::Mat4& m)
{
	QJsonArray a;
	for (double v : m)
		a.append(v);
	return QJsonObject{{QStringLiteral("columnMajor4x4"), a}};
}

bool writeJson(const QString& path, const QJsonObject& obj, QString* err)
{
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		if (err)
			*err = QStringLiteral("无法写 %1").arg(path);
		return false;
	}
	f.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
	return true;
}

} // namespace

QString industrialCameraRoot()
{
	return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("resource/industrial_camera"));
}

bool ensureIndustrialCameraRoot(QString* err)
{
	QDir d(industrialCameraRoot());
	if (d.exists())
		return true;
	if (!QDir().mkpath(d.absolutePath()))
	{
		if (err)
			*err = QStringLiteral("无法创建 %1").arg(d.absolutePath());
		return false;
	}
	return true;
}

QString saveCaptureSession(const industrial_camera::CameraDeviceInfo& info,
						   const industrial_camera::CameraFrame2D* color,
						   const industrial_camera::CameraFrame3D* cloud,
						   const industrial_camera::CameraIntrinsics* intrinsics,
						   QString* err)
{
	if (!ensureIndustrialCameraRoot(err))
		return {};
	const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
	const QString serial = QString::fromStdString(info.serial.empty() ? "unknown" : info.serial);
	const QString dir = QDir(industrialCameraRoot()).filePath(QStringLiteral("captures/%1_%2").arg(stamp, serial));
	if (!QDir().mkpath(dir))
	{
		if (err)
			*err = QStringLiteral("无法创建捕获目录");
		return {};
	}

	QJsonObject meta;
	meta.insert(QStringLiteral("timestamp"), QDateTime::currentDateTime().toString(Qt::ISODate));
	meta.insert(QStringLiteral("pluginVersion"), QStringLiteral("1.0.0"));
	meta.insert(QStringLiteral("brand"), QString::fromUtf8(industrial_camera::brandToString(info.brand)));
	meta.insert(QStringLiteral("model"), QString::fromStdString(info.model));
	meta.insert(QStringLiteral("serial"), serial);
	meta.insert(QStringLiteral("ip"), QString::fromStdString(info.ip));

	if (color && !color->bytes.empty())
	{
		QImage img;
		if (color->pixelFormat == industrial_camera::PixelFormat::Bgr8)
		{
			img = QImage(color->bytes.data(), color->width, color->height, color->width * 3, QImage::Format_RGB888)
					  .rgbSwapped()
					  .copy();
		}
		else
		{
			img = QImage(color->bytes.data(), color->width, color->height, color->width, QImage::Format_Grayscale8).copy();
		}
		const QString imgPath = QDir(dir).filePath(QStringLiteral("color.png"));
		img.save(imgPath);
		meta.insert(QStringLiteral("color"), QStringLiteral("color.png"));
	}
	if (cloud && !cloud->points.empty())
	{
		const QString ply = QDir(dir).filePath(QStringLiteral("cloud.ply"));
		std::string e;
		if (!industrial_camera::writePlyAscii(ply.toStdString(), *cloud, &e))
		{
			if (err)
				*err = QString::fromStdString(e);
			return {};
		}
		meta.insert(QStringLiteral("cloud"), QStringLiteral("cloud.ply"));
	}
	if (intrinsics)
	{
		QJsonObject K;
		K.insert(QStringLiteral("fx"), intrinsics->fx);
		K.insert(QStringLiteral("fy"), intrinsics->fy);
		K.insert(QStringLiteral("cx"), intrinsics->cx);
		K.insert(QStringLiteral("cy"), intrinsics->cy);
		K.insert(QStringLiteral("width"), intrinsics->width);
		K.insert(QStringLiteral("height"), intrinsics->height);
		writeJson(QDir(dir).filePath(QStringLiteral("intrinsics.json")), K, nullptr);
		saveDeviceIntrinsics(info, *intrinsics, nullptr);
	}
	writeJson(QDir(dir).filePath(QStringLiteral("meta.json")), meta, err);
	return dir;
}

bool saveDeviceIntrinsics(const industrial_camera::CameraDeviceInfo& info,
						  const industrial_camera::CameraIntrinsics& K,
						  QString* err)
{
	if (!ensureIndustrialCameraRoot(err))
		return false;
	const QString brand = QString::fromUtf8(industrial_camera::brandToString(info.brand));
	const QString serial = QString::fromStdString(info.serial.empty() ? "unknown" : info.serial);
	const QString dir = QDir(industrialCameraRoot()).filePath(QStringLiteral("cameras/%1_%2").arg(brand, serial));
	QDir().mkpath(dir);
	QJsonObject obj;
	obj.insert(QStringLiteral("fx"), K.fx);
	obj.insert(QStringLiteral("fy"), K.fy);
	obj.insert(QStringLiteral("cx"), K.cx);
	obj.insert(QStringLiteral("cy"), K.cy);
	obj.insert(QStringLiteral("width"), K.width);
	obj.insert(QStringLiteral("height"), K.height);
	obj.insert(QStringLiteral("timestamp"), QDateTime::currentDateTime().toString(Qt::ISODate));
	QJsonObject dev;
	dev.insert(QStringLiteral("ip"), QString::fromStdString(info.ip));
	dev.insert(QStringLiteral("serial"), serial);
	dev.insert(QStringLiteral("model"), QString::fromStdString(info.model));
	writeJson(QDir(dir).filePath(QStringLiteral("device_info.json")), dev, nullptr);
	return writeJson(QDir(dir).filePath(QStringLiteral("intrinsics.json")), obj, err);
}

QString saveCalibrationSession(industrial_camera::HandEyeMountMode mode,
							   const industrial_camera::HandEyeResult& result,
							   const industrial_camera::CameraIntrinsics* intrinsicsUsed,
							   QString* err)
{
	if (!ensureIndustrialCameraRoot(err))
		return {};
	const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
	const QString modeStr = (mode == industrial_camera::HandEyeMountMode::EyeInHand) ? QStringLiteral("eye_in_hand")
																					 : QStringLiteral("eye_to_hand");
	const QString dir = QDir(industrialCameraRoot()).filePath(QStringLiteral("calibrations/%1_%2").arg(stamp, modeStr));
	if (!QDir().mkpath(dir) || !QDir().mkpath(QDir(dir).filePath(QStringLiteral("methods"))))
	{
		if (err)
			*err = QStringLiteral("无法创建标定目录");
		return {};
	}

	QJsonObject session;
	session.insert(QStringLiteral("mode"), modeStr);
	session.insert(QStringLiteral("timestamp"), QDateTime::currentDateTime().toString(Qt::ISODate));
	session.insert(QStringLiteral("pluginVersion"), QStringLiteral("1.0.0"));
	writeJson(QDir(dir).filePath(QStringLiteral("session.json")), session, nullptr);

	for (const auto& sc : result.scores)
	{
		QJsonObject m;
		m.insert(QStringLiteral("ok"), sc.ok);
		m.insert(QStringLiteral("name"), QString::fromStdString(sc.name));
		m.insert(QStringLiteral("rotErrRad"), sc.rotErrRad);
		m.insert(QStringLiteral("transErrMm"), sc.transErrMm);
		m.insert(QStringLiteral("score"), sc.score);
		m.insert(QStringLiteral("error"), QString::fromStdString(sc.error));
		if (sc.ok)
			m.insert(QStringLiteral("T"), mat4ToJson(sc.T));
		writeJson(QDir(dir).filePath(QStringLiteral("methods/%1.json").arg(QString::fromStdString(sc.name))), m, nullptr);
	}

	QJsonObject res;
	res.insert(QStringLiteral("ok"), result.ok);
	res.insert(QStringLiteral("bestMethod"), QString::fromStdString(result.bestMethodName));
	res.insert(QStringLiteral("inlierMotionPairs"), result.inlierMotionPairs);
	res.insert(QStringLiteral("error"), QString::fromStdString(result.error));
	if (result.ok)
		res.insert(QStringLiteral("T"), mat4ToJson(result.T_best));
	QJsonArray scores;
	for (const auto& sc : result.scores)
	{
		scores.append(QJsonObject{{QStringLiteral("name"), QString::fromStdString(sc.name)},
								  {QStringLiteral("ok"), sc.ok},
								  {QStringLiteral("score"), sc.score},
								  {QStringLiteral("rotErrRad"), sc.rotErrRad},
								  {QStringLiteral("transErrMm"), sc.transErrMm}});
	}
	res.insert(QStringLiteral("scores"), scores);
	writeJson(QDir(dir).filePath(QStringLiteral("result.json")), res, err);

	if (intrinsicsUsed)
	{
		QJsonObject K;
		K.insert(QStringLiteral("fx"), intrinsicsUsed->fx);
		K.insert(QStringLiteral("fy"), intrinsicsUsed->fy);
		K.insert(QStringLiteral("cx"), intrinsicsUsed->cx);
		K.insert(QStringLiteral("cy"), intrinsicsUsed->cy);
		writeJson(QDir(dir).filePath(QStringLiteral("intrinsics_used.json")), K, nullptr);
	}
	return dir;
}

} // namespace industrial_camera_ui
