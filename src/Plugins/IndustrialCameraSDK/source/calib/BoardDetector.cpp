/// @file BoardDetector.cpp
/// @brief 板检测：CLOUDSIM_HAS_OPENCV 时用 findChessboardCorners / ArUco

#include "BoardDetector.h"

#include <cmath>
#include <vector>

#if defined(CLOUDSIM_HAS_OPENCV)
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace industrial_camera
{
namespace
{
Mat4 isometryToMat4(double r00, double r01, double r02, double tx, double r10, double r11, double r12, double ty,
					double r20, double r21, double r22, double tz)
{
	Mat4 m{};
	m[0] = r00;
	m[4] = r01;
	m[8] = r02;
	m[12] = tx;
	m[1] = r10;
	m[5] = r11;
	m[9] = r12;
	m[13] = ty;
	m[2] = r20;
	m[6] = r21;
	m[10] = r22;
	m[14] = tz;
	m[15] = 1.0;
	return m;
}
} // namespace

bool openCvAvailable()
{
#if defined(CLOUDSIM_HAS_OPENCV)
	return true;
#else
	return false;
#endif
}

BoardDetectResult detectBoardPose(const CameraFrame2D& image, const CameraIntrinsics& K, const BoardDetectParams& params)
{
	BoardDetectResult r;
#if !defined(CLOUDSIM_HAS_OPENCV)
	(void)image;
	(void)K;
	(void)params;
	r.error = "未编译 OpenCV（CLOUDSIM_HAS_OPENCV）。请部署 bin/SDK/opencv 或手填板位姿。";
	return r;
#else
	if (image.bytes.empty() || image.width <= 0 || image.height <= 0)
	{
		r.error = "图像为空";
		return r;
	}
	cv::Mat gray;
	if (image.pixelFormat == PixelFormat::Bgr8)
	{
		cv::Mat bgr(image.height, image.width, CV_8UC3, const_cast<std::uint8_t*>(image.bytes.data()));
		cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
	}
	else
	{
		gray = cv::Mat(image.height, image.width, CV_8UC1, const_cast<std::uint8_t*>(image.bytes.data())).clone();
	}

	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0, 1);
	if (K.fx <= 0 || K.fy <= 0)
	{
		cameraMatrix = (cv::Mat_<double>(3, 3) << image.width, 0, image.width * 0.5, 0, image.width, image.height * 0.5, 0,
						0, 1);
	}
	cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F);
	for (int i = 0; i < 5; ++i)
		dist.at<double>(i) = K.dist[static_cast<size_t>(i)];

	cv::Mat rvec, tvec;
	bool found = false;
	if (params.type == BoardType::Chessboard)
	{
		std::vector<cv::Point2f> corners;
		const cv::Size pattern(params.cornersX, params.cornersY);
		found = cv::findChessboardCorners(gray, pattern, corners,
										  cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{
			cv::cornerSubPix(gray, corners, {11, 11}, {-1, -1},
							 {cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01});
			std::vector<cv::Point3f> obj;
			obj.reserve(static_cast<size_t>(params.cornersX * params.cornersY));
			for (int y = 0; y < params.cornersY; ++y)
				for (int x = 0; x < params.cornersX; ++x)
					obj.emplace_back(static_cast<float>(x * params.squareSizeMm),
									 static_cast<float>(y * params.squareSizeMm), 0.f);
			found = cv::solvePnP(obj, corners, cameraMatrix, dist, rvec, tvec);
		}
		else
			r.error = "未找到棋盘格角点";
	}
	else
	{
		r.error = "当前构建未启用 ArUco 字典检测路径，请用棋盘格或手填";
		return r;
	}

	if (!found)
	{
		if (r.error.empty())
			r.error = "solvePnP 失败";
		return r;
	}
	cv::Mat R;
	cv::Rodrigues(rvec, R);
	r.T_cam_board = isometryToMat4(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec.at<double>(0),
								   R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec.at<double>(1),
								   R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec.at<double>(2));
	r.ok = true;
	return r;
#endif
}

} // namespace industrial_camera
