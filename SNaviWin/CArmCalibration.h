#pragma once

#pragma once
#include "GlobalParams.h"

#include <opencv2/opencv.hpp>

namespace calibtask {
	void InitializeTask(__GC* gcp);
	int RegisterCArmImage(const int sidScene, const int viewIdx,
		const cv::Mat& K, const cv::Mat& DistCoeffs, const glm::fmat4x4& matCA2RB, const glm::fmat4x4& matRB2WS, const cv::Mat& imgCArm);
	//bool CalibrationWithPhantom(glm::fmat4x4& matCArmRB2SourceCS, const cv::Mat& downloadedGrayImg, const track_info* trk, const bool useGlobal);
}

namespace mystudents {
	int Get2DPostionsFromFMarkersPhantom(const cv::Mat& inputImg, const int rows, const int cols, std::vector<cv::Point2f>& points2Ds
		, const float minArea = 2000, const float maxArea = 16000, const float minInterval = 100, std::vector<float>* circleRadius =NULL);
	int Get3DPostionsFromFMarkersPhantom(const glm::fvec3& marker0, const glm::fvec3& marker1, const glm::fvec3& marker2, const glm::fvec3& marker3,
		const track_info* trk, const int rows, const int cols,
		std::vector<cv::Point3f>& points3Ds);
}

//bool CalibrateIntrinsics(const std::vector<std::string>& imgFileNames, float& fx, float& fy, float& cx, float& cy, float& s, std::vector<float>& distCoeffs);
//bool CalibrateExtrinsics(const std::vector<std::string>& imgFileNames, glm::fmat4x4& matLS2WS);