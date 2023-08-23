#pragma once

#pragma once
#include "GlobalParams.h"

#include <opencv2/opencv.hpp>

namespace calibtask {
	void SetGlobalContainer(__GC* gcp);
	int RegisterCArmImage(const int sidScene, const std::string& carmScanParams, const std::string& scanName);
	bool CalibrationWithPhantom(glm::fmat4x4& matCArmRB2SourceCS, const cv::Mat& downloadedGrayImg, const track_info* trk, const bool useGlobal);
}

//bool CalibrateIntrinsics(const std::vector<std::string>& imgFileNames, float& fx, float& fy, float& cx, float& cy, float& s, std::vector<float>& distCoeffs);
//bool CalibrateExtrinsics(const std::vector<std::string>& imgFileNames, glm::fmat4x4& matLS2WS);