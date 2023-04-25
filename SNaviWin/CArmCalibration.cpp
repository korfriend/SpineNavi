#include <opencv2/opencv.hpp>

#include "CArmCalibration.h"

bool CalibrateIntrinsics(const std::vector<std::string>& imgFileNames, float& fx, float& fy, float& cx, float& cy, float& s, std::vector<float>& distCoeffs)
{
	return false;
}

bool CalibrateExtrinsics(const std::vector<std::string>& imgFileNames, glm::fmat4x4& matLS2WS)
{
	return false;
}