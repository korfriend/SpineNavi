#pragma once

// math using GLM
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"

bool CalibrateIntrinsics(const std::vector<std::string>& imgFileNames, float& fx, float& fy, float& cx, float& cy, float& s, std::vector<float>& distCoeffs);
bool CalibrateExtrinsics(const std::vector<std::string>& imgFileNames, glm::fmat4x4& matLS2WS);