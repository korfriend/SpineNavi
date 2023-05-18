#include "ApiUtility.hpp"
#include "naviHelpers.hpp"

#include "CArmCalibration.h"

#include <opencv2/opencv.hpp>


bool CalibrateIntrinsics(const std::vector<std::string>& imgFileNames, float& fx, float& fy, float& cx, float& cy, float& s, std::vector<float>& distCoeffs)
{
	return false;
}

bool CalibrateExtrinsics(const std::vector<std::string>& imgFileNames, glm::fmat4x4& matLS2WS)
{
	return false;
}



int RegisterCArmImage(const int sidScene, const std::string& carmScanParams, const std::string& scanName)
{
	cv::FileStorage fs(carmScanParams, cv::FileStorage::Mode::READ);
	if (!fs.isOpened())
		return -1;

	cv::Mat K;
	fs["K"] >> K;
	cv::Mat DistCoeffs;
	fs["DistCoeffs"] >> DistCoeffs;
	cv::Mat rvec, tvec;
	fs["rvec"] >> rvec;
	fs["tvec"] >> tvec;
	cv::Mat rb2wsMat;
	fs["rb2wsMat"] >> rb2wsMat; // memory storage ordering in opengl
	std::string imgFileName;
	fs["imgFile"] >> imgFileName;
	fs.release();

	int aidGroupCArmCam = 0;
	vzm::ActorParameters apGroupCams;
	vzm::NewActor(apGroupCams, "Tracking CAM Group:scanName", aidGroupCArmCam);
	vzm::AppendSceneItemToSceneTree(aidGroupCArmCam, sidScene);


	///////////////////////////////////
	// preprocessing information
	// *** IMPORTANT NOTE:
	// the C-arm image (Genoray) is aligned w.r.t. detector's view
	// the calibration image must be aligned w.r.t. source's view (view frustum's origin point)
	// therefore, the position mirrored horizontally
	cv::Mat imgCArm = cv::imread(imgFileName);

	double arrayMatK[9] = {};
	double arrayDistCoeffs[5] = {};
	memcpy(arrayMatK, K.ptr(), sizeof(double) * 9);
	memcpy(arrayDistCoeffs, DistCoeffs.ptr(), sizeof(double) * 5);

	cv::Mat matR;
	cv::Rodrigues(rvec, matR);
	// note, here camera frame (notation 'CA', opencv convention) is defined with
	// z axis as viewing direction
	// -y axis as up vector
	glm::fmat4x4 matRB2CA = glm::fmat4x4(1);
	matRB2CA[0][0] = (float)matR.at<double>(0, 0);
	matRB2CA[0][1] = (float)matR.at<double>(1, 0);
	matRB2CA[0][2] = (float)matR.at<double>(2, 0);
	matRB2CA[1][0] = (float)matR.at<double>(0, 1);
	matRB2CA[1][1] = (float)matR.at<double>(1, 1);
	matRB2CA[1][2] = (float)matR.at<double>(2, 1);
	matRB2CA[2][0] = (float)matR.at<double>(0, 2);
	matRB2CA[2][1] = (float)matR.at<double>(1, 2);
	matRB2CA[2][2] = (float)matR.at<double>(2, 2);
	matRB2CA[3][0] = (float)((double*)tvec.data)[0];
	matRB2CA[3][1] = (float)((double*)tvec.data)[1];
	matRB2CA[3][2] = (float)((double*)tvec.data)[2];

	glm::fmat4x4 matCA2RB = glm::inverse(matRB2CA);
	glm::fmat4x4 matRB2WS;
	memcpy(glm::value_ptr(matRB2WS), rb2wsMat.ptr(), sizeof(float) * 16);

	glm::fmat4x4 matCA2WS = matRB2WS * matCA2RB;
	int oidCamTris = 0, oidCamLines = 0, oidCamLabel = 0;
	navihelpers::CamOcv_Gen(matCA2WS, "C-Arm Source:" + scanName, oidCamTris, oidCamLines, oidCamLabel);
	vzm::ActorParameters apCamTris, apCamLines, apCamLabel;
	apCamTris.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamTris);
	apCamTris.color[3] = 0.5f;
	apCamLines.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLines);
	*(glm::fvec4*)apCamLines.phong_coeffs = glm::fvec4(0, 1, 0, 0);
	apCamLines.line_thickness = 2;
	apCamLabel.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLabel);
	*(glm::fvec4*)apCamLabel.phong_coeffs = glm::fvec4(0, 1, 0, 0);
	int aidCamTris = 0, aidCamLines = 0, aidCamLabel = 0;
	vzm::NewActor(apCamTris, "C-Arm Cam Tris:" + scanName, aidCamTris);
	vzm::NewActor(apCamLines, "C-Arm Lines:" + scanName, aidCamLines);
	vzm::NewActor(apCamLabel, "C-Arm Label:" + scanName, aidCamLabel);
	vzm::AppendSceneItemToSceneTree(aidCamTris, aidGroupCArmCam);
	vzm::AppendSceneItemToSceneTree(aidCamLines, aidGroupCArmCam);
	vzm::AppendSceneItemToSceneTree(aidCamLabel, aidGroupCArmCam);

	glm::fmat4x4 matCS2PS;
	navihelpers::ComputeMatCS2PS(arrayMatK[0], arrayMatK[4], arrayMatK[1], arrayMatK[2], arrayMatK[5],
		(float)imgCArm.cols, (float)imgCArm.rows, 0.1f, 1.2f, matCS2PS);
	// here, CS is defined with
	// -z axis as viewing direction
	// y axis as up vector
	glm::fmat4x4 matPS2CS = glm::inverse(matCS2PS);
	// so, we need to convert CS to OpenCV's Camera Frame by rotating 180 deg w.r.t. x axis
	glm::fmat4x4 matCS2CA = glm::rotate(glm::pi<float>(), glm::fvec3(1, 0, 0));
	glm::fmat4x4 matPS2WS = matCA2WS * matCS2CA * matPS2CS;


	// mapping the carm image to far plane
	glm::fvec3 ptsCArmPlanes[4] = { glm::fvec3(-1, 1, 1), glm::fvec3(1, 1, 1), glm::fvec3(1, -1, 1), glm::fvec3(-1, -1, 1) };
	for (int i = 0; i < 4; i++) {
		ptsCArmPlanes[i] = vzmutils::transformPos(ptsCArmPlanes[i], matPS2WS);
	}
	//glm::fvec2 ptsTexCoords[4] = { glm::fvec2(0, 0), glm::fvec2(1, 0), glm::fvec2(1, 1), glm::fvec2(0, 1) };
	glm::fvec3 ptsTexCoords[4] = { glm::fvec3(0, 0, 0), glm::fvec3(1, 0, 0), glm::fvec3(1, 1, 0), glm::fvec3(0, 1, 0) };
	//std::vector<glm::fvec3> posBuffer(6);
	//std::vector<glm::fvec2> texBuffer(6);
	unsigned int idxList[6] = { 0, 1, 3, 1, 2, 3 };

	int oidImage = 0;
	vzm::LoadImageFile(imgFileName, oidImage, true, true);
	int oidCArmPlane = 0;
	vzm::GeneratePrimitiveObject(__FP ptsCArmPlanes[0], NULL, NULL, __FP ptsTexCoords[0], 4, idxList, 2, 3, oidCArmPlane);
	vzm::ActorParameters apCArmPlane;
	apCArmPlane.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCArmPlane);
	apCArmPlane.SetResourceID(vzm::ActorParameters::TEXTURE_2D, oidImage);
	apCArmPlane.script_params.SetParam("matCA2WS", matCA2WS); // temporal storage :)
	apCArmPlane.script_params.SetParam("imageWH", glm::ivec2(imgCArm.cols, imgCArm.rows)); // temporal storage :)
	*(glm::fvec4*)apCArmPlane.phong_coeffs = glm::fvec4(1, 0, 0, 0);
	int aidCArmPlane = 0;
	vzm::NewActor(apCArmPlane, "CArm Plane:" + scanName, aidCArmPlane);
	vzm::AppendSceneItemToSceneTree(aidCArmPlane, aidGroupCArmCam);

	return aidGroupCArmCam;
}
