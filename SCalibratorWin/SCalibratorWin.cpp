// SCalibratorWin.cpp : Defines the entry point for the application.
//

#include "framework.h"
#include "SCalibratorWin.h"

#include <regex>
#include <vector>
#include <windowsx.h>
#include <iostream>
#include <winsock2.h> // udp
#include <stdio.h>

// math using GLM
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"
#include <glm/gtx/quaternion.hpp>

// SpineProjects
#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#include "../optitrk/optitrk.h"
#include "../SNaviWin/naviHelpers.hpp"
#include "rapidcsv/rapidcsv.h"

#include "../SNaviWin/CArmCalibration.h"
//#include "tinyxml2.h"

#define DESIRED_SCREEN_W 700
#define DESIRED_SCREEN_H 700
#define USE_WHND true

#define MAX_LOADSTRING 100

// Global Variables:
HWND g_hWnd, g_hWndDialog1;
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
LRESULT CALLBACK    DiagProc1(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

std::string folder_data = "";
std::string folder_capture = "";
std::string folder_trackingInfo = "";
std::string folder_optiSession = "";

void UpdateBMP(int cidCam, HWND hWnd) {
	unsigned char* ptr_rgba;
	float* ptr_zdepth;
	int w, h;
	if (vzm::GetRenderBufferPtrs(cidCam, &ptr_rgba, &ptr_zdepth, &w, &h))
	{
		// https://stackoverflow.com/questions/26005744/how-to-display-pixels-on-screen-directly-from-a-raw-array-of-rgb-values-faster-t
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hWnd, &ps);
		// Temp HDC to copy picture
		HDC src = CreateCompatibleDC(hdc); // hdc - Device context for window, I've got earlier with GetDC(hwnd__) or GetDC(NULL);
		// Creating temp bitmap
		HBITMAP map = CreateBitmap(w, h, 1, 8 * 4, (void*)ptr_rgba); // pointer to array
		SelectObject(src, map); // Inserting picture into our temp HDC
		// Copy image from temp HDC to window
		BitBlt(hdc, // Destination
			0,  // x and
			0,  // y - upper-left corner of place, where we'd like to copy
			w, // width of the region
			h, // height
			src, // source
			0,   // x and
			0,   // y of upper left corner  of part of the source, from where we'd like to copy
			SRCCOPY); // Defined DWORD to juct copy pixels. Watch more on msdn;

		//GraphicsPostDrawing(hdc, cidCam);
		DeleteDC(src); // Deleting temp HDC
		DeleteObject(map);

		EndPaint(hWnd, &ps);
	}
};

void SceneInit() {
	// we set 'meter scale world to world space

	int oidAxis = 0;
	vzm::GenerateAxisHelperObject(oidAxis, 0.5f);

	vzm::ActorParameters apAxis;
	apAxis.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidAxis);
	apAxis.line_thickness = 3;
	*(glm::fvec4*)apAxis.color = glm::fvec4(1, 1, 1, 1);
	int aidAxis = 0;
	vzm::NewActor(apAxis, "World Axis", aidAxis);

	int oidGrid, oidLines, oidTextX, oidTextZ;
	navihelpers::World_GridAxis_Gen(oidGrid, oidLines, oidTextX, oidTextZ);

	std::vector<glm::fvec3> pinfo(3);
	pinfo[0] = glm::fvec3(0, 0.5, 0);
	pinfo[1] = glm::fvec3(0, 0, -1);
	pinfo[2] = glm::fvec3(0, 1, 0);
	int oidFrameText;
	vzm::GenerateTextObject((float*)&pinfo[0], "Frame", 0.1, true, false, oidFrameText);

	vzm::ActorParameters apGrid, apLines, apTextX, apTextZ, apTextFrame;
	apGrid.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidGrid);
	apLines.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidLines);
	apTextX.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidTextX);
	apTextZ.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidTextZ);
	apTextFrame.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidFrameText);
	*(glm::fvec4*)apTextX.phong_coeffs = glm::fvec4(0, 1, 0, 0);
	*(glm::fvec4*)apTextZ.phong_coeffs = glm::fvec4(0, 1, 0, 0);
	*(glm::fvec4*)apTextFrame.phong_coeffs = glm::fvec4(0, 1, 0, 0);

	int aidGrid, aidLines, aidTextX, aidTextZ, aidTextFrame;
	vzm::NewActor(apGrid, "World Grid", aidGrid);
	vzm::NewActor(apLines, "World Lines", aidLines);
	vzm::NewActor(apTextX, "World Text X", aidTextX);
	vzm::NewActor(apTextZ, "World Text Y", aidTextZ);
	vzm::NewActor(apTextFrame, "Frame Text", aidTextFrame);

	RECT rcWorldView;
	GetClientRect(g_hWnd, &rcWorldView);

	vzm::CameraParameters cpCam1;
	*(glm::fvec3*)cpCam1.pos = glm::fvec3(-1.5, 1.5, -1.5);
	*(glm::fvec3*)cpCam1.up = glm::fvec3(0, 1, 0);
	*(glm::fvec3*)cpCam1.view = glm::fvec3(1, -1, 1);

	cv::FileStorage fs(folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
	if (fs.isOpened()) {
		cv::Mat ocvVec3;
		fs["POS"] >> ocvVec3;
		memcpy(cpCam1.pos, ocvVec3.ptr(), sizeof(float) * 3);
		fs["VIEW"] >> ocvVec3;
		memcpy(cpCam1.view, ocvVec3.ptr(), sizeof(float) * 3);
		fs["UP"] >> ocvVec3;
		memcpy(cpCam1.up, ocvVec3.ptr(), sizeof(float) * 3);
		fs.release();
	}

	cpCam1.w = rcWorldView.right - rcWorldView.left;
	cpCam1.h = rcWorldView.bottom - rcWorldView.top;

	cpCam1.np = 0.1f;
	cpCam1.fp = 100.f;

	float vFov = 3.141592654f / 4.f;
	if (0) {
		cpCam1.fov_y = vFov;
		cpCam1.aspect_ratio = (float)cpCam1.w / (float)cpCam1.h;
		cpCam1.projection_mode = vzm::CameraParameters::CAMERA_FOV;
	}
	else {
		float aspect_ratio = (float)cpCam1.w / (float)cpCam1.h;
		float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;

		//  fy = h/(2 * tan(f_y / 2)) 
		cpCam1.fx = cpCam1.w / (2.f * tan(hFov / 2.f));
		cpCam1.fy = cpCam1.h / (2.f * tan(vFov / 2.f));
		cpCam1.sc = 0;
		cpCam1.cx = cpCam1.w / 2.f;
		cpCam1.cy = cpCam1.h / 2.f;
		cpCam1.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
	}

	//cpCam1.SetOrthogonalProjection(true);
	cpCam1.hWnd = USE_WHND ? g_hWnd : NULL;
	int cidCam1 = 0;
	vzm::NewCamera(cpCam1, "World Camera", cidCam1);

	vzm::LightParameters lpLight1;
	lpLight1.is_on_camera = true;
	lpLight1.is_pointlight = false;
	*(glm::fvec3*)lpLight1.pos = *(glm::fvec3*)cpCam1.pos;
	*(glm::fvec3*)lpLight1.dir = *(glm::fvec3*)cpCam1.view;
	*(glm::fvec3*)lpLight1.up = *(glm::fvec3*)cpCam1.up;
	int lidLight1 = 0;
	vzm::NewLight(lpLight1, "World Light", lidLight1);

	// make a scene
	int sidScene = 0;
	vzm::NewScene("Scene1", sidScene);

	vzm::AppendSceneItemToSceneTree(cidCam1, sidScene);
	vzm::AppendSceneItemToSceneTree(lidLight1, sidScene);
	vzm::AppendSceneItemToSceneTree(aidAxis, sidScene);
	vzm::AppendSceneItemToSceneTree(aidGrid, sidScene);
	vzm::AppendSceneItemToSceneTree(aidLines, sidScene);
	vzm::AppendSceneItemToSceneTree(aidTextX, sidScene);
	vzm::AppendSceneItemToSceneTree(aidTextZ, sidScene);
	vzm::AppendSceneItemToSceneTree(aidTextFrame, sidScene);
}


int numAnimationCount = 50;
int arAnimationKeyFrame = -1;
std::vector<vzm::CameraParameters> cpInterCams(numAnimationCount);
std::string g_sceneName = "Scene1"; // Scene1, Scene2
std::string g_camName = "World Camera"; // World Camera, CArm Camera
std::string g_sceneName2 = "Scene2"; // Scene1, Scene2
std::string g_camName2 = "CArm Camera"; // World Camera, CArm Camera

void Render() {
	int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName); 
	int cidCam1 = vzmutils::GetSceneItemIdByName(g_camName); 

	if (sidScene != 0 && cidCam1 != 0) {
		// show case
		if (arAnimationKeyFrame >= 0) {
			vzm::CameraParameters cpCam = cpInterCams[arAnimationKeyFrame++];
			vzm::SetCameraParams(cidCam1, cpCam);
			if (arAnimationKeyFrame == numAnimationCount)
				arAnimationKeyFrame = -1;

			//UINT flags = SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE;
			//SetWindowPos(g_hWnd, NULL, 100, 100, cpCam.w, cpCam.h, flags);
		}

		vzm::RenderScene(sidScene, cidCam1);

		if (USE_WHND)
			vzm::PresentHWND(g_hWnd);
		else {
			UpdateBMP(cidCam1, g_hWnd);
			InvalidateRect(g_hWnd, NULL, FALSE);
		}
	}

	int sidScene2 = vzmutils::GetSceneItemIdByName(g_sceneName2);
	int cidCam2 = vzmutils::GetSceneItemIdByName(g_camName2);
	if (sidScene2 != 0 && cidCam2 != 0) {

		vzm::RenderScene(sidScene2, cidCam2);
		
		if (USE_WHND)
			vzm::PresentHWND(g_hWndDialog1);
		else {
			UpdateBMP(cidCam2, g_hWndDialog1);
			InvalidateRect(g_hWndDialog1, NULL, FALSE);
		}
	}
}

int main()
{
	return wWinMain(GetModuleHandle(NULL), NULL, GetCommandLine(), SW_SHOWNORMAL);
}


struct avr_trk
{
	std::string xRayScanImg;
	std::string xRayCirclesResult;
	glm::fquat q_cArmRb; // c-arm rb
	glm::fvec3 rot_cArmRb; // c-arm rb
	float rotDeg_cArmRb; // c-arm rb
	glm::fmat4x4 mat_rb2ws; // c-arm rb
	glm::fmat4x4 mat_ws2rb; // c-arm rb

	glm::fvec3 t_cArmRb; // c-arm rb
	float rbMSE_cArmRb; // c-arm rb

	std::vector<glm::fvec3> rbMarkers_calibRb; // calib rb
	std::vector<glm::fvec3> wsMarkers_calibRb; // calib rb
	float rbMSE_calibRb; // c-arm rb
	//navihelpers::track_info avrTrkInfo;

	avr_trk() {}
	avr_trk(const std::string& _xRayScanImg, const std::string& _xRayCirclesResult) {
		xRayScanImg = _xRayScanImg;
		xRayCirclesResult = _xRayCirclesResult;
	}
};

void CALLBACK TimerProc(HWND, UINT, UINT_PTR pcsvData, DWORD)
{
	static int frameCount = 0;
	//std::vector<navihelpers::track_info>& trackingFrames = *(std::vector<navihelpers::track_info>*)pcsvData;
	int totalFrames = 1;// (int)trackingFrames.size();
	int frame = (frameCount++) % totalFrames;
	// update frame text 
	{
		int aidFrameText = vzmutils::GetSceneItemIdByName("Frame Text");
		vzm::ActorParameters apFrameText;
		vzm::GetActorParams(aidFrameText, apFrameText);
		int oidFrameText = apFrameText.GetResourceID(vzm::ActorParameters::GEOMETRY);
		std::vector<glm::fvec3> pinfo(3);
		pinfo[0] = glm::fvec3(0, 0.3, 0);
		pinfo[1] = glm::fvec3(0, 0, -1);
		pinfo[2] = glm::fvec3(0, 1, 0);
		vzm::GenerateTextObject((float*)&pinfo[0], "Frame : " + std::to_string(frame), 0.1, true, false, oidFrameText);
		//vzm::GetActorParams(aidFrameText, apFrameText);
	}

	//UpdateTrackInfo2Scene(trackingFrames[frame]);

	Render();
}

std::vector<cv::Point2f> g_pts2ds;
std::vector<cv::Point3f> g_ptsWsMk3ds;
std::vector<cv::Point3f> g_ptsRbMk3ds;
glm::fvec3 g_posCenterMKsRBS = glm::fvec3(0);

void StoreParams(const std::string& targetFolder, const std::string& paramsFileName, const glm::fmat4x4& matRB2WS, const std::string& imgFileName, const bool isRb) {

	cv::FileStorage __fs(targetFolder + paramsFileName, cv::FileStorage::Mode::WRITE);

	cv::FileStorage fs(targetFolder + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
	cv::Mat _matK;
	fs["K"] >> _matK;
	__fs << "K" << _matK;
	cv::Mat _distCoeffs;
	fs["DistCoeffs"] >> _distCoeffs;
	__fs << "DistCoeffs" << _distCoeffs;
	fs.open(targetFolder + (isRb ? "rb2carm0.txt" : "rb2carm1.txt"), cv::FileStorage::Mode::READ);
	cv::Mat rvec, tvec;
	fs["rvec"] >> rvec;
	fs["tvec"] >> tvec;
	__fs << "rvec" << rvec;
	__fs << "tvec" << tvec;
	fs.release();


	cv::Mat ocvRb(4, 4, CV_32FC1);
	memcpy(ocvRb.ptr(), glm::value_ptr(matRB2WS), sizeof(float) * 16);
	__fs << "rb2wsMat" << ocvRb;
	__fs << "imgFile" << imgFileName;
	__fs.release();
};

float ComputeReprojErr(const std::vector<cv::Point2f>& pts2d, const std::vector<cv::Point2f>& pts2d_reproj, float& maxErr) {
	float avrDiff = 0;
	maxErr = 0;
	int numPts = (int)pts2d.size();
	for (int i = 0; i < (int)pts2d.size(); i++) {
		float diff = glm::distance(*(glm::fvec2*)&pts2d[i], *(glm::fvec2*)&pts2d_reproj[i]);
		maxErr = std::max(maxErr, diff);
		avrDiff += diff / (float)numPts;
	}
	return avrDiff;
};

void ComputePose(const std::vector<cv::Point2f>& pts2ds,
	const std::vector<cv::Point3f>& ptsRbMk3ds, const std::vector<cv::Point3f>& ptsWsMk3ds,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	cv::Mat& rvec, cv::Mat& tvec,
	const std::vector<cv::Point2f>& pts2ds_test,
	const std::vector<cv::Point3f>& ptsRbMk3ds_test, const std::vector<cv::Point3f>& ptsWsMk3ds_test) {
	using namespace std;
	for (int i = 0; i < 2; i++) {
		cv::Mat rvec, tvec;
		const vector<cv::Point3f>& pts3d = i == 0 ? ptsRbMk3ds : ptsWsMk3ds;
		//cv::solvePnP(pts3d, pts2ds, cameraMatrix, distCoeffs, rvec, tvec);
		cv::solvePnPRansac(pts3d, pts2ds, cameraMatrix, distCoeffs, rvec, tvec, false, 100, 10.f);
		//cv::solvePnPRefineLM(pts3d, pts2ds, cameraMatrix, distCoeffs, rvec, tvec);
		//cv::solvePnPRefineVVS(pts3d, pts2ds, cameraMatrix, distCoeffs, rvec, tvec);

		{
			// Reproject the 3D points onto the image plane using the camera calibration
			const vector<cv::Point3f>& pts3d_test = i == 0 ? ptsRbMk3ds_test : ptsWsMk3ds_test;
			std::vector<cv::Point2f> imagePointsReprojected;
			cv::projectPoints(pts3d_test, rvec, tvec, cameraMatrix, distCoeffs, imagePointsReprojected);

			// Compute the reprojection error
			float maxErr = 0;
			float reprojectionError = ComputeReprojErr(pts2ds_test, imagePointsReprojected, maxErr);
			//double reprojectionError = cv::norm(pts2ds, imagePointsReprojected, cv::NORM_L2) / pts2ds.size();
			cout << "************* " << (i == 0 ? "Rb" : "Ws") << " reprojectionError : " << reprojectionError << ", max error : " << maxErr << endl;
		}

		{
			cv::FileStorage fs(folder_trackingInfo + "rb2carm" + to_string(i) + ".txt", cv::FileStorage::Mode::WRITE);
			fs.write("rvec", rvec);
			fs.write("tvec", tvec);
			fs.release();
		}
	}
};

std::map<std::string, avr_trk> cArmCalibScans; // string ... tracking csv file
void CalibrateCamPoseForCArmRB(const std::string& intrinsicsFile, const std::string& cArmRbName, const std::string& calibRbName, const int numCalibRbMKs)
{
	using namespace std;

	cv::Mat cameraMatrix, distCoeffs;
	{
		cv::FileStorage fs(intrinsicsFile, cv::FileStorage::Mode::READ);
		fs["K"] >> cameraMatrix;
		fs["DistCoeffs"] >> distCoeffs;
		fs.release();
	}
	int oidAxis = 0, oidMarker = 0;
	{
		vzm::GenerateAxisHelperObject(oidAxis, 0.15f);
		glm::fvec4 pos(0, 0, 0, 1.f);
		vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);
	}
	//cv::Mat cameraMatrix(3, 3, CV_64FC1);
	//memcpy(cameraMatrix.ptr(), arrayMatK, sizeof(double) * 9);
	//cv::Mat distCoeffs(5, 1, CV_64FC1);
	//memcpy(distCoeffs.ptr(), arrayDistCoeffs, sizeof(double) * 5);

	if(0)
	{
		cv::Mat img1 = cv::imread(folder_capture + "7823.png");
		cv::Mat undist_img1 = cv::imread(folder_capture + "undist_7823.png");
		cv::Mat undist_img2;
		cv::undistort(img1, undist_img2, cameraMatrix, distCoeffs);
		cv::flip(img1, img1, 1);
		cv::flip(undist_img1, undist_img1, 1);
		cv::flip(undist_img2, undist_img2, 1);

		cv::FileStorage fs(folder_capture + "7823.txt", cv::FileStorage::Mode::READ | cv::FileStorage::Mode::FORMAT_YAML);
		assert(fs.isOpened());
		cv::Mat pos2Ds;
		fs["circle_center_pos"] >> pos2Ds;
		fs.release();

		vector<cv::Point2f> __pts2ds(7);
		for (int i = 0; i < 7; i++) {
			float x = img1.cols - (float)pos2Ds.at<double>(i, 0);
			float y = (float)pos2Ds.at<double>(i, 1);
			__pts2ds[i] = cv::Point2f(x, y);

			cv::drawMarker(img1, __pts2ds[i], cv::Scalar(0, 0, 255), cv::MARKER_CROSS);
			cv::drawMarker(undist_img1, __pts2ds[i], cv::Scalar(0, 0, 255), cv::MARKER_CROSS);
			//cv::drawMarker(undist_img2, __pts2ds[i], cv::Scalar(0, 0, 255), cv::MARKER_CROSS);

			cv::imshow("img1", img1);
			cv::imshow("undist_img1", undist_img1);
			//cv::imshow("undist_img2", undist_img2);
		}

		cv::waitKey();
		cv::destroyAllWindows();
		return;
	}

	cArmCalibScans[folder_optiSession + "Take 2023-05-20 07.17.24 PM.csv"] = avr_trk(folder_capture + "8274.png", folder_capture + "8274.txt");
	cArmCalibScans[folder_optiSession + "Take 2023-05-20 07.18.27 PM.csv"] = avr_trk(folder_capture + "8275.png", folder_capture + "8275.txt");
	cArmCalibScans[folder_optiSession + "Take 2023-05-20 07.19.33 PM.csv"] = avr_trk(folder_capture + "8276.png", folder_capture + "8276.txt");
	cArmCalibScans[folder_optiSession + "Take 2023-05-20 07.19.47 PM.csv"] = avr_trk(folder_capture + "8277.png", folder_capture + "8277.txt");
	cArmCalibScans[folder_optiSession + "Take 2023-05-20 07.20.13 PM.csv"] = avr_trk(folder_capture + "8278.png", folder_capture + "8278.txt");
	cArmCalibScans[folder_optiSession + "Take 2023-05-20 07.22.07 PM.csv"] = avr_trk(folder_capture + "8280.png", folder_capture + "8280.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-20 07.21.03 PM.csv"] = avr_trk(folder_capture + "8279.png", folder_capture + "8279.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_009.csv"] = avr_trk(folder_capture + "7809.png", folder_capture + "7809.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_010.csv"] = avr_trk(folder_capture + "7810.png", folder_capture + "7810.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_011.csv"] = avr_trk(folder_capture + "7811.png", folder_capture + "7811.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_012.csv"] = avr_trk(folder_capture + "7812.png", folder_capture + "7812.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_013.csv"] = avr_trk(folder_capture + "7813.png", folder_capture + "7813.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_014.csv"] = avr_trk(folder_capture + "7814.png", folder_capture + "7814.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_015.csv"] = avr_trk(folder_capture + "7815.png", folder_capture + "7815.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_016.csv"] = avr_trk(folder_capture + "7817.png", folder_capture + "7817.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_017.csv"] = avr_trk(folder_capture + "7818.png", folder_capture + "7818.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_018.csv"] = avr_trk(folder_capture + "7819.png", folder_capture + "7819.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_019.csv"] = avr_trk(folder_capture + "7820.png", folder_capture + "7820.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_020.csv"] = avr_trk(folder_capture + "7821.png", folder_capture + "7821.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_021.csv"] = avr_trk(folder_capture + "7822.png", folder_capture + "7822.txt");
	//cArmCalibScans[folder_optiSession + "Take 2023-05-09 06.27.02 PM_023.csv"] = avr_trk(folder_capture + "7823.png", folder_capture + "7823.txt");
	
	set<string> cArmCalibScansExcluded;
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_001.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_002.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_003.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_004.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_005.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_006.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_008.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_009.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_010.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_011.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_012.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_013.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_014.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_015.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_016.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_017.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_018.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_019.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_020.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_021.csv");
	//cArmCalibScansExcluded.insert(folder_optiSession + "Take 2023-05-09 06.27.02 PM_023.csv");

	auto string2cid = [](const std::string id) {
		std::bitset<128> cid;
		unsigned __int64 lbs;
		unsigned __int64 hbs;
		int length = id.length();
		if (length <= 16) {
			std::stringstream convert(id);
			convert >> std::hex >> lbs;
			cid |= lbs;
		}
		else {
			std::string id_lbs, id_hbs;
			id_hbs = id.substr(0, length - 16);
			id_lbs = id.substr(length - 16, 16);
			std::stringstream convertH(id_hbs);
			std::stringstream convertL(id_lbs);
			convertH >> std::hex >> hbs;
			convertL >> std::hex >> lbs;
			cid |= hbs;
			cid <<= 64;
			cid |= lbs;
		}
		return cid;
	};

	int numValidScan = (int)(cArmCalibScans.size() - cArmCalibScansExcluded.size());
	vector<cv::Point2f> pts2ds(numCalibRbMKs* numValidScan);
	vector<cv::Point3f> ptsWsMk3ds(numCalibRbMKs* numValidScan);
	vector<cv::Point3f> ptsRbMk3ds(numCalibRbMKs* numValidScan);

	vector<cv::Point2f>& pts2ds_test = g_pts2ds; 
	pts2ds_test.assign(numCalibRbMKs* cArmCalibScans.size(), cv::Point2f());
	vector<cv::Point3f>& ptsWsMk3ds_test = g_ptsWsMk3ds;
	ptsWsMk3ds_test.assign(numCalibRbMKs * cArmCalibScans.size(), cv::Point3f());
	vector<cv::Point3f>& ptsRbMk3ds_test = g_ptsRbMk3ds;
	ptsRbMk3ds_test.assign(numCalibRbMKs* cArmCalibScans.size(), cv::Point3f());
	int scanCount = 0;
	int scanCount_test = 0;

	glm::ivec2 imageWH;
	cv::Mat imgCArm = cv::imread(cArmCalibScans.begin()->second.xRayScanImg);
	imageWH = glm::ivec2(imgCArm.cols, imgCArm.rows);

	// https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
	//cameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(imageWH.x, imageWH.y))

	for (auto& calibScan : cArmCalibScans) {

		//{
		//	cv::Mat img = cv::imread(calibScan.second.xRayScanImg);
		//	cv::Mat imgUndist;
		//	cv::undistort(img, imgUndist, _matK, _distCoeffs);
		//	string fileName = calibScan.second.xRayScanImg;
		//	size_t lastindex = fileName.find_last_of("/");
		//	string path = fileName.substr(0, lastindex + 1);
		//	string name = fileName.substr(lastindex + 1, fileName.length() - 1);
		//	cv::imwrite(path + "undist_" + name, imgUndist);
		//}

		// trackingFrames
		avr_trk& avrTrkInfo = calibScan.second;
		ifstream _file(folder_trackingInfo + "avrTrk" + to_string(scanCount_test) + ".txt");
		bool existFile = _file.is_open();
		_file.close();
		if (existFile) {
			cv::FileStorage fsTrk(folder_trackingInfo + "avrTrk" + to_string(scanCount_test) + ".txt", cv::FileStorage::Mode::READ | cv::FileStorage::Mode::FORMAT_YAML);
			cv::Mat cvMat;
			fsTrk["q_cArmRb"] >> cvMat;
			memcpy(&avrTrkInfo.q_cArmRb, cvMat.ptr(), sizeof(float) * 4);
			fsTrk["rot_cArmRb"] >> cvMat;
			memcpy(&avrTrkInfo.rot_cArmRb, cvMat.ptr(), sizeof(float) * 3);
			fsTrk["rotDeg_cArmRb"] >> avrTrkInfo.rotDeg_cArmRb;
			fsTrk["mat_rb2ws"] >> cvMat;
			memcpy(&avrTrkInfo.mat_rb2ws, cvMat.ptr(), sizeof(float) * 16);
			fsTrk["mat_ws2rb"] >> cvMat;
			memcpy(&avrTrkInfo.mat_ws2rb, cvMat.ptr(), sizeof(float) * 16);
			fsTrk["t_cArmRb"] >> cvMat;
			memcpy(&avrTrkInfo.t_cArmRb, cvMat.ptr(), sizeof(float) * 3);
			fsTrk["rbMSE_cArmRb"] >> avrTrkInfo.rbMSE_cArmRb;
			fsTrk["rbMSE_calibRb"] >> avrTrkInfo.rbMSE_calibRb;
			fsTrk["rbMarkers_calibRb"] >> cvMat;
			avrTrkInfo.rbMarkers_calibRb.assign(numCalibRbMKs, glm::fvec3(0));
			memcpy(&avrTrkInfo.rbMarkers_calibRb[0], cvMat.ptr(), sizeof(float) * 3 * numCalibRbMKs);
			fsTrk["wsMarkers_calibRb"] >> cvMat;
			avrTrkInfo.wsMarkers_calibRb.assign(numCalibRbMKs, glm::fvec3(0));
			memcpy(&avrTrkInfo.wsMarkers_calibRb[0], cvMat.ptr(), sizeof(float) * 3 * numCalibRbMKs);
			fsTrk.release();
		}
		else {
			rapidcsv::Document trackingData(calibScan.first, rapidcsv::LabelParams(0, 0));

			// trackingData
			int frameRowIdx = trackingData.GetRowIdx("Frame");
			//trackingData.GetDataRowIdx
			assert(frameRowIdx > 0);
			const int startRowIdx = frameRowIdx + 1;
			const int startColIdx = 1;

			std::vector<std::string> rowTypes = trackingData.GetRow<std::string>(frameRowIdx - 4);
			std::vector<std::string> rowNames = trackingData.GetRow<std::string>(frameRowIdx - 3);
			std::vector<std::string> rowIds = trackingData.GetRow<std::string>(frameRowIdx - 2);
			std::vector<std::string> rotPosLabels = trackingData.GetRow<std::string>(frameRowIdx - 1);
			std::vector<std::string> xyzwLabels = trackingData.GetRow<std::string>(frameRowIdx);
			const int numCols = (int)rowTypes.size();//trackingData.GetColumnCount();
			const int numRows = (int)trackingData.GetRowCount();

			std::vector<navihelpers::track_info> trackingFrames(numRows - startRowIdx);
			int frameIdx = 0;

			int numAllFramesRBs = 0;
			int numAllFramesMKs = 0;
			int numTrackedCArmRbFrames = 0;
			int numUnstableTrackingFrames = 0;

			for (int rowIdx = startRowIdx; rowIdx < numRows; rowIdx++) {
				std::vector<std::string> rowStrValues = trackingData.GetRow<std::string>(rowIdx);
				if (rowStrValues.size() == 0)
					continue;
				navihelpers::track_info& trackInfo = trackingFrames[frameIdx++];

				int colIdx = startColIdx;
				int targetTrackedMKs = 0;
				while (colIdx < numCols) {
					std::string type = rowTypes[colIdx];
					std::string name = rowNames[colIdx];
					std::string id = rowIds[colIdx];
					std::bitset<128> cid = string2cid(id);
					std::string rotPos = rotPosLabels[colIdx];
					std::string xyzw = xyzwLabels[colIdx];

					std::map<std::string, std::map<navihelpers::track_info::MKINFO, std::any>> rbmkSet;

					if (type == "Rigid Body") {
						if (rowIdx == startRowIdx) numAllFramesRBs++;
						std::string startStrValue = rowStrValues[colIdx];
						if (startStrValue == "") {
							colIdx += 8;
							for (; colIdx < numCols; colIdx += 4) {
								std::string _type = rowTypes[colIdx];
								if (_type != "Rigid Body Marker") break;
							}
							continue;
						}
						float values[8];
						for (int i = 0; i < 8; i++)
							values[i] = std::stof(rowStrValues[colIdx + i]);

						if (name == cArmRbName) numTrackedCArmRbFrames++;

						// read 8 col-successive cells
						glm::fquat q(values[3], values[0], values[1], values[2]);
						glm::fvec3 t(values[4], values[5], values[6]);
						float mkMSE = values[7];

						glm::fmat4x4 mat_r = glm::toMat4(q);
						glm::fmat4x4 mat_t = glm::translate(t);
						glm::fmat4x4 mat_ls2ws = mat_t * mat_r;

						colIdx += 8;

						for (; colIdx < numCols; colIdx += 4) {
							std::string _type = rowTypes[colIdx];
							if (_type != "Rigid Body Marker") break;
							std::string startMkStrValue = rowStrValues[colIdx];
							if (startMkStrValue == "") {
								continue;
							}
							float mkValues[4];
							for (int i = 0; i < 4; i++)
								mkValues[i] = std::stof(rowStrValues[colIdx + i]);


							// read 4 col-successive cells
							std::string _name = rowNames[colIdx];
							std::string _id = rowIds[colIdx];
							std::bitset<128> _cid = string2cid(_id);

							glm::fvec3 p(mkValues[0], mkValues[1], mkValues[2]);
							float mq = mkValues[3];
							auto& v = rbmkSet[_name];
							v[navihelpers::track_info::MKINFO::POSITION] = p;
							v[navihelpers::track_info::MKINFO::MK_QUALITY] = mq;
							v[navihelpers::track_info::MKINFO::MK_NAME] = _name;
							v[navihelpers::track_info::MKINFO::CID] = _cid;
						}

						trackInfo.AddRigidBody(name, mat_ls2ws, q, t, mkMSE, rbmkSet);
					}
					if (type == "Marker") {
						if (rowIdx == startRowIdx) numAllFramesMKs++;
						std::string startMkStrValue = rowStrValues[colIdx];
						if (startMkStrValue == "") {
							colIdx += 3;
							continue;
						}
						// read 3 col-successive cells
						float mkValues[3];
						for (int i = 0; i < 3; i++)
							mkValues[i] = std::stof(rowStrValues[colIdx + i]);
						glm::fvec3 p(mkValues[0], mkValues[1], mkValues[2]);
						trackInfo.AddMarker(cid, p, name);
						colIdx += 3;

						if (name.find(calibRbName + ":") != std::string::npos) {
							targetTrackedMKs++;
						}
					}
				}
				if (targetTrackedMKs < numCalibRbMKs) numUnstableTrackingFrames++;
			}

			cout << "*******************************************" << endl;
			cout << calibScan.first << endl;
			cout << "# of total frames : " << trackingFrames.size() << endl;
			cout << "# of dst tracked frames : " << numTrackedCArmRbFrames << endl;
			cout << "# of unstable tracking frames : " << numUnstableTrackingFrames << endl;
			float normalizedNum = 1.f / (float)numTrackedCArmRbFrames;

			avrTrkInfo.rot_cArmRb = glm::fvec3(0, 0, 0);
			avrTrkInfo.rotDeg_cArmRb = 0;
			avrTrkInfo.t_cArmRb = glm::fvec3(0, 0, 0);
			avrTrkInfo.rbMSE_cArmRb = 0;
			avrTrkInfo.rbMSE_calibRb = 0;
			avrTrkInfo.rbMarkers_calibRb.assign(numCalibRbMKs, glm::fvec3());
			vector<int> rbMKsCounts;
			rbMKsCounts.assign(numCalibRbMKs, 0);
			avrTrkInfo.wsMarkers_calibRb.assign(numCalibRbMKs, glm::fvec3());
			vector<int> wsMKsCounts;
			wsMKsCounts.assign(numCalibRbMKs, 0);

			int numFrames = (int)trackingFrames.size();
			for (int i = 0; i < numFrames; i++) {
				// c-arm rigid body
				navihelpers::track_info& trk = trackingFrames[i];
				glm::fquat q;
				glm::fvec3 t;
				trk.GetRigidBodyQuatTVecByName(cArmRbName, &q, &t);
				float quatMagnitude = glm::length(q);
				q /= quatMagnitude;

				float rotationAngle = 2.0f * acos(q.w);

				float sinAngle = sin(rotationAngle / 2.0f);
				glm::fvec3 rotationAxis = glm::fvec3(q.x, q.y, q.z) / sinAngle;

				avrTrkInfo.rot_cArmRb += rotationAxis * normalizedNum;
				avrTrkInfo.rotDeg_cArmRb += rotationAngle * normalizedNum;

				avrTrkInfo.t_cArmRb += t * normalizedNum;

				float mkMSE_carm;
				trk.GetRigidBodyByName(cArmRbName, NULL, &mkMSE_carm, NULL);
				avrTrkInfo.rbMSE_cArmRb += mkMSE_carm * normalizedNum;

				// calib rigid body
				std::map<std::string, std::map<navihelpers::track_info::MKINFO, std::any>> rbmkSet;
				float mkMSE_calib;
				trk.GetRigidBodyByName(calibRbName, NULL, &mkMSE_calib, &rbmkSet);
				avrTrkInfo.rbMSE_calibRb += mkMSE_calib * normalizedNum;
				assert(numCalibRbMKs == rbmkSet.size());

				for (int j = 0; j < numCalibRbMKs; j++) {
					string mkName = calibRbName + ":Marker" + to_string(j + 1);
					auto it = rbmkSet.find(mkName);
					assert(it != rbmkSet.end());
					rbMKsCounts[j]++;
					glm::fvec3 posMK = any_cast<glm::fvec3>(it->second[navihelpers::track_info::MKINFO::POSITION]);
					avrTrkInfo.rbMarkers_calibRb[j] += posMK * normalizedNum;
					std::map<navihelpers::track_info::MKINFO, std::any> mk;
					if (trk.GetMarkerByName(mkName, mk)) {
						wsMKsCounts[j]++;
						posMK = any_cast<glm::fvec3>(mk[navihelpers::track_info::MKINFO::POSITION]);
						avrTrkInfo.wsMarkers_calibRb[j] += posMK * normalizedNum;
					}

				}
			}
			avrTrkInfo.rot_cArmRb = glm::normalize(avrTrkInfo.rot_cArmRb);
			avrTrkInfo.q_cArmRb = glm::fquat(avrTrkInfo.rotDeg_cArmRb, avrTrkInfo.rot_cArmRb);
			for (int j = 0; j < numCalibRbMKs; j++) {
				if (rbMKsCounts[j] != numFrames) {
					cout << "Error!! rbMKsCounts[j] != numCalibRbMKs" << endl;
					avrTrkInfo.rbMarkers_calibRb[j] *= (rbMKsCounts[j] / normalizedNum);
				}
				if (wsMKsCounts[j] != numFrames) {
					cout << "Error!! wsMKsCounts[j] != numCalibRbMKs" << endl;
					avrTrkInfo.wsMarkers_calibRb[j] *= (wsMKsCounts[j] / normalizedNum);
				}
			}
			glm::fmat4x4 mat_r = glm::rotate(avrTrkInfo.rotDeg_cArmRb, avrTrkInfo.rot_cArmRb);
			glm::fmat4x4 mat_t = glm::translate(avrTrkInfo.t_cArmRb);
			avrTrkInfo.mat_rb2ws = mat_t * mat_r;
			avrTrkInfo.mat_ws2rb = glm::inverse(avrTrkInfo.mat_rb2ws);

			/*
			std::string xRayScanImg;
			std::string xRayCirclesResult;
			glm::fquat q_cArmRb; // c-arm rb
			glm::fvec3 rot_cArmRb; // c-arm rb
			float rotDeg_cArmRb; // c-arm rb
			glm::fmat4x4 mat_rb2ws; // c-arm rb
			glm::fmat4x4 mat_ws2rb; // c-arm rb

			glm::fvec3 t_cArmRb; // c-arm rb
			float rbMSE_cArmRb; // c-arm rb

			std::vector<glm::fvec3> rbMarkers_calibRb; // calib rb
			std::vector<glm::fvec3> wsMarkers_calibRb; // calib rb
			float rbMSE_calibRb; // c-arm rb
			//navihelpers::track_info avrTrkInfo;
			/**/
			cv::FileStorage fsTrk(folder_trackingInfo + "avrTrk" + to_string(scanCount_test) + ".txt", cv::FileStorage::Mode::WRITE);
			cv::Mat cvMat3 = cv::Mat(1, 3, CV_32FC1);
			cv::Mat cvMat4 = cv::Mat(1, 4, CV_32FC1);
			cv::Mat cvMat44 = cv::Mat(4, 4, CV_32FC1);
			cv::Mat cvMatPts = cv::Mat(numCalibRbMKs, 3, CV_32FC1);
			memcpy(cvMat4.ptr(), &avrTrkInfo.q_cArmRb, sizeof(float) * 4);
			fsTrk.write("q_cArmRb", cvMat4);
			memcpy(cvMat3.ptr(), &avrTrkInfo.rot_cArmRb, sizeof(float) * 3);
			fsTrk.write("rot_cArmRb", cvMat3);
			fsTrk.write("rotDeg_cArmRb", avrTrkInfo.rotDeg_cArmRb);
			memcpy(cvMat44.ptr(), &avrTrkInfo.mat_rb2ws, sizeof(float) * 16);
			fsTrk.write("mat_rb2ws", cvMat44);
			memcpy(cvMat44.ptr(), &avrTrkInfo.mat_ws2rb, sizeof(float) * 16);
			fsTrk.write("mat_ws2rb", cvMat44);
			memcpy(cvMat3.ptr(), &avrTrkInfo.t_cArmRb, sizeof(float) * 3);
			fsTrk.write("t_cArmRb", cvMat3);
			fsTrk.write("rbMSE_cArmRb", avrTrkInfo.rbMSE_cArmRb);
			fsTrk.write("rbMSE_calibRb", avrTrkInfo.rbMSE_calibRb);
			memcpy(cvMatPts.ptr(), &avrTrkInfo.rbMarkers_calibRb[0], sizeof(float) * 3 * numCalibRbMKs);
			fsTrk.write("rbMarkers_calibRb", cvMatPts);
			memcpy(cvMatPts.ptr(), &avrTrkInfo.wsMarkers_calibRb[0], sizeof(float) * 3 * numCalibRbMKs);
			fsTrk.write("wsMarkers_calibRb", cvMatPts);
			fsTrk.release();
		}

		// Add a capture Scene
		int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
		if(sidScene != 0)
		{
			int aidGroupCArmCam = 0;
			vzm::ActorParameters apGroupCams;
			vzm::NewActor(apGroupCams, "Tracking C-Arm Group {" + to_string(scanCount + 1) + "}", aidGroupCArmCam);
			vzm::AppendSceneItemToSceneTree(aidGroupCArmCam, sidScene);
			
			vzm::ActorParameters apAxis;
			apAxis.SetResourceID(vzm::ActorParameters::GEOMETRY, oidAxis);
			apAxis.is_visible = true;
			apAxis.line_thickness = 3;
			apAxis.SetLocalTransform(__FP avrTrkInfo.mat_rb2ws);
			int aidRbAxis = 0;
			string actorName = cArmRbName + ":Scan" + to_string(scanCount + 1);
			vzm::NewActor(apAxis, actorName, aidRbAxis);
			vzm::AppendSceneItemToSceneTree(aidRbAxis, aidGroupCArmCam);

			std::vector<glm::fvec3> pinfo(3);
			pinfo[0] = glm::fvec3(0, 0, 0);
			pinfo[1] = glm::fvec3(-1, 0, 0);
			pinfo[2] = glm::fvec3(0, -1, 0);
			int oidLabelText = 0;
			vzm::GenerateTextObject((float*)&pinfo[0], actorName, 0.07, true, false, oidLabelText);
			vzm::ActorParameters apLabelText;
			apLabelText.SetResourceID(vzm::ActorParameters::GEOMETRY, oidLabelText);
			*(glm::fvec4*)apLabelText.phong_coeffs = glm::fvec4(0, 1, 0, 0);
			int aidLabelText = 0;
			vzm::NewActor(apLabelText, actorName + ":Label", aidLabelText);
			vzm::AppendSceneItemToSceneTree(aidLabelText, aidRbAxis);
			
			for (int i = 0; i < numCalibRbMKs; i++) {
				vzm::ActorParameters apMarker;
				apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
				apMarker.is_visible = true;
				apMarker.is_pickable = true;

				glm::fvec3 posRb = avrTrkInfo.rbMarkers_calibRb[i];
				glm::fvec3 posWs = avrTrkInfo.wsMarkers_calibRb[i];
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.005f)); // set 1 cm to the marker diameter
				glm::fmat4x4 matLS2WS_Rb = glm::translate(posRb);
				glm::fmat4x4 matLS2WS_Ws = glm::translate(posWs);
				matLS2WS_Rb = matLS2WS_Rb * matScale;
				matLS2WS_Ws = matLS2WS_Ws * matScale;

				*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 1.f, 0, 1.f); // rgba
				apMarker.SetLocalTransform(__FP matLS2WS_Rb);
				int aidMarker_Rb = 0;
				vzm::NewActor(apMarker, "Rb" + to_string(i) + ":Scan" + to_string(scanCount + 1), aidMarker_Rb);
				vzm::AppendSceneItemToSceneTree(aidMarker_Rb, aidGroupCArmCam);

				*(glm::fvec4*)apMarker.color = glm::fvec4(0, 1.f, 1.f, 1.f); // rgba
				apMarker.SetLocalTransform(__FP matLS2WS_Ws);
				int aidMarker_Ws = 0;
				vzm::NewActor(apMarker, "Ws" + to_string(i) + ":Scan" + to_string(scanCount + 1), aidMarker_Ws);
				vzm::AppendSceneItemToSceneTree(aidMarker_Ws, aidGroupCArmCam);

				std::vector<glm::fvec3> pinfo(3);
				pinfo[0] = glm::fvec3(0, 1, 0);
				pinfo[1] = glm::fvec3(0, 0, -1);
				pinfo[2] = glm::fvec3(0, -1, 0);
				int oidLabelText_Rb = 0, oidLabelText_Ws = 0;
				vzm::GenerateTextObject((float*)&pinfo[0], "Rb" + to_string(i) + ":Scan" + to_string(scanCount + 1), 2, true, false, oidLabelText_Rb, true);
				vzm::GenerateTextObject((float*)&pinfo[0], "Ws" + to_string(i) + ":Scan" + to_string(scanCount + 1), 2, true, false, oidLabelText_Ws, true);
				
				vzm::ActorParameters apLabelText;
				*(glm::fvec4*)apLabelText.phong_coeffs = glm::fvec4(0, 1, 0, 0);
				//apLabelText.script_params.SetParam("_bool_IsScaleFree", true);

				apLabelText.SetResourceID(vzm::ActorParameters::GEOMETRY, oidLabelText_Rb);
				int aidLabelText_Rb = 0;
				vzm::NewActor(apLabelText, "Rb" + to_string(i) + ":Scan" + to_string(scanCount + 1) + ":Label", aidLabelText_Rb);
				vzm::AppendSceneItemToSceneTree(aidLabelText_Rb, aidMarker_Rb);

				apLabelText.SetResourceID(vzm::ActorParameters::GEOMETRY, oidLabelText_Ws);
				int aidLabelText_Ws = 0;
				vzm::NewActor(apLabelText, "Ws" + to_string(i) + ":Scan" + to_string(scanCount + 1) + ":Label", aidLabelText_Ws);
				vzm::AppendSceneItemToSceneTree(aidLabelText_Ws, aidMarker_Ws);
			}

		}

		// compute calibation
		const bool mirrorHorizontal = true;

		cv::FileStorage fs(avrTrkInfo.xRayCirclesResult, cv::FileStorage::Mode::READ | cv::FileStorage::Mode::FORMAT_YAML);
		assert(fs.isOpened());
		cv::Mat pos2Ds;
		fs["undist_circle_center_pos"] >> pos2Ds;
		fs.release();

		vector<cv::Point2f> pts2d_single(numCalibRbMKs);
		vector<cv::Point3f> ptsRb3d_single(numCalibRbMKs);
		vector<cv::Point3f> ptsWs3d_single(numCalibRbMKs);

		bool isCalibScan = cArmCalibScansExcluded.find(calibScan.first) == cArmCalibScansExcluded.end();
		for (int i = 0; i < numCalibRbMKs; i++) {
			float x = mirrorHorizontal? imageWH.x - (float)pos2Ds.at<double>(i, 0) : (float)pos2Ds.at<double>(i, 0);
			float y = (float)pos2Ds.at<double>(i, 1);
			pts2ds_test[(scanCount_test)*numCalibRbMKs + i] = cv::Point2f(x, y);
			if(isCalibScan) pts2ds[(scanCount)*numCalibRbMKs + i] = cv::Point2f(x, y);
		}
		memcpy(&pts2d_single[0], &pts2ds_test[(scanCount_test)*numCalibRbMKs], sizeof(cv::Point2f) * numCalibRbMKs);

		for (int i = 0; i < numCalibRbMKs; i++) {
			glm::fvec3 p1 = vzmutils::transformPos(avrTrkInfo.rbMarkers_calibRb[i], avrTrkInfo.mat_ws2rb);
			glm::fvec3 p2 = vzmutils::transformPos(avrTrkInfo.wsMarkers_calibRb[i], avrTrkInfo.mat_ws2rb);
			memcpy(&ptsRbMk3ds_test[(scanCount_test)*numCalibRbMKs + i], &p1, sizeof(glm::fvec3));
			memcpy(&ptsWsMk3ds_test[(scanCount_test)*numCalibRbMKs + i], &p2, sizeof(glm::fvec3));
			if (isCalibScan) {
				memcpy(&ptsRbMk3ds[(scanCount)*numCalibRbMKs + i], &p1, sizeof(glm::fvec3));
				memcpy(&ptsWsMk3ds[(scanCount)*numCalibRbMKs + i], &p2, sizeof(glm::fvec3));
			}
		}
		memcpy(&ptsRb3d_single[0], &ptsRbMk3ds_test[(scanCount_test)*numCalibRbMKs], sizeof(cv::Point3f) * numCalibRbMKs);
		memcpy(&ptsWs3d_single[0], &ptsWsMk3ds_test[(scanCount_test)*numCalibRbMKs], sizeof(cv::Point3f) * numCalibRbMKs);


		for (int i = 0; i < 2; i++) {
			cv::Mat rvec, tvec;
			vector<cv::Point3f>& pts3d_single = i == 0 ? ptsRb3d_single : ptsWs3d_single;
			cv::solvePnP(pts3d_single, pts2d_single, cameraMatrix, distCoeffs, rvec, tvec);

			{
				// Reproject the 3D points onto the image plane using the camera calibration
				std::vector<cv::Point2f> imagePointsReprojected;
				cv::projectPoints(pts3d_single, rvec, tvec, cameraMatrix, distCoeffs, imagePointsReprojected);

				// Compute the reprojection error
				float maxErr = 0;
				float reprojectionError = ComputeReprojErr(pts2d_single, imagePointsReprojected, maxErr);
				//double reprojectionError = cv::norm(cv::Mat(pts2d_single), cv::Mat(imagePointsReprojected), cv::NORM_L2) / pts2d_single.size();
				//cout << (i == 0 ? "Rb" : "Ws") << " reprojectionError : " << reprojectionError << endl;
				cout << (i == 0 ? "Rb" : "Ws") << " reprojectionError : " << reprojectionError << ", max error : " << maxErr << endl;
			}

			{
				cv::FileStorage fs(folder_trackingInfo + "rb2carm" + to_string(i) + ".txt", cv::FileStorage::Mode::WRITE);
				fs.write("rvec", rvec);
				fs.write("tvec", tvec);
				fs.release();
			}
		}

		if (isCalibScan) {
			scanCount++;
		}
		scanCount_test++;

		//vector<cv::Point2i> points2Ds(numCalibRbMKs);
		//memcpy(&points2Ds[0], pos2Ds.ptr(), sizeof(cv::Point2f) * 7);
		//// pts2ds[scanCount]
		//std::map<int, cv::Point2f> pts2dMap; 
		//for(int i = 0; i < numCalibRbMKs; i++)
		//	pts2dMap[(1 + scanCount) * 100 + i] = cv::Point2f((float)pos2Ds.at<int>(i, 0), (float)pos2Ds.at<int>(i, 1));
	}

	{
		cv::Mat rvec, tvec;
		ComputePose(pts2ds, ptsRbMk3ds, ptsWsMk3ds, cameraMatrix, distCoeffs, rvec, tvec,
			pts2ds_test, ptsRbMk3ds_test, ptsWsMk3ds_test);
	}

	// scene 2
	{
		int sidScene2 = 0;
		vzm::NewScene("Scene2", sidScene2);

		vzm::ActorParameters apAxis;
		apAxis.SetResourceID(vzm::ActorParameters::GEOMETRY, oidAxis);
		apAxis.is_visible = true;
		apAxis.line_thickness = 5;
		int aidRbAxis = 0;
		vzm::NewActor(apAxis, "Scene2 Axis", aidRbAxis);
		vzm::AppendSceneItemToSceneTree(aidRbAxis, sidScene2);

		glm::fmat4x4 matCA2RB;
		for (int i = 0; i < 2; i++) { // Rb and then Ws

			cv::Mat rvec, tvec;
			cv::FileStorage fs(folder_trackingInfo + "rb2carm" + to_string(i) + ".txt", cv::FileStorage::Mode::READ);
			fs["rvec"] >> rvec;
			fs["tvec"] >> tvec;
			fs.release();

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

			matCA2RB = glm::inverse(matRB2CA);
			//glm::fmat4x4 matRB2WS;

			assert(pts2ds.size() == ptsRbMk3ds.size() && pts2ds.size() == ptsWsMk3ds.size());
			int numSamples = (int)pts2ds.size();
			for (int j = 0; j < numSamples; j++) {
				vzm::ActorParameters apMarker;
				apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
				apMarker.is_visible = true;
				apMarker.is_pickable = true;

				glm::fvec3 pos3d = *(glm::fvec3*)&(i == 0 ? ptsRbMk3ds[j] : ptsWsMk3ds[j]);
				if (i == 0) {
					g_posCenterMKsRBS += pos3d / (float)numSamples;
				}
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.005f)); // set 1 cm to the marker diameter
				glm::fmat4x4 matLS2RB = glm::translate(pos3d);
				matLS2RB = matLS2RB * matScale;

				*(glm::fvec4*)apMarker.color = i == 0? glm::fvec4(1.f, 0, 0, 1.f) : glm::fvec4(0, 1.f, 0, 1.f); // rgba
				apMarker.SetLocalTransform(__FP matLS2RB);
				// _float_VZThickness : note this parameter is only available for K-buffer rendering mode 
				// current rendering is based on my modified A-buffer + global z-thickness technique 
				//apMarker.test_params.SetParam("_float_VZThickness", 0.1f);
				int aidMarker = 0;
				string actorMkName = (i == 0 ? "Rb" : "Ws") + to_string(j);
				vzm::NewActor(apMarker, actorMkName, aidMarker);
				vzm::AppendSceneItemToSceneTree(aidMarker, sidScene2);

				/*
				std::vector<glm::fvec3> pinfo(3);
				pinfo[0] = glm::fvec3(0, 0, 1);
				pinfo[1] = glm::fvec3(1, 0, 0);
				pinfo[2] = glm::fvec3(0, 0, 1);
				int oidLabelText = 0;
				string labeMkName = (i == 0 ? "Rb:" : "Ws:") + to_string(j);
				vzm::GenerateTextObject((float*)&pinfo[0], labeMkName, 2, true, false, oidLabelText, true);

				vzm::ActorParameters apLabelText;
				*(glm::fvec4*)apLabelText.phong_coeffs = glm::fvec4(0, 1, 0, 0);
				//apLabelText.script_params.SetParam("_bool_IsScaleFree", true);

				apLabelText.SetResourceID(vzm::ActorParameters::GEOMETRY, oidLabelText);
				int aidLabelText = 0;
				vzm::NewActor(apLabelText, labeMkName + ":Label", aidLabelText);
				vzm::AppendSceneItemToSceneTree(aidLabelText, aidMarker);
				/**/
			}
		}

		vzm::CameraParameters cpCam2;
		*(glm::fvec3*)cpCam2.pos = glm::fvec3(-1.5, 1.5, -1.5);
		*(glm::fvec3*)cpCam2.up = glm::fvec3(0, 1, 0);
		*(glm::fvec3*)cpCam2.view = glm::fvec3(1, -1, 1);

		cv::FileStorage fs(folder_data + "SceneCamPose2.txt", cv::FileStorage::Mode::READ);
		if (fs.isOpened()) {
			cv::Mat ocvVec3;
			fs["POS"] >> ocvVec3;
			memcpy(cpCam2.pos, ocvVec3.ptr(), sizeof(float) * 3);
			fs["VIEW"] >> ocvVec3;
			memcpy(cpCam2.view, ocvVec3.ptr(), sizeof(float) * 3);
			fs["UP"] >> ocvVec3;
			memcpy(cpCam2.up, ocvVec3.ptr(), sizeof(float) * 3);
			fs.release();
		}

		RECT rc;
		GetClientRect(g_hWndDialog1, &rc);
		UINT widthWindow = rc.right - rc.left;
		UINT heightWindow = rc.bottom - rc.top;
		cpCam2.w = widthWindow;
		cpCam2.h = heightWindow;

		cpCam2.np = 0.1f;
		cpCam2.fp = 100.f;

		float vFov = 3.141592654f / 4.f;
		if (0) {
			cpCam2.fov_y = vFov;
			cpCam2.aspect_ratio = (float)cpCam2.w / (float)cpCam2.h;
			cpCam2.projection_mode = vzm::CameraParameters::CAMERA_FOV;
		}
		else {
			float aspect_ratio = (float)cpCam2.w / (float)cpCam2.h;
			float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;

			//  fy = h/(2 * tan(f_y / 2)) 
			cpCam2.fx = cpCam2.w / (2.f * tan(hFov / 2.f));
			cpCam2.fy = cpCam2.h / (2.f * tan(vFov / 2.f));
			cpCam2.sc = 0;
			cpCam2.cx = cpCam2.w / 2.f;
			cpCam2.cy = cpCam2.h / 2.f;
			cpCam2.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
		}

		cpCam2.hWnd = USE_WHND ? g_hWndDialog1 : NULL;

		const float intrinsicRatioX = (float)imageWH.x / widthWindow;
		const float intrinsicRatioY = (float)imageWH.y / heightWindow;

		cpCam2.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_INTRINSICS;
		glm::fvec3 pos(0, 0, 0);
		glm::fvec3 view(0, 0, 1);
		glm::fvec3 up(0, -1, 0);
		pos = *(glm::fvec3*)cpCam2.pos = vzmutils::transformPos(pos, matCA2RB);
		view = *(glm::fvec3*)cpCam2.view = vzmutils::transformVec(view, matCA2RB);
		up = *(glm::fvec3*)cpCam2.up = vzmutils::transformVec(up, matCA2RB);
		cpCam2.fx = cameraMatrix.at<double>(0, 0) / intrinsicRatioX;
		cpCam2.fy = cameraMatrix.at<double>(1, 1) / intrinsicRatioY;
		cpCam2.sc = cameraMatrix.at<double>(0, 1);
		cpCam2.cx = cameraMatrix.at<double>(0, 2) / intrinsicRatioX;
		cpCam2.cy = cameraMatrix.at<double>(1, 2) / intrinsicRatioY;
		cpCam2.np = 0.1;
		int cidCam2 = 0;
		vzm::NewCamera(cpCam2, "CArm Camera", cidCam2);

		vzm::LightParameters lpLight2;
		lpLight2.is_on_camera = true;
		lpLight2.is_pointlight = false;
		*(glm::fvec3*)lpLight2.pos = *(glm::fvec3*)cpCam2.pos;
		*(glm::fvec3*)lpLight2.dir = *(glm::fvec3*)cpCam2.view;
		*(glm::fvec3*)lpLight2.up = *(glm::fvec3*)cpCam2.up;
		int lidLight2 = 0;
		vzm::NewLight(lpLight2, "World Light 2", lidLight2);

		vzm::AppendSceneItemToSceneTree(cidCam2, sidScene2);
		vzm::AppendSceneItemToSceneTree(lidLight2, sidScene2);

		{
			// remove actor/.... remove associated resource... check if refcount == 0
			std::vector<glm::fvec3> pos_tris, pos_lines, clr_tris, clr_lines;
			navihelpers::Make_ViewportFromCamParams(pos_tris, clr_tris, pos_lines, clr_lines, glm::fvec3(1, 0.3, 0.5), glm::fvec3(1, 0.3, 0.5),
				pos, view, up, cpCam2.fx, cpCam2.fy, cpCam2.sc, cpCam2.cx, cpCam2.cy, cpCam2.w, cpCam2.h, 1.2f);
			int oidCamTris = 0, oidCamLines = 0;
			vzm::GenerateTrianglesObject((float*)&pos_tris[0], (float*)&clr_tris[0], (int)pos_tris.size() / 3, oidCamTris);
			vzm::GenerateLinesObject((float*)&pos_lines[0], (float*)&clr_lines[0], (int)pos_lines.size() / 2, oidCamLines);

			vzm::ActorParameters apCamTris, apCamLines;
			apCamTris.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamTris);
			apCamTris.color[3] = 0.5f;
			apCamLines.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLines);
			*(glm::fvec4*)apCamLines.phong_coeffs = glm::fvec4(0, 1, 0, 0);
			apCamLines.line_thickness = 2;
			int aidCamTris = 0, aidCamLines = 0;
			vzm::NewActor(apCamTris, "C-Arm Cam Tris:test0", aidCamTris);
			vzm::NewActor(apCamLines, "C-Arm Lines:test0", aidCamLines);
			vzm::AppendSceneItemToSceneTree(aidCamTris, sidScene2);
			vzm::AppendSceneItemToSceneTree(aidCamLines, sidScene2);
		}
	}
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

	auto getdatapath = []()
	{
		using namespace std;
		char ownPth[2048];
		GetModuleFileNameA(NULL, ownPth, (sizeof(ownPth)));
		string exe_path = ownPth;
		string exe_path_;
		size_t pos = 0;
		std::string token;
		string delimiter = "\\"; // windows
		while ((pos = exe_path.find(delimiter)) != std::string::npos) {
			token = exe_path.substr(0, pos);
			if (token.find(".exe") != std::string::npos) break;
			exe_path += token + "/";
			exe_path_ += token + "/";
			exe_path.erase(0, pos + delimiter.length());
		}
		return exe_path_ + "../../data/";
	};

	folder_data = getdatapath();
	folder_trackingInfo = getdatapath() + "Tracking 2023-05-20/";
	folder_capture = getdatapath() + "c-arm 2023-05-20/";
	folder_optiSession = getdatapath() + "Session 2023-05-20/";

	// TODO: Place code here.
	vzm::InitEngineLib("SpineNavi");
	vzm::SetLogConfiguration(true, 4);

	// Initialize global strings
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_SCALIBRATORWIN, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // Perform application initialization:
    if (!InitInstance (hInstance, nCmdShow))
    {
        return FALSE;
    }

	SceneInit();	// camera setting inside SceneInit should be called after InitInstance (which sets windows properties)
	CalibrateCamPoseForCArmRB(folder_trackingInfo + "carm_intrinsics.txt", "c-arm", "test1", 8);

	int sidScene1 = vzmutils::GetSceneItemIdByName(g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(g_camName);
	int sidScene2 = vzmutils::GetSceneItemIdByName(g_sceneName2);
	int cidCam2 = vzmutils::GetSceneItemIdByName(g_camName2);
	vzm::SetRenderTestParam("_float_GIVZThickness", 0.001f, sidScene1, cidCam1);
	vzm::SetRenderTestParam("_float_GIVZThickness", 0.001f, sidScene2, cidCam2);

	SetTimer(g_hWnd, NULL, 10, TimerProc);

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_SCALIBRATORWIN));
    MSG msg;

    // Main message loop:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
			if (IsDialogMessage(g_hWndDialog1, &msg) && msg.message == WM_KEYDOWN) {
				DiagProc1(g_hWndDialog1, msg.message, msg.wParam, msg.lParam);
			}
			else {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
        }
	}

	vzm::RemoveSceneItem(sidScene1, true);
	vzm::RemoveSceneItem(sidScene2, true);
	vzm::DeinitEngineLib();

    return (int) msg.wParam;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style          = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_SCALIBRATORWIN));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_SCALIBRATORWIN);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // Store instance handle in our global variable

   //HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
   //   CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	   10, 10, DESIRED_SCREEN_W, DESIRED_SCREEN_H, nullptr, nullptr, hInstance, nullptr);

   g_hWnd = hWnd;

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   
   g_hWndDialog1 = CreateDialog(hInst, MAKEINTRESOURCE(IDD_FORMVIEW), hWnd, DiagProc1);

   RECT rc1;
   GetWindowRect(g_hWnd, &rc1);
   SetWindowPos(g_hWndDialog1, 0, rc1.right + 5,10, DESIRED_SCREEN_W, DESIRED_SCREEN_H, SWP_SHOWWINDOW);
   if (!g_hWndDialog1)
   {
	   return FALSE;
   }
   ShowWindow(g_hWndDialog1, SW_SHOW);
   UpdateWindow(g_hWndDialog1);
   /**/
   return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE: Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(g_camName);
	float scene_stage_scale = 5.f;
	static int activeCarmIdx = 0;

	glm::fvec3 scene_stage_center = glm::fvec3();
	vzm::CameraParameters cpCam1;
	if (sidScene != 0 && cidCam1 != 0) {
		vzm::GetCameraParams(cidCam1, cpCam1);

		scene_stage_scale = glm::length(scene_stage_center - *(glm::fvec3*)cpCam1.pos) * 1.0f;
	}

	static vzmutils::GeneralMove general_move;
	static vzm::CameraParameters cpPrevCam;

#define INTERPOLCAM(PARAM) _cpCam.PARAM = cpCam1.PARAM + (cpNewCam1.PARAM - cpCam1.PARAM) * t;

	switch (message)
	{
	case WM_KEYDOWN:
	{
		using namespace std;
		switch (wParam) {
			case char('S') :
			{
				cv::FileStorage fs(folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::WRITE);
				cv::Mat ocvVec3(1, 3, CV_32FC1);
				memcpy(ocvVec3.ptr(), cpCam1.pos, sizeof(float) * 3);
				fs.write("POS", ocvVec3);
				memcpy(ocvVec3.ptr(), cpCam1.view, sizeof(float) * 3);
				fs.write("VIEW", ocvVec3);
				memcpy(ocvVec3.ptr(), cpCam1.up, sizeof(float) * 3);
				fs.write("UP", ocvVec3);
				fs.release();
			}break;
			case char('L') :
			{
				cv::FileStorage fs(folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
				if (fs.isOpened()) {
					cv::Mat ocvVec3;
					fs["POS"] >> ocvVec3;
					memcpy(cpCam1.pos, ocvVec3.ptr(), sizeof(float) * 3);
					fs["VIEW"] >> ocvVec3;
					memcpy(cpCam1.view, ocvVec3.ptr(), sizeof(float) * 3);
					fs["UP"] >> ocvVec3;
					memcpy(cpCam1.up, ocvVec3.ptr(), sizeof(float) * 3);
					fs.release();
				}
				vzm::SetCameraParams(cidCam1, cpCam1);
			}break;
			case char('A') :
			{
				cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
				cv::Mat cameraMatrix;
				fs["K"] >> cameraMatrix;
				fs.release();

				int aidCArmPlane = vzmutils::GetSceneItemIdByName("CArm Plane:test" + to_string(activeCarmIdx));

				vzm::ActorParameters apCArmPlane;
				vzm::GetActorParams(aidCArmPlane, apCArmPlane);
				glm::fmat4x4 matCA2WS = apCArmPlane.script_params.GetParam("matCA2WS", glm::fmat4x4(1));
				glm::ivec2 imageWH = apCArmPlane.script_params.GetParam("imageWH", glm::ivec2(0));

				vzm::CameraParameters cpNewCam1 = cpCam1;
				cpPrevCam = cpCam1;

				glm::fvec3 posPrev = *(glm::fvec3*)cpCam1.pos;
				glm::fvec3 viewPrev = *(glm::fvec3*)cpCam1.view;
				glm::fvec3 upPrev = *(glm::fvec3*)cpCam1.up;

				RECT rc;
				GetClientRect(g_hWnd, &rc);
				UINT widthWindow = rc.right - rc.left;
				UINT heightWindow = rc.bottom - rc.top;

				const float intrinsicRatioX = (float)imageWH.x / widthWindow;
				const float intrinsicRatioY = (float)imageWH.y / heightWindow;

				cpNewCam1.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_INTRINSICS;
				glm::fvec3 pos(0, 0, 0);
				glm::fvec3 view(0, 0, 1);
				glm::fvec3 up(0, -1, 0);
				pos = *(glm::fvec3*)cpNewCam1.pos = vzmutils::transformPos(pos, matCA2WS);
				view = *(glm::fvec3*)cpNewCam1.view = vzmutils::transformVec(view, matCA2WS);
				up = *(glm::fvec3*)cpNewCam1.up = vzmutils::transformVec(up, matCA2WS);
				cpNewCam1.fx = cameraMatrix.at<double>(0, 0) / intrinsicRatioX;
				cpNewCam1.fy = cameraMatrix.at<double>(1, 1) / intrinsicRatioY;
				cpNewCam1.sc = cameraMatrix.at<double>(0, 1);
				cpNewCam1.cx = cameraMatrix.at<double>(0, 2) / intrinsicRatioX;
				cpNewCam1.cy = cameraMatrix.at<double>(1, 2) / intrinsicRatioY;
				cpNewCam1.np = 0.2;

				glm::fquat q1 = glm::quatLookAtRH(glm::normalize(viewPrev), upPrev); // to world
				//glm::fvec3 _o(0, 0, 0);
				//glm::fmat4x4 m1 = glm::lookAtRH(_o, viewPrev, upPrev);
				//glm::fmat4x4 m2 = glm::toMat4(q1);
				//glm::fmat4x4 m3 = glm::inverse(m2);
				glm::fquat q2 = glm::quatLookAtRH(glm::normalize(view), up);

				float range = std::max((float)(numAnimationCount - 1), 1.f);
				for (int i = 0; i < numAnimationCount; i++) {
					float t = (float)i / range; // 0 to 1
					glm::fquat q = glm::slerp(q1, q2, t);
					glm::fmat4x4 invMatLookAt = glm::toMat4(q);
					//glm::fmat4x4 invMat = glm::inverse(matLookAt);
					glm::fvec3 _view(0, 0, -1);
					glm::fvec3 _up(0, 1, 0);
					_view = vzmutils::transformVec(_view, invMatLookAt);
					_up = vzmutils::transformVec(_up, invMatLookAt);

					vzm::CameraParameters& _cpCam = cpInterCams[i];
					_cpCam = cpNewCam1;
					*(glm::fvec3*)_cpCam.pos = posPrev + (pos - posPrev) * t;
					*(glm::fvec3*)_cpCam.view = _view;
					*(glm::fvec3*)_cpCam.up = _up;

					//INTERPOLCAM(w);
					//INTERPOLCAM(h);
					INTERPOLCAM(fx);
					INTERPOLCAM(fy);
					INTERPOLCAM(sc);
					INTERPOLCAM(cx);
					INTERPOLCAM(cy);
				}
				arAnimationKeyFrame = 0;
				//vzm::SetCameraParams(cidCam1, cpNewCam1);

				//cpCam1.
				// Call SetWindowPos to resize the window
				//UINT flags = SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE;
				//SetWindowPos(g_hWnd, NULL, 100, 100, imageWH.x, imageWH.y, flags);
			} break;
			case char('Z') :
			{
				vzm::CameraParameters cpNewCam1 = cpPrevCam;

				glm::fvec3 posPrev = *(glm::fvec3*)cpCam1.pos;
				glm::fvec3 viewPrev = *(glm::fvec3*)cpCam1.view;
				glm::fvec3 upPrev = *(glm::fvec3*)cpCam1.up;

				const float intrinsicRatioX = 1.f;
				const float intrinsicRatioY = 1.f;

				cpNewCam1.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_INTRINSICS;
				glm::fvec3 pos(0, 0, 0);
				glm::fvec3 view(0, 0, 1);
				glm::fvec3 up(0, -1, 0);
				pos = *(glm::fvec3*)cpNewCam1.pos;
				view = *(glm::fvec3*)cpNewCam1.view;
				up = *(glm::fvec3*)cpNewCam1.up;

				glm::fquat q1 = glm::quatLookAtRH(glm::normalize(viewPrev), upPrev); // to world
				//glm::fvec3 _o(0, 0, 0);
				//glm::fmat4x4 m1 = glm::lookAtRH(_o, viewPrev, upPrev);
				//glm::fmat4x4 m2 = glm::toMat4(q1);
				//glm::fmat4x4 m3 = glm::inverse(m2);
				glm::fquat q2 = glm::quatLookAtRH(glm::normalize(view), up);

				float range = std::max((float)(numAnimationCount - 1), 1.f);
				for (int i = 0; i < numAnimationCount; i++) {
					float t = (float)i / range; // 0 to 1
					glm::fquat q = glm::slerp(q1, q2, t);
					glm::fmat4x4 invMatLookAt = glm::toMat4(q);
					//glm::fmat4x4 invMat = glm::inverse(matLookAt);
					glm::fvec3 _view(0, 0, -1);
					glm::fvec3 _up(0, 1, 0);
					_view = vzmutils::transformVec(_view, invMatLookAt);
					_up = vzmutils::transformVec(_up, invMatLookAt);

					vzm::CameraParameters& _cpCam = cpInterCams[i];
					_cpCam = cpNewCam1;
					*(glm::fvec3*)_cpCam.pos = posPrev + (pos - posPrev) * t;
					*(glm::fvec3*)_cpCam.view = _view;
					*(glm::fvec3*)_cpCam.up = _up;

					//INTERPOLCAM(w);
					//INTERPOLCAM(h);
					INTERPOLCAM(fx);
					INTERPOLCAM(fy);
					INTERPOLCAM(sc);
					INTERPOLCAM(cx);
					INTERPOLCAM(cy);
				}
				arAnimationKeyFrame = 0;
				activeCarmIdx = -1;
				//vzm::SetCameraParams(cidCam1, cpNewCam1);

				//cpCam1.
				// Call SetWindowPos to resize the window
				//UINT flags = SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE;
				//SetWindowPos(g_hWnd, NULL, 100, 100, imageWH.x, imageWH.y, flags);
			} break;
		}

		static string trkNames[10] = {
			  folder_optiSession + "Take 2023-05-20 07.17.24 PM.csv"
			, folder_optiSession + "Take 2023-05-20 07.18.27 PM.csv"
			, folder_optiSession + "Take 2023-05-20 07.19.33 PM.csv"
			, folder_optiSession + "Take 2023-05-20 07.19.47 PM.csv"
			, folder_optiSession + "Take 2023-05-20 07.20.13 PM.csv"
			, folder_optiSession + "Take 2023-05-20 07.22.07 PM.csv"
			, folder_optiSession + "a"
			, folder_optiSession + "Take 2023-05-09 06.27.02 PM_009.csv"
			, folder_optiSession + "Take 2023-05-09 06.27.02 PM_010.csv"
			, folder_optiSession + "Take 2023-05-09 06.27.02 PM_011.csv"
		};
		//static map<int, int> mapAidGroupCArmCam;
		static char LOADKEYS[10] = { '1' , '2', '3', '4', '5', '6', '7', '8', '9', '0' };
		for (int i = 0; i < 10; i++) {
			if (wParam == LOADKEYS[i]) {
				// load case
				auto itScan = cArmCalibScans.find(trkNames[i]);
				if (itScan == cArmCalibScans.end())
					continue;
				
				string calibPosInfoFile = "calib_test" + to_string(i) + ".txt";
				avr_trk& trk = itScan->second;
				StoreParams(folder_trackingInfo, calibPosInfoFile, trk.mat_rb2ws, trk.xRayScanImg, false); // key 1

				static int aidGroup = 0;
				if(aidGroup != 0)
				vzm::RemoveSceneItem(aidGroup, true);
				aidGroup = RegisterCArmImage(sidScene, folder_trackingInfo + calibPosInfoFile, "calib_test" + to_string(i));
				if (aidGroup == -1)
					break;

				activeCarmIdx = i;

				break;
			}
		}
	}break;
	case WM_COMMAND:
	{
		int wmId = LOWORD(wParam);
		// 메뉴 선택을 구문 분석합니다:
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
	}
	break;
	case WM_SIZE:
	{
		if (cidCam1 != 0) {
			RECT rc;
			GetClientRect(hWnd, &rc);
			UINT width = rc.right - rc.left;
			UINT height = rc.bottom - rc.top;

			cpCam1.w = width;
			cpCam1.h = height;
			vzm::SetCameraParams(cidCam1, cpCam1);
		}
	}
	break;
	case WM_PAINT:
	{
		if (sidScene != 0)
		{
			if (USE_WHND) {

				PAINTSTRUCT ps;
				HDC hdc = BeginPaint(hWnd, &ps);
				//// TODO: 여기에 hdc를 사용하는 그리기 코드를 추가합니다...
				EndPaint(hWnd, &ps);

				//vzm::PresentHWND(hWnd); // this calls WM_PAINT again...
			}
			else {
				UpdateBMP(cidCam1, hWnd);
			} // no need to call invalidate
		}
	}
	break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	case WM_LBUTTONDOWN:
	case WM_RBUTTONDOWN:
	{
		int x = GET_X_LPARAM(lParam);
		int y = GET_Y_LPARAM(lParam);
		if (x == 0 && y == 0) break;

		// renderer does not need to be called
		glm::ivec2 pos_ss = glm::ivec2(x, y);
		general_move.Start((int*)&pos_ss, cpCam1, scene_stage_center, scene_stage_scale);
		break;
	}
	case WM_MOUSEMOVE:
	{
		int x = GET_X_LPARAM(lParam);
		int y = GET_Y_LPARAM(lParam);

		if ((wParam & MK_LBUTTON) || (wParam & MK_RBUTTON))
		{
			glm::ivec2 pos_ss = glm::ivec2(x, y);
			if (wParam & MK_LBUTTON)
				general_move.PanMove((int*)&pos_ss, cpCam1);
			else if (wParam & MK_RBUTTON)
				general_move.RotateMove((int*)&pos_ss, cpCam1);

			vzm::SetCameraParams(cidCam1, cpCam1);
			//Render();
			//vzm::RenderScene(sidScene, cidCam1);
			//UpdateWindow(hWnd);
			//InvalidateRect(hWnd, NULL, FALSE);
		}
		else {
			int aidPickedMK = 0;
			glm::fvec3 pos_picked;
			if (vzm::PickActor(aidPickedMK, __FP pos_picked, x, y, cidCam1))
			{
				std::string mkName;
				vzm::GetNameBySceneItemID(aidPickedMK, mkName);
				vzm::ActorParameters apMK;
				vzm::GetActorParams(aidPickedMK, apMK);
				glm::fmat4x4 matWorld = *(glm::fmat4x4*)apMK.GetWorldTransform();
				glm::fvec3 posOrigin(0, 0, 0);
				glm::fvec3 posMK = vzmutils::transformPos(posOrigin, matWorld);
				std::cout << mkName << "is picked : " << ", Pos : " << posMK.x << ", " << posMK.y << ", " << posMK.z << std::endl;

			}

		}
		break;
	}
	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
		break;
	case WM_MOUSEWHEEL:
	{
		int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);

		// by adjusting cam distance
		if (zDelta > 0)
			*(glm::fvec3*)cpCam1.pos += scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam1.view);
		else
			*(glm::fvec3*)cpCam1.pos -= scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam1.view);

		vzm::SetCameraParams(cidCam1, cpCam1);
	}break;
	case WM_ERASEBKGND:
		return TRUE; // tell Windows that we handled it. (but don't actually draw anything)
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}

	return 0;
}

LRESULT CALLBACK DiagProc1(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	int sidScene2 = vzmutils::GetSceneItemIdByName(g_sceneName2);
	int cidCam2 = vzmutils::GetSceneItemIdByName(g_camName2);
	float scene_stage_scale = 5.f;
	glm::fvec3 scene_stage_center = g_posCenterMKsRBS;// glm::fvec3();
	vzm::CameraParameters cpCam2;
	if (sidScene2 != 0 && cidCam2 != 0) {
		vzm::GetCameraParams(cidCam2, cpCam2);

		scene_stage_scale = glm::length(scene_stage_center - *(glm::fvec3*)cpCam2.pos) * 1.0f;
	}

	static vzmutils::GeneralMove general_move;
	static vzm::CameraParameters cpPrevCam;
	static std::set<int> excludedMKIndices;
	static bool mouseCamFlag = false;

	switch (message)
	{ 
	case WM_KEYDOWN:
	{
		switch (wParam) {
			case char('S') :
			{
				cv::FileStorage fs(folder_data + "SceneCamPose2.txt", cv::FileStorage::Mode::WRITE);
				cv::Mat ocvVec3(1, 3, CV_32FC1);
				memcpy(ocvVec3.ptr(), cpCam2.pos, sizeof(float) * 3);
				fs.write("POS", ocvVec3);
				memcpy(ocvVec3.ptr(), cpCam2.view, sizeof(float) * 3);
				fs.write("VIEW", ocvVec3);
				memcpy(ocvVec3.ptr(), cpCam2.up, sizeof(float) * 3);
				fs.write("UP", ocvVec3);
				fs.release();
			}break;
			case char('L') :
			{
				cv::FileStorage fs(folder_data + "SceneCamPose2.txt", cv::FileStorage::Mode::READ);
				if (fs.isOpened()) {
					cv::Mat ocvVec3;
					fs["POS"] >> ocvVec3;
					memcpy(cpCam2.pos, ocvVec3.ptr(), sizeof(float) * 3);
					fs["VIEW"] >> ocvVec3;
					memcpy(cpCam2.view, ocvVec3.ptr(), sizeof(float) * 3);
					fs["UP"] >> ocvVec3;
					memcpy(cpCam2.up, ocvVec3.ptr(), sizeof(float) * 3);
					fs.release();
				}
				vzm::SetCameraParams(cidCam2, cpCam2);
			}break;
			case char('C') :
			{
				// to do // calibration
				{
					cv::Mat cameraMatrix, distCoeffs;
					{
						cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
						fs["K"] >> cameraMatrix;
						fs["DistCoeffs"] >> distCoeffs;
						fs.release();
					}
					std::vector<cv::Point2f> pts2ds;
					std::vector<cv::Point3f> ptsRbMk3ds, ptsWsMk3ds;
					int numAllMKs = (int)g_pts2ds.size();
					for (int i = 0; i < numAllMKs; i++) {
						auto it = excludedMKIndices.find(i);
						if (it != excludedMKIndices.end()) continue;
						pts2ds.push_back(g_pts2ds[i]);
						ptsRbMk3ds.push_back(g_ptsRbMk3ds[i]);
						ptsWsMk3ds.push_back(g_ptsWsMk3ds[i]);
					}
					std::cout << "# Valid Point-Pairs : " << pts2ds.size() << ", excluding " << excludedMKIndices.size() << " points" << std::endl;
					cv::Mat rvec, tvec;
					ComputePose(pts2ds, ptsRbMk3ds, ptsWsMk3ds, cameraMatrix, distCoeffs, rvec, tvec,
						g_pts2ds, g_ptsRbMk3ds, g_ptsWsMk3ds);
				}
			} break;
			case char('Z') :
			{
				// to do // restore all points
				int numAllMKs = (int)g_pts2ds.size();
				for (int i = 0; i < 2; i++) {
					for (int j = 0; j < numAllMKs; j++) {
						int aidMarker = vzmutils::GetSceneItemIdByName((i == 0? "Rb" : "Ws") + std::to_string(j));
						vzm::ActorParameters apMarker;
						vzm::GetActorParams(aidMarker, apMarker);
						apMarker.is_visible = true;
						vzm::SetActorParams(aidMarker, apMarker);
					}
				}
				excludedMKIndices.clear();
			} break;
			case char('X') :
			{
				cv::Mat eset;
				{
					cv::FileStorage fs(folder_trackingInfo + "exclude_set.txt", cv::FileStorage::Mode::READ);
					fs["ExSET"] >> eset;
					fs.release();
				}

				excludedMKIndices.clear();
				// restore all points
				for (int i = 0; i < eset.cols; i++) {
					excludedMKIndices.insert(eset.at<int>(0, i));
				}

				int numAllMKs = (int)g_pts2ds.size();
				for (int i = 0; i < 2; i++) {
					for (int j = 0; j < numAllMKs; j++) {
						int aidMarker = vzmutils::GetSceneItemIdByName((i == 0 ? "Rb" : "Ws") + std::to_string(j));
						vzm::ActorParameters apMarker;
						vzm::GetActorParams(aidMarker, apMarker);
						apMarker.is_visible = excludedMKIndices.find(j) == excludedMKIndices.end();
						vzm::SetActorParams(aidMarker, apMarker);
					}
				}
			} break;
		}
	}break;
	break;
	case WM_SIZE:
	{
		if (cidCam2 != 0) {
			RECT rc;
			GetClientRect(hDlg, &rc);
			UINT width = rc.right - rc.left;
			UINT height = rc.bottom - rc.top;

			cpCam2.w = width;
			cpCam2.h = height;
			vzm::SetCameraParams(cidCam2, cpCam2);
		}
	}
	break;
	case WM_PAINT:
	{
		if (sidScene2 != 0)
		{
			if (USE_WHND) {

				PAINTSTRUCT ps;
				HDC hdc = BeginPaint(hDlg, &ps);
				//// TODO: 여기에 hdc를 사용하는 그리기 코드를 추가합니다...
				EndPaint(hDlg, &ps);

				//vzm::PresentHWND(hWnd); // this calls WM_PAINT again...
			}
			else {
				UpdateBMP(cidCam2, hDlg);
			} // no need to call invalidate
		}
	}
	break;
	case WM_LBUTTONDOWN:
	case WM_RBUTTONDOWN:
	{
		int x = GET_X_LPARAM(lParam);
		int y = GET_Y_LPARAM(lParam);
		if (x == 0 && y == 0) break;

		if (wParam & MK_CONTROL) {
			// exclude the picking mk...
			int aidPickedMK = 0;
			glm::fvec3 pos_picked;
			if (vzm::PickActor(aidPickedMK, __FP pos_picked, x, y, cidCam2))
			{
				std::string mkName;
				vzm::GetNameBySceneItemID(aidPickedMK, mkName);
				int idx = std::stoi(mkName.substr(2, mkName.size() - 1));
				excludedMKIndices.insert(idx);
				std::cout << "picking index : " << idx << std::endl;

				std::cout << "excluded index list : ";
				for (auto it : excludedMKIndices) {
					std::cout << it << ", ";
				}
				std::cout << std::endl;

				for (int i = 0; i < 2; i++) {
					int aidMarker = vzmutils::GetSceneItemIdByName((i == 0 ? "Rb" : "Ws") + std::to_string(idx));
					vzm::ActorParameters apMarker;
					vzm::GetActorParams(aidMarker, apMarker);
					apMarker.is_visible = false;
					vzm::SetActorParams(aidMarker, apMarker);
				}
			}
		}
		else {
			// renderer does not need to be called
			glm::ivec2 pos_ss = glm::ivec2(x, y);
			general_move.Start((int*)&pos_ss, cpCam2, scene_stage_center, scene_stage_scale);
			mouseCamFlag = true;
		}
		break;
	}
	case WM_MOUSEMOVE:
	{
		int x = GET_X_LPARAM(lParam);
		int y = GET_Y_LPARAM(lParam);

		if (((wParam & MK_LBUTTON) || (wParam & MK_RBUTTON)) && mouseCamFlag)
		{
			glm::ivec2 pos_ss = glm::ivec2(x, y);
			if (wParam & MK_LBUTTON)
				general_move.PanMove((int*)&pos_ss, cpCam2);
			else if (wParam & MK_RBUTTON)
				general_move.RotateMove((int*)&pos_ss, cpCam2);

			vzm::SetCameraParams(cidCam2, cpCam2);
		}
		break;
	}
	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
		mouseCamFlag = false;
		break;
	case WM_MOUSEWHEEL:
	{
		int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);

		// by adjusting cam distance
		if (zDelta > 0)
			*(glm::fvec3*)cpCam2.pos += scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam2.view);
		else
			*(glm::fvec3*)cpCam2.pos -= scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam2.view);

		vzm::SetCameraParams(cidCam2, cpCam2);
		//Render();
		//vzm::RenderScene(sidScene, cidCam1);

		//InvalidateRect(hWnd, NULL, FALSE);
		//UpdateWindow(hWnd);
	}break;
	//case WM_ERASEBKGND:
	//	return TRUE; // tell Windows that we handled it. (but don't actually draw anything)
	//default:
	//	return DefWindowProc(hDlg, message, wParam, lParam);
	}

	return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}
