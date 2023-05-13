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

#define DESIRED_SCREEN_W 1000
#define DESIRED_SCREEN_H 1000
#define USE_WHND true

#define MAX_LOADSTRING 100

// Global Variables:
HWND g_hWnd;
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

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

	cv::FileStorage fs("../data/SceneCamPose.txt", cv::FileStorage::Mode::READ);
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

void Render() {
	int sidScene = vzmutils::GetSceneItemIdByName("Scene2"); // Scene1
	int cidCam1 = vzmutils::GetSceneItemIdByName("CArm Camera"); // World Camera

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

void CalibrateCamPoseForCArmRB(const std::string& intrinsicsFile, const std::string& cArmRbName, const std::string& calibRbName, const int numCalibRbMKs)
{
	using namespace std;

	double arrayMatK[9] = {};
	double arrayDistCoeffs[5] = {};
	{
		cv::FileStorage fs(intrinsicsFile, cv::FileStorage::Mode::READ);
		cv::Mat _matK;
		fs["K"] >> _matK;
		memcpy(arrayMatK, _matK.ptr(), sizeof(double) * 9);
		cv::Mat _distCoeffs;
		fs["DistCoeffs"] >> _distCoeffs;
		memcpy(arrayDistCoeffs, _distCoeffs.ptr(), sizeof(double) * 5);
		fs.release();
	}
	int oidAxis = 0, oidMarker = 0;
	{
		vzm::GenerateAxisHelperObject(oidAxis, 0.15f);
		glm::fvec4 pos(0, 0, 0, 1.f);
		vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);
	}
	cv::Mat cameraMatrix(3, 3, CV_64FC1);
	memcpy(cameraMatrix.ptr(), arrayMatK, sizeof(double) * 9);
	cv::Mat distCoeffs(5, 1, CV_64FC1);
	memcpy(distCoeffs.ptr(), arrayDistCoeffs, sizeof(double) * 5);

	map<string, avr_trk> cArmCalibScans; // string ... tracking csv file
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_001.csv"] = avr_trk("../data/c-arm 2023-05-09/7802.png", "../data/c-arm 2023-05-09/7802");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_002.csv"] = avr_trk("../data/c-arm 2023-05-09/7803.png", "../data/c-arm 2023-05-09/7803");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_003.csv"] = avr_trk("../data/c-arm 2023-05-09/7804.png", "../data/c-arm 2023-05-09/7804");
	cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_004.csv"] = avr_trk("../data/c-arm 2023-05-09/7805.png", "../data/c-arm 2023-05-09/7805");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_005.csv"] = avr_trk("../data/c-arm 2023-05-09/7806.png", "../data/c-arm 2023-05-09/7806");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_006.csv"] = avr_trk("../data/c-arm 2023-05-09/7807.png", "../data/c-arm 2023-05-09/7807");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_008.csv"] = avr_trk("../data/c-arm 2023-05-09/7808.png", "../data/c-arm 2023-05-09/7808");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_009.csv"] = avr_trk("../data/c-arm 2023-05-09/7809.png", "../data/c-arm 2023-05-09/7809");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_010.csv"] = avr_trk("../data/c-arm 2023-05-09/7810.png", "../data/c-arm 2023-05-09/7810");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_011.csv"] = avr_trk("../data/c-arm 2023-05-09/7811.png", "../data/c-arm 2023-05-09/7811");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_012.csv"] = avr_trk("../data/c-arm 2023-05-09/7812.png", "../data/c-arm 2023-05-09/7812");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_013.csv"] = avr_trk("../data/c-arm 2023-05-09/7813.png", "../data/c-arm 2023-05-09/7813");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_014.csv"] = avr_trk("../data/c-arm 2023-05-09/7814.png", "../data/c-arm 2023-05-09/7814");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_015.csv"] = avr_trk("../data/c-arm 2023-05-09/7815.png", "../data/c-arm 2023-05-09/7815");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_016.csv"] = avr_trk("../data/c-arm 2023-05-09/7817.png", "../data/c-arm 2023-05-09/7817");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_017.csv"] = avr_trk("../data/c-arm 2023-05-09/7818.png", "../data/c-arm 2023-05-09/7818");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_018.csv"] = avr_trk("../data/c-arm 2023-05-09/7819.png", "../data/c-arm 2023-05-09/7819");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_019.csv"] = avr_trk("../data/c-arm 2023-05-09/7820.png", "../data/c-arm 2023-05-09/7820");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_020.csv"] = avr_trk("../data/c-arm 2023-05-09/7821.png", "../data/c-arm 2023-05-09/7821");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_021.csv"] = avr_trk("../data/c-arm 2023-05-09/7822.png", "../data/c-arm 2023-05-09/7822");
	//cArmCalibScans["../data/Session 2023-05-09/Take 2023-05-09 06.27.02 PM_023.csv"] = avr_trk("../data/c-arm 2023-05-09/7823.png", "../data/c-arm 2023-05-09/7823");

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

	vector<cv::Point2f> pts2ds(numCalibRbMKs* cArmCalibScans.size());
	vector<cv::Point3f> ptsWsMk3ds(numCalibRbMKs* cArmCalibScans.size());
	vector<cv::Point3f> ptsRbMk3ds(numCalibRbMKs* cArmCalibScans.size());
	int scanCount = 0;

	glm::ivec2 imageWH;

	for (auto& calibScan : cArmCalibScans) {
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

		// trackingFrames
		avr_trk& avrTrkInfo = calibScan.second;

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
		fs["my_data"] >> pos2Ds;
		fs.release();

		vector<cv::Point2f> pts2d_single(numCalibRbMKs);
		vector<cv::Point3f> ptsRb3d_single(numCalibRbMKs);
		vector<cv::Point3f> ptsWs3d_single(numCalibRbMKs);

		cv::Mat imgCArm = cv::imread(avrTrkInfo.xRayScanImg);
		{
			imageWH = glm::ivec2(imgCArm.cols, imgCArm.rows);
		}
		for (int i = 0; i < numCalibRbMKs; i++) {
			float x = mirrorHorizontal? imgCArm.cols - (float)pos2Ds.at<int>(i, 0) : (float)pos2Ds.at<int>(i, 0);
			float y = (float)pos2Ds.at<int>(i, 1);
			pts2ds[(scanCount)*numCalibRbMKs + i] = cv::Point2f((float)pos2Ds.at<int>(i, 0), (float)pos2Ds.at<int>(i, 1));
		}
		memcpy(&pts2d_single[0], &pts2ds[(scanCount)*numCalibRbMKs], sizeof(cv::Point2f) * numCalibRbMKs);

		for (int i = 0; i < numCalibRbMKs; i++) {
			glm::fvec3 p1 = vzmutils::transformPos(avrTrkInfo.rbMarkers_calibRb[i], avrTrkInfo.mat_ws2rb);
			glm::fvec3 p2 = vzmutils::transformPos(avrTrkInfo.wsMarkers_calibRb[i], avrTrkInfo.mat_ws2rb);
			memcpy(&ptsRbMk3ds[(scanCount)*numCalibRbMKs + i], &p1, sizeof(glm::fvec3));
			memcpy(&ptsWsMk3ds[(scanCount)*numCalibRbMKs + i], &p2, sizeof(glm::fvec3));
		}
		memcpy(&ptsRb3d_single[0], &ptsRbMk3ds[(scanCount)*numCalibRbMKs], sizeof(cv::Point3f) * numCalibRbMKs);
		memcpy(&ptsWs3d_single[0], &ptsWsMk3ds[(scanCount)*numCalibRbMKs], sizeof(cv::Point3f) * numCalibRbMKs);

		for (int i = 0; i < 2; i++) {
			cv::Mat rvec, tvec;
			vector<cv::Point3f>& pts3d_single = i == 0 ? ptsRb3d_single : ptsWs3d_single;
			cv::solvePnP(pts3d_single, pts2d_single, cameraMatrix, distCoeffs, rvec, tvec);

			{
				// Reproject the 3D points onto the image plane using the camera calibration
				std::vector<cv::Point2f> imagePointsReprojected;
				cv::projectPoints(pts3d_single, rvec, tvec, cameraMatrix, distCoeffs, imagePointsReprojected);

				// Compute the reprojection error
				double reprojectionError = cv::norm(pts2d_single, imagePointsReprojected, cv::NORM_L2) / pts2d_single.size();
				cout << (i == 0 ? "Rb" : "Ws") << " reprojectionError : " << reprojectionError << endl;
			}

			{
				cv::FileStorage fs("../data/Tracking 2023-05-09/rb2carm" + to_string(i) + ".txt", cv::FileStorage::Mode::WRITE);
				fs.write("rvec", rvec);
				fs.write("tvec", tvec);
				fs.release();
			}
		}



		//vector<cv::Point2i> points2Ds(numCalibRbMKs);
		//memcpy(&points2Ds[0], pos2Ds.ptr(), sizeof(cv::Point2f) * 7);
		//// pts2ds[scanCount]
		//std::map<int, cv::Point2f> pts2dMap; 
		//for(int i = 0; i < numCalibRbMKs; i++)
		//	pts2dMap[(1 + scanCount) * 100 + i] = cv::Point2f((float)pos2Ds.at<int>(i, 0), (float)pos2Ds.at<int>(i, 1));
		scanCount++;
	}

	for (int i = 0; i < 2; i++) {
		cv::Mat rvec, tvec;
		vector<cv::Point3f>& pts3d = i == 0 ? ptsRbMk3ds : ptsWsMk3ds;
		cv::solvePnP(pts3d, pts2ds, cameraMatrix, distCoeffs, rvec, tvec);
		cv::solvePnPRefineLM(pts3d, pts2ds, cameraMatrix, distCoeffs, rvec, tvec);
		cv::solvePnPRefineVVS(pts3d, pts2ds, cameraMatrix, distCoeffs, rvec, tvec);
		
		{
			// Reproject the 3D points onto the image plane using the camera calibration
			std::vector<cv::Point2f> imagePointsReprojected;
			cv::projectPoints(pts3d, rvec, tvec, cameraMatrix, distCoeffs, imagePointsReprojected);

			// Compute the reprojection error
			double reprojectionError = cv::norm(pts2ds, imagePointsReprojected, cv::NORM_L2) / pts2ds.size();
			cout << "************* " << (i == 0 ? "Rb" : "Ws") << " reprojectionError : " << reprojectionError << endl;
		}

		{
			cv::FileStorage fs("../data/Tracking 2023-05-09/rb2carm" + to_string(i) + ".txt", cv::FileStorage::Mode::WRITE);
			fs.write("rvec", rvec);
			fs.write("tvec", tvec);
			fs.release();
		}
	}

	// scene 2
	{
		int sidScene2 = 0;
		vzm::NewScene("Scene2", sidScene2);

		for (int i = 0; i < 2; i++) { // Rb and then Ws

			cv::Mat rvec, tvec;
			cv::FileStorage fs("../data/Tracking 2023-05-09/rb2carm" + to_string(i) + ".txt", cv::FileStorage::Mode::READ);
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

			glm::fmat4x4 matCA2RB = glm::inverse(matRB2CA);
			//glm::fmat4x4 matRB2WS;

			assert(pts2ds.size() == ptsRbMk3ds.size() && pts2ds.size() == ptsWsMk3ds.size());
			int numSamples = (int)pts2ds.size();
			for (int j = 0; j < numSamples; j++) {
				vzm::ActorParameters apMarker;
				apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
				apMarker.is_visible = true;
				apMarker.is_pickable = true;

				glm::fvec3 pos3d = *(glm::fvec3*)&(i == 0 ? ptsRbMk3ds[i] : ptsWsMk3ds[i]);
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.005f)); // set 1 cm to the marker diameter
				glm::fmat4x4 matLS2RB = glm::translate(pos3d);
				matLS2RB = matLS2RB * matScale;

				*(glm::fvec4*)apMarker.color = i == 0? glm::fvec4(1.f, 0, 0, 1.f) : glm::fvec4(0, 1.f, 0, 1.f); // rgba
				apMarker.SetLocalTransform(__FP matLS2RB);
				int aidMarker = 0;
				string actorMkName = (i == 0 ? "Rb" : "Ws") + to_string(j);
				vzm::NewActor(apMarker, actorMkName, aidMarker);
				vzm::AppendSceneItemToSceneTree(aidMarker, sidScene2);

				std::vector<glm::fvec3> pinfo(3);
				pinfo[0] = glm::fvec3(0, 1, 0);
				pinfo[1] = glm::fvec3(0, 0, -1);
				pinfo[2] = glm::fvec3(0, -1, 0);
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
			}

			if (i == 0) {
				int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
				int cidCam1 = vzmutils::GetSceneItemIdByName("World Camera");

				vzm::CameraParameters cpCam1;
				vzm::GetCameraParams(cidCam1, cpCam1);
				
				vzm::CameraParameters cpCam2 = cpCam1;

				RECT rc;
				GetClientRect(g_hWnd, &rc);
				UINT widthWindow = rc.right - rc.left;
				UINT heightWindow = rc.bottom - rc.top;

				const float intrinsicRatioX = (float)imageWH.x / widthWindow;
				const float intrinsicRatioY = (float)imageWH.y / heightWindow;

				cpCam2.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_INTRINSICS;
				glm::fvec3 pos(0, 0, 0);
				glm::fvec3 view(0, 0, 1);
				glm::fvec3 up(0, -1, 0);
				pos = *(glm::fvec3*)cpCam2.pos = vzmutils::transformPos(pos, matCA2RB);
				view = *(glm::fvec3*)cpCam2.view = vzmutils::transformVec(view, matCA2RB);
				up = *(glm::fvec3*)cpCam2.up = vzmutils::transformVec(up, matCA2RB);
				cpCam2.fx = arrayMatK[0] / intrinsicRatioX;
				cpCam2.fy = arrayMatK[4] / intrinsicRatioY;
				cpCam2.sc = arrayMatK[1];
				cpCam2.cx = arrayMatK[2] / intrinsicRatioX;
				cpCam2.cy = arrayMatK[5] / intrinsicRatioY;
				cpCam2.np = 0.2;
				int cidCam2 = 0;
				vzm::NewCamera(cpCam2, "CArm Camera", cidCam2);

				vzm::LightParameters lpLight2;
				lpLight2.is_on_camera = true;
				lpLight2.is_pointlight = false;
				*(glm::fvec3*)lpLight2.pos = *(glm::fvec3*)cpCam1.pos;
				*(glm::fvec3*)lpLight2.dir = *(glm::fvec3*)cpCam1.view;
				*(glm::fvec3*)lpLight2.up = *(glm::fvec3*)cpCam1.up;
				int lidLight2 = 0;
				vzm::NewLight(lpLight2, "World Light 2", lidLight2);

				vzm::AppendSceneItemToSceneTree(cidCam2, sidScene2);
				vzm::AppendSceneItemToSceneTree(lidLight2, sidScene2);
			}
		}
	}


	/*
	vector<cv::Point3f> pts3Ds(ptsRBS.size());
	vector<cv::Point2f> pts2Ds(pts2Dmap.size());
	memcpy(&pts3Ds[0], &ptsRBS[0], sizeof(fvec3)* numCalPts* numCalScans);
	int i = 0;
	for (auto it = pts2Dmap.begin(); it != pts2Dmap.end(); it++)
		pts2Ds[i++] = it->second;

	cv::Mat cameraMatrix(3, 3, CV_64FC1);
	memcpy(cameraMatrix.ptr(), arrayMatK, sizeof(double) * 9);
	cv::Mat distCoeffs(5, 1, CV_64FC1);
	memcpy(distCoeffs.ptr(), arrayDistCoeffs, sizeof(double) * 5);
	cv::solvePnP(pts3Ds, pts2Ds, cameraMatrix, distCoeffs, rvec, tvec);

	cv::FileStorage fs("../data/Tracking 2023-04-19/rb2carm.txt", cv::FileStorage::Mode::WRITE);
	fs.write("rvec", rvec);
	fs.write("tvec", tvec);
	fs.release();
	/**/
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

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
	CalibrateCamPoseForCArmRB("../data/Tracking 2023-05-09/carm_intrinsics.txt", "c-arm", "test1", 7);

	SetTimer(g_hWnd, NULL, 10, TimerProc);

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_SCALIBRATORWIN));
    MSG msg;

    // Main message loop:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
	}

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

   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   g_hWnd = hWnd;

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

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
	static std::string sceneName = "Scene1";
	static std::string camName = "World Camera";
	int sidScene = vzmutils::GetSceneItemIdByName("Scene2");
	int cidCam1 = vzmutils::GetSceneItemIdByName("CArm Camera");
	float scene_stage_scale = 5.f;
#ifdef USE_MOTIVE
	static int activeCarmIdx = -1;
#else
	static int activeCarmIdx = 0;
#endif
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
				cv::FileStorage fs("../data/SceneCamPose.txt", cv::FileStorage::Mode::WRITE);
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
					cv::FileStorage fs("../data/SceneCamPose.txt", cv::FileStorage::Mode::READ);
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
						double arrayMatK[9] = { 4.90314332e+03, 0.00000000e+00, 5.66976763e+02,
							0.00000000e+00, 4.89766748e+03, 6.45099412e+02,
							0.00000000e+00, 0.00000000e+00, 1.00000000e+00
						};
						double arrayDistCoeffs[5] = { -4.41147907e-02,  1.01898819e+00,  6.79242077e-04,
							-1.16990433e-02,  3.42748576e-01
						};

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
						cpNewCam1.fx = arrayMatK[0] / intrinsicRatioX;
						cpNewCam1.fy = arrayMatK[4] / intrinsicRatioY;
						cpNewCam1.sc = arrayMatK[1];
						cpNewCam1.cx = arrayMatK[2] / intrinsicRatioX;
						cpNewCam1.cy = arrayMatK[5] / intrinsicRatioY;
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

		auto StoreParams = [](const std::string& paramsFileName, const glm::fmat4x4& matRB2WS, const std::string& imgFileName) {

			cv::FileStorage __fs("../data/Tracking Test 2023-04-30/" + paramsFileName, cv::FileStorage::Mode::WRITE);

			double arrayMatK[9] = {};
			double arrayDistCoeffs[5] = {};
			cv::FileStorage fs("../data/Tracking 2023-04-19/carm_intrinsics.txt", cv::FileStorage::Mode::READ);
			cv::Mat _matK;
			fs["K"] >> _matK;
			__fs << "K" << _matK;
			memcpy(arrayMatK, _matK.ptr(), sizeof(double) * 9);
			cv::Mat _distCoeffs;
			fs["DistCoeffs"] >> _distCoeffs;
			__fs << "DistCoeffs" << _distCoeffs;
			memcpy(arrayDistCoeffs, _distCoeffs.ptr(), sizeof(double) * 5);
			fs.open("../data/Tracking 2023-04-19/rb2carm.txt", cv::FileStorage::Mode::READ);
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

		static map<int, int> mapAidGroupCArmCam;
		static char LOADKEYS[10] = { '1' , '2', '3', '4', '5', '6', '7', '8', '9', '0' };
		for (int i = 0; i < 10; i++) {
			if (wParam == LOADKEYS[i]) {
				// load case
				auto it = mapAidGroupCArmCam.find(i + 1);
				if (it != mapAidGroupCArmCam.end())
					vzm::RemoveSceneItem(it->second);
				int aidGroup = RegisterCArmImage(sidScene, string("../data/Tracking Test 2023-04-30/") + "test" + to_string(i) + ".txt", "test" + to_string(i));
				if (aidGroup == -1)
					break;
				mapAidGroupCArmCam[i + 1] = aidGroup;
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
		//Render();
		//vzm::RenderScene(sidScene, cidCam1);

		//InvalidateRect(hWnd, NULL, FALSE);
		//UpdateWindow(hWnd);
	}
	case WM_ERASEBKGND:
		return TRUE; // tell Windows that we handled it. (but don't actually draw anything)
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
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
