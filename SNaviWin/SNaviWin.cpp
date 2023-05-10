// SNaviWin.cpp : 애플리케이션에 대한 진입점을 정의합니다.
//

#include "framework.h"
#include "SNaviWin.h"

#include <regex>
#include <vector>
#include <windowsx.h>
#include <iostream>
#include <winsock2.h> // udp
#include <stdio.h>

#pragma comment(lib,"ws2_32.lib")

#include <objidl.h>
#include <gdiplus.h>
#include <gdipluspath.h>
#include <gdiplusgraphics.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")


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
#include "naviHelpers.hpp"
#include "rapidcsv/rapidcsv.h"
#include "CArmCalibration.h"

//#define USE_MOTIVE
#define DESIRED_SCREEN_W 1000
#define DESIRED_SCREEN_H 1000
#define USE_WHND true
bool storeAvrCarmTrackinfo = false;
std::map<std::string, glm::fmat4x4> rbMapTrAvr;
std::map<std::string, glm::fvec3> mkPosTest1Avr;

#define MAX_LOADSTRING 100
typedef unsigned long long u64;

static int testFileIdx = 1000;

// 전역 변수:
HWND g_hWnd;
HINSTANCE hInst;                                // 현재 인스턴스입니다.
WCHAR szTitle[MAX_LOADSTRING];                  // 제목 표시줄 텍스트입니다.
WCHAR szWindowClass[MAX_LOADSTRING];            // 기본 창 클래스 이름입니다.

// 이 코드 모듈에 포함된 함수의 선언을 전달합니다:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int main()
{
	return wWinMain(GetModuleHandle(NULL), NULL, GetCommandLine(), SW_SHOWNORMAL);
}

void GraphicsPostDrawing(const HDC hdc, const int cidCam)
{
	//std::vector<glm::fvec2> listPosSSs;
	//vzmutils::ComputeScreenPointsOnPanoSlicer(cidCam, listPosSSs);
	Graphics graphics(hdc);
	graphics.SetSmoothingMode(SmoothingModeAntiAlias);

	GraphicsPath path(FillModeAlternate);
	path.StartFigure();
	//path.AddLine(posSS_TL.x, posSS_TL.y, posSS_TR.x, posSS_TR.y);
	path.AddLine(0, 0, 200, 200);
	path.StartFigure();
	//path.AddLine(posSS_TR.x, posSS_TR.y, posSS_BR.x, posSS_BR.y);
	path.AddLine(200, 200, 200, 800);

	Pen pen(Color(255, 255, 255, 0), 2);
	graphics.DrawPath(&pen, &path);
}

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

void UpdateTrackInfo2Scene(navihelpers::track_info& trackInfo) {

	int numRBs = trackInfo.NumRigidBodies();
	int numMKs = trackInfo.NumMarkers();

	for (int i = 0; i < numRBs; i++) {
		std::string rbName;
		glm::fmat4x4 matLS2WS;
		if (trackInfo.GetRigidBodyByIdx(i, &rbName, &matLS2WS, NULL, NULL)) {
			int aidAxis = vzmutils::GetSceneItemIdByName(rbName);
			vzm::ActorParameters apAxis;
			vzm::GetActorParams(aidAxis, apAxis);
			apAxis.is_visible = true;

			if (storeAvrCarmTrackinfo)
				matLS2WS = rbMapTrAvr[rbName];

			apAxis.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidAxis, apAxis);
		}
	}

	for (int i = 0; i < numMKs; i++) {
		std::map<navihelpers::track_info::MKINFO, std::any> mk;
		if (trackInfo.GetMarkerByIdx(i, mk)) {
			std::string mkName = std::any_cast<std::string>(mk[navihelpers::track_info::MKINFO::MK_NAME]);
			glm::fvec3 pos = std::any_cast<glm::fvec3>(mk[navihelpers::track_info::MKINFO::POSITION]);

			if (storeAvrCarmTrackinfo)
			{
				auto mkp = mkPosTest1Avr.find(mkName);
				if (mkp != mkPosTest1Avr.end())
					pos = mkPosTest1Avr[mkName];
			}

			glm::fmat4x4 matLS2WS = glm::translate(pos);
			glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.005f)); // set 1 cm to the marker diameter
			matLS2WS = matLS2WS * matScale;
			int aidMarker = vzmutils::GetSceneItemIdByName(mkName);
			vzm::ActorParameters apMarker;
			vzm::GetActorParams(aidMarker, apMarker);
			apMarker.is_visible = true;
			apMarker.is_pickable = true;
			apMarker.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidMarker, apMarker);
		}
	}
}

int numAnimationCount = 50;
int arAnimationKeyFrame = -1;
std::vector<vzm::CameraParameters> cpInterCams(numAnimationCount);

void Render() {
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	int cidCam1 = vzmutils::GetSceneItemIdByName("World Camera");

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
		//Sleep(1);
	}
}

#ifdef USE_MOTIVE
navihelpers::concurrent_queue<navihelpers::track_info>* g_track_que = NULL;
#endif
void CALLBACK TimerProc(HWND, UINT, UINT_PTR pcsvData, DWORD)
{
	static int frameCount = 0;
#ifdef USE_MOTIVE
	int frame = frameCount++;
#else
	std::vector<navihelpers::track_info>& trackingFrames = *(std::vector<navihelpers::track_info>*)pcsvData;
	int totalFrames = (int)trackingFrames.size();
	int frame = (frameCount++) % totalFrames;
#endif
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


#ifdef USE_MOTIVE
	navihelpers::concurrent_queue<navihelpers::track_info>* track_que = 
		(navihelpers::concurrent_queue<navihelpers::track_info>*)pcsvData;
	navihelpers::track_info trackInfo;
	track_que->wait_and_pop(trackInfo);

	// generate scene actors with updated trackInfo
	{
		using namespace std;
		using namespace navihelpers;
		using namespace glm;

		int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
		int numMKs = trackInfo.NumMarkers();
		int numRBs = trackInfo.NumRigidBodies();

		// Tracking Cam Geometry and Actors
		{
			static int aidGroupCams = 0;
			if (aidGroupCams == 0) {
				vzm::ActorParameters apGroupCams;
				vzm::NewActor(apGroupCams, "Tracking CAM Group", aidGroupCams);
				vzm::AppendSceneItemToSceneTree(aidGroupCams, sidScene);

				static int oidCamModels[3] = { 0, 0, 0 };
				for (int i = 0; i < 3; i++) {
					fmat4x4 matCam2WS;
					optitrk::GetCameraLocation(i, __FP matCam2WS);
					int oidCamTris = 0, oidCamLines = 0, oidCamLabel = 0;
					Cam_Gen(matCam2WS, "CAM" + to_string(i), oidCamTris, oidCamLines, oidCamLabel);

					vzm::ActorParameters apCamTris, apCamLines, apCamLabel;
					apCamTris.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamTris);
					apCamTris.color[3] = 0.5f;
					apCamLines.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLines);
					*(fvec4*)apCamLines.phong_coeffs = fvec4(0, 1, 0, 0);
					apCamLines.line_thickness = 2;
					apCamLabel.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLabel);
					*(fvec4*)apCamLabel.phong_coeffs = fvec4(0, 1, 0, 0);
					int aidCamTris = 0, aidCamLines = 0, aidCamLabel = 0;
					vzm::NewActor(apCamTris, "Cam" + to_string(i) + " Tris", aidCamTris);
					vzm::NewActor(apCamLines, "Cam" + to_string(i) + " Lines", aidCamLines);
					vzm::NewActor(apCamLabel, "Cam" + to_string(i) + " Label", aidCamLabel);
					vzm::AppendSceneItemToSceneTree(aidCamTris, aidGroupCams);
					vzm::AppendSceneItemToSceneTree(aidCamLines, aidGroupCams);
					vzm::AppendSceneItemToSceneTree(aidCamLabel, aidGroupCams);
				}
			}
			//
		}

		static int oidAxis = 0;
		static int oidMarker = 0;
		if (oidAxis == 0) {
			vzm::GenerateAxisHelperObject(oidAxis, 0.15f);
		}
		if (oidMarker == 0) {
			glm::fvec4 pos(0, 0, 0, 1.f);
			vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);
		}

		static map<string, int> sceneActors;

		for (int i = 0; i < numRBs; i++) {
			std::string rbName;
			if (trackInfo.GetRigidBodyByIdx(i, &rbName, NULL, NULL, NULL)) {
				auto actorId = sceneActors.find(rbName);
				if (actorId == sceneActors.end()) {
					vzm::ActorParameters apAxis;
					apAxis.SetResourceID(vzm::ActorParameters::GEOMETRY, oidAxis);
					apAxis.is_visible = false;
					apAxis.line_thickness = 3;
					int aidRbAxis = 0;
					vzm::NewActor(apAxis, rbName, aidRbAxis);
					vzm::AppendSceneItemToSceneTree(aidRbAxis, sidScene);

					std::vector<glm::fvec3> pinfo(3);
					pinfo[0] = glm::fvec3(0, 0, 0);
					pinfo[1] = glm::fvec3(-1, 0, 0);
					pinfo[2] = glm::fvec3(0, -1, 0);
					int oidLabelText = 0;
					vzm::GenerateTextObject((float*)&pinfo[0], rbName, 0.07, true, false, oidLabelText);
					vzm::ActorParameters apLabelText;
					apLabelText.SetResourceID(vzm::ActorParameters::GEOMETRY, oidLabelText);
					*(glm::fvec4*)apLabelText.phong_coeffs = glm::fvec4(0, 1, 0, 0);
					int aidLabelText = 0;
					vzm::NewActor(apLabelText, rbName + ":Label", aidLabelText);
					vzm::AppendSceneItemToSceneTree(aidLabelText, aidRbAxis);

					sceneActors[rbName] = 2;
				}
			}
		}

		for (int i = 0; i < numMKs; i++) {
			std::map<navihelpers::track_info::MKINFO, std::any> mk;
			if (trackInfo.GetMarkerByIdx(i, mk)) {
				std::string mkName = std::any_cast<std::string>(mk[navihelpers::track_info::MKINFO::MK_NAME]);
				auto actorId = sceneActors.find(mkName);
				if (actorId == sceneActors.end()) {
					vzm::ActorParameters apMarker;
					apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
					apMarker.is_visible = false;
					*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 1.f, 1.f, 1.f); // rgba
					int aidMarker = 0;
					vzm::NewActor(apMarker, mkName, aidMarker);
					vzm::AppendSceneItemToSceneTree(aidMarker, sidScene);

					sceneActors[mkName] = 1;
				}
			}
		}
	}

	UpdateTrackInfo2Scene(trackInfo);
#else
	UpdateTrackInfo2Scene(trackingFrames[frame]);
#endif
	Render();
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	// engine initialization to use the core APIs of framework
	// must be paired with DeinitEngineLib()
	vzm::InitEngineLib("SpineNavi");
	vzm::SetLogConfiguration(true, 4);

#ifdef USE_MOTIVE
    bool optitrkMode = optitrk::InitOptiTrackLib();
#else
	//rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.12.57 PM.csv", rapidcsv::LabelParams(0, 0)); // 7081.png
	//std::string cArmTrackFile = "../data/Tracking 2023-04-19/c-arm-track1.txt";
	//rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.13.30 PM.csv", rapidcsv::LabelParams(0, 0));  // 7082.png
	//std::string cArmTrackFile = "../data/Tracking 2023-04-19/c-arm-track2.txt";
	//rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.14.05 PM.csv", rapidcsv::LabelParams(0, 0));  // 7083.png
	//std::string cArmTrackFile = "../data/Tracking 2023-04-19/c-arm-track3.txt";
	rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.14.56 PM.csv", rapidcsv::LabelParams(0, 0));  // 7084.png
	std::string cArmTrackFile = "../data/Tracking 2023-04-19/c-arm-track4.txt";

	std::string imgFile = "../data/c-arm 2023-04-19/7084.png";
	
	//rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.18.13 PM.csv", rapidcsv::LabelParams(0, 0)); // 7085.png
	//rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.19.59 PM.csv", rapidcsv::LabelParams(0, 0));
#endif

    // 전역 문자열을 초기화합니다.
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_SNAVIWIN, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // 애플리케이션 초기화를 수행합니다:
    if (!InitInstance (hInstance, nCmdShow))
    {
        return 0;
    }

	SceneInit();

#ifdef USE_MOTIVE
	//const int postpone = 3;
	std::vector<std::string> rbNames;
	int numAllFramesRBs = optitrk::GetRigidBodies(&rbNames);

	static navihelpers::concurrent_queue<navihelpers::track_info> track_que(10);
	g_track_que = &track_que;
    std::atomic_bool tracker_alive{true};
	std::thread tracker_processing_thread([&]() {
		using namespace std;
		using namespace navihelpers;
		using namespace glm;
		while (tracker_alive)
		{
			//Sleep(postpone);
			optitrk::UpdateFrame();

			track_info trk_info;
			for (int i = 0; i < numAllFramesRBs; i++)
			{
				string rbName;// = rbNames[i];
				fmat4x4 matLS2WS;
				float rbMSE;
				vector<float> posMKs;
				vector<float> mkQualities;
				bitset<128> rbCid;
				fquat qvec;
				fvec3 tvec;
				if (optitrk::GetRigidBodyLocationByIdx(i, (float*)&matLS2WS, &rbCid, &rbMSE, &posMKs, NULL, &mkQualities, &rbName, (float*)&qvec, (float*)&tvec)) {
					int numRbMks = (int)posMKs.size() / 3;
					vector<fvec3> mkPts(numRbMks);
					memcpy(&mkPts[0], &posMKs[0], sizeof(fvec3) * numRbMks);
					map<string, map<track_info::MKINFO, std::any>> rbmkSet;
					for (int j = 0; j < numRbMks; j++) {
						//CID = 0, // std::bitset<128>
						//POSITION = 1, // glm::fvec3, world space
						//MK_NAME = 2, // string
						//MK_QUALITY = 3 // float // only for RB_MKSET
						string mkName = rbName + ":Marker" + to_string(j + 1);
						auto& pt = rbmkSet[mkName];
						pt[track_info::MKINFO::CID] = rbCid;
						pt[track_info::MKINFO::POSITION] = mkPts[j];
						pt[track_info::MKINFO::MK_NAME] = mkName;
						pt[track_info::MKINFO::MK_QUALITY] = mkQualities[j];
					}
					trk_info.AddRigidBody(rbName, matLS2WS, qvec, tvec, rbMSE, rbmkSet);
				}
			}
			vector<float> mkPts;
			vector<float> mkResiduals;
			vector<bitset<128>> mkCIDs;
			optitrk::GetMarkersLocation(&mkPts, &mkResiduals, &mkCIDs);
			int numMKs = (int)mkCIDs.size();
			for (int i = 0; i < numMKs; i++) {
				fvec3 pos = fvec3(mkPts[3 * i + 0], mkPts[3 * i + 1], mkPts[3 * i + 2]);
				string mkName = "Marker" + to_string(i + 1);
				trk_info.AddMarker(mkCIDs[i], pos, mkName);
			}
			track_que.push(trk_info);
		}
	});


	/////////////////////////////////////////
#define BUF_SIZE 5000

	auto printError = [](std::string message) {
		std::cout << message << std::endl;
		fputc('\n', stderr);
		exit(1);
	};
	auto GetMicroCounter = []()
	{
		u64 Counter;

#if defined(_WIN32)
		u64 Frequency;
		QueryPerformanceFrequency((LARGE_INTEGER*)&Frequency);
		QueryPerformanceCounter((LARGE_INTEGER*)&Counter);
		Counter = 1000000 * Counter / Frequency;
#elif defined(__linux__) 
		struct timeval t;
		gettimeofday(&t, 0);
		Counter = 1000000 * t.tv_sec + t.tv_usec;
#endif

		return Counter;
	};
	/*
	u64 start, end;
	WSADATA wsaData;
	struct sockaddr_in local_addr;
	SOCKET s;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	if ((s = socket(PF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("Socket Creat Error.\n");
		exit(1);
	}

	memset(&local_addr, 0, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	local_addr.sin_port = htons(atoi("22222"));

	if (bind(s, (SOCKADDR*)&local_addr, sizeof(local_addr)) == SOCKET_ERROR)
		printError("BIND ERROR");

	printf("This server is waiting... \n");

	struct sockaddr_in client_addr;
	int len_addr = sizeof(client_addr);
	int totalBufferNum;
	int BufferNum;
	int readBytes;
	long file_size;
	long totalReadBytes;

	char buf[BUF_SIZE];

	start = GetMicroCounter();

	readBytes = recvfrom(s, buf, BUF_SIZE, 0, (struct sockaddr*)&client_addr, &len_addr);
	file_size = atol(buf);
	totalBufferNum = file_size / BUF_SIZE + 1;
	BufferNum = 0;
	totalReadBytes = 0;
	/**/
	std::atomic_bool udp_alive{ true };
	std::thread udp_processing_thread([&]() {
		/*
		FILE* fp;
		fopen_s(&fp, "../data/Tracking Test 2023-05-09/test.png", "wb");

		while (BufferNum != totalBufferNum) {
			readBytes = recvfrom(s, buf, BUF_SIZE, 0, (struct sockaddr*)&client_addr, &len_addr);
			BufferNum++;
			totalReadBytes += readBytes;
			printf("In progress: %d/%dByte(s) [%d%%]\n", totalReadBytes, file_size, ((BufferNum * 100) / totalBufferNum));
			if (readBytes > 0) {
				fwrite(buf, sizeof(char), readBytes, fp);
				readBytes = sendto(s, buf, 10, 0, (SOCKADDR*)&client_addr, sizeof(client_addr));
			}
			if (readBytes == SOCKET_ERROR)
			{
				printError("ERROR");
				break;
			}
		}
		fclose(fp);
		/**/
	});

	UINT_PTR customData = (UINT_PTR)&track_que;
#else
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

	struct rb_tr_smooth
	{
		int fr_count;
		std::vector<glm::fquat> qs;
		std::vector<glm::fvec3> ts;
	};

	std::map<std::string, rb_tr_smooth> rbMapTRs;
	std::map<std::string, std::vector<glm::fvec3>> test1MkMapPos;

	int numAllFramesRBs = 0;
	int numAllFramesMKs = 0;
	for (int rowIdx = startRowIdx; rowIdx < numRows; rowIdx++) {
		std::vector<std::string> rowStrValues = trackingData.GetRow<std::string>(rowIdx);
		if (rowStrValues.size() == 0)
			continue;
		navihelpers::track_info& trackInfo = trackingFrames[frameIdx++];

		int colIdx = startColIdx;
		while (colIdx < numCols) {
			std::string type = rowTypes[colIdx];
			std::string name = rowNames[colIdx];
			std::string id = rowIds[colIdx];
			std::bitset<128> cid = string2cid(id);
			std::string rotPos = rotPosLabels[colIdx];
			std::string xyzw = xyzwLabels[colIdx];

			std::map<std::string, std::map<navihelpers::track_info::MKINFO, std::any>> rbmkSet;

			if (type == "Rigid Body") {
				if(rowIdx == startRowIdx) numAllFramesRBs++;
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
				
				// read 8 col-successive cells
				glm::fquat q(values[3], values[0], values[1], values[2]);
				glm::fvec3 t(values[4], values[5], values[6]);
				float mkMSE = values[7];

				glm::fmat4x4 mat_r = glm::toMat4(q);
				glm::fmat4x4 mat_t = glm::translate(t);
				glm::fmat4x4 mat_ls2ws = mat_t * mat_r;

				rb_tr_smooth& rbTrSmooth = rbMapTRs[name];
				rbTrSmooth.qs.push_back(q);
				rbTrSmooth.ts.push_back(t);

				colIdx += 8;

				for (; colIdx < numCols; colIdx+=4) {
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

				if (name.find("test1:") != std::string::npos) {
					std::vector<glm::fvec3>& posMKs = test1MkMapPos[name];
					posMKs.push_back(p);
				}
			}
		}
	}
	
	// compute average of rigid body transforms
	//std::map<std::string, glm::fmat4x4> rbMapTrAvr;
	//std::map<std::string, glm::fvec3> mkPosTest1Avr;
	if (storeAvrCarmTrackinfo)
	{
		cv::FileStorage fs(cArmTrackFile, cv::FileStorage::Mode::WRITE);
		for (auto& v : rbMapTRs) {
			int numTRs = (int)v.second.qs.size();
			float normalizedNum = 1.f / (float)numTRs;

			glm::fvec3 rotAxisAvr = glm::fvec3(0, 0, 0);
			float rotAngleAvr = 0;
			glm::fvec3 trAvr = glm::fvec3(0, 0, 0);
			for (int i = 0; i < numTRs; i++) {
				glm::fquat q = v.second.qs[i];
				float quatMagnitude = glm::length(q);
				q /= quatMagnitude;

				float rotationAngle = 2.0f * acos(q.w);

				float sinAngle = sin(rotationAngle / 2.0f);
				glm::fvec3 rotationAxis = glm::fvec3(q.x, q.y, q.z) / sinAngle;

				rotAngleAvr += rotationAngle * normalizedNum;
				rotAxisAvr += rotationAxis * normalizedNum;

				glm::fvec3 t = v.second.ts[i];
				trAvr += t * normalizedNum;
			}
			rotAxisAvr = glm::normalize(rotAxisAvr);

			glm::fmat4x4 mat_r = glm::rotate(rotAngleAvr, rotAxisAvr);
			glm::fmat4x4 mat_t = glm::translate(trAvr);
			glm::fmat4x4 matLS2WSavr = mat_t * mat_r;
			rbMapTrAvr[v.first] = matLS2WSavr;

			cv::Mat ocvRbMapTrAvr(4, 4, CV_32FC1);
			memcpy(ocvRbMapTrAvr.ptr(), glm::value_ptr(matLS2WSavr), sizeof(float) * 16);
			fs.write(v.first, ocvRbMapTrAvr);
		}

		for (auto& pts : test1MkMapPos) {
			std::vector<glm::fvec3>& posMK = pts.second;
			int numTRs = (int)posMK.size();
			float normalizedNum = 1.f / (float)numTRs;

			glm::fvec3 trAvr = glm::fvec3(0, 0, 0);
			for (int i = 0; i < numTRs; i++)
				trAvr += posMK[i] * normalizedNum;
			mkPosTest1Avr[pts.first] = trAvr;

			static cv::Mat ocvPt(1, 3, CV_32FC1);
			memcpy(ocvPt.ptr(), &trAvr, sizeof(float) * 3);
			fs.write(std::regex_replace(pts.first, std::regex(":"), " "), ocvPt);
		}
		fs.release();
		// TO DO with rbMapTrAvr and mkPosTest1Avr for calibration task
	}

	// generate scene actors 
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	if(sidScene != 0)
	{
		navihelpers::track_info& trackInfo = trackingFrames[0];

		int oidAxis = 0;
		vzm::GenerateAxisHelperObject(oidAxis, 0.15f);
		int oidMarker = 0;
		glm::fvec4 pos(0, 0, 0, 1.f);
		vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);

		for (int i = 0; i < numAllFramesRBs; i++) {
			std::string rbName;
			if (trackInfo.GetRigidBodyByIdx(i, &rbName, NULL, NULL, NULL)) {
				vzm::ActorParameters apAxis;
				apAxis.SetResourceID(vzm::ActorParameters::GEOMETRY, oidAxis);
				apAxis.is_visible = false;
				apAxis.line_thickness = 3;
				int aidRbAxis = 0;
				vzm::NewActor(apAxis, rbName, aidRbAxis);
				vzm::AppendSceneItemToSceneTree(aidRbAxis, sidScene);

				std::vector<glm::fvec3> pinfo(3);
				pinfo[0] = glm::fvec3(0, 0, 0);
				pinfo[1] = glm::fvec3(-1, 0, 0);
				pinfo[2] = glm::fvec3(0, -1, 0);
				int oidLabelText = 0;
				vzm::GenerateTextObject((float*)&pinfo[0], rbName, 0.07, true, false, oidLabelText);
				vzm::ActorParameters apLabelText;
				apLabelText.SetResourceID(vzm::ActorParameters::GEOMETRY, oidLabelText);
				*(glm::fvec4*)apLabelText.phong_coeffs = glm::fvec4(0, 1, 0, 0);
				int aidLabelText = 0;
				vzm::NewActor(apLabelText, rbName + ":Label", aidLabelText);
				vzm::AppendSceneItemToSceneTree(aidLabelText, aidRbAxis);
			}
		}

		for (int i = 0; i < numAllFramesMKs; i++) {
			std::map<navihelpers::track_info::MKINFO, std::any> mk;
			if (trackInfo.GetMarkerByIdx(i, mk)) {
				vzm::ActorParameters apMarker;
				apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
				apMarker.is_visible = false;
				*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 1.f, 1.f, 1.f); // rgba
				int aidMarker = 0;
				std::string mkName = std::any_cast<std::string>(mk[navihelpers::track_info::MKINFO::MK_NAME]);
				vzm::NewActor(apMarker, mkName, aidMarker);
				vzm::AppendSceneItemToSceneTree(aidMarker, sidScene);

				if (mkName.find("test1") != std::string::npos) {
					std::vector<glm::fvec3> pinfo(3);
					pinfo[0] = glm::fvec3(0, 1, 0);
					pinfo[1] = glm::fvec3(0, 0, -1);
					pinfo[2] = glm::fvec3(0, -1, 0);
					int oidLabelText = 0;
					vzm::GenerateTextObject((float*)&pinfo[0], mkName.substr(6, mkName.length()-1), 2, true, false, oidLabelText, true);
					vzm::ActorParameters apLabelText;
					//apLabelText.is_visible = true;
					apLabelText.SetResourceID(vzm::ActorParameters::GEOMETRY, oidLabelText);
					//apLabelText.script_params.SetParam("_bool_IsScaleFree", true);
					*(glm::fvec4*)apLabelText.phong_coeffs = glm::fvec4(0, 1, 0, 0);
					int aidLabelText = 0;
					vzm::NewActor(apLabelText, mkName + ":Label", aidLabelText);
					vzm::AppendSceneItemToSceneTree(aidLabelText, aidMarker);
				}
			}
		}

		if(0)
		{
			int aidGroupCArmCam = 0;
			vzm::ActorParameters apGroupCams;
			vzm::NewActor(apGroupCams, "Tracking CAM Group", aidGroupCArmCam);
			vzm::AppendSceneItemToSceneTree(aidGroupCArmCam, sidScene);

			///////////////////////////////////
			// preprocessing information
			// *** IMPORTANT NOTE:
			// the C-arm image (Genoray) is aligned w.r.t. detector's view
			// the calibration image must be aligned w.r.t. source's view (view frustum's origin point)
			// therefore, the position mirrored horizontally
			cv::Mat imgCArm = cv::imread(imgFile);

			cv::FileStorage __fs("../data/Tracking 2023-04-19/testParams.txt", cv::FileStorage::Mode::WRITE);

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


			///////////////////////////////////

			cv::Mat rvec, tvec;
			const bool loadPrevRT = true;
			if(!loadPrevRT)
			{
				const bool mirrorHorizontal = true;
				std::map<int, cv::Point2f> pts2Dmap;
				pts2Dmap[100 + 6] = cv::Point2f(961, 215);
				pts2Dmap[100 + 3] = cv::Point2f(1146, 895);
				pts2Dmap[100 + 4] = cv::Point2f(263, 940);
				pts2Dmap[100 + 2] = cv::Point2f(756, 703);
				pts2Dmap[100 + 5] = cv::Point2f(1105, 485);
				pts2Dmap[100 + 1] = cv::Point2f(619, 325);
				pts2Dmap[100 + 7] = cv::Point2f(273, 341);

				pts2Dmap[200 + 6] = cv::Point2f(952, 350);
				pts2Dmap[200 + 3] = cv::Point2f(1155, 862);
				pts2Dmap[200 + 4] = cv::Point2f(245, 916);
				pts2Dmap[200 + 2] = cv::Point2f(753, 719);
				pts2Dmap[200 + 5] = cv::Point2f(1102, 547);
				pts2Dmap[200 + 1] = cv::Point2f(613, 435);
				pts2Dmap[200 + 7] = cv::Point2f(269, 456);

				pts2Dmap[300 + 6] = cv::Point2f(954, 494);
				pts2Dmap[300 + 3] = cv::Point2f(1168, 751);
				pts2Dmap[300 + 4] = cv::Point2f(260, 807);
				pts2Dmap[300 + 2] = cv::Point2f(766, 686);
				pts2Dmap[300 + 5] = cv::Point2f(1107, 590);
				pts2Dmap[300 + 1] = cv::Point2f(626, 551);
				pts2Dmap[300 + 7] = cv::Point2f(289, 569);

				pts2Dmap[400 + 6] = cv::Point2f(687, 653);
				pts2Dmap[400 + 3] = cv::Point2f(1173, 917);
				pts2Dmap[400 + 4] = cv::Point2f(389, 997);
				pts2Dmap[400 + 2] = cv::Point2f(736, 859);
				pts2Dmap[400 + 5] = cv::Point2f(939, 753);
				pts2Dmap[400 + 1] = cv::Point2f(434, 709);
				pts2Dmap[400 + 7] = cv::Point2f(118, 729);
				if (mirrorHorizontal)
				{
					for (auto it = pts2Dmap.begin(); it != pts2Dmap.end(); it++) {
						it->second.x = imgCArm.cols - it->second.x;
					}
				}

				using namespace std;
				using namespace glm;
				const int numCalPts = 7;
				const int numCalScans = 4;
				vector<fvec3> ptsRBS;
				ptsRBS.reserve(numCalPts* numCalScans);
				for (int i = 0; i < numCalScans; i++) {
					cv::FileStorage fs("../data/Tracking 2023-04-19/c-arm-track" + to_string(i + 1) + ".txt", 
						cv::FileStorage::Mode::READ);
					cv::Mat ocvMatRB2WS;
					fs["c-arm"] >> ocvMatRB2WS;

					fmat4x4 matRB2WS;
					memcpy(&matRB2WS, ocvMatRB2WS.ptr(), sizeof(float) * 16);
					fmat4x4 matWS2RB = glm::inverse(matRB2WS);

					for (int j = 0; j < numCalPts; j++) {
						cv::Mat ocvPos;
						fs["test1 Marker" + to_string(j + 1)] >> ocvPos;
						fvec3 posWS;
						memcpy(&posWS, ocvPos.ptr(), sizeof(float) * 3);
						fvec3 posRBS = vzmutils::transformPos(posWS, matWS2RB);
						ptsRBS.push_back(posRBS);
					}
					fs.release();
				}

				vector<cv::Point3f> pts3Ds(ptsRBS.size());
				vector<cv::Point2f> pts2Ds(pts2Dmap.size());
				memcpy(&pts3Ds[0], &ptsRBS[0], sizeof(fvec3) * numCalPts * numCalScans);
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
			}
			else {
				cv::FileStorage fs("../data/Tracking 2023-04-19/rb2carm.txt", cv::FileStorage::Mode::READ);
				fs["rvec"] >> rvec;
				fs["tvec"] >> tvec;
				__fs << "rvec" << rvec;
				__fs << "tvec" << tvec;
				fs.release();
			}

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
			
			const bool loadCArmTrackInfo = true;
			if(!loadCArmTrackInfo)
				trackInfo.GetRigidBodyByName("c-arm", &matRB2WS, NULL, NULL);
			else
			{
				// cArmTrackFile
				cv::FileStorage fs(cArmTrackFile, cv::FileStorage::Mode::READ);
				cv::Mat ocvRbMapTrAvr;
				fs["c-arm"] >> ocvRbMapTrAvr;
				memcpy(glm::value_ptr(matRB2WS), ocvRbMapTrAvr.ptr(), sizeof(float) * 16);
				fs.release();

				__fs << "rb2wsMat" << ocvRbMapTrAvr;
				__fs << "imgFile" << "../data/c-arm 2023-04-19/7084.png";
			}
			__fs.release();

			glm::fmat4x4 matCA2WS = matRB2WS * matCA2RB;
			int oidCamTris = 0, oidCamLines = 0, oidCamLabel = 0;
			navihelpers::CamOcv_Gen(matCA2WS, "C-Arm Source:test0", oidCamTris, oidCamLines, oidCamLabel);
			vzm::ActorParameters apCamTris, apCamLines, apCamLabel;
			apCamTris.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamTris);
			apCamTris.color[3] = 0.5f;
			apCamLines.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLines);
			*(glm::fvec4*)apCamLines.phong_coeffs = glm::fvec4(0, 1, 0, 0);
			apCamLines.line_thickness = 2;
			apCamLabel.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLabel);
			*(glm::fvec4*)apCamLabel.phong_coeffs = glm::fvec4(0, 1, 0, 0);
			int aidCamTris = 0, aidCamLines = 0, aidCamLabel = 0;
			vzm::NewActor(apCamTris, "C-Arm Cam Tris:test0", aidCamTris);
			vzm::NewActor(apCamLines, "C-Arm Lines:test0", aidCamLines);
			vzm::NewActor(apCamLabel, "C-Arm Label:test0", aidCamLabel);
			vzm::AppendSceneItemToSceneTree(aidCamTris, aidGroupCArmCam);
			vzm::AppendSceneItemToSceneTree(aidCamLines, aidGroupCArmCam);
			vzm::AppendSceneItemToSceneTree(aidCamLabel, aidGroupCArmCam);


			auto computeMatCS2PS = [](const float _fx, const float _fy, const float _s, const float _cx, const float _cy,
				const float widthPix, const float heightPix,
				const float near_p, const float far_p, glm::fmat4x4& matCS2PS)
			{
				double q = far_p / (near_p - far_p);
				double qn = far_p * near_p / (near_p - far_p);

				double fx = (double)_fx;
				double fy = (double)_fy;
				double sc = (double)_s;
				double cx = (double)_cx;
				double cy = (double)_cy;
				double width = (double)widthPix;
				double height = (double)heightPix;
				double x0 = 0, y0 = 0;

				matCS2PS[0][0] = 2.0 * fx / width;
				matCS2PS[1][0] = -2.0 * sc / width;
				matCS2PS[2][0] = (width + 2.0 * x0 - 2.0 * cx) / width;
				matCS2PS[3][0] = 0;
				matCS2PS[0][1] = 0;
				matCS2PS[1][1] = 2.0 * fy / height;
				matCS2PS[2][1] = -(height + 2.0 * y0 - 2.0 * cy) / height;
				matCS2PS[3][1] = 0;
				matCS2PS[0][2] = 0;
				matCS2PS[1][2] = 0;
				matCS2PS[2][2] = q;
				matCS2PS[3][2] = qn;
				matCS2PS[0][3] = 0;
				matCS2PS[1][3] = 0;
				matCS2PS[2][3] = -1.0;
				matCS2PS[3][3] = 0;
				//mat_cs2ps = glm::transpose(mat_cs2ps);
				//fmat_cs2ps = mat_cs2ps;
			};

			glm::fmat4x4 matCS2PS;
			computeMatCS2PS(arrayMatK[0], arrayMatK[4], arrayMatK[1], arrayMatK[2], arrayMatK[5],
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
			vzm::LoadImageFile(imgFile, oidImage, true, true);
			int oidCArmPlane = 0;
			vzm::GeneratePrimitiveObject(__FP ptsCArmPlanes[0], NULL, NULL, __FP ptsTexCoords[0], 4, idxList, 2, 3, oidCArmPlane);
			vzm::ActorParameters apCArmPlane;
			apCArmPlane.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCArmPlane);
			apCArmPlane.SetResourceID(vzm::ActorParameters::TEXTURE_2D, oidImage);
			apCArmPlane.script_params.SetParam("matCA2WS", matCA2WS); // temporal storage :)
			apCArmPlane.script_params.SetParam("imageWH", glm::ivec2(imgCArm.cols, imgCArm.rows)); // temporal storage :)
			*(glm::fvec4*)apCArmPlane.phong_coeffs = glm::fvec4(1, 0, 0, 0);
			int aidCArmPlane = 0;
			vzm::NewActor(apCArmPlane, "CArm Plane:test0", aidCArmPlane);
			vzm::AppendSceneItemToSceneTree(aidCArmPlane, sidScene);
		}
		else {
			RegisterCArmImage(sidScene, "../data/Tracking 2023-04-19/testParams.txt", "test0");
		}
	}
	UINT_PTR customData = (UINT_PTR)&trackingFrames;
#endif
	//UpdateFrame(csvData);
	//Render();
	SetTimer(g_hWnd, customData, 10, TimerProc);

    MSG msg;
	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_SNAVIWIN));

    // 기본 메시지 루프입니다:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
	}

#ifdef USE_MOTIVE
	tracker_alive = false; // make the thread finishes, this setting should be located right before the thread join
	tracker_processing_thread.join();
	udp_alive = false;
	udp_processing_thread.join();

	//end = GetMicroCounter();
	//printf("Elapsed Time (micro seconds) : %d", end - start);
	//
	//closesocket(s);
	//WSACleanup();

	optitrk::DeinitOptiTrackLib();
#endif
	vzm::DeinitEngineLib();

	GdiplusShutdown(gdiplusToken);

    return (int) msg.wParam;
}



//
//  함수: MyRegisterClass()
//
//  용도: 창 클래스를 등록합니다.
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
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_SNAVIWIN));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_SNAVIWIN);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

//
//   함수: InitInstance(HINSTANCE, int)
//
//   용도: 인스턴스 핸들을 저장하고 주 창을 만듭니다.
//
//   주석:
//
//        이 함수를 통해 인스턴스 핸들을 전역 변수에 저장하고
//        주 프로그램 창을 만든 다음 표시합니다.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // 인스턴스 핸들을 전역 변수에 저장합니다.

   //WND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	//   CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);
   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	   -1200, 10, DESIRED_SCREEN_W, DESIRED_SCREEN_H, nullptr, nullptr, hInstance, nullptr);
   //WND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	//   CW_USEDEFAULT, 0, DESIRED_SCREEN_W, DESIRED_SCREEN_H, nullptr, nullptr, hInstance, nullptr);
   if (!hWnd)
   {
      return FALSE;
   }
   g_hWnd = hWnd;

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

//
//  함수: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  용도: 주 창의 메시지를 처리합니다.
//
//  WM_COMMAND  - 애플리케이션 메뉴를 처리합니다.
//  WM_PAINT    - 주 창을 그립니다.
//  WM_DESTROY  - 종료 메시지를 게시하고 반환합니다.
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	int cidCam1 = vzmutils::GetSceneItemIdByName("World Camera");
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
#ifdef USE_MOTIVE
					if (activeCarmIdx < 0)
						break;
#endif

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

#ifdef USE_MOTIVE
			static char SAVEKEYS[10] = { 'Q' , 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P' };
			const int maxSamples = 30;
			const float normalizedNum = 1.f / (float)maxSamples;

			for (int i = 0; i < 10; i++) {
				if (wParam == SAVEKEYS[i]) {
					glm::fvec3 rotAxisAvr = glm::fvec3(0, 0, 0);
					float rotAngleAvr = 0;
					glm::fvec3 trAvr = glm::fvec3(0, 0, 0);
					int captureCount = 0;
					while (captureCount < maxSamples) {
						navihelpers::track_info trackInfo;
						g_track_que->wait_and_pop(trackInfo);
						Sleep(50);
						glm::fquat q;
						glm::fvec3 t;
						if (trackInfo.GetRigidBodyQuatTVecByName("c-arm", &q, &t)) {
							float quatMagnitude = glm::length(q);
							q /= quatMagnitude;

							float rotationAngle = 2.0f * acos(q.w);

							float sinAngle = sin(rotationAngle / 2.0f);
							glm::fvec3 rotationAxis = glm::fvec3(q.x, q.y, q.z) / sinAngle;

							rotAngleAvr += rotationAngle * normalizedNum;
							rotAxisAvr += rotationAxis * normalizedNum;

							trAvr += t * normalizedNum;
							captureCount++;
						}
						std::cout << "Capturing... " << captureCount << std::endl;
					}
					std::cout << "Capturing... DONE!" << std::endl;
					rotAxisAvr = glm::normalize(rotAxisAvr);
					glm::fmat4x4 mat_r = glm::rotate(rotAngleAvr, rotAxisAvr);
					glm::fmat4x4 mat_t = glm::translate(trAvr);
					glm::fmat4x4 matRB2WS = mat_t * mat_r;

					StoreParams("test" + to_string(i) + ".txt", matRB2WS, "../data/Tracking Test 2023-05-09/test.png");
					break;
				}
			}
#endif

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

// 정보 대화 상자의 메시지 처리기입니다.
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
