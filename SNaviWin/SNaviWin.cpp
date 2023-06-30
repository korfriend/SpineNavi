// SNaviWin.cpp : 애플리케이션에 대한 진입점을 정의합니다.
// DOJO : 자체적으로 개발한 namespace
// 엔진 : vzm, vzmutils, vzmproc
// 어플리케이션 helper : navihelpers
// 위의 namespace 외엔 glm 과 같은 commonly-used 3rd party lib 이거나 C++ standard lib 또는 windows SDK lib 임

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

#define USE_MOTIVE
#define DESIRED_SCREEN_W 950
#define DESIRED_SCREEN_H 950
#define USE_WHND true
bool storeAvrCarmTrackinfo = false;
std::map<std::string, glm::fmat4x4> rbMapTrAvr;
std::map<std::string, glm::fvec3> mkPosTest1Avr;

#define MAX_LOADSTRING 100
typedef unsigned long long u64;

std::string folder_data = "";
std::string folder_capture = "";
std::string folder_trackingInfo = "";
std::string folder_optiSession = "";

// 전역 변수:
HWND g_hWnd, g_hWndDialog1;
HINSTANCE hInst;                                // 현재 인스턴스입니다.
WCHAR szTitle[MAX_LOADSTRING];                  // 제목 표시줄 텍스트입니다.
WCHAR szWindowClass[MAX_LOADSTRING];            // 기본 창 클래스 이름입니다.

// 이 코드 모듈에 포함된 함수의 선언을 전달합니다:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
LRESULT CALLBACK    DiagProc1(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

#define SCANIMG_W 1296
#define SCANIMG_H 1296
cv::Mat g_curScanImg(SCANIMG_W, SCANIMG_H, CV_8UC1);

std::atomic_bool download_completed{ false };

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
	// DOJO : 렌더링된 버퍼를 받아서 GUI window buffer 에 그리기
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
	// DOJO : Scene 은 Camera(s), Light source(s), Actor(s)로 구성, World Space 의 Coordinate System 을 정의함 
	// 
	// we set 'meter scale world to world space

	// DOJO : Actor 는 Scene 에 그려지는 Model 이며, 
	// Actor 를 구성하는 actor resource (called resource object) 는 geometry 및 material (e.g., texture) 등으로 정의
	// vzm::GenerateXXXObject 는 actor resource 를 생성 
	// vzm::LoadXXX 는 file 로부터 actor resource 를 생성
	// actor resource 의 geometry 는 기본적으로 Object Space (Model Space) 에서 정의된 것으로 봄
	
	// DOJO : Axis 를 위한 actor resource 를 생성
	// oidAxis (objId) 은 생성된 actor resource 의 ID 를 받아옴
	// objId = 0 인 경우, 항상 새로운 actor resource 를 생성하고 새로운 ID 를 받음
	// objId != 0 인 경우, 해당 ID 의 actor resource 가 있을 때, 해당 actor resource 를 update 함 (ID 는 그대로 사용)
	// objId != 0 인 경우, 해당 ID 의 actor resource 가 없을 때, 새로운 actor resource 를 생성하고 새로운 ID 를 받음
	int oidAxis = 0;
	vzm::GenerateAxisHelperObject(oidAxis, 0.5f);

	// DOJO : Actor Parameter (vzm::ActorParameters) 와 actor resource 로 actor 생성
	vzm::ActorParameters apAxis;
	// vzm::ActorParameters::SetResourceID 로 다양한 actor resource 를 actor 에 할당할 수 있음
	// 기본적으로 vzm::ActorParameters::RES_USAGE::GEOMETRY 가 있어야 함, 그 외 여러 종류의 texture 들이 올 수 있음 (ref. vzm::ActorParameters::RES_USAGE)
	apAxis.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidAxis);
	// line 으로 정의된 geometry 에 대해 렌더링에서 사용되는 parameter (c.f., triangle geometry 의 경우 해당 parameter 가 렌더링에서 사용되지 않음)
	apAxis.line_thickness = 3;
	// color 의 경우, 해당 actor 에 대해 global 로 적용됨 
	// geometry resource 가 vertex 단위로 color 가 정의되어 있다면, 여기선 alpha 값만 사용됨
	*(glm::fvec4*)apAxis.color = glm::fvec4(1, 1, 1, 1);
	// DOJO : 새로운 액터 생성
	// actor name parameter 와 actor parameter 필요
	// actor name parameter 로 actor 의 ID 를 가져올 수 있음
	int aidAxis = 0;
	vzm::NewActor(apAxis, "World Axis", aidAxis);

	// DOJO : navihelpers 는 이번 프로젝트에서 사용할 헬퍼 함수를 담고 있음
	int oidGrid, oidLines, oidTextX, oidTextZ;
	// scene 의 바닥에 그려지는 actor resources 를 생성
	navihelpers::World_GridAxis_Gen(oidGrid, oidLines, oidTextX, oidTextZ);

	std::vector<glm::fvec3> pinfo(3);
	pinfo[0] = glm::fvec3(0, 0.5, 0);
	pinfo[1] = glm::fvec3(0, 0, -1);
	pinfo[2] = glm::fvec3(0, 1, 0);
	// DOJO : "Frame" 이라는 text 를 그리는 actor resource 를 생성 (자세한 사용법은 문의 바람)
	// 여기서 int oidFrameText; 이렇게 했는데, 새롭게 생성하는 경우 oidFrameText = 0 을 권장
	// DEBUG 모드에서는 initialize 안 하면, 보통 0 값이 들어 가지만, 
	// RELEASE 모드에서는 initialize 안 하면, 임의의 값이 들어 가고, 이 때, (거의 발생하진 않지만) 다른 actor resource 의 ID 값이 들어가면 비정상 동작할 수 있음
	int oidFrameText; // recommend "int oidFrameText = 0;"
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

	// DOJO : (world view 에서의) Camera 를 정의하기 위한 파라미터
	vzm::CameraParameters cpCam1;
	*(glm::fvec3*)cpCam1.pos = glm::fvec3(-1.5, 1.5, -1.5);
	*(glm::fvec3*)cpCam1.up = glm::fvec3(0, 1, 0);
	*(glm::fvec3*)cpCam1.view = glm::fvec3(1, -1, 1);

	// YAML 로 이전에 저장된 (world view 에서의) 카메라 위치 정보 읽어 들임
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

	// 렌더링될 buffer 사이즈
	cpCam1.w = rcWorldView.right - rcWorldView.left;
	cpCam1.h = rcWorldView.bottom - rcWorldView.top;

	// 렌더링 파이프라인의 View Frustum 에서의 near plane, far plane distances
	cpCam1.np = 0.1f;
	cpCam1.fp = 100.f;

	float vFov = 3.141592654f / 4.f;
	// NOTE : union 으로 저장되어 있음
	// 카메라 view projection 의 자연스러운 이동 (to C-arm View) 을 위해, 
	// 카메라 설정 convention 을 camera (intrinsic) parameters 로 사용
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
	// DOJO : scene 에 등록될 카메라 생성, 여기선 두 개 생성 (AP, Lateral 용)
	int cidCam1 = 0, cidCam2 = 0;
	vzm::NewCamera(cpCam1, "Cam1", cidCam1);
	cpCam1.hWnd = USE_WHND ? g_hWndDialog1 : NULL;
	vzm::NewCamera(cpCam1, "Cam2", cidCam2);

	// DOJO : Light 를 정의하기 위한 파라미터
	vzm::LightParameters lpLight1;
	lpLight1.is_on_camera = true;
	lpLight1.is_pointlight = false;
	*(glm::fvec3*)lpLight1.pos = *(glm::fvec3*)cpCam1.pos;
	*(glm::fvec3*)lpLight1.dir = *(glm::fvec3*)cpCam1.view;
	*(glm::fvec3*)lpLight1.up = *(glm::fvec3*)cpCam1.up;
	int lidLight1 = 0;
	// DOJO : scene 에 등록될 Light 생성
	vzm::NewLight(lpLight1, "World Light", lidLight1);

	// DOJO : scene 생성
	int sidScene = 0;
	vzm::NewScene("Scene1", sidScene);

	// DOJO : scene 에 camera 1 (cidCam1) 을 연결 
	vzm::AppendSceneItemToSceneTree(cidCam1, sidScene);
	// DOJO : scene 에 camera 2 (cidCam2) 을 연결 
	vzm::AppendSceneItemToSceneTree(cidCam2, sidScene);
	// DOJO : scene 에 Light (lidLight1) 을 연결 
	vzm::AppendSceneItemToSceneTree(lidLight1, sidScene);
	// DOJO : scene 에 axis actor (aidAxis) 연결 
	vzm::AppendSceneItemToSceneTree(aidAxis, sidScene);
	// DOJO : scene 에 ground grid actor (aidGrid) 을 연결 
	vzm::AppendSceneItemToSceneTree(aidGrid, sidScene);
	// DOJO : scene 에 ground grid 에서의 진한 라인 actor (aidLines) 을 연결 
	vzm::AppendSceneItemToSceneTree(aidLines, sidScene);
	// DOJO : scene 에 ground grid 위에 있는 "X" 글자 actor (aidTextX) 을 연결 
	vzm::AppendSceneItemToSceneTree(aidTextX, sidScene);
	// DOJO : scene 에 ground grid 위에 있는 "Z" 글자 actor (aidTextZ) 을 연결 
	vzm::AppendSceneItemToSceneTree(aidTextZ, sidScene);
	// DOJO : scene 에 "Frame" 글자 actor (aidTextFrame) 을 연결 
	vzm::AppendSceneItemToSceneTree(aidTextFrame, sidScene);
	// NOTE : 여기에서는 모든 actor 및 camera, light 등 scene item 들이 모두 scene (i.e., root) 에 바로 연결되었지만,
	// 특정 actor 의 자식으로 붙을 수도 있다 (e.g., scene<-aidAxis<-aidTextZ<-cidCam1 ,...)
}

void UpdateTrackInfo2Scene(navihelpers::track_info& trackInfo) {

	int numRBs = trackInfo.NumRigidBodies();
	int numMKs = trackInfo.NumMarkers();

	for (int i = 0; i < numRBs; i++) {
		std::string rbName;
		glm::fmat4x4 matLS2WS;
		if (trackInfo.GetRigidBodyByIdx(i, &rbName, &matLS2WS, NULL, NULL)) {
			
			int aidRb = vzmutils::GetSceneItemIdByName(rbName);
			vzm::ActorParameters apRb;
			vzm::GetActorParams(aidRb, apRb);
			apRb.is_visible = true;

			if (storeAvrCarmTrackinfo)
				matLS2WS = rbMapTrAvr[rbName];

			apRb.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidRb, apRb);
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
			apMarker.is_pickable = false;
			apMarker.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidMarker, apMarker);
		}
	}
}

int numAnimationCount = 50;
int arAnimationKeyFrame1 = -1;
std::vector<vzm::CameraParameters> cpInterCams1(numAnimationCount);
int arAnimationKeyFrame2 = -1;
std::vector<vzm::CameraParameters> cpInterCams2(numAnimationCount);
std::string g_sceneName = "Scene1"; // Scene1, Scene2
std::string g_camName = "Cam1"; // World Camera, CArm Camera
std::string g_camName2 = "Cam2"; // World Camera, CArm Camera

void Render() {
	int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(g_camName);

	if (sidScene != 0 && cidCam1 != 0) {
		// show case
		if (arAnimationKeyFrame1 >= 0) {
			vzm::CameraParameters cpCam = cpInterCams1[arAnimationKeyFrame1++];
			vzm::SetCameraParams(cidCam1, cpCam);
			if (arAnimationKeyFrame1 == numAnimationCount)
				arAnimationKeyFrame1 = -1;

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

	int cidCam2 = vzmutils::GetSceneItemIdByName(g_camName2);
	if (sidScene != 0 && cidCam2 != 0) {
		// show case
		if (arAnimationKeyFrame2 >= 0) {
			vzm::CameraParameters cpCam = cpInterCams2[arAnimationKeyFrame2++];
			vzm::SetCameraParams(cidCam2, cpCam);
			if (arAnimationKeyFrame2 == numAnimationCount)
				arAnimationKeyFrame2 = -1;

			//UINT flags = SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE;
			//SetWindowPos(g_hWnd, NULL, 100, 100, cpCam.w, cpCam.h, flags);
		}

		vzm::RenderScene(sidScene, cidCam2);

		if (USE_WHND)
			vzm::PresentHWND(g_hWndDialog1);
		else {
			UpdateBMP(cidCam2, g_hWndDialog1);
			InvalidateRect(g_hWndDialog1, NULL, FALSE);
		}
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

		int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
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
		static int oidProbe = 0;
		if (oidAxis == 0) {
			vzm::GenerateAxisHelperObject(oidAxis, 0.15f);
		}
		if (oidMarker == 0) {
			glm::fvec4 pos(0, 0, 0, 1.f);
			vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);
		}
		if (oidProbe == 0) {
			//vzm::LoadModelFile(folder_data + "probe.obj", oidProbe);

			glm::fvec3 posS(0, 0, 0.25f);
			glm::fvec3 posE(0, 0, 0);
			vzm::GenerateArrowObject(__FP posS, __FP posE, 0.0025f, 0.0025f, oidProbe);
		}

		static map<string, int> sceneActors;

		for (int i = 0; i < numRBs; i++) {
			std::string rbName;
			if (trackInfo.GetRigidBodyByIdx(i, &rbName, NULL, NULL, NULL)) {
				auto actorId = sceneActors.find(rbName);
				if (actorId == sceneActors.end()) {
					vzm::ActorParameters apRb;
					if (rbName == "probe") {;
						apRb.SetResourceID(vzm::ActorParameters::GEOMETRY, oidProbe);
						*(glm::fvec4*)apRb.color = glm::fvec4(1, 0.3, 0.2, 1);
					}
					else {
						apRb.SetResourceID(vzm::ActorParameters::GEOMETRY, oidAxis);
						apRb.line_thickness = 3;
					}
					apRb.is_visible = false;
					int aidRb = 0;
					vzm::NewActor(apRb, rbName, aidRb);
					vzm::AppendSceneItemToSceneTree(aidRb, sidScene);

					if (rbName == "probe") {
						sceneActors[rbName] = 1;
					}
					else {
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
						vzm::AppendSceneItemToSceneTree(aidLabelText, aidRb);
						sceneActors[rbName] = 2;
					}

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

	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	// engine initialization to use the core APIs of framework
	// must be paired with DeinitEngineLib()
	vzm::InitEngineLib("SpineNavi");
	vzm::SetLogConfiguration(true, 4);

#ifdef USE_MOTIVE
    bool optitrkMode = optitrk::InitOptiTrackLib();

	optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-05-20.motive", folder_data + "System Calibration.cal");
#else
	//rapidcsv::Document trackingData(folder_optiSession + "Take 2023-04-19 05.12.57 PM.csv", rapidcsv::LabelParams(0, 0)); // 7081.png
	//std::string cArmTrackFile = folder_trackingInfo + "c-arm-track1.txt";
	//rapidcsv::Document trackingData(folder_optiSession + "Take 2023-04-19 05.13.30 PM.csv", rapidcsv::LabelParams(0, 0));  // 7082.png
	//std::string cArmTrackFile = folder_trackingInfo + "c-arm-track2.txt";
	//rapidcsv::Document trackingData(folder_optiSession + "Take 2023-04-19 05.14.05 PM.csv", rapidcsv::LabelParams(0, 0));  // 7083.png
	//std::string cArmTrackFile = folder_trackingInfo + "c-arm-track3.txt";
	rapidcsv::Document trackingData(folder_optiSession + "Take 2023-04-19 05.14.56 PM.csv", rapidcsv::LabelParams(0, 0));  // 7084.png
	std::string cArmTrackFile = folder_trackingInfo + "c-arm-track4.txt";

	std::string imgFile = folder_capture + "7084.png";
	
	//rapidcsv::Document trackingData(folder_trackingInfo + "Take 2023-04-19 05.18.13 PM.csv", rapidcsv::LabelParams(0, 0)); // 7085.png
	//rapidcsv::Document trackingData(folder_trackingInfo + "Take 2023-04-19 05.19.59 PM.csv", rapidcsv::LabelParams(0, 0));
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
	int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
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

		std::ifstream _file(folder_trackingInfo + "testParams.txt");
		bool existFile = _file.is_open();
		_file.close();
		if(!existFile)
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

			cv::FileStorage __fs(folder_trackingInfo + "testParams.txt", cv::FileStorage::Mode::WRITE);

			cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
			cv::Mat matK;
			fs["K"] >> matK;
			__fs << "K" << matK;
			cv::Mat distCoeffs;
			fs["DistCoeffs"] >> distCoeffs;
			__fs << "DistCoeffs" << distCoeffs;

			//cv::Mat undistImg;
			//cv::undistort(imgCArm, undistImg, _matK, _distCoeffs);
			//
			////std::string imgFile = folder_capture + "7084.png";
			//cv::imwrite(folder_capture + "undist_7084.png", undistImg);
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
					cv::FileStorage fs(folder_trackingInfo + "c-arm-track" + to_string(i + 1) + ".txt", 
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

				cv::solvePnP(pts3Ds, pts2Ds, matK, distCoeffs, rvec, tvec);

				cv::FileStorage fs(folder_trackingInfo + "rb2carm.txt", cv::FileStorage::Mode::WRITE);
				fs.write("rvec", rvec);
				fs.write("tvec", tvec);
				fs.release();
			}
			else {
				//cv::FileStorage fs(folder_trackingInfo + "rb2carm.txt", cv::FileStorage::Mode::READ);
				cv::FileStorage fs(folder_trackingInfo + "rb2carm.txt", cv::FileStorage::Mode::READ);
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
				__fs << "imgFile" << folder_capture + "7084.png";
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


			glm::fmat4x4 matCS2PS;

			navihelpers::ComputeMatCS2PS(matK.at<double>(0, 0), matK.at<double>(1, 1), matK.at<double>(0, 1), 
				matK.at<double>(0, 2), matK.at<double>(1, 2), (float)imgCArm.cols, (float)imgCArm.rows, 0.1f, 1.2f, matCS2PS);
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
			vzm::LoadImageFile(folder_capture + "undist_7084.png", oidImage, true, true);
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
			RegisterCArmImage(sidScene, folder_trackingInfo + "testParams.txt", "test0");
		}
	}
	UINT_PTR customData = (UINT_PTR)&trackingFrames;
#endif
	//UpdateFrame(csvData);
	//Render();
	SetTimer(g_hWnd, customData, 10, TimerProc);

	/////////////////////////////////////////////////////////
	// TCP routine
#define CFG_LISTEN_PORT 22222
#define CFG_SIZE_MAX_FD 1000
#define CFG_SIZE_MAX_RECV_BUFFER 5000

	std::atomic_bool network_alive{ true };
	std::thread network_processing_thread([&]() {
		WSADATA wsaData;
		WSAStartup(MAKEWORD(2, 2), &wsaData);


		/* create socket */
		SOCKET serverSocket;
		serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (serverSocket < 0) {
			printf("can not create socket\n");
			return -1;
		}

		/* bind socket */
		struct sockaddr_in serverAddr;
		memset(&serverAddr, 0, sizeof(serverAddr));
		serverAddr.sin_family = AF_INET;
		serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		serverAddr.sin_port = htons(CFG_LISTEN_PORT);

		if (bind(serverSocket, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
		{
			printf("bind error\n");
			return -1;
		}

		/* listen */
		if (listen(serverSocket, CFG_SIZE_MAX_FD) == SOCKET_ERROR) {
			printf("listen error\n");
			return -1;
		}

		printf("listen %d port...\n", CFG_LISTEN_PORT);

		/* fd_set에 서버 소켓 설정 */
		fd_set reads;
		fd_set copy_reads;
		FD_ZERO(&reads);
		FD_SET(serverSocket, &reads);
		int maxFd = serverSocket;

		/* select timeout 1초 설정 */
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		int clientSocket = -1;
		struct sockaddr_in clientAddr;
		unsigned int clientAddrSize = sizeof(clientAddr);

		const int bufferSize = CFG_SIZE_MAX_RECV_BUFFER;
		char buffer[bufferSize];

		int totalBufferNum;
		int BufferNum;
		int readBytes;
		long file_size;
		long totalReadBytes;
		uint numTimeOut = 0;

		while (network_alive) {

			while (true && !download_completed) {
				/* timeout 시간 재설정 */
				timeout.tv_sec = 1;
				timeout.tv_usec = 0;

				/* fd_set 복사 */
				copy_reads = reads;

				/* select */
				int result = select(maxFd + 1, &copy_reads, NULL, NULL, &timeout);

				/* select 결과 예외처리 */
				if (result < 0) {
					printf("\rselect error");
					fflush(stdout);
					break;
				}

				if (result == 0) {
					printf("\rtime out...%d", ++numTimeOut);
					fflush(stdout);
					break;
				}

				/* server listen socket에 input event가 있는지 확인 */
				if (FD_ISSET(serverSocket, &copy_reads)) {
					/* input event가 있으면 accept */
					clientSocket = accept(serverSocket,
						(struct sockaddr*)&clientAddr,
						(int*)&clientAddrSize);

					if (clientSocket < 0) {
						printf("\ninvalid client socket. [clientSocket:%d]\n",
							clientSocket);
						break;
					}

					printf("\naccept new client. [clientSocket:%d]\n",
						clientSocket);

					//int safeTmpBytes = 5000;
					//readBytes = recv(clientSocket, buffer, bufferSize, 0);
					//safeTmpBytes = readBytes;

					file_size = SCANIMG_W * SCANIMG_H;// atol(buffer);

					unsigned char* data = new unsigned char[file_size];
					totalReadBytes = 0;

					//bool isOffsetProcCompleted = false;

					//FILE* fp;
					//fopen_s(&fp, (folder_capture + "result.raw").c_str(), "wb");
					std::vector<char> bufferTmp(file_size * 4);
					while (file_size > totalReadBytes) {
						readBytes = recv(clientSocket, buffer, bufferSize, 0);
						//if (totalTmpBytes < 5000) continue;
						if (readBytes < 0) break;

						//fwrite(buffer, sizeof(char), readBytes, fp);
						memcpy(&bufferTmp[totalReadBytes], buffer, readBytes);

						totalReadBytes += readBytes;
						printf("\rIn progress: %d/%d byte(s) [%d]", totalReadBytes, file_size, (int)std::min((totalReadBytes * 100.f) / file_size, 100.f));
						fflush(stdout);
					}
					if (readBytes > 0) {
						download_completed = true;
						memcpy(g_curScanImg.ptr(), &bufferTmp[0], file_size);
						//cv::imshow("test", g_curScanImg);
						//cv::waitKey(1);
					}

					//fclose(fp);
					/* client socket 종료 */
					closesocket(clientSocket);
					printf("close [clientSocket:%d]\n", clientSocket);
				}
			}

		}

		if (serverSocket != -1) {
			closesocket(serverSocket);
		}
		WSACleanup();
	});


    MSG msg;
	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_SNAVIWIN));

    // 기본 메시지 루프입니다:
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

#ifdef USE_MOTIVE
	tracker_alive = false; // make the thread finishes, this setting should be located right before the thread join
	tracker_processing_thread.join();

	optitrk::DeinitOptiTrackLib();
#endif
	vzm::DeinitEngineLib();

	network_alive = false;
	network_processing_thread.join();
	//end = GetMicroCounter();
	//printf("Elapsed Time (micro seconds) : %d", end - start);
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
	   -1915, 5, DESIRED_SCREEN_W, DESIRED_SCREEN_H, nullptr, nullptr, hInstance, nullptr);
   //HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	//   CW_USEDEFAULT, 0, DESIRED_SCREEN_W, DESIRED_SCREEN_H, nullptr, nullptr, hInstance, nullptr);
   if (!hWnd)
   {
      return FALSE;
   }
   g_hWnd = hWnd;

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   g_hWndDialog1 = CreateDialog(hInst, MAKEINTRESOURCE(IDD_FORMVIEW), hWnd, DiagProc1);

   RECT rc1;
   GetWindowRect(g_hWnd, &rc1);
   SetWindowPos(g_hWndDialog1, 0, rc1.right + 5, 10, DESIRED_SCREEN_W, DESIRED_SCREEN_H, SWP_SHOWWINDOW);
   if (!g_hWndDialog1)
   {
	   return FALSE;
   }
   ShowWindow(g_hWndDialog1, SW_SHOW);
   UpdateWindow(g_hWndDialog1);

   return TRUE;
}

void MoveCameraToCArmView(const int carmIdx, const int cidCam,
	std::vector<vzm::CameraParameters>& cpInterCams, int& arAnimationKeyFrame) {

	vzm::CameraParameters cpCam;
	if (!vzm::GetCameraParams(cidCam, cpCam))
		return;

	cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
	cv::Mat cameraMatrix;
	fs["K"] >> cameraMatrix;
	fs.release();

	int aidCArmPlane = vzmutils::GetSceneItemIdByName("CArm Plane:test" + std::to_string(carmIdx));
	if (aidCArmPlane == 0)
		return;

	vzm::ActorParameters apCArmPlane;
	vzm::GetActorParams(aidCArmPlane, apCArmPlane);
	glm::fmat4x4 matCA2WS = apCArmPlane.script_params.GetParam("matCA2WS", glm::fmat4x4(1));
	glm::ivec2 imageWH = apCArmPlane.script_params.GetParam("imageWH", glm::ivec2(0));

	vzm::CameraParameters cpNewCam = cpCam;

	glm::fvec3 posPrev = *(glm::fvec3*)cpCam.pos;
	glm::fvec3 viewPrev = *(glm::fvec3*)cpCam.view;
	glm::fvec3 upPrev = *(glm::fvec3*)cpCam.up;

	UINT widthWindow = cpCam.w;
	UINT heightWindow = cpCam.h;

	const float intrinsicRatioX = (float)imageWH.x / widthWindow;
	const float intrinsicRatioY = (float)imageWH.y / heightWindow;

	cpNewCam.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_INTRINSICS;
	glm::fvec3 pos(0, 0, 0);
	glm::fvec3 view(0, 0, 1);
	glm::fvec3 up(0, -1, 0);
	pos = *(glm::fvec3*)cpNewCam.pos = vzmutils::transformPos(pos, matCA2WS);
	view = *(glm::fvec3*)cpNewCam.view = vzmutils::transformVec(view, matCA2WS);
	up = *(glm::fvec3*)cpNewCam.up = vzmutils::transformVec(up, matCA2WS);
	cpNewCam.fx = cameraMatrix.at<double>(0, 0) / intrinsicRatioX;
	cpNewCam.fy = cameraMatrix.at<double>(1, 1) / intrinsicRatioY;
	cpNewCam.sc = cameraMatrix.at<double>(0, 1);
	cpNewCam.cx = cameraMatrix.at<double>(0, 2) / intrinsicRatioX;
	cpNewCam.cy = cameraMatrix.at<double>(1, 2) / intrinsicRatioY;
	cpNewCam.np = 0.2;

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
		_cpCam = cpNewCam;
		*(glm::fvec3*)_cpCam.pos = posPrev + (pos - posPrev) * t;
		*(glm::fvec3*)_cpCam.view = _view;
		*(glm::fvec3*)_cpCam.up = _up;

#define INTERPOLCAM(PARAM) _cpCam.PARAM = cpCam.PARAM + (cpNewCam.PARAM - cpCam.PARAM) * t;

		//INTERPOLCAM(w);
		//INTERPOLCAM(h);
		INTERPOLCAM(fx);
		INTERPOLCAM(fy);
		INTERPOLCAM(sc);
		INTERPOLCAM(cx);
		INTERPOLCAM(cy);
	}
	arAnimationKeyFrame = 0;
}

auto StoreParams = [](const std::string& paramsFileName, const glm::fmat4x4& matRB2WS, const std::string& imgFileName) {

	cv::FileStorage __fs(folder_trackingInfo + paramsFileName, cv::FileStorage::Mode::WRITE);

	cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
	cv::Mat matK;
	fs["K"] >> matK;
	__fs << "K" << matK;
	cv::Mat distCoeffs;
	fs["DistCoeffs"] >> distCoeffs;
	__fs << "DistCoeffs" << distCoeffs;
	fs.open(folder_trackingInfo + "rb2carm1.txt", cv::FileStorage::Mode::READ);
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

auto SaveAndChangeViewState = [](const int keyParam, const int sidScene, const int cidCam,
	std::vector<vzm::CameraParameters>& cpInterCams, int& arAnimationKeyFrame) {
	using namespace std;
	static char LOADKEYS[10] = { '1' , '2', '3', '4', '5', '6', '7', '8', '9', '0' };
	static map<int, int> mapAidGroupCArmCam;
	for (int i = 0; i < 10; i++) {
		if (keyParam == LOADKEYS[i]) {
#ifdef USE_MOTIVE
			if (download_completed) {
				glm::fvec3 rotAxisAvr = glm::fvec3(0, 0, 0);
				float rotAngleAvr = 0;
				glm::fvec3 trAvr = glm::fvec3(0, 0, 0);
				int captureCount = 0;
				const int maxSamples = 30;
				const float normalizedNum = 1.f / (float)maxSamples;
				while (captureCount < maxSamples) {
					navihelpers::track_info trackInfo;
					g_track_que->wait_and_pop(trackInfo);
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
					//std::cout << "Capturing... " << captureCount << std::endl;
					printf("\rCapturing... %d", captureCount);
					fflush(stdout);
				}
				std::cout << "Capturing... DONE!" << std::endl;
				rotAxisAvr = glm::normalize(rotAxisAvr);
				glm::fmat4x4 mat_r = glm::rotate(rotAngleAvr, rotAxisAvr);
				glm::fmat4x4 mat_t = glm::translate(trAvr);
				glm::fmat4x4 matRB2WS = mat_t * mat_r;

				cv::Mat downloadImg;
				cv::cvtColor(g_curScanImg, downloadImg, cv::COLOR_GRAY2RGB);
				string downloadImgFileName = folder_trackingInfo + "test" + to_string(i) + ".png";
				std::cout << "STORAGE COMPLETED!!!  1" << std::endl;
				cv::imwrite(downloadImgFileName, downloadImg);
				std::cout << "STORAGE COMPLETED!!!  2" << std::endl;
				StoreParams("test" + to_string(i) + ".txt", matRB2WS, downloadImgFileName);
				download_completed = false;
				std::cout << "STORAGE COMPLETED!!!  3" << std::endl;
			}
			else { // !download_completed

			}
#endif
			// load case
			auto it = mapAidGroupCArmCam.find(i + 1);
			if (it != mapAidGroupCArmCam.end())
				vzm::RemoveSceneItem(it->second, true);
			int aidGroup = RegisterCArmImage(sidScene, folder_trackingInfo + "test" + to_string(i) + ".txt", "test" + to_string(i));
			if (aidGroup == -1)
				break;
			mapAidGroupCArmCam[i + 1] = aidGroup;
			MoveCameraToCArmView(i, cidCam, cpInterCams, arAnimationKeyFrame);
			break;
		}
	}
};

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
	int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(g_camName);
	float scene_stage_scale = 5.f;

	glm::fvec3 scene_stage_center = glm::fvec3();
	vzm::CameraParameters cpCam1;
	if (sidScene != 0 && cidCam1 != 0) {
		vzm::GetCameraParams(cidCam1, cpCam1);

		scene_stage_scale = glm::length(scene_stage_center - *(glm::fvec3*)cpCam1.pos) * 1.0f;
	}

	static vzmutils::GeneralMove general_move;

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
				break;
			}
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
				break;
			}
			case char('A') :
			{
				//cpCam1.
				// Call SetWindowPos to resize the window
				//UINT flags = SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE;
				//SetWindowPos(g_hWnd, NULL, 100, 100, imageWH.x, imageWH.y, flags);
				break;
			}
		}

		SaveAndChangeViewState(wParam, sidScene, cidCam1, cpInterCams1, arAnimationKeyFrame1);
		break;
	}
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


LRESULT CALLBACK DiagProc1(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
	int cidCam2 = vzmutils::GetSceneItemIdByName(g_camName2);
	float scene_stage_scale = 5.f;
	glm::fvec3 scene_stage_center = glm::fvec3();
	vzm::CameraParameters cpCam2;
	if (sidScene != 0 && cidCam2 != 0) {
		vzm::GetCameraParams(cidCam2, cpCam2);

		scene_stage_scale = glm::length(scene_stage_center - *(glm::fvec3*)cpCam2.pos) * 1.0f;
	}

	static vzmutils::GeneralMove general_move;
	static std::set<int> excludedMKIndices;
	static bool mouseCamFlag = false;

	switch (message)
	{
	case WM_KEYDOWN:
	{
		SaveAndChangeViewState(wParam, sidScene, cidCam2, cpInterCams2, arAnimationKeyFrame2);
		break;
	}
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
		break;
	}
	case WM_PAINT:
	{
		if (sidScene != 0)
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
		break;
	}
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
		break;
	}
	//case WM_ERASEBKGND:
	//	return TRUE; // tell Windows that we handled it. (but don't actually draw anything)
	//default:
	//	return DefWindowProc(hDlg, message, wParam, lParam);
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
