// SCalibratorWin.cpp : Defines the entry point for the application.
//

#include "framework.h"
#include "SCalibratorWin.h"

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

void Render() {
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	int cidCam1 = vzmutils::GetSceneItemIdByName("World Camera");

	if (sidScene != 0 && cidCam1 != 0) {
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

int main()
{
	return wWinMain(GetModuleHandle(NULL), NULL, GetCommandLine(), SW_SHOWNORMAL);
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

			//if (storeAvrCarmTrackinfo)
			//	matLS2WS = rbMapTrAvr[rbName];

			apAxis.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidAxis, apAxis);
		}
	}

	for (int i = 0; i < numMKs; i++) {
		std::map<navihelpers::track_info::MKINFO, std::any> mk;
		if (trackInfo.GetMarkerByIdx(i, mk)) {
			std::string mkName = std::any_cast<std::string>(mk[navihelpers::track_info::MKINFO::MK_NAME]);
			glm::fvec3 pos = std::any_cast<glm::fvec3>(mk[navihelpers::track_info::MKINFO::POSITION]);

			//if (storeAvrCarmTrackinfo)
			//{
			//	auto mkp = mkPosTest1Avr.find(mkName);
			//	if (mkp != mkPosTest1Avr.end())
			//		pos = mkPosTest1Avr[mkName];
			//}

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

void CALLBACK TimerProc(HWND, UINT, UINT_PTR pcsvData, DWORD)
{
	static int frameCount = 0;
	std::vector<navihelpers::track_info>& trackingFrames = *(std::vector<navihelpers::track_info>*)pcsvData;
	int totalFrames = (int)trackingFrames.size();
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

	UpdateTrackInfo2Scene(trackingFrames[frame]);

	Render();
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

	// Initialize global strings
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_SCALIBRATORWIN, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // Perform application initialization:
    if (!InitInstance (hInstance, nCmdShow))
    {
        return FALSE;
    }

	SceneInit();

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
    switch (message)
    {
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            // Parse the menu selections:
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
    case WM_PAINT:
        {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);
            // TODO: Add any drawing code that uses hdc here...
            EndPaint(hWnd, &ps);
        }
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
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
