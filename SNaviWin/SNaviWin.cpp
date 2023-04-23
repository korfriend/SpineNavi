// SNaviWin.cpp : 애플리케이션에 대한 진입점을 정의합니다.
//

#include "framework.h"
#include "SNaviWin.h"

#include <vector>
#include <windowsx.h>
#include <iostream>

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

//#define USE_MOTIVE
#define USE_WHND true

#define MAX_LOADSTRING 100

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

void UpdateBuffer(int cam_id, HWND _hwnd, bool updateHwnd)
{
	auto update_bmp = [](int cam_id, HWND hwnd__) {
		unsigned char* ptr_rgba;
		float* ptr_zdepth;
		int w, h;
		if (vzm::GetRenderBufferPtrs(cam_id, &ptr_rgba, &ptr_zdepth, &w, &h))
		{
			// https://stackoverflow.com/questions/26005744/how-to-display-pixels-on-screen-directly-from-a-raw-array-of-rgb-values-faster-t
			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hwnd__, &ps);
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

			DeleteObject(map);
			DeleteDC(src); // Deleting temp HDC
			EndPaint(hwnd__, &ps);
			InvalidateRect(hwnd__, NULL, FALSE);
		}
	};

	if (updateHwnd)
		vzm::PresentHWND(_hwnd);
	else
		update_bmp(cam_id, _hwnd);
}

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

	vzm::ActorParameters apGrid, apLines, apTextX, apTextZ;
	apGrid.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidGrid);
	apLines.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidLines);
	apTextX.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidTextX);
	apTextZ.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidTextZ);

	int aidGrid, aidLines, aidTextX, aidTextZ;
	vzm::NewActor(apGrid, "World Grid", aidGrid);
	vzm::NewActor(apLines, "World Lines", aidLines);
	vzm::NewActor(apTextX, "World Text X", aidTextX);
	vzm::NewActor(apTextZ, "World Text Y", aidTextZ);

	RECT rcWorldView;
	GetClientRect(g_hWnd, &rcWorldView);

	vzm::CameraParameters cpCam1;
	*(glm::fvec3*)cpCam1.pos = glm::fvec3(3, 3, 3);
	*(glm::fvec3*)cpCam1.up = glm::fvec3(0, 1, 0);
	*(glm::fvec3*)cpCam1.view = glm::fvec3(-1, -1, -1);
	cpCam1.np = 0.01f;
	cpCam1.fp = 100.f;
	cpCam1.fov_y = 3.141592654f / 4.f;
	cpCam1.projection_mode = vzm::CameraParameters::CAMERA_FOV;
	cpCam1.w = rcWorldView.right - rcWorldView.left;
	cpCam1.h = rcWorldView.bottom - rcWorldView.top;
	cpCam1.SetOrthogonalProjection(true);
	cpCam1.hWnd = USE_WHND ? g_hWnd : NULL;
	cpCam1.aspect_ratio = (float)cpCam1.w / (float)cpCam1.h;
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
}

void Render() {
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	int cidCam1 = vzmutils::GetSceneItemIdByName("World Camera");

	if (sidScene != 0 && cidCam1 != 0) {
		vzm::RenderScene(sidScene, cidCam1);
		UpdateBuffer(cidCam1, g_hWnd, true);
	}
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

	// engine initialization to use the core APIs of framework
	// must be paired with DeinitEngineLib()
	vzm::InitEngineLib("SpineNavi");
	vzm::SetLogConfiguration(true, 4);
#ifdef USE_MOTIVE
    bool optitrkMode = optitrk::InitOptiTrackLib();
#else
	rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.12.57 PM.csv", rapidcsv::LabelParams(0, 0));
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

#define NUM_RBS 3
	static std::string _rb_names[NUM_RBS] = { "c-arm" , "test1", "probe" };

#ifdef USE_MOTIVE
	const int postpone = 3;
	navihelpers::concurrent_queue<navihelpers::track_info> track_que(10);
    std::atomic_bool tracker_alive{true};
	std::thread tracker_processing_thread([&]() {
		while (tracker_alive)
		{
			Sleep(postpone);
			optitrk::UpdateFrame();

			navihelpers::track_info cur_trk_info;
			for (int i = 0; i < NUM_RBS; i++)
			{
				glm::fmat4x4 mat_lfrm2ws;
				bool is_detected = optitrk::GetRigidBodyLocationByName(_rb_names[i], (float*)&mat_lfrm2ws);
				cur_trk_info.SetLFrmInfo(_rb_names[i], is_detected, mat_lfrm2ws);
			}

			//cout << cur_is_rsrb_detected << ", " << cur_is_probe_detected << endl;

			optitrk::GetMarkersLocation(&cur_trk_info.mk_xyz_list, &cur_trk_info.mk_residue_list, &cur_trk_info.mk_cid_list);
			cur_trk_info.is_updated = true;
			track_que.push(cur_trk_info);
		}
	});
#else
	// trackingData
	const int startRowIdx = 6;
	const int startColIdx = 1;

	std::vector<std::string> rowTypes = trackingData.GetRow<std::string>(1);
	std::vector<std::string> rowNames = trackingData.GetRow<std::string>(2);
	std::vector<std::string> rowIds = trackingData.GetRow<std::string>(3);
	std::vector<std::string> rotPosLabels = trackingData.GetRow<std::string>(4);
	std::vector<std::string> xyzwLabels = trackingData.GetRow<std::string>(5);
	const int numCols = (int)rowTypes.size();//trackingData.GetColumnCount();
	const int numRows = (int)trackingData.GetRowCount();

	std::vector<navihelpers::track_info2> trackingFrames(numRows - startRowIdx);
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


	for (int rowIdx = startRowIdx; rowIdx < numRows; rowIdx++) {
		std::vector<float> rowValues = trackingData.GetRow<float>(rowIdx);
		navihelpers::track_info2& trackInfo = trackingFrames[frameIdx++];

		int colIdx = startColIdx;
		while (colIdx < numCols) {
			std::string type = rowTypes[colIdx];
			std::string name = rowNames[colIdx];
			std::string id = rowIds[colIdx];
			std::bitset<128> cid = string2cid(id);
			std::string rotPos = rotPosLabels[colIdx];
			std::string xyzw = xyzwLabels[colIdx];

			std::map<std::string, std::map<navihelpers::MKINFO, std::any>> rbmkSet;

			if (type == "Rigid Body") {
				// read 8 col-successive cells

				glm::fquat q(rowValues[colIdx + 3], rowValues[colIdx], rowValues[colIdx + 1], rowValues[colIdx + 2]);
				glm::fvec3 t(rowValues[colIdx + 4], rowValues[colIdx + 5], rowValues[colIdx + 6]);
				float mkMSE = rowValues[colIdx + 7];

				glm::fmat4x4 mat_r = glm::toMat4(q);
				glm::fmat4x4 mat_t = glm::translate(t);
				glm::fmat4x4 mat_ls2ws = mat_t * mat_r;

				colIdx += 8;

				for (; colIdx < numCols; colIdx+=4) {
					std::string _type = rowTypes[colIdx];
					if (_type != "Rigid Body Marker") break;
					// read 4 col-successive cells
					std::string _name = rowNames[colIdx];
					std::string _id = rowIds[colIdx];
					std::bitset<128> _cid = string2cid(_id);

					glm::fvec3 p(rowValues[colIdx], rowValues[colIdx + 1], rowValues[colIdx + 2]);
					float mq = rowValues[colIdx + 3];
					auto& v = rbmkSet[_name];
					v[navihelpers::MKINFO::POSITION] = p;
					v[navihelpers::MKINFO::MK_QUALITY] = mq;
					v[navihelpers::MKINFO::MK_NAME] = _name;
					v[navihelpers::MKINFO::CID] = _cid;
				}

				trackInfo.AddRigidBody(name, mat_ls2ws, mkMSE, rbmkSet);
			}
			if (type == "Marker") {
				// read 3 col-successive cells
				glm::fvec3 p(rowValues[colIdx], rowValues[colIdx + 1], rowValues[colIdx + 2]);
				trackInfo.AddMarker(cid, p, name);
				colIdx += 3;
			}
		}
	}
		
#endif

#ifdef _EVENT_DRIVEN
    MSG msg;

    // 기본 메시지 루프입니다:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
	}
#else
	MSG msg = { 0 };
	while (WM_QUIT != msg.message)
	{
#ifdef USE_MOTIVE
		navihelpers::track_info trk_info;
		track_que.wait_and_pop(trk_info);
#endif
		if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			Render();
		}
	}
#endif

#ifdef USE_MOTIVE
	tracker_alive = false; // make the thread finishes, this setting should be located right before the thread join
	tracker_processing_thread.join();
	optitrk::DeinitOptiTrackLib();
#endif
	vzm::DeinitEngineLib();

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

   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);
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
	vzm::CameraParameters cpCam1;
	if (sidScene != 0 && cidCam1 != 0) {
		vzm::GetCameraParams(cidCam1, cpCam1);
	}

	static glm::fvec3 scene_stage_center = glm::fvec3();
	static float scene_stage_scale = 5.f;
	static vzmutils::GeneralMove general_move;

    switch (message)
    {
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
    case WM_PAINT:
        {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);
            // TODO: 여기에 hdc를 사용하는 그리기 코드를 추가합니다...
			EndPaint(hWnd, &ps);
			//UpdateBuffer(cidCam1, hWnd, USE_WHND);
			break;
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

			//vzm::RenderScene(sidScene, cidCam1);
			//UpdateWindow(hWnd);
			//InvalidateRect(hWnd, NULL, FALSE);
		}
		break;
	}
	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
		break;
	case WM_MOUSEWHEEL:
	{
		int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
		vzmutils::ZoomImageplane(zDelta > 0, cpCam1.ip_w, cpCam1.ip_h);

		vzm::SetCameraParams(cidCam1, cpCam1);
		//vzm::RenderScene(sidScene, cidCam1);

		//InvalidateRect(hWnd, NULL, FALSE);
		//UpdateWindow(hWnd);
	}
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
