// SNaviWin.cpp : 애플리케이션에 대한 진입점을 정의합니다.
// DOJO : 자체적으로 개발한 namespace
// 엔진 : vzm, vzmutils, vzmproc
// 어플리케이션 helper : navihelpers
// 트래킹 APIs (Optirack APIs Wrapper) : optitrk 
// 위의 namespace 외엔 glm 과 같은 commonly-used 3rd party lib 이거나 C++ standard lib 또는 windows SDK lib 임

#include "GlobalParams.h"

#include "framework.h"
#include "SNaviWin.h"

#include "NetworkTask.h"
#include "TrackingTask.h"
#include "RenderingTask.h"
#include "../optitrk/optitrk.h"

#include <opencv2/opencv.hpp>

#include <regex>
#include <vector>
#include <windowsx.h>
#include <iostream>
#include <stdio.h>
#include <ctime>

#include <objidl.h>
#include <gdiplus.h>
#include <gdipluspath.h>
#include <gdiplusgraphics.h>

using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")

// SpineProjects
#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#include "naviHelpers.hpp"
#include "rapidcsv/rapidcsv.h"
#include "CArmCalibration.h"

#define DESIRED_SCREEN_W 950
#define DESIRED_SCREEN_H 950
#define USE_WHND true
#define RECODE_MODE

#define MAX_LOADSTRING 100
typedef unsigned long long u64;

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

cv::Mat g_curScanGrayImg(SCANIMG_W, SCANIMG_H, CV_8UC1);

__GC __gc;

auto ___LoadMyVariable = [](const std::string& pname, const int idx) {
	cv::Mat ocvVec3(1, 3, CV_32FC1);
	cv::FileStorage fs(__gc.g_folder_trackingInfo + "my_test_variables.txt", cv::FileStorage::Mode::READ);
	fs[pname] >> ocvVec3;
	glm::fvec3 p1;
	memcpy(&p1, ocvVec3.ptr(), sizeof(float) * 3);
	fs.release();

	return ((float*)&p1)[idx];
};

// DOJO : 스캔 시 정보를 저장할 파일
// 현 버전에서는 carm_intrinsics.txt (presetting) 에 저장된 intrinsic parameter 와
// rb2carm1.txt (presetting + 매 스캔 시 저장됨) 에 저장된 c-arm source pose (extrinsic parameter) 를 로드하여 이를 파일로 저장
// TODO : 추후 해당 부분을 file to file 이 아닌, memory to memory 로 관리해야 함
// TODO : 매 스캔마다 camera intrinsics 와 extrinsics 를 계산하도록 수정해야 함
auto storeParams = [](const cv::Mat downloadedGrayImg, const std::string& paramsFileName, const glm::fmat4x4& matCArmRB2WS, const std::string& imgFileName) {

	if (downloadedGrayImg.empty())
	{
		std::cout << "\nno downloaded image" << std::endl;
		return;
	}

	// now g_curScanImg is valid
	// test3.png is for extrinsic calibration image
	cv::Mat downloadColorImg;
	cv::cvtColor(downloadedGrayImg, downloadColorImg, cv::COLOR_GRAY2RGB);
	cv::imwrite(imgFileName, downloadColorImg);

	cv::FileStorage __fs(__gc.g_folder_trackingInfo + paramsFileName, cv::FileStorage::Mode::WRITE);

	cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
	cv::Mat matK;
	fs["K"] >> matK;
	__fs << "K" << matK;
	cv::Mat distCoeffs;
	fs["DistCoeffs"] >> distCoeffs;
	__fs << "DistCoeffs" << distCoeffs;

	fs.open(__gc.g_folder_trackingInfo + "rb2carm1.txt", cv::FileStorage::Mode::READ);
	cv::Mat rvec, tvec;
	fs["rvec"] >> rvec;
	fs["tvec"] >> tvec;
	__fs << "rvec" << rvec;
	__fs << "tvec" << tvec;
	fs.release();

	cv::Mat ocvRb(4, 4, CV_32FC1);
	memcpy(ocvRb.ptr(), glm::value_ptr(matCArmRB2WS), sizeof(float) * 16);
	__fs << "rb2wsMat" << ocvRb;
	__fs << "imgFile" << imgFileName;
	__fs.release();
};


// DOJO : 매 스캔 시 호출 (c-arm 스캔 정보를 scene 에 적용), network thread 에서 호출되야 함
// AP 인지 LATERAL 인지 자동으로 확인하고, 1 이나 2번 할당하는 작업 필요
// note : this function needs to be called in the render thread
auto saveAndChangeViewState = [](const track_info& trackInfo, const int keyParam, 
	const int sidScene, const int cidCam, const int viewIdx) 
{
	using namespace std;
	char LOADKEYS[10] = { '0' , '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	for (int i = 0; i < 10; i++) {
		if (keyParam != LOADKEYS[i]) continue;

		track_info* trk = (track_info*)&trackInfo;
		glm::fquat q;
		glm::fvec3 t;
		if (!trk->GetRigidBodyQuatTVecByName("c-arm", &q, &t)) {
			cout << "\nfailure to get c-arm rigid body" << endl;
			return false;
		}

		glm::fmat4x4 mat_r = glm::toMat4(q);
		glm::fmat4x4 mat_t = glm::translate(t);
		glm::fmat4x4 matRB2WS = mat_t * mat_r;

		if (__gc.g_downloadCompleted > 0) {
			cv::Mat downloadColoredImg;
			cv::cvtColor(g_curScanGrayImg, downloadColoredImg, cv::COLOR_GRAY2RGB);
			string downloadImgFileName = __gc.g_folder_trackingInfo + "test" + to_string(i) + ".png";
			cv::imwrite(downloadImgFileName, downloadColoredImg);

			storeParams(g_curScanGrayImg, "test" + to_string(i) + ".txt", matRB2WS, downloadImgFileName);
			std::cout << "\nSTORAGE COMPLETED!!!" << std::endl;
		}

		auto it = __gc.g_mapAidGroupCArmCam.find(EXTRINSIC_PHANTOM_IMG_INDEX);
		if (it != __gc.g_mapAidGroupCArmCam.end())
			vzm::RemoveSceneItem(it->second, true);

		// load case
		it = __gc.g_mapAidGroupCArmCam.find(i);
		if (it != __gc.g_mapAidGroupCArmCam.end())
			vzm::RemoveSceneItem(it->second, true);
		int aidGroup = calibtask::RegisterCArmImage(sidScene, __gc.g_folder_trackingInfo + "test" + to_string(i) + ".txt", "test" + to_string(i));
		if (aidGroup != -1) {
			__gc.g_mapAidGroupCArmCam[i] = aidGroup;

			// DOJO : 스캔 시 (carmIdx) 저장된 이미지(이미지 texture 가 포함된 plane actor)에 fitting 되도록 cpInterCams (by slerp) 지정 
			// slerp 의 시작점은 현재 cidCam 카메라의 pose, 끝점은 스캔 시 (carmIdx) 저장된 이미지 fitting 카메라 pose
			// 매 스캔 시에 호출되야 하는 SaveAndChangeViewState() 에서 호출함
			cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
			cv::Mat cameraMatrix;
			fs["K"] >> cameraMatrix;
			fs.release();

			int aidCArmPlane = vzmutils::GetSceneItemIdByName("CArm Plane:test" + std::to_string(i));
			if (aidCArmPlane == 0)
				return false;

			vzm::ActorParameters apCArmPlane;
			vzm::GetActorParams(aidCArmPlane, apCArmPlane);
			glm::fmat4x4 matCA2WS = apCArmPlane.script_params.GetParam("matCA2WS", glm::fmat4x4(1));

			rendertask::SetAnimationCamTo(cidCam, viewIdx,
				cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(1, 1), cameraMatrix.at<double>(0, 1),
				cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2), matCA2WS);
		}
		break;
	}
	return true;
};


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
		//StretchBlt(hdc, 0, 0, w, h, hdc, w - 1, 0, -w, h, SRCCOPY);

		//GraphicsPostDrawing(hdc, cidCam);
		DeleteDC(src); // Deleting temp HDC
		DeleteObject(map);

		EndPaint(hWnd, &ps);
	}
};

// DOJO: windows SDK 에서 호출되는 SetTimer 의 callback 함수
// 여기에서 UpdateTrackInfo2Scene() 과 Render() 이 호출된다.
// timer 가 main thread 에서 활성화되므로 rendering thread (timer)는 main thread 에 종속
// (thread safe 관계)
void CALLBACK TimerProc(HWND, UINT, UINT_PTR pcsvData, DWORD)
{
	// DOJO: concurrent queue 에서 가장 최신에 저장된 tracking frame 정보 pop out
	track_info trackInfo;

	//for smaller overhead
	__gc.g_track_que.wait_and_pop(trackInfo);

	if (__gc.g_renderEvent == RENDER_THREAD_CALIBRATION) {
		memcpy(g_curScanGrayImg.ptr(), &__gc.g_downloadImgBuffer[0], __gc.g_downloadImgBuffer.size());

		cv::Mat downloadColorImg;
		cv::cvtColor(g_curScanGrayImg, downloadColorImg, cv::COLOR_GRAY2RGB);
		cv::imwrite(__gc.g_folder_trackingInfo + "test_downloaded.png", downloadColorImg);

		if (__gc.g_selectedMkNames.size() == 4) {
			if (calibtask::CalibrationWithPhantom(g_curScanGrayImg, &trackInfo, __gc.g_useGlobalPairs)) {
				int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
				int cidCam1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
				saveAndChangeViewState(trackInfo, char('3'), sidScene, cidCam1, 0);
				__gc.g_ui_banishing_count = 100;
			}

			__gc.g_calribmodeToggle = false;
		}
		__gc.g_renderEvent = RENDER_THREAD_FREE;
	}

	//track_info* ptrackInfo = NULL;
	//if (!__gc.g_track_que.empty()) {
	//	trackInfo = __gc.g_track_que.front();
	//	ptrackInfo = &trackInfo;
	//}

	rendertask::RenderTrackingScene(&trackInfo);

	// DOJO : windows SDK 에서 화면 업데이트 (QT 방식은 UpdateBMP 참조)
	// USE_WHND 가 false 로 되어 있어야 함
	int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
	int cidCam2 = vzmutils::GetSceneItemIdByName(__gc.g_camName2);
	if (USE_WHND) {
		vzm::PresentHWND(g_hWnd);
		vzm::PresentHWND(g_hWndDialog1);
	}
	else {
		UpdateBMP(cidCam1, g_hWnd);
		UpdateBMP(cidCam2, g_hWndDialog1);
		InvalidateRect(g_hWnd, NULL, FALSE);
		InvalidateRect(g_hWndDialog1, NULL, FALSE);
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

	__gc.Init();
	__gc.g_folder_data = getdatapath();
	__gc.g_folder_trackingInfo = getdatapath() + "Tracking 2023-08-09/";
	__gc.g_profileFileName = __gc.g_folder_data + "Motive Profile - 2023-08-08.motive";

	rendertask::SetGlobalContainer(&__gc);
	nettask::SetGlobalContainer(&__gc);
	trackingtask::SetGlobalContainer(&__gc);
	calibtask::SetGlobalContainer(&__gc);

	//{
		cv::Mat __curScanImg = cv::imread(__gc.g_folder_trackingInfo + "test_downloaded.png");
		if (!__curScanImg.empty()) {

			int chs = __curScanImg.channels();
			if (__curScanImg.channels() == 3)
				cv::cvtColor(__curScanImg, g_curScanGrayImg, cv::COLOR_BGR2GRAY);
			else if (__curScanImg.channels() == 1)
				g_curScanGrayImg = __curScanImg;

			cv::Mat imgGray;
			cv::flip(__curScanImg, imgGray, 1);

			cv::bitwise_not(imgGray, imgGray);

			cv::GaussianBlur(imgGray, imgGray, cv::Size(15, 15), 0);
			
			//Add mask
			//cv::Mat imgSrc = imgGray.clone(); // gray image(=imgSrc)로 circle detect 할 것.
			// Blob Detector Params
			cv::SimpleBlobDetector::Params params;
			params.filterByArea = true;
			params.filterByCircularity = true;
			params.filterByConvexity = false;
			params.filterByInertia = true;
			params.filterByColor = false;
			params.blobColor = 0;

			//params.minArea = 500; // The size of the blob filter to be applied.If the corresponding value is increased, small circles are not detected.
			//params.maxArea = 1000;
			//params.minCircularity = 0.01; // 1 >> it detects perfect circle.Minimum size of center angle

			params.minArea = 2000; // The size of the blob filter to be applied.If the corresponding value is increased, small circles are not detected.
			params.maxArea = 8000;
			params.minCircularity = 0.3; // 1 >> it detects perfect circle.Minimum size of center angle

			params.minInertiaRatio = 0.1; // 1 >> it detects perfect circle. short / long axis
			params.minRepeatability = 2;
			params.minDistBetweenBlobs = 100;

			cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
			std::vector<cv::KeyPoint> keypoints;
			detector->detect(imgGray, keypoints); // circle detect.

			// 원을 감지했는지 확인.

			cv::Mat img = __curScanImg.clone();
			cv::flip(img, img, 1);

			int r = 0;
			for (const auto& keypoint : keypoints) {
				int x = cvRound(keypoint.pt.x);
				int y = cvRound(keypoint.pt.y);
				int s = cvRound(keypoint.size);
				r = cvRound(s / 2);
				cv::circle(img, cv::Point(x, y), r, cv::Scalar(0, 0, 256), 2);
				std::string text = " (" + std::to_string(x) + ", " + std::to_string(y) + ")";
				cv::putText(img, text, cv::Point(x - 25, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
			}
			cv::imwrite(__gc.g_folder_trackingInfo + "test_circle_detect.png", img); // cv imshow()로 보이기.

			std::vector<cv::Point2f> points2d;
			//mystudents::Get2DPostionsFromLegoPhantom(__curScanImg, points2d);
			int gg = 0;
		}
	//}

	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	// engine initialization to use the core APIs of framework
	// must be paired with DeinitEngineLib()
	vzm::InitEngineLib("SpineNavi");
	vzm::SetLogConfiguration(true, 4);

    bool optitrkMode = optitrk::InitOptiTrackLib();

	optitrk::LoadProfileAndCalibInfo(__gc.g_profileFileName, "");
	//optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-08-06.motive", folder_data + "System Calibration.cal");
	//optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-07-04.motive", folder_data + "System Calibration.cal");

	optitrk::SetCameraSettings(0, 4, 16, 230);
	optitrk::SetCameraSettings(1, 4, 16, 230);
	optitrk::SetCameraSettings(2, 4, 16, 230);

    // 전역 문자열을 초기화합니다.
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_SNAVIWIN, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // 애플리케이션 초기화를 수행합니다:
    if (!InitInstance (hInstance, nCmdShow))
    {
        return 0;
    }

	RECT rcView1, rcView2;
	GetClientRect(g_hWnd, &rcView1);
	GetClientRect(g_hWndDialog1, &rcView2);

	rendertask::SceneInit(USE_WHND ? g_hWnd : NULL, rcView1, USE_WHND ? g_hWndDialog1 : NULL, rcView2);

	std::thread tracker_processing_thread(trackingtask::OptiTrackingProcess);
	std::thread network_processing_thread(nettask::NetworkProcess);

	SetTimer(g_hWnd, NULL, 10, TimerProc);

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


	__gc.g_tracker_alive = false; // make the thread finishes, this setting should be located right before the thread join
	tracker_processing_thread.join();
	__gc.g_network_alive = false;
	network_processing_thread.join();

	optitrk::DeinitOptiTrackLib();
	vzm::DeinitEngineLib();

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
//
//        주 프로그램 창을 만든 다음 표시합니다.
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // 인스턴스 핸들을 전역 변수에 저장합니다.

   //WND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	//   CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);
   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	   -1915, 5, DESIRED_SCREEN_W, DESIRED_SCREEN_H, nullptr, nullptr, hInstance, nullptr); //-1915
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

// DOJO IMPORTANT NOTE :
// key 1 ~ 9 : save and add the scan geometry to the scene (global key event)
// key S : save current camera state (main window only)
// key L : load the latest-stored camera state (main window only)
// key C : calibration mode for selecting markers whose position is used to compute c-arm extrinsic calibration
// key U : update c-arm marker set
// key X : compute c-arm extrinsics based on the c-arm marker set and the selected marker positions

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
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
		if (__gc.g_optiEvent != OPTTRK_THREAD_FREE 
			|| __gc.g_renderEvent != RENDER_THREAD_FREE
			|| __gc.g_networkEvent != NETWORK_THREAD_FREE) break;

		using namespace std;
		switch (wParam) {
			case char('S') :
			{
				if (__gc.g_calribmodeToggle) break;

				cv::FileStorage fs(__gc.g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::WRITE);
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
				if (__gc.g_calribmodeToggle) break;

				cv::FileStorage fs(__gc.g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
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
			case char('R') :
			{
				// recode mode.
				break;
			}
			case char('C') :
			{
				__gc.g_calribmodeToggle = !__gc.g_calribmodeToggle;

				vzm::TextItem textItem;
				textItem.textStr = __gc.g_calribmodeToggle? "SELECTING MARKER MODE" : "";
				textItem.fontSize = 30.f;
				textItem.iColor = 0xFFFF00;
				textItem.posScreenX = 0;
				textItem.posScreenY = 0;

				if (!__gc.g_calribmodeToggle) {
					__gc.g_testMKs.clear();
					__gc.g_selectedMkNames.clear();
					int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
					if (aidTestGroup != 0)
						vzm::RemoveSceneItem(aidTestGroup);
				}

				cout << "\n" << "calibration mode " << (__gc.g_calribmodeToggle ? "ON" : "OFF") << endl;

				using namespace glm;
				fmat4x4 matCam2WS;
				optitrk::GetCameraLocation(0, __FP matCam2WS);

				*(fvec3*)cpCam1.pos = vzmutils::transformPos(fvec3(0, 0, 0), matCam2WS);
				*(fvec3*)cpCam1.view = vzmutils::transformVec(fvec3(0, 0, -1), matCam2WS);
				*(fvec3*)cpCam1.up = vzmutils::transformVec(fvec3(0, 1, 0), matCam2WS);
				cpCam1.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_FOV;
				cpCam1.fov_y = glm::pi<float>() / 2.f;
				cpCam1.aspect_ratio = cpCam1.ip_w / cpCam1.ip_h;
				cpCam1.np = 0.15f;

				cpCam1.text_items.SetParam("OPERATION_MODE", textItem);

				vzm::SetCameraParams(cidCam1, cpCam1);
				break;
			}
			case char('U') :
			{
				if (!__gc.g_calribmodeToggle)
					return 0;
				// to do
				// update "c-arm" RB
				int numSelectedMKs = (int)__gc.g_selectedMkNames.size();
				if (numSelectedMKs < 3) {
					cout << "\n" << "at least 3 points are needed to register a rigid body!! current # of points : " << numSelectedMKs << endl;
					return 0;
				}

				cout << "\n" << "rigidbody for c-arm is registered with " << numSelectedMKs << " points" << endl;

				__gc.g_optiEvent = OPTTRK_THREAD_C_ARM_REGISTER;

				while (__gc.g_optiEvent == OPTTRK_THREAD_C_ARM_REGISTER) { Sleep(2); }

				// int aidRbCarm = vzmutils::GetSceneItemIdByName("c-arm");
				// vzm::RemoveSceneItem(aidRbCarm);
				// 이 떄마다, "c-arm" 관련 scene item 모두 지우고, 해당 element 지우기..
				// UpdateTrackInfo2Scene 에서 rigid boby name 의 cid 를 확인하여, 다를 경우 (새로 생성, rbmk 개수 변경, 경우) 기존 것 삭제하고 actor group 재생성
				break;
			}
			case char('D') : {
				for (int i = 0; i < 9;  i++) {
					auto it = __gc.g_mapAidGroupCArmCam.find(i + 1);
					if (it != __gc.g_mapAidGroupCArmCam.end())
						vzm::RemoveSceneItem(it->second, true);
				}
				__gc.g_mapAidGroupCArmCam.clear();

				__gc.g_testMKs.clear();
				__gc.g_selectedMkNames.clear();
				int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
				if (aidTestGroup != 0)
					vzm::RemoveSceneItem(aidTestGroup);
				break;
			}
			case char('Y') : {
				track_info trk;
				trk = __gc.g_track_que.front();
				glm::fmat4x4 matRB2WS;
				if (trk.GetRigidBodyByName("probe", &matRB2WS, NULL, NULL, NULL)) {
					glm::fvec3 p = vzmutils::transformPos(glm::fvec3(0, 0, 0), matRB2WS);

					//if (trk.GetRigidBodyByName("c-arm", &matRB2WS, NULL, NULL, NULL)) 
					{

						//glm::fmat4x4 matWS2RB = glm::inverse(matRB2WS);
						//p = vzmutils::transformPos(p, matWS2RB);
						cv::Mat ocvVec3(1, 3, CV_32FC1);
						memcpy(ocvVec3.ptr(), &p, sizeof(float) * 3);
						cv::FileStorage fs(__gc.g_folder_trackingInfo + "test_tip.txt", cv::FileStorage::Mode::WRITE);
						fs << "probe_tip" << ocvVec3;
						fs.release();

						int idx = (int)___LoadMyVariable("test0", 0);

						cv::FileStorage __fs(__gc.g_folder_trackingInfo + "test_tip_sphere.txt", cv::FileStorage::Mode::READ);
						__fs["inter_sphere_" + std::to_string(idx)] >> ocvVec3;
						glm::fvec3 p1;
						memcpy(&p1, ocvVec3.ptr(), sizeof(float) * 3);
						__fs.release();

						float r = glm::length(p1 - p);
						std::cout << "\ntest : " << r << std::endl;
					}

				}
				break;
			}
			case char('T') : {
				if (!__gc.g_calribmodeToggle) {
					cout << "\n" << "not calibration mode!!" << endl;
					return 0;
				}
				if (__gc.g_selectedMkNames.size() < 4) {
					cout << "\n" << "at least 4 points (last one is for tool tip) are needed!!" << endl;
					return 0;
				}

				// 툴 등록하기..
				__gc.g_optiEvent = OPTTRK_THREAD_TOOL_REGISTER;
				while (__gc.g_optiEvent != OPTTRK_THREAD_FREE) { Sleep(2); }
				__gc.g_testMKs.clear();
				__gc.g_testMKs.push_back(glm::fvec3(0));
				__gc.g_testMKs.push_back(glm::fvec3(0));
				__gc.g_testMKs.push_back(glm::fvec3(0));
				break;
			}
			//case char('E') : {
			//	if (!calribmodeToggle) {
			//		cout << "\n" << "not calibration mode!!" << endl;
			//		return 0;
			//	}
			//	// 툴 끝점 등록하기..
			//	break;
			//}
			case char('G') : {
				__gc.g_useGlobalPairs != !__gc.g_calribmodeToggle;
				break;
			}
			case char('X') :
			{
				if (!__gc.g_calribmodeToggle) {
					cout << "\n" << "not calibration mode!!" << endl;
					return 0;
				}
				//#define MYDEFINE
				g_curScanGrayImg = cv::imread(__gc.g_folder_trackingInfo + "test" + to_string(EXTRINSIC_PHANTOM_IMG_INDEX) + ".png");
				if (g_curScanGrayImg.empty()) {
					cout << "\n" << "no test3 image" << endl;
					return false;
				}
				__gc.g_downloadCompleted = 100;
				cv::cvtColor(g_curScanGrayImg, g_curScanGrayImg, cv::COLOR_RGB2GRAY);
				cout << "\n" << "calibration with the previous test2 image" << endl;

				track_info trk;
				//__gc.g_track_que.wait_and_pop(trk);
				trk = __gc.g_track_que.front();
				calibtask::CalibrationWithPhantom(g_curScanGrayImg, &trk, __gc.g_useGlobalPairs);

				//download_completed = false; 
				saveAndChangeViewState(trk, char('3'), sidScene, cidCam1, 0);
				__gc.g_ui_banishing_count = 100;
				break;
			}
			default: {
				track_info trk;
				//__gc.g_track_que.wait_and_pop(trk);
				trk = __gc.g_track_que.front();
				saveAndChangeViewState(trk, wParam, sidScene, cidCam1, 0);
				break;
			}
		}

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
		if (__gc.g_optiEvent == OPTTRK_THREAD_TOOL_REGISTER) break;
		int x = GET_X_LPARAM(lParam);
		int y = GET_Y_LPARAM(lParam);
		if (x == 0 && y == 0) break;

		// renderer does not need to be called
		glm::ivec2 pos_ss = glm::ivec2(x, y);

		if (__gc.g_calribmodeToggle) {
			if (message == WM_LBUTTONDOWN) {
				int aidPicked = 0;
				glm::fvec3 posPick;
				if (vzm::PickActor(aidPicked, __FP posPick, x, y, cidCam1)) {
					std::string actorName;
					vzm::GetSceneItemName(aidPicked, actorName);
					if (__gc.g_selectedMkNames.find(actorName) == __gc.g_selectedMkNames.end()) {
						int idx = __gc.g_selectedMkNames.size();
						__gc.g_selectedMkNames[actorName] = idx;
						std::cout << "\npicking count : " << __gc.g_selectedMkNames.size() << std::endl;
					}
				}
			}
			else {
				__gc.g_selectedMkNames.clear();
			}
		}
		else {
			general_move.Start((int*)&pos_ss, cpCam1, scene_stage_center, scene_stage_scale);
		}
		break;
	}
	case WM_MOUSEMOVE:
	{
		if (__gc.g_calribmodeToggle && (wParam & MK_LBUTTON)) break;

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
				//std::cout << "\n" <<mkName << "is picked : " << ", Pos : " << posMK.x << ", " << posMK.y << ", " << posMK.z << std::endl;

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
	int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	int cidCam2 = vzmutils::GetSceneItemIdByName(__gc.g_camName2);
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
		track_info trk;
		__gc.g_track_que.wait_and_pop(trk);
		saveAndChangeViewState(trk, wParam, sidScene, cidCam2, 1);
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
				std::cout << "\n" <<"picking index : " << idx << std::endl;

				std::cout << "\n" <<"excluded index list : ";
				for (auto it : excludedMKIndices) {
					std::cout << "\n" <<it << ", ";
				}
				std::cout << "\n" <<std::endl;

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
