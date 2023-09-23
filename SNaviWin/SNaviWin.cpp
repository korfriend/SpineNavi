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

#include <filesystem>


using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")

// SpineProjects
#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#include "naviHelpers.hpp"
#include "rapidcsv/rapidcsv.h"
#include "CArmCalibration.h"

#define DESIRED_SCREEN_W 512
#define DESIRED_SCREEN_H 512
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
	cv::FileStorage fs(__gc.g_folder_trackingInfo + "my_test_variables.txt", cv::FileStorage::Mode::WRITE);
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
auto storeParams = [](const std::string& paramsFileName, const glm::fmat4x4& matCArmRB2WS, const std::string& imgFileName) {

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
/**/


// DOJO : 매 스캔 시 호출 (c-arm 스캔 정보를 scene 에 적용), network thread 에서 호출되야 함
// AP 인지 LATERAL 인지 자동으로 확인하고, 1 이나 2번 할당하는 작업 필요
// note : this function needs to be called in the render thread
auto saveAndChangeViewState = [](const int keyParam, const int sidScene, const int cidCam, const int viewIdx, 
	const glm::fmat4x4* pmatCArmRB2WS, const cv::Mat* pcolored_img)
{
	// note: matCArmRB2WS is used only when a valid pcolored_img is not NULL 
	using namespace std;
	char LOADKEYS[10] = { '0' , '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	for (int i = 0; i < 10; i++) {
		if (keyParam != LOADKEYS[i]) continue;

		//track_info* trk = (track_info*)&trackInfo;
		//glm::fquat q;
		//glm::fvec3 t;
		//if (!trk->GetRigidBodyQuatTVecByName("c-arm", &q, &t)) {
		//	cout << "\nfailure to get c-arm rigid body" << endl;
		//	return false;
		//}
		//
		//glm::fmat4x4 mat_r = glm::toMat4(q);
		//glm::fmat4x4 mat_t = glm::translate(t);
		//glm::fmat4x4 matCArmRB2WS = mat_t * mat_r;

		__gc.g_showCalibMarkers = keyParam == '3';

		string timePack;

		if (pcolored_img)
		{
			const cv::Mat& colored_img = *pcolored_img;
			const glm::fmat4x4& matCArmRB2WS = *pmatCArmRB2WS;
			if (colored_img.empty()) {
				__gc.SetErrorCode("The Image is Empty!");
				return false;
			}

			timePack = to_string(navihelpers::GetCurrentTimePack());

			string downloadImgFileName = __gc.g_folder_trackingInfo + "test" + to_string(i) + "_" + timePack + ".png";
			cv::imwrite(downloadImgFileName, colored_img);

			storeParams("test" + to_string(i) + "_" + timePack + ".txt", matCArmRB2WS, downloadImgFileName);
			std::cout << "\nSTORAGE COMPLETED!!!" << std::endl;

			if (__gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD) {
				if (colored_img.empty()) {
					__gc.SetErrorCode("Not Allowed Calibration Image during Recording");
					__gc.g_optiRecordMode = OPTTRK_RECMODE::NONE;
					return false;
				}
				if (!__gc.g_recScanStream.is_open()) {
					__gc.g_recScanStream.open(__gc.g_recScanName);
					if (!__gc.g_recScanStream.is_open()) {
						__gc.SetErrorCode("Invalid Rec File!");
						__gc.g_optiRecordMode = OPTTRK_RECMODE::NONE;
						return false;
					}

					std::vector<std::vector<std::string>> csvData(1);
					std::vector<std::string>& csvRow = csvData[0];
					csvRow = { "FRAME", "TIME", "AP/LL"};

					// Write CSV data to the file
					for (int j = 0; j < (int)csvData.size(); j++) {
						std::vector<std::string>& csvRow = csvData[j];
						for (int k = 0; k < (int)csvRow.size(); k++) {
							__gc.g_recScanStream << csvRow[k];
							if (k < csvRow.size() - 1) {
								__gc.g_recScanStream << ',';
							}
							else 
								__gc.g_recScanStream << '\n';
						}
					}
				}
				else {

					std::vector<std::string> csvRow = { to_string(__gc.g_optiRecordFrame), timePack, i == 1? "AP" : "LL" };

					// Write CSV data to the file
					for (int j = 0; j < (int)csvRow.size(); j++) {
						__gc.g_recScanStream << csvRow[j];
						if (j < csvRow.size() - 1) {
							__gc.g_recScanStream << ',';
						}
						else 
							__gc.g_recScanStream << '\n';
					}
				}
			} // if (__gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD)
			else {
				if (__gc.g_recScanStream.is_open()) {
					std::cout << "CSV (Scan) data has been written to " << __gc.g_recScanName << std::endl;
					__gc.g_recScanStream.close();
				}
			}
		} // if (pcolored_img)
		else { // // if (pcolored_img == NULL)
			// load case
			using std::filesystem::directory_iterator;

			set<unsigned long long> latestTimePacks;
			for (const auto& file : directory_iterator(__gc.g_folder_trackingInfo)) {
				const std::string ss = file.path().u8string();
				//char* ptr = std::strrchr(file.path().c_str(), '\\');     //문자열(path)의 뒤에서부터 '\'의 위치를 검색하여 반환

				std::filesystem::path filePath = file.path().filename();
				if (filePath.extension() == ".png") {
					std::string fn = filePath.u8string();
					if (fn.find("test" + to_string(i)) != std::string::npos) {
						std::string::size_type filePos = fn.rfind('_');
						//std::cout << file.path().filename().u8string() << std::endl;

						if (filePos != std::string::npos)
							++filePos;
						else
							filePos = 0;
						std::string timePackExt = fn.substr(filePos);
						timePackExt.erase(timePackExt.find_last_of("."), string::npos);
						unsigned long long itimePackExt = std::stoll(timePackExt);
						latestTimePacks.insert(itimePackExt);
					}
				}
			}

			timePack = to_string(*latestTimePacks.rbegin());
		}

		auto it = __gc.g_mapAidGroupCArmCam.find(EXTRINSIC_PHANTOM_IMG_INDEX);
		if (it != __gc.g_mapAidGroupCArmCam.end())
			vzm::RemoveSceneItem(it->second, true);

		// load case
		it = __gc.g_mapAidGroupCArmCam.find(i);
		if (it != __gc.g_mapAidGroupCArmCam.end())
			vzm::RemoveSceneItem(it->second, true);
		int aidGroup = calibtask::RegisterCArmImage(sidScene, 
			__gc.g_folder_trackingInfo + "test" + to_string(i) + "_" + timePack + ".txt", "test" + to_string(i));
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

CRITICAL_SECTION lock;
// DOJO: windows SDK 에서 호출되는 SetTimer 의 callback 함수
// 여기에서 UpdateTrackInfo2Scene() 과 Render() 이 호출된다.
// timer 가 main thread 에서 활성화되므로 rendering thread (timer)는 main thread 에 종속
// (thread safe 관계)
void CALLBACK TimerProc(HWND, UINT, UINT_PTR pcsvData, DWORD)
{
	using namespace std;
	//::EnterCriticalSection(&lock);
	// DOJO: concurrent queue 에서 가장 최신에 저장된 tracking frame 정보 pop out
	track_info trackInfo;

	if (__gc.g_optiRecordFrame > 0)
		__gc.g_optiRecordFrame++;

	if (!__gc.g_track_que.empty()) {
		__gc.g_track_que.wait_and_pop(trackInfo);
	}

	int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
	int cidCam2 = vzmutils::GetSceneItemIdByName(__gc.g_camName2);

	static int recRowCount = -1;
	if (__gc.g_optiRecordMode == OPTTRK_RECMODE::LOAD) {
		if (__gc.g_recScanStream.is_open()) {
			std::cout << "CSV (Scan) data has been written to " << __gc.g_recScanName << std::endl;
			__gc.g_recScanStream.close();
		}

		static rapidcsv::Document csvTrkData;
		if (recRowCount < 0) {
			csvTrkData.Load(__gc.g_recScanName, rapidcsv::LabelParams(0, 0));
			recRowCount = 0;
		}

		int recordFrame = 100000000;
		if (recRowCount >= (int)csvTrkData.GetRowCount()) {
			if (__gc.g_optiRecordFrame <= 2)
				recRowCount = 0;
		}
		else {
			std::string frameStr = csvTrkData.GetRowName(recRowCount);
			recordFrame = std::stoi(frameStr);
		}
		if (__gc.g_optiRecordFrame > recordFrame) {
			std::vector<std::string> rowData = csvTrkData.GetRow<std::string>(recRowCount);
			recRowCount++;

			bool isAP = rowData[1] == "AP";
			unsigned long long timePack = stoull(rowData[0]);

			string scanImgNamePrefix = __gc.g_folder_trackingInfo + "test" + to_string(isAP ? 1 : 2) + "_" + rowData[0];
			if (scanImgNamePrefix.back() == ' ') scanImgNamePrefix.pop_back();
			cv::FileStorage fs(scanImgNamePrefix + ".txt", cv::FileStorage::Mode::READ);
			if (fs.isOpened()) {
				cv::Mat rb2wsMat;
				fs["rb2wsMat"] >> rb2wsMat; // memory storage ordering in opengl
				std::string imgFileName;
				fs["imgFile"] >> imgFileName;
				fs.release();

				glm::fmat4x4 matCArmRB2WS = *(glm::fmat4x4*)rb2wsMat.ptr();
				cv::Mat downloadColorImg = cv::imread(imgFileName);

				if (isAP) {
					saveAndChangeViewState(char('1'), sidScene, cidCam1, 0, &matCArmRB2WS, &downloadColorImg);
				}
				else {
					saveAndChangeViewState(char('2'), sidScene, cidCam2, 1, &matCArmRB2WS, &downloadColorImg);
				}
			}

		}
	}
	else {
		recRowCount = -1;
		if (__gc.g_renderEvent == RENDER_THREAD::DOWNLOAD_IMG_PROCESS) {
			//__gc.g_renderEvent = RENDER_THREAD::BUSY;
			memcpy(g_curScanGrayImg.ptr(), &__gc.g_downloadImgBuffer[0], __gc.g_downloadImgBuffer.size());

			cv::Mat downloadColorImg;
			cv::cvtColor(g_curScanGrayImg, downloadColorImg, cv::COLOR_GRAY2RGB);
			cv::imwrite(__gc.g_folder_trackingInfo + "test_downloaded.png", downloadColorImg);

			glm::fmat4x4 matCArmRB2WS;
			if (trackInfo.GetRigidBodyByName("c-arm", &matCArmRB2WS, NULL, NULL, NULL)) {
				if (__gc.g_calribmodeToggle) {
					if (calibtask::CalibrationWithPhantom(__gc.g_CArmRB2SourceCS, g_curScanGrayImg, &trackInfo, __gc.g_useGlobalPairs)) {

						cv::Mat processColorImg = cv::imread(__gc.g_folder_trackingInfo + "test_circle_sort.png");
						saveAndChangeViewState(char('3'), sidScene, cidCam1, 0, &matCArmRB2WS, &processColorImg);
					}
				}
				else {
					// trackInfo.matCArmRB2WS, trackInfo.matCam2WS, downloadColorImg
					glm::fmat4x4 matSourceCS2WS = matCArmRB2WS * glm::inverse(__gc.g_CArmRB2SourceCS);
					//glm::fvec3 posCArmDet = vzmutils::transformPos(glm::fvec3(0, 0, 0), matCArmRB2WS);
					glm::fvec3 dirCArm = vzmutils::transformVec(glm::fvec3(0, 0, -1), matSourceCS2WS);
					dirCArm = glm::normalize(dirCArm);

					glm::fmat4x4 matCam2WS;
					trackInfo.GetTrackingCamPose(0, matCam2WS);
					glm::fvec3 camRight = vzmutils::transformVec(glm::fvec3(1, 0, 0), matCam2WS);
					camRight = glm::normalize(camRight);

					float horizonAngle = fabs(glm::dot(camRight, dirCArm));
					if (horizonAngle < 0.3f)
						saveAndChangeViewState(char('1'), sidScene, cidCam1, 0, &matCArmRB2WS, &downloadColorImg);
					else
						saveAndChangeViewState(char('2'), sidScene, cidCam2, 1, &matCArmRB2WS, &downloadColorImg);
				}
			}
			else {
				__gc.SetErrorCode("C-Arm is Not Detected!\nCheck the C-Arm Markers and Retry!");
			}

			__gc.g_renderEvent = RENDER_THREAD::FREE;
		}
	}

	//track_info* ptrackInfo = NULL;
	//if (!__gc.g_track_que.empty()) {
	//	trackInfo = __gc.g_track_que.front();
	//	ptrackInfo = &trackInfo;
	//}

	if (trackInfo.IsValidTracking())
		rendertask::RenderTrackingScene(&trackInfo);

	// DOJO : windows SDK 에서 화면 업데이트 (QT 방식은 UpdateBMP 참조)
	// USE_WHND 가 false 로 되어 있어야 함
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
	//::LeaveCriticalSection(&lock);
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
	__gc.g_folder_trackingInfo = getdatapath() + "Tracking 2023-09-15/";
	__gc.g_profileFileName = __gc.g_folder_data + "Motive Profile - 2023-09-15.motive";
	__gc.g_recFileName = __gc.g_folder_trackingInfo + "TrackingRecord.csv";
	__gc.g_recScanName = __gc.g_folder_trackingInfo + "ScanRecord.csv";
	__gc.g_configFileName = __gc.g_folder_trackingInfo + "Config.txt";

	{
		cv::FileStorage __config_fs(__gc.g_configFileName, cv::FileStorage::Mode::WRITE);
		__config_fs << "RecordeMode" << std::string("LOAD");
		__config_fs << "RecordePeriod" << 2;
		__config_fs << "PivotSamples" << 100;
		__config_fs.release();

		cv::FileStorage config_fs(__gc.g_configFileName, cv::FileStorage::Mode::READ);
		std::string configLoad;
		config_fs["RecordeMode"] >> configLoad;
		config_fs["RecordePeriod"] >> __gc.g_optiRecordPeriod;
		config_fs["PivotSamples"] >> __gc.g_optiPivotSamples;
		config_fs.release();

		if (configLoad == "LOAD") {
			__gc.g_optiRecordMode = OPTTRK_RECMODE::LOAD;
		}
	}

	rendertask::InitializeTask(&__gc);
	nettask::InitializeTask(&__gc);
	trackingtask::InitializeTask(&__gc);
	calibtask::InitializeTask(&__gc);

	//{
		cv::Mat __curScanImg = cv::imread(__gc.g_folder_trackingInfo + "test_downloaded.png");
		if (0)//!__curScanImg.empty()) 
		{
			int chs = __curScanImg.channels();
			if (__curScanImg.channels() == 3)
				cv::cvtColor(__curScanImg, g_curScanGrayImg, cv::COLOR_BGR2GRAY);
			else if (__curScanImg.channels() == 1)
				g_curScanGrayImg = __curScanImg;

			cv::Mat imgGray;
			cv::flip(g_curScanGrayImg, imgGray, 1);

			cv::bitwise_not(imgGray, imgGray);

			cv::GaussianBlur(imgGray, imgGray, cv::Size(5, 5), 0);
			cv::imwrite(__gc.g_folder_trackingInfo + "test_butwise.png", imgGray); // cv imshow()로 보이기.
			
			//Add mask
			//cv::Mat imgSrc = imgGray.clone(); // gray image(=imgSrc)로 circle detect 할 것.
			// Blob Detector Params
			cv::SimpleBlobDetector::Params params;
			params.filterByArea = true;
			params.filterByCircularity = true;
			params.filterByConvexity = false;
			params.filterByInertia = true;
			params.filterByColor = false;
			//params.blobColor = 255;
			params.minThreshold = 100;

			//params.minArea = 500; // The size of the blob filter to be applied.If the corresponding value is increased, small circles are not detected.
			//params.maxArea = 1000;
			//params.minCircularity = 0.01; // 1 >> it detects perfect circle.Minimum size of center angle

			params.minArea = 2000; // The size of the blob filter to be applied.If the corresponding value is increased, small circles are not detected.
			params.maxArea = 15000;
			params.minCircularity = 0.3; // 1 >> it detects perfect circle.Minimum size of center angle
			//params.minConvexity = 0.8;
			params.minInertiaRatio = 0.01; // 1 >> it detects perfect circle. short / long axis
			params.minRepeatability = 2;
			params.minDistBetweenBlobs = 50;

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
				cv::circle(img, cv::Point(x, y), r, cv::Scalar(0, 0, 255), 2);
				std::string text = " (" + std::to_string(x) + ", " + std::to_string(y) + ")";
				cv::putText(img, text, cv::Point(x - 25, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
			}
			cv::imwrite(__gc.g_folder_trackingInfo + "test_circle_detect.png", img); // cv imshow()로 보이기.

			std::vector<cv::Point2f> points2d;
			//mystudents::Get2DPostionsFromLegoPhantom(__curScanImg, points2d);
			return 0;
		}
		if (0)//!__curScanImg.empty()) 
		{

			int chs = __curScanImg.channels();
			if (__curScanImg.channels() == 3)
				cv::cvtColor(__curScanImg, g_curScanGrayImg, cv::COLOR_BGR2GRAY);
			else if (__curScanImg.channels() == 1)
				g_curScanGrayImg = __curScanImg;

			cv::Mat imgGray;
			cv::flip(g_curScanGrayImg, imgGray, 1);

			//cv::bitwise_not(imgGray, imgGray);

			cv::GaussianBlur(imgGray, imgGray, cv::Size(15, 15), 0);
			cv::imwrite(__gc.g_folder_trackingInfo + "test_butwise.png", imgGray); // cv imshow()로 보이기.
			 
			std::vector<cv::Point3f> circles;
			cv::HoughCircles(imgGray, circles, cv::HOUGH_GRADIENT, 1,
				200, // change this value to detect circles with different distances to each other
				110, 10, 20, 100 // change the last two parameters
				// (min_radius & max_radius) to detect larger circles
			);

			// 원을 감지했는지 확인.

			cv::Mat img = __curScanImg.clone();
			cv::flip(img, img, 1);

			int r = 0;
			for (const auto& keypoint : circles) {
				int x = cvRound(keypoint.x);
				int y = cvRound(keypoint.y);
				int s = cvRound(keypoint.z);
				r = cvRound(s);
				cv::circle(img, cv::Point(x, y), r, cv::Scalar(0, 0, 255), 2);
				std::string text = " (" + std::to_string(x) + ", " + std::to_string(y) + ")";
				cv::putText(img, text, cv::Point(x - 25, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
			}
			cv::imwrite(__gc.g_folder_trackingInfo + "test_circle_detect.png", img); // cv imshow()로 보이기.

			std::vector<cv::Point2f> points2d;
			//mystudents::Get2DPostionsFromLegoPhantom(__curScanImg, points2d);
			return 0;
		}
	//}

	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	// engine initialization to use the core APIs of framework
	// must be paired with DeinitEngineLib()
	vzm::InitEngineLib("SpineNavi");
	vzm::SetLogConfiguration(true, 4);

	if (__gc.g_optiRecordMode != OPTTRK_RECMODE::LOAD) {
		bool optitrkMode = optitrk::InitOptiTrackLib();
		optitrk::LoadProfileAndCalibInfo(__gc.g_profileFileName, "");

		//optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-08-06.motive", folder_data + "System Calibration.cal");
		//optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-07-04.motive", folder_data + "System Calibration.cal");

		optitrk::SetCameraSettings(0, 4, 16, 200);
		optitrk::SetCameraSettings(1, 4, 16, 200);
		optitrk::SetCameraSettings(2, 4, 16, 200);
	}

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
	
	if (__gc.g_optiRecordMode != OPTTRK_RECMODE::LOAD) {
		optitrk::DeinitOptiTrackLib();
	}

	vzm::DeinitEngineLib();
	__gc.Deinit();

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
	   5, 5, DESIRED_SCREEN_W, DESIRED_SCREEN_H, nullptr, nullptr, hInstance, nullptr); //-1915
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
//     1 : AP (left view), 2 : Lateral (right view),  3 : calibration (left view)
//     TO DO... calibration history...
// key S : save current camera state (main window only)
// key L : load the latest-stored camera state (main window only)
// key C : calibration mode for selecting markers whose position is used to compute c-arm extrinsic calibration
// key U : update c-arm marker set (rigid body)
// key T : update tool marker set (rigid body)
// key X : compute c-arm extrinsics based on the c-arm marker set and the selected marker positions
// key D : just for UI (clear test marker spheres)
// key Y : tool tip test for estimated mk position

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
	float scene_stage_scale = 5.f;

	// DOJO : NOTE scene_stage_center must not be the same as camera position
	glm::fvec3 scene_stage_center = glm::fvec3(0, 0, 2); 
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
		if (__gc.g_optiEvent != OPTTRK_THREAD::FREE 
			|| __gc.g_renderEvent != RENDER_THREAD::FREE
			|| __gc.g_networkEvent != NETWORK_THREAD_FREE) break;

		using namespace std;
		switch (wParam) {
			case char('S') :
			{
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
				if (__gc.g_calribmodeToggle) {
					__gc.SetErrorCode("Loading Camera is Not Allowed during Calib Mode!");
					break;
				}

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
				if (__gc.g_optiRecordMode == OPTTRK_RECMODE::LOAD) {
					__gc.SetErrorCode("Recording is Not Allowed during Playing Rec File!");
					break;
				}
				// recode mode.
				// toggle
				
				__gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD ? __gc.g_optiRecordMode = OPTTRK_RECMODE::NONE
					: __gc.g_optiRecordMode = OPTTRK_RECMODE::RECORD;

				break;
			}
			case char('C') :
			{
				if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
					__gc.SetErrorCode("Not Allowed during Rec Processing!");
					break;
				}

				__gc.g_calribmodeToggle = !__gc.g_calribmodeToggle;

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
				track_info trk;
				trk = __gc.g_track_que.front();
				trk.GetTrackingCamPose(0, matCam2WS);

				*(fvec3*)cpCam1.pos = vzmutils::transformPos(fvec3(0, 0, 0), matCam2WS);
				*(fvec3*)cpCam1.view = vzmutils::transformVec(fvec3(0, 0, -1), matCam2WS);
				*(fvec3*)cpCam1.up = vzmutils::transformVec(fvec3(0, 1, 0), matCam2WS);
				cpCam1.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_FOV;
				cpCam1.fov_y = glm::pi<float>() / 2.f;
				cpCam1.aspect_ratio = cpCam1.ip_w / cpCam1.ip_h;
				cpCam1.np = 0.15f;

				vzm::SetCameraParams(cidCam1, cpCam1);
				break;
			}
			case char('U') :
			{
				if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
					__gc.SetErrorCode("Not Allowed during Rec Processing!");
					break;
				}

				if (!__gc.g_calribmodeToggle) {
					__gc.SetErrorCode("No Marker Selection!");
					break;
				}

				// update "c-arm" RB
				int numSelectedMKs = (int)__gc.g_selectedMkNames.size();
				if (numSelectedMKs < 3) {
					__gc.SetErrorCode("At least 3 Points are Needed!");
					cout << "\n" << "at least 3 points are needed to register a rigid body!! current # of points : " << numSelectedMKs << endl;
					break;
				}

				cout << "\n" << "rigidbody for c-arm is registered with " << numSelectedMKs << " points" << endl;

				__gc.g_optiEvent = OPTTRK_THREAD::C_ARM_REGISTER;

				while (__gc.g_optiEvent == OPTTRK_THREAD::C_ARM_REGISTER) { Sleep(2); }

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
			case char('Y') : { // just for test
				//track_info trk;
				//trk = __gc.g_track_que.front();
				//glm::fmat4x4 matRB2WS;
				//if (trk.GetRigidBodyByName("probe", &matRB2WS, NULL, NULL, NULL)) {
				//	glm::fvec3 p = vzmutils::transformPos(glm::fvec3(0, 0, 0), matRB2WS);
				//
				//	//if (trk.GetRigidBodyByName("c-arm", &matRB2WS, NULL, NULL, NULL)) 
				//	{
				//
				//		//glm::fmat4x4 matWS2RB = glm::inverse(matRB2WS);
						//p = vzmutils::transformPos(p, matWS2RB);
				//		cv::Mat ocvVec3(1, 3, CV_32FC1);
				//		memcpy(ocvVec3.ptr(), &p, sizeof(float) * 3);
				//		cv::FileStorage fs(__gc.g_folder_trackingInfo + "test_tip.txt", cv::FileStorage::Mode::WRITE);
				//		fs << "probe_tip" << ocvVec3;
				//		fs.release();
				//
				//		int idx = (int)___LoadMyVariable("test0", 0);
				//
				//		cv::FileStorage __fs(__gc.g_folder_trackingInfo + "test_tip_sphere.txt", cv::FileStorage::Mode::READ);
				//		__fs["inter_sphere_" + std::to_string(idx)] >> ocvVec3;
				//		glm::fvec3 p1;
				//		memcpy(&p1, ocvVec3.ptr(), sizeof(float) * 3);
				//		__fs.release();
				//
				//		float r = glm::length(p1 - p);
				//		std::cout << "\ntest : " << r << std::endl;
				//	}
				//
				//}
				break;
			}
			case char('T') : {
				if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
					__gc.SetErrorCode("Not Allowed during Rec Processing!");
					break;
				}
				if (!__gc.g_calribmodeToggle) {
					__gc.SetErrorCode("No Marker Selection!");
					cout << "\n" << "not calibration mode!!" << endl;
					break;
				}
				if (__gc.g_selectedMkNames.size() < 4) {
					__gc.SetErrorCode("At least 3 Points are Needed!");
					cout << "\n" << "at least 4 points are needed!!" << endl;
					break;
				}

				// 툴 등록하기..
				__gc.g_optiEvent = OPTTRK_THREAD::TOOL_REGISTER;
				while (__gc.g_optiEvent != OPTTRK_THREAD::FREE) { Sleep(2); }
				//__gc.g_testMKs.clear();
				//__gc.g_testMKs.push_back(glm::fvec3(0));
				//__gc.g_testMKs.push_back(glm::fvec3(0));
				//__gc.g_testMKs.push_back(glm::fvec3(0));
				break;
			}
			case char('P') : {
				if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
					__gc.SetErrorCode("Not Allowed during Rec Processing!");
					break;
				}
				if (__gc.g_calribmodeToggle) {
					__gc.SetErrorCode("Pivoting is Not Allowed during Marker Selection!");
					break;
				}
				if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_RESET_PIVOT) {
					__gc.SetErrorCode("Wait for Canceling Pivot Process!");
					break;
				}
				track_info trk = __gc.g_track_que.front(); // __gc.g_track_que.wait_and_pop(trk);
				if (!trk.GetRigidBodyQuatTVecByName("tool", NULL, NULL)) {
					__gc.SetErrorCode("No Tool is Registered!");
					break;
				}
				if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_PIVOT) __gc.g_optiEvent = OPTTRK_THREAD::TOOL_RESET_PIVOT;
				else __gc.g_optiEvent = OPTTRK_THREAD::TOOL_PIVOT;
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
			case char('[') : {
				__gc.g_probeTipCorrection -= 0.0002;
				break;
			}
			case char(']') : {
				__gc.g_probeTipCorrection += 0.0002;
				break;
			}
			case char('X') :
			{
				if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
					__gc.SetErrorCode("Not Allowed during Rec Processing!");
					break;
				}
				if (!__gc.g_calribmodeToggle) {
					__gc.SetErrorCode("No Marker Selection!");
					cout << "\n" << "not calibration mode!!" << endl;
					break;
				}
				//#define MYDEFINE

				cv::Mat downloadedImg = cv::imread(__gc.g_folder_trackingInfo + "test_downloaded.png");

				if (downloadedImg.empty()) {
					__gc.SetErrorCode("No Download Image!");
					cout << "\n" << "no downloaded image" << endl;
					break;
				}
				__gc.g_downloadCompleted = 100;
				cv::cvtColor(downloadedImg, g_curScanGrayImg, cv::COLOR_RGB2GRAY);
				cout << "\n" << "calibration with the previous downloaded image" << endl;

				track_info trk = __gc.g_track_que.front(); // __gc.g_track_que.wait_and_pop(trk);
				if (trk.IsValidTracking()) {
					if (calibtask::CalibrationWithPhantom(__gc.g_CArmRB2SourceCS, g_curScanGrayImg, &trk, __gc.g_useGlobalPairs)) {
						cv::Mat processColorImg = cv::imread(__gc.g_folder_trackingInfo + "test_circle_sort.png");
						//download_completed = false; 


						glm::fmat4x4 matCArmRB2WS;
						if (!trk.GetRigidBodyByName("c-arm", &matCArmRB2WS, NULL, NULL, NULL)) {
							__gc.SetErrorCode("C-Arm is Not Detected!\nCheck the C-Arm Markers and Retry!");
							break;
						}

						saveAndChangeViewState(char('3'), sidScene, cidCam1, 0, &matCArmRB2WS, &processColorImg);
						__gc.g_ui_banishing_count = 100;
					}
				}
				break;
			}
			default: {
				if (wParam >= char('0') && wParam <= char('9')) {
					if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
						__gc.SetErrorCode("Not Allowed during Rec Processing!");
						break;
					}
					saveAndChangeViewState(wParam, sidScene, cidCam1, 0, NULL, NULL);
				}
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
		if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_REGISTER) break;
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
				general_move.Start((int*)&pos_ss, cpCam1, scene_stage_center, scene_stage_scale);
			}
		}
		else {
			general_move.Start((int*)&pos_ss, cpCam1, scene_stage_center, scene_stage_scale);
		}
		break;
	}
	case WM_MOUSEMOVE:
	{
		if (__gc.g_calribmodeToggle && !(wParam & MK_RBUTTON)) break;

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

		if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_REGISTER) {
			if (zDelta > 0)
				cpCam1.fov_y *= 0.9f;
			else
				cpCam1.fov_y *= 1.1f;
		}
		else {
			// by adjusting cam distance
			if (zDelta > 0)
				*(glm::fvec3*)cpCam1.pos += scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam1.view);
			else
				*(glm::fvec3*)cpCam1.pos -= scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam1.view);
		}

		vzm::SetCameraParams(cidCam1, cpCam1);
		break;
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
	glm::fvec3 scene_stage_center = glm::fvec3(0, 0, 2);
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
		if (__gc.g_calribmodeToggle) {
			__gc.SetErrorCode("Not Allowed during the Marker Selection!");
			break;
		}
		if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
			__gc.SetErrorCode("Not Allowed during Rec Processing!");
			break;
		}
		saveAndChangeViewState(wParam, sidScene, cidCam2, 1, NULL, NULL);
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
