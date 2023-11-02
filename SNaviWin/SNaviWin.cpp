// Dear ImGui: standalone example application for DirectX 11
// If you are new to Dear ImGui, read documentation from the docs/ folder + read the top of imgui.cpp.
// Read online: https://github.com/ocornut/imgui/tree/master/docs

#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx11.h"
#include <d3d11.h>
#include <tchar.h>

#include "GlobalParams.h"

#include "framework.h"
#include "SNaviWin.h"

#include "NetworkTask.h"
#include "TrackingTask.h"
#include "RenderingTask.h"
#include "VmGui.h"
#include "../optitrk/optitrk.h"

#include <opencv2/opencv.hpp>

#include <regex>
#include <vector>
#include <windowsx.h>
#include <playsoundapi.h>
#include <iostream>
#include <stdio.h>
#include <ctime>

#include <filesystem>

// SpineProjects
#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#include "naviHelpers.hpp"
#include "rapidcsv/rapidcsv.h"
#include "CArmCalibration.h"

#define MAX_LOADSTRING 100
typedef unsigned long long u64;

// Data
static ID3D11Device* g_pd3dDevice = nullptr;
static ID3D11DeviceContext* g_pd3dDeviceContext = nullptr;
static IDXGISwapChain* g_pSwapChain = nullptr;
static UINT                     g_ResizeWidth = 0, g_ResizeHeight = 0;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;

// Forward declarations of helper functions
bool CreateDeviceD3D(HWND hWnd);
void CleanupDeviceD3D();
void CreateRenderTarget();
void CleanupRenderTarget();
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

HWND hwnd;
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
auto ___StoreParams = [](const std::string& paramsFileName, const glm::fmat4x4& matCArmRB2WS, const std::string& imgFileName) {

	cv::FileStorage __fs(__gc.g_folder_trackingInfo + paramsFileName, cv::FileStorage::Mode::WRITE);

	cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
	cv::Mat matK;
	fs["K"] >> matK;
	__fs << "K" << matK;
	cv::Mat distCoeffs;
	fs["DistCoeffs"] >> distCoeffs;
	__fs << "DistCoeffs" << distCoeffs;
	fs.release();

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
auto ___SaveAndChangeViewState = [](const int keyParam, const int sidScene, const int cidCam, const int viewIdx, 
	const glm::fmat4x4* pmatCArmRB2WS, const cv::Mat* pcolored_img, const std::string* load_timePack = NULL)
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
			__gc.g_lastStoredImageName = "test" + to_string(i) + "_" + timePack;
			cv::imwrite(downloadImgFileName, colored_img);

			___StoreParams("test" + to_string(i) + "_" + timePack + ".txt", matCArmRB2WS, downloadImgFileName);
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

			if (load_timePack == NULL) {
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
			else {
				timePack = *load_timePack;
			}
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

int g_posWinX = 0, g_posWinY = 0;
int g_sizeWinX = 1280, g_sizeWinY = 800;
int g_sizeRightPanelW = 400, g_sizeConsolePanelH = 290;
BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData) {
	static int monitor_idx = 0;
	MONITORINFOEX monitorInfo;
	monitorInfo.cbSize = sizeof(MONITORINFOEX);
	if (GetMonitorInfo(hMonitor, &monitorInfo)) {
		std::cout << "Monitor Name: " << monitorInfo.szDevice << std::endl;
		std::cout << "Monitor Resolution: " << monitorInfo.rcMonitor.right - monitorInfo.rcMonitor.left
			<< "x" << monitorInfo.rcMonitor.bottom - monitorInfo.rcMonitor.top << std::endl;
		std::cout << "Monitor Position: (" << monitorInfo.rcMonitor.left << "," << monitorInfo.rcMonitor.top << ")"
			<< std::endl;
		std::cout << "------------------------" << std::endl;
		//if (monitor_idx == 1) 
		{
			g_posWinX = monitorInfo.rcMonitor.left;
			g_posWinY = monitorInfo.rcMonitor.top;
			g_sizeWinX = monitorInfo.rcMonitor.right - monitorInfo.rcMonitor.left;
			g_sizeWinY = monitorInfo.rcMonitor.bottom - monitorInfo.rcMonitor.top;
		}
	}
	monitor_idx++;
	return TRUE;
}

void CallBackMouseDown(vmgui::ImGuiVzmWindow* pThis) {
	
	if (!__gc.g_calribmodeToggle)
		return;

	ImGuiIO& io = ImGui::GetIO();
	glm::fvec2 mPos = pThis->GetMousePos();

	// renderer does not need to be called
	glm::ivec2 pos_ss = glm::ivec2(mPos);

	if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
		int aidPicked = 0;
		glm::fvec3 posPick;
		int cidRender1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
		if (vzm::PickActor(aidPicked, __FP posPick, pos_ss.x, pos_ss.y, cidRender1)) {
			std::string actorName;
			vzm::GetSceneItemName(aidPicked, actorName);
			if (__gc.g_selectedMkNames.find(actorName) == __gc.g_selectedMkNames.end()) {
				int idx = __gc.g_selectedMkNames.size();
				__gc.g_selectedMkNames[actorName] = idx;
				__gc.g_engineLogger->info("# of Picking Markers : {}", __gc.g_selectedMkNames.size());
			}
			pThis->SetStartEvent(vmgui::ImGuiVzmWindow::OnEvents::CUSTOM_DOWN);
		}
	}
}

std::map<std::string, ID3D11ShaderResourceView*> g_mapImgSRVs;

// Simple helper function to load an image into a DX11 texture with common settings
ID3D11ShaderResourceView* LoadTextureFromFile(const unsigned char* image_data, const char* texture_name, const int width, const int height)
{
	// Create texture
	D3D11_TEXTURE2D_DESC desc;
	ZeroMemory(&desc, sizeof(desc));
	desc.Width = width;
	desc.Height = height;
	desc.MipLevels = 1;
	desc.ArraySize = 1;
	desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	desc.SampleDesc.Count = 1;
	desc.Usage = D3D11_USAGE_DEFAULT;
	desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	desc.CPUAccessFlags = 0;

	ID3D11Texture2D* pTexture = NULL;
	D3D11_SUBRESOURCE_DATA subResource;
	subResource.pSysMem = image_data;
	subResource.SysMemPitch = desc.Width * 4;
	subResource.SysMemSlicePitch = 0;
	g_pd3dDevice->CreateTexture2D(&desc, &subResource, &pTexture);

	ID3D11ShaderResourceView** ppSRV = NULL;
	auto it = g_mapImgSRVs.find(texture_name);
	if (it != g_mapImgSRVs.end()) {
		ppSRV = &it->second;
		(*ppSRV)->Release();
	}
	else {
		ppSRV = &g_mapImgSRVs[texture_name];
	}
	(*ppSRV) = NULL;

	// Create texture view
	D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
	ZeroMemory(&srvDesc, sizeof(srvDesc));
	srvDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	srvDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Texture2D.MipLevels = desc.MipLevels;
	srvDesc.Texture2D.MostDetailedMip = 0;
	g_pd3dDevice->CreateShaderResourceView(pTexture, &srvDesc, ppSRV);
	pTexture->Release();

	return *ppSRV;
};

void mygui(ImGuiIO& io) {

	static track_info trackInfo;

	if (!__gc.g_track_que.empty())
	{
		__gc.g_track_que.wait_and_pop(trackInfo);
	}

	int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	int cidRender1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
	int cidRender2 = vzmutils::GetSceneItemIdByName(__gc.g_camName2);

	if (trackInfo.IsValidTracking()) {

		//::EnterCriticalSection(&lock);
		// DOJO: concurrent queue 에서 가장 최신에 저장된 tracking frame 정보 pop out
		if (__gc.g_optiRecordFrame > 0)
			__gc.g_optiRecordFrame++;

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
				if (__gc.g_optiRecordFrame <= 2) {
					recRowCount = 0;

					glm::fvec3 posInit, viewInit, upInit;
					cv::FileStorage fs(__gc.g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
					if (fs.isOpened()) {
						cv::Mat ocvVec3;
						fs["POS"] >> ocvVec3;
						memcpy(&posInit, ocvVec3.ptr(), sizeof(float) * 3);
						fs["VIEW"] >> ocvVec3;
						memcpy(&viewInit, ocvVec3.ptr(), sizeof(float) * 3);
						fs["UP"] >> ocvVec3;
						memcpy(&upInit, ocvVec3.ptr(), sizeof(float) * 3);
						fs.release();
					}

					float vFov = 3.141592654f / 4.f;
					int cidCams[2] = { cidRender1 , cidRender2 };
					for (int i = 0; i < 2; i++) {
						vzm::CameraParameters cpCam;
						vzm::GetCameraParams(cidCams[i], cpCam);

						float aspect_ratio = (float)cpCam.w / (float)cpCam.h;
						float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;
						cpCam.fx = cpCam.w / (2.f * tan(hFov / 2.f));
						cpCam.fy = cpCam.h / (2.f * tan(vFov / 2.f));
						cpCam.sc = 0;
						cpCam.cx = cpCam.w / 2.f;
						cpCam.cy = cpCam.h / 2.f;
						cpCam.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
						*(glm::fvec3*)cpCam.pos = posInit;
						*(glm::fvec3*)cpCam.view = viewInit;
						*(glm::fvec3*)cpCam.up = upInit;
						vzm::SetCameraParams(cidCams[i], cpCam);
					}

					for (auto it = __gc.g_mapAidGroupCArmCam.begin(); it != __gc.g_mapAidGroupCArmCam.end(); it++) {
						vzm::RemoveSceneItem(it->second, true);
					}
				}
			}
			else {
				std::string frameStr = csvTrkData.GetRowName(recRowCount);
				recordFrame = std::stoi(frameStr);
			}
			if (__gc.g_optiRecordFrame > recordFrame) {
				std::vector<std::string> rowData = csvTrkData.GetRow<std::string>(recRowCount);
				recRowCount++;

				bool isAP = rowData[1] == "AP";

				std::string timePack = rowData[0];
				if (timePack.back() == ' ') timePack.pop_back();
				if (isAP) {
					___SaveAndChangeViewState(char('1'), sidScene, cidRender1, 0, NULL, NULL, &timePack);
				}
				else {
					___SaveAndChangeViewState(char('2'), sidScene, cidRender2, 1, NULL, NULL, &timePack);
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

					if (__gc.g_is_ready_for_shotmoment)
					{
						glm::fquat q_cur, q_prev;
						glm::fvec3 t_cur, t_prev;
						trackInfo.GetRigidBodyQuatTVecByName("c-arm", &q_cur, &t_cur);
						__gc.g_track_info_shotmoment.GetRigidBodyQuatTVecByName("c-arm", &q_prev, &t_prev);

						glm::fmat4x4 matCArmRB2WS_prev;
						__gc.g_track_info_shotmoment.GetRigidBodyByName("c-arm", &matCArmRB2WS_prev, NULL, NULL, NULL);

						glm::fvec3 o_cur, o_prev;
						o_cur = vzmutils::transformPos(glm::fvec3(0), matCArmRB2WS);
						o_prev = vzmutils::transformPos(glm::fvec3(0), matCArmRB2WS_prev);
						glm::fquat q_diff = q_cur * glm::inverse(q_prev);
						float angle = glm::degrees(2.0f * acos(q_diff.w));
						float err_t_dist = glm::length(o_cur - o_prev);
						float err_t = glm::length(t_cur - t_prev);
						//__gc.g_is_ready_for_shotmoment = false;

						__gc.SetErrorCode("angle:" + std::to_string(angle) + "\n"
							+ "err_t_dist:" + std::to_string(err_t_dist) + "\n"
							+ "err_t:" + std::to_string(err_t), 1000);
					}

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
						___SaveAndChangeViewState(char('1'), sidScene, cidRender1, 0, &matCArmRB2WS, &downloadColorImg);
					else
						___SaveAndChangeViewState(char('2'), sidScene, cidRender2, 1, &matCArmRB2WS, &downloadColorImg);
				}
				else {
					PlaySound((LPCTSTR)SND_ALIAS_SYSTEMWELCOME, NULL, SND_ALIAS_ID | SND_ASYNC);
					__gc.SetErrorCode("C-Arm is Not Detected!\nCheck the C-Arm Markers and Retry!", 300);
				}

				__gc.g_renderEvent = RENDER_THREAD::FREE;
			}
		}

		rendertask::RenderTrackingScene(&trackInfo);
	}

	vzm::CameraParameters cpCam1, cpCam2;
	vzm::GetCameraParams(cidRender1, cpCam1);
	vzm::GetCameraParams(cidRender2, cpCam2);

	static bool viewInit = false;
	static vmgui::ImGuiVzmWindow view1("View 1", sidScene, cidRender1, g_pd3dDevice);
	static vmgui::ImGuiVzmWindow view2("View 2", sidScene, cidRender2, g_pd3dDevice);

	if (!viewInit) {
		viewInit = true;
		view1.SetStageCenterScale(glm::fvec3(0, 0, 1.2f), 1.f);
		view2.SetStageCenterScale(glm::fvec3(0, 0, 1.2f), 1.f);
		view1.AddCallback(vmgui::ImGuiVzmWindow::OnEvents::CUSTOM_DOWN, CallBackMouseDown);
		view1.SkipRender(true);
		view2.SkipRender(true);
	}

	static ImVec4 clear_color = ImVec4(0.f, 0.2f, 0.3f, 0.95f);

	{
		static ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings
			| ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoBackground;
		// tree view 넣기

		static int counter = 0;

		ImGui::SetNextWindowPos(ImVec2(g_sizeWinX - g_sizeRightPanelW + 20, 5), ImGuiCond_Once);
		ImGui::Begin("Control Widgets");                          // Create a window called "Hello, world!" and append into it.
		ImGui::ColorEdit4("background color", (float*)&clear_color); // Edit 4 floats representing a color

		if (ImGui::Button("Save Cam"))
		{
			cv::FileStorage fs(__gc.g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::WRITE);
			cv::Mat ocvVec3(1, 3, CV_32FC1);
			memcpy(ocvVec3.ptr(), cpCam1.pos, sizeof(float) * 3);
			fs.write("POS", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam1.view, sizeof(float) * 3);
			fs.write("VIEW", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam1.up, sizeof(float) * 3);
			fs.write("UP", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam2.pos, sizeof(float) * 3);
			fs.write("POS2", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam2.view, sizeof(float) * 3);
			fs.write("VIEW2", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam2.up, sizeof(float) * 3);
			fs.write("UP2", ocvVec3);
			fs.release();
		}
		ImGui::SameLine();
		if (ImGui::Button("Load Cam"))
		{
			if (__gc.g_calribmodeToggle) {
				__gc.SetErrorCode("Loading Camera is Not Allowed during Calib Mode!");
			}
			else {
				cv::FileStorage fs(__gc.g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
				if (fs.isOpened()) {
					cv::Mat ocvVec3;
					fs["POS"] >> ocvVec3;
					memcpy(cpCam1.pos, ocvVec3.ptr(), sizeof(float) * 3);
					fs["VIEW"] >> ocvVec3;
					memcpy(cpCam1.view, ocvVec3.ptr(), sizeof(float) * 3);
					fs["UP"] >> ocvVec3;
					memcpy(cpCam1.up, ocvVec3.ptr(), sizeof(float) * 3);
					fs["POS2"] >> ocvVec3;
					memcpy(cpCam2.pos, ocvVec3.ptr(), sizeof(float) * 3);
					fs["VIEW2"] >> ocvVec3;
					memcpy(cpCam2.view, ocvVec3.ptr(), sizeof(float) * 3);
					fs["UP2"] >> ocvVec3;
					memcpy(cpCam2.up, ocvVec3.ptr(), sizeof(float) * 3);
					fs.release();
				}
				vzm::SetCameraParams(cidRender1, cpCam1);
				vzm::SetCameraParams(cidRender2, cpCam2);
			}
		}
		static int buttonHeight = 50;
		if (ImGui::Button("Marker Selection", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else {

				__gc.g_calribmodeToggle = !__gc.g_calribmodeToggle;

				if (!__gc.g_calribmodeToggle) {
					view1.EnableCustomDown(false);
					__gc.g_testMKs.clear();
					__gc.g_selectedMkNames.clear();
					int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
					if (aidTestGroup != 0)
						vzm::RemoveSceneItem(aidTestGroup);
				}
				else 
					view1.EnableCustomDown(true);

				__gc.g_engineLogger->info("calibration mode " + __gc.g_calribmodeToggle ? "ON" : "OFF");

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

				vzm::SetCameraParams(cidRender1, cpCam1);
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Clear Marker Selection", ImVec2(0, buttonHeight)))
		{
			__gc.g_selectedMkNames.clear();
		}

		if (ImGui::Button("Register C-Arm", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (!__gc.g_calribmodeToggle) {
				__gc.SetErrorCode("No Marker Selection!");
			}
			else {
				// update "c-arm" RB
				int numSelectedMKs = (int)__gc.g_selectedMkNames.size();
				if (numSelectedMKs < 3) {
					__gc.SetErrorCode("At least 3 Points are Needed!");
				}

				__gc.g_engineLogger->info("rigidbody for c-arm is registered with {} points", numSelectedMKs);

				__gc.g_optiEvent = OPTTRK_THREAD::C_ARM_REGISTER;

				while (__gc.g_optiEvent == OPTTRK_THREAD::C_ARM_REGISTER) { Sleep(2); }
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Register T-Niddle", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (!__gc.g_calribmodeToggle) {
				__gc.SetErrorCode("No Marker Selection!");
			}
			else if (__gc.g_selectedMkNames.size() < 4) {
				__gc.SetErrorCode("At least 3 Points are Needed!");
			}
			else {
				// 툴 등록하기..
				__gc.g_optiEvent = OPTTRK_THREAD::TOOL_REGISTER;
				__gc.g_optiToolId = 1;
				while (__gc.g_optiEvent != OPTTRK_THREAD::FREE) { Sleep(2); }
				__gc.g_optiToolId = 0;
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Register Troca", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (!__gc.g_calribmodeToggle) {
				__gc.SetErrorCode("No Marker Selection!");
			}
			else if (__gc.g_selectedMkNames.size() < 4) {
				__gc.SetErrorCode("At least 3 Points are Needed!");
			}
			else {
				// 툴 등록하기..
				__gc.g_optiEvent = OPTTRK_THREAD::TOOL_REGISTER;
				__gc.g_optiToolId = 2;
				while (__gc.g_optiEvent != OPTTRK_THREAD::FREE) { Sleep(2); }
				__gc.g_optiToolId = 0;
			}
		}

		if (ImGui::Button("T-Needle Pivoting", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (__gc.g_calribmodeToggle) {
				__gc.SetErrorCode("Pivoting is Not Allowed during Marker Selection!");
			}
			else if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_RESET_PIVOT) {
				__gc.SetErrorCode("Wait for Canceling Pivot Process!");
			}
			else {
				track_info trk = __gc.g_track_que.front(); // __gc.g_track_que.wait_and_pop(trk);
				if (!trk.GetRigidBodyQuatTVecByName("t-needle", NULL, NULL)) {
					__gc.SetErrorCode("No Tool is Registered!");
				}
				else {
					__gc.g_optiToolId = 1;
					if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_PIVOT) 
						__gc.g_optiEvent = OPTTRK_THREAD::TOOL_RESET_PIVOT;
					else __gc.g_optiEvent = OPTTRK_THREAD::TOOL_PIVOT;;
				}
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Troca Pivoting", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (__gc.g_calribmodeToggle) {
				__gc.SetErrorCode("Pivoting is Not Allowed during Marker Selection!");
			}
			else if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_RESET_PIVOT) {
				__gc.SetErrorCode("Wait for Canceling Pivot Process!");
			}
			else {
				track_info trk = __gc.g_track_que.front(); // __gc.g_track_que.wait_and_pop(trk);
				if (!trk.GetRigidBodyQuatTVecByName("troca", NULL, NULL)) {
					__gc.SetErrorCode("No Tool is Registered!");
				}
				else {
					__gc.g_optiToolId = 2;
					if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_PIVOT)
						__gc.g_optiEvent = OPTTRK_THREAD::TOOL_RESET_PIVOT;
					else __gc.g_optiEvent = OPTTRK_THREAD::TOOL_PIVOT;;
				}
			}
		}

		if (ImGui::Button("Record", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode == OPTTRK_RECMODE::LOAD) {
				__gc.SetErrorCode("Recording is Not Allowed during Playing Rec File!");
			}
			else {
				__gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD ? __gc.g_optiRecordMode = OPTTRK_RECMODE::NONE
					: __gc.g_optiRecordMode = OPTTRK_RECMODE::RECORD;
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete Scans", ImVec2(0, buttonHeight)))
		{
			for (int i = 0; i < 9; i++) {
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
		}

		if (ImGui::Button("View AP", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else {
				___SaveAndChangeViewState(char('1'), sidScene, cidRender1, 0, NULL, NULL);
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("View LL", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else {
				___SaveAndChangeViewState(char('2'), sidScene, cidRender2, 1, NULL, NULL);
			}
		}

		auto DrawCircles = [](const std::vector<cv::Point2f>& points2Ds, std::vector<float>& circleRadiis, cv::Mat& img) {
			for (int i = 0; i < (int)points2Ds.size(); i++) {
				cv::Point2f center = points2Ds[i];
				float x = (center.x); // cvRound
				float y = (center.y);
				float r = (circleRadiis[i]);

				cv::circle(img, cv::Point(x, y), r, cv::Scalar(0, 0, 255), 5);
				//std::string text = " (" + std::to_string(x) + ", " + std::to_string(y) + ")";
				//cv::putText(img, text, cv::Point(x - 25, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);

				// index 출력
				cv::putText(img, std::to_string(i), cv::Point(x - 20, y - 20), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0), 2);
			}
		};
		auto ComputeReprojectionErrors = [](const std::vector<std::vector<cv::Point3f>>& objectPoints,
			const std::vector<std::vector<cv::Point2f> >& imagePoints,
			const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
			const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
		{
			// https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
			std::vector<cv::Point2f> imagePoints2;
			size_t totalPoints = 0;
			double totalErr = 0, err;
			//perViewErrors.resize(objectPoints.size());
			for (size_t i = 0; i < objectPoints.size(); ++i)
			{
				projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
				err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);
				size_t n = objectPoints[i].size();
				//perViewErrors[i] = (float)std::sqrt(err * err / n);
				totalErr += err * err;
				totalPoints += n;
			}
			return std::sqrt(totalErr / totalPoints);
		};

		///////////////////////
		// intrinsics
		///////////////////////
		{
			static std::vector<cv::Point3f> corners;
			static int indexIntrinsic = -1;
			static std::vector<std::string> files;
			if (indexIntrinsic == -1) {
				indexIntrinsic = 0;
				for (const auto& entry : std::filesystem::directory_iterator(__gc.g_folder_trackingInfo + "int_images")) {
					if (entry.path().extension().string() == ".png")
						files.push_back(entry.path().string());
				}
			}
			static ID3D11ShaderResourceView* pSRV = NULL;
			static std::vector<cv::Point2f> points2D;
			static std::vector<std::vector<cv::Point3f>> calib_points3Ds;
			static std::vector<std::vector<cv::Point2f>> calib_points2Ds;
			static int intrinsicWidth = 8, intrinsicHeight = 8;
			static int w, h;
			static bool isCalibratedIntrinsic = false;
			if (ImGui::Button("Add Intrinsic", ImVec2(0, buttonHeight)))
			{
				cv::Mat img;
				if (__gc.g_optiRecordMode == OPTTRK_RECMODE::LOAD)
					img = cv::imread(files[indexIntrinsic]);
				else if (g_curScanGrayImg.data != NULL) {
					cv::cvtColor(g_curScanGrayImg, img, cv::COLOR_GRAY2RGBA);
					cv::imwrite(__gc.g_folder_trackingInfo + "int_images/" + __gc.g_lastStoredImageName + ".png", img);
				}
				else {
					__gc.SetErrorCode("No Image!!");
				}

				cv::FileStorage fs(__gc.g_folder_trackingInfo + "calib_settings.txt", cv::FileStorage::Mode::READ);
				std::string calib_mode = "FMARKERS";
				std::string det_mode = "SimpleBlobDetector";
				if (fs.isOpened()) {
					fs["INT_ROWS"] >> intrinsicHeight;
					fs["INT_COLS"] >> intrinsicWidth;
					fs["INT_MODE"] >> calib_mode;
					fs["INT_DETECTOR"] >> det_mode;
					fs.release();
				}

				// Load from disk into a raw RGBA buffer
				if (img.data != NULL) {
					int image_width = img.cols;
					int image_height = img.rows;

					// optional
					cv::flip(img, img, 1);

					cv::Mat imgGray;
					cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
					// processing..

					std::vector<float> circleRadiis;
					mystudents::Get2DPostionsFromFMarkersPhantom(imgGray, intrinsicWidth, intrinsicHeight, points2D, 2000.f, 10000.f, 50.f, &circleRadiis);

					DrawCircles(points2D, circleRadiis, img);
					
					//cv::cvtColor(imgGray, img, cv::COLOR_GRAY2RGBA);
					//image_data = img.data;

					if (img.channels() == 3) {
						// First create the image with alpha channel
						cv::cvtColor(img, img, cv::COLOR_RGB2RGBA);
					
						// # Split the image for access to alpha channel
						std::vector<cv::Mat> channels(4);
						cv::split(img, channels);
						
						// Assign the mask to the last channel of the image
						channels[3] = 255;
						
						// Finally concat channels for rgba image
						cv::merge(channels, img);
					}

					__gc.g_engineLogger->info("image index : {}", indexIntrinsic);

					pSRV = LoadTextureFromFile(img.data, "INTRINSIC_IMG", image_width, image_height);
					indexIntrinsic++;
					if (__gc.g_optiRecordMode == OPTTRK_RECMODE::LOAD) indexIntrinsic = indexIntrinsic % files.size();
					isCalibratedIntrinsic = false;
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Apply Intrinsic", ImVec2(0, buttonHeight)))
			{
				if (points2D.size() != intrinsicWidth * intrinsicHeight) {
					__gc.SetErrorCode("Failure to detect the entire circles!");
				}
				if (isCalibratedIntrinsic) {
					__gc.SetErrorCode("The same scan image is already applied!");
				}
				else {
					isCalibratedIntrinsic = true;

					if (corners.size() == 0) {
						const float squareSize = 15.9375f; // mm unit
						for (int i = 0; i < intrinsicHeight; ++i) {
							for (int j = 0; j < intrinsicWidth; ++j) {
								corners.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
							}
						}
					}

					calib_points2Ds.push_back(points2D);
					calib_points3Ds.push_back(corners);

					__gc.g_engineLogger->info("# of {}-pair sets : {}", intrinsicWidth* intrinsicHeight, calib_points2Ds.size());

					cv::Mat cameraMatrix, distCoeffs;
					std::vector<cv::Mat> rvecs, tvecs;
					float err = (float)cv::calibrateCamera(calib_points3Ds, calib_points2Ds, cv::Size(intrinsicWidth, intrinsicHeight), cameraMatrix, distCoeffs, rvecs, tvecs);

					float err1 = ComputeReprojectionErrors(calib_points3Ds, calib_points2Ds, rvecs, tvecs, cameraMatrix, distCoeffs);
					__gc.g_engineLogger->info("Calibtation Error : {}, Reprojection Error : {}", err, err1);

					// to do //
					//cv::Mat cameraMatrixRO, distCoeffsRO, rvec, tvec;
					//cv::calibrateCameraRO(corners, points2Ds, cv::Size(intrinsicWidth, intrinsicHeight), 1, cameraMatrix, distCoeffs, rvec, tvec);

					cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::WRITE);
					fs << "K" << cameraMatrix;
					fs << "DistCoeffs" << distCoeffs;
					//fs << "K_RO" << cameraMatrix;
					//fs << "DistCoeffs_RO" << distCoeffs;
					fs.release();
				}
			}
			if (pSRV != NULL) {
				ImVec2 pos = ImGui::GetCursorScreenPos();
				ImGui::Image(pSRV, ImVec2(g_sizeRightPanelW - 60, g_sizeRightPanelW - 60), ImVec2(0, 0), ImVec2(1, 1));
				//ImVec2 uv_min = ImVec2(0.0f, 0.0f);                 // Top-left
				//ImVec2 uv_max = ImVec2(1.0f, 1.0f);                 // Lower-right
				//ImGui::Image(my_tex_id, ImVec2(my_tex_w, my_tex_h), uv_min, uv_max);
			}
		}

		///////////////////////
		// extrinsics
		///////////////////////
		{
			static int indexExtrinsics = -1;
			static std::vector<std::string> files;
			if (indexExtrinsics == -1) {
				for (const auto& entry : std::filesystem::directory_iterator(__gc.g_folder_trackingInfo + "ext_images")) {
					if (entry.path().extension().string() == ".png")
						files.push_back(entry.path().string());
				}
				indexExtrinsics = 0;
			}
			static ID3D11ShaderResourceView* pSRV = NULL;
			static std::vector<cv::Point3f> points3D;
			static std::vector<cv::Point2f> points2D;
			static std::vector<std::vector<cv::Point3f>> calib_points3Ds;
			static std::vector<std::vector<cv::Point2f>> calib_points2Ds;
			static int extrinsicWidth = 3, extrinsicHeight = 3;
			static int w, h;
			static bool isCalibratedExtrinsic = false;
			static glm::fmat4x4 matRB2WS, matWS2RB;
			static cv::Mat imgProcessing;

			if (ImGui::Button("Add Extrinsic", ImVec2(0, buttonHeight)))
			{
				bool errResult = false;

				glm::fquat q;
				glm::fvec3 t;
				if (trackInfo.GetRigidBodyQuatTVecByName("c-arm", &q, &t)) {
					glm::fmat4x4 mat_r = glm::toMat4(q);
					glm::fmat4x4 mat_t = glm::translate(t);
					matRB2WS = mat_t * mat_r;
					matWS2RB = glm::inverse(matRB2WS);
				}
				else {
					__gc.SetErrorCode("Failure to Detect C-Arm Rigidbody!");
					errResult = true;
				}

				if (!__gc.g_calribmodeToggle) {
					__gc.SetErrorCode("Must be Marker Selection Mode!");
					errResult = true;
				}
				else if (__gc.g_selectedMkNames.size() != 4) {
					__gc.SetErrorCode("4 markers must be selected!! Current Selection Markers {}", __gc.g_selectedMkNames.size());
					errResult = true;
				}

				if (!errResult)
				{
					cv::FileStorage fs(__gc.g_folder_trackingInfo + "calib_settings.txt", cv::FileStorage::Mode::READ);
					std::string calib_mode = "FMARKERS";
					std::string det_mode = "SimpleBlobDetector";
					if (fs.isOpened()) {
						fs["EXT_ROWS"] >> extrinsicHeight;
						fs["EXT_COLS"] >> extrinsicWidth;
						fs["EXT_MODE"] >> calib_mode;
						fs["EXT_DETECTOR"] >> det_mode;
						fs.release();
					}

					// gather 3D points 
					std::vector<cv::Point3f> posWsMarkers(__gc.g_selectedMkNames.size());
					for (auto it = __gc.g_selectedMkNames.begin(); it != __gc.g_selectedMkNames.end(); it++) {
						std::map<track_info::MKINFO, std::any> mkInfo;
						trackInfo.GetMarkerByName(it->first, mkInfo);
						glm::fvec3 pos = std::any_cast<glm::fvec3>(mkInfo[track_info::MKINFO::POSITION]);
						posWsMarkers[it->second] = *(cv::Point3f*)&pos;
					}

					glm::fvec3 v01 = *(glm::fvec3*)&posWsMarkers[1] - *(glm::fvec3*)&posWsMarkers[0];
					glm::fvec3 v03 = *(glm::fvec3*)&posWsMarkers[3] - *(glm::fvec3*)&posWsMarkers[0];
					glm::fvec3 selDir = glm::cross(v01, v03);
					glm::fvec3 posCenterCArmRB = vzmutils::transformPos(glm::fvec3(0, 0, 0), matRB2WS);
					glm::fvec3 cal2DetDir = posCenterCArmRB - *(glm::fvec3*)&posWsMarkers[0];
					//if (glm::dot(selDir, cal2DetDir) < 0)
					{
						//std::cout << "\ncorrecting the selecting direction!" << std::endl;
						// 이상함... 아래것으로 해야 하는데... 이렇게 하면 카메라 위치가 반대로 감...
						// 일단은 데모를 위해 주석 처리...
						// 0123 to 1032
						std::vector<cv::Point3f> posWsMarkersTmp = posWsMarkers;
						posWsMarkers[0] = posWsMarkersTmp[1];
						posWsMarkers[1] = posWsMarkersTmp[0];
						posWsMarkers[2] = posWsMarkersTmp[3];
						posWsMarkers[3] = posWsMarkersTmp[2];
					}

					mystudents::Get3DPostionsFromFMarkersPhantom(__cv3__ & posWsMarkers[0], __cv3__ & posWsMarkers[1], __cv3__ & posWsMarkers[2], __cv3__ & posWsMarkers[3],
							&trackInfo, extrinsicHeight, extrinsicWidth, points3D);

					if (points3D.size() != extrinsicHeight * extrinsicWidth) {
						__gc.SetErrorCode("4 markers must be selected!! Current Selection Markers {}", __gc.g_selectedMkNames.size());
						errResult = true;
					}
					else {
						__gc.g_testMKs.clear();
						__gc.g_testMKs.assign(points3D.size(), glm::fvec3(0));
						memcpy(&__gc.g_testMKs[0], &points3D[0], sizeof(glm::fvec3) * points3D.size());

						for (int i = 0; i < points3D.size(); i++) {
							*(glm::fvec3*)&points3D[i] = vzmutils::transformPos(*(glm::fvec3*)&points3D[i], matWS2RB);
						}
					}
				}
				
				if (!errResult) {
					cv::Mat img;
					if (__gc.g_optiRecordMode == OPTTRK_RECMODE::LOAD)
						img = cv::imread(files[indexExtrinsics]);
					else if (g_curScanGrayImg.data != NULL) {
						cv::cvtColor(g_curScanGrayImg, img, cv::COLOR_GRAY2RGB);
						cv::imwrite(__gc.g_folder_trackingInfo + "ext_images/" + __gc.g_lastStoredImageName + ".png", img);

						cv::FileStorage fs(__gc.g_folder_trackingInfo + "ext_images/" + __gc.g_lastStoredImageName + "_RBPts.txt", cv::FileStorage::Mode::WRITE);
						fs << "RB_POSITIONS" << cv::Mat(points3D);
						fs.release();
					}
					else {
						__gc.SetErrorCode("No Image!!");
						errResult = true;
					}

					// Load into a raw RGBA buffer
					if (img.data != NULL) {
						int image_width = img.cols;
						int image_height = img.rows;

						// optional
						cv::flip(img, img, 1);

						cv::Mat imgGray;
						cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
						// processing..

						std::vector<float> circleRadiis;
						mystudents::Get2DPostionsFromFMarkersPhantom(imgGray, extrinsicWidth, extrinsicHeight, points2D, 2000.f, 16000.f, 100.f, &circleRadiis);

						DrawCircles(points2D, circleRadiis, img);

						if (img.channels() == 3) {
							// First create the image with alpha channel
							cv::cvtColor(img, img, cv::COLOR_RGB2RGBA);
					
							// # Split the image for access to alpha channel
							std::vector<cv::Mat> channels(4);
							cv::split(img, channels);
						
							// Assign the mask to the last channel of the image
							channels[3] = 255;
						
							// Finally concat channels for rgba image
							cv::merge(channels, img);
						}

						__gc.g_engineLogger->info("image index : {}", indexExtrinsics);

						pSRV = LoadTextureFromFile(img.data, "EXTRINSIC_IMG", image_width, image_height);
						indexExtrinsics++;
						if (__gc.g_optiRecordMode == OPTTRK_RECMODE::LOAD) indexExtrinsics = indexExtrinsics % files.size();
						isCalibratedExtrinsic = false;

						cv::flip(img, imgProcessing, 1);
					}
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Apply Extrinsic", ImVec2(0, buttonHeight)))
			{
				if (points2D.size() != extrinsicWidth * extrinsicHeight) {
					__gc.SetErrorCode("Failure to detect the entire circles!");
				}
				if (isCalibratedExtrinsic) {
					__gc.SetErrorCode("The same scan image is already applied!");
				}
				else {
					isCalibratedExtrinsic = true;
					calib_points2Ds.push_back(points2D);
					calib_points3Ds.push_back(points3D);

					__gc.g_engineLogger->info("# of {}-pair sets : {}", extrinsicWidth * extrinsicHeight, calib_points2Ds.size());

					cv::Mat cameraMatrix, distCoeffs;
					cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
					if (fs.isOpened()) {
						fs["K"] >> cameraMatrix;
						fs["DistCoeffs"] >> distCoeffs;
						fs.release();
						
						for (int i = 0; i < points2D.size(); i++) {
							__gc.g_homographyPairs[__cv3__ & points3D[i]] = __cv2__ & points2D[i];
						}

						std::vector<cv::Point2f> global_points2d;
						std::vector<cv::Point3f> global_points3d;
						for (auto& _pair : __gc.g_homographyPairs) {
							global_points2d.push_back(*(cv::Point2f*)&(_pair.second));
							global_points3d.push_back(*(cv::Point3f*)&(_pair.first));
						}
						cv::Mat rvec, tvec;
						cv::solvePnP(global_points3d, global_points2d, cameraMatrix, distCoeffs, rvec, tvec);

						auto ComputeReprojErr = [](const std::vector<cv::Point2f>& pts2d, const std::vector<cv::Point2f>& pts2d_reproj, float& maxErr) {
							float avrDiff = 0;
							int numPts = (int)pts2d.size();
							for (int i = 0; i < (int)pts2d.size(); i++) {
								float diff = glm::distance(*(glm::fvec2*)&pts2d[i], *(glm::fvec2*)&pts2d_reproj[i]);
								maxErr = std::max(maxErr, diff);
								avrDiff += diff / (float)numPts;
							}
							return avrDiff;
						};

						// Reproject the 3D points onto the image plane using the camera calibration
						std::vector<cv::Point2f> imagePointsReprojected;
						cv::projectPoints(global_points3d, rvec, tvec, cameraMatrix, distCoeffs, imagePointsReprojected);

						// Compute the reprojection error
						float maxErr = 0;
						float reprojectionError = ComputeReprojErr(global_points2d, imagePointsReprojected, maxErr);

						__gc.g_engineLogger->info("reprojectionError of {} pairs : {}, max error : {}", global_points2d.size(), reprojectionError, maxErr);

						//maxErr = 0;
						//reprojectionError = 0;
						//for (int i = 0; i < (int)calib_points2Ds.size(); i++) {
						//	reprojectionError += ComputeReprojErr(points2D, imagePointsReprojected, maxErr);
						//}
						//reprojectionError /= (float)calib_points2Ds.size();
						//__gc.g_engineLogger->info("entire set ==> reprojectionError : {}, max error : {}", reprojectionError, maxErr);


						//matCArmRB2SourceCS
						cv::Mat matR;
						cv::Rodrigues(rvec, matR);
						// note, here camera frame (notation 'CA', opencv convention) is defined with
						// z axis as viewing direction
						// -y axis as up vector
						__gc.g_CArmRB2SourceCS = glm::fmat4x4(1);
						__gc.g_CArmRB2SourceCS[0][0] = (float)matR.at<double>(0, 0);
						__gc.g_CArmRB2SourceCS[0][1] = (float)matR.at<double>(1, 0);
						__gc.g_CArmRB2SourceCS[0][2] = (float)matR.at<double>(2, 0);
						__gc.g_CArmRB2SourceCS[1][0] = (float)matR.at<double>(0, 1);
						__gc.g_CArmRB2SourceCS[1][1] = (float)matR.at<double>(1, 1);
						__gc.g_CArmRB2SourceCS[1][2] = (float)matR.at<double>(2, 1);
						__gc.g_CArmRB2SourceCS[2][0] = (float)matR.at<double>(0, 2);
						__gc.g_CArmRB2SourceCS[2][1] = (float)matR.at<double>(1, 2);
						__gc.g_CArmRB2SourceCS[2][2] = (float)matR.at<double>(2, 2);
						__gc.g_CArmRB2SourceCS[3][0] = (float)((double*)tvec.data)[0];
						__gc.g_CArmRB2SourceCS[3][1] = (float)((double*)tvec.data)[1];
						__gc.g_CArmRB2SourceCS[3][2] = (float)((double*)tvec.data)[2];

						cv::FileStorage fs(__gc.g_folder_trackingInfo + "rb2carm1.txt", cv::FileStorage::Mode::WRITE);
						fs.write("rvec", rvec);
						fs.write("tvec", tvec);
						fs.release();

						___SaveAndChangeViewState(char('1'), sidScene, cidRender1, 0, &matRB2WS, &imgProcessing);
					}
					else {
						__gc.SetErrorCode("Cannot load the Intrinsics!!");
					}
				}
			}
			if (pSRV != NULL) {
				ImVec2 pos = ImGui::GetCursorScreenPos();
				ImGui::Image(pSRV, ImVec2(g_sizeRightPanelW - 60, g_sizeRightPanelW - 60), ImVec2(0, 0), ImVec2(1, 1));
			}
		}

		if (ImGui::Button("Shot Pose Capture", ImVec2(g_sizeRightPanelW - 10, buttonHeight)))
		{
			__gc.g_optiEvent = OPTTRK_THREAD::C_ARM_SHOT_MOMENT;
		}


		ImGui::Text("Render Count = %d", vzmpf::GetRenderCount());
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
		ImGui::Text((std::string("Network task : ") + __gc.g_networkState).c_str());
		ImGui::End();
	}

	//bool debugWindowShow = false;
	//ImGui::ShowDebugLogWindow(&debugWindowShow);

	// 3. Show another simple window.
	if (ImGui::CollapsingHeader(view1.GetWindowName().c_str(), ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::SetNextWindowPos(ImVec2(5, 5), ImGuiCond_Once);

		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
		ImGui::PushStyleColor(ImGuiCol_WindowBg, clear_color); // Set window background to 
		ImGui::Begin(view1.GetWindowName().c_str());   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::PopStyleVar();
		ImGui::PopStyleColor();
		view1.View(NULL);
		ImGui::End();
	}

	if (ImGui::CollapsingHeader(view2.GetWindowName().c_str(), ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::SetNextWindowPos(ImVec2((g_sizeWinX - g_sizeRightPanelW) / 2 + 10, 5), ImGuiCond_Once);

		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
		ImGui::PushStyleColor(ImGuiCol_WindowBg, clear_color); // Set window background to 
		ImGui::Begin(view2.GetWindowName().c_str());   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::PopStyleVar();
		ImGui::PopStyleColor();
		view2.View(NULL);
		ImGui::End();
	}
}

int main()
{
	EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, 0);
	WNDCLASSEXW wc = { sizeof(wc), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr, L"ImGui Example", nullptr };
	::RegisterClassExW(&wc);
	hwnd = ::CreateWindowW(wc.lpszClassName, L"SNAVI SW ver 1.0", WS_OVERLAPPEDWINDOW, g_posWinX, g_posWinY, g_sizeWinX, g_sizeWinY - 50, nullptr, nullptr, wc.hInstance, nullptr);

	// Initialize Direct3D
	if (!CreateDeviceD3D(hwnd))
	{
		CleanupDeviceD3D();
		::UnregisterClassW(wc.lpszClassName, wc.hInstance);
		return 1;
	}

	// Show the window
	::ShowWindow(hwnd, SW_SHOWDEFAULT);
	::UpdateWindow(hwnd);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsLight();

	// Setup Platform/Renderer backends
	ImGui_ImplWin32_Init(hwnd);
	ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

	// Load Fonts
	// - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
	// - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
	// - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
	// - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
	// - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
	// - Read 'docs/FONTS.md' for more instructions and details.
	// - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
	//io.Fonts->AddFontDefault();
	//io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
	//ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
	//IM_ASSERT(font != nullptr);

	vmgui::SetImGuiContext(ImGui::GetCurrentContext());
	vmgui::ImTermWindow viewTerm;

	// Our state
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	// https://fsunuc.physics.fsu.edu/git/gwm17/imgui/src/commit/7ac16c02cc1e1c6bcbdecdd5ac0746b157395084/docs/FONTS.md
	//ImFont* font1 = io.Fonts->AddFontFromFileTTF("c:/windows/fonts/arial.ttf", 20);
	ImFontConfig config;
	config.OversampleH = 2;
	config.OversampleV = 2;
	config.GlyphExtraSpacing.x = 1.0f;
	ImFont* font1 = io.Fonts->AddFontDefault();
	ImFont* font2 = io.Fonts->AddFontFromFileTTF("c:/windows/fonts/arial.ttf", 12, &config);
	ImFont* font3 = io.Fonts->AddFontFromFileTTF("c:/windows/fonts/arial.ttf", 30, &config);

	__gc.Init();
	__gc.g_folder_data = GetDataPath();
	__gc.g_configFileName = __gc.g_folder_data + "config.txt";

	cv::FileStorage config_fs(__gc.g_configFileName, cv::FileStorage::Mode::READ);
	std::string configLoad, motiveProfile, trackDataFolder;
	if (config_fs.isOpened()) {
		config_fs["TrackDataFolder"] >> trackDataFolder;
		config_fs["MotiveProfile"] >> motiveProfile;
		config_fs["RecordMode"] >> configLoad;
		config_fs["RecordPeriod"] >> __gc.g_optiRecordPeriod;
		config_fs["PivotSamples"] >> __gc.g_optiPivotSamples;
		std::cout << (int)__gc.g_optiPivotSamples << std::endl;
	}
	else {
		trackDataFolder = "Tracking 2023-10-05";
		motiveProfile = "Motive Profile - 2023-09-15.motive";
	}
	config_fs.release();

	if (configLoad == "LOAD") {
		__gc.g_optiRecordMode = OPTTRK_RECMODE::LOAD;
	}
	__gc.g_profileFileName = __gc.g_folder_data + motiveProfile;
	__gc.g_folder_trackingInfo = __gc.g_folder_data + trackDataFolder + "/";
	__gc.g_recFileName = __gc.g_folder_trackingInfo + "TrackingRecord.csv";
	__gc.g_recScanName = __gc.g_folder_trackingInfo + "ScanRecord.csv";

	rendertask::InitializeTask(&__gc);
	nettask::InitializeTask(&__gc);
	trackingtask::InitializeTask(&__gc);
	calibtask::InitializeTask(&__gc);

	//vzm::InitEngineLib();
	vzm::InitEngineLib("SpineNavi");
	//vzm::SetLogConfiguration(true, 4);

	std::shared_ptr<void> nativeLogger;
	vzm::GetLoggerPointer(nativeLogger);
	__gc.g_engineLogger = std::static_pointer_cast<spdlog::logger, void>(nativeLogger);

	if (__gc.g_optiRecordMode != OPTTRK_RECMODE::LOAD) {
		bool optitrkMode = optitrk::InitOptiTrackLib();
		optitrk::LoadProfileAndCalibInfo(__gc.g_profileFileName, "");

		//optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-08-06.motive", folder_data + "System Calibration.cal");
		//optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-07-04.motive", folder_data + "System Calibration.cal");

		optitrk::SetCameraSettings(0, 4, 16, 200);
		optitrk::SetCameraSettings(1, 4, 16, 200);
		optitrk::SetCameraSettings(2, 4, 16, 200);
	}

	rendertask::SceneInit((g_sizeWinX - g_sizeRightPanelW) / 2, g_sizeWinY - g_sizeConsolePanelH - 30);

	int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	int cidRender1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
	int cidRender2 = vzmutils::GetSceneItemIdByName(__gc.g_camName2);
	vzm::RenderScene(sidScene, cidRender1);
	vzm::RenderScene(sidScene, cidRender2);

	std::thread tracker_processing_thread(trackingtask::OptiTrackingProcess);
	std::thread network_processing_thread(nettask::NetworkProcess);
	// Main loop
	bool done = false;
	while (!done)
	{
		// Poll and handle messages (inputs, window resize, etc.)
		// See the WndProc() function below for our to dispatch events to the Win32 backend.
		MSG msg;
		while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
		{
			::TranslateMessage(&msg);
			::DispatchMessage(&msg);
			if (msg.message == WM_QUIT)
				done = true;
		}
		if (done)
			break;

		// Handle window resize (we don't resize directly in the WM_SIZE handler)
		if (g_ResizeWidth != 0 && g_ResizeHeight != 0)
		{
			CleanupRenderTarget();
			g_pSwapChain->ResizeBuffers(0, g_ResizeWidth, g_ResizeHeight, DXGI_FORMAT_UNKNOWN, 0);
			g_ResizeWidth = g_ResizeHeight = 0;
			CreateRenderTarget();
		}

		// Start the Dear ImGui frame
		ImGui_ImplDX11_NewFrame();
		ImGui_ImplWin32_NewFrame();
		ImGui::NewFrame();

		mygui(io);

		ImGui::SetNextWindowPos(ImVec2(5, g_sizeWinY - g_sizeConsolePanelH), ImGuiCond_Once);
		//ImGui::SetNextWindowSize(ImVec2(g_sizeWinX - 10, g_sizeConsolePanelH - 50), ImGuiCond_Once); // not work
		viewTerm.ShowTerminal();

		// Rendering
		ImGui::Render();
		const float clear_color_with_alpha[4] = { clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w };
		g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
		g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color_with_alpha);
		ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

		g_pSwapChain->Present(1, 0); // Present with vsync
		//g_pSwapChain->Present(0, 0); // Present without vsync
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

	// Cleanup
	ImGui_ImplDX11_Shutdown();
	ImGui_ImplWin32_Shutdown();
	ImGui::DestroyContext();

	for (auto kv : g_mapImgSRVs)
		kv.second->Release();

	CleanupDeviceD3D();
	::DestroyWindow(hwnd);
	::UnregisterClassW(wc.lpszClassName, wc.hInstance);

	return 0;
}

// Helper functions

bool CreateDeviceD3D(HWND hWnd)
{
	// Setup swap chain
	DXGI_SWAP_CHAIN_DESC sd;
	ZeroMemory(&sd, sizeof(sd));
	sd.BufferCount = 2;
	sd.BufferDesc.Width = 0;
	sd.BufferDesc.Height = 0;
	sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.BufferDesc.RefreshRate.Numerator = 60;
	sd.BufferDesc.RefreshRate.Denominator = 1;
	sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
	sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.OutputWindow = hWnd;
	sd.SampleDesc.Count = 1;
	sd.SampleDesc.Quality = 0;
	sd.Windowed = TRUE;
	sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

	UINT createDeviceFlags = 0;
	//createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
	D3D_FEATURE_LEVEL featureLevel;
	const D3D_FEATURE_LEVEL featureLevelArray[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0, };
	HRESULT res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
	if (res == DXGI_ERROR_UNSUPPORTED) // Try high-performance WARP software driver if hardware is not available.
		res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
	if (res != S_OK)
		return false;

	CreateRenderTarget();
	return true;
}

void CleanupDeviceD3D()
{
	CleanupRenderTarget();
	if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
	if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
	if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}

void CreateRenderTarget()
{
	ID3D11Texture2D* pBackBuffer;
	g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
	g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
	pBackBuffer->Release();
}

void CleanupRenderTarget()
{
	if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
}

// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Win32 message handler
// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
		return true;

	switch (msg)
	{
	case WM_SIZE:
		if (wParam == SIZE_MINIMIZED)
			return 0;
		g_ResizeWidth = (UINT)LOWORD(lParam); // Queue resize
		g_ResizeHeight = (UINT)HIWORD(lParam);
		return 0;
	case WM_SYSCOMMAND:
		if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
			return 0;
		break;
	case WM_DESTROY:
		::PostQuitMessage(0);
		return 0;
	}
	return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}
