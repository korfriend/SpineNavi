#pragma once

#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx11.h"
#include <d3d11.h>
#include <tchar.h>

#include "GlobalParams.h"

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

// Data
static ID3D11Device* g_pd3dDevice = nullptr;
static ID3D11DeviceContext* g_pd3dDeviceContext = nullptr;
static IDXGISwapChain* g_pSwapChain = nullptr;
static UINT                     g_ResizeWidth = 0, g_ResizeHeight = 0;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;

static cv::Mat g_curScanGrayImg(SCANIMG_W, SCANIMG_H, CV_8UC1);
static __GC __gc;
static int g_posWinX = 0, g_posWinY = 0;
static int g_sizeWinX = 1280, g_sizeWinY = 800;
static int g_sizeRightPanelW = 500, g_sizeConsolePanelH = 290;
static std::map<std::string, ID3D11ShaderResourceView*> g_mapImgSRVs;

namespace opcode {

	static int sidScene, cidRender1, cidRender2;
	static vzm::CameraParameters cpCam1, cpCam2;
	static int buttonHeight;
	static track_info trackInfo;

	auto ComputeRefCArmPhase = [](float& dist, float& angle) {
		glm::fquat q_0, q_1;
		glm::fvec3 t_0, t_1;
		if (trackInfo.GetRigidBodyQuatTVecByName("c-arm", &q_0, &t_0) &&
			trackInfo.GetRigidBodyQuatTVecByName("checker", &q_1, &t_1))
		{
			dist = glm::distance(t_0, t_1);
			glm::fquat q_diff = q_0 * glm::inverse(q_1);
			angle = glm::degrees(2.0f * acos(q_diff.w));
			return true;
		}
		return false;
	};
	auto DrawCircles = [](const std::vector<cv::Point2f>& points2Ds, std::vector<float>& circleRadiis, const int thickness, cv::Mat& img) {
		for (int i = 0; i < (int)points2Ds.size(); i++) {
			cv::Point2f center = points2Ds[i];
			float x = (center.x); // cvRound
			float y = (center.y);
			float r = (circleRadiis[i]);

			cv::circle(img, cv::Point(x, y), r, cv::Scalar(0, 255, 255), thickness);
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

	void ComputePoseMatrixFromRTVec(const cv::Mat& rvec, const cv::Mat& tvec, glm::fmat4x4& matRB2CA, glm::fmat4x4& matCA2RB)
	{
		cv::Mat matR;
		cv::Rodrigues(rvec, matR);
		// note, here camera frame (notation 'CA', opencv convention) is defined with
		// z axis as viewing direction
		// -y axis as up vector
		matRB2CA = glm::fmat4x4(1);
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
	}

	// viewIdx (1) refers to AP, viewIndex (2) to LL
	bool LoadScanInfo(const int viewIdx, const std::string& timePack,
		cv::Mat& K, cv::Mat& DistCoeffs, glm::fmat4x4& matRB2CA, glm::fmat4x4& matCA2RB, glm::fmat4x4& matRB2WS, std::string& imgFileName)
	{
		if (viewIdx != 1 && viewIdx != 2)
			__gc.g_engineLogger->error("viewIdx should be 1 or 2, not {}", viewIdx);

		std::string scanGeoFileName = __gc.g_folder_trackingInfo + "test" + std::to_string(viewIdx) + "_" + timePack + ".txt";

		cv::FileStorage fs(scanGeoFileName, cv::FileStorage::Mode::READ);
		if (!fs.isOpened())
			return false;

		fs["K"] >> K;
		fs["DistCoeffs"] >> DistCoeffs;
		fs["imgFile"] >> imgFileName;

		cv::Mat rvec, tvec, rb2wsMat;
		fs["rvec"] >> rvec;
		fs["tvec"] >> tvec;
		fs["rb2wsMat"] >> rb2wsMat; // memory storage ordering in opengl
		fs.release();

		ComputePoseMatrixFromRTVec(rvec, tvec, matRB2CA, matCA2RB);
		memcpy(glm::value_ptr(matRB2WS), rb2wsMat.ptr(), sizeof(float) * 16);

		return true;
	}

	void SetAnimation(const int cidCam, const int viewIdx, const cv::Mat& K, const int CArmPlaneIdx = -1)
	{
		// DOJO : 스캔 시 (carmIdx) 저장된 이미지(이미지 texture 가 포함된 plane actor)에 fitting 되도록 cpInterCams (by slerp) 지정 
		// slerp 의 시작점은 현재 cidCam 카메라의 pose, 끝점은 스캔 시 (carmIdx) 저장된 이미지 fitting 카메라 pose
		int aidCArmPlane = vzmutils::GetSceneItemIdByName("CArm Plane:" + std::to_string(CArmPlaneIdx < 0 ? viewIdx : CArmPlaneIdx));
		if (aidCArmPlane == 0)
			__gc.g_engineLogger->error("CArm Plane is not defined!");
		else {
			vzm::ActorParameters apCArmPlane;
			vzm::GetActorParams(aidCArmPlane, apCArmPlane);
			glm::fmat4x4 matCA2WS = apCArmPlane.script_params.GetParam("matCA2WS", glm::fmat4x4(1));

			rendertask::SetAnimationCamTo(cidCam, viewIdx - 1,
				K.at<double>(0, 0), K.at<double>(1, 1), K.at<double>(0, 1), K.at<double>(0, 2), K.at<double>(1, 2),
				matCA2WS);
		}
	}

	void SetScan2View(const int viewIdx, const std::string& timePack) {
		std::string imgFileName;
		cv::Mat K, D;
		glm::fmat4x4 matRB2CA, matCA2RB, matRB2WS;
		LoadScanInfo(viewIdx, timePack, K, D, matRB2CA, matCA2RB, matRB2WS, imgFileName);

		auto it = __gc.g_mapAidGroupCArmCam.find(viewIdx);
		if (it != __gc.g_mapAidGroupCArmCam.end())
			vzm::RemoveSceneItem(it->second, true);

		cv::Mat imgCArm = cv::imread(__gc.g_folder_trackingInfo + imgFileName);
		cv::flip(imgCArm, imgCArm, 1);
		int aidGroup = calibtask::RegisterCArmImage(sidScene, viewIdx, K, D, matCA2RB, matRB2WS, imgCArm);
		__gc.g_mapAidGroupCArmCam[viewIdx] = aidGroup;
		SetAnimation(viewIdx == 1 ? cidRender1 : cidRender2, viewIdx, K);
	}

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
	}

	void TrackInfo2Scene()
	{
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

					cv::FileStorage fs(__gc.g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
					if (fs.isOpened()) {
						glm::fvec3 posInit[2], viewInit[2], upInit[2];
						float fx[2], fy[2], sc[2], cx[2], cy[2];
						cv::Mat ocvVec3;
						for (int i = 0; i < 2; i++) {
							fs[i == 0 ? "POS" : "POS2"] >> ocvVec3;
							memcpy(&posInit[i], ocvVec3.ptr(), sizeof(float) * 3);
							fs[i == 0 ? "VIEW" : "VIEW2"] >> ocvVec3;
							memcpy(&viewInit[i], ocvVec3.ptr(), sizeof(float) * 3);
							fs[i == 0 ? "UP" : "UP2"] >> ocvVec3;
							memcpy(&upInit[i], ocvVec3.ptr(), sizeof(float) * 3);
							fs[i == 0 ? "fx" : "fx2"] >> fx[i];
							fs[i == 0 ? "fy" : "fy2"] >> fy[i];
							fs[i == 0 ? "sc" : "sc2"] >> sc[i];
							fs[i == 0 ? "cx" : "cx2"] >> cx[i];
							fs[i == 0 ? "cy" : "cy2"] >> cy[i];
						}
						fs.release();

						//float vFov = 3.141592654f / 4.f;
						int cidCams[2] = { cidRender1 , cidRender2 };
						for (int i = 0; i < 2; i++) {
							vzm::CameraParameters cpCam;
							vzm::GetCameraParams(cidCams[i], cpCam);

							//float aspect_ratio = (float)cpCam.w / (float)cpCam.h;
							//float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;
							cpCam.fx = fx[i]; //cpCam.w / (2.f * tan(hFov / 2.f));
							cpCam.fy = fy[i]; //cpCam.h / (2.f * tan(vFov / 2.f));
							cpCam.sc = sc[i]; //0;
							cpCam.cx = cx[i]; //cpCam.w / 2.f;
							cpCam.cy = cy[i]; //cpCam.h / 2.f;
							cpCam.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
							*(glm::fvec3*)cpCam.pos = posInit[i];
							*(glm::fvec3*)cpCam.view = viewInit[i];
							*(glm::fvec3*)cpCam.up = upInit[i];
							vzm::SetCameraParams(cidCams[i], cpCam);
						}
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

				int viewIdx = isAP ? 1 : 2;
				SetScan2View(viewIdx, timePack);
			}
		}
		else {
			recRowCount = -1;
			if (__gc.g_renderEvent == RENDER_THREAD::DOWNLOAD_IMG_PROCESS) {
				//__gc.g_renderEvent = RENDER_THREAD::BUSY;
				memcpy(g_curScanGrayImg.ptr(), &__gc.g_downloadImgBuffer[0], __gc.g_downloadImgBuffer.size());

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

					glm::fmat4x4 matSourceCS2WS = matCArmRB2WS * glm::inverse(__gc.g_CArmRB2SourceCS);
					glm::fvec3 dirCArm = vzmutils::transformVec(glm::fvec3(0, 0, -1), matSourceCS2WS);
					dirCArm = glm::normalize(dirCArm);

					glm::fmat4x4 matCam2WS;
					trackInfo.GetTrackingCamPose(0, matCam2WS);
					glm::fvec3 camRight = vzmutils::transformVec(glm::fvec3(1, 0, 0), matCam2WS);
					camRight = glm::normalize(camRight);

					float horizonAngle = fabs(glm::dot(camRight, dirCArm));
					bool isAP = horizonAngle < 0.3f;
					int viewIdx = isAP ? 1 : 2;

					// OPTI TRACK MODE //
					std::string timePack = std::to_string(navihelpers::GetCurrentTimePack());
					__gc.g_lastStoredImageName = "test" + std::to_string(viewIdx) + "_" + timePack;

					cv::Mat imgCArm;
					cv::cvtColor(g_curScanGrayImg, imgCArm, cv::COLOR_GRAY2RGB);				
					cv::imwrite(__gc.g_folder_trackingInfo + __gc.g_lastStoredImageName + ".png", imgCArm);

					cv::Mat K, D, rvec, tvec;
					cv::FileStorage fs_intrinsic(__gc.g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
					cv::FileStorage fs_extrinsic(__gc.g_folder_trackingInfo + "rb2carm1.txt", cv::FileStorage::Mode::READ);
					if (fs_intrinsic.isOpened()) {
						fs_intrinsic["K"] >> K;
						fs_intrinsic["DistCoeffs"] >> D;
						fs_intrinsic.release();
					}
					else {
						__gc.g_engineLogger->error("No carm_intrinsics.txt!!");
					}
					if (fs_extrinsic.isOpened()) {
						fs_extrinsic["rvec"] >> rvec;
						fs_extrinsic["tvec"] >> tvec;
						fs_extrinsic.release();
					}
					else {
						__gc.g_engineLogger->error("No rb2carm1.txt!!");
					}
					cv::FileStorage fs(__gc.g_folder_trackingInfo + __gc.g_lastStoredImageName + ".txt", cv::FileStorage::Mode::WRITE);
					fs << "K" << K;
					fs << "DistCoeffs" << D;
					fs << "rvec" << rvec;
					fs << "tvec" << tvec;
					cv::Mat ocvRb(4, 4, CV_32FC1);
					memcpy(ocvRb.ptr(), glm::value_ptr(matCArmRB2WS), sizeof(float) * 16);
					fs << "rb2wsMat" << ocvRb;
					fs << "imgFile" << __gc.g_lastStoredImageName + ".png";

					glm::fmat4x4 matRB2CA, matCA2RB;
					ComputePoseMatrixFromRTVec(rvec, tvec, matRB2CA, matCA2RB);

					if (__gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD) {
						if (imgCArm.empty()) {
							__gc.SetErrorCode("Not Allowed Calibration Image during Recording");
							__gc.g_optiRecordMode = OPTTRK_RECMODE::NONE;
						}
						if (!__gc.g_recScanStream.is_open()) {
							__gc.g_recScanStream.open(__gc.g_recScanName);
							if (!__gc.g_recScanStream.is_open()) {
								__gc.SetErrorCode("Invalid Rec File!");
								__gc.g_optiRecordMode = OPTTRK_RECMODE::NONE;
							}

							std::vector<std::vector<std::string>> csvData(1);
							std::vector<std::string>& csvRow = csvData[0];
							csvRow = { "FRAME", "TIME", "AP/LL" };

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

							std::vector<std::string> csvRow = { to_string(__gc.g_optiRecordFrame), timePack, viewIdx == 1 ? "AP" : "LL" };

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
					
					cv::flip(imgCArm, imgCArm, 1);

					auto it = __gc.g_mapAidGroupCArmCam.find(viewIdx);
					if (it != __gc.g_mapAidGroupCArmCam.end())
						vzm::RemoveSceneItem(it->second, true);
					int aidGroup = calibtask::RegisterCArmImage(sidScene, viewIdx, K, D, matCA2RB, matCArmRB2WS, imgCArm);
					__gc.g_mapAidGroupCArmCam[viewIdx] = aidGroup;
					SetAnimation(isAP ? cidRender1 : cidRender2, viewIdx, K);
				}
				else {
					PlaySound((LPCTSTR)SND_ALIAS_SYSTEMWELCOME, NULL, SND_ALIAS_ID | SND_ASYNC);
					__gc.SetErrorCode("C-Arm is Not Detected!\nCheck the C-Arm Markers and Retry!", 300);
				}

				__gc.g_renderEvent = RENDER_THREAD::FREE;
			}
		}

		rendertask::UpdateTrackInfo2Scene(trackInfo);
	}

	void ControlWidgets(const ImVec4& clear_color)
	{
		ImGui::ColorEdit4("background color", (float*)&clear_color); // Edit 4 floats representing a color

		static bool showGridView1 = true, showGridView2 = true;
		ImGui::Checkbox("View1: Show Grid", &showGridView1);
		ImGui::SameLine();
		ImGui::Checkbox("View2: Show Grid", &showGridView2);
		int aidWorldGrid = vzmutils::GetSceneItemIdByName("World Grid");
		if (showGridView1) cpCam1.DeactiveHiddenActor(aidWorldGrid);
		else  cpCam1.HideActor(aidWorldGrid);
		if (showGridView2) cpCam2.DeactiveHiddenActor(aidWorldGrid);
		else  cpCam2.HideActor(aidWorldGrid);
		vzm::SetCameraParams(cidRender1, cpCam1);
		vzm::SetCameraParams(cidRender2, cpCam2);

		if (ImGui::Button("Save Cam", ImVec2(0, buttonHeight)))
		{
			cv::FileStorage fs(__gc.g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::WRITE);
			cv::Mat ocvVec3(1, 3, CV_32FC1);
			memcpy(ocvVec3.ptr(), cpCam1.pos, sizeof(float) * 3);
			fs.write("POS", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam1.view, sizeof(float) * 3);
			fs.write("VIEW", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam1.up, sizeof(float) * 3);
			fs.write("UP", ocvVec3);
			fs.write("fx", cpCam1.fx);
			fs.write("fy", cpCam1.fy);
			fs.write("sc", cpCam1.sc);
			fs.write("cx", cpCam1.cx);
			fs.write("cy", cpCam1.cy);

			memcpy(ocvVec3.ptr(), cpCam2.pos, sizeof(float) * 3);
			fs.write("POS2", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam2.view, sizeof(float) * 3);
			fs.write("VIEW2", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam2.up, sizeof(float) * 3);
			fs.write("UP2", ocvVec3);
			fs.write("fx2", cpCam2.fx);
			fs.write("fy2", cpCam2.fy);
			fs.write("sc2", cpCam2.sc);
			fs.write("cx2", cpCam2.cx);
			fs.write("cy2", cpCam2.cy);
			fs.release();
		}
		ImGui::SameLine();
		if (ImGui::Button("Load Cam", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_markerSelectionMode) {
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
					fs["fx"] >> cpCam1.fx;
					fs["fy"] >> cpCam1.fy;
					fs["sc"] >> cpCam1.sc;
					fs["cx"] >> cpCam1.cx;
					fs["cy"] >> cpCam1.cy;

					fs["POS2"] >> ocvVec3;
					memcpy(cpCam2.pos, ocvVec3.ptr(), sizeof(float) * 3);
					fs["VIEW2"] >> ocvVec3;
					memcpy(cpCam2.view, ocvVec3.ptr(), sizeof(float) * 3);
					fs["UP2"] >> ocvVec3;
					memcpy(cpCam2.up, ocvVec3.ptr(), sizeof(float) * 3);
					fs["fx2"] >> cpCam2.fx;
					fs["fy2"] >> cpCam2.fy;
					fs["sc2"] >> cpCam2.sc;
					fs["cx2"] >> cpCam2.cx;
					fs["cy2"] >> cpCam2.cy;
					fs.release();
				}
				vzm::SetCameraParams(cidRender1, cpCam1);
				vzm::SetCameraParams(cidRender2, cpCam2);
			}
		}
		//ImGui::SameLine();
		//if (ImGui::Button("Delete Scans", ImVec2(0, buttonHeight)))
		//{
		//	for (int i = 0; i < 9; i++) {
		//		auto it = __gc.g_mapAidGroupCArmCam.find(i + 1);
		//		if (it != __gc.g_mapAidGroupCArmCam.end())
		//			vzm::RemoveSceneItem(it->second, true);
		//	}
		//	__gc.g_mapAidGroupCArmCam.clear();
		//
		//	__gc.g_testMKs.clear();
		//	__gc.g_selectedMkNames.clear();
		//	int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
		//	if (aidTestGroup != 0)
		//		vzm::RemoveSceneItem(aidTestGroup);
		//}

		if (__gc.g_optiRecordMode != OPTTRK_RECMODE::CALIB_EDIT) {
			static bool hideAP = false, hideLL = false;
			auto GetTimePackFromFiles = [](const int viewIdx) {
				if (viewIdx != 1 && viewIdx != 2) {
					__gc.g_engineLogger->error("viewIdx should be 1 or 2, not {}", viewIdx);
					return std::string("");
				}

				using namespace std;
				using std::filesystem::directory_iterator;

				set<unsigned long long> latestTimePacks;
				for (const auto& file : directory_iterator(__gc.g_folder_trackingInfo)) {
					const std::string ss = file.path().u8string();
					//char* ptr = std::strrchr(file.path().c_str(), '\\');     //문자열(path)의 뒤에서부터 '\'의 위치를 검색하여 반환

					std::filesystem::path filePath = file.path().filename();
					if (filePath.extension() == ".png") {
						std::string fn = filePath.u8string();
						if (fn.find("test" + to_string(viewIdx)) != std::string::npos) {
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

				return std::to_string(*latestTimePacks.rbegin());
			};
			if (ImGui::Button("View AP", ImVec2(0, buttonHeight)))
			{
				hideAP = false;
				std::string timePack = GetTimePackFromFiles(1);
				if (timePack == "") {
					__gc.SetErrorCode("There is No AP file!");
				}
				else {
					SetScan2View(1, timePack);
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("View LL", ImVec2(0, buttonHeight)))
			{
				hideLL = false;
				std::string timePack = GetTimePackFromFiles(2);
				if (timePack == "") {
					__gc.SetErrorCode("There is No LL file!");
				}
				else {
					SetScan2View(2, timePack);
				}
			}
			ImGui::SameLine();
			if (ImGui::Button(hideAP ? "Show AP Image" : "Hide AP Image", ImVec2(0, buttonHeight)))
			{
				hideAP = !hideAP;
				int aidImgPlane = vzmutils::GetSceneItemIdByName("CArm Plane:1");
				if (aidImgPlane != 0) {
					vzm::ActorParameters apImg;
					vzm::GetActorParams(aidImgPlane, apImg);
					apImg.is_visible = !hideAP;
					vzm::SetActorParams(aidImgPlane, apImg);
				}
			}
			ImGui::SameLine();
			if (ImGui::Button(hideLL ? "Show LL Image" : "Hide LL Image", ImVec2(0, buttonHeight)))
			{
				hideLL = !hideLL;
				int aidImgPlane = vzmutils::GetSceneItemIdByName("CArm Plane:2");
				if (aidImgPlane != 0) {
					vzm::ActorParameters apImg;
					vzm::GetActorParams(aidImgPlane, apImg);
					apImg.is_visible = !hideLL;
					vzm::SetActorParams(aidImgPlane, apImg);
				}
			}
		}

		if (__gc.g_optiRecordMode != OPTTRK_RECMODE::LOAD) {
			ImGui::SeparatorText("Image Processing Display:");
			if (ImGui::Button("Key")) ImGui::SetKeyboardFocusHere();
			ImGui::SameLine();
			ImGui::SetNextItemWidth(200);
			ImGui::SliderInt("Circle Line Thickness", &__gc.g_circleThickness, 1, 5);
		}
	}

	void TrackerOperation(vmgui::ImGuiVzmWindow& view1, vmgui::ImGuiVzmWindow& view2)
	{
		ImGui::SeparatorText("Marker Selection:");
		if (ImGui::Button(__gc.g_markerSelectionMode ? "Selection OFF" : "Selection ON", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else {

				__gc.g_markerSelectionMode = !__gc.g_markerSelectionMode;

				if (!__gc.g_markerSelectionMode) {
					view1.EnableCustomDown(false);
					__gc.g_testMKs.clear();
					__gc.g_selectedMkNames.clear();
					int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
					if (aidTestGroup != 0)
						vzm::RemoveSceneItem(aidTestGroup);
				}
				else
					view1.EnableCustomDown(true);

				__gc.g_engineLogger->info("Marker Selection mode " + __gc.g_markerSelectionMode ? "ON" : "OFF");

				using namespace glm;
				fmat4x4 matCam2WS;
				trackInfo.GetTrackingCamPose(0, matCam2WS);

				*(fvec3*)cpCam1.pos = vzmutils::transformPos(fvec3(0, 0, 0), matCam2WS);
				*(fvec3*)cpCam1.view = vzmutils::transformVec(fvec3(0, 0, -1), matCam2WS);
				*(fvec3*)cpCam1.up = vzmutils::transformVec(fvec3(0, 1, 0), matCam2WS);

				float vFov = 3.141592654f / 4.f;
				float aspect_ratio = (float)cpCam1.w / (float)cpCam1.h;
				float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;

				//  fy = h/(2 * tan(f_y / 2)) 
				cpCam1.fx = cpCam1.w / (2.f * tan(hFov / 2.f));
				cpCam1.fy = cpCam1.h / (2.f * tan(vFov / 2.f));
				cpCam1.sc = 0;
				cpCam1.cx = cpCam1.w / 2.f;
				cpCam1.cy = cpCam1.h / 2.f;
				cpCam1.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
				cpCam1.np = 0.15f;

				vzm::SetCameraParams(cidRender1, cpCam1);
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Clear Markers", ImVec2(0, buttonHeight)))
		{
			__gc.g_selectedMkNames.clear();
		}

		ImGui::SeparatorText("Register:");
		if (ImGui::Button("C-Arm", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (!__gc.g_markerSelectionMode) {
				__gc.SetErrorCode("No Marker Selection!");
			}
			else {
				// update "c-arm" RB
				int numSelectedMKs = (int)__gc.g_selectedMkNames.size();
				if (numSelectedMKs < 3) {
					__gc.SetErrorCode("At least 3 Points are Needed!");
				}

				__gc.g_optiEvent = OPTTRK_THREAD::C_ARM_REGISTER;

				while (__gc.g_optiEvent == OPTTRK_THREAD::C_ARM_REGISTER) { Sleep(2); }
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("T-Niddle", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (!__gc.g_markerSelectionMode) {
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
		if (ImGui::Button("Troca", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (!__gc.g_markerSelectionMode) {
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
		ImGui::SameLine();
		if (ImGui::Button("Calibration Rig", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (!__gc.g_markerSelectionMode) {
				__gc.SetErrorCode("No Marker Selection!");
			}
			else {
				int numSelectedMKs = (int)__gc.g_selectedMkNames.size();
				if (numSelectedMKs < 3) {
					__gc.SetErrorCode("At least 3 Points are Needed!");
				}

				__gc.g_optiEvent = OPTTRK_THREAD::CALIB_REGISTER;

				while (__gc.g_optiEvent == OPTTRK_THREAD::CALIB_REGISTER) { Sleep(2); }
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Checker", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (!__gc.g_markerSelectionMode) {
				__gc.SetErrorCode("No Marker Selection!");
			}
			else {
				int numSelectedMKs = (int)__gc.g_selectedMkNames.size();
				if (numSelectedMKs < 3) {
					__gc.SetErrorCode("At least 3 Points are Needed!");
				}

				__gc.g_optiEvent = OPTTRK_THREAD::CHECKER_REGISTER;

				while (__gc.g_optiEvent == OPTTRK_THREAD::CHECKER_REGISTER) { Sleep(2); }
			}
		}

		ImGui::SeparatorText("Pivoting:");
		if (ImGui::Button("T-Needle(P)", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (__gc.g_markerSelectionMode) {
				__gc.SetErrorCode("Pivoting is Not Allowed during Marker Selection!");
			}
			else if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_RESET_PIVOT) {
				__gc.SetErrorCode("Wait for Canceling Pivot Process!");
			}
			else {
				if (!trackInfo.GetRigidBodyQuatTVecByName("t-needle", NULL, NULL)) {
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
		if (ImGui::Button("Troca(P)", ImVec2(0, buttonHeight)))
		{
			if (__gc.g_optiRecordMode != OPTTRK_RECMODE::NONE) {
				__gc.SetErrorCode("Not Allowed during Rec Processing!");
			}
			else if (__gc.g_markerSelectionMode) {
				__gc.SetErrorCode("Pivoting is Not Allowed during Marker Selection!");
			}
			else if (__gc.g_optiEvent == OPTTRK_THREAD::TOOL_RESET_PIVOT) {
				__gc.SetErrorCode("Wait for Canceling Pivot Process!");
			}
			else {
				if (!trackInfo.GetRigidBodyQuatTVecByName("troca", NULL, NULL)) {
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

		ImGui::SeparatorText("Record:");
		if (ImGui::Button(__gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD ? "Stop" : "Record", ImVec2(g_sizeRightPanelW - 50, buttonHeight)))
		{
			if (__gc.g_optiRecordMode == OPTTRK_RECMODE::LOAD) {
				__gc.SetErrorCode("Recording is Not Allowed during Playing Rec File!");
			}
			else {
				__gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD ? __gc.g_optiRecordMode = OPTTRK_RECMODE::NONE
					: __gc.g_optiRecordMode = OPTTRK_RECMODE::RECORD;
			}
		}
		// To Do... Add... Recording File Names
		ImGui::SeparatorText("Safe Instructions:");
		if (ImGui::Button("Shot Pose Capture", ImVec2(0, buttonHeight)))
		{
			__gc.g_optiEvent = OPTTRK_THREAD::C_ARM_SHOT_MOMENT;
			while (__gc.g_optiEvent == OPTTRK_THREAD::C_ARM_SHOT_MOMENT) { Sleep(2); }
		}
		ImGui::SameLine();
		if (__gc.g_refCArmRbDist < 0) {
			ComputeRefCArmPhase(__gc.g_refCArmRbDist, __gc.g_refCArmRbAngle);
		}
		if (ImGui::Button("C-Arm Reference", ImVec2(0, buttonHeight)))
		{
			if (ComputeRefCArmPhase(__gc.g_refCArmRbDist, __gc.g_refCArmRbAngle)) {
				cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_checker.txt", cv::FileStorage::Mode::WRITE);
				fs << "DIST_REF" << __gc.g_refCArmRbDist;
				fs << "ANGLE_REF" << __gc.g_refCArmRbAngle;
				fs.release();
			}
			else
				__gc.SetErrorCode("Both C-Arm and Checker should be detected!");
		}
		if (__gc.g_optiRecordMode != OPTTRK_RECMODE::LOAD)
		{
			ImGui::SeparatorText("Tracking Camera 0:");
			ImVec2 canvas_size = ImGui::GetContentRegionAvail();
			// Optitrack tri:120 uses 640x480 image
			static cv::Size imgSize(640, 480);
			cv::Mat imgGray = cv::Mat(imgSize, CV_8UC1);
			optitrk::GetCameraBuffer(0, imgSize.width, imgSize.height, imgGray.data);

			cv::Mat img;
			cv::cvtColor(imgGray, img, cv::COLOR_GRAY2RGB);
			if (img.channels() == 3) {
				cv::cvtColor(img, img, cv::COLOR_RGB2RGBA);
				std::vector<cv::Mat> channels(4);
				cv::split(img, channels);
				channels[3] = 255;
				cv::merge(channels, img);
			}

			static ID3D11ShaderResourceView* pSRV = NULL;
			pSRV = LoadTextureFromFile(img.data, "OPTI_CAMERA1", imgSize.width, imgSize.height);
			if (pSRV != NULL) {
				//ImVec2 pos = ImGui::GetCursorScreenPos();
				ImGui::Image(pSRV, ImVec2(canvas_size.x, canvas_size.x * ((float)imgSize.height / (float)imgSize.width)), ImVec2(0, 0), ImVec2(1, 1));
			}
		}
	}

	void CalibrateIntrinsics()
	{
		static std::vector<cv::Point3f> corners;
		static int indexIntrinsic = -2;
		static std::vector<std::string> files;
		static std::map<int, bool> mapScanToCalib;
		if (indexIntrinsic == -2) {
			indexIntrinsic = -1;
			int count = 0;
			for (const auto& entry : std::filesystem::directory_iterator(__gc.g_folder_trackingInfo + "int_images")) {
				if (entry.path().extension().string() == ".png") {
					files.push_back(entry.path().string());
					mapScanToCalib[count++] = false;
				}
			}
		}
		static ID3D11ShaderResourceView* pSRV = NULL;
		static std::vector<cv::Point2f> points2D;
		static std::vector<std::vector<cv::Point3f>> calib_points3Ds;
		static std::vector<std::vector<cv::Point2f>> calib_points2Ds;
		static int intrinsicWidth = 8, intrinsicHeight = 8;
		static float intrinsicSpaceX = 1.6f, intrinsicSpaceY = 1.6f;
		static int w, h;
		static bool isCalibratedIntrinsic = false;
		static bool imgReady = false;
		static int processAll = -1;
		if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
			for (int i = 0; i < (int)files.size(); i++)
			{
				bool colorHighlight = i == indexIntrinsic;
				if (colorHighlight) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6, 0.1, 0.2, 1));
				else if (mapScanToCalib[i]) {
					ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1, 0.6, 0.2, 1));
					colorHighlight = true;
				}

				if(ImGui::Button(((i < 10? "I:0" : "I:") + std::to_string(i)).c_str()) || processAll == i) {
					indexIntrinsic = i;
					imgReady = true;
				}
				if ((i % 10 != 0 || i == 0) && i != (int)files.size() - 1) ImGui::SameLine();
				if (colorHighlight) ImGui::PopStyleColor();
			}
		}

		if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT ? imgReady : ImGui::Button("Add Intrinsic", ImVec2(0, buttonHeight)))
		{
			imgReady = false;
			cv::Mat img;
			if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
				img = cv::imread(files[indexIntrinsic]);
				__gc.g_engineLogger->info("image index : {}", indexIntrinsic);
			}
			else if (g_curScanGrayImg.data != NULL) {
				cv::cvtColor(g_curScanGrayImg, img, cv::COLOR_GRAY2RGB);
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
				fs["INT_SPACE_X"] >> intrinsicSpaceX;
				fs["INT_SPACE_Y"] >> intrinsicSpaceY;
				fs["INT_MODE"] >> calib_mode;
				fs["INT_DETECTOR"] >> det_mode;
				fs.release();
			}
			if (corners.size() == 0) {
				for (int i = 0; i < intrinsicHeight; ++i) {
					for (int j = 0; j < intrinsicWidth; ++j) {
						corners.push_back(cv::Point3f(j * intrinsicSpaceX, i * intrinsicSpaceY, 0));
					}
				}
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
				points2D.clear();
				mystudents::Get2DPostionsFromFMarkersPhantom(imgGray, intrinsicWidth, intrinsicHeight, points2D, 1000.f, 10000.f, 50.f, &circleRadiis);

				DrawCircles(points2D, circleRadiis, __gc.g_circleThickness, img);

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

				pSRV = LoadTextureFromFile(img.data, "INTRINSIC_IMG", image_width, image_height);
				isCalibratedIntrinsic = false;
			}
		}
		if (__gc.g_optiRecordMode != OPTTRK_RECMODE::CALIB_EDIT) ImGui::SameLine();
		if (ImGui::Button("Apply Intrinsic", ImVec2(0, buttonHeight)) || processAll >= 0)
		{
			if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
				if (mapScanToCalib[indexIntrinsic]) {
					isCalibratedIntrinsic = true;
				}
				mapScanToCalib[indexIntrinsic] = true;
			}
			if (points2D.size() != intrinsicWidth * intrinsicHeight) {
				__gc.SetErrorCode("Failure to detect the entire circles!");
			}
			else if (isCalibratedIntrinsic) {
				__gc.SetErrorCode("The same scan image is already applied!");
			}
			else {
				isCalibratedIntrinsic = true;

				calib_points2Ds.push_back(points2D);
				calib_points3Ds.push_back(corners);

				__gc.g_engineLogger->info("# of {}-pair sets : {}", intrinsicWidth * intrinsicHeight, calib_points2Ds.size());

				cv::Mat cameraMatrix, distCoeffs;
				std::vector<cv::Mat> rvecs, tvecs;
				float err = (float)cv::calibrateCamera(calib_points3Ds, calib_points2Ds, cv::Size(g_curScanGrayImg.cols, g_curScanGrayImg.rows), cameraMatrix, distCoeffs, rvecs, tvecs);

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
		ImGui::SameLine();
		if (ImGui::Button("Reset Intrinsic", ImVec2(0, buttonHeight))) {
			if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT)
			{
				for (auto it = mapScanToCalib.begin(); it != mapScanToCalib.end(); it++) {
					it->second = false;
				}
			}


			calib_points2Ds.clear();
			calib_points3Ds.clear();
			__gc.g_engineLogger->info("Intrinsic Points are cleaned!");
		}
		if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
			if (ImGui::Button("Process All Images for Intrinsics", ImVec2(0, buttonHeight)) || processAll >= 0) {
				processAll++;
				if (processAll == (int)files.size())
					processAll = -1;
			}
		}

		ImGui::SeparatorText("Parameter Settings:");
		if (ImGui::Button("Key")) ImGui::SetKeyboardFocusHere();
		ImGui::SameLine();
		ImGui::SetNextItemWidth(200);
		ImGui::SliderFloat("Min Circularity", &__gc.g_intParam_minCircularity, 0.0f, 1.0f);
		if (ImGui::Button("Key")) ImGui::SetKeyboardFocusHere();
		ImGui::SameLine();
		ImGui::SetNextItemWidth(200);
		ImGui::SliderFloat("Min InertiaRatio", &__gc.g_intParam_minInertiaRatio, 0.0f, 1.0f);
		if (ImGui::Button("Key")) ImGui::SetKeyboardFocusHere();
		ImGui::SameLine();
		ImGui::SetNextItemWidth(200);
		ImGui::SliderInt("Min Repeatability", &__gc.g_intParam_minRepeatability, 1, 10);

		if (pSRV != NULL) {
			ImGui::SeparatorText("Source Image:");
			//ImVec2 pos = ImGui::GetCursorScreenPos();
			ImVec2 canvas_size = ImGui::GetContentRegionAvail();
			ImGui::Image(pSRV, ImVec2(canvas_size.x, canvas_size.x), ImVec2(0, 0), ImVec2(1, 1));
			//ImVec2 uv_min = ImVec2(0.0f, 0.0f);                 // Top-left
			//ImVec2 uv_max = ImVec2(1.0f, 1.0f);                 // Lower-right
			//ImGui::Image(my_tex_id, ImVec2(my_tex_w, my_tex_h), uv_min, uv_max);
		}
	}

	void CalibrateExtrinsics()
	{
		static int indexExtrinsics = -2;
		static std::vector<std::string> img_files;
		static std::vector<std::string> geo_files;
		static std::map<int, bool> mapScanToCalib;
		if (indexExtrinsics == -2) {
			int count = 0;
			for (const auto& entry : std::filesystem::directory_iterator(__gc.g_folder_trackingInfo + "ext_images")) {
				if (entry.path().extension().string() == ".png") {
					std::string fileName = entry.path().string();
					img_files.push_back(fileName);

					size_t lastindex = fileName.find_last_of(".");
					std::string geoFileName = fileName.substr(0, lastindex) + "_RBPts.txt";
					geo_files.push_back(geoFileName);

					mapScanToCalib[count++] = false;
				}
			}
			indexExtrinsics = -1;
		}
		static ID3D11ShaderResourceView* pSRV = NULL;
		static std::vector<cv::Point3f> pointsWS3D;
		static std::vector<cv::Point3f> pointsRB3D;
		static std::vector<cv::Point2f> points2D;
		static std::vector<std::vector<cv::Point3f>> calib_pointsRB3Ds;
		static std::vector<std::vector<cv::Point3f>> calib_pointsWS3Ds;
		static std::vector<std::vector<cv::Point2f>> calib_points2Ds;
		static int extrinsicWidth = 3, extrinsicHeight = 3;
		static int w, h;
		static bool isCalibratedExtrinsic = false;
		static bool imgReady = false;
		static glm::fmat4x4 matRB2WS, matWS2RB;
		static cv::Mat imgOriginal, imgFlip;
		static bool showOnlyCurrentSelectionScan = false;
		static int processAll = -1;
		static int aidMarkerGroup = 0;
		if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
			for (int i = 0; i < (int)img_files.size(); i++)
			{
				
				bool colorHighlight = i == indexExtrinsics;
				if (colorHighlight) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6, 0.1, 0.2, 1));
				else if (mapScanToCalib[i]) {
					ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1, 0.6, 0.2, 1));
					colorHighlight = true;
				}
				
				if (ImGui::Button(((i < 10 ? "E:0" : "E:") + std::to_string(i)).c_str()) || processAll == i) {
					indexExtrinsics = i;
					imgReady = true;
				}
				if ((i % 8 != 0 || i == 0) && i != (int)img_files.size() - 1) ImGui::SameLine();
				if (colorHighlight) ImGui::PopStyleColor();
			}
		}

		if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT ? imgReady : ImGui::Button("Add Extrinsic", ImVec2(0, buttonHeight)))
		{
			imgReady = false;
			bool errResult = false;

			cv::FileStorage fs(__gc.g_folder_trackingInfo + "calib_settings.txt", cv::FileStorage::Mode::READ);
			//std::string calib_mode = "FMARKERS";
			//std::string det_mode = "SimpleBlobDetector";
			if (fs.isOpened()) {
				fs["EXT_ROWS"] >> extrinsicHeight;
				fs["EXT_COLS"] >> extrinsicWidth;
				//fs["EXT_MODE"] >> calib_mode;
				//fs["EXT_DETECTOR"] >> det_mode;
				fs.release();
			}

			// set matRB2WS, matWS2RB
			// set pointsWS3D (set to __gc.g_testMKs), pointsRB3D
			if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
				cv::FileStorage fs(geo_files[indexExtrinsics], cv::FileStorage::Mode::READ);
				if (fs.isOpened()) {
					cv::Mat ocvMat;
					fs["RB_POSITIONS"] >> ocvMat;
					pointsRB3D = ocvMat;
					fs["WS_POSITIONS"] >> ocvMat;
					pointsWS3D = ocvMat;
					__gc.g_testMKs.clear();
					__gc.g_testMKs.assign(pointsWS3D.size(), glm::fvec3(0));
					memcpy(&__gc.g_testMKs[0], &pointsWS3D[0], sizeof(glm::fvec3) * pointsWS3D.size());
					fs["MAT_WS2RB"] >> ocvMat;
					memcpy(glm::value_ptr(matWS2RB), ocvMat.ptr(), sizeof(float) * 16);
					fs["MAT_RB2WS"] >> ocvMat;
					memcpy(glm::value_ptr(matRB2WS), ocvMat.ptr(), sizeof(float) * 16);
					fs.release();

					// 구슬 띄우기...
					if (aidMarkerGroup != 0)
						vzm::RemoveSceneItem(aidMarkerGroup);
					vzm::NewActor(vzm::ActorParameters(), "Extrinsic Marker Group", aidMarkerGroup);
					vzm::AppendSceneItemToSceneTree(aidMarkerGroup, sidScene);
					static int oidSphere = 0;
					if (oidSphere == 0) {
						glm::fvec4 xyzr(0, 0, 0, 1.f);
						vzm::GenerateSpheresObject(__FP xyzr, NULL, 1, oidSphere);
					}
					vzm::ActorParameters ap;
					ap.SetResourceID(vzm::ActorParameters::GEOMETRY, oidSphere);


					for (int i = 0; i < (int)pointsWS3D.size(); i++) {
						glm::fmat4x4 matS = glm::scale(glm::fvec3(0.003f));
						glm::fmat4x4 matT = glm::translate(*(glm::fvec3*)&pointsWS3D[i]);
						glm::fmat4x4 matTr = matT * matS;
						ap.SetLocalTransform(__FP matTr);
						*(glm::fvec4*)ap.color = glm::fvec4(1, 0, 0, 1);
						int aid = 0;
						vzm::NewActor(ap, "extrinsic inner marker:" + std::to_string(i + 1), aid);
						vzm::AppendSceneItemToSceneTree(aid, aidMarkerGroup);

						matS = glm::scale(glm::fvec3(0.007f));
						matT = glm::translate(*(glm::fvec3*)&pointsWS3D[i]);
						matTr = matT * matS;
						ap.SetLocalTransform(__FP matTr);
						*(glm::fvec4*)ap.color = glm::fvec4(1, 1, 1, 0.5);
						vzm::NewActor(ap, "extrinsic outer marker:" + std::to_string(i + 1), aid);
						vzm::AppendSceneItemToSceneTree(aid, aidMarkerGroup);
					}
				}
				else {
					__gc.SetErrorCode("The image has no scan information!");
					errResult = true;
				}
			}
			else {
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

				std::map<std::string, std::map<track_info::MKINFO, std::any>> rbmkSet;
				if (trackInfo.GetRigidBodyByName("calib", NULL, NULL, &rbmkSet, NULL)) {
					if (rbmkSet.size() < extrinsicWidth * extrinsicHeight) {
						__gc.SetErrorCode(fmt::format("# of Markers of Calib-rig Rigidbody ({}) must be {}", rbmkSet.size(), extrinsicWidth * extrinsicHeight));
						errResult = true;
					}
				}
				else {
					__gc.SetErrorCode("Failure to Detect Calib-rig Rigidbody!");
					errResult = true;
				}

				if (!errResult)
				{
					pointsWS3D.clear();
					pointsRB3D.clear();

					int count = 0;
					for (auto& it : rbmkSet) {
						if (count >= extrinsicWidth * extrinsicHeight)
							break;
						glm::fvec3 p_ws = std::any_cast<glm::fvec3>(it.second[track_info::MKINFO::POSITION_RBPC]); // must be track_info::MKINFO::POSITION_RBPC
						glm::fvec3 p_rb = vzmutils::transformPos(p_ws, matWS2RB);
						pointsWS3D.push_back(*(cv::Point3f*)&p_ws);
						pointsRB3D.push_back(*(cv::Point3f*)&p_rb);
						count++;
					}
				}
			}

			if (!errResult)
			{
				imgOriginal = cv::Mat();
				imgFlip = cv::Mat();
				if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
					imgOriginal = cv::imread(img_files[indexExtrinsics]);
				}
				else if (g_curScanGrayImg.data != NULL) {
					cv::cvtColor(g_curScanGrayImg, imgOriginal, cv::COLOR_GRAY2RGB);
					cv::imwrite(__gc.g_folder_trackingInfo + "ext_images/" + __gc.g_lastStoredImageName + ".png", imgOriginal);

					cv::FileStorage fs(__gc.g_folder_trackingInfo + "ext_images/" + __gc.g_lastStoredImageName + "_RBPts.txt", cv::FileStorage::Mode::WRITE);
					fs << "RB_POSITIONS" << cv::Mat(pointsRB3D);
					fs << "WS_POSITIONS" << cv::Mat(pointsWS3D);
					cv::Mat ocvMat(4, 4, CV_32FC1);
					memcpy(ocvMat.ptr(), glm::value_ptr(matWS2RB), sizeof(float) * 16);
					fs << "MAT_WS2RB" << ocvMat;
					memcpy(ocvMat.ptr(), glm::value_ptr(matRB2WS), sizeof(float) * 16);
					fs << "MAT_RB2WS" << ocvMat;

					// add CAM pos
					fs.release();
				}
				else {
					__gc.SetErrorCode("No Image!!");
				}

				// Load into a raw RGBA buffer
				if (imgOriginal.data != NULL) {
					int image_width = imgOriginal.cols;
					int image_height = imgOriginal.rows;

					// optional
					cv::flip(imgOriginal, imgFlip, 1);

					cv::Mat imgGray;
					cv::cvtColor(imgFlip, imgGray, cv::COLOR_RGB2GRAY);

					std::vector<float> circleRadiis;
					points2D.clear();
					mystudents::Get2DPostionsFromFMarkersPhantom(imgGray, extrinsicWidth, extrinsicHeight, points2D, 1500.f, 16000.f, 100.f, &circleRadiis);

					DrawCircles(points2D, circleRadiis, __gc.g_circleThickness, imgFlip);

					if (imgFlip.channels() == 3) {
						// First create the image with alpha channel
						cv::cvtColor(imgFlip, imgFlip, cv::COLOR_RGB2RGBA);

						// # Split the image for access to alpha channel
						std::vector<cv::Mat> channels(4);
						cv::split(imgFlip, channels);

						// Assign the mask to the last channel of the image
						channels[3] = 255;

						// Finally concat channels for rgba image
						cv::merge(channels, imgFlip);
					}

					__gc.g_engineLogger->info("image index : {}", indexExtrinsics);

					pSRV = LoadTextureFromFile(imgFlip.data, "EXTRINSIC_IMG", image_width, image_height);
					isCalibratedExtrinsic = false;


					if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
						if (mapScanToCalib[indexExtrinsics]) {
							cv::Mat K, D;
							cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
							if (fs.isOpened()) {
								fs["K"] >> K;
								fs["DistCoeffs"] >> D;
								fs.release();
								SetAnimation(cidRender1, 1, K, indexExtrinsics);
							}
						}
					}
				}
			}
		}
		if (__gc.g_optiRecordMode != OPTTRK_RECMODE::CALIB_EDIT) ImGui::SameLine();
		if (ImGui::Button("Apply Extrinsic", ImVec2(0, buttonHeight)) || processAll >= 0)
		{
			if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
				if (mapScanToCalib[indexExtrinsics]) {
					isCalibratedExtrinsic = true;
				}
				else {
					mapScanToCalib[indexExtrinsics] = true;
					showOnlyCurrentSelectionScan = false;

					for (auto it = __gc.g_mapAidGroupCArmCam.begin(); it != __gc.g_mapAidGroupCArmCam.end(); it++) {
						int aidImgPlane = vzmutils::GetSceneItemIdByName("CArm Plane:" + std::to_string(it->first));
						if (aidImgPlane != 0) {
							vzm::ActorParameters apImg;
							vzm::GetActorParams(aidImgPlane, apImg);
							apImg.is_visible = true;
							vzm::SetActorParams(aidImgPlane, apImg);
						}
					}
				}
			}

			if (points2D.size() != extrinsicWidth * extrinsicHeight) {
				__gc.SetErrorCode("Failure to detect the entire circles!");
			}
			else if (isCalibratedExtrinsic) {
				__gc.SetErrorCode("The same scan image is already applied!");
			}
			else {
				isCalibratedExtrinsic = true;
				calib_points2Ds.push_back(points2D);
				calib_pointsRB3Ds.push_back(pointsRB3D);
				calib_pointsWS3Ds.push_back(pointsWS3D);

				__gc.g_engineLogger->info("# of {}-pair sets : {}", extrinsicWidth * extrinsicHeight, calib_points2Ds.size());

				cv::Mat K, D;
				cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
				if (fs.isOpened()) {
					fs["K"] >> K;
					fs["DistCoeffs"] >> D;
					fs.release();

					for (int i = 0; i < points2D.size(); i++) {
						__gc.g_homographyPairs[__cv3__ & pointsRB3D[i]] = __cv2__ & points2D[i];
					}

					std::vector<cv::Point2f> global_points2d;
					std::vector<cv::Point3f> global_points3d;
					for (auto& _pair : __gc.g_homographyPairs) {
						global_points2d.push_back(*(cv::Point2f*)&(_pair.second));
						global_points3d.push_back(*(cv::Point3f*)&(_pair.first));
					}
					cv::Mat rvec, tvec;
					cv::solvePnP(global_points3d, global_points2d, K, D, rvec, tvec);

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
					cv::projectPoints(global_points3d, rvec, tvec, K, D, imagePointsReprojected);

					// Compute the reprojection error
					float maxErr = 0;
					float reprojectionError = ComputeReprojErr(global_points2d, imagePointsReprojected, maxErr);

					__gc.g_engineLogger->info("reprojectionError of {} pairs : {}, max error : {}", global_points2d.size(), reprojectionError, maxErr);

					glm::fmat4x4 matRB2CA, matCA2RB;
					ComputePoseMatrixFromRTVec(rvec, tvec, matRB2CA, matCA2RB);
					__gc.g_CArmRB2SourceCS = matRB2CA;

					cv::FileStorage fs(__gc.g_folder_trackingInfo + "rb2carm1.txt", cv::FileStorage::Mode::WRITE);
					fs.write("rvec", rvec);
					fs.write("tvec", tvec);
					fs.release();


					std::string timePack = std::to_string(navihelpers::GetCurrentTimePack());
					auto it = __gc.g_mapAidGroupCArmCam.find(indexExtrinsics);
					if (it != __gc.g_mapAidGroupCArmCam.end())
						vzm::RemoveSceneItem(it->second, true);

					cv::cvtColor(imgFlip, imgFlip, cv::COLOR_BGR2RGB);
					int aidGroup = calibtask::RegisterCArmImage(sidScene, indexExtrinsics, K, D, matCA2RB, matRB2WS, imgFlip);
					__gc.g_mapAidGroupCArmCam[indexExtrinsics] = aidGroup;

					SetAnimation(cidRender1, 1, K, indexExtrinsics);
				}
				else {
					__gc.SetErrorCode("Cannot load the Intrinsics!!");
				}
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Reset Extrinsic", ImVec2(0, buttonHeight))) {

			if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT)
			{
				// CALIB_EDIT 아닐 때는 일반 AP, LL 로 쓰이므로, CALIB_EDIT 일 때만 g_mapAidGroupCArmCam, mapScanToCalib 를 처리함
				for (auto it = __gc.g_mapAidGroupCArmCam.begin(); it != __gc.g_mapAidGroupCArmCam.end(); it++) {
					vzm::RemoveSceneItem(it->second, true);
				}
				__gc.g_mapAidGroupCArmCam.clear(); 
				for (auto it = mapScanToCalib.begin(); it != mapScanToCalib.end(); it++) {
					it->second = false;
				}
			}

			vzm::RemoveSceneItem(aidMarkerGroup);

			calib_points2Ds.clear();
			calib_pointsWS3Ds.clear();
			calib_pointsRB3Ds.clear();
			__gc.g_homographyPairs.clear();
			__gc.g_engineLogger->info("Extrinsic Points are cleaned!");
		}
		ImGui::SameLine();
		if (__gc.g_optiRecordMode == OPTTRK_RECMODE::CALIB_EDIT) {
			if (ImGui::Button(showOnlyCurrentSelectionScan ? "Show All Scans" : "Show Current Scan", ImVec2(0, buttonHeight))) {
				showOnlyCurrentSelectionScan = !showOnlyCurrentSelectionScan;
				for (auto it = __gc.g_mapAidGroupCArmCam.begin(); it != __gc.g_mapAidGroupCArmCam.end(); it++) {
					int aidImgPlane = vzmutils::GetSceneItemIdByName("CArm Plane:" + std::to_string(it->first));
					if (aidImgPlane != 0) {
						vzm::ActorParameters apImg;
						vzm::GetActorParams(aidImgPlane, apImg);
						if (showOnlyCurrentSelectionScan) {
							apImg.is_visible = it->first == indexExtrinsics;
						}
						else apImg.is_visible = true;
						vzm::SetActorParams(aidImgPlane, apImg);
					}
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("TODO:", ImVec2(0, buttonHeight))) {
				// TO DO
				// toggle ON / OFF
				// when ON save the previous cam2 (static) and load the previous CA-cam2 (static)
				// when OFF save the previous CA-cam2 (static) load the previous cam2 (static) 
				// MARKERS to C-ARM Space using RB2CA
				// CA to the tracking camera position (-z dir)
				// set scene stage to view2
			}

			if (ImGui::Button("Process All Images for Extrinsics", ImVec2(0, buttonHeight)) || processAll >= 0) {
				processAll++;
				if (processAll == (int)img_files.size())
					processAll = -1;
			}
		}
		else {
			if (ImGui::Button(!__gc.g_showCalibMarkers ? "Show Calib Points" : "Hide Calib Points", ImVec2(0, buttonHeight)))
			{
				__gc.g_showCalibMarkers = !__gc.g_showCalibMarkers;

				if (__gc.g_showCalibMarkers) {
					using namespace glm;
					fmat4x4 matCam2WS;
					trackInfo.GetTrackingCamPose(0, matCam2WS);

					*(fvec3*)cpCam1.pos = vzmutils::transformPos(fvec3(0, 0, 0), matCam2WS);
					*(fvec3*)cpCam1.view = vzmutils::transformVec(fvec3(0, 0, -1), matCam2WS);
					*(fvec3*)cpCam1.up = vzmutils::transformVec(fvec3(0, 1, 0), matCam2WS);

					float vFov = 3.141592654f / 4.f;
					float aspect_ratio = (float)cpCam1.w / (float)cpCam1.h;
					float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;

					//  fy = h/(2 * tan(f_y / 2)) 
					cpCam1.fx = cpCam1.w / (2.f * tan(hFov / 2.f));
					cpCam1.fy = cpCam1.h / (2.f * tan(vFov / 2.f));
					cpCam1.sc = 0;
					cpCam1.cx = cpCam1.w / 2.f;
					cpCam1.cy = cpCam1.h / 2.f;
					cpCam1.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
					cpCam1.np = 0.15f;

					vzm::SetCameraParams(cidRender1, cpCam1);
				}
			}
		}

		ImGui::SeparatorText("Parameter Settings:");
		if (ImGui::Button("Key")) ImGui::SetKeyboardFocusHere();
		ImGui::SameLine();
		ImGui::SetNextItemWidth(200);
		ImGui::SliderFloat("Min Circularity", &__gc.g_extParam_minCircularity, 0.0f, 1.0f);
		if (ImGui::Button("Key")) ImGui::SetKeyboardFocusHere();
		ImGui::SameLine();
		ImGui::SetNextItemWidth(200);
		ImGui::SliderFloat("Min InertiaRatio", &__gc.g_extParam_minInertiaRatio, 0.0f, 1.0f);
		if (ImGui::Button("Key")) ImGui::SetKeyboardFocusHere();
		ImGui::SameLine();
		ImGui::SetNextItemWidth(200);
		ImGui::SliderInt("Min Repeatability", &__gc.g_extParam_minRepeatability, 1, 10);

		if (pSRV != NULL) {
			ImGui::SeparatorText("Source Image:");
			//ImVec2 pos = ImGui::GetCursorScreenPos();
			ImVec2 canvas_size = ImGui::GetContentRegionAvail();
			ImGui::Image(pSRV, ImVec2(canvas_size.x, canvas_size.x), ImVec2(0, 0), ImVec2(1, 1));
		}
	}

	void SystemInfo()
	{
		ImGuiIO& io = ImGui::GetIO();

		ImGui::Text("Render Count = %d", vzmpf::GetRenderCount());
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
		ImGui::Text((std::string("Network task : ") + __gc.g_networkState).c_str());
		float dist_cur = -1, angle_cur = 0;
		ComputeRefCArmPhase(dist_cur, angle_cur);
		if (dist_cur >= 0) {
			ImGui::Text("C-Arm & Checker Difference (Dist:%.3f mm)(Angle:%.3f deg)", abs(__gc.g_refCArmRbDist - dist_cur) * 1000.f, abs(__gc.g_refCArmRbAngle - angle_cur));
		}
		else {
			ImGui::Text("C-Arm & Checker Difference (N/A)");
		}

		ImGui::SeparatorText("TRACKING ASSETS:");
		ImGui::Text(__gc.g_optiRbStates.c_str());
	}
}