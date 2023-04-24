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
	vzm::AppendSceneItemToSceneTree(aidTextFrame, sidScene);
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
	//rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.13.30 PM.csv", rapidcsv::LabelParams(0, 0));
	//rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.14.05 PM.csv", rapidcsv::LabelParams(0, 0));
	//rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.14.56 PM.csv", rapidcsv::LabelParams(0, 0));
	//rapidcsv::Document trackingData("../data/Tracking 2023-04-19/Take 2023-04-19 05.18.13 PM.csv", rapidcsv::LabelParams(0, 0));
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
#define NUM_RBS 3
	static std::string _rb_names[NUM_RBS] = { "c-arm" , "test1", "probe" };
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

	struct rb_tr_smooth
	{
		int fr_count;
		std::vector<glm::fquat> qs;
		std::vector<glm::fvec3> ts;
	};

	std::map<std::string, rb_tr_smooth> rbMapTRs;
	std::map<std::string, std::vector<glm::fvec3>> test1MkMapPos;
	const bool staticPoseAvrTest = true;

	for (int rowIdx = startRowIdx; rowIdx < numRows; rowIdx++) {
		std::vector<std::string> rowStrValues = trackingData.GetRow<std::string>(rowIdx);
		if (rowStrValues.size() == 0)
			continue;
		navihelpers::track_info2& trackInfo = trackingFrames[frameIdx++];

		int colIdx = startColIdx;
		while (colIdx < numCols) {
			std::string type = rowTypes[colIdx];
			std::string name = rowNames[colIdx];
			std::string id = rowIds[colIdx];
			std::bitset<128> cid = string2cid(id);
			std::string rotPos = rotPosLabels[colIdx];
			std::string xyzw = xyzwLabels[colIdx];

			std::map<std::string, std::map<navihelpers::track_info2::MKINFO, std::any>> rbmkSet;

			if (type == "Rigid Body") {
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
					v[navihelpers::track_info2::MKINFO::POSITION] = p;
					v[navihelpers::track_info2::MKINFO::MK_QUALITY] = mq;
					v[navihelpers::track_info2::MKINFO::MK_NAME] = _name;
					v[navihelpers::track_info2::MKINFO::CID] = _cid;
				}

				trackInfo.AddRigidBody(name, mat_ls2ws, mkMSE, rbmkSet);
			}
			if (type == "Marker") {
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
	std::map<std::string, glm::fmat4x4> rbMapTrAvr;
	std::map<std::string, glm::fvec3> mkPosTest1Avr;
	{
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
		}

		for (auto& pts : test1MkMapPos) {
			std::vector<glm::fvec3>& posMK = pts.second;
			int numTRs = (int)posMK.size();
			float normalizedNum = 1.f / (float)numTRs;

			glm::fvec3 trAvr = glm::fvec3(0, 0, 0);
			for (int i = 0; i < numTRs; i++)
				trAvr += posMK[i] * normalizedNum;
			mkPosTest1Avr[pts.first] = trAvr;
		}
	}
	// TO DO with rbMapTrAvr and mkPosTest1Avr for calibration task

	// generate scene actors 
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	if(sidScene != 0)
	{
		navihelpers::track_info2& trackInfo = trackingFrames[0];
		int numMKs = trackInfo.NumMarkers();
		int numRBs = trackInfo.NumRigidBodies();

		int oidAxis = 0;
		vzm::GenerateAxisHelperObject(oidAxis, 0.15f);
		int oidMarker = 0;
		glm::fvec4 pos(0, 0, 0, 1.f);
		vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);

		for (int i = 0; i < numRBs; i++) {
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

		for (int i = 0; i < numMKs; i++) {
			std::map<navihelpers::track_info2::MKINFO, std::any> mk;
			if (trackInfo.GetMarkerByIdx(i, mk)) {
				vzm::ActorParameters apMarker;
				apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
				apMarker.is_visible = false;
				*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 1.f, 1.f, 1.f); // rgba
				int aidMarker = 0;
				std::string mkName = std::any_cast<std::string>(mk[navihelpers::track_info2::MKINFO::MK_NAME]);
				vzm::NewActor(apMarker, mkName, aidMarker);
				vzm::AppendSceneItemToSceneTree(aidMarker, sidScene);
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
#else
		static int frameCount = 0;
		Sleep(10);

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

		navihelpers::track_info2& trackInfo = trackingFrames[frame];
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
				
				if(staticPoseAvrTest)
					matLS2WS = rbMapTrAvr[rbName];
				
				apAxis.SetLocalTransform(__FP matLS2WS);
				vzm::SetActorParams(aidAxis, apAxis);
			}
		}

		for (int i = 0; i < numMKs; i++) {
			std::map<navihelpers::track_info2::MKINFO, std::any> mk;
			if (trackInfo.GetMarkerByIdx(i, mk)) {
				std::string mkName = std::any_cast<std::string>(mk[navihelpers::track_info2::MKINFO::MK_NAME]);
				glm::fvec3 pos = std::any_cast<glm::fvec3>(mk[navihelpers::track_info2::MKINFO::POSITION]);

				if (staticPoseAvrTest)
				{
					auto mkp = mkPosTest1Avr.find(mkName);
					if (mkp != mkPosTest1Avr.end())
						pos = mkPosTest1Avr[mkName];
				}

				glm::fmat4x4 matLS2WS = glm::translate(pos);
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.01f)); // set 1 cm to the marker diameter
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
			Render();
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
		vzmutils::ZoomImageplane(zDelta > 0, cpCam1.ip_w, cpCam1.ip_h);

		vzm::SetCameraParams(cidCam1, cpCam1);
		Render();
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
