#include <regex>
#include <vector>
#include <windowsx.h>
#include <iostream>
#include <winsock2.h> // udp
#include <stdio.h>

#pragma comment(lib,"ws2_32.lib")

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

//#define USE_MOTIVE
#define SCANIMG_W 1296
#define SCANIMG_H 1296
cv::Mat g_curScanImg(SCANIMG_W, SCANIMG_H, CV_8UC1);

#define CFG_LISTEN_PORT 22222
#define CFG_SIZE_MAX_FD 1000
#define CFG_SIZE_MAX_RECV_BUFFER 5000

typedef unsigned long long u64;

static int testFileIdx = 1000;

std::string folder_data = "";
std::string folder_capture = "";
std::string folder_trackingInfo = "";
std::string folder_optiSession = "";

std::atomic_bool download_completed{ false };
std::atomic_bool tracker_alive{ true };
std::atomic_bool network_alive{ true };
std::thread tracker_processing_thread;
std::thread network_processing_thread;

#ifdef USE_MOTIVE
navihelpers::concurrent_queue<navihelpers::track_info>* g_track_que = NULL;
#else
std::vector<navihelpers::track_info> g_trackingFrames;
int g_numAllFramesRBs = 0;
int g_numAllFramesMKs = 0;
#endif

#include "Viewer.h"

int numAnimationCount = 5;
int arAnimationKeyFrame = -1;
std::vector<vzm::CameraParameters> cpInterCams(numAnimationCount);

void Render(Viewer* vp) {
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	int cidCam1 = vzmutils::GetSceneItemIdByName("Cam1");
	int cidCam2 = vzmutils::GetSceneItemIdByName("Cam2");

	if (sidScene != 0 && cidCam1 != 0 && cidCam2 != 0) {
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
		vzm::RenderScene(sidScene, cidCam2);

		vp->setLeftRightImg();
	}
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

			apAxis.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidAxis, apAxis);
		}
	}

	for (int i = 0; i < numMKs; i++) {
		std::map<navihelpers::track_info::MKINFO, std::any> mk;
		if (trackInfo.GetMarkerByIdx(i, mk)) {
			std::string mkName = std::any_cast<std::string>(mk[navihelpers::track_info::MKINFO::MK_NAME]);
			glm::fvec3 pos = std::any_cast<glm::fvec3>(mk[navihelpers::track_info::MKINFO::POSITION]);

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

void Viewer::TimerProc()
{
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	if (sidScene == 0)
		return;

	static int frameCount = 0;
#ifdef USE_MOTIVE
	int frame = frameCount++;
#else
	std::vector<navihelpers::track_info>& trackingFrames = g_trackingFrames;
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
	navihelpers::concurrent_queue<navihelpers::track_info>* track_que = g_track_que;
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
	Render(this);
}

void SceneInit();
Viewer::Viewer(QWidget *parent)
    : QMainWindow(parent)
{
	this->setFocusPolicy(Qt::StrongFocus);
    ui.setupUi(this);

    num_imgs = 0;

    QHBoxLayout* hboxLayout = new QHBoxLayout(this->centralWidget());
    qlistwidget = new QListWidget(); // thumbnail view
    leftView = new QLabel();
    rightView = new QLabel();
    
    qlistwidget->setViewMode(QListWidget::IconMode);
    qlistwidget->setIconSize(QSize(200, 150));
    qlistwidget->setResizeMode(QListWidget::Adjust);

    setImagePath("../data/c-arm 2023-04-19");

    hboxLayout->addWidget(leftView, 40);
    hboxLayout->addWidget(rightView, 40);
    hboxLayout->addWidget(qlistwidget, 20);
    setImages();
    //ui.centralWidget->setLayout(hboxLayout);

    connect(qlistwidget, SIGNAL(itemDoubleClicked(QListWidgetItem*)),this, SLOT(clickedImage()));


    /////////////////////////////
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
	folder_trackingInfo = getdatapath() + "Tracking 2023-04-19/";
	folder_capture = getdatapath() + "c-arm 2023-04-19/";
	folder_optiSession = getdatapath() + "Session 2023-04-19/";

	// engine initialization to use the core APIs of framework
	// must be paired with DeinitEngineLib()
	vzm::InitEngineLib("SpineNavi");
	vzm::SetLogConfiguration(true, 4);

#ifdef USE_MOTIVE
	bool optitrkMode = optitrk::InitOptiTrackLib();
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

#ifdef USE_MOTIVE
	std::vector<std::string> rbNames;
	int numAllFramesRBs = optitrk::GetRigidBodies(&rbNames);

	static navihelpers::concurrent_queue<navihelpers::track_info> track_que(10);
	g_track_que = &track_que;
	tracker_processing_thread = std::thread([&]() {
		using namespace std;
		using namespace navihelpers;
		using namespace glm;
		while (tracker_alive)
		{
			int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
			if (sidScene == 0) continue;
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

	g_trackingFrames.assign(numRows - startRowIdx, navihelpers::track_info());
	std::vector<navihelpers::track_info>& trackingFrames = g_trackingFrames;

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

				if (name.find("test1:") != std::string::npos) {
					std::vector<glm::fvec3>& posMKs = test1MkMapPos[name];
					posMKs.push_back(p);
				}
			}
		}
	}

	g_numAllFramesRBs = numAllFramesRBs;
	g_numAllFramesMKs = numAllFramesMKs;
#endif

	SceneInit();

	m_qtimer = new QTimer(this);
	connect(m_qtimer, SIGNAL(timeout()), this, SLOT(TimerProc()));
	m_qtimer->start(10);

	network_processing_thread = std::thread([&]() {
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

					readBytes = recv(clientSocket, buffer, bufferSize, 0);
					if (readBytes < 5000) break;
					file_size = atol(buffer);

					unsigned char* data = new unsigned char[file_size];
					totalReadBytes = 0;

					//FILE* fp;
					//fopen_s(&fp, (folder_capture + "result.raw").c_str(), "wb");
					std::vector<char> bufferTmp(file_size * 2);
					while (file_size > totalReadBytes) {
						readBytes = recv(clientSocket, buffer, bufferSize, 0);
						if (readBytes < 0) break;

						//fwrite(buffer, sizeof(char), readBytes, fp);
						memcpy(&bufferTmp[totalReadBytes], buffer, readBytes);

						totalReadBytes += readBytes;
						printf("In progress: %d/%d byte(s) [%d]", totalReadBytes, file_size, (int)std::min((totalReadBytes * 100.f) / file_size, 100.f));
						fflush(stdout);
					}
					if (readBytes > 0) {
						printf("download Image");
						download_completed = true;
						memcpy(g_curScanImg.ptr(), &bufferTmp[0], file_size);
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
}

Viewer::~Viewer()
{
#ifdef USE_MOTIVE
	tracker_alive = false; // make the thread finishes, this setting should be located right before the thread join
	tracker_processing_thread.join();

	optitrk::DeinitOptiTrackLib();
#endif
	vzm::DeinitEngineLib();

	network_alive = false;
	network_processing_thread.join();
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

	vzm::CameraParameters cpCam1;
	*(glm::fvec3*)cpCam1.pos = glm::fvec3(-1.5, 1.5, -1.5);
	*(glm::fvec3*)cpCam1.up = glm::fvec3(0, 1, 0);
	*(glm::fvec3*)cpCam1.view = glm::fvec3(1, -1, 1);

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

	cpCam1.w = 700;
	cpCam1.h = 700;

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
	cpCam1.hWnd = NULL;
	int cidCam1 = 0;
	vzm::NewCamera(cpCam1, "Cam1", cidCam1);
	int cidCam2 = 0;
	vzm::NewCamera(cpCam1, "Cam2", cidCam2);

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
	vzm::AppendSceneItemToSceneTree(cidCam2, sidScene);
	vzm::AppendSceneItemToSceneTree(lidLight1, sidScene);
	vzm::AppendSceneItemToSceneTree(aidAxis, sidScene);
	vzm::AppendSceneItemToSceneTree(aidGrid, sidScene);
	vzm::AppendSceneItemToSceneTree(aidLines, sidScene);
	vzm::AppendSceneItemToSceneTree(aidTextX, sidScene);
	vzm::AppendSceneItemToSceneTree(aidTextZ, sidScene);
	vzm::AppendSceneItemToSceneTree(aidTextFrame, sidScene);

#ifndef USE_MOTIVE
	std::vector<navihelpers::track_info>& trackingFrames = g_trackingFrames;
	navihelpers::track_info& trackInfo = trackingFrames[0];

	int oidAxis2 = 0;
	vzm::GenerateAxisHelperObject(oidAxis2, 0.15f);
	int oidMarker = 0;
	glm::fvec4 pos(0, 0, 0, 1.f);
	vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);

	for (int i = 0; i < g_numAllFramesRBs; i++) {
		std::string rbName;
		if (trackInfo.GetRigidBodyByIdx(i, &rbName, NULL, NULL, NULL)) {
			vzm::ActorParameters apAxis;
			apAxis.SetResourceID(vzm::ActorParameters::GEOMETRY, oidAxis2);
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

	for (int i = 0; i < g_numAllFramesMKs; i++) {
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
				vzm::GenerateTextObject((float*)&pinfo[0], mkName.substr(6, mkName.length() - 1), 2, true, false, oidLabelText, true);
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

	RegisterCArmImage(sidScene, folder_trackingInfo + "testParams.txt", "test0");
#endif
}


void Viewer::setImages()
{
    for (auto const& dir_entry : std::filesystem::directory_iterator{ image_path })
    {
        qDebug() << dir_entry.path().u8string() << '\n';
        if (dir_entry.path().extension() == ".png")
        {
            QListWidgetItem* item = new QListWidgetItem(QIcon(dir_entry.path().u8string().c_str()), QString(dir_entry.path().filename().u8string().c_str()));
            image_paths.push_back(dir_entry.path().u8string().c_str());
            items.push_back(item);
            if (num_imgs == 0)
            {
                leftImg = QImage(dir_entry.path().u8string().c_str());
            }
            if (num_imgs == 1)
            {
                rightImg = QImage(dir_entry.path().u8string().c_str());
            }
            num_imgs++;
        }
    }    
    setThumbnailItems();
    setLeftRightImg();
}

void Viewer::setLeftRightImg()
{
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
    int cidCams[] = { vzmutils::GetSceneItemIdByName("Cam1"), vzmutils::GetSceneItemIdByName("Cam2") };
    QLabel* qlabels[] = { leftView , rightView };

	for (int i = 0; i < 2; i++) {
        QLabel* qlabel = qlabels[i];
		int w = qlabel->width();
		int h = qlabel->height();

		unsigned char* ptr_rgba;
		float* ptr_zdepth;
		int bufW, bufH;
        if (vzm::GetRenderBufferPtrs(cidCams[i], &ptr_rgba, &ptr_zdepth, &bufW, &bufH)) {
            QImage qimg = QImage(ptr_rgba, bufW, bufW, QImage::Format_RGBA8888);
			qlabel->setPixmap(QPixmap::fromImage(qimg).scaled(w, h, Qt::KeepAspectRatio));
        }
    }
}

void Viewer::resizeEvent(QResizeEvent* event)
{
    updateGeometry();
    setLeftRightImg();
}

void Viewer::setThumbnailItems()
{
    for (auto i : items)
    {
        qlistwidget->addItem(i);
    }

}

void Viewer::clickedImage()
{
    QModelIndex index = qlistwidget->currentIndex();
    int inx = index.row();
    leftImg = QImage(image_paths[inx]);

    setLeftRightImg();


}

static vzmutils::GeneralMove general_move;
static vzm::CameraParameters cpPrevCam;
static float scene_stage_scale = 5.f;
#ifdef USE_MOTIVE
static int activeCarmIdx = -1;
#else
static int activeCarmIdx = 0;
#endif
int mouseFlag = 0;
void Viewer::mousePressEvent(QMouseEvent* event) {
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	int cidCam1 = vzmutils::GetSceneItemIdByName("Cam1");
	int cidCam2 = vzmutils::GetSceneItemIdByName("Cam2");

	glm::fvec3 scene_stage_center = glm::fvec3();
	vzm::CameraParameters cpCam1;
	vzm::CameraParameters cpCam2;
	if (sidScene != 0 && cidCam1 != 0 && cidCam2 != 0) {
		vzm::GetCameraParams(cidCam1, cpCam1);
		scene_stage_scale = glm::length(scene_stage_center - *(glm::fvec3*)cpCam1.pos) * 1.0f;
	}

	// renderer does not need to be called
	int x = event->pos().x();
	int y = event->pos().y();
	glm::ivec2 pos_ss = glm::ivec2(x, y);
	general_move.Start((int*)&pos_ss, cpCam1, scene_stage_center, scene_stage_scale);

	qDebug() << "Mouse pressed at " << event->pos();
}

void Viewer::mouseMoveEvent(QMouseEvent* event) {
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	int cidCam1 = vzmutils::GetSceneItemIdByName("Cam1");
	int cidCam2 = vzmutils::GetSceneItemIdByName("Cam2");

	glm::fvec3 scene_stage_center = glm::fvec3();
	vzm::CameraParameters cpCam1;
	vzm::CameraParameters cpCam2;
	if (sidScene != 0 && cidCam1 != 0 && cidCam2 != 0) {
		vzm::GetCameraParams(cidCam1, cpCam1);
		vzm::GetCameraParams(cidCam2, cpCam2);
		scene_stage_scale = glm::length(scene_stage_center - *(glm::fvec3*)cpCam1.pos) * 1.0f;
	}

	int x = event->pos().x();
	int y = event->pos().y();
	glm::ivec2 pos_ss = glm::ivec2(x, y);
	if (event->buttons() == Qt::MouseButton::LeftButton)
		general_move.PanMove((int*)&pos_ss, cpCam1);
	else if (event->buttons() == Qt::MouseButton::RightButton)
		general_move.RotateMove((int*)&pos_ss, cpCam1);

	vzm::SetCameraParams(cidCam1, cpCam1);
	vzm::SetCameraParams(cidCam2, cpCam1);
}

void Viewer::mouseReleaseEvent(QMouseEvent* event) {
	mouseFlag = 0;
}

void Viewer::keyPressEvent(QKeyEvent* event) {
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	int cidCam1 = vzmutils::GetSceneItemIdByName("Cam1");
	int cidCam2 = vzmutils::GetSceneItemIdByName("Cam2");
	vzm::CameraParameters cpCam1, cpCam2;
	if (sidScene != 0 && cidCam1 != 0) {
		vzm::GetCameraParams(cidCam1, cpCam1);
		vzm::GetCameraParams(cidCam2, cpCam2);
	}

	static vzm::CameraParameters cpPrevCam;

#define INTERPOLCAM(PARAM) _cpCam.PARAM = cpCam1.PARAM + (cpNewCam1.PARAM - cpCam1.PARAM) * t;
	using namespace std;
	switch (event->key()) {
	case Qt::Key_S :
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
		}break;
	case Qt::Key_L:
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
	}break;
	case Qt::Key_A:
	{
#ifdef USE_MOTIVE
		if (activeCarmIdx < 0)
			break;
#endif
		cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
		cv::Mat cameraMatrix;
		fs["K"] >> cameraMatrix;
		fs.release();

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

		UINT widthWindow = cpCam1.w;
		UINT heightWindow = cpCam1.h;

		const float intrinsicRatioX = (float)imageWH.x / widthWindow;
		const float intrinsicRatioY = (float)imageWH.y / heightWindow;

		cpNewCam1.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_INTRINSICS;
		glm::fvec3 pos(0, 0, 0);
		glm::fvec3 view(0, 0, 1);
		glm::fvec3 up(0, -1, 0);
		pos = *(glm::fvec3*)cpNewCam1.pos = vzmutils::transformPos(pos, matCA2WS);
		view = *(glm::fvec3*)cpNewCam1.view = vzmutils::transformVec(view, matCA2WS);
		up = *(glm::fvec3*)cpNewCam1.up = vzmutils::transformVec(up, matCA2WS);
		cpNewCam1.fx = cameraMatrix.at<double>(0, 0) / intrinsicRatioX;
		cpNewCam1.fy = cameraMatrix.at<double>(1, 1) / intrinsicRatioY;
		cpNewCam1.sc = cameraMatrix.at<double>(0, 1);
		cpNewCam1.cx = cameraMatrix.at<double>(0, 2) / intrinsicRatioX;
		cpNewCam1.cy = cameraMatrix.at<double>(1, 2) / intrinsicRatioY;
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
	case Qt::Key_Z:
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

		cv::FileStorage __fs(folder_trackingInfo + paramsFileName, cv::FileStorage::Mode::WRITE);

		cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
		cv::Mat matK;
		fs["K"] >> matK;
		__fs << "K" << matK;
		cv::Mat distCoeffs;
		fs["DistCoeffs"] >> distCoeffs;
		__fs << "DistCoeffs" << distCoeffs;
		fs.open(folder_trackingInfo + "rb2carm.txt", cv::FileStorage::Mode::READ);
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

	static Qt::Key SAVEKEYS[10] = { Qt::Key_Q , Qt::Key_W, Qt::Key_E, Qt::Key_R, Qt::Key_T, Qt::Key_Y, Qt::Key_U, Qt::Key_I, Qt::Key_O, Qt::Key_P };
	const int maxSamples = 30;
	const float normalizedNum = 1.f / (float)maxSamples;
#ifdef USE_MOTIVE
	for (int i = 0; i < 10; i++) {
		if (event->key() == SAVEKEYS[i]) {
			glm::fvec3 rotAxisAvr = glm::fvec3(0, 0, 0);
			float rotAngleAvr = 0;
			glm::fvec3 trAvr = glm::fvec3(0, 0, 0);
			int captureCount = 0;
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
				fflush(stdout)
			}
			std::cout << "Capturing... DONE!" << std::endl;
			rotAxisAvr = glm::normalize(rotAxisAvr);
			glm::fmat4x4 mat_r = glm::rotate(rotAngleAvr, rotAxisAvr);
			glm::fmat4x4 mat_t = glm::translate(trAvr);
			glm::fmat4x4 matRB2WS = mat_t * mat_r;

			StoreParams("test" + to_string(i) + ".txt", matRB2WS, folder_trackingInfo + "test.png");
			break;
		}
	}
#else
	//std::atomic_bool udp_alive{ true };
	//
	for (int i = 0; i < 10; i++) {
		if (event->key() == SAVEKEYS[i]) {
			// to do //
			string trkInfoFile = "test" + to_string(i) + ".txt";
			ifstream _file(folder_trackingInfo + trkInfoFile);
			bool fileExist = _file.is_open();
			_file.close();
			if (!fileExist) break;

			cout << "*** Save " << trkInfoFile << " !!" << endl;

			cv::FileStorage __fs(folder_trackingInfo + trkInfoFile, cv::FileStorage::Mode::READ);
			cv::Mat rb2wsMat;
			__fs["rb2wsMat"] >> rb2wsMat;
			glm::fmat4x4 matRB2WS;
			memcpy(&matRB2WS, rb2wsMat.ptr(), sizeof(glm::fmat4x4));

			cv::Mat downloadImg;
			cv::cvtColor(g_curScanImg, downloadImg, cv::COLOR_GRAY2BGR);
			string downloadImgFileName = folder_trackingInfo + "test" + to_string(i) + ".png";
			cv::imwrite(downloadImgFileName, downloadImg);
			cv::waitKey(1);
			StoreParams("test" + to_string(i) + ".txt", matRB2WS, downloadImgFileName);
			download_completed = false;
			break;
		}
	}
#endif

	static map<int, int> mapAidGroupCArmCam;
	static Qt::Key LOADKEYS[10] = { Qt::Key_1 , Qt::Key_2, Qt::Key_3, Qt::Key_4, Qt::Key_5, Qt::Key_6, Qt::Key_7, Qt::Key_8, Qt::Key_9, Qt::Key_0 };
	for (int i = 0; i < 10; i++) {
		if (event->key() == LOADKEYS[i]) {
			// load case
			auto it = mapAidGroupCArmCam.find(i + 1);
			if (it != mapAidGroupCArmCam.end())
				vzm::RemoveSceneItem(it->second, true);
			int aidGroup = RegisterCArmImage(sidScene, folder_trackingInfo + "test" + to_string(i) + ".txt", "test" + to_string(i));
			if (aidGroup == -1)
				break;
			mapAidGroupCArmCam[i + 1] = aidGroup;
			activeCarmIdx = i;
			break;
		}
	}
}