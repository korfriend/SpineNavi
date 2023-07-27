#include "Engine.h"

Engine::Engine(QString optiProfilePath, QString optiCalfilePath)
{
	m_optiProfilePath = optiProfilePath;
	m_optiCalfilepath = optiCalfilePath;
	m_numImg = 0;

	m_network_processing_thread = new networkThread();
	m_qtimer = new QTimer();


	connect(m_network_processing_thread, SIGNAL(sigImageArrived(cv::Mat)), this, SLOT(slotImageArrived(cv::Mat)));

	this->EngineInit();

	

}

Engine::~Engine()
{
	SAFE_DELETE_OBJECT(m_qtimer);

#ifdef USE_MOTIVE
	tracker_alive = false; // make the thread finishes, this setting should be located right before the thread join
	tracker_processing_thread.join();

	optitrk::DeinitOptiTrackLib();
#endif
	vzm::DeinitEngineLib();
	SAFE_DELETE_OBJECT(m_network_processing_thread);


}

void Engine::EngineInit()
{

	vzm::InitEngineLib("SpintNavi");
	vzm::SetLogConfiguration(true, 4);

#ifdef USE_MOTIVE
	m_optitrackMode = optitrk::InitOptiTrackLib();
#else

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

	std::string folder_trackingInfo = getdatapath() + "Tracking 2023-04-19/";
	std::string folder_capture = getdatapath() + "c-arm 2023-04-19/";
	std::string folder_optiSession = getdatapath() + "Session 2023-04-19/";
	rapidcsv::Document trackingData(folder_optiSession + "Take 2023-04-19 05.14.56 PM.csv", rapidcsv::LabelParams(0, 0));  // 7084.png
	std::string cArmTrackFile = folder_trackingInfo + "c-arm-track4.txt";
	std::string imgFile = folder_capture + "7084.png";
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

	m_trackingFrames.assign(numRows - startRowIdx, navihelpers::track_info());
	std::vector<navihelpers::track_info>& trackingFrames = m_trackingFrames;

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

	m_cidCam1 = 0;
	m_Light1.is_on_camera = true;
	m_Light1.is_pointlight = false;
	m_lidLight1 = 0;

	m_sidScene = 0;

	connect(m_qtimer, SIGNAL(timeout()), this, SLOT(TimerProc()));
	this->SceneInit();
#endif	
}

void Engine::SceneInit()
{
	
	*(glm::fvec3*)m_cpCam1.pos = glm::fvec3(-1.5, 1.5, -1.5);
	*(glm::fvec3*)m_cpCam1.up = glm::fvec3(0, 1, 0);
	*(glm::fvec3*)m_cpCam1.view = glm::fvec3(1, -1, 1);
	m_cpCam1.w = SCANIMG_W;
	m_cpCam1.h = SCANIMG_H;
	m_cpCam1.np = 0.1f;
	m_cpCam1.fp = 100.f;
	float vFov = 3.141592654f / 4.f;
	float aspect_ratio = (float)m_cpCam1.w / (float)m_cpCam1.h;
	float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;
	m_cpCam1.fx = m_cpCam1.w / (2.f * tan(hFov / 2.f));
	m_cpCam1.fy = m_cpCam1.h / (2.f * tan(vFov / 2.f));
	m_cpCam1.sc = 0;
	m_cpCam1.cx = m_cpCam1.w / 2.f;
	m_cpCam1.cy = m_cpCam1.h / 2.f;
	m_cpCam1.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;

	
	vzm::NewCamera(m_cpCam1, "Cam1", m_cidCam1);

	*(glm::fvec3*)m_Light1.pos = *(glm::fvec3*)m_cpCam1.pos;
	*(glm::fvec3*)m_Light1.dir = *(glm::fvec3*)m_cpCam1.view;
	*(glm::fvec3*)m_Light1.up = *(glm::fvec3*)m_cpCam1.up;

	vzm::NewLight(m_Light1, "World Light", m_lidLight1);

	vzm::NewScene("Scene1", m_sidScene);

	vzm::AppendSceneItemToSceneTree(m_cidCam1, m_sidScene);
	vzm::AppendSceneItemToSceneTree(m_lidLight1, m_sidScene);
	

	m_qtimer->start(10);
	m_network_processing_thread->start();
}

void Engine::Render(vzm::CameraParameters cpCam)
{
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	int cidCam1 = vzmutils::GetSceneItemIdByName("Cam1");
	//int cidCam2 = vzmutils::GetSceneItemIdByName("Cam2");

	if (sidScene != 0 && cidCam1 != 0) {
		// show case
		
		vzm::SetCameraParams(cidCam1, cpCam);

		vzm::RenderScene(sidScene, cidCam1);
		//vzm::RenderScene(sidScene, cidCam2);

		//vp->setLeftRightImg();

		//TODO : 여기서 image return???
		//
	}
}

void Engine::slotImageArrived(cv::Mat img)
{
	QImage tempimg = QImage((uchar*)img.data, SCANIMG_W, SCANIMG_H, QImage::Format_RGBA8888);
	Image2D temp(m_numImg++, 0, tempimg);
	m_ImgList.push_back(temp);

	m_network_processing_thread->setDownloadFlag(false);
}

void Engine::UpdateTrackInfo2Scene(navihelpers::track_info& trackInfo)
{
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

void Engine::TimerProc()
{
	// engine 에서 굳이 계속해서 그릴 필요 없음.
	// viewer 가 요청 할 때만 그려서 image만 넘겨주면 됨.
	// tracking 은 계속 돼야함.
	// render()만 호출 안하면 될듯?
	int sidScene = vzmutils::GetSceneItemIdByName("Scene1");
	if (sidScene == 0)
		return;

	static int frameCount = 0;
#ifdef USE_MOTIVE
	int frame = frameCount++;
#else
	std::vector<navihelpers::track_info>& trackingFrames = m_trackingFrames;
	int totalFrames = (int)trackingFrames.size();
	int frame = (frameCount++) % totalFrames;
#endif

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

	// 제일 최근 trackinfo 만 저장해두고
	//viewer 에서 요청시 update and render 하는 식으로 해야함.

	//UpdateTrackInfo2Scene(trackingFrames[frame]);
#endif
	//Render();

}