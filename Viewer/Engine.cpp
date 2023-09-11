#include "Engine.h"

//#define USE_MOTIVE

Engine::Engine(ViewLayout* layout, QString dataPath, QString optiProfilePath, QString optiCalfilePath)
{
	__gc.Init();
	__gc.g_folder_data = dataPath.toStdString();
	__gc.g_folder_trackingInfo = dataPath.toStdString() + "Tracking 2023-08-24/";
	__gc.g_profileFileName = __gc.g_folder_data + "Motive Profile - 2023-08-24.motive";

	m_renderer = new RenderEngine();
	m_renderer->InitializeTask(&__gc);
	m_calibrator = new CalibrationEngine();
	m_calibrator->InitializeTask(&__gc);

	m_optiProfilePath = optiProfilePath;
	m_optiCalfilepath = optiCalfilePath;
	m_numImg = 0;

	m_network_processing_thread = new networkThread();
	m_network_processing_thread->InitializeTask(&__gc);
	m_trackingThread = new trackingThread();
	m_trackingThread->InitializeTask(&__gc);

	m_qtimer = new QTimer();


	connect(m_network_processing_thread, SIGNAL(sigImageArrived(cv::Mat)), this, SLOT(slotImageArrived(cv::Mat)));
	m_sceneName = "Scene1";
	m_camAPName = "Cam1";
	m_camLateralName = "Cam2";

	cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
	fs["K"] >> m_cameraMatrix;
	fs.release();

	m_dataPath = dataPath.toStdString();

	m_isAP = true;
	m_track_que = nullptr;

	m_InterCamsAP.resize(2);
	m_InterCamsLateral.resize(2);

	m_AP_set = false;
	m_Lateral_set = false;


	this->EngineInit();

	
	m_viewMgr = new ViewMgr(layout, m_renderer, this);
	connect(m_viewMgr, SIGNAL(sigCalibMousePress(glm::ivec2)), this, SLOT(slotCalibMousePress(glm::ivec2)));
	connect(m_renderer, SIGNAL(signumberkey(int)), this, SLOT(slotNumberKey(int)));
}

Engine::~Engine()
{
	SAFE_DELETE_OBJECT(m_qtimer);

#ifdef USE_MOTIVE
	m_trackingThread->terminate();

	optitrk::DeinitOptiTrackLib();
#endif
	vzm::DeinitEngineLib();

	m_network_processing_thread->terminate();
	
	SAFE_DELETE_OBJECT(m_network_processing_thread);


}

void Engine::EngineInit()
{

	vzm::InitEngineLib("SpintNavi");
	vzm::SetLogConfiguration(true, 4);

#ifdef USE_MOTIVE
	m_optitrackMode = optitrk::InitOptiTrackLib();
	//optitrk::LoadProfileAndCalibInfo(m_optiProfilePath.toStdString(), m_optiCalfilepath.toStdString());
	optitrk::LoadProfileAndCalibInfo(__gc.g_profileFileName, "");
	optitrk::SetCameraSettings(0, 4, 16, 230);
	optitrk::SetCameraSettings(1, 4, 16, 230);
	optitrk::SetCameraSettings(2, 4, 16, 230);
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

	m_trackingFrames.assign(numRows - startRowIdx, track_info());
	std::vector<track_info>& trackingFrames = m_trackingFrames;

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
		track_info& trackInfo = trackingFrames[frameIdx++];

		int colIdx = startColIdx;
		while (colIdx < numCols) {
			std::string type = rowTypes[colIdx];
			std::string name = rowNames[colIdx];
			std::string id = rowIds[colIdx];
			std::bitset<128> cid = string2cid(id);
			std::string rotPos = rotPosLabels[colIdx];
			std::string xyzw = xyzwLabels[colIdx];

			std::map<std::string, std::map<track_info::MKINFO, std::any>> rbmkSet;

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
					v[track_info::MKINFO::POSITION] = p;
					v[track_info::MKINFO::MK_QUALITY] = mq;
					v[track_info::MKINFO::MK_NAME] = _name;
					v[track_info::MKINFO::MK_CID] = _cid;
				}

				trackInfo.AddRigidBody(name, NULL,mat_ls2ws, q, t, mkMSE, rbmkSet);
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

	

	
#endif	
	m_cidCamAP = 0;
	m_cidCamLateral = 0;
	m_Light1.is_on_camera = true;
	m_Light1.is_pointlight = false;
	m_lidLight1 = 0;

	m_sidScene = 0;

	connect(m_qtimer, SIGNAL(timeout()), this, SLOT(TimerProc()));
	m_renderer->SceneInit();
	m_qtimer->start(10);
	m_network_processing_thread->start();

#ifdef USE_MOTIVE
	m_trackingThread->start();
#else
#endif
}

void Engine::SceneInit()
{
	
	*(glm::fvec3*)m_cpCamAP.pos = glm::fvec3(-1.5, 1.5, -1.5);
	*(glm::fvec3*)m_cpCamAP.up = glm::fvec3(0, 1, 0);
	*(glm::fvec3*)m_cpCamAP.view = glm::fvec3(1, -1, 1);
	m_cpCamAP.w = SCANIMG_W;
	m_cpCamAP.h = SCANIMG_H;
	m_cpCamAP.np = 0.1f;
	m_cpCamAP.fp = 100.f;
	float vFov = 3.141592654f / 4.f;
	float aspect_ratio = (float)m_cpCamAP.w / (float)m_cpCamAP.h;
	float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;
	m_cpCamAP.fx = m_cpCamAP.w / (2.f * tan(hFov / 2.f));
	m_cpCamAP.fy = m_cpCamAP.h / (2.f * tan(vFov / 2.f));
	m_cpCamAP.sc = 0;
	m_cpCamAP.cx = m_cpCamAP.w / 2.f;
	m_cpCamAP.cy = m_cpCamAP.h / 2.f;
	m_cpCamAP.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;


	vzm::NewCamera(m_cpCamAP, m_camAPName, m_cidCamAP);
	vzm::NewCamera(m_cpCamAP, m_camLateralName, m_cidCamLateral);

	*(glm::fvec3*)m_Light1.pos = *(glm::fvec3*)m_cpCamAP.pos;
	*(glm::fvec3*)m_Light1.dir = *(glm::fvec3*)m_cpCamAP.view;
	*(glm::fvec3*)m_Light1.up = *(glm::fvec3*)m_cpCamAP.up;

	vzm::NewLight(m_Light1, "World Light", m_lidLight1);

	vzm::NewScene(m_sceneName, m_sidScene);


	vzm::AppendSceneItemToSceneTree(m_cidCamAP, m_sidScene);
	vzm::AppendSceneItemToSceneTree(m_cidCamLateral, m_sidScene);
	vzm::AppendSceneItemToSceneTree(m_lidLight1, m_sidScene);
	


}

void Engine::Render()
{
	int sidScene = vzmutils::GetSceneItemIdByName(m_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(m_camAPName);
	int cidCam2 = vzmutils::GetSceneItemIdByName(m_camLateralName);

	if (sidScene != 0 && cidCam1 != 0 && cidCam2 != 0) {
		// show case

		unsigned char* ptr_rgba;
		float* ptr_zdepth;
		int bufW, bufH;
		if (m_AP_set) {
			/*vzm::CameraParameters cpCam = m_InterCamsAP[1];
			vzm::SetCameraParams(cidCam1, cpCam);
			vzm::GetCameraParams(cidCam1, m_cpCamAP);
			vzm::RenderScene(sidScene, cidCam1);*/
			if (vzm::GetRenderBufferPtrs(cidCam1, &ptr_rgba, &ptr_zdepth, &bufW, &bufH)) {
				QImage qimg = QImage(ptr_rgba, bufW, bufW, QImage::Format_RGBA8888);
				//qlabel->setPixmap(QPixmap::fromImage(qimg).scaled(w, h, Qt::KeepAspectRatio));
				Image2D tempImg(-1, VIEW_TYPE::AP, qimg);
				m_viewMgr->setAPImg(tempImg);
			}
		}

		unsigned char* ptr_rgbaL;
		float* ptr_zdepthL;
		int bufWL, bufHL;

		if (m_Lateral_set)
		{
			/*vzm::CameraParameters cpCam = m_InterCamsLateral[1];
			vzm::SetCameraParams(cidCam2, cpCam);
			vzm::GetCameraParams(cidCam2, m_cpCamLateral);
			vzm::RenderScene(sidScene, cidCam2);*/
			if (vzm::GetRenderBufferPtrs(cidCam2, &ptr_rgbaL, &ptr_zdepthL, &bufWL, &bufHL)) {
				QImage qimg = QImage(ptr_rgbaL, bufWL, bufWL, QImage::Format_RGBA8888);
				//qlabel->setPixmap(QPixmap::fromImage(qimg).scaled(w, h, Qt::KeepAspectRatio));
				Image2D tempImg(-1, VIEW_TYPE::LATERAL, qimg);
				m_viewMgr->setLateralImg(tempImg);
			}
		}

		/*vzm::CameraParameters cpCam = m_InterCamsAP[1];
		vzm::SetCameraParams(cidCam1, cpCam);
		vzm::GetCameraParams(cidCam1, m_cpCamAP);
		vzm::RenderScene(sidScene, cidCam1);*/
		if (vzm::GetRenderBufferPtrs(cidCam1, &ptr_rgba, &ptr_zdepth, &bufW, &bufH)) {
			QImage qimg = QImage(ptr_rgba, bufW, bufW, QImage::Format_RGBA8888);
			//qlabel->setPixmap(QPixmap::fromImage(qimg).scaled(w, h, Qt::KeepAspectRatio));
			Image2D tempImg(-1, VIEW_TYPE::AP, qimg);
			m_viewMgr->setCalibImg(tempImg);
		}
	}
}

void Engine::StoreParams(std::string& paramsFileName, glm::fmat4x4& matRB2WS, std::string& imgFileName)
{
	cv::FileStorage __fs(m_dataPath + paramsFileName, cv::FileStorage::Mode::WRITE);

	cv::FileStorage fs(m_dataPath + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
	cv::Mat matK;
	fs["K"] >> matK;
	__fs << "K" << matK;
	cv::Mat distCoeffs;
	fs["DistCoeffs"] >> distCoeffs;
	__fs << "DistCoeffs" << distCoeffs;

	fs.open(m_dataPath + "rb2carm1.txt", cv::FileStorage::Mode::READ);
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
}

void Engine::StoreParams(const std::string& paramsFileName, const glm::fmat4x4& matCArmRB2WS, const std::string& imgFileName)
{
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
}

void Engine::SaveAndChangeViewState(const int keyParam, const int sidScene, const int cidCam, std::vector<vzm::CameraParameters>& cpInterCams, int& arAnimationKeyFrame)
{
	using namespace std;
	static char LOADKEYS[10] = { '1' , '2', '3', '4', '5', '6', '7', '8', '9', '0' };
	static map<int, int> mapAidGroupCArmCam;
	for (int i = 0; i < 10; i++) {
		if (keyParam == i) {
			glm::fvec3 rotAxisAvr = glm::fvec3(0, 0, 0);
			float rotAngleAvr = 0;
			glm::fvec3 trAvr = glm::fvec3(0, 0, 0);
			int captureCount = 0;
			const int maxSamples = 30;
			const float normalizedNum = 1.f / (float)maxSamples;
			while (captureCount < maxSamples) {
				track_info trackInfo;
				m_trackingThread->pop(trackInfo);
				//g_track_que->wait_and_pop(trackInfo);
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
			cv::cvtColor(m_cvImgList[m_numImg-1], downloadImg, cv::COLOR_GRAY2RGB);
			string downloadImgFileName = m_dataPath + "test" + to_string(i) + ".png";
			std::cout << "STORAGE COMPLETED!!!  1" << std::endl;
			cv::imwrite(downloadImgFileName, downloadImg);
			std::cout << "STORAGE COMPLETED!!!  2" << std::endl;
			std::string paramname = "test" + to_string(i) + ".txt";
			StoreParams(paramname, matRB2WS, downloadImgFileName);
			std::cout << "STORAGE COMPLETED!!!  3" << std::endl;

			// load case
			auto it = mapAidGroupCArmCam.find(i + 1);
			if (it != mapAidGroupCArmCam.end())
				vzm::RemoveSceneItem(it->second, true);
			int aidGroup = RegisterCArmImage(sidScene, m_dataPath + "test" + to_string(i) + ".txt", "test" + to_string(i));
			if (aidGroup == -1)
				break;
			mapAidGroupCArmCam[i + 1] = aidGroup;
			MoveCameraToCArmView(i, cidCam, cpInterCams, arAnimationKeyFrame);
			break;
		}
	}
}

void Engine::SaveAndChangeViewState(const track_info& trackInfo, const int keyParam,
	const int sidScene, const int cidCam, const int viewIdx, const cv::Mat& colored_img)
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
			return;
		}

		glm::fmat4x4 mat_r = glm::toMat4(q);
		glm::fmat4x4 mat_t = glm::translate(t);
		glm::fmat4x4 matRB2WS = mat_t * mat_r;

		__gc.g_showCalibMarkers = keyParam == '3';

		if (!colored_img.empty())
		{
			//cv::Mat coloredImg;
			//cv::cvtColor(img, coloredImg, cv::COLOR_GRAY2RGB);
			string downloadImgFileName = __gc.g_folder_trackingInfo + "test" + to_string(i) + ".png";
			cv::imwrite(downloadImgFileName, colored_img);

			StoreParams("test" + to_string(i) + ".txt", matRB2WS, downloadImgFileName);
			std::cout << "\nSTORAGE COMPLETED!!!" << std::endl;
		}

		auto it = __gc.g_mapAidGroupCArmCam.find(EXTRINSIC_PHANTOM_IMG_INDEX);
		if (it != __gc.g_mapAidGroupCArmCam.end())
			vzm::RemoveSceneItem(it->second, true);

		// load case
		it = __gc.g_mapAidGroupCArmCam.find(i);
		if (it != __gc.g_mapAidGroupCArmCam.end())
			vzm::RemoveSceneItem(it->second, true);
		int aidGroup = m_calibrator->RegisterCArmImage(sidScene, __gc.g_folder_trackingInfo + "test" + to_string(i) + ".txt", "test" + to_string(i));
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
				return;

			vzm::ActorParameters apCArmPlane;
			vzm::GetActorParams(aidCArmPlane, apCArmPlane);
			glm::fmat4x4 matCA2WS = apCArmPlane.script_params.GetParam("matCA2WS", glm::fmat4x4(1));

			m_renderer->SetAnimationCamTo(cidCam, viewIdx,
				cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(1, 1), cameraMatrix.at<double>(0, 1),
				cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2), matCA2WS);
		}
		break;
	}
	return;
}
void Engine::MoveCameraToCArmView(int carmIdx, int cidCam, std::vector<vzm::CameraParameters>& cpInterCams, int& arAnimationKeyFrame)
{

	vzm::CameraParameters cpCam;
	if (!vzm::GetCameraParams(cidCam, cpCam))
		return;

	cv::FileStorage fs(m_dataPath + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
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

	float range = std::max((float)(2 - 1), 1.f);
	for (int i = 0; i < 2; i++) {
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

void Engine::slotImageArrived(cv::Mat img)
{
	// image 다운 받았을 때. slot

#ifdef DEBUG
	qDebug() << "Imagearrived called";
#endif
	int sidScene = vzmutils::GetSceneItemIdByName(m_sceneName);

	if (m_isAP)
	{
		m_AP_set = true;
		int cidCam1 = vzmutils::GetSceneItemIdByName(m_camAPName);
		QImage tempimg = QImage((uchar*)img.data, SCANIMG_W, SCANIMG_H, QImage::Format_Grayscale8);
		Image2D temp(m_numImg++, VIEW_TYPE::AP, tempimg);
		m_ImgList.push_back(temp);
		m_cvImgList.push_back(img);


		SaveAndChangeViewState(0, sidScene, cidCam1, m_InterCamsAP, m_arAnimationKeyFrame);
	}
	else
	{
		m_Lateral_set = true;
		int cidCam2 = vzmutils::GetSceneItemIdByName(m_camLateralName);
		QImage tempimg = QImage((uchar*)img.data, SCANIMG_W, SCANIMG_H, QImage::Format_Grayscale8);
		Image2D temp(m_numImg++, VIEW_TYPE::LATERAL, tempimg);
		m_ImgList.push_back(temp);
		m_cvImgList.push_back(img);

		SaveAndChangeViewState(1, sidScene, cidCam2, m_InterCamsLateral, m_arAnimationKeyFrame);
	}
	m_isAP = !m_isAP;
	m_network_processing_thread->setDownloadFlag(false);
}

void Engine::slotSetCalibMode()
{
	m_viewMgr->setCalibMode();
	std::cout << "setCalib triggered" << std::endl;
}

void Engine::slotSetNaviMode()
{
	m_viewMgr->setNaviMode();
	
}

void Engine::slotNumberKey(int key)
{
	qDebug() << "numberinput";
	int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
	track_info trk;
	//__gc.g_track_que.wait_and_pop(trk);
	trk = __gc.g_track_que.front();
	cv::Mat emptyImg;
	SaveAndChangeViewState(trk, key, sidScene, cidCam1, 0, emptyImg);
}

void Engine::slotCalibMousePress(glm::ivec2 pos)
{
	std::cout << pos.x;
}

void Engine::UpdateTrackInfo2Scene(track_info& trackInfo)
{
	int numRBs = trackInfo.NumRigidBodies();
	int numMKs = trackInfo.NumMarkers();

	for (int i = 0; i < numRBs; i++) {
		std::string rbName;
		glm::fmat4x4 matLS2WS;
		if (trackInfo.GetRigidBodyByIdx(i, &rbName, &matLS2WS, NULL, NULL, NULL)) {
			int aidAxis = vzmutils::GetSceneItemIdByName(rbName);
			vzm::ActorParameters apAxis;
			vzm::GetActorParams(aidAxis, apAxis);
			apAxis.is_visible = true;

			apAxis.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidAxis, apAxis);
		}
	}

	for (int i = 0; i < numMKs; i++) {
		std::map<track_info::MKINFO, std::any> mk;
		if (trackInfo.GetMarkerByIdx(i, mk)) {
			std::string mkName = std::any_cast<std::string>(mk[track_info::MKINFO::MK_NAME]);
			glm::fvec3 pos = std::any_cast<glm::fvec3>(mk[track_info::MKINFO::POSITION]);

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
	std::vector<track_info>& trackingFrames = m_trackingFrames;
	int totalFrames = (int)trackingFrames.size();
	int frame = (frameCount++) % totalFrames;
#endif

#ifdef USE_MOTIVE
	track_info trackInfo;

	//for smaller overhead
	__gc.g_track_que.wait_and_pop(trackInfo);

	//image download 됐을 때
	if (__gc.g_renderEvent == RENDER_THREAD_DOWNLOAD_IMG_PROCESS) {
		//__gc.g_renderEvent = RENDER_THREAD_BUSY;
		cv::Mat g_curScanGrayImg(SCANIMG_W, SCANIMG_H, CV_8UC1);
		memcpy(g_curScanGrayImg.ptr(), &__gc.g_downloadImgBuffer[0], __gc.g_downloadImgBuffer.size());


		int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
		int cidCam1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
		int cidCam2 = vzmutils::GetSceneItemIdByName(__gc.g_camName2);

		cv::Mat downloadColorImg;
		cv::cvtColor(g_curScanGrayImg, downloadColorImg, cv::COLOR_GRAY2RGB);
		cv::imwrite(__gc.g_folder_trackingInfo + "test_downloaded.png", downloadColorImg);

		using namespace std;
		if (__gc.g_calribmodeToggle) {
			if (m_calibrator->CalibrationWithPhantom(__gc.g_CArmRB2SourceCS, g_curScanGrayImg, &trackInfo, __gc.g_useGlobalPairs)) {

				cv::Mat processColorImg = cv::imread(__gc.g_folder_trackingInfo + "test_circle_sort.png");
				SaveAndChangeViewState(trackInfo, char('3'), sidScene, cidCam1, 0, processColorImg);
				//__gc.g_calribmodeToggle = false;

				qDebug() << "calib render";
			}
		}
		else {

			// test //
			// __gc.g_CArmRB2SourceCS
			glm::fmat4x4 matCArmRB2WS;
			trackInfo.GetRigidBodyByName("c-arm", &matCArmRB2WS, NULL, NULL, NULL);
			glm::fmat4x4 matSourceCS2WS = matCArmRB2WS * glm::inverse(__gc.g_CArmRB2SourceCS);
			//glm::fvec3 posCArmDet = vzmutils::transformPos(glm::fvec3(0, 0, 0), matCArmRB2WS);
			glm::fvec3 dirCArm = vzmutils::transformVec(glm::fvec3(0, 0, -1), matSourceCS2WS);
			dirCArm = glm::normalize(dirCArm);

			glm::fmat4x4 matCam2WS;
			trackInfo.GetTrackingCamPose(0, matCam2WS);
			glm::fvec3 camRight = vzmutils::transformVec(glm::fvec3(1, 0, 0), matCam2WS);
			camRight = glm::normalize(camRight);

			float horizonAngle = fabs(glm::dot(camRight, dirCArm));
			if (horizonAngle < 0.3f) {
				m_AP_set = true;
				SaveAndChangeViewState(trackInfo, char('1'), sidScene, cidCam1, 0, downloadColorImg);
			}
			else{
				m_Lateral_set = true;
				SaveAndChangeViewState(trackInfo, char('2'), sidScene, cidCam2, 1, downloadColorImg);
			}
		}
		__gc.g_renderEvent = RENDER_THREAD_FREE;
	}

	//track_info* ptrackInfo = NULL;
	//if (!__gc.g_track_que.empty()) {
	//	trackInfo = __gc.g_track_que.front();
	//	ptrackInfo = &trackInfo;
	//}

	m_renderer->RenderTrackingScene(&trackInfo);
#else
	//UpdateTrackInfo2Scene(trackingFrames[frame]);
#endif
	//Render();

}