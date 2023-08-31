#pragma once

#include<QWidget>
#include<QString>
#include<QTimer>
#include<QImage>


#include "networkThread.h"
#include "Image2D.h"
#include "ViewLayout.h"
#include "ViewMgr.h"
#include "trackingThread.h"
#include "RenderEngine.h"
#include "CalibrationEngine.h"

#include <regex>
#include <vector>
#include <iostream>
#include <stdio.h>

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
#include "../optitrk/optitrk.h"
#include "../SNaviWin/naviHelpers.hpp"
#include "rapidcsv/rapidcsv.h"
#include "../SNaviWin/CArmCalibration.h"
#include "ApiUtility.hpp"

#include "Defines.h"

class Q_DECL_EXPORT Engine : public QWidget
{

	Q_OBJECT

public:

	Engine(ViewLayout* layout, QString dataPath ,QString optiProfilePath, QString optiCalfilePath);
	~Engine();

	void EngineInit();
	void SceneInit();
	void setCameraPrams(int id, vzm::CameraParameters prams);

	void UpdateTrackInfo2Scene(track_info& trackInfo);
	void Render();

	void StoreParams(std::string& paramsFileName, glm::fmat4x4& matRB2WS, std::string& imgFileName);
	void StoreParams(const std::string& paramsFileName, const glm::fmat4x4& matCArmRB2WS, const std::string& imgFileName);

	void SaveAndChangeViewState(int keyParam, int sidScene, int cidCam,
		std::vector<vzm::CameraParameters>& cpInterCams, int& arAnimationKeyFrame);

	void SaveAndChangeViewState(const track_info& trackInfo, const int keyParam,
		const int sidScene, const int cidCam, const int viewIdx, const cv::Mat& colored_img);
	void MoveCameraToCArmView(int carmIdx, int cidCam, std::vector<vzm::CameraParameters>& cpInterCams, int& arAnimationKeyFrame);


private slots:
	
	void TimerProc();
	void slotImageArrived(cv::Mat img);
	void slotSetCalibMode();
	void slotSetNaviMode();

private:

	//optitrack info files
	bool m_optitrackMode;
	QString m_optiProfilePath;
	QString m_optiCalfilepath;

	// Camera
	std::string m_camAPName;
	vzm::CameraParameters m_cpCamAP;
	std::vector<vzm::CameraParameters> m_InterCamsAP;
	int m_cidCamAP;
	int m_arAnimationKeyFrame;

	std::string m_camLateralName;
	vzm::CameraParameters m_cpCamLateral;
	std::vector<vzm::CameraParameters> m_InterCamsLateral;
	int m_cidCamLateral;

	//Light
	vzm::LightParameters m_Light1;
	int m_lidLight1;

	//Scene
	int m_sidScene;
	std::string m_sceneName;

	//timer for rendering
	QTimer* m_qtimer;

	//thread
	networkThread* m_network_processing_thread;
	trackingThread* m_trackingThread;

	std::vector<Image2D> m_ImgList;
	std::vector<cv::Mat> m_cvImgList;
	int m_numImg;

	// without optitrack
	std::vector<track_info> m_trackingFrames;

	//UI
	ViewMgr* m_viewMgr;

	//intrinsic
	cv::Mat m_cameraMatrix;

	//image type flag
	bool m_isAP;
	bool m_AP_set;
	bool m_Lateral_set;

	// dataPath
	std::string m_dataPath;

	//track info
	concurrent_queue<track_info>* m_track_que;

	//Rendering Engine
	RenderEngine* m_renderer;
	
	//Calibration Engine
	CalibrationEngine* m_calibrator;
	__GC __gc;
};