#pragma once

#include<QWidget>
#include<QString>
#include<QTimer>

#include "Defines.h"
#include "networkThread.h"
#include "Image2D.h"

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


class Q_DECL_EXPORT Engine : public QWidget
{

	Q_OBJECT

public:

	Engine(QString optiProfilePath, QString optiCalfilePath);
	~Engine();

	void EngineInit();
	void SceneInit();
	void setCameraPrams(int id, vzm::CameraParameters prams);

	void UpdateTrackInfo2Scene(navihelpers::track_info& trackInfo);
	void Render(vzm::CameraParameters cpCam);


private slots:
	
	void TimerProc();
	void slotImageArrived(cv::Mat img);

private:

	//optitrack info files
	bool m_optitrackMode;
	QString m_optiProfilePath;
	QString m_optiCalfilepath;

	// Camera
	vzm::CameraParameters m_cpCam1;
	int m_cidCam1;

	//Light
	vzm::LightParameters m_Light1;
	int m_lidLight1;

	//Scene
	int m_sidScene;

	//timer for rendering
	QTimer* m_qtimer;

	//thread
	networkThread* m_network_processing_thread;

	std::vector<Image2D> m_ImgList;
	int m_numImg;
	std::vector<navihelpers::track_info> m_trackingFrames;

	
};