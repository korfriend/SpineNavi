#pragma once
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

#include "Defines.h"
#include "Image2D.h"
#include "networkThread.h"

#include<QWidget>
#include<QString>
#include<QTimer>

class Engine : public QWidget
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