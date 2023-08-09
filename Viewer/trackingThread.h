#pragma once
#pragma once

#include <algorithm>

#include <QThread>
#include <QDebug>

#include "Defines.h"

#include <opencv2/opencv.hpp>

#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#include "../optitrk/optitrk.h"
#include "../SNaviWin/naviHelpers.hpp"
#include "rapidcsv/rapidcsv.h"
#include "../SNaviWin/CArmCalibration.h"


class Q_DECL_EXPORT trackingThread : public QThread
{
	Q_OBJECT

public:
	trackingThread(QObject* parent = 0);
	~trackingThread();

	navihelpers::concurrent_queue<navihelpers::track_info> m_track_que;

	void pop(navihelpers::track_info& trackInfo) { m_track_que.wait_and_pop(trackInfo); };

	void setalive(bool flag) { m_tracker_alive = flag; };


private:
	void run();

	bool m_tracker_alive;

	
};