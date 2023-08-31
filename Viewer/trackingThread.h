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


	void InitializeTask(__GC* gcp);
	concurrent_queue<track_info> m_track_que;

	void pop(track_info& trackInfo) { m_track_que.wait_and_pop(trackInfo); };

	void setalive(bool flag) { m_tracker_alive = flag; };


private:
	void run();

	bool m_tracker_alive;

	__GC* __gc;

	
};