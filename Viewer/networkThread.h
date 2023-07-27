#pragma once

#include <windowsx.h>
#include <winsock2.h> // udp

#pragma comment(lib, "ws2_32.lib")

#include <algorithm>

#include <QThread>
#include <QDebug>

#include "Defines.h"

#include <opencv2/opencv.hpp>


class Q_DECL_EXPORT networkThread : public QThread
{
	Q_OBJECT

public:
	networkThread(int port = 22222, int bufferSize = 5000, int FD = 1000, QObject* parent = 0);
	~networkThread();

	void setAlive(bool flag) { m_network_alive = flag; };
	void setDownloadFlag(bool flag) { m_download_completed = flag; };


signals:

	void sigImageArrived(cv::Mat);

private:
	void run();

	bool m_network_alive;
	bool m_download_completed;

	unsigned short m_port;
	const int m_bufferSize;
	const int m_FD;

	char* m_buffer;
};