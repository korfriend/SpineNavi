#include "networkThread.h"


networkThread::networkThread(int port, int bufferSize, int FD, QObject* parent) :
	QThread(parent), m_bufferSize(bufferSize), m_FD(FD)
{
	m_network_alive = true;
	m_download_completed = false;
	m_port = port;

	m_buffer = new char[m_bufferSize];
}

networkThread::~networkThread()
{
	SAFE_DELETE_ARRAY(m_buffer);

	this->wait();
}

void networkThread::run()
{
	qDebug() << "CALL NETWORK THREAD";
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);


	/* create socket */
	SOCKET serverSocket;
	serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serverSocket < 0) {
		printf("can not create socket\n");
		return;
	}

	/* bind socket */
	struct sockaddr_in serverAddr;
	memset(&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_port = htons(m_port);

	if (bind(serverSocket, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
	{
		printf("bind error\n");
		return;
	}

	/* listen */
	if (listen(serverSocket, m_bufferSize) == SOCKET_ERROR) {
		printf("listen error\n");
		return;
	}

	printf("listen %d port...\n", m_port);

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

	int totalBufferNum;
	int BufferNum;
	int readBytes;
	long file_size;
	long totalReadBytes;
	uint numTimeOut = 0;

	while (m_network_alive) {

		while (true && !m_download_completed) {
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

				readBytes = recv(clientSocket, m_buffer, m_bufferSize, 0);
				if (readBytes < 5000) break;
				file_size = atol(m_buffer);

				unsigned char* data = new unsigned char[file_size];
				totalReadBytes = 0;

				//FILE* fp;
				//fopen_s(&fp, (folder_capture + "result.raw").c_str(), "wb");
				std::vector<char> bufferTmp(file_size * 2);
				while (file_size > totalReadBytes) {
					readBytes = recv(clientSocket, m_buffer, m_bufferSize, 0);
					if (readBytes < 0) break;

					//fwrite(buffer, sizeof(char), readBytes, fp);
					memcpy(&bufferTmp[totalReadBytes], m_buffer, readBytes);

					totalReadBytes += readBytes;
					printf("\rIn progress: %d/%d byte(s) [%d]", totalReadBytes, file_size, (int)std::min((totalReadBytes * 100.f) / file_size, 100.f));
					fflush(stdout);
				}
				if (readBytes > 0) {
					printf("download Image");
					m_download_completed = true;
					cv::Mat g_curScanImg(SCANIMG_W, SCANIMG_H, CV_8UC1);
					memcpy(g_curScanImg.ptr(), &bufferTmp[0], file_size);

					emit sigImageArrived(g_curScanImg);
					
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
}
