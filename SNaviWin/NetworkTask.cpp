#include "TrackingTask.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

//#include <winsock2.h> // udp
#pragma comment(lib,"ws2_32.lib")

#define CFG_LISTEN_PORT 22222
#define CFG_SIZE_MAX_FD 1000
#define CFG_SIZE_MAX_RECV_BUFFER 20000


namespace nettask {

	__GC* __gc = NULL;

	void InitializeTask(__GC* gcp) {
		__gc = gcp;
	}

	int NetworkProcess() {
		if (__gc == NULL) return -1;


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
		unsigned int numTimeOut = 0;

		while (__gc->g_network_alive) {

			while (true && __gc->g_downloadCompleted == 0) {
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

					//int safeTmpBytes = 5000;
					//readBytes = recv(clientSocket, buffer, bufferSize, 0);
					//safeTmpBytes = readBytes;

					file_size = SCANIMG_W * SCANIMG_H;// atol(buffer);

					unsigned char* data = new unsigned char[file_size];
					totalReadBytes = 0;

					//bool isOffsetProcCompleted = false;

					//FILE* fp;
					//fopen_s(&fp, (folder_capture + "result.raw").c_str(), "wb");
					std::vector<char> bufferTmp(file_size * 4);
					while (file_size > totalReadBytes) {
						readBytes = recv(clientSocket, buffer, bufferSize, 0);
						//if (totalTmpBytes < 5000) continue;
						if (readBytes < 0) break;

						//fwrite(buffer, sizeof(char), readBytes, fp);
						memcpy(&bufferTmp[totalReadBytes], buffer, readBytes);

						totalReadBytes += readBytes;
						printf("\rIn progress: %d/%d byte(s) [%d]", totalReadBytes, file_size, (int)std::min((totalReadBytes * 100.f) / file_size, 100.f));
						fflush(stdout);
					}
					if (readBytes > 0) {

						__gc->g_downloadCompleted = 100;
						__gc->g_downloadImgBuffer.clear();
						__gc->g_downloadImgBuffer.assign(file_size, 0);
						memcpy(&__gc->g_downloadImgBuffer[0], &bufferTmp[0], file_size);


						// fence 역할...
						__gc->g_renderEvent = RENDER_THREAD::DOWNLOAD_IMG_PROCESS;
						while (__gc->g_renderEvent != RENDER_THREAD::FREE) Sleep(5);

						__gc->g_downloadCompleted = 99;
						//cv::imshow("test", g_curScanImg);
						//cv::waitKey(1);
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
}