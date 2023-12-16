#pragma once
#include "GlobalParams.h"

namespace camtask {
	bool InitializeTask(__GC* gcp);
	void DeinitializeTask();
	void CameraProcess();

	// this function will be called in the main thread
	bool GetCamTrackInfo_Safe(glm::ivec2* colorFbSize, void** colorBuffer, glm::ivec2* depthFbSize, void** depthColoredBuffer/*bool& isMoveDetected, image buffers...*/);
}