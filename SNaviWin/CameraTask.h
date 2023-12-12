#pragma once
#include "GlobalParams.h"

namespace camtask {
	void InitializeTask(__GC* gcp);
	void CameraProcess();

	// this function will be called in the main thread
	bool GetCamTrackInfo(/*bool& isMoveDetected, image buffers...*/);
}