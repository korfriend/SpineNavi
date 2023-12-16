#pragma once
#include "GlobalParams.h"

namespace trackingtask {
	bool InitializeTask(__GC* gcp);

	void OptiTrackingProcess();

	// this function will be called in the main thread
	bool GetOptiTrackCamera_Safe(const int camIdx, const int w, const int h, void* buffer);

	bool DebugTest();
}