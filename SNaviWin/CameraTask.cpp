#include "CameraTask.h"

#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#include "rapidcsv/rapidcsv.h"

#include <time.h>
#include <opencv2/opencv.hpp>
#include "../camtrk/camtrk.h"

namespace camtask {

	__GC* __gc = NULL;

	void InitializeTask(__GC* gcp) {
		__gc = gcp;

		camtrk::InitCamTrackLib();
	}

	void CameraProcess() {
		if (__gc == NULL) return;

		while (__gc->g_camera_alive)
		{
			camtrk::UpdateCamera();
		}

		camtrk::DeinitCamTrackLib();
	}

	bool GetCamTrackInfo(/*bool& isMoveDetected, image buffers...*/) {
		static camtrk::CamTrackInfo camTrkInfo;
		if (!camtrk::GetLatestCamTrackInfoSafe(camTrkInfo))
			return false;
		// to do

		return true;
	}
}