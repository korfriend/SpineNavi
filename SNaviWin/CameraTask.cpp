#include "CameraTask.h"

#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#include "rapidcsv/rapidcsv.h"

#include <time.h>
#include <opencv2/opencv.hpp>
#include "../camtrk/camtrk.h"

namespace camtask {

	__GC* __gc = NULL;

	bool InitializeTask(__GC* gcp) {
		__gc = gcp;

		return camtrk::InitCamTrackLib();
	}

	void CameraProcess() {
		if (__gc == NULL) return;

		while (__gc->g_camera_alive)
		{
			camtrk::UpdateCamera();
		}

		camtrk::DeinitCamTrackLib();
	}

	// this is supposed to be called in the main thread
	bool GetCamTrackInfo_Safe(glm::ivec2* colorFbSize, void** colorBuffer, glm::ivec2* depthFbSize, void** depthColoredBuffer) {
		static camtrk::CamTrackInfo camTrkInfo;
		if (!camtrk::GetLatestCamTrackInfo_Safe(camTrkInfo))
			return false;

		if (colorFbSize) *colorFbSize = glm::ivec2(camTrkInfo.color_w, camTrkInfo.color_h);
		if (colorBuffer) *colorBuffer = &camTrkInfo.colorBuffer[0];
		if (depthFbSize) *depthFbSize = glm::ivec2(camTrkInfo.depth_w, camTrkInfo.depth_h);
		if (depthColoredBuffer) *depthColoredBuffer = &camTrkInfo.depthColorCodedBuffer[0];
		// to do

		return true;
	}
}