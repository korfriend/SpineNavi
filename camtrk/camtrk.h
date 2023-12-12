#pragma once

#ifndef __dojostatic
#define __dojostatic extern "C" __declspec(dllexport)
#endif

#include <string>
#include <vector>

namespace camtrk {
	struct CamTrackInfo {
		int color_w, color_h;
		std::vector<unsigned char> colorBuffer; // CV_8UC3
		int depth_w, depth_h;
		std::vector<unsigned char> depthColorCodedBuffer; // CV_8UC3
		// pose...
		// ...
	};

	__dojostatic bool InitCamTrackLib();
	__dojostatic bool DeinitCamTrackLib();

	__dojostatic void SetAlignmentDir(bool toColorDir);
	__dojostatic bool UpdateCamera();
	__dojostatic bool GetLatestCamTrackInfoSafe(CamTrackInfo& camtrkInfo);

	// add your functions, referring to optitrk.h
}
