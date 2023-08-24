#pragma once
#include "GlobalParams.h"

namespace rendertask {
	void InitializeTask(__GC* gcp);
	
	void SceneInit(const HWND view1Hwnd, const RECT rcWorldView1, const HWND view2Hwnd, const RECT rcWorldView2);

	void SetAnimationCamTo(const int cidCam, const int viewIdx,
		const float fx, const float fy, const float s, const float cx, const float cy, 
		const glm::fmat4x4& matCS2WS);
	
	void RenderTrackingScene(const track_info* trackInfo);

}