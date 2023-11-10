#pragma once
#include "GlobalParams.h"

namespace rendertask {
	void InitializeTask(__GC* gcp);
	
	void SceneInit(int initBufW, int initBufH);

	void SetAnimationCamTo(const int cidCam, const int viewIdx,
		const float fx, const float fy, const float s, const float cx, const float cy,
		const glm::fmat4x4& matCS2WS);
	
	void AnimateFrame(const int cidRender1, const int cidRender2);
	void UpdateTrackInfo2Scene(track_info& trackInfo);
}