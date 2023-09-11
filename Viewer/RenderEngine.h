#pragma once
#include<QWidget>
#include<qevent.h>

#include "VisMtvApi.h"
#include "ApiUtility.hpp"

#include "../SNaviWin/naviHelpers.hpp"

#include <opencv2/opencv.hpp>

#include "Defines.h"

class Q_DECL_EXPORT RenderEngine : public QWidget
{

	Q_OBJECT


public:

	RenderEngine();
	~RenderEngine();

	void InitializeTask(__GC* gcp);

	void SceneInit();

	void SetAnimationCamTo(const int cidCam, const int viewIdx,
		const float fx, const float fy, const float s, const float cx, const float cy,
		const glm::fmat4x4& matCS2WS);

	void RenderTrackingScene(const track_info* trackInfo);
	void UpdateTrackInfo2Scene(track_info& trackInfo);

signals:
	void signumberkey(int);

private slots:
	void slotEventHandler(QEvent* pEvent);

private:

	__GC* __gc;

	const int g_numAnimationCount;
	std::vector<int> g_arAnimationKeyFrame;
	std::vector<vzm::CameraParameters> g_cpInterCams[2];
};