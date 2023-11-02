#include "RenderingTask.h"

#include "VisMtvApi.h"
#include "ApiUtility.hpp"

#include "naviHelpers.hpp"

#include <opencv2/opencv.hpp>


// DOJO : Scene 은 Camera(s), Light source(s), Actor(s)로 구성, World Space 의 Coordinate System 을 정의함 
// 
// we set 'meter scale world to world space

// DOJO : Actor 는 Scene 에 그려지는 Model 이며, 
// Actor 를 구성하는 actor resource (called resource object) 는 geometry 및 material (e.g., texture) 등으로 정의
// vzm::GenerateXXXObject 는 actor resource 를 생성 
// vzm::LoadXXX 는 file 로부터 actor resource 를 생성
// actor resource 의 geometry 는 기본적으로 Object Space (Model Space) 에서 정의된 것으로 봄

namespace rendertask {

	__GC* __gc = NULL;

	void InitializeTask(__GC* gcp) {
		__gc = gcp;
	}

	void SceneInit(int initBufW, int initBufH) {
		if (__gc == NULL) return;

		// DOJO : Axis 를 위한 actor resource 를 생성
		// oidAxis (objId) 은 생성된 actor resource 의 ID 를 받아옴
		// objId = 0 인 경우, 항상 새로운 actor resource 를 생성하고 새로운 ID 를 받음
		// objId != 0 인 경우, 해당 ID 의 actor resource 가 있을 때, 해당 actor resource 를 update 함 (ID 는 그대로 사용)
		// objId != 0 인 경우, 해당 ID 의 actor resource 가 없을 때, 새로운 actor resource 를 생성하고 새로운 ID 를 받음
		int oidAxis = 0;
		vzm::GenerateAxisHelperObject(oidAxis, 0.5f);

		// DOJO : Actor Parameter (vzm::ActorParameters) 와 actor resource 로 actor 생성
		vzm::ActorParameters apAxis;
		// vzm::ActorParameters::SetResourceID 로 다양한 actor resource 를 actor 에 할당할 수 있음
		// 기본적으로 vzm::ActorParameters::RES_USAGE::GEOMETRY 가 있어야 함, 그 외 여러 종류의 texture 들이 올 수 있음 (ref. vzm::ActorParameters::RES_USAGE)
		apAxis.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidAxis);
		// line 으로 정의된 geometry 에 대해 렌더링에서 사용되는 parameter (c.f., triangle geometry 의 경우 해당 parameter 가 렌더링에서 사용되지 않음)
		apAxis.line_thickness = 3;
		// color 의 경우, 해당 actor 에 대해 global 로 적용됨 
		// geometry resource 가 vertex 단위로 color 가 정의되어 있다면, 여기선 alpha 값만 사용됨
		*(glm::fvec4*)apAxis.color = glm::fvec4(1, 1, 1, 1);
		apAxis.phong_coeffs[1] = 1.f;
		// DOJO : 새로운 액터 생성
		// actor name parameter 와 actor parameter 필요
		// actor name parameter 로 actor 의 ID 를 가져올 수 있음
		int aidAxis = 0;
		vzm::NewActor(apAxis, "World Axis", aidAxis);

		// DOJO : navihelpers 는 이번 프로젝트에서 사용할 헬퍼 함수를 담고 있음
		int oidGrid, oidLines, oidTextX, oidTextZ;
		// scene 의 바닥에 그려지는 actor resources 를 생성
		navihelpers::World_GridAxis_Gen(oidGrid, oidLines, oidTextX, oidTextZ);

		std::vector<glm::fvec3> pinfo(3);
		pinfo[0] = glm::fvec3(0, 0.5, 0);
		pinfo[1] = glm::fvec3(0, 0, -1);
		pinfo[2] = glm::fvec3(0, 1, 0);
		// DOJO : "Frame" 이라는 text 를 그리는 actor resource 를 생성 (자세한 사용법은 문의 바람)
		// 여기서 int oidFrameText; 이렇게 했는데, 새롭게 생성하는 경우 oidFrameText = 0 을 권장
		// DEBUG 모드에서는 initialize 안 하면, 보통 0 값이 들어 가지만, 
		// RELEASE 모드에서는 initialize 안 하면, 임의의 값이 들어 가고, 이 때, (거의 발생하진 않지만) 다른 actor resource 의 ID 값이 들어가면 비정상 동작할 수 있음
		//int oidFrameText; // recommend "int oidFrameText = 0;"
		//vzm::GenerateTextObject((float*)&pinfo[0], "Frame", 0.1, true, false, oidFrameText);

		vzm::ActorParameters apGrid, apLines, apTextX, apTextZ, apTextFrame;
		apGrid.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidGrid);
		apLines.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidLines);
		apTextX.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidTextX);
		apTextZ.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidTextZ);
		//apTextFrame.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidFrameText);
		*(glm::fvec4*)apTextX.phong_coeffs = glm::fvec4(0, 1, 0, 0);
		*(glm::fvec4*)apTextZ.phong_coeffs = glm::fvec4(0, 1, 0, 0);
		//*(glm::fvec4*)apTextFrame.phong_coeffs = glm::fvec4(0, 1, 0, 0);

		int aidGrid = 0, aidLines = 0, aidTextX = 0, aidTextZ = 0;// , aidTextFrame;
		vzm::NewActor(apGrid, "World Grid", aidGrid);
		vzm::NewActor(apLines, "World Lines", aidLines);
		vzm::NewActor(apTextX, "World Text X", aidTextX);
		vzm::NewActor(apTextZ, "World Text Y", aidTextZ);
		//vzm::NewActor(apTextFrame, "Frame Text", aidTextFrame);

		// DOJO : (world view 에서의) Camera 를 정의하기 위한 파라미터
		vzm::CameraParameters cpCam;
		*(glm::fvec3*)cpCam.pos = glm::fvec3(-1.5, 1.5, -1.5);
		*(glm::fvec3*)cpCam.up = glm::fvec3(0, 1, 0);
		*(glm::fvec3*)cpCam.view = glm::fvec3(1, -1, 1);
		cpCam.displayActorLabel = cpCam.displayCamTextItem = true;

		// YAML 로 이전에 저장된 (world view 에서의) 카메라 위치 정보 읽어 들임
		cv::FileStorage fs(__gc->g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
		if (fs.isOpened()) {
			cv::Mat ocvVec3;
			fs["POS"] >> ocvVec3;
			memcpy(cpCam.pos, ocvVec3.ptr(), sizeof(float) * 3);
			fs["VIEW"] >> ocvVec3;
			memcpy(cpCam.view, ocvVec3.ptr(), sizeof(float) * 3);
			fs["UP"] >> ocvVec3;
			memcpy(cpCam.up, ocvVec3.ptr(), sizeof(float) * 3);
			fs.release();
		}

		// 렌더링될 buffer 사이즈
		cpCam.w = initBufW;
		cpCam.h = initBufH;
		cpCam.skip_sys_fb_update = true;

		// 렌더링 파이프라인의 View Frustum 에서의 near plane, far plane distances
		cpCam.np = 0.15f;
		cpCam.fp = 100.f;

		float vFov = 3.141592654f / 4.f;
		// NOTE : union 으로 저장되어 있음
		// 카메라 view projection 의 자연스러운 이동 (to C-arm View) 을 위해, 
		// 카메라 설정 convention 을 camera (intrinsic) parameters 로 사용
		if (0) {
			cpCam.fov_y = vFov;
			cpCam.aspect_ratio = (float)cpCam.w / (float)cpCam.h;
			cpCam.projection_mode = vzm::CameraParameters::CAMERA_FOV;
		}
		else {
			float aspect_ratio = (float)cpCam.w / (float)cpCam.h;
			float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;

			//  fy = h/(2 * tan(f_y / 2)) 
			cpCam.fx = cpCam.w / (2.f * tan(hFov / 2.f));
			cpCam.fy = cpCam.h / (2.f * tan(vFov / 2.f));
			cpCam.sc = 0;
			cpCam.cx = cpCam.w / 2.f;
			cpCam.cy = cpCam.h / 2.f;
			cpCam.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
		}

		//cpCam1.SetOrthogonalProjection(true);
		// DOJO : scene 에 등록될 카메라 생성, 여기선 두 개 생성 (AP, Lateral 용)
		int cidCam1 = 0, cidCam2 = 0;
		vzm::NewCamera(cpCam, __gc->g_camName, cidCam1);
		vzm::NewCamera(cpCam, __gc->g_camName2, cidCam2);

		// DOJO : Light 를 정의하기 위한 파라미터
		vzm::LightParameters lpLight1;
		lpLight1.is_on_camera = true;
		lpLight1.is_pointlight = false;
		*(glm::fvec3*)lpLight1.pos = *(glm::fvec3*)cpCam.pos;
		*(glm::fvec3*)lpLight1.dir = *(glm::fvec3*)cpCam.view;
		*(glm::fvec3*)lpLight1.up = *(glm::fvec3*)cpCam.up;
		int lidLight1 = 0;
		// DOJO : scene 에 등록될 Light 생성
		vzm::NewLight(lpLight1, "World Light", lidLight1);

		// DOJO : scene 생성
		int sidScene = 0;
		vzm::NewScene(__gc->g_sceneName, sidScene);

		// DOJO : scene 에 camera 1 (cidCam1) 을 연결 
		vzm::AppendSceneItemToSceneTree(cidCam1, sidScene);
		// DOJO : scene 에 camera 2 (cidCam2) 을 연결 
		vzm::AppendSceneItemToSceneTree(cidCam2, sidScene);
		// DOJO : scene 에 Light (lidLight1) 을 연결 
		vzm::AppendSceneItemToSceneTree(lidLight1, sidScene);
		// DOJO : scene 에 axis actor (aidAxis) 연결 
		vzm::AppendSceneItemToSceneTree(aidAxis, sidScene);
		// DOJO : scene 에 ground grid actor (aidGrid) 을 연결 
		vzm::AppendSceneItemToSceneTree(aidGrid, sidScene);
		// DOJO : scene 에 ground grid 에서의 진한 라인 actor (aidLines) 을 연결 
		vzm::AppendSceneItemToSceneTree(aidLines, sidScene);
		// DOJO : scene 에 ground grid 위에 있는 "X" 글자 actor (aidTextX) 을 연결 
		vzm::AppendSceneItemToSceneTree(aidTextX, sidScene);
		// DOJO : scene 에 ground grid 위에 있는 "Z" 글자 actor (aidTextZ) 을 연결 
		vzm::AppendSceneItemToSceneTree(aidTextZ, sidScene);
		// DOJO : scene 에 "Frame" 글자 actor (aidTextFrame) 을 연결 
		//vzm::AppendSceneItemToSceneTree(aidTextFrame, sidScene);
		// NOTE : 여기에서는 모든 actor 및 camera, light 등 scene item 들이 모두 scene (i.e., root) 에 바로 연결되었지만,
		// 특정 actor 의 자식으로 붙을 수도 있다 (e.g., scene<-aidAxis<-aidTextZ<-cidCam1 ,...)
	}

	// DOJO : 특별한 기능은 아니며, 카메라 전환 시 부드러운 변화를 위해 slerp 로 카메라 정보를 저장한 구조체
	const int g_numAnimationCount = 50;
	std::vector<int> g_arAnimationKeyFrame = { -1, -1 };
	std::vector<vzm::CameraParameters> g_cpInterCams[2] = { std::vector<vzm::CameraParameters>(g_numAnimationCount), std::vector<vzm::CameraParameters>(g_numAnimationCount) };

	void SetAnimationCamTo(const int cidCam, const int viewIdx,
		const float fx, const float fy, const float s, const float cx, const float cy, 
		const glm::fmat4x4& matCS2WS)
	{
		if (__gc == NULL) return;

		if (viewIdx >= g_arAnimationKeyFrame.size() || viewIdx < 0)
			return;

		vzm::CameraParameters cpCam;
		if (!vzm::GetCameraParams(cidCam, cpCam))
			return;

		vzm::CameraParameters cpNewCam = cpCam;

		glm::fvec3 posPrev = *(glm::fvec3*)cpCam.pos;
		glm::fvec3 viewPrev = *(glm::fvec3*)cpCam.view;
		glm::fvec3 upPrev = *(glm::fvec3*)cpCam.up;

		UINT widthWindow = cpCam.w;
		UINT heightWindow = cpCam.h;

		const float intrinsicRatioX = (float)SCANIMG_W / widthWindow;
		const float intrinsicRatioY = (float)SCANIMG_H / heightWindow;

		cpNewCam.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_INTRINSICS;
		glm::fvec3 pos(0, 0, 0);
		glm::fvec3 view(0, 0, 1);
		glm::fvec3 up(0, -1, 0);
		pos = *(glm::fvec3*)cpNewCam.pos = vzmutils::transformPos(pos, matCS2WS);
		view = *(glm::fvec3*)cpNewCam.view = vzmutils::transformVec(view, matCS2WS);
		up = *(glm::fvec3*)cpNewCam.up = vzmutils::transformVec(up, matCS2WS);
		cpNewCam.fx = fx / intrinsicRatioX;
		cpNewCam.fy = fy / intrinsicRatioY;
		cpNewCam.sc = s;
		cpNewCam.cx = cx / intrinsicRatioX;
		cpNewCam.cy = cy / intrinsicRatioY;
		cpNewCam.np = 0.15f;

		glm::fquat q1 = glm::quatLookAtRH(glm::normalize(viewPrev), upPrev); // to world
		//glm::fvec3 _o(0, 0, 0);
		//glm::fmat4x4 m1 = glm::lookAtRH(_o, viewPrev, upPrev);
		//glm::fmat4x4 m2 = glm::toMat4(q1);
		//glm::fmat4x4 m3 = glm::inverse(m2);
		glm::fquat q2 = glm::quatLookAtRH(glm::normalize(view), up);

		float range = std::max((float)(g_numAnimationCount - 1), 1.f);

		std::vector<vzm::CameraParameters>& cpInterCams = g_cpInterCams[viewIdx];

		for (int i = 0; i < g_numAnimationCount; i++) {
			float t = (float)i / range; // 0 to 1
			glm::fquat q = glm::slerp(q1, q2, t);
			glm::fmat4x4 invMatLookAt = glm::toMat4(q);
			//glm::fmat4x4 invMat = glm::inverse(matLookAt);
			glm::fvec3 _view(0, 0, -1);
			glm::fvec3 _up(0, 1, 0);
			_view = vzmutils::transformVec(_view, invMatLookAt);
			_up = vzmutils::transformVec(_up, invMatLookAt);

			vzm::CameraParameters& _cpCam = cpInterCams[i];
			_cpCam = cpNewCam;
			*(glm::fvec3*)_cpCam.pos = posPrev + (pos - posPrev) * t;
			*(glm::fvec3*)_cpCam.view = _view;
			*(glm::fvec3*)_cpCam.up = _up;

#define INTERPOLCAM(PARAM) _cpCam.PARAM = cpCam.PARAM + (cpNewCam.PARAM - cpCam.PARAM) * t;

			//INTERPOLCAM(w);
			//INTERPOLCAM(h);
			INTERPOLCAM(fx);
			INTERPOLCAM(fy);
			INTERPOLCAM(sc);
			INTERPOLCAM(cx);
			INTERPOLCAM(cy);
		}

		// safe set
		{
			cpInterCams[g_numAnimationCount - 1] = cpNewCam;
		}

		g_arAnimationKeyFrame[viewIdx] = 0;
	}

	// DOJO : Tracking 되는 정보를 Scene 에 반영시키는 함수
	// Timer 에 등록된 CALLBACK TimerProc 에서 호출됨 (note: Timer 가 본 어플리케이션에서 rendering thread 로 사용됨)
	// track_info& trackInfo : 트래킹 Thread 에서 concurrent queue (또는 async queue 라도고 함) 로 저장된 가장 최신 정보
	// numRBs : rigid bodies
	// numMKs : 개별 markers
	void UpdateTrackInfo2Scene(track_info& trackInfo)
	{
		using namespace std;
		using namespace navihelpers;
		using namespace glm;

		if (__gc == NULL) return;

		int sidScene = vzmutils::GetSceneItemIdByName(__gc->g_sceneName);

		// Register Actors //
		
		int aidGroupCams = vzmutils::GetSceneItemIdByName("Tracking CAM Group");
		if (aidGroupCams == 0) {
			vzm::ActorParameters apGroupCams;
			vzm::NewActor(apGroupCams, "Tracking CAM Group", aidGroupCams);
			vzm::AppendSceneItemToSceneTree(aidGroupCams, sidScene);
			for (int i = 0; i < 3; i++) {
				int aidCamTris = 0, aidCamLines = 0, aidCamLabel = 0;
				vzm::ActorParameters apCamRes;
				vzm::NewActor(apCamRes, "Cam" + to_string(i) + " Tris", aidCamTris);
				vzm::NewActor(apCamRes, "Cam" + to_string(i) + " Lines", aidCamLines);
				vzm::NewActor(apCamRes, "Cam" + to_string(i) + " Label", aidCamLabel);
				vzm::AppendSceneItemToSceneTree(aidCamTris, aidGroupCams);
				vzm::AppendSceneItemToSceneTree(aidCamLines, aidGroupCams);
				vzm::AppendSceneItemToSceneTree(aidCamLabel, aidGroupCams);
			}
		}
		else {
			static int oidCamTris[3]{}, oidCamLines[3]{}, oidCamLabel[3]{};
			for (int i = 0; i < 3; i++) {
				fmat4x4 matCam2WS(1);
				if (!trackInfo.GetTrackingCamPose(i, matCam2WS))
					__gc->g_engineLogger->error("camera {} not found!", i);

				int aidCamTris = vzmutils::GetSceneItemIdByName("Cam" + to_string(i) + " Tris");
				int aidCamLines = vzmutils::GetSceneItemIdByName("Cam" + to_string(i) + " Lines");
				int aidCamLabel = vzmutils::GetSceneItemIdByName("Cam" + to_string(i) + " Label");
				vzm::ActorParameters apCamTris, apCamLines, apCamLabel;
				vzm::GetActorParams(aidCamTris, apCamTris);
				vzm::GetActorParams(aidCamLines, apCamLines);
				vzm::GetActorParams(aidCamLabel, apCamLabel);

				Cam_Gen(matCam2WS, "CAM" + to_string(i), oidCamTris[i], oidCamLines[i], oidCamLabel[i]);

				apCamTris.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamTris[i]);
				apCamTris.color[3] = 0.5f;
				apCamLines.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLines[i]);
				*(fvec4*)apCamLines.phong_coeffs = fvec4(0, 1, 0, 0);
				apCamLines.line_thickness = 2;
				apCamLabel.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLabel[i]);

				vzm::SetActorParams(aidCamTris, apCamTris);
				vzm::SetActorParams(aidCamLines, apCamLines);
				vzm::SetActorParams(aidCamLabel, apCamLabel);
			}
		}
		
		static int oidAxis = 0;
		static int oidMarker = 0;
		static int oidProbe = 0;
		// axis 를 그리는 resource object 생성
		if (vzm::GetResObjType(oidAxis) == vzm::ResObjType::UNDEFINED) {
			vzm::GenerateAxisHelperObject(oidAxis, 0.15f);
		}
		// spherical marker 를 그리는 resource object 생성
		if (vzm::GetResObjType(oidMarker) == vzm::ResObjType::UNDEFINED) {
			glm::fvec4 pos(0, 0, 0, 1.f);
			vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);
		}
		// probe 를 그리는 resource object 생성
		if (vzm::GetResObjType(oidProbe) == vzm::ResObjType::UNDEFINED) {
			glm::fvec3 posS(0, 0, 0.25f);
			glm::fvec3 posE(0, 0, 0);
			//vzm::GenerateArrowObject(__FP posS, __FP posE, 0.0025f, 0.0025f, oidProbe);
			glm::fvec3 pp[2] = { posE , posS };
			float rr[2] = { 0.001, 0.001 };
			//vzm::GenerateCylindersObject((float*)pp, __FP posE, 0.0025f, 0.0025f, oidProbe);
			vzm::GenerateCylindersObject((float*)pp, (float*)rr, NULL, 1, oidProbe);
			//glm::fvec4 pos(0, 0, 0, 0.001f);
			//vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidProbe);
		}

		const int numRBs = trackInfo.NumRigidBodies();
		for (int i = 0; i < numRBs; i++) {
			string rbName;
			float rb_error = 0;
			bitset<128> rb_cid = 0;
			fmat4x4 matRbLS2WS(1);
			map<string, map<track_info::MKINFO, std::any>> rbmkSet;

			// 여기에서 index 는 optitrack lib 에서 사용하는 index 와 다르다! track_info 구조체에서 사용하는 index 임!
			bool isRbTracked = trackInfo.GetRigidBodyByIdx(i, &rbName, &matRbLS2WS, &rb_error, &rbmkSet, &rb_cid);
			int aidRb = vzmutils::GetSceneItemIdByName(rbName);
			vzm::ActorParameters apRb;
			if (aidRb == 0) {
				if (rbName == "probe") {
					apRb.SetResourceID(vzm::ActorParameters::GEOMETRY, oidProbe);
					*(glm::fvec4*)apRb.color = glm::fvec4(1, 0.3, 0.2, 1);
				}
				else {
					apRb.SetResourceID(vzm::ActorParameters::GEOMETRY, oidAxis);
					apRb.line_thickness = 3;
					apRb.phong_coeffs[1] = 1.f;
				}

				vzm::NewActor(apRb, rbName, aidRb);
				vzm::AppendSceneItemToSceneTree(aidRb, sidScene);
			}
			vzm::GetActorParams(aidRb, apRb);
			apRb.label.textStr = rbName;
			apRb.label.fontSize = 12.f;
			if (isRbTracked) {
				apRb.is_visible = true;
				apRb.SetLocalTransform(__FP matRbLS2WS);
			}
			else {
				apRb.is_visible = false;
			}

			if (rbName == "t-needle" || rbName == "troca") {

				int aidToolBody = vzmutils::GetSceneItemIdByName(rbName + ":Body");
				int aidToolTip = vzmutils::GetSceneItemIdByName(rbName + ":Tip");
				vzm::ActorParameters apToolBody, apToolTip;
				if (aidToolBody == 0) {
					vzm::NewActor(apToolBody, rbName + ":Body", aidToolBody);
					apToolTip.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
					*(glm::fvec4*)apToolTip.color = glm::fvec4(1, 0.3, 0.7, 1.f);
					apToolTip.phong_coeffs[1] = 0.7f;
					vzm::NewActor(apToolTip, rbName + ":Tip", aidToolTip);
					vzm::AppendSceneItemToSceneTree(aidToolBody, aidRb);
					vzm::AppendSceneItemToSceneTree(aidToolTip, aidRb);
				}
				vzm::GetActorParams(aidToolBody, apToolBody);
				vzm::GetActorParams(aidToolTip, apToolTip);
				bool isPivoted = __gc->g_rbLocalUserPoints.find(rbName) != __gc->g_rbLocalUserPoints.end();
				if (isRbTracked && isPivoted) {

					apRb.is_visible = false;
					apToolBody.is_visible = true;

					{
						vector<fvec3> toolUserData = __gc->g_rbLocalUserPoints[rbName];
						// compute the line geometry in the tool's rigid body
						fvec3 linePos[2] = { toolUserData[0], fvec3(0) };
						static int oidToolLine = 0;
						vzm::GenerateLinesObject((float*)linePos, NULL, 1, oidToolLine);

						apToolBody.SetResourceID(vzm::ActorParameters::GEOMETRY, oidToolLine);
						apToolBody.line_thickness = 3;
						apToolBody.phong_coeffs[1] = 1.f;
						*(glm::fvec4*)apToolBody.color = glm::fvec4(1, 1, 1, 1);
					}

					apToolTip.is_visible = true;
					glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.002f)); // set 1 cm to the marker diameter
					apToolTip.SetLocalTransform(__FP matScale);
				}
				else {
					apToolBody.is_visible = false;
					apToolTip.is_visible = false;
				}
				vzm::SetActorParams(aidToolBody, apToolBody);
				vzm::SetActorParams(aidToolTip, apToolTip);
			}
			
			// Rb markers //
			
			int aidGroupRbMks = vzmutils::GetSceneItemIdByName(rbName + ":Markers");
			int numPrevRbMks = apRb.test_params.GetParam("numRbMarkers", (int)0);
			int numCurRbMks = (int)rbmkSet.size();
			vzm::ActorParameters apRbMk;
			if (aidGroupRbMks == 0) {
				vzm::NewActor(apRbMk, rbName + ":Markers", aidGroupRbMks);
				vzm::AppendSceneItemToSceneTree(aidGroupRbMks, aidRb);
			}
			else if (numPrevRbMks != numCurRbMks) {
				vzm::RemoveSceneItem(aidGroupRbMks);
				vzm::NewActor(apRbMk, rbName + ":Markers", aidGroupRbMks);
				vzm::AppendSceneItemToSceneTree(aidGroupRbMks, aidRb);
			}
			apRb.test_params.SetParam("numRbMarkers", numCurRbMks);

			glm::fmat4x4 matWS2RbLS = glm::inverse(matRbLS2WS);
			for (auto it = rbmkSet.begin(); it != rbmkSet.end(); it++) {
				string rbmkName = it->first;
				int aidRbMk = vzmutils::GetSceneItemIdByName(rbmkName);
				if (aidRbMk == 0) {
					apRbMk.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
					*(glm::fvec4*)apRbMk.color = glm::fvec4(1.f, 0.f, 0.f, 1.0f); // rgba
					vzm::NewActor(apRbMk, rbmkName, aidRbMk);
					vzm::AppendSceneItemToSceneTree(aidRbMk, aidGroupRbMks);
				}
				vzm::GetActorParams(aidRbMk, apRbMk);
				auto& rbmkInfo = it->second;

				bool isTrackedMk = std::any_cast<bool>(rbmkInfo[track_info::MKINFO::TRACKED]);
				if (isTrackedMk) {
					apRbMk.is_visible = true;

					fvec3 posWS = std::any_cast<fvec3>(rbmkInfo[track_info::MKINFO::POSITION]);
					fvec3 posRbLS = vzmutils::transformPos(posWS, matWS2RbLS);

					glm::fmat4x4 matT = glm::translate(posRbLS);
					glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.005f)); // set 0.7 cm to the marker diameter

					matT = matT * matScale;
					apRbMk.is_visible = true;
					apRbMk.is_pickable = false;
					//apRbMk.label.textStr = to_string(aidRbMk);
					apRbMk.SetLocalTransform(__FP matT);
				}
				else {
					apRbMk.is_visible = false;
				}

				vzm::SetActorParams(aidRbMk, apRbMk);
			}

			vzm::SetActorParams(aidRb, apRb);
		} // for (int i = 0; i < numRBs; i++)

		/**/
		auto displayMarkers = [&sidScene](const map<string, fvec3>& mks, const string& groupName
			, const fvec4& mkColor, const float r, const bool isPickable = false, const bool isVisible = true) {
			int numCurMKs = (int)mks.size();
			int aidGroupMks = vzmutils::GetSceneItemIdByName(groupName);
			vzm::ActorParameters apMkGroup;
			if (aidGroupMks == 0) {
				vzm::NewActor(apMkGroup, groupName, aidGroupMks);
				vzm::AppendSceneItemToSceneTree(aidGroupMks, sidScene);
			}
			else {
				vzm::GetActorParams(aidGroupMks, apMkGroup);
				int numPrevMks = apMkGroup.test_params.GetParam("numMarkers", (int)0);
				if (numCurMKs != numPrevMks) {
					vzm::RemoveSceneItem(aidGroupMks);
					vzm::NewActor(apMkGroup, groupName, aidGroupMks);
					vzm::AppendSceneItemToSceneTree(aidGroupMks, sidScene);
				}
			}
			apMkGroup.test_params.SetParam("numMarkers", numCurMKs);
			vzm::SetActorParams(aidGroupMks, apMkGroup);

			for (auto& mk : mks) {

				int aidMarker = vzmutils::GetSceneItemIdByName(mk.first);
				vzm::ActorParameters apMarker;
				if (aidMarker == 0) {
					apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
					vzm::NewActor(apMarker, mk.first, aidMarker);
					vzm::AppendSceneItemToSceneTree(aidMarker, aidGroupMks);
				}
				vzm::GetActorParams(aidMarker, apMarker);
				// note: the individual markers given by GetMarkerByIdx are always tracked 

				fvec3 posWS = std::any_cast<fvec3>(mk.second);

				float targetR = __gc->g_calribmodeToggle ? 2.f * r : r;
				*(glm::fvec4*)apMarker.color = mkColor; // rgba
				//apMarker.color[3] = __gc->g_calribmodeToggle ? 0.3f : 0.3f;
				auto it = __gc->g_selectedMkNames.find(mk.first);
				if (it != __gc->g_selectedMkNames.end()) {
					*(glm::fvec4*)apMarker.color = glm::fvec4(0, 1, 0, 0.3);
					apMarker.label.textStr = "p:" + std::to_string(it->second);
					apMarker.label.fontSize = 15.f;
				}

				glm::fmat4x4 matT = glm::translate(posWS);
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(targetR)); // set 0.7 cm to the marker diameter

				matT = matT * matScale;
				apMarker.is_visible = isVisible;
				apMarker.is_pickable = isPickable;
				apMarker.SetLocalTransform(__FP matT);

				vzm::SetActorParams(aidMarker, apMarker);
			}
		};

		map<string, fvec3> optiMarkers;
		for (int i = 0; i < (int)trackInfo.NumMarkers(); i++) {
			std::map<track_info::MKINFO, std::any> mkInfo;
			trackInfo.GetMarkerByIdx(i, mkInfo);
			// "Marker[i+1]"
			bool isTracked = std::any_cast<bool>(mkInfo[track_info::MKINFO::TRACKED]);
			if (isTracked)
				optiMarkers[std::any_cast<std::string>(mkInfo[track_info::MKINFO::MK_NAME])]
				= std::any_cast<fvec3>(mkInfo[track_info::MKINFO::POSITION]);
		}
		displayMarkers(optiMarkers, "OptiMarkers", glm::fvec4(1.f, 1.f, 1.f, 0.6f), 0.007f, true);

		
		map<string, fvec3> testMarkers;
		for (int i = 0; i < (int)__gc->g_testMKs.size(); i++) {
			testMarkers["testMK" + to_string(i + 1)] = __gc->g_testMKs[i];
		}
		displayMarkers(testMarkers, "TestMarkers", glm::fvec4(0, 0, 1.f, 0.6f), 0.007f);


		map<string, fvec3> calibMarkers;
		int mkIndex = 1;
		glm::fmat4x4 matCArmRB2WS;
		trackInfo.GetRigidBodyByName("c-arm", &matCArmRB2WS, NULL, NULL, NULL);
		for (auto it : __gc->g_homographyPairs) {
			calibMarkers["calibMK" + to_string(mkIndex++)] = it.first;
		}
		displayMarkers(calibMarkers, "CalibMarkers", glm::fvec4(1.f, 1.f, 0, 0.6f), 0.003f, false, __gc->g_showCalibMarkers);
		
		/**/
	}

	// DOJO: Scene 에 Text GUI 생성
	void SetTextDisplay(const int cidCam1)
	{
		//int aidFrameText = vzmutils::GetSceneItemIdByName("Frame Text");
		//vzm::ActorParameters apFrameText;
		//vzm::GetActorParams(aidFrameText, apFrameText);
		//int oidFrameText = apFrameText.GetResourceID(vzm::ActorParameters::GEOMETRY);
		//std::vector<glm::fvec3> pinfo(3);
		//pinfo[0] = glm::fvec3(0, 0.3, 0);
		//pinfo[1] = glm::fvec3(0, 0, -1);
		//pinfo[2] = glm::fvec3(0, 1, 0);
		//vzm::GenerateTextObject((float*)&pinfo[0], "Frame : " + std::to_string(frame), 0.1, true, false, oidFrameText);

		static int frameCount = 0;
		int frame = __gc->g_optiRecordMode == OPTTRK_RECMODE::LOAD ? (int)__gc->g_optiRecordFrame : frameCount++;

		vzm::CameraParameters cpCam1;
		vzm::GetCameraParams(cidCam1, cpCam1);
		vzm::TextItem textItem;
		textItem.textStr = "Frame : " + std::to_string(frame);
		textItem.fontSize = 30.f;
		textItem.iColor = 0xFFFFFF;
		textItem.posScreenX = 0;
		textItem.posScreenY = cpCam1.h - 40;
		cpCam1.text_items.SetParam("FRAME", textItem);

		if (__gc->g_downloadCompleted == 0)
			textItem.textStr = "";
		else if (__gc->g_downloadCompleted > 100)
			textItem.textStr = fmt::format("Downloading... {} %", __gc->g_downloadCompleted - 100);
		else 
			textItem.textStr = "Download Completed";
		textItem.fontSize = 30.f;
		textItem.alpha = (float)std::min((int)__gc->g_downloadCompleted, (int)100) * 0.01f;
		textItem.iColor = 0xFFFF00;
		textItem.posScreenX = 300;
		textItem.posScreenY = cpCam1.h - 40;
		cpCam1.text_items.SetParam("DOWNLOADED", textItem);

		static float blinkAlpha = 1.f;
		static bool isDecrease = true;
		blinkAlpha = isDecrease? blinkAlpha - 0.02f : blinkAlpha + 0.02f;
		blinkAlpha = std::max(std::min(blinkAlpha, 1.f), 0.f);
		if (blinkAlpha <= 0.5f) isDecrease = false;
		else if (blinkAlpha >= 1.0f) isDecrease = true;

		// top-right
		switch (__gc->g_optiRecordMode) {
		case OPTTRK_RECMODE::RECORD: textItem.textStr = "*REC*"; break;
		case OPTTRK_RECMODE::LOAD: textItem.textStr = "*PLAY REC*"; break;
		case OPTTRK_RECMODE::NONE:
		default:
			textItem.textStr = ""; break;
		}
		textItem.fontSize = 30.f; 
		textItem.alpha = blinkAlpha;
		textItem.fontWeight = 9;
		textItem.iColor = 0xFF3030;
		textItem.alignment = "RIGHT";
		textItem.posScreenX = cpCam1.w - 500;
		textItem.posScreenY = 10;
		cpCam1.text_items.SetParam("RECORD", textItem);

		textItem.textStr = __gc->g_calribmodeToggle ? "SELECTING MARKER MODE (" + std::to_string(__gc->g_selectedMkNames.size()) + ")" : "";
		textItem.iColor = 0xFFFF30;
		textItem.posScreenY = 50;
		cpCam1.text_items.SetParam("OPERATION_MODE", textItem);

		textItem.textStr = __gc->g_optiEvent == OPTTRK_THREAD::TOOL_PIVOT ? "PIVOTING MODE " + std::to_string(__gc->g_optiPivotProgress) + "%" : "";
		cpCam1.text_items.SetParam("PIVOT_PROGRESS", textItem);

		if (__gc->g_downloadCompleted > 0 && __gc->g_downloadCompleted <= 100) __gc->g_downloadCompleted--;

		// error code display
		if (__gc->g_error_duration == 0) {
			__gc->g_error_text = "";
		}
		textItem.alignment = "CENTER";
		textItem.fontSize = 40.f;
		textItem.iColor = 0xFF5050;
		textItem.fontWeight = 7;
		textItem.alpha = __gc->g_error_duration * 0.01f;
		textItem.posScreenX = 10;
		textItem.posScreenY = cpCam1.h / 2 - 30;
		textItem.textStr = __gc->g_error_text;
		__gc->g_error_duration = std::max(__gc->g_error_duration - 1, (int)0);
		cpCam1.text_items.SetParam("ERRORCODE", textItem);

		vzm::SetCameraParams(cidCam1, cpCam1);
	}
	// DOJO : 현재 Scene (__gc->g_sceneName) 에서 등록된 camera (__gc->g_camName, __gc->g_camName2) 들에 대해 렌더링하는 함수
	// Timer 에 등록된 CALLBACK TimerProc 에서 호출됨 (note: Timer 가 본 어플리케이션에서 rendering thread 로 사용됨)
	void RenderTrackingScene(track_info* trackInfo)
	{
		if (__gc == NULL) return;

		// DOJO : Tracking 되는 기본 세트 (individual markers + rigid body frames) 를 Scene 에 반영시키는 함수
		if (trackInfo)
			UpdateTrackInfo2Scene(*(track_info*)trackInfo);

		int sidScene = vzmutils::GetSceneItemIdByName(__gc->g_sceneName);
		int cidCam1 = vzmutils::GetSceneItemIdByName(__gc->g_camName);
		int cidCam2 = vzmutils::GetSceneItemIdByName(__gc->g_camName2);

		if ((sidScene & cidCam1 & cidCam2) == 0)
			return;

		if (__gc->g_optiRecordMode == OPTTRK_RECMODE::LOAD && __gc->g_optiRecordFrame <= 2) {
			for (int i = 0; i < g_arAnimationKeyFrame.size(); i++) {
				g_arAnimationKeyFrame[i] = -1;
			}
		}

		// DOJO : Animation effect
		// cpInterCams 는 매 스캔 시 업데이트되며, 
		// slerp 로 인터폴레이션된 camera 정보를 받아 옴
		int cidCams[2] = { cidCam1 , cidCam2 };
		for (int i = 0; i < g_arAnimationKeyFrame.size(); i++) {

			int frameIdx = g_arAnimationKeyFrame[i];
			if (frameIdx >= 0) {
				vzm::CameraParameters& cpCam = g_cpInterCams[i][frameIdx];
				cpCam.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
				vzm::SetCameraParams(cidCams[i], cpCam);
				if (frameIdx == g_numAnimationCount - 1)
					frameIdx = -1;
				else
					frameIdx++;

				g_arAnimationKeyFrame[i] = frameIdx;
				//UINT flags = SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE;
				//SetWindowPos(g_hWnd, NULL, 100, 100, cpCam.w, cpCam.h, flags);
			}
		}

		SetTextDisplay(cidCam1);

		vzm::RenderScene(sidScene, cidCam1);
		vzm::RenderScene(sidScene, cidCam2);
	}
}