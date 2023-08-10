#include "RenderingTask.h"

#include "VisMtvApi.h"
#include "ApiUtility.hpp"

#include "naviHelpers.hpp"

#include <opencv2/opencv.hpp>


// DOJO : Scene �� Camera(s), Light source(s), Actor(s)�� ����, World Space �� Coordinate System �� ������ 
// 
// we set 'meter scale world to world space

// DOJO : Actor �� Scene �� �׷����� Model �̸�, 
// Actor �� �����ϴ� actor resource (called resource object) �� geometry �� material (e.g., texture) ������ ����
// vzm::GenerateXXXObject �� actor resource �� ���� 
// vzm::LoadXXX �� file �κ��� actor resource �� ����
// actor resource �� geometry �� �⺻������ Object Space (Model Space) ���� ���ǵ� ������ ��

namespace rendertask {

	__GC* __gc = NULL;

	void SetGlobalContainer(__GC* gcp) {
		__gc = gcp;
	}

	void SceneInit(const HWND view1Hwnd, const RECT rcWorldView1, const HWND view2Hwnd, const RECT rcWorldView2) {
		if (__gc == NULL) return;

		// DOJO : Axis �� ���� actor resource �� ����
		// oidAxis (objId) �� ������ actor resource �� ID �� �޾ƿ�
		// objId = 0 �� ���, �׻� ���ο� actor resource �� �����ϰ� ���ο� ID �� ����
		// objId != 0 �� ���, �ش� ID �� actor resource �� ���� ��, �ش� actor resource �� update �� (ID �� �״�� ���)
		// objId != 0 �� ���, �ش� ID �� actor resource �� ���� ��, ���ο� actor resource �� �����ϰ� ���ο� ID �� ����
		int oidAxis = 0;
		vzm::GenerateAxisHelperObject(oidAxis, 0.5f);

		// DOJO : Actor Parameter (vzm::ActorParameters) �� actor resource �� actor ����
		vzm::ActorParameters apAxis;
		// vzm::ActorParameters::SetResourceID �� �پ��� actor resource �� actor �� �Ҵ��� �� ����
		// �⺻������ vzm::ActorParameters::RES_USAGE::GEOMETRY �� �־�� ��, �� �� ���� ������ texture ���� �� �� ���� (ref. vzm::ActorParameters::RES_USAGE)
		apAxis.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidAxis);
		// line ���� ���ǵ� geometry �� ���� ���������� ���Ǵ� parameter (c.f., triangle geometry �� ��� �ش� parameter �� ���������� ������ ����)
		apAxis.line_thickness = 3;
		// color �� ���, �ش� actor �� ���� global �� ����� 
		// geometry resource �� vertex ������ color �� ���ǵǾ� �ִٸ�, ���⼱ alpha ���� ����
		*(glm::fvec4*)apAxis.color = glm::fvec4(1, 1, 1, 1);
		// DOJO : ���ο� ���� ����
		// actor name parameter �� actor parameter �ʿ�
		// actor name parameter �� actor �� ID �� ������ �� ����
		int aidAxis = 0;
		vzm::NewActor(apAxis, "World Axis", aidAxis);

		// DOJO : navihelpers �� �̹� ������Ʈ���� ����� ���� �Լ��� ��� ����
		int oidGrid, oidLines, oidTextX, oidTextZ;
		// scene �� �ٴڿ� �׷����� actor resources �� ����
		navihelpers::World_GridAxis_Gen(oidGrid, oidLines, oidTextX, oidTextZ);

		std::vector<glm::fvec3> pinfo(3);
		pinfo[0] = glm::fvec3(0, 0.5, 0);
		pinfo[1] = glm::fvec3(0, 0, -1);
		pinfo[2] = glm::fvec3(0, 1, 0);
		// DOJO : "Frame" �̶�� text �� �׸��� actor resource �� ���� (�ڼ��� ������ ���� �ٶ�)
		// ���⼭ int oidFrameText; �̷��� �ߴµ�, ���Ӱ� �����ϴ� ��� oidFrameText = 0 �� ����
		// DEBUG ��忡���� initialize �� �ϸ�, ���� 0 ���� ��� ������, 
		// RELEASE ��忡���� initialize �� �ϸ�, ������ ���� ��� ����, �� ��, (���� �߻����� ������) �ٸ� actor resource �� ID ���� ���� ������ ������ �� ����
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

		// DOJO : (world view ������) Camera �� �����ϱ� ���� �Ķ����
		vzm::CameraParameters cpCam;
		*(glm::fvec3*)cpCam.pos = glm::fvec3(-1.5, 1.5, -1.5);
		*(glm::fvec3*)cpCam.up = glm::fvec3(0, 1, 0);
		*(glm::fvec3*)cpCam.view = glm::fvec3(1, -1, 1);
		cpCam.displayActorLabel = cpCam.displayCamTextItem = true;

		// YAML �� ������ ����� (world view ������) ī�޶� ��ġ ���� �о� ����
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

		// �������� buffer ������
		cpCam.w = rcWorldView1.right - rcWorldView1.left;
		cpCam.h = rcWorldView1.bottom - rcWorldView1.top;

		// ������ ������������ View Frustum ������ near plane, far plane distances
		cpCam.np = 0.15f;
		cpCam.fp = 100.f;

		float vFov = 3.141592654f / 4.f;
		// NOTE : union ���� ����Ǿ� ����
		// ī�޶� view projection �� �ڿ������� �̵� (to C-arm View) �� ����, 
		// ī�޶� ���� convention �� camera (intrinsic) parameters �� ���
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
		cpCam.hWnd = view1Hwnd;
		// DOJO : scene �� ��ϵ� ī�޶� ����, ���⼱ �� �� ���� (AP, Lateral ��)
		int cidCam1 = 0, cidCam2 = 0;
		vzm::NewCamera(cpCam, __gc->g_camName, cidCam1);

		cpCam.h = rcWorldView2.bottom - rcWorldView1.top;
		cpCam.w = rcWorldView2.right - rcWorldView1.left;
		cpCam.hWnd = view2Hwnd;
		vzm::NewCamera(cpCam, __gc->g_camName2, cidCam2);

		// DOJO : Light �� �����ϱ� ���� �Ķ����
		vzm::LightParameters lpLight1;
		lpLight1.is_on_camera = true;
		lpLight1.is_pointlight = false;
		*(glm::fvec3*)lpLight1.pos = *(glm::fvec3*)cpCam.pos;
		*(glm::fvec3*)lpLight1.dir = *(glm::fvec3*)cpCam.view;
		*(glm::fvec3*)lpLight1.up = *(glm::fvec3*)cpCam.up;
		int lidLight1 = 0;
		// DOJO : scene �� ��ϵ� Light ����
		vzm::NewLight(lpLight1, "World Light", lidLight1);

		// DOJO : scene ����
		int sidScene = 0;
		vzm::NewScene(__gc->g_sceneName, sidScene);

		// DOJO : scene �� camera 1 (cidCam1) �� ���� 
		vzm::AppendSceneItemToSceneTree(cidCam1, sidScene);
		// DOJO : scene �� camera 2 (cidCam2) �� ���� 
		vzm::AppendSceneItemToSceneTree(cidCam2, sidScene);
		// DOJO : scene �� Light (lidLight1) �� ���� 
		vzm::AppendSceneItemToSceneTree(lidLight1, sidScene);
		// DOJO : scene �� axis actor (aidAxis) ���� 
		vzm::AppendSceneItemToSceneTree(aidAxis, sidScene);
		// DOJO : scene �� ground grid actor (aidGrid) �� ���� 
		vzm::AppendSceneItemToSceneTree(aidGrid, sidScene);
		// DOJO : scene �� ground grid ������ ���� ���� actor (aidLines) �� ���� 
		vzm::AppendSceneItemToSceneTree(aidLines, sidScene);
		// DOJO : scene �� ground grid ���� �ִ� "X" ���� actor (aidTextX) �� ���� 
		vzm::AppendSceneItemToSceneTree(aidTextX, sidScene);
		// DOJO : scene �� ground grid ���� �ִ� "Z" ���� actor (aidTextZ) �� ���� 
		vzm::AppendSceneItemToSceneTree(aidTextZ, sidScene);
		// DOJO : scene �� "Frame" ���� actor (aidTextFrame) �� ���� 
		//vzm::AppendSceneItemToSceneTree(aidTextFrame, sidScene);
		// NOTE : ���⿡���� ��� actor �� camera, light �� scene item ���� ��� scene (i.e., root) �� �ٷ� ����Ǿ�����,
		// Ư�� actor �� �ڽ����� ���� ���� �ִ� (e.g., scene<-aidAxis<-aidTextZ<-cidCam1 ,...)
	}

	// DOJO : Ư���� ����� �ƴϸ�, ī�޶� ��ȯ �� �ε巯�� ��ȭ�� ���� slerp �� ī�޶� ������ ������ ����ü
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

	// DOJO : Tracking �Ǵ� ������ Scene �� �ݿ���Ű�� �Լ�
	// Timer �� ��ϵ� CALLBACK TimerProc ���� ȣ��� (note: Timer �� �� ���ø����̼ǿ��� rendering thread �� ����)
	// track_info& trackInfo : Ʈ��ŷ Thread ���� concurrent queue (�Ǵ� async queue �󵵰� ��) �� ����� ���� �ֽ� ����
	// numRBs : rigid bodies
	// numMKs : ���� markers
	void UpdateTrackInfo2Scene(track_info& trackInfo)
	{
		if (__gc == NULL) return;

		int numMKs = trackInfo.NumMarkers();
		int numRBs = trackInfo.NumRigidBodies();

		//__gc->g_ui_banishing_count = std::max(--__gc->g_ui_banishing_count, (int)0);
		//if (__gc->g_ui_banishing_count == 1) {
		//	__gc->g_testMKs.clear();
		//	int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
		//	if (aidTestGroup != 0)
		//		vzm::RemoveSceneItem(aidTestGroup);
		//}

		// DOJO : ���� �ѹ��� ���ҽ� ������Ʈ ���� (static ���� ������ ���ҽ� ������Ʈ ID = 0 �� ��, ����) 
		// �̸� actor �� �����ϰ� scene tree �� ��ġ
		{
			using namespace std;
			using namespace navihelpers;
			using namespace glm;

			int sidScene = vzmutils::GetSceneItemIdByName(__gc->g_sceneName);

			// tracking camera (�� ����) �� �׸��� actors
			static int aidGroupCams = 0;
			if (aidGroupCams == 0) {
				vzm::ActorParameters apGroupCams;
				vzm::NewActor(apGroupCams, "Tracking CAM Group", aidGroupCams);
				vzm::AppendSceneItemToSceneTree(aidGroupCams, sidScene);

				static int oidCamModels[3] = { 0, 0, 0 };
				for (int i = 0; i < 3; i++) {
					fmat4x4 matCam2WS;
					trackInfo.GetTrackingCamPose(i, matCam2WS);

					int oidCamTris = 0, oidCamLines = 0, oidCamLabel = 0;
					Cam_Gen(matCam2WS, "CAM" + to_string(i), oidCamTris, oidCamLines, oidCamLabel);

					vzm::ActorParameters apCamTris, apCamLines, apCamLabel;
					apCamTris.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamTris);
					apCamTris.color[3] = 0.5f;
					apCamLines.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLines);
					*(fvec4*)apCamLines.phong_coeffs = fvec4(0, 1, 0, 0);
					apCamLines.line_thickness = 2;
					apCamLabel.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLabel);
					*(fvec4*)apCamLabel.phong_coeffs = fvec4(0, 1, 0, 0);
					int aidCamTris = 0, aidCamLines = 0, aidCamLabel = 0;
					vzm::NewActor(apCamTris, "Cam" + to_string(i) + " Tris", aidCamTris);
					vzm::NewActor(apCamLines, "Cam" + to_string(i) + " Lines", aidCamLines);
					vzm::NewActor(apCamLabel, "Cam" + to_string(i) + " Label", aidCamLabel);
					vzm::AppendSceneItemToSceneTree(aidCamTris, aidGroupCams);
					vzm::AppendSceneItemToSceneTree(aidCamLines, aidGroupCams);
					vzm::AppendSceneItemToSceneTree(aidCamLabel, aidGroupCams);
				}
			}
			//

			static int oidAxis = 0;
			static int oidMarker = 0;
			static int oidProbe = 0;
			//static int oidLine = 0;
			// axis �� �׸��� resource object ����
			if (vzm::GetResObjType(oidAxis) == vzm::ResObjType::UNDEFINED) {
				vzm::GenerateAxisHelperObject(oidAxis, 0.15f);
			}
			// spherical marker �� �׸��� resource object ����
			if (vzm::GetResObjType(oidMarker) == vzm::ResObjType::UNDEFINED) {
				glm::fvec4 pos(0, 0, 0, 1.f);
				vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);
			}
			// probe �� �׸��� resource object ����
			if (vzm::GetResObjType(oidProbe) == vzm::ResObjType::UNDEFINED) {
				//vzm::LoadModelFile(folder_data + "probe.obj", oidProbe);

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
			//if (oidLine == 0) {
			//	glm::fvec3 linePos[2] = { glm::fvec3(0, 0, 0), glm::fvec3(0, 0, 1) };
			//	vzm::GenerateLinesObject((float*)linePos, NULL, 1, oidLine);
			//}

			// tracking �� rigid bodies ��, probe �� �׿� �Ϲ� rigid body frame �� ��Ÿ�� scene actor �� ������ resource (oidProbe, oidAxis) �� �Ҵ�
			// ���� �� ���� �Ҵ��ϸ� scene �� tracking unit �� �̸��� �ִ����� Ȯ���ϰ� ���� ��, �Ҵ�
			// rigid body �� ��ϵ� ��Ŀ�� ���� ���
			for (int i = 0; i < numRBs; i++) {
				std::string rbName;
				float rb_error = 0;
				std::bitset<128> rb_cid = 0;
				map<string, map<track_info::MKINFO, std::any>> rbmkSet;

				static std::map<std::string, std::bitset<128>> actorTrackObjMapper;

				// ���⿡�� index �� optitrack lib ���� ����ϴ� index �� �ٸ���! track_info ����ü���� ����ϴ� index ��!
				if (trackInfo.GetRigidBodyByIdx(i, &rbName, NULL, &rb_error, &rbmkSet, &rb_cid)) {

					int aidRb = vzmutils::GetSceneItemIdByName(rbName);
					if (aidRb != 0) {
						auto it = actorTrackObjMapper.find(rbName);
						if (it == actorTrackObjMapper.end()) {
							vzm::RemoveSceneItem(aidRb);
							aidRb = 0;
						}
						else {
							if (it->second != rb_cid) {
								vzm::RemoveSceneItem(aidRb);
								aidRb = 0;
							}
						}

						actorTrackObjMapper[rbName] = rb_cid;
					}

					if (aidRb == 0) {
						vzm::ActorParameters apRb;
						if (rbName == "probe") {
							apRb.SetResourceID(vzm::ActorParameters::GEOMETRY, oidProbe);
							*(glm::fvec4*)apRb.color = glm::fvec4(1, 0.3, 0.2, 1);
						}
						else {
							apRb.SetResourceID(vzm::ActorParameters::GEOMETRY, oidAxis);
							apRb.line_thickness = 3;
						}
						apRb.is_visible = false;
						vzm::NewActor(apRb, rbName, aidRb);
						vzm::AppendSceneItemToSceneTree(aidRb, sidScene);

						if (rbName == "probe") {
						}
						else if (rbName == "tool") {
							vzm::ActorParameters apEmpty;
							int aidToolBody = 0, aidToolTip = 0;
							vzm::NewActor(apEmpty, rbName + ":Body", aidToolBody);
							apEmpty.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
							vzm::NewActor(apEmpty, rbName + ":Tip", aidToolTip);
							vzm::AppendSceneItemToSceneTree(aidToolBody, aidRb);
							vzm::AppendSceneItemToSceneTree(aidToolTip, aidRb);
						}
						else {
							std::vector<glm::fvec3> pinfo(3);
							pinfo[0] = glm::fvec3(0, 0, 0);
							pinfo[1] = glm::fvec3(-1, 0, 0);
							pinfo[2] = glm::fvec3(0, -1, 0);
							int oidLabelText = 0;
							vzm::GenerateTextObject((float*)&pinfo[0], rbName, 0.035, true, false, oidLabelText);
							vzm::ActorParameters apLabelText;
							apLabelText.SetResourceID(vzm::ActorParameters::GEOMETRY, oidLabelText);
							*(glm::fvec4*)apLabelText.phong_coeffs = glm::fvec4(0, 1, 0, 0);
							int aidLabelText = 0;
							vzm::NewActor(apLabelText, rbName + ":Label", aidLabelText);
							vzm::AppendSceneItemToSceneTree(aidLabelText, aidRb);
						}

						// rigid body �� markers ���� ���� �Ҵ�
						// ������ �޶��� �� �ִ�...  ���1. rbuid (��Ŀ/RB �� ���� �۾� Ȯ��) Ȯ��, ���2. mk ���� üũ 
						for (auto& rbmk : rbmkSet) {
							map<track_info::MKINFO, std::any>& rbmk_v = rbmk.second;

							string rbmk_name = std::any_cast<std::string>(rbmk_v[track_info::MKINFO::MK_NAME]);
							int aidMarker = vzmutils::GetSceneItemIdByName(rbmk_name);
							if (aidMarker == 0) {
								vzm::ActorParameters apMarker;
								apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
								apMarker.is_visible = false;
								apMarker.is_pickable = false;
								*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 0.f, 0.f, 0.3f); // rgba
								vzm::NewActor(apMarker, rbmk_name, aidMarker);
								vzm::AppendSceneItemToSceneTree(aidMarker, aidRb);
							}
						}
					}
				}
			}

			// tracking �� individual markers �� ��Ÿ�� spherical shape �� scene actor �� ������ resource (oidMarker) �� �Ҵ�
			// ���� �� ���� �Ҵ��ϸ� scene �� tracking unit �� �̸��� �ִ����� Ȯ���ϰ� ���� ��, �Ҵ�
			for (int i = 0; i < numMKs; i++) {
				std::map<track_info::MKINFO, std::any> mk;
				if (trackInfo.GetMarkerByIdx(i, mk)) {
					std::string mkName = std::any_cast<std::string>(mk[track_info::MKINFO::MK_NAME]);
					int aidMarker = vzmutils::GetSceneItemIdByName(mkName);
					if (aidMarker == 0) {
						vzm::ActorParameters apMarker;
						apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
						apMarker.is_visible = false;
						*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 1.f, 1.f, 1.f); // rgba
						vzm::NewActor(apMarker, mkName, aidMarker);
						vzm::AppendSceneItemToSceneTree(aidMarker, sidScene);
					}
				}
			}

			int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
			if (aidTestGroup == 0) {
				vzm::ActorParameters apGroup;
				vzm::NewActor(apGroup, "testMK Group", aidTestGroup);
				vzm::AppendSceneItemToSceneTree(aidTestGroup, sidScene);
			}
			for (int i = 0; i < (int)__gc->g_testMKs.size(); i++) {
				std::string testMkName = "testMk-" + std::to_string(i);
				int aidMarkerTest = vzmutils::GetSceneItemIdByName(testMkName);
				if (aidMarkerTest == 0) {
					vzm::ActorParameters apMarker;
					apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
					apMarker.is_visible = false;
					*(glm::fvec4*)apMarker.color = glm::fvec4(0, 0, 1.f, 1.f); // rgba
					vzm::NewActor(apMarker, testMkName, aidMarkerTest);
					vzm::AppendSceneItemToSceneTree(aidMarkerTest, aidTestGroup);
				}
			}
		}

		// DOJO : scene tree �� ��ġ�� (rigid body) actor ���� ��ġ�� tracking ������ �������� ��ȯ �̵� 
		for (int i = 0; i < numRBs; i++) {
			std::string rbName;
			glm::fmat4x4 matLS2WS;
			std::map<std::string, std::map<track_info::MKINFO, std::any>> rbmkSet;
			if (trackInfo.GetRigidBodyByIdx(i, &rbName, &matLS2WS, NULL, &rbmkSet, NULL)) {

				int aidRb = vzmutils::GetSceneItemIdByName(rbName);
				vzm::ActorParameters apRb;
				vzm::GetActorParams(aidRb, apRb);
				apRb.is_visible = true;

				apRb.SetLocalTransform(__FP matLS2WS);
				vzm::SetActorParams(aidRb, apRb);

				glm::fmat4x4 matWS2LS = glm::inverse(matLS2WS);

				for (auto& rbmk : rbmkSet) {
					std::map<track_info::MKINFO, std::any>& rbmk_v = rbmk.second;

					std::string rbmk_name = std::any_cast<std::string>(rbmk_v[track_info::MKINFO::MK_NAME]);
					glm::fvec3 pos = std::any_cast<glm::fvec3>(rbmk_v[track_info::MKINFO::POSITION]);
					glm::fvec3 posRb = vzmutils::transformPos(pos, matWS2LS);

					glm::fmat4x4 matMkLS2WS = glm::translate(posRb);
					glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.007f)); // set 1 cm to the marker diameter
					matMkLS2WS = matMkLS2WS * matScale;
					int aidMarker = vzmutils::GetSceneItemIdByName(rbmk_name);
					vzm::ActorParameters apMarker;
					vzm::GetActorParams(aidMarker, apMarker);
					apMarker.is_visible = true;
					apMarker.is_pickable = false;
					apMarker.SetLocalTransform(__FP matMkLS2WS);
					vzm::SetActorParams(aidMarker, apMarker);
				}

				if (rbName == "tool") {
					using namespace glm;

					const float tipMarkerRadius = (float)(0.019 * 0.5);
					int numToolPoints = rbmkSet.size();

					std::vector<fvec3> toolLocalPts;
					for (auto& rbmk : rbmkSet) {
						std::map<track_info::MKINFO, std::any>& rbmk_v = rbmk.second;
						glm::fvec3 pos = std::any_cast<glm::fvec3>(rbmk_v[track_info::MKINFO::POSITION]);
						glm::fvec3 posRb = vzmutils::transformPos(pos, matWS2LS);
						toolLocalPts.push_back(posRb);
					}

					fvec3 posCenterLS = fvec3(0, 0, 0);
					fvec3 posTipMkLS = __gc->g_toolTipPointLS;
					for (int i = 0; i < numToolPoints - 1; i++) {
						posCenterLS += toolLocalPts[i];
					}
					posCenterLS /= (float)(numToolPoints - 1);

					// compute normal
					fvec3 nrlLS = navihelpers::ComputeNormalVector(toolLocalPts); // LS

					// correct the normal direction using cam view
					int cidCam1 = vzmutils::GetSceneItemIdByName(__gc->g_camName); // Cam1
					vzm::CameraParameters cam1Param;
					vzm::GetCameraParams(cidCam1, cam1Param);
					fvec3 camViewLS = vzmutils::transformVec(*(fvec3*)cam1Param.view, matWS2LS);
					if (dot(nrlLS, camViewLS) > 0) nrlLS *= -1.f;

					// compute line point of the tool
					fvec3 posLineLS = posCenterLS - nrlLS * 0.07f;

					// compute the direction of a tip (using inverse of the cam up)
					fvec3 camUpLS = vzmutils::transformVec(*(fvec3*)cam1Param.up, matWS2LS);
					fvec3 dirToolLS = normalize(posTipMkLS - posLineLS);

					// compute the line geometry in the tool's rigid body
					fvec3 posTipLS = posTipMkLS - dirToolLS * tipMarkerRadius;
					glm::fvec3 linePos[2] = { posLineLS, posTipLS };
					static int oidToolLine = 0;
					if (vzm::GetResObjType(oidToolLine) == vzm::ResObjType::UNDEFINED)
						vzm::GenerateLinesObject((float*)linePos, NULL, 1, oidToolLine);


					//{
					//	// test //
					//__gc->g_testMKs[0] = vzmutils::transformPos(posCenterLS, matLS2WS);
					//__gc->g_testMKs[1] = vzmutils::transformPos(posLineLS, matLS2WS);
					//__gc->g_testMKs[2] = vzmutils::transformPos(posTipLS, matLS2WS);
					//}


					int aidToolBody = vzmutils::GetSceneItemIdByName(rbName + ":Body");
					int aidToolTip = vzmutils::GetSceneItemIdByName(rbName + ":Tip");
					assert(aidToolBody != 0 && aidToolTip != 0);

					vzm::ActorParameters apToolBody, apToolTip;
					vzm::GetActorParams(aidToolBody, apToolBody);
					apToolBody.SetResourceID(vzm::ActorParameters::GEOMETRY, oidToolLine);
					apToolBody.line_thickness = 3;
					*(fvec4*)apToolBody.color = fvec4(1, 1, 1, 1);
					vzm::SetActorParams(aidToolBody, apToolBody);

					// local transform to the tip sphere
					vzm::GetActorParams(aidToolTip, apToolTip);
					*(fvec4*)apToolTip.color = fvec4(1, 1, 0, 1);
					glm::fmat4x4 matLS = glm::translate(posTipLS);
					glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.002f)); // set 1 cm to the marker diameter
					matLS = matLS * matScale;
					apToolTip.SetLocalTransform(__FP matLS);
					vzm::SetActorParams(aidToolTip, apToolTip);
				}
			}
		}

		// DOJO : scene tree �� ��ġ�� (marker) actor ���� ��ġ�� tracking ������ �������� ��ȯ �̵�
		for (int i = 0; i < numMKs; i++) {
			std::map<track_info::MKINFO, std::any> mk;
			if (trackInfo.GetMarkerByIdx(i, mk)) {
				std::string mkName = std::any_cast<std::string>(mk[track_info::MKINFO::MK_NAME]);
				glm::fvec3 pos = std::any_cast<glm::fvec3>(mk[track_info::MKINFO::POSITION]);

				glm::fmat4x4 matLS2WS = glm::translate(pos);
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(__gc->g_calribmodeToggle ? 0.01f : 0.005f)); // set 1 cm to the marker diameter
				matLS2WS = matLS2WS * matScale;
				int aidMarker = vzmutils::GetSceneItemIdByName(mkName);
				vzm::ActorParameters apMarker;
				vzm::GetActorParams(aidMarker, apMarker);
				apMarker.is_visible = true;
				apMarker.is_pickable = true;

				*(glm::fvec4*)apMarker.color = glm::fvec4(1, 1, 1, 1);
				if (__gc->g_selectedMkNames.find(mkName) != __gc->g_selectedMkNames.end()) {
					*(glm::fvec4*)apMarker.color = glm::fvec4(0, 1, 0, 0.5);
				}

				apMarker.SetLocalTransform(__FP matLS2WS);
				vzm::SetActorParams(aidMarker, apMarker);
			}
		}

		// test display
		{
			// for testing
			int test_count = 1;
			for (int i = 0; i < (int)__gc->g_testMKs.size(); i++) {
				std::string testMkName = "testMk-" + std::to_string(i);
				int aidMarkerTest = vzmutils::GetSceneItemIdByName(testMkName);
				if (aidMarkerTest != 0) {
					glm::fmat4x4 matLS2WS = glm::translate(__gc->g_testMKs[i]);
					glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.001f)); // set 1 cm to the marker diameter
					matLS2WS = matLS2WS * matScale;

					vzm::ActorParameters apMarker;
					vzm::GetActorParams(aidMarkerTest, apMarker);
					apMarker.is_visible = true;

					if (test_count == 7) test_count++;
					else if (test_count == 57) test_count++;

					if (__gc->g_testMKs.size() > 50)
						*(glm::fvec4*)apMarker.color = glm::fvec4((test_count % 8) / 8.f, ((test_count / 8.f) + 1.f) / 8.f, 0, 1.f); // rgba
					else
						*(glm::fvec4*)apMarker.color = glm::fvec4(1, 0, 1, 1);

					test_count++;

					apMarker.SetLocalTransform(__FP matLS2WS);
					vzm::SetActorParams(aidMarkerTest, apMarker);
				}
			}
		}
	}

	// DOJO : ���� Scene (__gc->g_sceneName) ���� ��ϵ� camera (__gc->g_camName, __gc->g_camName2) �鿡 ���� �������ϴ� �Լ�
	// Timer �� ��ϵ� CALLBACK TimerProc ���� ȣ��� (note: Timer �� �� ���ø����̼ǿ��� rendering thread �� ����)
	void RenderTrackingScene(const track_info* trackInfo)
	{
		if (__gc == NULL) return;

		// DOJO : Tracking �Ǵ� �⺻ ��Ʈ (individual markers + rigid body frames) �� Scene �� �ݿ���Ű�� �Լ�
		if (trackInfo)
			UpdateTrackInfo2Scene(*(track_info*)trackInfo);

		static int frameCount = 0;

		int frame = frameCount++;

		int sidScene = vzmutils::GetSceneItemIdByName(__gc->g_sceneName);
		int cidCam1 = vzmutils::GetSceneItemIdByName(__gc->g_camName);
		int cidCam2 = vzmutils::GetSceneItemIdByName(__gc->g_camName2);

		if ((sidScene & cidCam1 & cidCam2) == 0)
			return;

		// DOJO: Scene �� "Frame : ����" Text �߰� �ϴ� (Actor�� ����) ���ҽ�(������Ʈ) ���� 
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

			vzm::CameraParameters cpCam1;
			vzm::GetCameraParams(cidCam1, cpCam1);
			vzm::TextItem textItem;
			textItem.textStr = "Frame : " + std::to_string(frame);
			textItem.fontSize = 30.f;
			textItem.iColor = 0xFFFFFF;
			textItem.posScreenX = 0;
			textItem.posScreenY = cpCam1.h - 40;
			cpCam1.text_items.SetParam("FRAME", textItem);

			textItem.textStr = __gc->g_useGlobalPairs? "Global Pairs (" + std::to_string(__gc->g_homographyPairs.size()) + ")" : "Local Pairs";
			textItem.fontSize = 30.f;
			textItem.iColor = 0xFFFFFF;
			textItem.posScreenX = 0;
			textItem.posScreenY = cpCam1.h - 80;
			cpCam1.text_items.SetParam("PAIRMODE", textItem);

			textItem.textStr = __gc->g_downloadCompleted > 0 ? "Download Completed" : "";
			textItem.fontSize = 20.f + __gc->g_downloadCompleted / 10.f;
			textItem.iColor = 0xFF00FF;
			textItem.posScreenX = 300;
			textItem.posScreenY = cpCam1.h - 40;
			cpCam1.text_items.SetParam("DOWNLOADED", textItem);

			if (__gc->g_downloadCompleted > 0) __gc->g_downloadCompleted--;

			vzm::SetCameraParams(cidCam1, cpCam1);
		}

		// generate scene actors with updated trackInfo
		{
			// DOJO : Animation effect
			// cpInterCams �� �� ��ĵ �� ������Ʈ�Ǹ�, 
			// slerp �� ���������̼ǵ� camera ������ �޾� ��
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

			vzm::RenderScene(sidScene, cidCam1);
			vzm::RenderScene(sidScene, cidCam2);
		}
	}
}