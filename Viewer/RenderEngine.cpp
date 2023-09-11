#include "RenderEngine.h"

RenderEngine::RenderEngine() : g_numAnimationCount(50)
{
	g_arAnimationKeyFrame = { -1, -1 };
	g_cpInterCams[0] = std::vector<vzm::CameraParameters>(g_numAnimationCount);
	g_cpInterCams[1] = std::vector<vzm::CameraParameters>(g_numAnimationCount);
}

RenderEngine::~RenderEngine()
{
}

void RenderEngine::InitializeTask(__GC* gcp) {
	__gc = gcp;
}

void RenderEngine::SceneInit() {
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
	cpCam.w = SCANIMG_W;
	cpCam.h = SCANIMG_H;

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
	//cpCam.hWnd = view1Hwnd;
	// DOJO : scene �� ��ϵ� ī�޶� ����, ���⼱ �� �� ���� (AP, Lateral ��)
	int cidCam1 = 0, cidCam2 = 0;
	vzm::NewCamera(cpCam, __gc->g_camName, cidCam1);

	cpCam.h = SCANIMG_W;
	cpCam.w = SCANIMG_H;
	//cpCam.hWnd = view2Hwnd;
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


void RenderEngine::SetAnimationCamTo(const int cidCam, const int viewIdx,
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
void RenderEngine::UpdateTrackInfo2Scene(track_info& trackInfo)
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
		static float probeCorrection = 0;
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
		if (vzm::GetResObjType(oidProbe) == vzm::ResObjType::UNDEFINED ||
			probeCorrection != __gc->g_probeTipCorrection) {
			probeCorrection = __gc->g_probeTipCorrection;
			//vzm::LoadModelFile(folder_data + "probe.obj", oidProbe);

			glm::fvec3 posS(0, 0, 0.25f);
			glm::fvec3 posE(0, 0, -probeCorrection);
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

		int aidCalibMkGroup = vzmutils::GetSceneItemIdByName("Calib MK Group");
		if (aidCalibMkGroup == 0) {
			vzm::ActorParameters apGroup;
			vzm::NewActor(apGroup, "Calib MK Group", aidTestGroup);
			vzm::AppendSceneItemToSceneTree(aidTestGroup, sidScene);
		}
		for (int i = 0; i < (int)__gc->g_homographyPairs.size(); i++) {
			std::string calibMkName = "calibMk-" + std::to_string(i);
			int aidMarkerCalib = vzmutils::GetSceneItemIdByName(calibMkName);
			if (aidMarkerCalib == 0) {
				vzm::ActorParameters apMarker;
				apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
				apMarker.is_visible = false;
				*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 0, 1.f, 1.f); // rgba
				vzm::NewActor(apMarker, calibMkName, aidMarkerCalib);
				vzm::AppendSceneItemToSceneTree(aidMarkerCalib, aidTestGroup);

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
			apMarker.label.textStr = "";

			*(glm::fvec4*)apMarker.color = glm::fvec4(1, 1, 1, __gc->g_calribmodeToggle ? 0.5f : 1);
			auto it = __gc->g_selectedMkNames.find(mkName);
			if (it != __gc->g_selectedMkNames.end()) {
				*(glm::fvec4*)apMarker.color = glm::fvec4(0, 1, 0, 0.5);

				apMarker.label.textStr = "p:" + std::to_string(it->second);
				apMarker.label.fontSize = 15.f;
			}

			apMarker.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidMarker, apMarker);
		}
	}

	// test display
	{
		// for testing
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
				apMarker.label.textStr = __gc->g_calribmodeToggle ? "c:" + std::to_string(i) : "";
				apMarker.label.iColor = 0xFF00FF;
				apMarker.label.fontSize = 15.f;
				apMarker.label.posScreenX = 10;
				*(glm::fvec4*)apMarker.color = glm::fvec4(1, 0, 1, 0.5);

				apMarker.SetLocalTransform(__FP matLS2WS);
				vzm::SetActorParams(aidMarkerTest, apMarker);
			}
		}
	}


	//int aidCalibMkGroup = vzmutils::GetSceneItemIdByName("Calib MK Group");
	//if (aidCalibMkGroup == 0) {
	//	vzm::ActorParameters apGroup;
	//	vzm::NewActor(apGroup, "Calib MK Group", aidTestGroup);
	//	vzm::AppendSceneItemToSceneTree(aidTestGroup, sidScene);
	//}
	//for (int i = 0; i < (int)__gc->g_homographyPairs.size(); i++) {
	//	std::string calibMkName = "calibMk-" + std::to_string(i);
	//	int aidMarkerCalib = vzmutils::GetSceneItemIdByName(calibMkName);
	//	if (aidMarkerCalib == 0) {
	//		vzm::ActorParameters apMarker;
	//		apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
	//		apMarker.is_visible = false;
	//		*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 0, 1.f, 1.f); // rgba
	//		vzm::NewActor(apMarker, calibMkName, aidMarkerCalib);
	//		vzm::AppendSceneItemToSceneTree(aidMarkerCalib, aidTestGroup);
	//
	//	}
	//}

	{
		glm::fmat4x4 matCArmRB2WS;
		trackInfo.GetRigidBodyByName("c-arm", &matCArmRB2WS, NULL, NULL, NULL);

		int mkIndex = 0;
		for (auto it = __gc->g_homographyPairs.begin(); it != __gc->g_homographyPairs.end(); it++)
		{
			glm::fvec3 posCalibMKRB = it->first;
			std::string testMkName = "calibMk-" + std::to_string(mkIndex++);
			int aidMarkerCalib = vzmutils::GetSceneItemIdByName(testMkName);
			if (aidMarkerCalib != 0) {
				glm::fmat4x4 matLS2WS = matCArmRB2WS * glm::translate(posCalibMKRB);
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.001f)); // set 1 cm to the marker diameter
				matLS2WS = matLS2WS * matScale;

				vzm::ActorParameters apMarker;
				vzm::GetActorParams(aidMarkerCalib, apMarker);
				apMarker.is_visible = __gc->g_showCalibMarkers;
				//apMarker.label.textStr = "(" + std::to_string(it->second.x) + ", " + std::to_string(it->second.y) + ")";
				apMarker.SetLocalTransform(__FP matLS2WS);
				vzm::SetActorParams(aidMarkerCalib, apMarker);
			}
		}
	}
}

// DOJO : ���� Scene (__gc->g_sceneName) ���� ��ϵ� camera (__gc->g_camName, __gc->g_camName2) �鿡 ���� �������ϴ� �Լ�
// Timer �� ��ϵ� CALLBACK TimerProc ���� ȣ��� (note: Timer �� �� ���ø����̼ǿ��� rendering thread �� ����)
void RenderEngine::RenderTrackingScene(const track_info* trackInfo)
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

		textItem.textStr = __gc->g_useGlobalPairs ? "Global Pairs (" + std::to_string(__gc->g_homographyPairs.size()) + ")" : "Local Pairs";
		textItem.fontSize = 30.f;
		textItem.iColor = 0xFFFFFF;
		textItem.posScreenX = 0;
		textItem.posScreenY = cpCam1.h - 80;
		cpCam1.text_items.SetParam("PAIRMODE", textItem);

		textItem.textStr = "Probe Tip Correction : " + std::to_string(__gc->g_probeTipCorrection * 0.001f) + " mm";
		textItem.fontSize = 20.f;
		textItem.iColor = 0xFFFFFF;
		textItem.posScreenX = 0;
		textItem.posScreenY = cpCam1.h - 120;
		cpCam1.text_items.SetParam("PROBETIP_CORRECTION", textItem);

		textItem.textStr = __gc->g_downloadCompleted > 0 ? "Download Completed" : "";
		textItem.fontSize = 30.f;
		textItem.alpha = __gc->g_downloadCompleted * 0.01f;
		textItem.iColor = 0xFF00FF;
		textItem.posScreenX = 300;
		textItem.posScreenY = cpCam1.h - 40;
		cpCam1.text_items.SetParam("DOWNLOADED", textItem);

		if (__gc->g_downloadCompleted > 0) __gc->g_downloadCompleted--;

		// error code display
		if (__gc->g_error_duration == 0) {
			__gc->g_error_code = ERROR_CODE_NONE;
		}
		textItem.fontSize = 30.f;
		textItem.iColor = 0xFF0000;
		textItem.alpha = __gc->g_error_duration * 0.01f;
		textItem.posScreenX = 300;
		textItem.posScreenY = cpCam1.h - 80;
		switch (__gc->g_error_code)
		{
		case ERROR_CODE_NOT_ENOUGH_SELECTION:
			textItem.textStr = "Not Enough Selection!";
			break;
		case ERROR_CODE_CARM_TRACKING_FAILURE:
			textItem.textStr = "C-Arm RB Tracking Failure!";
			break;
		case ERROR_CODE_INVALID_CALIB_PATTERN_DETECTED:
			textItem.textStr = "Failure to Detect Calibration Patterns!";
			break;
		case ERROR_CODE_NONE:
		default:
			textItem.textStr = "";
			break;
		}
		__gc->g_error_duration = std::max(__gc->g_error_duration - 1, (int)0);
		cpCam1.text_items.SetParam("ERRORCODE", textItem);

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

void RenderEngine::slotEventHandler(QEvent* pEvent)
{
	int sidScene = vzmutils::GetSceneItemIdByName(__gc->g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(__gc->g_camName);
	float scene_stage_scale = 5.f;

	glm::fvec3 scene_stage_center = glm::fvec3();
	vzm::CameraParameters cpCam1;
	if (sidScene != 0 && cidCam1 != 0) {
		vzm::GetCameraParams(cidCam1, cpCam1);

		scene_stage_scale = glm::length(scene_stage_center - *(glm::fvec3*)cpCam1.pos) * 1.0f;
	}
	static vzmutils::GeneralMove general_move;

	switch (pEvent->type())
	{
	case QEvent::MouseButtonPress:
	{
		QMouseEvent* mouse = static_cast<QMouseEvent*>(pEvent);

		
		if (__gc->g_optiEvent == OPTTRK_THREAD_TOOL_REGISTER) break;
		int x = mouse->pos().x();
		int y = mouse->pos().y();


		if (x == 0 && y == 0) break;

		// renderer does not need to be called
		glm::ivec2 pos_ss = glm::ivec2(x, y);

		if (__gc->g_calribmodeToggle) {
			if (mouse->buttons() == Qt::LeftButton) {
				int aidPicked = 0;
				glm::fvec3 posPick;
				if (vzm::PickActor(aidPicked, __FP posPick, x, y, cidCam1)) {
					std::string actorName;
					vzm::GetSceneItemName(aidPicked, actorName);
					if (__gc->g_selectedMkNames.find(actorName) == __gc->g_selectedMkNames.end()) {
						int idx = __gc->g_selectedMkNames.size();
						__gc->g_selectedMkNames[actorName] = idx;
						std::cout << "\npicking count : " << __gc->g_selectedMkNames.size() << std::endl;
					}
				}
			}
			else {
				__gc->g_selectedMkNames.clear();
				general_move.Start((int*)&pos_ss, cpCam1, scene_stage_center, scene_stage_scale);
			}
		}
		else {
			general_move.Start((int*)&pos_ss, cpCam1, scene_stage_center, scene_stage_scale);
		}
		break;
	}

	case QEvent::MouseMove:
	{
		QMouseEvent* mouse = static_cast<QMouseEvent*>(pEvent);

		if (__gc->g_calribmodeToggle && !(mouse->buttons() == Qt::RightButton)) break;

		int x = mouse->pos().x();
		int y = mouse->pos().y();

		if ((mouse->buttons() == Qt::RightButton) || (mouse->buttons() == Qt::LeftButton))
		{
			glm::ivec2 pos_ss = glm::ivec2(x, y);
			if (mouse->buttons() == Qt::LeftButton)
				general_move.PanMove((int*)&pos_ss, cpCam1);
			else if (mouse->buttons() == Qt::RightButton)
				general_move.RotateMove((int*)&pos_ss, cpCam1);

			vzm::SetCameraParams(cidCam1, cpCam1);
			//Render();
			//vzm::RenderScene(sidScene, cidCam1);
			//UpdateWindow(hWnd);
			//InvalidateRect(hWnd, NULL, FALSE);
		}
		else {
			int aidPickedMK = 0;
			glm::fvec3 pos_picked;
			if (vzm::PickActor(aidPickedMK, __FP pos_picked, x, y, cidCam1))
			{
				std::string mkName;
				vzm::GetNameBySceneItemID(aidPickedMK, mkName);
				vzm::ActorParameters apMK;
				vzm::GetActorParams(aidPickedMK, apMK);
				glm::fmat4x4 matWorld = *(glm::fmat4x4*)apMK.GetWorldTransform();
				glm::fvec3 posOrigin(0, 0, 0);
				glm::fvec3 posMK = vzmutils::transformPos(posOrigin, matWorld);
				//std::cout << "\n" <<mkName << "is picked : " << ", Pos : " << posMK.x << ", " << posMK.y << ", " << posMK.z << std::endl;

			}

		}
		break;
	}
	case QEvent::Wheel:
	{
		QWheelEvent* mouse = static_cast<QWheelEvent*>(pEvent);
		int zDelta = mouse->angleDelta().y();

		// by adjusting cam distance
		if (zDelta > 0)
			*(glm::fvec3*)cpCam1.pos += scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam1.view);
		else
			*(glm::fvec3*)cpCam1.pos -= scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam1.view);

		vzm::SetCameraParams(cidCam1, cpCam1);
		break;
	}

	case QEvent::KeyPress:
	{
		QKeyEvent* key = static_cast<QKeyEvent*>(pEvent);

		switch (key->key())
		{
		case Qt::Key_S:
		{
			if (__gc->g_calribmodeToggle) break;

			cv::FileStorage fs(__gc->g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::WRITE);
			cv::Mat ocvVec3(1, 3, CV_32FC1);
			memcpy(ocvVec3.ptr(), cpCam1.pos, sizeof(float) * 3);
			fs.write("POS", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam1.view, sizeof(float) * 3);
			fs.write("VIEW", ocvVec3);
			memcpy(ocvVec3.ptr(), cpCam1.up, sizeof(float) * 3);
			fs.write("UP", ocvVec3);
			fs.release();;
			break;
		}
		case Qt::Key_L:
		{
			if (__gc->g_calribmodeToggle) break;

			cv::FileStorage fs(__gc->g_folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
			if (fs.isOpened()) {
				cv::Mat ocvVec3;
				fs["POS"] >> ocvVec3;
				memcpy(cpCam1.pos, ocvVec3.ptr(), sizeof(float) * 3);
				fs["VIEW"] >> ocvVec3;
				memcpy(cpCam1.view, ocvVec3.ptr(), sizeof(float) * 3);
				fs["UP"] >> ocvVec3;
				memcpy(cpCam1.up, ocvVec3.ptr(), sizeof(float) * 3);
				fs.release();
			}
			vzm::SetCameraParams(cidCam1, cpCam1);
			break;
		}
		case Qt::Key_R:
		{
			break;
		}
		case Qt::Key_C:
		{
			__gc->g_calribmodeToggle = !__gc->g_calribmodeToggle;

			vzm::TextItem textItem;
			textItem.textStr = __gc->g_calribmodeToggle ? "SELECTING MARKER MODE" : "";
			textItem.fontSize = 30.f;
			textItem.iColor = 0xFFFF00;
			textItem.posScreenX = 0;
			textItem.posScreenY = 0;

			if (!__gc->g_calribmodeToggle) {
				__gc->g_testMKs.clear();
				__gc->g_selectedMkNames.clear();
				int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
				if (aidTestGroup != 0)
					vzm::RemoveSceneItem(aidTestGroup);
			}

			std::cout << "\n" << "calibration mode " << (__gc->g_calribmodeToggle ? "ON" : "OFF") << std::endl;

			using namespace glm;
			fmat4x4 matCam2WS;
			track_info trk;
			trk = __gc->g_track_que.front();
			trk.GetTrackingCamPose(0, matCam2WS);

			*(fvec3*)cpCam1.pos = vzmutils::transformPos(fvec3(0, 0, 0), matCam2WS);
			*(fvec3*)cpCam1.view = vzmutils::transformVec(fvec3(0, 0, -1), matCam2WS);
			*(fvec3*)cpCam1.up = vzmutils::transformVec(fvec3(0, 1, 0), matCam2WS);
			cpCam1.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_FOV;
			cpCam1.fov_y = glm::pi<float>() / 2.f;
			cpCam1.aspect_ratio = cpCam1.ip_w / cpCam1.ip_h;
			cpCam1.np = 0.15f;

			cpCam1.text_items.SetParam("OPERATION_MODE", textItem);

			vzm::SetCameraParams(cidCam1, cpCam1);
			break;
		}
		case Qt::Key_U:
		{
			if (!__gc->g_calribmodeToggle)
				return;
			// to do
			// update "c-arm" RB
			int numSelectedMKs = (int)__gc->g_selectedMkNames.size();
			if (numSelectedMKs < 3) {
				std::cout << "\n" << "at least 3 points are needed to register a rigid body!! current # of points : " << numSelectedMKs << std::endl;
				return;
			}

			std::cout << "\n" << "rigidbody for c-arm is registered with " << numSelectedMKs << " points" << std::endl;

			__gc->g_optiEvent = OPTTRK_THREAD_C_ARM_REGISTER;

			while (__gc->g_optiEvent == OPTTRK_THREAD_C_ARM_REGISTER) { Sleep(2); }

			// int aidRbCarm = vzmutils::GetSceneItemIdByName("c-arm");
			// vzm::RemoveSceneItem(aidRbCarm);
			// �� ������, "c-arm" ���� scene item ��� �����, �ش� element �����..
			// UpdateTrackInfo2Scene ���� rigid boby name �� cid �� Ȯ���Ͽ�, �ٸ� ��� (���� ����, rbmk ���� ����, ���) ���� �� �����ϰ� actor group �����
			break;
		}
		case Qt::Key_D:
		{
			for (int i = 0; i < 9; i++) {
				auto it = __gc->g_mapAidGroupCArmCam.find(i + 1);
				if (it != __gc->g_mapAidGroupCArmCam.end())
					vzm::RemoveSceneItem(it->second, true);
			}
			__gc->g_mapAidGroupCArmCam.clear();

			__gc->g_testMKs.clear();
			__gc->g_selectedMkNames.clear();
			int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
			if (aidTestGroup != 0)
				vzm::RemoveSceneItem(aidTestGroup);
			break;
		}
		case Qt::Key_Y:
		{
			track_info trk;
			trk = __gc->g_track_que.front();
			glm::fmat4x4 matRB2WS;
			if (trk.GetRigidBodyByName("probe", &matRB2WS, NULL, NULL, NULL)) {
				glm::fvec3 p = vzmutils::transformPos(glm::fvec3(0, 0, 0), matRB2WS);

				//if (trk.GetRigidBodyByName("c-arm", &matRB2WS, NULL, NULL, NULL)) 
				{

					//glm::fmat4x4 matWS2RB = glm::inverse(matRB2WS);
					//p = vzmutils::transformPos(p, matWS2RB);
					cv::Mat ocvVec3(1, 3, CV_32FC1);
					memcpy(ocvVec3.ptr(), &p, sizeof(float) * 3);
					cv::FileStorage fs(__gc->g_folder_trackingInfo + "test_tip.txt", cv::FileStorage::Mode::WRITE);
					fs << "probe_tip" << ocvVec3;
					fs.release();

					auto ___LoadMyVariable = [=](const std::string& pname, const int idx) {
						cv::Mat ocvVec3(1, 3, CV_32FC1);
						cv::FileStorage fs(__gc->g_folder_trackingInfo + "my_test_variables.txt", cv::FileStorage::Mode::READ);
						fs[pname] >> ocvVec3;
						glm::fvec3 p1;
						memcpy(&p1, ocvVec3.ptr(), sizeof(float) * 3);
						fs.release();

						return ((float*)&p1)[idx];
					};

					int idx = (int)___LoadMyVariable("test0", 0);

					cv::FileStorage __fs(__gc->g_folder_trackingInfo + "test_tip_sphere.txt", cv::FileStorage::Mode::READ);
					__fs["inter_sphere_" + std::to_string(idx)] >> ocvVec3;
					glm::fvec3 p1;
					memcpy(&p1, ocvVec3.ptr(), sizeof(float) * 3);
					__fs.release();

					float r = glm::length(p1 - p);
					std::cout << "\ntest : " << r << std::endl;
				}

			}
			break;
		}
		case Qt::Key_T:
		{
			if (!__gc->g_calribmodeToggle) {
				std::cout << "\n" << "not calibration mode!!" << std::endl;
				return;
			}
			if (__gc->g_selectedMkNames.size() < 4) {
				std::cout << "\n" << "at least 4 points (last one is for tool tip) are needed!!" << std::endl;
				return;
			}

			// �� ����ϱ�..
			__gc->g_optiEvent = OPTTRK_THREAD_TOOL_REGISTER;
			while (__gc->g_optiEvent != OPTTRK_THREAD_FREE) { Sleep(2); }
			__gc->g_testMKs.clear();
			__gc->g_testMKs.push_back(glm::fvec3(0));
			__gc->g_testMKs.push_back(glm::fvec3(0));
			__gc->g_testMKs.push_back(glm::fvec3(0));
			break;
		}
		case Qt::Key_G:
		{
			__gc->g_useGlobalPairs != !__gc->g_calribmodeToggle;
			break;
		}
		case Qt::Key_BraceLeft:
		{
			__gc->g_probeTipCorrection -= 0.0002;
			break;
		}
		case Qt::Key_BraceRight:
		{
			__gc->g_probeTipCorrection += 0.0002;
			break;
		}
		case Qt::Key_X:
		{
			if (!__gc->g_calribmodeToggle) {
				std::cout << "\n" << "not calibration mode!!" << std::endl;
				return;
			}
			//#define MYDEFINE

			cv::Mat downloadedImg = cv::imread(__gc->g_folder_trackingInfo + "test_downloaded.png");

			if (downloadedImg.empty()) {
				std::cout << "\n" << "no downloaded image" << std::endl;
				return;
			}
			__gc->g_downloadCompleted = 100;
			//cv::cvtColor(downloadedImg, g_curScanGrayImg, cv::COLOR_RGB2GRAY);
			std::cout << "\n" << "calibration with the previous downloaded image" << std::endl;

			track_info trk;
			//__gc->g_track_que.wait_and_pop(trk);
			trk = __gc->g_track_que.front();
			//if (calibtask::CalibrationWithPhantom(__gc->g_CArmRB2SourceCS, g_curScanGrayImg, &trk, __gc->g_useGlobalPairs)) {
			//	cv::Mat processColorImg = cv::imread(__gc->g_folder_trackingInfo + "test_circle_sort.png");
			//	//download_completed = false; 
			//	saveAndChangeViewState(trk, char('3'), sidScene, cidCam1, 0, processColorImg);
			//	__gc->g_ui_banishing_count = 100;
			//}
			break;
		}
		case Qt::Key_3:
		{
			emit signumberkey(char('3'));
			break;
		}
		case Qt::Key_1:
		{
			emit signumberkey(char('1'));
			break;
		}
		case Qt::Key_2:
		{
			emit signumberkey(char('2'));
			break;
		}
		default: {
			emit signumberkey(1);
			break;
		}
		}
	}
	}
}