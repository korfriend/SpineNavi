#pragma once
#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#define POSITIVE 1
#define NEGATIVE -1

//#ifndef SOCKET_CLIENT_CLASS
//#define SOCKET_CLIENT_CLASS
//#ifndef BOOST_ASIO_HPP
//#include "../ApiUtility.hpp"
//#endif
//#endif

// math using GLM
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"

namespace vzmtools {
	using namespace glm;

	enum class OnState {
		NOBINDING = 0,
		READY,
		ONSTART,
		ONMOVE
	};

	 // recommend : cursor update using CheckOpValidity 

	__dojoclass ClipBox {
	private:
		bool __isVisible;
		int __groupActorId;
		int __solidBoxActorId;
		int __pickPlaneActorId;
		int __wireBoxActorId;
		int __currentCamId;
		OnState __onState;
		vzm::BoxTr __clipBoxLS; // groupActor's local space
		int __sceneId; // automatic update
		std::set<int> __boundActors;

		glm::fvec2 __startPosSS;
		glm::fvec3 __startPlaneDirLS;
		glm::fmat4x4 __startWS2LS;
		vzm::BoxTr __startClipBoxLS;
		void ApplyClipBox();
	public:
		ClipBox();
		~ClipBox();

		// note that clipbox is applied to the bound actors 
		// when calling
		// SetClipBoxGeometry, SetAndAppendClipBoxToActor, OnMove (valid case)
		void SetClipBoxVisibility(const bool visible);
		bool GetClipBoxVisibility();
		void SetClipBoxColors(const glm::fvec4& edgeColor, const glm::fvec4& planeColor);
		void GetClipBoxColors(glm::fvec4& edgeColor, glm::fvec4& planeColor);
		void SetClipBoxGeometry(const vzm::BoxTr& boxTr);
		bool SetAndAppendClipBoxToActor(const int actorId, const bool fitClipBoxToActor = true); // the actor should have a geometry
		void GetClipBoxGeometry(vzm::BoxTr& boxTr); // note that boxTr is used to set the Local Space

		void SetSceneCameraID(const int camId);
		void GetSceneCameraID(int& camId);

		bool BindActor(const int actorId);
		bool UnbindActor(const int actorId);

		bool CheckOpValidity(const glm::fvec2& pos2d);
		bool OnStart(const glm::fvec2& pos2d);
		bool OnMove(const glm::fvec2& pos2d);
		void OnRelease();

		OnState GetOnState();
	};

	__dojoclass TransformTools{
	public:
		enum TransformMode {
			UNDEFINED_AXIS = 0,
			TRANSLATE,
			SCALE,
			ROTATE
		};
	private:
		int m_target_actor_id;
		int m_picked_actor_id;
		int m_hover_actor_id;
		glm::fvec4 m_prev_hover_actor_color;
		int m_scene_id;
		int m_cam_id;
		int m_handle_id;
		TransformMode m_mode; // 0 : Axis    1 : Translate    2 : Scale    3 : Rotate
		float m_init_distance = 1.f;
		int m_rotate_sign = 0;

		int m_line_actor_id, m_translate_actor_id, m_scale_actor_id, m_rotate_actor_id;
		int m_line_actor_ids[3];
		int m_translate_actor_ids[4];
		int m_scale_actor_ids[4];
		int m_rotate_actor_ids[3];
		int m_rotate_line_actor_ids[3];
		int m_onmove_actor_id;
		int m_onmove_frame_actor_id;
		int m_onmove_frameLine_actor_id;
		int m_onmove_text_actor_id;

		int m_line_object_ids[3];
		int m_translate_object_ids[4];
		int m_scale_object_ids[4];
		int m_rotate_object_ids[3];
		int m_rotate_line_object_ids[3];
		int m_onmove_object_id;
		int m_onmove_frame_object_id;
		int m_onmove_frameLine_object_id;
		int m_onmove_text_object_id;

		glm::ivec2 m_pos_ss_original;
		vzm::ActorParameters m_original_params, m_handle_params;
		float getRatioActor_NP2WS();
		void setInitDistance(const int actor_id);
		float getInitDistance();
		void updateHandleScale();
		void testFunction(); //Only For Test
		glm::fvec3 getActorWorldPos(const int actor_id);
		glm::fvec3 getActorLocalPos(const int actor_id);
		void MakeHandleActor();
		void RemoveHandleActor();
		float max_f(float a, float b, float c);
		void followActorColor(const int followed_actor, const int following_actor); 
		bool CorrectScaleAxisColor(const int axis_id); // if (mode != SCALE) return false
	public:
		TransformTools();
		~TransformTools();

		void SetSceneCameraID(const int camId);
		void GetSceneCameraID(int& camId);

		bool CheckOpValidity(const int ss_X, const int ss_Y, int* pickedActorId = NULL);
		bool OnHover(const int ss_X, const int ss_Y); // call render inside the function
		bool OnStart(const int ss_X, const int ss_Y); // 'void start' is renamed 
		bool OnMove(const int ss_X, const int ss_Y); // unify translate, scale, and rotate
		void OnRelease(); // call render inside the function

		TransformMode GetMode();
		void SetMode(const TransformMode mode);  // 0 : Axis    1 : Translate    2 : Scale    3 : Rotate
		bool SetTargetActor(const int actor_id); // 'void attach' is renamed 
	};
}