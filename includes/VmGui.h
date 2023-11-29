#pragma once
#include <memory>
#include <string>
#include <stdexcept>
#include <functional>

#include "VisMtvApi.h"
#include "ApiUtility.hpp"

// math using GLM
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"

inline std::string GetDataPath()
{
	using namespace std;
	char ownPth[2048];
	GetModuleFileNameA(NULL, ownPth, (sizeof(ownPth)));
	string exe_path = ownPth;
	string exe_path_;
	size_t pos = 0;
	std::string token;
	string delimiter = "\\";
	while ((pos = exe_path.find(delimiter)) != std::string::npos) {
		token = exe_path.substr(0, pos);
		if (token.find(".exe") != std::string::npos) break;
		exe_path += token + "\\";
		exe_path_ += token + "\\";
		exe_path.erase(0, pos + delimiter.length());
	}
	return exe_path_ + "..\\..\\data\\";
};

// https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf#:~:text=%23include%20%3Cmemory,0%27%20inside%0A%7D
template<typename ... Args>
inline std::string StringFormat(const std::string& format, Args ... args)
{
	int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
	if (size_s <= 0) { throw std::runtime_error("Error during formatting."); }
	auto size = static_cast<size_t>(size_s);
	std::unique_ptr<char[]> buf(new char[size]);
	std::snprintf(buf.get(), size, format.c_str(), args ...);
	return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

namespace vmgui {

	__dojostatic void SetImGuiContext(void* imGuiContext);

	__dojoclass ImGuiVzmWindow {
	public:
		enum class OnEvents : int {
			NONE = 0x0,
			GENERAL_MOVE = 0x1,
			GENERAL_ZOOM = 0x1 << 1,
			TRANSFORMER = 0x1 << 2,
			CLIPPER = 0x1 << 3,
			CUSTOM_MOVE = 0x1 << 4,
			CUSTOM_DOWN = 0x1 << 5,
			CUSTOM_WHEEL = 0x1 << 6,
		};
	private:
		std::string __windowName = "";
		int __sidScene = 0;
		int __cidRender = 0;
		int __w = 0, __h = 0;
		glm::fvec3 __centerStage = glm::fvec3(0);
		float __centerStageScale = 100.f;

		vzm::CameraParameters __cpRender; // current
		vzmutils::GeneralMove __gmEvent;

		bool __zoomMoveEnabled = false, __zoomMoveLDragPan = true;
		bool __zoomMoveUserMoveInvert = false, __zoomMovePreserveCenter = false;

		void* __components = NULL;

		glm::fvec2 __prevMousePos = glm::fvec2(-10000, -10000);
		glm::fvec2 __curMousePos = glm::fvec2(-10000, -10000);

		int __eventFlag = (int)OnEvents::GENERAL_MOVE | (int)OnEvents::GENERAL_ZOOM;
		OnEvents __eventStart = OnEvents::NONE;

		void* __dx11ImGuiDevice = nullptr;

		typedef std::function<void(ImGuiVzmWindow*)> CFn;
		std::map<OnEvents, std::vector<CFn>> __callbacks;

		bool __skipRender = false;

	public:

		ImGuiVzmWindow(const std::string& windowName, const int sidScene, const int cidRender, const void* dx11ImGuiDevice);
		~ImGuiVzmWindow();
		std::string GetWindowName();
		void SetStageCenterScale(const glm::fvec3& centerStage, const float centerStageScale);
		void GetStageCenterScale(glm::fvec3& centerStage, float& centerStageScale) {
			centerStage = __centerStage;
			centerStageScale = __centerStageScale;
		}
		glm::fvec2 GetMousePos() {
			return __curMousePos;
		}
		void SkipRender(const bool skipRender) { __skipRender = skipRender; }
		void GetSceneAndCamera(int& sid, int& cid) {
			sid = __sidScene;
			cid = __cidRender;
		}
		void AddCallback(const OnEvents oe, void (*fn)(ImGuiVzmWindow* pThis)) {
			std::vector <CFn>& fnList = __callbacks[oe];
			fnList.push_back(fn);
		}
		void RemoveCallback(const OnEvents oe, void (*fn)(ImGuiVzmWindow* pThis)) {
			std::vector <CFn>& fnList = __callbacks[oe];
			int idx = -1;
			for (int i = 0; i < fnList.size(); i++) {
				idx++;
				if ((void*)&fnList[i] == fn) break;
			}
			if (idx >= 0)
				fnList.erase(fnList.begin() + idx);
		}
		void EnableTransformer(const bool enabled);
		void EnableCustomMove(const bool enabled);
		void EnableCustomDown(const bool enabled);
		void EnableCustomWHEEL(const bool enabled);
		void EnableBoxClipper(const bool enabled, const int aidTarget);
		void EnableMouseMove(const bool enabled);
		void EnableMouseZoom(const bool enabled);
		void View(glm::ivec2* xyOnImage);
		void SetStartEvent(const OnEvents oe) {
			__eventStart = oe;
		}
		// note then the MRight Rotation will be disabled
		void EnableZoomMoveOnMRightDrag(const bool zoomMoveEnabled, const bool mLDragPan = true, // (zoomMoveEnabled && !mLDragPan) then Rotate
			const bool zoomMoveUserMoveInvert = false, const bool zoomMovePreserveCenter = false) {
			__zoomMoveEnabled = zoomMoveEnabled;
			__zoomMoveLDragPan = mLDragPan;
			__zoomMoveUserMoveInvert = zoomMoveUserMoveInvert;
			__zoomMovePreserveCenter = zoomMovePreserveCenter;
		}
	};

	__dojoclass ImTermWindow
	{
	private:
	public:
		ImTermWindow();
		bool ShowTerminal();
	};
}