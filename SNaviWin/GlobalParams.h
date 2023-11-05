#pragma once

#include <iostream>
#include <windows.h>
#include <any>
#include <map>
#include <queue>
#include <bitset>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>

#include <spdlog/spdlog.h>

// math using GLM
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"
#include <glm/gtx/quaternion.hpp>

#define SCANIMG_W 1296
#define SCANIMG_H 1296
#define FLIP_SCANIMAGE true
#define EXTRINSIC_PHANTOM_IMG_INDEX 3

// https://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
template<typename Data>
class concurrent_queue // single consumer
{
private:
	std::queue<Data> the_queue;
	mutable std::mutex the_mutex;

	std::condition_variable the_condition_variable;
	int MAX_QUEUE;
	Data undefinedData;

public:
	concurrent_queue(int cap = 10)
	{
		MAX_QUEUE = cap;
	}

	void SetMaxQueue(int cap) {
		MAX_QUEUE = cap;
	}

	void wait_for_data()
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		while (the_queue.empty())
		{
			the_condition_variable.wait(lock);
		}
	}

	void wait_and_pop(Data& popped_value)
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		while (the_queue.empty())
		{
			the_condition_variable.wait(lock);
		}

		popped_value = the_queue.front();
		the_queue.pop();
	}

	//void postphone_back_and_clear(Data& popped_value)
	//{
	//	boost::mutex::scoped_lock lock(the_mutex);
	//	while (the_queue.size() < MAX_QUEUE)
	//	{
	//		the_condition_variable.wait(lock);
	//	}
	//
	//	popped_value = the_queue.back();
	//	std::queue<Data> empty_queue;
	//	std::swap(the_queue, empty_queue);
	//}

	void push(const Data& data) //enque
	{
		//boost::mutex::scoped_lock lock(the_mutex);
		//the_queue.push(data);

		std::unique_lock<std::mutex> lock(the_mutex);
		bool const was_empty = the_queue.empty();
		the_queue.push(data);
		if (the_queue.size() > MAX_QUEUE)
		{
			the_queue.pop();
		}

		lock.unlock(); // unlock the mutex

		if (was_empty)
		{
			the_condition_variable.notify_one();
		}
	}

	bool empty() const
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		return the_queue.empty();
	}

	Data& front()
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		if (the_queue.empty())
			return undefinedData;
		return the_queue.front();
	}

	Data const& front() const
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		if (the_queue.empty())
			return undefinedData;
		return the_queue.front();
	}

	void pop() // deque
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		the_queue.pop();
	}
};

template<size_t sz> struct bitset_comparer {
	bool operator() (const std::bitset<sz>& b1, const std::bitset<sz>& b2) const {
		std::string b1ll = b1.to_string();
		std::string b2ll = b2.to_string();
		return b1ll < b2ll;
	}
};

struct track_info
{
	// key : rigidbody name
public:
	enum class MKINFO : int
	{
		MK_CID = 0, // std::bitset<128>
		POSITION = 1, // glm::fvec3, world space
		MK_NAME = 2, // string
		MK_QUALITY = 3, // float // only for RB_MKSET
		POSITION_RBPC = 4, // glm::fvec3, only for RB's PCMK
		TRACKED = 5 // bool
	};
	std::map<std::string, std::any> testData; // including serialized...
private:
	enum class RBINFO : int
	{
		RB_CID = 0, // std::bitset<128>
		LS2WS = 1, // glm::fmat4x4
		MK_MSE = 2, // float
		RB_MKSET = 3, // map<string, map<MKINFO, any>>
		QT_LS2WS = 4, // glm::fquat
		TV_LS2WS = 5, // glm::fvec3
		TRACKED = 6, // bool
	};

	std::map<std::string, std::map<RBINFO, std::any>> __rbinfo;
	std::map<std::bitset<128>, std::map<MKINFO, std::any>, bitset_comparer<128>> __mksByCid;
	std::map<std::string, std::bitset<128>> __mksByName;

	std::vector<glm::fmat4x4> __matrice_Cam2WS;

	void MKInfoSerialize() {

	}
	void MKInfoDeserialize() {

	}
public:
	track_info() { }

	void AddTrackingCamPose(const glm::fmat4x4& matCam2WS) {
		__matrice_Cam2WS.push_back(matCam2WS);
	}

	bool GetTrackingCamPose(const int camIdx, glm::fmat4x4& matCam2WS) {
		if (camIdx >= __matrice_Cam2WS.size()) 
			return false;
		matCam2WS = __matrice_Cam2WS[camIdx];
		return true;
	}

	bool IsValidTracking() {
		return NumCameras() + NumRigidBodies() + NumMarkers() > 0;
	}

	int NumCameras() {
		return (int)__matrice_Cam2WS.size();
	}

	int NumRigidBodies() {
		return (int)__rbinfo.size();
	}

	bool GetRigidBodyByIdx(const int rbIdx, std::string* rbName, glm::fmat4x4* matLS2WS, float* mkMSE, 
		std::map<std::string, std::map<MKINFO, std::any>>* rbmkSet, std::bitset<128>* cid) {
		auto it = __rbinfo.begin();

		if (rbIdx >= (int)__rbinfo.size()) return false;
		
		std::advance(it, rbIdx);

		if (it == __rbinfo.end()) return false;

		auto& v = it->second;
		if (rbName)
			*rbName = it->first;
		if (matLS2WS)
			*matLS2WS = std::any_cast<glm::fmat4x4>(v[RBINFO::LS2WS]);
		if (mkMSE)
			*mkMSE = std::any_cast<float>(v[RBINFO::MK_MSE]);
		if (rbmkSet)
			*rbmkSet = std::any_cast<std::map<std::string, std::map<MKINFO, std::any>>>(v[RBINFO::RB_MKSET]);
		if (cid)
			*cid = std::any_cast<std::bitset<128>>(v[RBINFO::RB_CID]);

		return std::any_cast<bool>(v[RBINFO::TRACKED]);
	}

	bool GetRigidBodyByName(const std::string& rbName, glm::fmat4x4* matLS2WS, float* mkMSE, 
		std::map<std::string, std::map<MKINFO, std::any>>* rbmkSet, 
		std::bitset<128>* cid) {
		auto it = __rbinfo.find(rbName);
		if (it == __rbinfo.end())
			return false;

		auto& v = it->second;
		if (matLS2WS)
			*matLS2WS = std::any_cast<glm::fmat4x4>(v[RBINFO::LS2WS]);
		if (mkMSE)
			*mkMSE = std::any_cast<float>(v[RBINFO::MK_MSE]);
		if (rbmkSet)
			*rbmkSet = std::any_cast<std::map<std::string, std::map<MKINFO, std::any>>>(v[RBINFO::RB_MKSET]);
		if (cid)
			*cid = std::any_cast<std::bitset<128>>(v[RBINFO::RB_CID]);

		return std::any_cast<bool>(v[RBINFO::TRACKED]);
	}

	bool GetRigidBodyQuatTVecByName(const std::string& rbName, glm::fquat* q, glm::fvec3* t) {
		auto it = __rbinfo.find(rbName);
		if (it == __rbinfo.end())
			return false;

		auto& v = it->second;
		if (q)
			*q = std::any_cast<glm::fquat>(v[RBINFO::QT_LS2WS]);
		if (t)
			*t = std::any_cast<glm::fvec3>(v[RBINFO::TV_LS2WS]);

		return std::any_cast<bool>(v[RBINFO::TRACKED]);
	}

	bool AddRigidBody(const std::string& rbName, const std::bitset<128>& cid, const glm::fmat4x4& matLS2WS, 
		const glm::fquat& qtLS2WS, const glm::fvec3& tvec, const float mkMSE, const bool isTracked,
		const std::map<std::string, std::map<MKINFO, std::any>>& rbmkSet)
	{
		if (__rbinfo.find(rbName) != __rbinfo.end())
			return false;

		std::map<RBINFO, std::any>& v = __rbinfo[rbName];
		v[RBINFO::RB_CID] = cid;
		v[RBINFO::LS2WS] = matLS2WS;
		v[RBINFO::MK_MSE] = mkMSE;
		v[RBINFO::RB_MKSET] = rbmkSet;
		v[RBINFO::QT_LS2WS] = qtLS2WS;
		v[RBINFO::TV_LS2WS] = tvec;
		v[RBINFO::TRACKED] = isTracked;

		//std::vector<char> _buffer(sizeof(glm::fmat4x4) + sizeof(glm::fquat) + sizeof(glm::fvec3) + sizeof(float) + sizeof(___));
	}

	bool GetMarkerByName(const std::string& mkName, std::map<MKINFO, std::any>& mk)
	{
		auto it = __mksByName.find(mkName);
		if (it == __mksByName.end()) return false;
		return GetMarkerByCID(it->second, mk);
	}

	bool GetMarkerByCID(const std::bitset<128>& cid, std::map<MKINFO, std::any>& mk)
	{
		auto it = __mksByCid.find(cid);
		if (it == __mksByCid.end()) return false;
		mk = it->second;
		return true;
	}

	int NumMarkers() {
		return (int)__mksByName.size();
	}

	bool GetMarkerByIdx(const int mkIdx, std::map<MKINFO, std::any>& mk)
	{
		if (mkIdx >= __mksByCid.size()) return false;
		auto it = __mksByCid.begin();
		std::advance(it, mkIdx);
		mk = it->second;
		return true;
	}

	//void SetMarkers(const std::vector<std::bitset<128>>& cids, const std::vector<glm::fvec3>& mkPts,
	//	const std::vector<string>& rbNames, const std::vector<float>& residues)
	//{
	//	int count = (int)cids.size();
	//	for (int i = 0; i < count; i++) {
	//		const std::bitset<128> cid = cids[i];
	//		const glm::fvec3 pos = mkPts[i];
	//		const string rbName = rbNames[i];
	//		const float residue = residues[i];
	//		map<MKINFO, std::any>& v = __mksByCid[cid];
	//		v[CID] = cid;
	//		v[POSITION] = pos;
	//		v[RB_NAME] = rbName;
	//		v[RESIDUE] = residue;
	//
	//		__mksByName[rbName] = cid;
	//	}
	//}

	void AddMarker(const std::bitset<128>& cid, const glm::fvec3& mkPos, const std::string& mkName)
	{
		std::map<MKINFO, std::any>& v = __mksByCid[cid];
		v[MKINFO::MK_CID] = cid;
		v[MKINFO::POSITION] = mkPos;
		v[MKINFO::MK_NAME] = mkName;
		if (mkPos.x > 9999.f && mkPos.y > 9999.f && mkPos.z > 9999.f) {
			v[MKINFO::MK_QUALITY] = 0;
			v[MKINFO::TRACKED] = false;
		}
		else {
			v[MKINFO::MK_QUALITY] = 1.f;
			v[MKINFO::TRACKED] = true;

		}

		__mksByName[mkName] = cid;
	}
};

enum class OPTTRK_THREAD {
	FREE = 0,
	CALIB_REGISTER,
	CHECKER_REGISTER,
	C_ARM_REGISTER,
	C_ARM_SHOT_MOMENT,
	TOOL_REGISTER,
	TOOL_UPDATE,
	TOOL_PIVOT,
	TOOL_RESET_PIVOT,
};

enum class OPTTRK_RECMODE {
	NONE = 0,
	RECORD,
	LOAD,
};

enum class RENDER_THREAD {
	FREE = 0,
	DOWNLOAD_IMG_PROCESS,
	BUSY,
};

typedef struct GlobalContainer {

	//================ NOT THREAD SAFE GROUP 0
	std::string g_folder_data;
	std::string g_folder_trackingInfo;
	std::string g_profileFileName;
	std::string g_recFileName;
	std::string g_recScanName;
	std::string g_configFileName;

	std::string g_sceneName;// = "Scene1"; // Scene1, Scene2
	std::string g_camName;// = "Cam1"; // World Camera, CArm Camera
	std::string g_camName2;// = "Cam2"; // World Camera, CArm Camera

	std::ofstream g_recFileStream;
	std::ofstream g_recScanStream;

	std::map<std::string, int> g_selectedMkNames;
	std::vector<std::string> g_calibSortMKNames;
	std::map<int, int> g_mapAidGroupCArmCam;

	std::map<std::string, std::vector<glm::fvec3>> g_rbLocalUserPoints;

	std::vector<glm::fvec3> g_testMKs;
	std::vector<glm::fvec3> g_global_calib_MKs;

	glm::fmat4x4 g_CArmRB2SourceCS;

	std::vector<char> g_downloadImgBuffer; // grayscale

	struct mfvec3
	{
		glm::fvec3 p;
		mfvec3()
		{
		}
		mfvec3(glm::fvec3 _p)
		{
			p = _p;
		}
		bool operator == (mfvec3 other) const
		{
			if (p.x == other.p.x && p.y == other.p.y && p.z == other.p.z)
				return true;
			else
				return false;
		}
		bool operator < (mfvec3 other) const
		{
			double dTemp = 100000;
			double d1 = p.z * dTemp * dTemp + p.y * dTemp + p.x;
			double d2 = other.p.z * dTemp * dTemp + other.p.y * dTemp + other.p.x;
			if (d1 < d2)
				return true;
			else
				return false;
		}
	};

	struct homographyPairCompare
	{
		bool operator() (const glm::fvec3& lhs, const glm::fvec3& rhs) const
		{
			double dTemp = 100; // enough for  meter scale...
			double d1 = lhs.z * dTemp * dTemp + lhs.y * dTemp + lhs.x;
			double d2 = rhs.z * dTemp * dTemp + rhs.y * dTemp + rhs.x;
			if (d1 < d2)
				return true;
			else
				return false;
		}
	};

	bool g_showCalibMarkers = false;
	std::map<glm::fvec3, glm::fvec2, homographyPairCompare> g_homographyPairs;
	std::string g_optiRbStates;

	//================ THREAD SAFE GROUP ==================
	std::atomic<OPTTRK_THREAD> g_optiEvent; // { OPTTRK_THREAD::FREE };
	std::atomic<OPTTRK_RECMODE> g_optiRecordMode;
	std::atomic_int g_optiToolId = 0;
	std::atomic_int g_optiRecordFrame; // count in main thread (same as render thread)
	int g_optiRecordPeriod;
	int g_optiPivotSamples;
	std::atomic_int g_optiPivotProgress;

	std::atomic<RENDER_THREAD> g_renderEvent;// { RENDER_THREAD::FREE };

#define NETWORK_THREAD_FREE 0
	std::atomic_int g_networkEvent;// { NETWORK_THREAD_FREE };

	std::atomic_bool g_tracker_alive;// { true };
	std::atomic_bool g_network_alive;

	// DOJO: asynchronous (read write modify) queue of tracking info.
	// tracking thread 에서 queue 에 등록 (최신 정보 insert) 및 삭제 (오래된 element push out)
	// timer thread (rendering thread) 에서 queue 의 정보를 pop out (삭제) 하여 사용 

	std::atomic_bool g_markerSelectionMode;// { false };
	std::atomic_int g_downloadCompleted;// { 0 };
	std::atomic_int g_ui_banishing_count;// { 0 };

	concurrent_queue<track_info> g_track_que;
	track_info g_track_info_shotmoment;
	std::atomic_int g_is_ready_for_shotmoment = false;

	std::map<std::string, std::any> g_customParams;

	std::string g_lastStoredImageName = "";

	std::string  g_networkState = ""; // frames
	std::atomic_int g_error_duration; // frames
	std::string g_error_text;
	std::shared_ptr<spdlog::logger> g_engineLogger;
	void SetErrorCode(std::string errorText, int duration = 100) {
		g_error_text = errorText;
		g_error_duration = duration;
		g_engineLogger->error(errorText);
	}
	void Init() {

		g_folder_data = "";
		g_folder_trackingInfo = "";
		g_profileFileName = "";
		g_recFileName = "";

		g_sceneName = "Scene1"; // Scene1, Scene2
		g_camName = "Cam1"; // World Camera, CArm Camera
		g_camName2 = "Cam2"; // World Camera, CArm Camera

		g_showCalibMarkers = false;

		g_optiEvent = OPTTRK_THREAD::FREE;
		g_optiRecordMode = OPTTRK_RECMODE::NONE;
		g_renderEvent = RENDER_THREAD::FREE;
		g_networkEvent = NETWORK_THREAD_FREE;
		g_optiRecordFrame = 0;
		g_optiRecordPeriod = 10;
		g_optiPivotSamples = 10000;
		g_optiPivotProgress = 0;

		g_tracker_alive = true;
		g_network_alive = true;

		g_markerSelectionMode = false;
		g_downloadCompleted = 0;
		g_ui_banishing_count = 0;

		g_error_duration = 0;
		g_error_text = "";

	}

	void Deinit() {
		if (g_recFileStream.is_open()) {
			g_recFileStream.close();
		}
	}
} __GC;