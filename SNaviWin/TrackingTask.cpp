#include "TrackingTask.h"

#include "VisMtvApi.h"
#include "ApiUtility.hpp"

#include "../optitrk/optitrk.h"

#include <opencv2/opencv.hpp>

namespace trackingtask {

	__GC* __gc = NULL;

	void SetGlobalContainer(__GC* gcp) {
		__gc = gcp;
	}

	void OptiTrackingProcess() {
		if (__gc == NULL) return;

		using namespace std;
		using namespace glm;

		cv::FileStorage fs(__gc->g_folder_trackingInfo + "tooltip.txt", cv::FileStorage::Mode::READ);
		if (fs.isOpened()) {
			cv::Mat ocv3(1, 3, CV_32FC1);
			fs["ToolTip"] >> ocv3;
			memcpy(&__gc->g_toolTipPointLS, ocv3.ptr(), sizeof(glm::fvec3));
		}
		fs.release();

		while (__gc->g_tracker_alive)
		{
			//Sleep(postpone);
			optitrk::UpdateFrame();

			std::vector<std::string> rbNames;
			int numAllFramesRBs = optitrk::GetRigidBodies(&rbNames);

			track_info trk_info;

			// we are using optitrack trio:120 (using 3 cameras)
			for (int i = 0; i < 3; i++) {
				fmat4x4 matCam2WS;
				optitrk::GetCameraLocation(i, __FP matCam2WS);
				trk_info.AddTrackingCamPose(matCam2WS);
			}

			for (int i = 0; i < numAllFramesRBs; i++)
			{
				string rbName;// = rbNames[i];
				fmat4x4 matLS2WS;
				float rbMSE;
				vector<float> posMKs;
				vector<float> mkQualities;
				bitset<128> rbCid;
				fquat qvec;
				fvec3 tvec;
				if (optitrk::GetRigidBodyLocationByIdx(i, (float*)&matLS2WS, &rbCid, &rbMSE, &posMKs, NULL, &mkQualities, &rbName, (float*)&qvec, (float*)&tvec)) {

					if (rbMSE > 0.01) cout << "\n" << "problematic error!! " << rbName << " : " << rbMSE << endl;

					int numRbMks = (int)posMKs.size() / 3;
					vector<fvec3> mkPts(numRbMks);
					memcpy(&mkPts[0], &posMKs[0], sizeof(fvec3) * numRbMks);
					map<string, map<track_info::MKINFO, std::any>> rbmkSet;
					for (int j = 0; j < numRbMks; j++) {
						//CID = 0, // std::bitset<128>
						//POSITION = 1, // glm::fvec3, world space
						//MK_NAME = 2, // string
						//MK_QUALITY = 3 // float // only for RB_MKSET
						string mkName = rbName + ":Marker" + to_string(j + 1);
						auto& pt = rbmkSet[mkName];
						pt[track_info::MKINFO::MK_CID] = 0;
						pt[track_info::MKINFO::POSITION] = mkPts[j];
						pt[track_info::MKINFO::MK_NAME] = mkName;
						pt[track_info::MKINFO::MK_QUALITY] = mkQualities[j];


						float mk_qual = mkQualities[j];
						//if (mk_qual < 0.9) cout << "\n" << "rb(" << j << ") marker problematic error!!  : " << mk_qual << endl;
					}
					trk_info.AddRigidBody(rbName, rbCid, matLS2WS, qvec, tvec, rbMSE, rbmkSet);

				}
			}
			vector<float> mkPts;
			vector<float> mkResiduals;
			vector<bitset<128>> mkCIDs;
			optitrk::GetMarkersLocation(&mkPts, &mkResiduals, &mkCIDs);
			int numMKs = (int)mkCIDs.size();
			for (int i = 0; i < numMKs; i++) {
				fvec3 pos = fvec3(mkPts[3 * i + 0], mkPts[3 * i + 1], mkPts[3 * i + 2]);
				string mkName = "Marker" + to_string(i + 1);
				trk_info.AddMarker(mkCIDs[i], pos, mkName);
			}

			auto GetWsMarkersFromSelectedMarkerNames = [&trk_info](const std::map<std::string, int>& selectedMkNames, std::vector<glm::fvec3>& selectedMks) {

				int numMks = (int)selectedMkNames.size();
				selectedMks.assign(numMks, glm::fvec3(0));

				int numSuccess = 0;
				for (auto it = selectedMkNames.begin(); it != selectedMkNames.end(); it++) {
					std::map<track_info::MKINFO, std::any> mkInfo;
					if (trk_info.GetMarkerByName(it->first, mkInfo)) {
						numSuccess++;
						glm::fvec3 pos = std::any_cast<glm::fvec3>(mkInfo[track_info::MKINFO::POSITION]);
						selectedMks[it->second] = pos;
					}
				}

				return numSuccess == numMks;
			};
			auto UpdateRigidBodiesInThread = [&]() {
				rbNames.clear();
				numAllFramesRBs = optitrk::GetRigidBodies(&rbNames);
				for (int i = 0; i < numAllFramesRBs; i++) {
					cout << rbNames[i] << endl;
				}
			};
			// because the critical section APIs are not successfully applied, naive path is added to avoid the read/write/modify hazard
			// e.g., this tracking thread is trying to access the optitrack tracking resources while the main thread (event) is modifying the optitrack tracking resources
			static std::time_t timeBegin;
			if (__gc->g_optiEvent != OPTTRK_THREAD_FREE) {
				std::vector<glm::fvec3> selectedMks;
				GetWsMarkersFromSelectedMarkerNames(__gc->g_selectedMkNames, selectedMks); // WS

				// if you want to use switch with g_optiMode, make sure thread-safe of g_optiMode
				if (__gc->g_optiEvent == OPTTRK_THREAD_C_ARM_REGISTER) {
					optitrk::SetRigidBody("c-arm", __gc->g_selectedMkNames.size(), (float*)&selectedMks[0]);
					UpdateRigidBodiesInThread();
					optitrk::StoreProfile(__gc->g_profileFileName);
					__gc->g_optiEvent = OPTTRK_THREAD_FREE;
				}
				else if (__gc->g_optiEvent == OPTTRK_THREAD_TOOL_REGISTER) {
					if (std::time(NULL) - timeBegin > 0) {
						cout << "\nregister tool success!" << endl;
						int numSelectedMKs = (int)__gc->g_selectedMkNames.size();
						optitrk::SetRigidBody("tool", numSelectedMKs - 1, (float*)&selectedMks[0]);

						__gc->g_optiEvent = OPTTRK_THREAD_TOOL_UPDATE;
					}
				}
				else if (__gc->g_optiEvent == OPTTRK_THREAD_TOOL_UPDATE) {

					glm::fmat4x4 rbLS2WS;
					int rbIdx = -1;
					std::vector<float> buf_rb_mks;
					//std::vector<float> ws_mks;
					if (optitrk::GetRigidBodyLocationByName("tool", __FP rbLS2WS, &rbIdx, NULL, NULL, &buf_rb_mks)) {
						std::vector<glm::fvec3> pos_rb_mks(buf_rb_mks.size() / 3);
						memcpy(&pos_rb_mks[0], &buf_rb_mks[0], sizeof(float) * buf_rb_mks.size());

						glm::fmat4x4 rbWS2LS = glm::inverse(rbLS2WS);

						__gc->g_toolTipPointLS = vzmutils::transformPos(selectedMks[selectedMks.size() - 1], rbWS2LS);

						cv::FileStorage fs(__gc->g_folder_trackingInfo + "tooltip.txt", cv::FileStorage::Mode::WRITE);
						cv::Mat ocv3(1, 3, CV_32FC1);
						memcpy(ocv3.ptr(), &__gc->g_toolTipPointLS, sizeof(glm::fvec3));
						fs << "ToolTip" << ocv3;
						fs.release();

						// use cam direction...
						UpdateRigidBodiesInThread();
						optitrk::StoreProfile(__gc->g_profileFileName);
						__gc->g_optiEvent = OPTTRK_THREAD_FREE;
					}
				}
			}
			else {
				timeBegin = std::time(NULL);
			}

			__gc->g_track_que.push(trk_info);
		}
	}
}
