#include "TrackingTask.h"

#include "VisMtvApi.h"
#include "ApiUtility.hpp"
//#include "naviHelpers.hpp"

#include "../optitrk/optitrk.h"

#include "rapidcsv/rapidcsv.h"

#include <time.h>
#include <opencv2/opencv.hpp>

namespace trackingtask {

	__GC* __gc = NULL;

	void InitializeTask(__GC* gcp) {
		__gc = gcp;
	}

	void OptiTrackingProcess() {
		if (__gc == NULL) return;

		using namespace std;
		using namespace glm;

		std::vector<std::string> toolNames = {"t-needle", "troca"};
		for (int i = 0; i < 2; i++) {
			cv::FileStorage fs(__gc->g_folder_trackingInfo + "tooltip_" + toolNames[i] + ".txt", cv::FileStorage::Mode::READ);
			if (fs.isOpened()) {
				std::vector<glm::fvec3> toolUserData(2);

				cv::Mat ocv3(1, 3, CV_32FC1);
				fs["ToolPos"] >> ocv3;
				memcpy(&toolUserData[0], ocv3.ptr(), sizeof(glm::fvec3));
				fs["ToolVec"] >> ocv3;
				memcpy(&toolUserData[1], ocv3.ptr(), sizeof(glm::fvec3));

				__gc->g_rbLocalUserPoints[toolNames[i]] = toolUserData;
			}
			fs.release();
		}

		std::time_t timeBegin;
		std::vector<std::string> rbNames;

		rapidcsv::Document csvTrkData;
		std::vector<std::string> rowTypes, rowNames, rowIds, rowDataType;
		int numCols, numRows;

		{
			optitrk::SetRigidBodyPropertyByName("c-arm", 0.1f, 0);
			optitrk::SetRigidBodyPropertyByName("t-needle", 0, 0);
			optitrk::SetRigidBodyPropertyByName("troca", 0, 0);
			optitrk::SetRigidBodyPropertyByName("probe", 0, 0);
		}


		while (__gc->g_tracker_alive)
		{
			track_info trk_info;
			timeBegin = std::time(NULL);
			
			auto recFinished = []() {
				if (__gc->g_recFileStream.is_open()) {
					std::cout << "CSV data has been written to " << __gc->g_recFileName << std::endl;
					__gc->g_recFileStream.close();
					__gc->g_optiRecordFrame = 0;
				}
			};
			auto string2cid = [](const std::string id) {
				std::bitset<128> cid;
				unsigned __int64 lbs;
				unsigned __int64 hbs;
				int length = id.length();
				if (length <= 16) {
					std::stringstream convert(id);
					convert >> std::hex >> lbs;
					cid |= lbs;
				}
				else {
					std::string id_lbs, id_hbs;
					id_hbs = id.substr(0, length - 16);
					id_lbs = id.substr(length - 16, 16);
					std::stringstream convertH(id_hbs);
					std::stringstream convertL(id_lbs);
					convertH >> std::hex >> hbs;
					convertL >> std::hex >> lbs;
					cid |= hbs;
					cid <<= 64;
					cid |= lbs;
				}
				return cid;
			};

			std::string optRbState = "";
			if (__gc->g_optiRecordMode == OPTTRK_RECMODE::LOAD) {

				recFinished();
				static int recDataRowCount = 0;

				if (__gc->g_optiRecordFrame == 0) {

					cv::FileStorage fs(__gc->g_folder_trackingInfo + "tooltipRecord.txt", cv::FileStorage::Mode::READ);
					if (fs.isOpened()) {
						std::vector<glm::fvec3> toolUserData(2);

						std::vector<std::string> toolNames = { "t-needle", "troca" };
						for (int i = 0; i < 2; i++) {
							cv::Mat ocv3(1, 3, CV_32FC1);
							fs["ToolPos_" + to_string(i)] >> ocv3;
							memcpy(&toolUserData[0], ocv3.ptr(), sizeof(glm::fvec3));
							fs["ToolVec_" + to_string(i)] >> ocv3;
							memcpy(&toolUserData[1], ocv3.ptr(), sizeof(glm::fvec3));
							__gc->g_rbLocalUserPoints[toolNames[i]] = toolUserData;
						}
					}
					fs.release();


					csvTrkData.Load(__gc->g_recFileName, rapidcsv::LabelParams(0, 0));

					rowTypes = csvTrkData.GetRow<std::string>(-1);
					rowNames = csvTrkData.GetRow<std::string>(0);
					rowIds = csvTrkData.GetRow<std::string>(1);
					rowDataType = csvTrkData.GetRow<std::string>(2);

					numCols = (int)csvTrkData.GetColumnCount();
					numRows = (int)csvTrkData.GetRowCount();

					// get rbNames 
					rbNames.clear();
					std::set<std::string> rbNamesChecker;
					for (int i = 0; i < numCols; i++) {
						if (rowTypes[i] == "Rigid Body") {
							string rbName = rowNames[i];
							if (rbNamesChecker.find(rbName) == rbNamesChecker.end()) {
								rbNamesChecker.insert(rbName);
								rbNames.push_back(rbName);
							}
						}
					}

					__gc->g_optiRecordFrame = 1;
					recDataRowCount = 0;
				}

				const int frameRowIdx = 4;
				const int numTotalFrames = numRows - 4;

				if (recDataRowCount >= numTotalFrames) {
					recDataRowCount = 0;
					__gc->g_optiRecordFrame = 1;
				}

				std::string frameStr = csvTrkData.GetRowName((int)frameRowIdx + (int)recDataRowCount);
				int recordFrame = std::stoi(frameStr);
				if (__gc->g_optiRecordFrame > recordFrame) {

					std::vector<std::string> rowData = csvTrkData.GetRow<std::string>((int)frameRowIdx + (int)recDataRowCount);
					recDataRowCount++;

					glm::fmat4x4 matData;
					float* pmat = glm::value_ptr(matData);
					glm::fvec3 posData;
					float* ppos = glm::value_ptr(posData);
					glm::fquat qtData;
					float* pqt = glm::value_ptr(qtData);

					//int numAllFramesRBs = (int)rbNames.size();
					vector< map<string, map<track_info::MKINFO, std::any>>> rbMkSets;// (numAllFramesRBs);
					for (int i = 0; i < numCols - 1; i++) {
						if (rowTypes[i] == "Opti Camera") {
							for (int j = 0; j < 16; j++, i++) {
								float fv = std::stof(rowData[i]);
								pmat[j] = fv;
							}
							trk_info.AddTrackingCamPose(matData);
						}
						else if (rowTypes[i] == "Rigid Body") {
							string rbName = rowNames[i];
							bitset<128> cid = string2cid(rowIds[i]);
							for (int j = 0; j < 4; j++, i++) {
								float fv = std::stof(rowData[i]);
								pqt[j] = fv;
							}
							for (int j = 0; j < 3; j++, i++) {
								float fv = std::stof(rowData[i]);
								ppos[j] = fv;
							}

							glm::fmat4x4 mat_r = glm::toMat4(qtData);
							glm::fmat4x4 mat_t = glm::translate(posData);
							glm::fmat4x4 matLS2WS = mat_t * mat_r;

							float rbMSE = std::stof(rowData[i++]);

							std::map<std::string, std::map<track_info::MKINFO, std::any>> rbmkSet;
							while (rowTypes[i] == "Rigid Body Marker") {
								string mkName = rowNames[i];
								bitset<128> mk_cid = string2cid(rowIds[i]);
								glm::fvec3 pos;
								float quality;
								for (int j = 0; j < 4; j++, i++) {
									float fv = std::stof(rowData[i]);
									if (j != 3)
										((float*)&pos)[j] = fv;
									else
										quality = fv;
								}
								std::map<track_info::MKINFO, std::any>& mk = rbmkSet[mkName];
								mk[track_info::MKINFO::MK_NAME] = mkName;
								mk[track_info::MKINFO::MK_CID] = mk_cid;
								mk[track_info::MKINFO::POSITION] = pos;
								mk[track_info::MKINFO::MK_QUALITY] = quality;
								mk[track_info::MKINFO::TRACKED] = (bool)(quality > 0.5f);
							}
							rbMkSets.push_back(rbmkSet);

							if (rbMSE >= 0)
								optRbState += rbName + "(O) ERR:" + to_string(rbMSE * 1000.f) + " mm\n";
							else
								optRbState += rbName + "(X) ERR: N/A\n";

							// 저장 시 tvec, quaternion 버전으로..
							if (!trk_info.AddRigidBody(rbName, cid, matLS2WS, qtData, posData, rbMSE, rbMSE >= 0, rbmkSet))
								__gc->g_engineLogger->error("(" + rbName + ") is already added to <track_info>!!");
						} // else if (rowTypes[i] == "Rigid Body")
						else if (rowTypes[i] == "Marker") {
							int mkIdx = 1;
							int mk_count = 0;
							for (int j = 0; j < (int)rbMkSets.size(); j++) {
								std::map<std::string, std::map<track_info::MKINFO, std::any>>& rbmkSet = rbMkSets[j];
								for (auto it = rbmkSet.begin(); it != rbmkSet.end(); it++) {
									std::bitset<128> cid = j << 8 | mk_count++;

									std::string mkName = any_cast<std::string>(it->second[track_info::MKINFO::MK_NAME]);
									//glm::fvec3 mkPos = any_cast<glm::fvec3>(it->second[track_info::MKINFO::POSITION_RBPC]);
									glm::fvec3 mkPos;
									mkPos.x = std::stof(rowData[i++]);
									mkPos.y = std::stof(rowData[i++]);
									mkPos.z = std::stof(rowData[i++]);
									//trk_info.AddMarker(cid, mkPos, "Marker" + to_string(mk_count));
									trk_info.AddMarker(cid, mkPos, mkName + " PC");
								}
							}
							i = numCols + 1;
						}
						else {
							i++;
						}
						i--;
					} // for (int i = 0; i < numCols; i++)
				}
			}
			else {	// __gc->g_optiRecordMode != OPTTRK_RECMODE::LOAD

				optitrk::UpdateFrame();

				int numAllFramesRBs = optitrk::GetRigidBodies(&rbNames);

				// we are using optitrack trio:120 (using 3 cameras)
				for (int i = 0; i < 3; i++) {
					fmat4x4 matCam2WS;
					optitrk::GetCameraLocation(i, __FP matCam2WS);
					trk_info.AddTrackingCamPose(matCam2WS);
				}

				for (int i = 0; i < numAllFramesRBs; i++)
				{
					std::string rbName;// = rbNames[i];
					fmat4x4 matLS2WS;
					float rbMSE;
					vector<float> posMKs;
					vector<float> posPCMKs;
					vector<float> mkQualities;
					vector<bool> mkTrackeds;
					bitset<128> rbCid;
					fquat qvec;
					fvec3 tvec;

					bool isTracked = optitrk::GetRigidBodyLocationByIdx(i, (float*)&matLS2WS, &rbCid, &rbMSE, &posMKs, &posPCMKs, &mkTrackeds, &mkQualities, &rbName, (float*)&qvec, (float*)&tvec);
					if (isTracked) {
						if (rbMSE > 0.01) cout << "\n" << "problematic error!! " << rbName << " : " << rbMSE << endl;
					}

					if (isTracked)
						optRbState += rbName + "(O) ERR:" + to_string(rbMSE * 1000.f) + " mm\n";
					else
						optRbState += rbName + "(X) ERR: N/A\n";


					int numRbMks = (int)posMKs.size() / 3;
					vector<fvec3> mkPts(numRbMks);
					vector<fvec3> pcmkPts(numRbMks);
					memcpy(&mkPts[0], &posMKs[0], sizeof(fvec3)* numRbMks);
					memcpy(&pcmkPts[0], &posPCMKs[0], sizeof(fvec3)* numRbMks);
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
						pt[track_info::MKINFO::POSITION_RBPC] = pcmkPts[j];
						pt[track_info::MKINFO::MK_NAME] = mkName;
						pt[track_info::MKINFO::MK_QUALITY] = mkQualities[j];
						pt[track_info::MKINFO::TRACKED] = (bool)mkTrackeds[j];

						float mk_qual = mkQualities[j];
						//if (mk_qual < 0.9) cout << "\n" << "rb(" << j << ") marker problematic error!!  : " << mk_qual << endl;
					}
					
					if (!trk_info.AddRigidBody(rbName, rbCid, matLS2WS, qvec, tvec, rbMSE, isTracked, rbmkSet))
						__gc->g_engineLogger->error("(" + rbName + ") is already added to <track_info>!!");
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
					std::string rbAssetInfo = "Current RB Assets: ";
					int i = 0;
					for (; i < numAllFramesRBs - 1; i++) {
						rbAssetInfo += rbNames[i] + ", ";
					}
					if (numAllFramesRBs > 0)
						rbAssetInfo += rbNames[i];
					__gc->g_engineLogger->info(rbAssetInfo);
				};
				// because the critical section APIs are not successfully applied, naive path is added to avoid the read/write/modify hazard
				// e.g., this tracking thread is trying to access the optitrack tracking resources while the main thread (event) is modifying the optitrack tracking resources

				static int pivotState = 0;

				static clock_t prevTime = clock(), curTime;

				switch (__gc->g_optiEvent) {
				case OPTTRK_THREAD::CHECKER_REGISTER: {
					std::vector<glm::fvec3> selectedMks;
					GetWsMarkersFromSelectedMarkerNames(__gc->g_selectedMkNames, selectedMks); // WS
					optitrk::SetRigidBody("checker", selectedMks.size(), (float*)&selectedMks[0]);
					UpdateRigidBodiesInThread();
					optitrk::StoreProfile(__gc->g_profileFileName);
					__gc->g_engineLogger->info("C-Arm Checker is successfully registered!");
					__gc->g_optiEvent = OPTTRK_THREAD::FREE;
				} break;
				case OPTTRK_THREAD::CALIB_REGISTER: {
					std::vector<glm::fvec3> selectedMks;
					GetWsMarkersFromSelectedMarkerNames(__gc->g_selectedMkNames, selectedMks); // WS
					optitrk::SetRigidBody("calib", selectedMks.size(), (float*)&selectedMks[0]);
					UpdateRigidBodiesInThread();
					optitrk::StoreProfile(__gc->g_profileFileName);
					__gc->g_engineLogger->info("Calibration Rig is successfully registered!");
					__gc->g_optiEvent = OPTTRK_THREAD::FREE;
				} break;
				case OPTTRK_THREAD::C_ARM_REGISTER: {
					std::vector<glm::fvec3> selectedMks;
					GetWsMarkersFromSelectedMarkerNames(__gc->g_selectedMkNames, selectedMks); // WS
					optitrk::SetRigidBody("c-arm", selectedMks.size(), (float*)&selectedMks[0]);
					UpdateRigidBodiesInThread();
					optitrk::StoreProfile(__gc->g_profileFileName);
					__gc->g_engineLogger->info("C-Arm is successfully registered!");
					__gc->g_optiEvent = OPTTRK_THREAD::FREE;
				} break;
				case OPTTRK_THREAD::TOOL_REGISTER: {
					std::vector<glm::fvec3> selectedMks;
					GetWsMarkersFromSelectedMarkerNames(__gc->g_selectedMkNames, selectedMks); // WS

					int numSelectedMKs = (int)selectedMks.size();
					if (__gc->g_optiToolId == 1) {
						__gc->g_engineLogger->info("T-Needle is successfully registered!");
						optitrk::SetRigidBody("t-needle", numSelectedMKs, (float*)&selectedMks[0]);
					}
					else if (__gc->g_optiToolId == 2) {
						__gc->g_engineLogger->info("Troca is successfully registered!");
						optitrk::SetRigidBody("troca", numSelectedMKs, (float*)&selectedMks[0]);
					}
					UpdateRigidBodiesInThread();
					optitrk::StoreProfile(__gc->g_profileFileName);
					__gc->g_optiEvent = OPTTRK_THREAD::FREE;
				} break;
				case OPTTRK_THREAD::TOOL_UPDATE: {

					glm::fmat4x4 rbLS2WS;
					int rbIdx = -1;
					std::vector<float> buf_rb_mks;
					//std::vector<float> ws_mks;
					std::string rbName = __gc->g_optiToolId == 1 ? "t-needle" : "troca";
					if (optitrk::GetRigidBodyLocationByName(rbName, __FP rbLS2WS, &rbIdx, NULL, NULL, &buf_rb_mks)) {

						std::vector<glm::fvec3> pos_rb_mks(buf_rb_mks.size() / 3);
						memcpy(&pos_rb_mks[0], &buf_rb_mks[0], sizeof(float) * buf_rb_mks.size());

						glm::fmat4x4 rbWS2LS = glm::inverse(rbLS2WS);
						for (int i = 0; i < (int)pos_rb_mks.size(); i++)
							pos_rb_mks[i] = vzmutils::transformPos(pos_rb_mks[i], rbWS2LS);

						// compute center pos
						glm::fvec3 v02 = pos_rb_mks[2] - pos_rb_mks[0];
						glm::fvec3 p02 = pos_rb_mks[0] + v02 * (float)(65.0 / 110.0);
						glm::fvec3 v13 = pos_rb_mks[3] - pos_rb_mks[1];
						glm::fvec3 p13 = pos_rb_mks[1] + v13 * (float)(40.41 / 90.82);
						glm::fvec3 p1234 = (p02 + p13) * 0.5f;

						// compute normal vec
						glm::fvec3 nrl1234 = glm::normalize(glm::cross(v02, v13));
						//fvec3 nrlLS = navihelpers::ComputeNormalVector(pos_rb_mks); // LS

						// compute face vec
						glm::fvec3 faceDir = glm::normalize(glm::cross(nrl1234, v02));

						// compute line 
						glm::fvec3 p_line1 = p1234 - nrl1234 * (float)(65.36 * 0.001); // meter

						std::vector<glm::fvec3> toolUserData = { p_line1, faceDir };
						__gc->g_rbLocalUserPoints[rbName] = toolUserData;

						cv::FileStorage fs(__gc->g_folder_trackingInfo + "tooltip_" + rbName + ".txt", cv::FileStorage::Mode::WRITE);
						cv::Mat ocv3(1, 3, CV_32FC1);
						memcpy(ocv3.ptr(), &p_line1, sizeof(glm::fvec3));
						fs << "ToolPos" << ocv3;
						memcpy(ocv3.ptr(), &faceDir, sizeof(glm::fvec3));
						fs << "ToolVec" << ocv3;
						fs.release();

						//__gc->g_toolTipPointLS = vzmutils::transformPos(selectedMks[selectedMks.size() - 1], rbWS2LS);
						//cv::FileStorage fs(__gc->g_folder_trackingInfo + "tooltip.txt", cv::FileStorage::Mode::WRITE);
						//cv::Mat ocv3(1, 3, CV_32FC1);
						//memcpy(ocv3.ptr(), &__gc->g_toolTipPointLS, sizeof(glm::fvec3));
						//fs << "ToolTip" << ocv3;
						//fs.release();

						// use cam direction...
						UpdateRigidBodiesInThread();
						optitrk::StoreProfile(__gc->g_profileFileName);

						__gc->g_optiEvent = OPTTRK_THREAD::FREE;

					}
				} break;
				case OPTTRK_THREAD::C_ARM_SHOT_MOMENT: {
					__gc->g_track_info_shotmoment = trk_info;
					__gc->g_is_ready_for_shotmoment = true;
					__gc->g_optiEvent = OPTTRK_THREAD::FREE;

				} break;
				case OPTTRK_THREAD::TOOL_PIVOT: {
					std::string rbName = __gc->g_optiToolId == 1 ? "t-needle" : "troca";
					bitset<128> rbCID;
					if (trk_info.GetRigidBodyByName(rbName, NULL, NULL, NULL, &rbCID)) {
						float progress, initErr, returnErr;
						switch (pivotState) {
						case 0:
						{
							cv::FileStorage config_fs(__gc->g_configFileName, cv::FileStorage::Mode::READ);
							std::string configLoad;
							config_fs["PivotSamples"] >> __gc->g_optiPivotSamples;
							std::cout << (int)__gc->g_optiPivotSamples << std::endl;
							config_fs.release();
						}
							if (optitrk::StartPivotSample(rbCID, __gc->g_optiPivotSamples)) pivotState = 1;
							break;
						case 1:
							curTime = clock();
							if ((double)(curTime - prevTime) / CLOCKS_PER_SEC > 0.01) {
								prevTime = curTime;
								if (optitrk::ProcessPivotSample(&progress, &initErr, &returnErr)) {
									__gc->g_optiPivotProgress = (int)progress;

									if (returnErr >= 0.f) {
										pivotState = 0;
										//__gc->g_optiEvent = OPTTRK_THREAD::FREE;
										__gc->g_optiEvent = OPTTRK_THREAD::TOOL_UPDATE;
										__gc->g_engineLogger->info("Pivoting Completed! InitErr: {} mm, RetErr: {}mm", initErr, returnErr);
									}
								}
								else {
									__gc->SetErrorCode("Pivoting Sample Error!");
								}
							}
							break;
						}
					}
				} break;
				case OPTTRK_THREAD::TOOL_RESET_PIVOT: {
					optitrk::ResetPivot();
					pivotState = 0;
					__gc->g_optiEvent = OPTTRK_THREAD::FREE;
					__gc->g_optiToolId = 0;
					__gc->SetErrorCode("Pivoting Canceled");
				} break;
				case OPTTRK_THREAD::FREE:
				default: break;
				}

				if (__gc->g_optiRecordMode == OPTTRK_RECMODE::RECORD) {

					if (__gc->g_optiRecordFrame == 0) {
						std::vector<std::string> toolNames = { "t-needle", "troca" };
						cv::FileStorage fs(__gc->g_folder_trackingInfo + "tooltipRecord.txt", cv::FileStorage::Mode::WRITE);
						for (int i = 0; i < 2; i++) {
							std::vector<glm::fvec3>& toolUserData = __gc->g_rbLocalUserPoints[toolNames[i]];
							if (toolUserData.size() == 2) {
								cv::Mat ocv3(1, 3, CV_32FC1);
								memcpy(ocv3.ptr(), &toolUserData[0], sizeof(glm::fvec3));
								fs << "ToolPos_" + to_string(i) << ocv3;
								memcpy(ocv3.ptr(), &toolUserData[1], sizeof(glm::fvec3));
								fs << "ToolVec_" + to_string(i) << ocv3;
							}
							//else {
							//	__gc->SetErrorCode("No Tool is Registered!");
							//	__gc->g_optiRecordMode = OPTTRK_RECMODE::NONE;
							//}
						}
						fs.release();

						__gc->g_recFileStream.open(__gc->g_recFileName);
						if (!__gc->g_recFileStream.is_open()) {
							__gc->SetErrorCode("Invalid Rec File!");
							__gc->g_optiRecordMode = OPTTRK_RECMODE::NONE;
						}
						std::vector<std::vector<std::string>> csvData(5);
						std::vector<std::string>& csvRow1 = csvData[0];
						std::vector<std::string>& csvRow2 = csvData[1];
						std::vector<std::string>& csvRow3 = csvData[2];
						std::vector<std::string>& csvRow4 = csvData[3];
						std::vector<std::string>& csvRow5 = csvData[4];
						csvRow1 = { "", "TYPE" };
						csvRow2 = { "", "Name" };
						csvRow3 = { "", "ID" };
						csvRow4 = { "", "" };
						csvRow5 = { "Frame", "Time" };

						for (int i = 0; i < 3; i++) {
							for (int j = 0; j < 16; j++) {
								csvRow1.push_back("Opti Camera");
								csvRow2.push_back("center cam");
								csvRow3.push_back("v120 " + to_string(i));
								csvRow4.push_back("Mat Cam2WS");
								csvRow5.push_back("mat[" + to_string(j) + "]");
							}
						}
						vector< map<string, map<track_info::MKINFO, std::any>>> rbMkSets(numAllFramesRBs);
						for (int i = 0; i < numAllFramesRBs; i++) {
							std::string rbName;
							//glm::fmat4x4 matLS2WS;
							//float mkMSE;
							map<string, map<track_info::MKINFO, std::any>>& rbmkSet = rbMkSets[i];
							std::bitset<128> cid;
							trk_info.GetRigidBodyByIdx(i, &rbName, NULL, NULL, &rbmkSet, &cid);

							const string cid_str = cid.to_string();
							bitset<64>bss1(cid_str.substr(0, cid_str.size() / 2));          // first half
							bitset<64>bss2(cid_str.substr(cid_str.size() / 2, cid_str.size())); // second half
							unsigned long long int bss1ll = bss1.to_ullong();
							unsigned long long int bss2ll = bss2.to_ullong();
							stringstream res;
							res << hex << bss2ll << hex << bss1ll;

							for (int j = 0; j < 7; j++) {
								csvRow1.push_back("Rigid Body");
								csvRow2.push_back(rbName);

								csvRow3.push_back(res.str());
								csvRow4.push_back(j < 4 ? "Quaternion" : "TVec");
								switch (j) {
								case 0: csvRow5.push_back("X"); break;
								case 1: csvRow5.push_back("Y"); break;
								case 2: csvRow5.push_back("Z"); break;
								case 3: csvRow5.push_back("W"); break;
								case 4: csvRow5.push_back("X"); break;
								case 5: csvRow5.push_back("Y"); break;
								case 6: csvRow5.push_back("Z"); break;
								}
							}

							csvRow1.push_back("Rigid Body");
							csvRow2.push_back(rbName);
							csvRow3.push_back(cid.to_string());
							csvRow4.push_back("Mean Marker Error");
							csvRow5.push_back("error");

							for (auto it = rbmkSet.begin(); it != rbmkSet.end(); it++) {
								for (int j = 0; j < 4; j++) {
									csvRow1.push_back("Rigid Body Marker");
									csvRow2.push_back(it->first);
									csvRow3.push_back(cid.to_string());
									csvRow4.push_back(j < 3 ? "Position" : "Quality");
									switch (j) {
									case 0: csvRow5.push_back("X"); break;
									case 1: csvRow5.push_back("Y"); break;
									case 2: csvRow5.push_back("Z"); break;
									case 3: csvRow5.push_back(""); break;
									}
								}
							}
						} // for (int i = 0; i < numAllFramesRBs; i++)

						// vector< map<string, map<track_info::MKINFO, std::any>>> rbMkSets(numAllFramesRBs);
						for (int i = 0; i < numAllFramesRBs; i++) {
							map<string, map<track_info::MKINFO, std::any>>& rbmkSet = rbMkSets[i];

							int mk_count = 0;
							for (auto it = rbmkSet.begin(); it != rbmkSet.end(); it++) {
								for (int j = 0; j < 3; j++) {
									csvRow1.push_back("Marker");
									csvRow2.push_back(it->first + " PC");
									csvRow3.push_back(to_string(i << 8 | mk_count++));
									csvRow4.push_back("Position");
									switch (j) {
									case 0: csvRow5.push_back("X"); break;
									case 1: csvRow5.push_back("Y"); break;
									case 2: csvRow5.push_back("Z"); break;
									}
								}
							}
						}

						// Write CSV data to the file
						for (int i = 0; i < (int)csvData.size(); i++) {
							std::vector<std::string>& csvRow = csvData[i];
							for (int j = 0; j < (int)csvRow.size(); j++) {
								__gc->g_recFileStream << csvRow[j];
								if (j < csvRow.size() - 1) {
									__gc->g_recFileStream << ',';
								}
								else
									__gc->g_recFileStream << '\n';
							}
						}
					}
					
					if (__gc->g_optiRecordFrame % __gc->g_optiRecordPeriod == 0) {

						std::vector<std::string> csvRow = { to_string(__gc->g_optiRecordFrame), to_string(timeBegin) };

						std::vector<std::pair<std::string, glm::fvec3>> rb_pcmks;
						glm::fmat4x4 matCam2WS;
						for (int i = 0; i < 3; i++) {
							trk_info.GetTrackingCamPose(i, matCam2WS);
							const float* matData = (const float*)glm::value_ptr(matCam2WS);
							for (int j = 0; j < 16; j++) {
								csvRow.push_back(to_string(matData[j]));
							}
						}
						vector< map<string, map<track_info::MKINFO, std::any>>> rbMkSets(numAllFramesRBs);
						for (int i = 0; i < numAllFramesRBs; i++) {
							string rbName;
							//glm::fmat4x4 matLS2WS;
							float mkMSE;
							map<string, map<track_info::MKINFO, std::any>>& rbmkSet = rbMkSets[i];
							trk_info.GetRigidBodyByIdx(i, &rbName, NULL, &mkMSE, &rbmkSet, NULL);
							glm::fquat q;
							glm::fvec3 tvec;
							trk_info.GetRigidBodyQuatTVecByName(rbName, &q, &tvec);
							const float* qData = (const float*)glm::value_ptr(q);
							const float* tData = (const float*)glm::value_ptr(tvec);
							for (int j = 0; j < 4; j++) {
								csvRow.push_back(to_string(qData[j]));
							}
							for (int j = 0; j < 3; j++) {
								csvRow.push_back(to_string(tData[j]));
							}
							csvRow.push_back(std::to_string(mkMSE));

							for (auto it = rbmkSet.begin(); it != rbmkSet.end(); it++) {
								glm::fvec3 posMk = any_cast<glm::fvec3>(it->second[track_info::MKINFO::POSITION]);
								float qualMk = any_cast<float>(it->second[track_info::MKINFO::MK_QUALITY]);

								csvRow.push_back(to_string(posMk.x));
								csvRow.push_back(to_string(posMk.y));
								csvRow.push_back(to_string(posMk.z));
								csvRow.push_back(to_string(qualMk));
							}
						} // for (int i = 0; i < numAllFramesRBs; i++)

						for (int i = 0; i < numAllFramesRBs; i++) {
							map<string, map<track_info::MKINFO, std::any>>& rbmkSet = rbMkSets[i];

							for (auto it = rbmkSet.begin(); it != rbmkSet.end(); it++) {
								glm::fvec3 posMk = any_cast<glm::fvec3>(it->second[track_info::MKINFO::POSITION_RBPC]);
								csvRow.push_back(to_string(posMk.x));
								csvRow.push_back(to_string(posMk.y));
								csvRow.push_back(to_string(posMk.z));
							}
						}

						// Write CSV data to the file
						for (int j = 0; j < csvRow.size(); j++) {
							__gc->g_recFileStream << csvRow[j];
							if (j < csvRow.size() - 1) {
								__gc->g_recFileStream << ',';
							}
							else 
								__gc->g_recFileStream << '\n';
						}
						__gc->g_optiRecordFrame++;
					}
					if (__gc->g_optiRecordFrame == 0) {
						__gc->g_optiRecordFrame = 1;
					}
				}
				else {
					recFinished();
				}
				// __gc->g_optiRecordMode != OPTTRK_RECMODE::LOAD
			} 

			if (optRbState != "")
				__gc->g_optiRbStates = optRbState;

			if (trk_info.NumMarkers() > 0 || trk_info.NumRigidBodies() > 0)
				__gc->g_track_que.push(trk_info);
		}
	}
}
