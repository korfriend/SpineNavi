#include "TrackingTask.h"

#include "VisMtvApi.h"
#include "ApiUtility.hpp"

#include "../optitrk/optitrk.h"

#include "rapidcsv/rapidcsv.h"

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

		cv::FileStorage fs(__gc->g_folder_trackingInfo + "tooltip.txt", cv::FileStorage::Mode::READ);
		if (fs.isOpened()) {
			cv::Mat ocv3(1, 3, CV_32FC1);
			fs["ToolTip"] >> ocv3;
			memcpy(&__gc->g_toolTipPointLS, ocv3.ptr(), sizeof(glm::fvec3));
		}
		fs.release();

		std::time_t timeBegin;
		std::vector<std::string> rbNames;

		rapidcsv::Document csvTrkData;
		std::vector<std::string> rowTypes, rowNames, rowIds, rowDataType;
		int numCols, numRows;

		while (__gc->g_tracker_alive)
		{
			track_info trk_info;
			timeBegin = std::time(NULL);
			
			auto recFinished = []() {
				if (__gc->g_recFileStream.is_open()) {
					std::cout << "CSV data has been written to " << __gc->g_recFileName << std::endl;
					__gc->g_recFileStream.close();
				}
				__gc->g_optiRecordFrame = 0;
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

			if (__gc->g_optiRecordMode == OPTTRK_RECMODE_LOAD) {

				recFinished();

				if (__gc->g_optiRecordFrame == 0) {
					csvTrkData.Load(__gc->g_recFileName, rapidcsv::LabelParams(0, 0));

					rowTypes = csvTrkData.GetRow<std::string>(0);
					rowNames = csvTrkData.GetRow<std::string>(1);
					rowIds = csvTrkData.GetRow<std::string>(2);
					rowDataType = csvTrkData.GetRow<std::string>(3);

					numCols = (int)csvTrkData.GetColumnCount();
					numRows = (int)csvTrkData.GetRowCount();

					// get rbNames 
					std::set<std::string> rbNamesChecker;
					for (int i = 0; i < numCols; i++) {
						if (rowTypes[i] == "Rigid Body") {
							string rbName = rowNames[i];
							if (rbNamesChecker.find(rbName) == rbNamesChecker.end()) {
								rbNames.push_back(rbName);
							}
						}
					}
				}

				const int frameRowIdx = 5;

				std::vector<std::string> rowData = csvTrkData.GetRow<std::string>((int)frameRowIdx + (int)__gc->g_optiRecordFrame);

				glm::fmat4x4 matData;
				float* pmat = glm::value_ptr(matData);
				glm::fvec3 posData;
				float* ppos = glm::value_ptr(posData);
				glm::fquat qtData;
				float* pqt = glm::value_ptr(qtData);

				//int numAllFramesRBs = (int)rbNames.size();
				vector< map<string, map<track_info::MKINFO, std::any>>> rbMkSets;// (numAllFramesRBs);
				for (int i = 0; i < numCols; i++) {
					if (rowTypes[i] == "Opti Camera") {
						for (int j = 0; j < 16; j++, i++) {
							float fv = std::stof(rowData[j]);
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

						float mkMSE = std::stof(rowData[i++]);
						std::map<std::string, std::map<track_info::MKINFO, std::any>> rbmkSet;
						while (rowTypes[i] == "Rigid Body Marker") {
							string mkName = rowNames[i];
							bitset<128> mk_cid = string2cid(rowIds[i]);
							glm::fvec3 pos;
							float quality;
							for (int j = 0; j < 4; j++) {
								float fv = std::stof(rowData[i++]);
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

							rbMkSets.push_back(rbmkSet);
						}

						// 저장 시 tvec, quaternion 버전으로..
						trk_info.AddRigidBody(rbName, cid, matLS2WS, qtData, posData, mkMSE, rbmkSet);
					} // else if (rowTypes[i] == "Rigid Body")
					else if (rowTypes[i] == "Marker") {
						for (int j = 0; j < (int)rbMkSets.size(); j++) {
							std::map<std::string, std::map<track_info::MKINFO, std::any>>& rbmkSet = rbMkSets[j];
							int mk_count = 0;
							for (auto it = rbmkSet.begin(); it != rbmkSet.end(); it++) {
								std::bitset<128> cid = j << 8 | mk_count++;
								std::string mkName = any_cast<std::string>(it->second[track_info::MKINFO::MK_NAME]);
								glm::fvec3 mkPos = any_cast<glm::fvec3>(it->second[track_info::MKINFO::POSITION]);
								trk_info.AddMarker(cid, mkPos, mkName);
							}
						}
					}
					i--;
				} // for (int i = 0; i < numCols; i++)

				// __gc->g_optiRecordMode == OPTTRK_RECMODE_LOAD

				__gc->g_optiRecordFrame++;
			}
			else {	// __gc->g_optiRecordMode != OPTTRK_RECMODE_LOAD

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
					string rbName;// = rbNames[i];
					fmat4x4 matLS2WS;
					float rbMSE;
					vector<float> posMKs;
					vector<float> posPCMKs;
					vector<float> mkQualities;
					bitset<128> rbCid;
					fquat qvec;
					fvec3 tvec;
					if (optitrk::GetRigidBodyLocationByIdx(i, (float*)&matLS2WS, &rbCid, &rbMSE, &posMKs, &posPCMKs, NULL, &mkQualities, &rbName, (float*)&qvec, (float*)&tvec)) {

						if (rbMSE > 0.01) cout << "\n" << "problematic error!! " << rbName << " : " << rbMSE << endl;

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
						cout << "\nregister tool success!" << endl;
						int numSelectedMKs = (int)__gc->g_selectedMkNames.size();
						optitrk::SetRigidBody("tool", numSelectedMKs, (float*)&selectedMks[0]);

						__gc->g_optiEvent = OPTTRK_THREAD_TOOL_UPDATE;
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

				if (__gc->g_optiRecordMode == OPTTRK_RECMODE_RECORD) {

					if (__gc->g_optiRecordFrame == 0) {

						__gc->g_recFileStream.open(__gc->g_recFileName);
						if (!__gc->g_recFileStream.is_open()) {
							__gc->SetErrorCode(ERROR_CODE_INVALID_RECFILE);
							__gc->g_optiRecordMode = OPTTRK_RECMODE_NONE;
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
						for (int i = 0; i < 16; i++) {
							csvRow1.push_back("Opti Camera");
							csvRow2.push_back("center cam");
							csvRow3.push_back("v120 1");
							csvRow4.push_back("Mat Cam2WS");
							csvRow5.push_back("mat[" + to_string(i) + "]");
						}
						vector< map<string, map<track_info::MKINFO, std::any>>> rbMkSets(numAllFramesRBs);
						for (int i = 0; i < numAllFramesRBs; i++) {
							std::string rbName;
							//glm::fmat4x4 matLS2WS;
							//float mkMSE;
							map<string, map<track_info::MKINFO, std::any>>& rbmkSet = rbMkSets[i];
							std::bitset<128> cid;
							trk_info.GetRigidBodyByIdx(i, &rbName, NULL, NULL, &rbmkSet, &cid);
							for (int j = 0; j < 7; j++) {
								csvRow1.push_back("Rigid Body");
								csvRow2.push_back(rbName);
								csvRow3.push_back(cid.to_string());
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
						for (int i = 0; i < 4; i++) {
							std::vector<std::string>& csvRow = csvData[i];
							for (int j = 0; j < (int)csvRow.size(); j++) {
								__gc->g_recFileStream << csvRow[j];
								if (j < csvRow.size() - 1) {
									__gc->g_recFileStream << ',';
								}
								__gc->g_recFileStream << '\n';
							}
						}
					}
					
					if (__gc->g_optiRecordFrame % 20 == 0) {

						std::vector<std::string> csvRow = { to_string(__gc->g_optiRecordFrame), to_string(timeBegin) };

						std::vector<std::pair<std::string, glm::fvec3>> rb_pcmks;
						glm::fmat4x4 matCam2WS;
						trk_info.GetTrackingCamPose(1, matCam2WS);
						const float* matData = (const float*)glm::value_ptr(matCam2WS);
						for (int i = 0; i < 16; i++) {
							csvRow.push_back(to_string(matData[i]));
						}
						for (int i = 0; i < numAllFramesRBs; i++) {
							string rbName;
							//glm::fmat4x4 matLS2WS;
							float mkMSE;
							map<string, map<track_info::MKINFO, std::any>> rbmkSet;
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

						// to do : add markers...
						//int numMks = trk_info.NumMarkers();
						//for (int i = 0; i < numMks; i++) {
						//	std::map<track_info::MKINFO, std::any> mk;
						//	trk_info.GetMarkerByIdx(i, mk);
						//	//mk[track_info::MKINFO::]
						//}

						// Write CSV data to the file
						for (int j = 0; j < csvRow.size(); j++) {
							__gc->g_recFileStream << csvRow[j];
							if (j < csvRow.size() - 1) {
								__gc->g_recFileStream << ',';
							}
							__gc->g_recFileStream << '\n';
						}

					}
					__gc->g_optiRecordFrame++;
				}
				else {
					recFinished();
				}
				// __gc->g_optiRecordMode != OPTTRK_RECMODE_LOAD
			} 
			__gc->g_track_que.push(trk_info);
		}
	}
}
