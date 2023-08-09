#include "trackingThread.h"

trackingThread::trackingThread(QObject* parent):
	QThread(parent)
{
	m_tracker_alive = true;
}


trackingThread::~trackingThread()
{

	this->wait();
}


void trackingThread::run()
{
	using namespace std;
	using namespace navihelpers;
	using namespace glm;

	std::vector<std::string> rbNames;
	int numAllFramesRBs = optitrk::GetRigidBodies(&rbNames);
	while (m_tracker_alive)
	{
		//Sleep(postpone);
		optitrk::UpdateFrame();

		track_info trk_info;
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
					pt[track_info::MKINFO::CID] = rbCid;
					pt[track_info::MKINFO::POSITION] = mkPts[j];
					pt[track_info::MKINFO::MK_NAME] = mkName;
					pt[track_info::MKINFO::MK_QUALITY] = mkQualities[j];
				}
				trk_info.AddRigidBody(rbName, matLS2WS, qvec, tvec, rbMSE, rbmkSet);
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
		m_track_que.push(trk_info);
	}

}