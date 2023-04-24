#pragma once

#include <iostream>
#include <windows.h>

#include <any>
#include <map>
#include <queue>
#include <bitset>
#include <string>
#include <sstream>
#include <iomanip>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <opencv2/opencv.hpp>

#define __cv3__ *(glm::fvec3*)
#define __cv4__ *(glm::fvec4*)
#define __cm4__ *(glm::fmat4x4*)
#define __PR(A, INTERVAL) A[0] << INTERVAL << A[1] << INTERVAL << A[2]

namespace navihelpers {
	using namespace std;
	using namespace cv;

	glm::fvec3 tr_pt(const glm::fmat4x4& mat, const glm::fvec3& p)
	{
		glm::fvec4 _p(p, 1.f);
		_p = mat * _p;
		return glm::fvec3(_p.x / _p.w, _p.y / _p.w, _p.z / _p.w);
	}

	glm::fvec3 tr_vec(const glm::fmat4x4& mat, const glm::fvec3& v)
	{
		glm::fmat3x3 r = mat;
		glm::fvec3 _v = r * v;
		return _v;
	}

	glm::fvec3 tr_vec2(const glm::fmat4x4& mat, const glm::fvec3& v)
	{
		glm::fvec3 _p0 = tr_pt(mat, glm::fvec3(0));
		glm::fvec3 _p1 = tr_pt(mat, glm::fvec3(0) + v);

		return _p1 - _p0;
	}

	// http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
	std::vector<std::string> split(const std::string& text, char sep) {
		std::vector<std::string> tokens;
		int start = 0, end = 0;
		while ((end = text.find(sep, start)) != std::string::npos) {
			tokens.push_back(text.substr(start, end - start));
			start = end + 1;
		}
		tokens.push_back(text.substr(start));
		return tokens;
	}
	// https://stackoverflow.com/questions/9150538/how-do-i-tostring-an-enum-in-c
#define ENUM(name, ...)\
enum name \
{\
__VA_ARGS__\
};\
std::vector<std::string> name##Map = split(#__VA_ARGS__, ',');\
    std::string EtoString(const name v) { return name##Map.at(v);}


#include <mutex>
#include <thread>
// https://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
	template<typename Data>
	class concurrent_queue // single consumer
	{
	private:
		std::queue<Data> the_queue;
		mutable std::mutex the_mutex;

		std::condition_variable the_condition_variable;
		int MAX_QUEUE;

	public:
		concurrent_queue(int cap = 10)
		{
			MAX_QUEUE = cap;
		}

		void wait_for_data()
		{
			std::lock_guard<std::mutex> lock(the_mutex);
			while (the_queue.empty())
			{
				the_condition_variable.wait(lock);
			}
		}

		void wait_and_pop(Data& popped_value)
		{
			std::lock_guard<std::mutex> lock(the_mutex);
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
			std::lock_guard<std::mutex> lock(the_mutex);
			return the_queue.empty();
		}

		Data& front()
		{
			std::lock_guard<std::mutex> lock(the_mutex);
			return the_queue.front();
		}

		Data const& front() const
		{
			std::lock_guard<std::mutex> lock(the_mutex);
			return the_queue.front();
		}

		void pop() // deque
		{
			std::lock_guard<std::mutex> lock(the_mutex);
			the_queue.pop();
		}
	};

	template<size_t sz> struct bitset_comparer {
		bool operator() (const bitset<sz>& b1, const bitset<sz>& b2) const {
			return b1.to_ulong() < b2.to_ulong();
		}
	};

	struct track_info2
	{
		// key : rigidbody name
	public:
		enum MKINFO : int
		{
			CID = 0, // std::bitset<128>
			POSITION = 1, // glm::fvec3, world space
			MK_NAME = 2, // string
			MK_QUALITY = 3 // float // only for RB_MKSET
		};
	private:
		enum RBINFO : int
		{
			LS2WS = 0, // glm::fmat4x4
			MK_MSE = 1, // float
			RB_MKSET = 2, // map<string, map<MKINFO, any>>
		};

		std::map<string, map<RBINFO, std::any>> __rbinfo;
		std::map<std::bitset<128>, map<MKINFO, std::any>, bitset_comparer<128>> __mksByCid;
		std::map<string, std::bitset<128>> __mksByName;
	public:
		track_info2() { }

		bool GetRigidBody(const string& rbName, glm::fmat4x4* matLS2WS, float* mkMSE, map<string, map<MKINFO, std::any>>* rbmkSet) {
			auto it = __rbinfo.find(rbName);
			if (it == __rbinfo.end()) return false;
			auto& v = it->second;
			if (matLS2WS)
				*matLS2WS = std::any_cast<glm::fmat4x4>(v[LS2WS]);
			if (mkMSE)
				*mkMSE = std::any_cast<float>(v[MK_MSE]);
			if (rbmkSet)
				*rbmkSet = std::any_cast<map<string, map<MKINFO, std::any>>>(v[RB_MKSET]);
			return true;
		}

		int NumRigidBodies() {
			return (int)__rbinfo.size();
		}

		bool GetRigidBodyByIdx(const int rbIdx, string* rbName, glm::fmat4x4* matLS2WS, float* mkMSE, map<string, map<MKINFO, std::any>>* rbmkSet) {
			auto it = __rbinfo.begin();
			std::advance(it, rbIdx);

			if (it == __rbinfo.end()) return false;

			auto& v = it->second;
			if (rbName)
				*rbName = it->first;
			if (matLS2WS)
				*matLS2WS = std::any_cast<glm::fmat4x4>(v[LS2WS]);
			if (mkMSE)
				*mkMSE = std::any_cast<float>(v[MK_MSE]);
			if (rbmkSet)
				*rbmkSet = std::any_cast<map<string, map<MKINFO, std::any>>>(v[RB_MKSET]);

			return true;
		}

		void AddRigidBody(const string& rbName, const glm::fmat4x4& matLS2WS, const float mkMSE,
			const map<string, map<MKINFO, std::any>>& rbmkSet)
		{
			map<RBINFO, std::any>& v = __rbinfo[rbName];
			v[LS2WS] = matLS2WS;
			v[MK_MSE] = mkMSE;
			v[RB_MKSET] = rbmkSet;
		}

		bool GetMarkerByName(const string& mkName, map<MKINFO, std::any>& mk)
		{
			auto it = __mksByName.find(mkName);
			if (it == __mksByName.end()) return false;
			return GetMarkerByCID(it->second, mk);
		}

		bool GetMarkerByCID(const std::bitset<128>& cid, map<MKINFO, std::any>& mk)
		{
			auto it = __mksByCid.find(cid);
			if (it == __mksByCid.end()) return false;
			mk = it->second;
			return true;
		}

		int NumMarkers() {
			return (int)__mksByName.size();
		}

		bool GetMarkerByIdx(const int mkIdx, map<MKINFO, std::any>& mk)
		{
			auto it = __mksByCid.begin();
			std::advance(it, mkIdx);

			if (it == __mksByCid.end()) return false;

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

		void AddMarker(const std::bitset<128>& cid, const glm::fvec3& mkPos, const string& mkName)
		{
			map<MKINFO, std::any>& v = __mksByCid[cid];
			v[CID] = cid;
			v[POSITION] = mkPos;
			v[MK_NAME] = mkName;

			__mksByName[mkName] = cid;
		}
	};

	struct track_info
	{
		//  "rs_cam" , "probe" , "ss_tool_v1" , "ss_head" , "breastbody" 
		std::map<string, pair<bool, glm::fmat4x4>> map_ls2ws;

		bool GetLFrmInfo(const string& name, glm::fmat4x4& mat_lfrm2ws) const
		{
			bool is_detected = false;
			auto it = map_ls2ws.find(name);
			if (it != map_ls2ws.end())
			{
				is_detected = get<0>(it->second);
				mat_lfrm2ws = get<1>(it->second);
			}
			return is_detected;
		}

		void SetLFrmInfo(const string& name, const bool is_detected, const glm::fmat4x4& mat_lfrm2ws)
		{
			map_ls2ws[name] = pair<bool, glm::fmat4x4>(is_detected, mat_lfrm2ws);
		}
		// when changing this, please consider the serialization
		//"rs_cam"
		//glm::fmat4x4 mat_rbcam2ws;
		//bool is_detected_rscam;
		//
		////"probe"
		//glm::fmat4x4 mat_probe2ws;
		//bool is_detected_probe;
		//
		////"ss_tool_v1"
		//glm::fmat4x4 mat_tfrm2ws;
		//bool is_detected_sstool;
		//
		////"ss_head"
		//glm::fmat4x4 mat_headfrm2ws;
		//bool is_detected_sshead;
		//
		////"breastbody"
		//glm::fmat4x4 mat_bodyfrm2ws;
		//bool is_detected_brbody;

		std::vector<float> mk_xyz_list;
		std::vector<float> mk_residue_list;
		std::vector<std::bitset<128>> mk_cid_list;

		bool is_updated;
		track_info() { is_updated = false; }

		bool GetProbePinPoint(glm::fvec3& pos)
		{
			glm::fmat4x4 mat_lfrm2ws;
			if (GetLFrmInfo("probe", mat_lfrm2ws))
			{
				pos = tr_pt(mat_lfrm2ws, glm::fvec3(0));
				return true;
			}
			return false;
		}

		glm::fvec3 GetMkPos(int idx)
		{
			if (idx < 0 || idx >= mk_xyz_list.size() / 3) return glm::fvec3(0);
			return ((glm::fvec3*)&mk_xyz_list[0])[idx];
		}

		char* GetSerialBuffer(size_t& bytes_size)
		{
			int num_mks = (int)mk_xyz_list.size();
			int num_lfrms = (int)map_ls2ws.size();
			bytes_size = (sizeof(glm::fmat4x4) + sizeof(bool) + sizeof(char) * 100) * num_lfrms + (sizeof(float) * 2 + (128 / 8)) * num_mks + 4 * 2; // last 4 * 2 means num_mks, num_lfrms
			char* buf = new char[bytes_size];
			memset(buf, 0, bytes_size);
			*(int*)&buf[0] = num_mks;
			*(int*)&buf[4] = num_lfrms;

			memcpy(&buf[8], &mk_xyz_list[0], sizeof(float) * num_mks);
			memcpy(&buf[8 + sizeof(float) * num_mks], &mk_residue_list[0], sizeof(float) * num_mks);
			memcpy(&buf[8 + sizeof(float) * num_mks * 2], &mk_cid_list[0], (128 / 8) * num_mks);

			int offset = 8 + sizeof(float) * num_mks * 2 + (128 / 8) * num_mks;
			int unit_size = sizeof(bool) + sizeof(glm::fmat4x4) + sizeof(char) * 100;
			int i = 0;
			for (auto it = map_ls2ws.begin(); it != map_ls2ws.end(); it++, i++)
			{
				memcpy(&buf[offset + unit_size * i], it->first.c_str(), sizeof(char) * it->first.length());
				*(bool*)&buf[offset + unit_size * i + 100] = get<0>(it->second);
				*(glm::fmat4x4*)&buf[offset + unit_size * i + 100 + sizeof(bool)] = get<1>(it->second);
			}

			return buf;
		}

		void SetFromSerialBuffer(const char* buf)
		{
			int num_mks = *(int*)buf[0];
			int num_lfrms = *(int*)buf[4];
			mk_xyz_list.assign(num_mks, 0);
			mk_residue_list.assign(num_mks, 0);
			mk_cid_list.assign(num_mks, 0);

			memcpy(&mk_xyz_list[0], &buf[8], sizeof(float) * num_mks);
			memcpy(&mk_residue_list[0], &buf[8 + sizeof(float) * num_mks], sizeof(float) * num_mks);
			memcpy(&mk_cid_list[0], &buf[8 + sizeof(float) * num_mks * 2], (128 / 8) * num_mks);

			int offset = 8 + sizeof(float) * num_mks * 2 + (128 / 8) * num_mks;
			int unit_size = sizeof(bool) + sizeof(glm::fmat4x4) + sizeof(char) * 100;
			for (int i = 0; i < num_lfrms; i++)
			{
				char arry[100];
				memcpy(arry, &buf[offset + unit_size * i], sizeof(char) * 100);
				string name = arry;
				bool is_detected = *(bool*)&buf[offset + unit_size * i + 100];
				glm::fmat4x4 mat_lfrm2ws = *(glm::fmat4x4*)&buf[offset + unit_size * i + 100 + sizeof(bool)];
				SetLFrmInfo(name, is_detected, mat_lfrm2ws);
			}
		}

		bool CheckExistCID(const std::bitset<128>& cid, int* mk_idx = NULL)
		{
			bool exist_mk_cid = false;
			if (mk_idx) *mk_idx = -1;
			for (int i = 0; i < (int)mk_cid_list.size(); i++)
			{
				if (mk_cid_list[i] == cid)
				{
					exist_mk_cid = true;
					if (mk_idx) *mk_idx = i;
					break;
				}
			}

			return exist_mk_cid;
		}
	};

	void Make_Viewport(std::vector<glm::fvec3>& pos_tris, std::vector<glm::fvec3>& clr_tris, std::vector<glm::fvec3>& pos_lines, std::vector<glm::fvec3>& clr_lines,
		const glm::fvec3 clr_tri, const glm::fvec3 clr_line,
		const glm::fvec3 pos_c, const glm::fvec3 view, const glm::fvec3 up, const float fov_hor, const float fov_ver, const float max_length)
	{
		glm::fvec3 right = glm::cross(view, up); // x

		glm::fvec3 pos_e = pos_c + view * max_length;
		// up
		float yhalf = tan(fov_ver / 2.f) * max_length;
		float xhalf = tan(fov_hor / 2.f) * max_length;
		glm::fvec3 vec_y = up * yhalf;
		glm::fvec3 vec_x = right * yhalf;

		glm::fvec3 pos_0 = pos_c;
		glm::fvec3 pos_1 = pos_e + vec_x + vec_y;
		glm::fvec3 pos_2 = pos_e - vec_x + vec_y;
		glm::fvec3 pos_3 = pos_e + vec_x - vec_y;
		glm::fvec3 pos_4 = pos_e - vec_x - vec_y;

		pos_tris.push_back(pos_0);
		pos_tris.push_back(pos_3);
		pos_tris.push_back(pos_1);

		pos_tris.push_back(pos_0);
		pos_tris.push_back(pos_1);
		pos_tris.push_back(pos_2);

		pos_tris.push_back(pos_0);
		pos_tris.push_back(pos_2);
		pos_tris.push_back(pos_4);

		pos_tris.push_back(pos_0);
		pos_tris.push_back(pos_4);
		pos_tris.push_back(pos_3);

		pos_tris.push_back(pos_1);
		pos_tris.push_back(pos_3);
		pos_tris.push_back(pos_2);
		pos_tris.push_back(pos_2);
		pos_tris.push_back(pos_3);
		pos_tris.push_back(pos_4);

		pos_lines.push_back(pos_0);
		pos_lines.push_back(pos_1);
		pos_lines.push_back(pos_0);
		pos_lines.push_back(pos_2);
		pos_lines.push_back(pos_0);
		pos_lines.push_back(pos_3);
		pos_lines.push_back(pos_0);
		pos_lines.push_back(pos_4);

		pos_lines.push_back(pos_1);
		pos_lines.push_back(pos_2);
		pos_lines.push_back(pos_2);
		pos_lines.push_back(pos_4);
		pos_lines.push_back(pos_4);
		pos_lines.push_back(pos_3);
		pos_lines.push_back(pos_3);
		pos_lines.push_back(pos_1);

		for (int i = 0; i < pos_tris.size(); i++)
			clr_tris.push_back(clr_tri);
		for (int i = 0; i < pos_lines.size(); i++)
			clr_lines.push_back(clr_line);
	}

	void Cam_Gen(const glm::vec3& pos_cam_ws, const glm::vec3& view_ws, const glm::vec3& up_ws, const string cam_label, int& cam_tris_is, int& cam_lines_id, int& cam_label_id)
	{
		std::vector<glm::fvec3> pos_tris, pos_lines, clr_tris, clr_lines;
		Make_Viewport(pos_tris, clr_tris, pos_lines, clr_lines, glm::fvec3(1, 0.3, 0.5), glm::fvec3(1, 0.3, 0.5),
			pos_cam_ws, view_ws, up_ws, 3.141592654f * 56.f / 180.f, 3.141592654f * 46.f / 180.f, 0.1f);
		vzm::GenerateTrianglesObject((float*)&pos_tris[0], (float*)&clr_tris[0], (int)pos_tris.size() / 3, cam_tris_is);
		vzm::GenerateLinesObject((float*)&pos_lines[0], (float*)&clr_lines[0], (int)pos_lines.size() / 2, cam_lines_id);

		std::vector<glm::fvec3> pinfo(3);
		pinfo[0] = pos_lines[5];
		pinfo[1] = -view_ws;
		pinfo[2] = up_ws;
		vzm::GenerateTextObject((float*)&pinfo[0], cam_label, 0.05, true, false, cam_label_id);
	}

	void Cam_Gen(const glm::fmat4x4& mat_cs2ws, const string cam_label, int& cam_tris_is, int& cam_lines_id, int& cam_label_id)
	{
		glm::fvec3 pos_cam_ws = tr_pt(mat_cs2ws, glm::fvec3());
		glm::fvec3 view_ws = tr_vec(mat_cs2ws, glm::fvec3(0, 0, -1));
		glm::fvec3 up_ws = tr_vec(mat_cs2ws, glm::fvec3(0, 1, 0));
		Cam_Gen(pos_cam_ws, view_ws, up_ws, cam_label, cam_tris_is, cam_lines_id, cam_label_id);
	}

	void World_GridAxis_Gen(int& coord_grid_obj_id, int& axis_lines_obj_id, int& axis_texX_obj_id, int& axis_texZ_obj_id)
		// make world coordinate grid objects
	{
		std::vector<glm::fvec3> xyz_coord_grid;
		std::vector<glm::fvec3> clr_coord_grid;
		std::vector<float> r_coord_grid;
		int count_cyls = 0;
		double cyl_interval = 0.1;
		for (int i = 0; i < 200; i++)
		{
			xyz_coord_grid.push_back(glm::fvec3(-10, 0, -10 + i * cyl_interval));
			xyz_coord_grid.push_back(glm::fvec3(10, 0, -10 + i * cyl_interval));

			xyz_coord_grid.push_back(glm::fvec3(-10 + i * cyl_interval, 0, -10));
			xyz_coord_grid.push_back(glm::fvec3(-10 + i * cyl_interval, 0, 10));

			glm::fvec3 _color(0.3, 0.3, 0.3);
			float _r = 0.002f;
			if (i % 5 == 0)
			{
				_color = glm::fvec3(0.8, 0.8, 0.8);
				_r = 0.003f;
			}
			clr_coord_grid.push_back(_color);
			clr_coord_grid.push_back(_color);
			r_coord_grid.push_back(_r);
			clr_coord_grid.push_back(_color);
			clr_coord_grid.push_back(_color);
			r_coord_grid.push_back(_r);
		}
		//vzm::GenerateCylindersObject((float*)&pos_cyl_coord_plane[0], (float*)&r_cyl_coord_plane[0], (float*)&clr_cyl_coord_plane[0], (int)r_cyl_coord_plane.size(), coord_plane_obj2);
		vzm::GenerateLinesObject((float*)&xyz_coord_grid[0], (float*)&clr_coord_grid[0], (int)xyz_coord_grid.size() / 2, coord_grid_obj_id);

		std::vector<glm::fvec3> xyz_axis_lines;
		std::vector<glm::fvec3> clr_axis_lines;
		std::vector<float> r_axis_lines;
		xyz_axis_lines.push_back(glm::fvec3());
		xyz_axis_lines.push_back(glm::fvec3(0.1, 0, 0));
		xyz_axis_lines.push_back(glm::fvec3());
		xyz_axis_lines.push_back(glm::fvec3(0, 0.1, 0));
		xyz_axis_lines.push_back(glm::fvec3());
		xyz_axis_lines.push_back(glm::fvec3(0, 0, 0.1));
		clr_axis_lines.push_back(glm::fvec3(1, 0, 0));
		clr_axis_lines.push_back(glm::fvec3(0, 1, 0));
		clr_axis_lines.push_back(glm::fvec3(0, 0, 1));
		r_axis_lines.push_back(0.007f);
		r_axis_lines.push_back(0.007f);
		r_axis_lines.push_back(0.007f);
		r_axis_lines.push_back(0.007f);
		//vzm::GenerateLinesObject((float*)&xyz_axis_lines[0], (float*)&clr_axis_lines[0], (int)clr_axis_lines.size(), axis_lines_obj_id);
		vzm::GenerateCylindersObject((float*)&xyz_axis_lines[0], (float*)&r_axis_lines[0], (float*)&clr_axis_lines[0], (int)clr_axis_lines.size(), axis_lines_obj_id);

		std::vector<glm::fvec3> pinfo(3);
		pinfo[0] = glm::fvec3(0.10, 0, 0);
		pinfo[1] = glm::fvec3(0, -1, 0);
		pinfo[2] = glm::fvec3(1, 0, 0);
		vzm::GenerateTextObject((float*)&pinfo[0], "X", 0.1, true, false, axis_texX_obj_id);
		pinfo[0] = glm::fvec3(0, 0, 0.10);
		pinfo[1] = glm::fvec3(0, -1, 0);
		pinfo[2] = glm::fvec3(0, 0, 1);
		vzm::GenerateTextObject((float*)&pinfo[0], "Z", 0.1, true, false, axis_texZ_obj_id);
	}

	void ComputeClosestPointBetweenLineAndPoint(const glm::fvec3& pos_line, const glm::fvec3& dir_line, const glm::fvec3& pos_point, glm::fvec3& pos_closest_point)
	{
		float len = glm::length(dir_line);
		if (len <= 0.000001f) return;
		//http://math.stackexchange.com/questions/748315/finding-the-coordinates-of-a-point-on-a-line-that-produces-the-shortest-distance
		float t = ((pos_point.x * dir_line.x + pos_point.y * dir_line.y + pos_point.z * dir_line.z) - (pos_line.x * dir_line.x + pos_line.y * dir_line.y + pos_line.z * dir_line.z)) / len;
		pos_closest_point = pos_line + dir_line * t;
	}

	template <typename T>
	std::string to_string_with_precision(const T a_value, const int n = 6)
	{
		std::ostringstream out;
		out << std::setprecision(n) << a_value;
		return out.str();
	}

	//void MakeTrackeffect(const int track_fade_num, const glm::fvec3& pos_dst, int& track_spheres_id, std::vector<glm::fvec3>& track_points)
	void MakeTrackeffect(const int track_fade_num, const float min_move_dist, const glm::fvec3& pos_dst, int& track_spheres_id)
	{
		//const int track_fade_num = 100;
		//static int track_spheres_id = 0;
		static std::vector<glm::fvec3> track_points;
		if (track_points.size() == 0 || glm::length(track_points[0] - pos_dst) > min_move_dist)
		{
			if (track_points.size() < track_fade_num)
			{
				track_points.push_back(pos_dst);
			}
			else
			{
				memcpy(&track_points[0], &track_points[1], sizeof(glm::fvec3) * (track_fade_num - 1));
				track_points[track_fade_num - 1] = pos_dst;
			}
			std::vector<glm::fvec4> trackspheres(track_points.size());
			for (int i = 0; i < (int)track_points.size(); i++)
			{
				trackspheres[i] = glm::fvec4(track_points[i], 0.001f + i * 0.005f / track_fade_num);
			}
			vzm::GenerateSpheresObject(__FP trackspheres[0], NULL, track_points.size(), track_spheres_id);
			//vzm::ObjStates obj_state_track_spheres;
			//vzm::ReplaceOrAddSceneObject(0, track_spheres_id, obj_state_track_spheres);
		}
	}

	int GL_CLR_CHANNELS = 4;
	void copy_back_ui_buffer(unsigned char* data_ui, unsigned char* data_render_bf, int w, int h, bool v_flib)
	{
		// cpu mem ==> dataPtr
		int width_uibuf_pitch = w * 3;
		int width_fbbuf_pitch = w * GL_CLR_CHANNELS;
#pragma omp parallel for 
		for (int i = 0; i < h; i++)
			for (int j = 0; j < w; j++)
			{
				int y = v_flib ? (h - 1 - i) : i;

				unsigned int rgba;
				memcpy(&rgba, &data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 0], sizeof(int));

				//unsigned char r = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 0];
				//unsigned char g = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 1];
				//unsigned char b = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 2];
				//unsigned char a = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 3];

				unsigned char r = rgba & 0xFF;
				unsigned char g = (rgba >> 8) & 0xFF;
				unsigned char b = (rgba >> 16) & 0xFF;
				unsigned char a = (rgba >> 24) & 0xFF;

				if (a > 0)
				{
					unsigned int rgb;
					memcpy(&rgb, &data_ui[i * width_uibuf_pitch + j * 3 + 0], 3);

					//unsigned char _r = data_ui[i * width_uibuf_pitch + j * 3 + 0];
					//unsigned char _g = data_ui[i * width_uibuf_pitch + j * 3 + 1];
					//unsigned char _b = data_ui[i * width_uibuf_pitch + j * 3 + 2];

					unsigned char _r = rgb & 0xFF;
					unsigned char _g = (rgb >> 8) & 0xFF;
					unsigned char _b = (rgb >> 16) & 0xFF;

					float fa = (float)a / 255.f;
					float fr = (1.f - fa) * (float)_r + (float)r * fa;
					float fg = (1.f - fa) * (float)_g + (float)g * fa;
					float fb = (1.f - fa) * (float)_b + (float)b * fa;

					//data_ui[i * width_uibuf_pitch + j * 3 + 0] = (unsigned char)min((int)fr, (int)255);
					//data_ui[i * width_uibuf_pitch + j * 3 + 1] = (unsigned char)min((int)fg, (int)255);
					//data_ui[i * width_uibuf_pitch + j * 3 + 2] = (unsigned char)min((int)fb, (int)255);

					rgb = (unsigned char)min((int)fr, (int)255) | ((unsigned char)min((int)fg, (int)255) << 8) | ((unsigned char)min((int)fb, (int)255) << 16);
					memcpy(&data_ui[i * width_uibuf_pitch + j * 3 + 0], &rgb, 3);
				}
			}
	};

	void copy_back_ui_buffer_local(unsigned char* data_ui, int w, int h, unsigned char* data_render_bf, int w_bf, int h_bf, int offset_x, int offset_y, bool v_flib, bool smooth_mask, float _a, float _b, bool opaque_bg)
	{
		auto alpha_mask = [&smooth_mask, &_a, &_b](float r) -> float
		{
			if (!smooth_mask) return 1.f;
			//const float _a = 0.2;
			return min(max((atan(_a * (r - _b)) + atan(_a * 100.f)) / (atan(_a * 1000.f) * 2.f), 0.f), 1.f) * 0.8f;
		};
		// cpu mem ==> dataPtr
		int width_uibuf_pitch = w * 3;
		int width_fbbuf_pitch = w_bf * GL_CLR_CHANNELS;
		glm::fvec2 _c = glm::fvec2(w_bf * 0.5, h_bf * 0.5);
#pragma omp parallel for 
		for (int i = 0; i < h_bf; i++)
			for (int j = 0; j < w_bf; j++)
			{
				min(offset_y + h_bf, h);
				min(offset_x + w_bf, w);//

				int y = v_flib ? (h - 1 - i) : i;

				unsigned int rgba;
				memcpy(&rgba, &data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 0], sizeof(int));

				unsigned char r = rgba & 0xFF;
				unsigned char g = (rgba >> 8) & 0xFF;
				unsigned char b = (rgba >> 16) & 0xFF;
				unsigned char a = opaque_bg ? 255 : (rgba >> 24) & 0xFF;

				if ((a > 0) && (j + offset_y < w) && (i + offset_y < h))
				{
					unsigned int rgb;
					int ui_x = j + offset_x;
					int ui_y = i + offset_y;
					memcpy(&rgb, &data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 0], 3);

					//unsigned char _r = data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 0];
					//unsigned char _g = data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 1];
					//unsigned char _b = data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 2];

					unsigned char _r = rgb & 0xFF;
					unsigned char _g = (rgb >> 8) & 0xFF;
					unsigned char _b = (rgb >> 16) & 0xFF;

					//float _a = alpha_mask(glm::length(_c - glm::fvec2(j, i)));
					int _x = min(min(i, j), min(w_bf - j, h_bf - i));
					float _a = alpha_mask((float)_x);
					//if (i % 20 == 1 && j % 20 == 1)
					//	cout << _a << ", ";
					float fa = (float)a / 255.f * _a;
					float fr = (1.f - fa) * (float)_r + (float)r * fa;
					float fg = (1.f - fa) * (float)_g + (float)g * fa;
					float fb = (1.f - fa) * (float)_b + (float)b * fa;

					//data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 0] = (unsigned char)min((int)fr, (int)255);
					//data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 1] = (unsigned char)min((int)fg, (int)255);
					//data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 2] = (unsigned char)min((int)fb, (int)255);

					rgb = (unsigned char)min((int)fr, (int)255) | ((unsigned char)min((int)fg, (int)255) << 8) | ((unsigned char)min((int)fb, (int)255) << 16);
					memcpy(&data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 0], &rgb, 3);
				}
			}
	};

#define PAIR_MAKE(P2D, P3D) std::pair<cv::Point2f, cv::Point3f>(cv::Point2f(P2D.x, P2D.y), cv::Point3f(P3D.x, P3D.y, P3D.z))
#define double_vec3(D) ((double*)D.data)[0], ((double*)D.data)[1], ((double*)D.data)[2]
	bool CalibrteCamLocalFrame(const vector<glm::fvec2>& points_2d, const vector<glm::fvec3>& points_3dws, const glm::fmat4x4& mat_ws2clf,
		const float fx, const float fy, const float cx, const float cy, glm::fmat4x4& mat_rscs2clf, float* err, int* num_samples,
		vector<pair<Point2f, Point3f>>& pair_pts)//, const int img_w, const int img_h

	{
		if (points_2d.size() == 0) return false;
		if (points_2d.size() != points_3dws.size()) return false;
		using namespace glm;

		//	pair<Point2f, Point3f>& _pair = pair_pts

		vector<Point3f> points_buf_3d_clf;
		vector<Point2f> points_buf_2d;
		for (int i = 0; i < (int)pair_pts.size(); i++)
		{
			pair<Point2f, Point3f>& _pair = pair_pts[i];
			points_buf_2d.push_back(std::get<0>(_pair));
			points_buf_3d_clf.push_back(std::get<1>(_pair));
		}

		int num_incoming_pts = (int)points_3dws.size();
		for (int i = 0; i < num_incoming_pts; i++)
		{
			Point2f p2d = *(Point2f*)&points_2d[i];
			glm::fvec3 ppp = tr_pt(mat_ws2clf, points_3dws[i]);
			Point3f p3d = *(Point3f*)&ppp;

			points_buf_2d.push_back(p2d);
			points_buf_3d_clf.push_back(p3d);
		}

		if (num_samples) *num_samples = points_buf_2d.size();
		if (points_buf_2d.size() < 12)
		{
			for (int i = 0; i < (int)points_buf_2d.size(); i++)
			{
				Point2f p2d = *(Point2f*)&points_buf_2d[i];
				Point3f p3d = *(Point3f*)&points_buf_3d_clf[i];

				pair_pts.push_back(PAIR_MAKE(p2d, p3d));
			}
			return false;
		}

		Mat cam_mat = cv::Mat::zeros(3, 3, CV_64FC1); // intrinsic camera parameters
		cam_mat.at<double>(0, 0) = fx;       //      [ fx   0  cx ]
		cam_mat.at<double>(1, 1) = fy;       //      [  0  fy  cy ]
		cam_mat.at<double>(0, 2) = cx;       //      [  0   0   1 ]
		cam_mat.at<double>(1, 2) = cy;
		cam_mat.at<double>(2, 2) = 1;
		cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);    // vector of distortion coefficients
		cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
		cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector
		//cv::solvePnP(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, false, SOLVEPNP_DLS);
		//cv::solvePnP(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, true, SOLVEPNP_ITERATIVE);
		cv::solvePnP(points_buf_3d_clf, points_buf_2d, cam_mat, distCoeffs, rvec, tvec, false, SOLVEPNP_DLS);
		cv::solvePnP(points_buf_3d_clf, points_buf_2d, cam_mat, distCoeffs, rvec, tvec, true, SOLVEPNP_ITERATIVE);

		float err_proj = 0;
		{
			vector<cv::Point2f> reprojectPoints;
			cv::projectPoints(Mat(*(vector<Point3f>*) & points_buf_3d_clf), rvec, tvec, cam_mat, cv::noArray(), reprojectPoints);
			float reproj_err_sum = 0.;
			reproj_err_sum = cv::norm(Mat(reprojectPoints), Mat(*(vector<Point2f>*) & points_buf_2d)); //  default L2
			err_proj = sqrt(reproj_err_sum * reproj_err_sum / points_buf_2d.size());
			//cout << "PnP reprojection error : " << err_proj << " pixels, # of point pairs L " << points_buf_2d.size() << endl;
		}

		const float err_criterion = 7.f;
		Mat inliers_ids;
		if ((pair_pts.size() > 20 && err_proj > 5.f)
			|| (pair_pts.size() > 50 && err_proj > 3.f)
			|| (pair_pts.size() > 100 && err_proj > 2.f))
		{
			float confidence = 0.9f;
			if (pair_pts.size() >= 100) confidence = 0.8f;
			cv::solvePnPRansac(Mat(*(vector<Point3f>*) & points_buf_3d_clf), Mat(*(vector<Point2f>*) & points_buf_2d), cam_mat, distCoeffs, rvec, tvec, true, 5, err_criterion, confidence, inliers_ids, SOLVEPNP_ITERATIVE);
			cout << "# of inliers : " << inliers_ids.rows << endl;
			if (inliers_ids.rows > 0)
			{
				vector<Point3f> points_buf_3d_clf_tmp = points_buf_3d_clf;
				vector<Point2f> points_buf_2d_tmp = points_buf_2d;
				points_buf_3d_clf.clear();
				points_buf_2d.clear();
				for (int i = 0; i < inliers_ids.rows; i++)
				{
					int index = inliers_ids.at<int>(i, 0);
					//cout << i << ",";
					points_buf_3d_clf.push_back(points_buf_3d_clf_tmp[index]);
					points_buf_2d.push_back(points_buf_2d_tmp[index]);
				}
				//cout << endl;
			}
		}

		pair_pts.clear();
		for (int i = 0; i < (int)points_buf_2d.size(); i++)
		{
			Point2f p2d = *(Point2f*)&points_buf_2d[i];
			Point3f p3d = *(Point3f*)&points_buf_3d_clf[i];

			pair_pts.push_back(PAIR_MAKE(p2d, p3d));
		}

		//cv::calibrateCamera(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), Size(img_w, img_h), cam_mat, distCoeffs, rvec, tvec, 
		//	CALIB_USE_INTRINSIC_GUESS | CALIB_FIX_PRINCIPAL_POINT | CALIB_FIX_ASPECT_RATIO | CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K1 | CALIB_FIX_K2 | CALIB_FIX_K3 | CALIB_FIX_K4 | CALIB_FIX_K5 | CALIB_FIX_K6, 
		//	TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON));

		glm::fvec3 rot_v = glm::fvec3(double_vec3(rvec));
		float rad = glm::length(rot_v);

		fmat4x4 mat_clf2cvf = glm::translate(glm::fvec3(double_vec3(tvec)));
		mat_clf2cvf = glm::rotate(mat_clf2cvf, rad, rot_v / rad);
		fmat4x4 mat_cvf2clf = glm::inverse(mat_clf2cvf);

		fmat4x4 mat_cvf2rscs = glm::rotate((float)CV_PI, glm::fvec3(1, 0, 0));
		fmat4x4 mat_rscs2cvf = glm::inverse(mat_cvf2rscs);

		mat_rscs2clf = mat_cvf2clf * mat_rscs2cvf;

		if (err)
		{
			vector<cv::Point2f> reprojectPoints;
			cv::projectPoints(Mat(*(vector<Point3f>*) & points_buf_3d_clf), rvec, tvec, cam_mat, cv::noArray(), reprojectPoints);

			float reproj_err_sum = 0.;
			reproj_err_sum = cv::norm(Mat(reprojectPoints), Mat(*(vector<Point2f>*) & points_buf_2d)); //  default L2
			*err = sqrt(reproj_err_sum * reproj_err_sum / points_buf_2d.size());
			cout << "PnP reprojection error : " << *err << " pixels, # of point pairs L " << points_buf_2d.size() << endl;
		}

		// TEST //
		//if (points_buf_2d.size() > 100)
		//{
		//	cv::solvePnPRansac(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, false);
		//
		//	vector<cv::Point2f> reprojectPoints;
		//	cv::projectPoints(Mat(*(vector<Point3f>*)&points_buf_3d_clf), rvec, tvec, cam_mat, cv::noArray(), reprojectPoints);
		//	float reproj_err_sum = 0.;
		//	reproj_err_sum = cv::norm(Mat(reprojectPoints), Mat(*(vector<Point2f>*)&points_buf_2d)); //  default L2
		//	*err = sqrt(reproj_err_sum * reproj_err_sum / points_buf_2d.size());
		//	cout << "PnP Ransac reprojection error : " << *err << " pixels" << endl;
		//}

		return true;
	};

	void ComputeCameraStates(const glm::fmat4x4& mat_rscs2clf, // computed through calibration using CalibrteCamLocalFrame
		const glm::fmat4x4& mat_clf2ws, // at every frame, stereo IR cams gives this matrix by tracking rs's rigid IR markers
		vzm::CameraParameters& cam_state // only update CameraParameters::pos, up, view
	)
	{
		using namespace glm;
		fvec3 pos_c_rscs(0);
		fvec3 vec_up_rscs(0, 1, 0);
		fvec3 vec_view_rscs(0, 0, -1);

		fmat4x4 mat_rscs2ws = mat_clf2ws * mat_rscs2clf;
		pos_c_rscs = tr_pt(mat_rscs2ws, pos_c_rscs);
		vec_up_rscs = normalize(tr_vec(mat_rscs2ws, vec_up_rscs));
		vec_view_rscs = normalize(tr_vec(mat_rscs2ws, vec_view_rscs));

		__cv3__ cam_state.pos = pos_c_rscs;
		__cv3__ cam_state.up = vec_up_rscs;
		__cv3__ cam_state.view = vec_view_rscs;
	};
}