#pragma once

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_angle.hpp>

#include "VisMtvApi.h"
#include "ApiUtility.hpp"

#include "GlobalParams.h"

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

#define __cv3__ *(glm::fvec3*)
#define __cv4__ *(glm::fvec4*)
#define __cm4__ *(glm::fmat4x4*)
#define __PR(A, INTERVAL) A[0] << INTERVAL << A[1] << INTERVAL << A[2]

namespace navihelpers {
	using namespace std;

	inline glm::fvec3 tr_pt(const glm::fmat4x4& mat, const glm::fvec3& p)
	{
		glm::fvec4 _p(p, 1.f);
		_p = mat * _p;
		return glm::fvec3(_p.x / _p.w, _p.y / _p.w, _p.z / _p.w);
	}

	inline glm::fvec3 tr_vec(const glm::fmat4x4& mat, const glm::fvec3& v)
	{
		glm::fmat3x3 r = mat;
		glm::fvec3 _v = r * v;
		return _v;
	}

	inline glm::fvec3 tr_vec2(const glm::fmat4x4& mat, const glm::fvec3& v)
	{
		glm::fvec3 _p0 = tr_pt(mat, glm::fvec3(0));
		glm::fvec3 _p1 = tr_pt(mat, glm::fvec3(0) + v);

		return _p1 - _p0;
	}

	// http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
	inline std::vector<std::string> split(const std::string& text, char sep) {
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


	inline void Make_Viewport(std::vector<glm::fvec3>& pos_tris, std::vector<glm::fvec3>& clr_tris, std::vector<glm::fvec3>& pos_lines, std::vector<glm::fvec3>& clr_lines,
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

	inline void ComputeMatCS2PS(const float _fx, const float _fy, const float _s, const float _cx, const float _cy,
		const float widthPix, const float heightPix,
		const float near_p, const float far_p, glm::fmat4x4& matCS2PS)
	{
		double q = far_p / (near_p - far_p);
		double qn = far_p * near_p / (near_p - far_p);

		double fx = (double)_fx;
		double fy = (double)_fy;
		double sc = (double)_s;
		double cx = (double)_cx;
		double cy = (double)_cy;
		double width = (double)widthPix;
		double height = (double)heightPix;
		double x0 = 0, y0 = 0;

		matCS2PS[0][0] = 2.0 * fx / width;
		matCS2PS[1][0] = -2.0 * sc / width;
		matCS2PS[2][0] = (width + 2.0 * x0 - 2.0 * cx) / width;
		matCS2PS[3][0] = 0;
		matCS2PS[0][1] = 0;
		matCS2PS[1][1] = 2.0 * fy / height;
		matCS2PS[2][1] = -(height + 2.0 * y0 - 2.0 * cy) / height;
		matCS2PS[3][1] = 0;
		matCS2PS[0][2] = 0;
		matCS2PS[1][2] = 0;
		matCS2PS[2][2] = q;
		matCS2PS[3][2] = qn;
		matCS2PS[0][3] = 0;
		matCS2PS[1][3] = 0;
		matCS2PS[2][3] = -1.0;
		matCS2PS[3][3] = 0;
		//mat_cs2ps = glm::transpose(mat_cs2ps);
		//fmat_cs2ps = mat_cs2ps;
	};
	inline void Make_ViewportFromCamParams(std::vector<glm::fvec3>& pos_tris, std::vector<glm::fvec3>& clr_tris, std::vector<glm::fvec3>& pos_lines, std::vector<glm::fvec3>& clr_lines,
		const glm::fvec3 clr_tri, const glm::fvec3 clr_line,
		const glm::fvec3 pos_c, const glm::fvec3 view, const glm::fvec3 up, 
		const float fx, const float fy, const float s, const float cx, const float cy, const float w, const float h, const float max_length)
	{
		glm::fmat4x4 matWS2CS = glm::lookAtRH(pos_c, pos_c + view, up);
		glm::fmat4x4 matCS2WS = glm::inverse(matWS2CS);

		glm::fmat4x4 matCS2PS;
		ComputeMatCS2PS(fx, fy, s, cx, cy, w, h, 0.01, max_length, matCS2PS);
		glm::fmat4x4 matPS2WS = matCS2WS * glm::inverse(matCS2PS);

		glm::fvec3 pos_0 = pos_c;
		glm::fvec3 pos_1 = tr_pt(matPS2WS, glm::fvec3(-1, 1, 1));
		glm::fvec3 pos_2 = tr_pt(matPS2WS, glm::fvec3( 1, 1, 1));
		glm::fvec3 pos_3 = tr_pt(matPS2WS, glm::fvec3( -1,-1, 1));
		glm::fvec3 pos_4 = tr_pt(matPS2WS, glm::fvec3(1,-1, 1));

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

	inline void Cam_Gen(const glm::vec3& pos_cam_ws, const glm::vec3& view_ws, const glm::vec3& up_ws, const string cam_label, int& cam_tris_is, int& cam_lines_id, int& cam_label_id)
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

	inline void Cam_Gen(const glm::fmat4x4& mat_cs2ws, const string cam_label, int& cam_tris_is, int& cam_lines_id, int& cam_label_id)
	{
		glm::fvec3 pos_cam_ws = tr_pt(mat_cs2ws, glm::fvec3());
		glm::fvec3 view_ws = tr_vec(mat_cs2ws, glm::fvec3(0, 0, -1));
		glm::fvec3 up_ws = tr_vec(mat_cs2ws, glm::fvec3(0, 1, 0));
		Cam_Gen(pos_cam_ws, view_ws, up_ws, cam_label, cam_tris_is, cam_lines_id, cam_label_id);
	}

	inline void CamOcv_Gen(const glm::fmat4x4& mat_cs2ws, const string cam_label, int& cam_tris_is, int& cam_lines_id, int& cam_label_id)
	{
		glm::fvec3 pos_cam_ws = tr_pt(mat_cs2ws, glm::fvec3());
		glm::fvec3 view_ws = tr_vec(mat_cs2ws, glm::fvec3(0, 0, 1));
		glm::fvec3 up_ws = tr_vec(mat_cs2ws, glm::fvec3(0, -1, 0));
		Cam_Gen(pos_cam_ws, view_ws, up_ws, cam_label, cam_tris_is, cam_lines_id, cam_label_id);
	}

	inline void World_GridAxis_Gen(int& coord_grid_obj_id, int& axis_lines_obj_id, int& axis_texX_obj_id, int& axis_texZ_obj_id)
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



	inline glm::fvec3 ComputeNormalVector(std::vector<glm::fvec3>& points) {
		Eigen::Vector3d normalVector;
		{
			Eigen::Matrix3d A;

			int nunmPts = (int)points.size();
			if (nunmPts < 3)
				return glm::fvec3(0, 0, 1);	// error case

			glm::fvec3 centerPos = glm::fvec3(0, 0, 0);
			for (int i = 0; i < nunmPts; i++) {
				centerPos += points[i];
			}
			centerPos /= (float)nunmPts;

			// Construct the matrix A for the least-squares problem
			//for (int i = 0; i < nunmPts; ++i) {
			//	A(i, 0) = points[i].x - centerPos.x;
			//	A(i, 1) = points[i].y - centerPos.y;
			//	A(i, 2) = points[i].z - centerPos.z;
			//}

			for (int i = 0; i < 3; ++i) {
				A(i, 0) = points[i + 1].x - points[0].x;
				A(i, 1) = points[i + 1].y - points[0].y;
				A(i, 2) = points[i + 1].z - points[0].z;
			}

			// Compute the normal vector as the null space of A
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullV);
			normalVector = svd.matrixV().col(2);
			normalVector.normalize();
		}

		return glm::fvec3(normalVector.x(), normalVector.y(), normalVector.z());
	};
}