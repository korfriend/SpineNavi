#pragma once
#include "VisMtvApi.h"
// math using GLM
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"

namespace vzmutils {
	using namespace std;
	using namespace glm;

	inline glm::fvec3 transformPos(const glm::fvec3& pos, const glm::fmat4x4& m)
	{
		glm::fvec4 pos4 = glm::fvec4(pos, 1);
		pos4 = m * pos4;
		pos4 /= pos4.w;
		return glm::fvec3(pos4);
	};

	inline glm::fvec3 transformVec(const glm::fvec3& vec, const glm::fmat4x4& m)
	{
		glm::fvec4 vec4 = glm::fvec4(vec, 0);
		vec4 = m * vec4;
		return glm::fvec3(vec4);
	};

	inline glm::fvec3 comp_mul(const glm::fvec3& vec1, const glm::fvec3& vec2)
	{
		return fvec3(vec1.x * vec2.x, vec1.y * vec2.y, vec1.z * vec2.z);
	};

	inline bool check_curvedSlicer(const int slicer_cam_id, vzm::CameraParameters& pano_slicer, std::vector<float>& v_curved_points, std::vector<float>& v_curved_ups, std::vector<float>& v_curved_tangents)
	{
		if (!vzm::GetCameraParams(slicer_cam_id, pano_slicer)) return false;
		if (!pano_slicer.script_params.GetParamCheck("ARRAY_CURVE_INTERPOLATION_POS", v_curved_points)
			|| !pano_slicer.script_params.GetParamCheck("ARRAY_CURVE_INTERPOLATION_UP", v_curved_ups)
			|| !pano_slicer.script_params.GetParamCheck("ARRAY_CURVE_INTERPOLATION_TANGENT", v_curved_tangents)
			) return false;
		return true;
	};

	inline bool ComputeNearestPointBtwLineAndPoint(glm::fvec3& pos_nearest, float& t, const glm::fvec3& pos_line, const glm::fvec3& vec_line, const glm::fvec3& pos_pt)
	{
		//http://math.stackexchange.com/questions/748315/finding-the-coordinates-of-a-point-on-a-line-that-produces-the-shortest-distance
		float vec_length = glm::length(vec_line);
		if (vec_length == 0)
			return false;
		t = ((pos_pt.x * vec_line.x + pos_pt.y * vec_line.y + pos_pt.z * vec_line.z) - (pos_line.x * vec_line.x + pos_line.y * vec_line.y + pos_line.z * vec_line.z)) / vec_length;
		pos_nearest = pos_line + vec_line * t;
		return true;
	}

	inline bool ComputeCrossPointsBtwLineAndAlignedCube(const glm::fvec3& aabb_min, const glm::fvec3& aabb_max, const glm::fvec3& pos_begin, const glm::fvec3& vec_dir, glm::fvec2& t0t1) {
		// intersect ray with a box
		// http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter3.htm
		fvec3 invR = fvec3(1.0 / vec_dir.x, 1.0 / vec_dir.y, 1.0 / vec_dir.z);
		fvec3 tbot = comp_mul(invR, aabb_min - pos_begin);
		fvec3 ttop = comp_mul(invR, aabb_max - pos_begin);

		// re-order intersections to find smallest and largest on each axis
		fvec3 tmin = glm::min(ttop, tbot);
		fvec3 tmax = glm::max(ttop, tbot);

		// find the largest tmin and the smallest tmax
		if (vec_dir.x == 0)
			tmin.x = -FLT_MAX;
		if (vec_dir.y == 0)
			tmin.y = -FLT_MAX;
		if (vec_dir.z == 0)
			tmin.z = -FLT_MAX;

		if (vec_dir.x == 0)
			tmax.x = FLT_MAX;
		if (vec_dir.y == 0)
			tmax.y = FLT_MAX;
		if (vec_dir.z == 0)
			tmax.z = FLT_MAX;

		float largest_tmin = std::max(std::max(tmin.x, tmin.y), std::max(tmin.x, tmin.z));
		float smallest_tmax = std::min(std::min(tmax.x, tmax.y), std::min(tmax.x, tmax.z));

		t0t1.x = largest_tmin; // Math.Max(largest_tmin, 0.0);
		t0t1.y = smallest_tmax;

		return t0t1.x < t0t1.y;
	}
	inline bool ComputeCrossPointsBtwLineAndActorCube(const int actor_id, const glm::fvec3& pos_begin_ws, const glm::fvec3& vec_dir_ws, glm::fvec2& t0t1) {
		vzm::ActorParameters _actor;
		vzm::GetActorParams(actor_id, _actor);
		int obj_id = _actor.GetResourceID(vzm::ActorParameters::GEOMETRY);
		glm::fmat4x4 os2ws = *(glm::fmat4x4*)_actor.GetWorldTransform();
		glm::fmat4x4 ws2os = glm::inverse(os2ws);

		glm::fvec3 pos_line_os = transformPos(pos_begin_ws, ws2os);
		glm::fvec3 vec_line_os = transformPos(vec_dir_ws, ws2os);

		glm::fvec3 aabb_min_os, aabb_max_os;
		vzm::GetObjBoundingBox(obj_id, __FP aabb_min_os, __FP aabb_max_os);

		return ComputeCrossPointsBtwLineAndAlignedCube(aabb_min_os, aabb_max_os, pos_line_os, vec_line_os, t0t1);
	}
	inline float ComputeCrossPointsBtwLineSetAndActorCube(const int actor_id, const std::vector<glm::fvec3>& pos_begins_ws, const std::vector<glm::fvec3>& vec_dirs_ws, std::vector<glm::fvec2>& t0t1s) {
		vzm::ActorParameters _actor;
		vzm::GetActorParams(actor_id, _actor);
		int obj_id = _actor.GetResourceID(vzm::ActorParameters::GEOMETRY);
		glm::fmat4x4 os2ws = *(glm::fmat4x4*)_actor.GetWorldTransform();
		glm::fmat4x4 ws2os = glm::inverse(os2ws);

		glm::fvec3 aabb_min_os, aabb_max_os;
		vzm::GetObjBoundingBox(obj_id, __FP aabb_min_os, __FP aabb_max_os);

		int num_pts = (int)pos_begins_ws.size();
		t0t1s.reserve(num_pts);
		fvec2 valid_minmax_t(FLT_MAX, -FLT_MAX);
		for (int i = 0; i < num_pts; i++) {
			glm::fvec3 pos_line_os = transformPos(pos_begins_ws[i], ws2os);
			glm::fvec3 vec_line_os = transformPos(vec_dirs_ws[i], ws2os);
			fvec2 t0t1;
			if (ComputeCrossPointsBtwLineAndAlignedCube(aabb_min_os, aabb_max_os, pos_line_os, vec_line_os, t0t1)) {
				valid_minmax_t.x = std::min(valid_minmax_t.x, t0t1.x);
				valid_minmax_t.y = std::max(valid_minmax_t.y, t0t1.y);
			}
			t0t1s.push_back(t0t1);
		}
		return valid_minmax_t.x < valid_minmax_t.y ? valid_minmax_t.y - valid_minmax_t.x : 0;
	}
	inline bool ComputeCurvePlaneFitSizeOnActor(const int actor_id, const float* curve_pos_points, const float* curve_up_points, const int num_curve_points, const int fb_w, const int fb_h, float& curve_w, float& curve_h) {
		//vzm::ActorParameters _actor;
		//vzm::GetActorParams(actor_id, _actor);
		//int obj_id = _actor.GetResourceID(vzm::ActorParameters::GEOMETRY);
		//glm::fvec3 aabb_min_os, aabb_max_os;
		//vzm::GetObjBoundingBox(obj_id, __FP aabb_min_os, __FP aabb_max_os);
		//float expected_length = length(aabb_max_os - aabb_min_os);

		fvec3* v3curved_points = (fvec3*)curve_pos_points;
		fvec3* v3curved_dirs = (fvec3*)curve_up_points;
		fvec3 pos_old = v3curved_points[0];
		curve_w = 0;
		std::vector<fvec3> pos_begins_ws(num_curve_points);
		memcpy(&pos_begins_ws[0], v3curved_points, sizeof(fvec3) * num_curve_points);
		std::vector<fvec3> vec_dirs_ws(num_curve_points);
		vec_dirs_ws[0] = normalize(v3curved_dirs[0]);

		for (int i = 1; i < num_curve_points; i++)
		{
			fvec3 pos_cur = v3curved_points[i];
			curve_w += length(pos_cur - pos_old);
			pos_old = pos_cur;

			vec_dirs_ws[i] = normalize(v3curved_dirs[i]);
		}
		std::vector<glm::fvec2> t0t1s;
		curve_h = ComputeCrossPointsBtwLineSetAndActorCube(actor_id, pos_begins_ws, vec_dirs_ws, t0t1s);

		//curve_h = curve_w * ((float)fb_h / (float)fb_w);

		return curve_h > 0;
	}
	inline bool FitCurvedSlicerCameraOnActor(const int actor_id, const float* curve_pos_points, const float* curve_up_points, const float* curve_tan_points, const int num_curve_points, const int fb_w, const int fb_h, vzm::CameraParameters& pano_cam_params) {
		*(glm::fvec3*)pano_cam_params.pos = glm::fvec3(0, 0, 0);
		*(glm::fvec3*)pano_cam_params.up = glm::fvec3(0, 1, 0);
		*(glm::fvec3*)pano_cam_params.view = glm::fvec3(0, 0, -1);
		pano_cam_params.w = fb_w;
		pano_cam_params.h = fb_h;
		pano_cam_params.np = 0;
		pano_cam_params.fp = 10000.f;
		pano_cam_params.projection_mode = vzm::CameraParameters::SLICER_CURVED;

		// heuristic check : pano-viewing direction 
		glm::fvec3* pos = (glm::fvec3*)curve_pos_points;
		glm::fvec3 pS = pos[0];
		glm::fvec3 pM = pos[num_curve_points / 2];
		glm::fvec3 pE = pos[num_curve_points - 1];
		glm::fvec3 pSE = (pS + pE) * 0.5f;
		glm::fvec3 vPano = pSE - pM;
		glm::fvec3* tan = (glm::fvec3*)curve_tan_points;
		glm::fvec3* up = (glm::fvec3*)curve_up_points;
		glm::fvec3 tanM = tan[num_curve_points / 2];
		glm::fvec3 upM = up[num_curve_points / 2];
		glm::fvec3 vM = glm::cross(upM, tanM);
		
		bool isRightSide = glm::dot(vM, vPano) < 0;
		pano_cam_params.script_params.SetParam("RIGHT_SIDE", isRightSide);

		float ip_curve_w, ip_curve_h;
		ComputeCurvePlaneFitSizeOnActor(actor_id, curve_pos_points, curve_up_points, num_curve_points, pano_cam_params.w, pano_cam_params.h, ip_curve_w, ip_curve_h);
		pano_cam_params.ip_w = ip_curve_w;
		pano_cam_params.ip_h = ip_curve_w * ((float)fb_h / (float)fb_w);
		//curve_h = curve_w * ((float)fb_h / (float)fb_w);
		float pano_plane_height = ip_curve_h;
		pano_cam_params.SetCurvedSlicer(ip_curve_w, pano_plane_height, (float*)curve_pos_points, (float*)curve_up_points, (float*)curve_tan_points, num_curve_points);

		return true;
	}

	inline bool GetWorldPositionOnScreen(glm::fvec3& pos_ws, const int x, const int y, const int cam_id) {
		vzm::CameraParameters cam_param;
		vzm::GetCameraParams(cam_id, cam_param);
		if (cam_param.projection_mode == vzm::CameraParameters::ProjectionMode::SLICER_CURVED) {

			float curved_plane_w, curved_plane_h;
			int num_curve_width_pts;
			std::vector<float>* vf_curve_pos_pts = cam_param.script_params.GetParamPtr<std::vector<float>>("ARRAY_CURVE_INTERPOLATION_POS");
			std::vector<float>* vf_curve_up_pts = cam_param.script_params.GetParamPtr<std::vector<float>>("ARRAY_CURVE_INTERPOLATION_UP");
			if (!cam_param.script_params.GetParamCheck("CURVED_PLANE_WIDTH", curved_plane_w) ||
				!cam_param.script_params.GetParamCheck("CURVED_PLANE_HEIGHT", curved_plane_h) ||
				!cam_param.script_params.GetParamCheck("COUNT_INTERPOLATION_POINTS", num_curve_width_pts) ||
				vf_curve_pos_pts == NULL || vf_curve_up_pts == NULL) return false;

			fvec3* curve_pos_pts = (fvec3*)&vf_curve_pos_pts->at(0);
			fvec3* curve_up_pts = (fvec3*)&vf_curve_up_pts->at(0);

			fvec3 cam_pos_cws = *(fvec3*)cam_param.pos;
			fvec3 cam_dir_cws = *(fvec3*)cam_param.view;
			fvec3 cam_up_cws = *(fvec3*)cam_param.up;
			fvec3 cam_right_cws = normalize(cross(cam_dir_cws, cam_up_cws));
			// if view is +z and up is +y, then x dir is left, which means that curve index increases along right to left
			fvec3 pos_tl_cws = cam_pos_cws - cam_right_cws * cam_param.ip_w * 0.5f + cam_up_cws * cam_param.ip_h * 0.5f;
			fvec3 pos_tr_cws = cam_pos_cws + cam_right_cws * cam_param.ip_w * 0.5f + cam_up_cws * cam_param.ip_h * 0.5f;
			fvec3 pos_bl_cws = cam_pos_cws - cam_right_cws * cam_param.ip_w * 0.5f - cam_up_cws * cam_param.ip_h * 0.5f;
			fvec3 pos_br_cws = cam_pos_cws + cam_right_cws * cam_param.ip_w * 0.5f - cam_up_cws * cam_param.ip_h * 0.5f;
			float curve_plane_pitch = curved_plane_w / (float)num_curve_width_pts;
			float num_curve_height_pts = curved_plane_h / curve_plane_pitch;
			float num_curve_midheight_pts = num_curve_height_pts * 0.5f;

			fmat4x4 mat_scale = scale(fvec3(curve_plane_pitch));
			fmat4x4 mat_translate = translate(fvec3(-curved_plane_w * 0.5f, -curved_plane_h * 0.5f, -curve_plane_pitch * 0.5f));
			fmat4x4 mat_cos2cws = mat_translate * mat_scale;
			fmat4x4 mat_cws2cos = inverse(mat_cos2cws);

			fvec3 pos_tl_cos = transformPos(pos_tl_cws, mat_cws2cos);
			fvec3 pos_tr_cos = transformPos(pos_tr_cws, mat_cws2cos);
			fvec3 pos_bl_cos = transformPos(pos_bl_cws, mat_cws2cos);
			fvec3 pos_br_cos = transformPos(pos_br_cws, mat_cws2cos);

			fvec2 pos_inter_top_cos, pos_inter_bottom_cos, pos_sample_cos;
			float fRatio0 = (float)(((cam_param.w - 1) - x) / (float)(cam_param.w - 1));
			float fRatio1 = (float)(x) / (float)(cam_param.w - 1);
			pos_inter_top_cos.x = fRatio0 * pos_tl_cos.x + fRatio1 * pos_tr_cos.x;
			pos_inter_top_cos.y = fRatio0 * pos_tl_cos.y + fRatio1 * pos_tr_cos.y;
			if (pos_inter_top_cos.x >= 0 && pos_inter_top_cos.x < (float)(num_curve_width_pts - 1)) {
				int x_sample_cos = (int)floor(pos_inter_top_cos.x);
				float x_ratio = pos_inter_top_cos.x - (float)x_sample_cos;
				int addr_minmix_x = std::min(std::max(x_sample_cos, 0), num_curve_width_pts - 1);
				int addr_minmax_nextx = std::min(std::max(x_sample_cos + 1, 0), num_curve_width_pts - 1);
				fvec3 pos_sample_ws_c0 = curve_pos_pts[addr_minmix_x];
				fvec3 pos_sample_ws_c1 = curve_pos_pts[addr_minmax_nextx];
				fvec3 pos_sample_ws_c = pos_sample_ws_c0 * (1.f - x_ratio) + pos_sample_ws_c1 * x_ratio;

				fvec3 up_sample_ws_c0 = curve_up_pts[addr_minmix_x];
				fvec3 up_sample_ws_c1 = curve_up_pts[addr_minmax_nextx];
				fvec3 up_sample_ws_c = up_sample_ws_c0 * (1.f - x_ratio) + up_sample_ws_c1 * x_ratio;

				up_sample_ws_c = normalize(up_sample_ws_c);
				up_sample_ws_c *= curve_plane_pitch;
				pos_inter_bottom_cos.x = fRatio0 * pos_bl_cos.x + fRatio1 * pos_br_cos.x;
				pos_inter_bottom_cos.y = fRatio0 * pos_bl_cos.y + fRatio1 * pos_br_cos.y;

				//================== y
				float y_ratio0 = (float)((cam_param.h - 1) - y) / (float)(cam_param.h - 1);
				float y_ratio1 = (float)(y) / (float)(cam_param.h - 1);
				pos_sample_cos.x = y_ratio0 * pos_inter_top_cos.x + y_ratio1 * pos_inter_bottom_cos.x;
				pos_sample_cos.y = y_ratio0 * pos_inter_top_cos.y + y_ratio1 * pos_inter_bottom_cos.y;
				if (pos_sample_cos.y < 0 || pos_sample_cos.y > num_curve_height_pts)
					return false;
				pos_ws = pos_sample_ws_c + up_sample_ws_c * (pos_sample_cos.y - num_curve_midheight_pts);
			}
			else 
				return false;
		}
		else {
			fmat4x4 matSS2WS;
			if (!vzm::GetCamProjMatrix(cam_id, NULL, (float*)&matSS2WS)) return false;
			fvec4 pos_out = matSS2WS * fvec4(x, y, 0, 1.f);
			pos_ws = fvec3(pos_out / pos_out.w);
		}
		return true;
	}
	inline bool GetVolumePositionOnScreen(glm::fvec3& pos_vs, const int volume_actor_id, const int x, const int y, const int cam_id) {
		fvec3 pos_ws;
		if (!GetWorldPositionOnScreen(pos_ws, x, y, cam_id)) return false;

		glm::fmat4x4 mat_vs2os;
		vzm::ActorParameters vol_actor;
		vzm::GetActorParams(volume_actor_id, vol_actor);
		int vol_id = vol_actor.GetResourceID(vzm::ActorParameters::GEOMETRY);
		vzm::GetVolumeInfo(vol_id, NULL, NULL, NULL, NULL, NULL, NULL, __FP mat_vs2os, NULL, NULL);

		glm::fmat4x4 mat_os2ws = *(glm::fmat4x4*)vol_actor.GetWorldTransform();
		glm::fmat4x4 mat_ws2os = glm::inverse(mat_os2ws);
		glm::fmat4x4 mat_os2vs = glm::inverse(mat_vs2os);

		glm::fmat4x4 mat_ws2vs = mat_os2vs * mat_ws2os;

		pos_vs = transformPos(pos_ws, mat_ws2vs);
		return true;
	}
	inline bool GetDicomHUValueOnScreen(float& sample_value, const int volume_actor_id, const int x, const int y, const int cam_id) {
		fvec3 pos_vs;
		GetVolumePositionOnScreen(pos_vs, volume_actor_id, x, y, cam_id);

		unsigned short** slices;
		glm::ivec3 vol_size;
		glm::fvec2 stored_minmax;
		glm::fvec2 hu_minmax;
		vzm::ActorParameters vol_actor;
		vzm::GetActorParams(volume_actor_id, vol_actor);
		int vol_id = vol_actor.GetResourceID(vzm::ActorParameters::GEOMETRY);
		int stride;
		vzm::GetVolumeInfo(vol_id, (void***)&slices, (int*)&vol_size, NULL, &stride, __FP stored_minmax, __FP hu_minmax, NULL, NULL, NULL);
		if (stride != 2) // only support 16 bit unsigned volume!
			return false;

		ivec3 ipos_vs = ivec3((int)pos_vs.x, (int)pos_vs.y, (int)pos_vs.z);

		if (ipos_vs.x < 0 || ipos_vs.y < 0 || ipos_vs.z < 0 || ipos_vs.x >= vol_size.x || ipos_vs.y >= vol_size.y || ipos_vs.z >= vol_size.z)
			return false;

		glm::ivec3 bnd_size(2);
		int sample_w = vol_size.x + 2 * bnd_size.x;
		int iMinMaxAddrX = std::min(std::max((int)ipos_vs.x, (int)0), vol_size.x - 1) + bnd_size.x;
		int iMinMaxAddrNextX = std::min(std::max(ipos_vs.x + 1, 0), vol_size.x - 1) + bnd_size.x;
		int iMinMaxAddrY = (std::min(std::max(ipos_vs.y, 0), vol_size.y - 1) + bnd_size.y) * sample_w;
		int iMinMaxAddrNextY = (std::min(std::max(ipos_vs.y + 1, 0), vol_size.y - 1) + bnd_size.y) * sample_w;

		int sample_addr0 = iMinMaxAddrX + iMinMaxAddrY;
		int sample_addr1 = iMinMaxAddrNextX + iMinMaxAddrY;
		int sample_addr2 = iMinMaxAddrX + iMinMaxAddrNextY;
		int sample_addr3 = iMinMaxAddrNextX + iMinMaxAddrNextY;
		int sample_addrZ0 = ipos_vs.z + bnd_size.z;
		int sample_addrZ1 = ipos_vs.z + bnd_size.z + 1;

		if (ipos_vs.z < 0)
			sample_addrZ0 = sample_addrZ1;
		else if (ipos_vs.z >= vol_size.z - 1)
			sample_addrZ1 = sample_addrZ0;

		unsigned short sample_values[8];
		sample_values[0] = slices[sample_addrZ0][sample_addr0];
		sample_values[1] = slices[sample_addrZ0][sample_addr1];
		sample_values[2] = slices[sample_addrZ0][sample_addr2];
		sample_values[3] = slices[sample_addrZ0][sample_addr3];
		sample_values[4] = slices[sample_addrZ1][sample_addr0];
		sample_values[5] = slices[sample_addrZ1][sample_addr1];
		sample_values[6] = slices[sample_addrZ1][sample_addr2];
		sample_values[7] = slices[sample_addrZ1][sample_addr3];

		fvec3 ratio;
		ratio.x = pos_vs.x - ipos_vs.x;
		ratio.y = pos_vs.y - ipos_vs.y;
		ratio.z = pos_vs.z - ipos_vs.z;

		float interpolate_w[8];
		interpolate_w[0] = (1.f - ratio.z) * (1.f - ratio.y) * (1.f - ratio.x);
		interpolate_w[1] = (1.f - ratio.z) * (1.f - ratio.y) * ratio.x;
		interpolate_w[2] = (1.f - ratio.z) * ratio.y * (1.f - ratio.x);
		interpolate_w[3] = (1.f - ratio.z) * ratio.y * ratio.x;
		interpolate_w[4] = ratio.z * (1.f - ratio.y) * (1.f - ratio.x);
		interpolate_w[5] = ratio.z * (1.f - ratio.y) * ratio.x;
		interpolate_w[6] = ratio.z * ratio.y * (1.f - ratio.x);
		interpolate_w[7] = ratio.z * ratio.y * ratio.x;

		sample_value = 0;
		for (int m = 0; m < 8; m++)
		{
			sample_value += sample_values[m] * interpolate_w[m];
		}

		float hu_diff = hu_minmax[1] - hu_minmax[0];
		float store_diff = stored_minmax[1] - stored_minmax[0];
		if (hu_diff != 0 && store_diff != 0) {
			// reference : GetStoredValueFromHUValue
			float slope = hu_diff / store_diff;
			sample_value = slope * (sample_value  - stored_minmax[0]) + hu_minmax[0];
		}

		return true;
	}
	inline bool GetNearestPositionOnCurvedPlane(float* pos_nearest_curve_ws, const float* pos_ws, const float* pos_curve_array, const float* up_curve_array, const float* tan_curve_array, const int num_pts, 
		int* array_nearest_idx = NULL, int* array_2nd_nearest_idx = NULL, float* idx_ratio_t = NULL, float* height_plane = NULL) {
		glm::fvec3& pos_src_ws = *(glm::fvec3*)pos_ws;
		glm::fvec3* pos_curve_pts = (glm::fvec3*)pos_curve_array;
		glm::fvec3* up_curve_pts = (glm::fvec3*)up_curve_array;
		glm::fvec3* tan_curve_pts = (glm::fvec3*)tan_curve_array;

		fvec3 pos_src_plane = pos_curve_pts[num_pts / 2];
		fvec3 vec_plane = glm::normalize(up_curve_pts[num_pts / 2]);

		// compute pos on plane for pos_src_ws
		fvec3 vec_src2p = pos_src_ws - pos_src_plane;
		float _dot = glm::dot(vec_src2p, vec_plane);
		fvec3 pos_src_on_plane = pos_src_ws - _dot * vec_plane;

		// find the nearest point by greedy method 
		int nearest_idx = -1;
		float nearest_length_sq = FLT_MAX;
		for (int i = 0; i < num_pts; i++) {
			fvec3 pos_on_curve = pos_curve_pts[i];
			float lenghsq = glm::length2(pos_src_on_plane - pos_on_curve);
			if (lenghsq < nearest_length_sq) {
				nearest_length_sq = lenghsq;
				nearest_idx = i;
			}
		}

		// interpolation for more precise result
		fvec3 pos_nearest_pt = pos_curve_pts[nearest_idx];
		if (array_nearest_idx) *array_nearest_idx = nearest_idx;
		fvec3 pos_nearest_pt_side1 = pos_curve_pts[std::max(nearest_idx - 1, (int)0)];
		fvec3 pos_nearest_pt_side2 = pos_curve_pts[std::min(nearest_idx + 1, std::max(num_pts - 1, 0))];
		fvec3 pos_2nd_nearest_pt;
		if (glm::length2(pos_src_on_plane - pos_nearest_pt_side1) < glm::length2(pos_src_on_plane - pos_nearest_pt_side2)) {
			pos_2nd_nearest_pt = pos_nearest_pt_side1;
			if (array_2nd_nearest_idx) *array_2nd_nearest_idx = std::max(nearest_idx - 1, (int)0);
		}
		else {
			pos_2nd_nearest_pt = pos_nearest_pt_side2;
			if (array_2nd_nearest_idx) *array_2nd_nearest_idx = std::min(nearest_idx + 1, std::max(num_pts - 1, 0));
		}

		fvec3 pos_nearest_on_plane = pos_nearest_pt;
		fvec3 vec_nearest_line = pos_2nd_nearest_pt - pos_nearest_pt;
		if (idx_ratio_t) *idx_ratio_t = 0;
		if (glm::length2(vec_nearest_line) > 0.000001f)
		{
			fvec3 pos_nearest_tmp;
			float t;
			ComputeNearestPointBtwLineAndPoint(pos_nearest_tmp, t, pos_nearest_pt, vec_nearest_line, pos_src_on_plane);
			if (t > 0 || t < 1) {
				pos_nearest_on_plane = pos_nearest_tmp;
				if (idx_ratio_t) *idx_ratio_t = t;
			}
		}

		if (height_plane) *height_plane = _dot;

		*(fvec3*)pos_nearest_curve_ws = pos_nearest_on_plane + _dot * vec_plane;
		return true;
	}
	inline bool GetNearestPositionOnCurvedSlicer(float* pos_nearest_curve_ws, const float* pos_ws, const int slicer_cam_id) {
		vzm::CameraParameters pano_slicer;
		std::vector<float> v_curved_points, v_curved_ups, v_curved_tangents;
		if (!check_curvedSlicer(slicer_cam_id, pano_slicer, v_curved_points, v_curved_ups, v_curved_tangents)) return false;

		return GetNearestPositionOnCurvedPlane(pos_nearest_curve_ws, pos_ws, &v_curved_points[0], &v_curved_ups[0], &v_curved_tangents[0], (int)v_curved_points.size() / 3);
	}
	inline bool GetWorldLookAtOnCurvedSlicer(float* pos_ws, float* up_ws, float* view_ws, const int x, const int y, const int slicer_cam_id) {
		vzm::CameraParameters pano_slicer;
		std::vector<float> v_curved_points, v_curved_ups, v_curved_tangents;
		if (!check_curvedSlicer(slicer_cam_id, pano_slicer, v_curved_points, v_curved_ups, v_curved_tangents)) return false;

		float curved_plane_w, curved_plane_h;
		int num_curve_width_pts;
		if (!pano_slicer.script_params.GetParamCheck("CURVED_PLANE_WIDTH", curved_plane_w) ||
			!pano_slicer.script_params.GetParamCheck("CURVED_PLANE_HEIGHT", curved_plane_h) ||
			!pano_slicer.script_params.GetParamCheck("COUNT_INTERPOLATION_POINTS", num_curve_width_pts)) return false;

		fvec3* curve_pos_pts = (fvec3*)&v_curved_points[0];
		fvec3* curve_up_pts = (fvec3*)&v_curved_ups[0];
		fvec3* curve_tan_pts = (fvec3*)&v_curved_tangents[0];

		fvec3 cam_pos_cws = *(fvec3*)pano_slicer.pos;
		fvec3 cam_dir_cws = *(fvec3*)pano_slicer.view;
		fvec3 cam_up_cws = *(fvec3*)pano_slicer.up;
		fvec3 cam_right_cws = normalize(cross(cam_dir_cws, cam_up_cws)); 
		fvec3 pos_tl_cws = cam_pos_cws - cam_right_cws * pano_slicer.ip_w * 0.5f + cam_up_cws * pano_slicer.ip_h * 0.5f;
		fvec3 pos_tr_cws = cam_pos_cws + cam_right_cws * pano_slicer.ip_w * 0.5f + cam_up_cws * pano_slicer.ip_h * 0.5f;
		fvec3 pos_bl_cws = cam_pos_cws - cam_right_cws * pano_slicer.ip_w * 0.5f - cam_up_cws * pano_slicer.ip_h * 0.5f;
		fvec3 pos_br_cws = cam_pos_cws + cam_right_cws * pano_slicer.ip_w * 0.5f - cam_up_cws * pano_slicer.ip_h * 0.5f;
		float curve_plane_pitch = curved_plane_w / (float)num_curve_width_pts;
		float num_curve_height_pts = curved_plane_h / curve_plane_pitch;
		float num_curve_midheight_pts = num_curve_height_pts * 0.5f;

		fmat4x4 mat_scale = scale(fvec3(curve_plane_pitch));
		fmat4x4 mat_translate = translate(fvec3(-curved_plane_w * 0.5f, -curved_plane_h * 0.5f, -curve_plane_pitch * 0.5f));
		fmat4x4 mat_cos2cws = mat_translate * mat_scale;
		fmat4x4 mat_cws2cos = inverse(mat_cos2cws);

		fvec3 pos_tl_cos = transformPos(pos_tl_cws, mat_cws2cos);
		fvec3 pos_tr_cos = transformPos(pos_tr_cws, mat_cws2cos);
		fvec3 pos_bl_cos = transformPos(pos_bl_cws, mat_cws2cos);
		fvec3 pos_br_cos = transformPos(pos_br_cws, mat_cws2cos);

		fvec2 pos_inter_top_cos, pos_inter_bottom_cos, pos_sample_cos;
		float fRatio0 = (float)(((pano_slicer.w - 1) - x) / (float)(pano_slicer.w - 1));
		float fRatio1 = (float)(x) / (float)(pano_slicer.w - 1);
		pos_inter_top_cos.x = fRatio0 * pos_tl_cos.x + fRatio1 * pos_tr_cos.x;
		pos_inter_top_cos.y = fRatio0 * pos_tl_cos.y + fRatio1 * pos_tr_cos.y;
		if (pos_inter_top_cos.x >= 0 && pos_inter_top_cos.x < (float)(num_curve_width_pts - 1)) {
			int x_sample_cos = (int)floor(pos_inter_top_cos.x);
			float x_ratio = pos_inter_top_cos.x - (float)x_sample_cos;
			int addr_minmix_x = std::min(std::max(x_sample_cos, 0), num_curve_width_pts - 1);
			int addr_minmax_nextx = std::min(std::max(x_sample_cos + 1, 0), num_curve_width_pts - 1);
			fvec3 pos_sample_ws_c0 = curve_pos_pts[addr_minmix_x];
			fvec3 pos_sample_ws_c1 = curve_pos_pts[addr_minmax_nextx];
			fvec3 pos_sample_ws_c = pos_sample_ws_c0 * (1.f - x_ratio) + pos_sample_ws_c1 * x_ratio;

			fvec3 up_sample_ws_c0 = curve_up_pts[addr_minmix_x];
			fvec3 up_sample_ws_c1 = curve_up_pts[addr_minmax_nextx];
			fvec3 up_sample_ws_c = up_sample_ws_c0 * (1.f - x_ratio) + up_sample_ws_c1 * x_ratio;

			up_sample_ws_c = normalize(up_sample_ws_c);

			{
				*(fvec3*)up_ws = up_sample_ws_c;

				fvec3 tan_sample_ws_c0 = curve_tan_pts[addr_minmix_x];
				fvec3 tan_sample_ws_c1 = curve_tan_pts[addr_minmax_nextx];
				fvec3 tan_sample_ws_c = tan_sample_ws_c0 * (1.f - x_ratio) + tan_sample_ws_c1 * x_ratio;

				// if panocam view is +z and up is +y, then x dir is left, which means that curve index increases along right to left
				fvec3 right_c = normalize(tan_sample_ws_c);// cam_dir_cws.z > 0 ? -normalize(tan_sample_ws_c) : normalize(tan_sample_ws_c);
				*(fvec3*)view_ws = normalize(cross(up_sample_ws_c, right_c));
			}

			up_sample_ws_c *= curve_plane_pitch;
			pos_inter_bottom_cos.x = fRatio0 * pos_bl_cos.x + fRatio1 * pos_br_cos.x;
			pos_inter_bottom_cos.y = fRatio0 * pos_bl_cos.y + fRatio1 * pos_br_cos.y;

			//================== y
			float y_ratio0 = (float)((pano_slicer.h - 1) - y) / (float)(pano_slicer.h - 1);
			float y_ratio1 = (float)(y) / (float)(pano_slicer.h - 1);
			pos_sample_cos.x = y_ratio0 * pos_inter_top_cos.x + y_ratio1 * pos_inter_bottom_cos.x;
			pos_sample_cos.y = y_ratio0 * pos_inter_top_cos.y + y_ratio1 * pos_inter_bottom_cos.y;
			if (pos_sample_cos.y < 0 || pos_sample_cos.y > num_curve_height_pts)
				return false;
			*(fvec3*)pos_ws = pos_sample_ws_c + up_sample_ws_c * (pos_sample_cos.y - num_curve_midheight_pts);
		}
		else 
			return false;

		return true;
	}

	inline bool GetScreenPositionFromWorld(glm::fvec3& pos_ss, const glm::fvec3& pos_ws, const int cam_id) {
		vzm::CameraParameters cam_param;
		vzm::GetCameraParams(cam_id, cam_param);
		if (cam_param.projection_mode == vzm::CameraParameters::ProjectionMode::SLICER_CURVED) {
			float curved_plane_w, curved_plane_h;
			int num_curve_width_pts;
			std::vector<float>* vf_curve_pos_pts = cam_param.script_params.GetParamPtr<std::vector<float>>("ARRAY_CURVE_INTERPOLATION_POS");
			std::vector<float>* vf_curve_up_pts = cam_param.script_params.GetParamPtr<std::vector<float>>("ARRAY_CURVE_INTERPOLATION_UP");
			std::vector<float>* vf_curve_tan_pts = cam_param.script_params.GetParamPtr<std::vector<float>>("ARRAY_CURVE_INTERPOLATION_TANGENT");
			if (!cam_param.script_params.GetParamCheck("CURVED_PLANE_WIDTH", curved_plane_w) ||
				!cam_param.script_params.GetParamCheck("CURVED_PLANE_HEIGHT", curved_plane_h) ||
				!cam_param.script_params.GetParamCheck("COUNT_INTERPOLATION_POINTS", num_curve_width_pts) ||
				vf_curve_pos_pts == NULL || vf_curve_up_pts == NULL || vf_curve_tan_pts == NULL) return false;

			glm::fvec3* curve_tan_pts = (glm::fvec3*)&vf_curve_tan_pts->at(0);
			glm::fvec3* curve_up_pts = (glm::fvec3*)&vf_curve_up_pts->at(0);

			fvec3 pos_on_plane_ws;
			int array_nearest_idx, array_2nd_nearest_idx;
			float ratio_t, height_plane;
			if (!GetNearestPositionOnCurvedPlane(__FP pos_on_plane_ws, __FP pos_ws,
				&vf_curve_pos_pts->at(0), &vf_curve_up_pts->at(0), &vf_curve_tan_pts->at(0), (int)vf_curve_pos_pts->size() / 3, 
				&array_nearest_idx, &array_2nd_nearest_idx, &ratio_t, &height_plane))
				return false;

			float curve_plane_pitch = curved_plane_w / (float)num_curve_width_pts;
			float num_curve_height_pts = curved_plane_h / curve_plane_pitch;
			float num_curve_midheight_pts = num_curve_height_pts * 0.5f;

			fmat4x4 mat_scale = scale(fvec3(curve_plane_pitch));
			fmat4x4 mat_translate = translate(fvec3(-curved_plane_w * 0.5f, -curved_plane_h * 0.5f, -curve_plane_pitch * 0.5f));
			fmat4x4 mat_cos2cws = mat_translate * mat_scale;

			fvec3 pos_cos = fvec3((float)array_nearest_idx + (float)(array_2nd_nearest_idx - array_nearest_idx) * ratio_t, 
				num_curve_midheight_pts + height_plane / curve_plane_pitch, 0);
			fvec3 pos_cws = transformPos(pos_cos, mat_cos2cws);
			fmat4x4 mat_cws2ss;
			vzm::GetCamProjMatrix(cam_id, __FP mat_cws2ss);
			pos_ss = transformPos(pos_cws, mat_cws2ss);
			fvec3 vec_z = pos_ws - pos_on_plane_ws;
			fvec3 right = cam_param.view[2] > 0 ? -curve_tan_pts[array_nearest_idx] : curve_tan_pts[array_nearest_idx];
			fvec3 view = cross(curve_up_pts[array_nearest_idx], right);
			// if panocam view is +z and up is +y, then x dir is left, which means that curve index increases along right to left
			pos_ss.z = dot(vec_z, view)? length(vec_z) : -length(vec_z);
		}
		else {
			fmat4x4 mat_ws2ss;
			vzm::GetCamProjMatrix(cam_id, __FP mat_ws2ss);
			pos_ss = transformPos(pos_ws, mat_ws2ss);
		}

		return true;
	}
	inline bool ComputeDistanceBtw2PointsOnCurvedSlicer(float* distance, const int x1, const int y1, const int x2, const int y2, const int slicer_cam_id) {
		// to do...
		return true;
	}

	inline bool ZoomImageplane(const bool is_zoom_in, float& ip_w, float& ip_h, const float sensitivity = 1.0f) {
		if (is_zoom_in) {
			ip_w *= 0.9f * sensitivity;
			ip_h *= 0.9f * sensitivity;
		}
		else {
			ip_w *= 1.1f * sensitivity;
			ip_h *= 1.1f * sensitivity;
		}
		return true;
	}

	inline float GetStoredValueFromHUValue(const float HU_value, const float* stored_minmax_values, const float* HU_minmax_values) {
		float hu_diff = HU_minmax_values[1] - HU_minmax_values[0];
		float store_diff = stored_minmax_values[1] - stored_minmax_values[0];
		if (hu_diff == 0 || store_diff == 0) return stored_minmax_values[0];
		float slope = store_diff / hu_diff;
		return slope * HU_value + stored_minmax_values[0] - slope * HU_minmax_values[0];
	}

	inline int GetSceneItemIdByName(const std::string& name) {
		std::vector<int> ids;
		vzm::GetSceneItemsByName(name, ids);
		if (ids.size() > 0)
			return ids[0];
		return 0;
	}

	inline void SetGroupActorsVisibilty(int actorId, const bool visible) {
		vector<int> actorsGroup;
		vector<vzm::SceneItemType> actorsSiType;
		vzm::ActorParameters actorParams;
		if (vzm::GetChildSceneItems(actorId, actorsGroup, actorsSiType)) {
			for (int i = 0; i < (int)actorsGroup.size(); i++) {
				if (actorsSiType[i] == vzm::SceneItemType::ACTOR) {
					int childActorId = actorsGroup[i];
					vzm::GetActorParams(childActorId, actorParams);
					actorParams.is_visible = visible;
					vzm::SetActorParams(childActorId, actorParams);

					SetGroupActorsVisibilty(childActorId, visible);
				}
			}
		}
		vzm::GetActorParams(actorId, actorParams);
		actorParams.is_visible = visible;
		vzm::SetActorParams(actorId, actorParams);
	}

	inline int ComputeCrossPointsBtwPlaneAndAlignedCube(const glm::fvec3& aabb_min, const glm::fvec3& aabb_max, const glm::fvec3& pos_plane, const glm::fvec3& vec_plane, std::vector<glm::fvec3>& points) {
		// now compute the intersection points from pos_plane_os and vec_plane_os with aabb
		std::vector<glm::fvec3> unordered_points;
		glm::fvec3 uvec_plane = glm::normalize(vec_plane);
		double d = -uvec_plane.x * pos_plane.x - uvec_plane.y * pos_plane.y - uvec_plane.z * pos_plane.z;

		// Compute Max 6 Points //
		if (uvec_plane.x != 0)
		{
			// 4 Cases //
			glm::fvec2 pos_ortho[4];
			pos_ortho[0] = glm::fvec2(aabb_min.y, aabb_min.z);
			pos_ortho[1] = glm::fvec2(aabb_min.y, aabb_max.z);
			pos_ortho[2] = glm::fvec2(aabb_max.y, aabb_min.z);
			pos_ortho[3] = glm::fvec2(aabb_max.y, aabb_max.z);

			for (int i = 0; i < 4; i++)
			{
				double _x = (-d - uvec_plane.y * pos_ortho[i].x - uvec_plane.z * pos_ortho[i].y) / uvec_plane.x;
				if (aabb_min.x <= _x && aabb_max.x >= _x)
				{
					glm::fvec3 pos_cross = glm::fvec3(_x, pos_ortho[i].x, pos_ortho[i].y);
					unordered_points.push_back(pos_cross);
				}
			}
		}

		if (uvec_plane.y != 0)
		{
			// 4 Cases //
			glm::fvec2 pos_ortho[4];
			pos_ortho[0] = glm::fvec2(aabb_min.x, aabb_min.z);
			pos_ortho[1] = glm::fvec2(aabb_min.x, aabb_max.z);
			pos_ortho[2] = glm::fvec2(aabb_max.x, aabb_min.z);
			pos_ortho[3] = glm::fvec2(aabb_max.x, aabb_max.z);

			for (int i = 0; i < 4; i++)
			{
				double _y = (-d - uvec_plane.x * pos_ortho[i].x - uvec_plane.z * pos_ortho[i].y) / uvec_plane.y;
				if (aabb_min.y <= _y && aabb_max.y >= _y)
				{
					glm::fvec3 pos_cross = glm::fvec3(pos_ortho[i].x, _y, pos_ortho[i].y);
					if (std::find(unordered_points.begin(), unordered_points.end(), pos_cross) == unordered_points.end())
						unordered_points.push_back(pos_cross);
				}
			}
		}

		if (uvec_plane.z != 0)
		{
			// 4 Cases //
			glm::fvec2 pos_ortho[4];
			pos_ortho[0] = glm::fvec2(aabb_min.x, aabb_min.y);
			pos_ortho[1] = glm::fvec2(aabb_min.x, aabb_max.y);
			pos_ortho[2] = glm::fvec2(aabb_max.x, aabb_min.y);
			pos_ortho[3] = glm::fvec2(aabb_max.x, aabb_max.y);

			for (int i = 0; i < 4; i++)
			{
				double dZ = (-d - uvec_plane.x * pos_ortho[i].x - uvec_plane.y * pos_ortho[i].y) / uvec_plane.z;
				if (aabb_min.z <= dZ && aabb_max.z >= dZ)
				{
					glm::fvec3 pos_cross = glm::fvec3(pos_ortho[i].x, pos_ortho[i].y, dZ);
					if (std::find(unordered_points.begin(), unordered_points.end(), pos_cross) == unordered_points.end())
						unordered_points.push_back(pos_cross);
				}
			}
		}

		// Sorting with CW //
		int num_points = (int)unordered_points.size();
		if (num_points < 3)
			return 0;

		glm::fvec3 pos_center;
		for (int i = 0; i < num_points; i++)
		{
			pos_center.x += unordered_points[i].x;
			pos_center.y += unordered_points[i].y;
			pos_center.z += unordered_points[i].z;
		}
		pos_center.x /= num_points;
		pos_center.y /= num_points;
		pos_center.z /= num_points;

		glm::fvec3 edge_vecs[6];
		for (int i = 0; i < num_points; i++)
		{
			edge_vecs[i] = glm::normalize(glm::fvec3(unordered_points[i].x - pos_center.x, unordered_points[i].y - pos_center.y, unordered_points[i].z - pos_center.z));
		}

		// 0 to 2PI
		std::map<float, int> idx_rad;
		for (int i = 1; i < num_points; i++)
		{
			// 1st Compute 0 to PI
			float dot = glm::dot(edge_vecs[0], edge_vecs[i]);
			dot = std::min(dot, 1.f);
			dot = std::max(dot, -1.f);
			float rad = acos(dot);

			// Check if over PI
			glm::fvec3 vec_cross = glm::cross(edge_vecs[0], edge_vecs[i]);

			float dot_cross_normal = glm::dot(vec_cross, uvec_plane);
			dot_cross_normal = std::min(dot_cross_normal, 1.0f);
			dot_cross_normal = std::max(dot_cross_normal, -1.0f);
			const float _pi = glm::pi<float>();
			if (acos(dot_cross_normal) < _pi * 0.5f) // Same Direction : CW case over PI 
			{
				rad = _pi * 2.f - rad;
			}
			if (idx_rad.find(rad) != idx_rad.end())
			{
				rad += 0.0000000001;
			}
			idx_rad[rad] = i;
		}

		points.push_back(unordered_points[0]);
		for (auto it = idx_rad.begin(); it != idx_rad.end(); it++) {
			points.push_back(unordered_points[it->second]);
		}

		return (int)points.size();
	}
	inline int ComputeCrossPointsBtwPlaneAndBox(const vzm::BoxTr& boxtr, const glm::fvec3& pos_plane_ws, const glm::fvec3& vec_plane_ws, std::vector<glm::fvec3>& points_os, std::vector<glm::fvec3>& points_ws) {
		vzm::BoxTr& _boxtr = *(vzm::BoxTr*)&boxtr;
		glm::fmat4x4 mat2unitBox, mat2obBox;
		_boxtr.GetMatrix(__FP mat2unitBox, __FP mat2obBox);

		glm::fvec3 pos_plane_os = transformPos(pos_plane_ws, mat2unitBox);
		glm::fvec3 vec_plane_os = transformPos(vec_plane_ws, mat2unitBox); // consider non uniform scale?!

		glm::fvec3 aabb_min_os(-0.5f, -0.5f, -0.5f), aabb_max_os(0.5f, 0.5f, 0.5f);

		int num_points = ComputeCrossPointsBtwPlaneAndAlignedCube(aabb_min_os, aabb_max_os, pos_plane_os, vec_plane_os, points_os);

		for (int i = 0; i < num_points; i++) {
			points_ws.push_back(transformPos(points_os[i], mat2obBox));
		}

		return num_points;
	}
	inline int ComputeCrossPointsBtwPlaneAndActorCube(const int actor_id, const glm::fvec3& pos_plane_ws, const glm::fvec3& vec_plane_ws, std::vector<glm::fvec3>& points_os, std::vector<glm::fvec3>& points_ws) {
		vzm::ActorParameters _actor;
		vzm::GetActorParams(actor_id, _actor);
		int obj_id = _actor.GetResourceID(vzm::ActorParameters::GEOMETRY);
		//glm::fmat4x4 mat_vs2os;
		//vzm::GetVolumeInfo(vol_id, NULL, NULL, NULL, NULL, NULL, NULL, __FP mat_vs2os, NULL, NULL);
		glm::fmat4x4 os2ws = *(glm::fmat4x4*)_actor.GetWorldTransform();
		glm::fmat4x4 ws2os = glm::inverse(os2ws);

		glm::fvec3 pos_plane_os = transformPos(pos_plane_ws, ws2os);
		glm::fvec3 vec_plane_os = transformPos(vec_plane_ws, ws2os); // consider non uniform scale?!

		glm::fvec3 aabb_min_os, aabb_max_os;
		vzm::GetObjBoundingBox(obj_id, __FP aabb_min_os, __FP aabb_max_os);

		glm::fvec3 old_aabb_min_os = aabb_min_os;
		glm::fvec3 old_aabb_max_os = aabb_max_os;
		aabb_min_os.x = std::min(aabb_min_os.x, old_aabb_min_os.x);
		aabb_min_os.y = std::min(aabb_min_os.y, old_aabb_min_os.y);
		aabb_min_os.z = std::min(aabb_min_os.z, old_aabb_min_os.z);
		aabb_max_os.x = std::max(aabb_max_os.x, old_aabb_max_os.x);
		aabb_max_os.y = std::max(aabb_max_os.y, old_aabb_max_os.y);
		aabb_max_os.z = std::max(aabb_max_os.z, old_aabb_max_os.z);

		int num_points = ComputeCrossPointsBtwPlaneAndAlignedCube(aabb_min_os, aabb_max_os, pos_plane_os, vec_plane_os, points_os);

		for (int i = 0; i < num_points; i++) {
			points_ws.push_back(transformPos(points_os[i], os2ws));
		}

		return num_points;
	}
	inline bool FitSlicerCamera(const std::vector<glm::fvec3>& points_ws, const bool invert_view_points, const glm::fvec3& up, const int fb_w, const int fb_h, vzm::CameraParameters& mpr_cam_params) {
		int num_points = (int)points_ws.size();
		if (num_points < 3) return false;
		fvec3 pos_center = fvec3(0);
		for (const fvec3& pos : points_ws) {
			pos_center += pos;
		}
		pos_center /= (float)num_points;

		fvec3 v0 = points_ws[0] - pos_center;
		fvec3 v1 = points_ws[1] - pos_center;
		fvec3 view = normalize(cross(v0, v1));
		if (invert_view_points) view *= -1.f;

		fvec3 right = normalize(cross(view, up));
		fvec3 cam_up = normalize(cross(right, view));

		fmat4x4 mat_ws2cs = glm::lookAtRH(pos_center, pos_center + view, cam_up);
		fvec2 width_minmax(FLT_MAX, -FLT_MAX), height_minmax(FLT_MAX, -FLT_MAX);
		for (const fvec3& pos : points_ws) {
			fvec3 pos_cs = transformPos(pos, mat_ws2cs);
			width_minmax.x = std::min(width_minmax.x, pos_cs.x);
			width_minmax.y = std::max(width_minmax.y, pos_cs.x);
			height_minmax.x = std::min(height_minmax.x, pos_cs.y);
			height_minmax.y = std::max(height_minmax.y, pos_cs.y);
		}
		float fit_w = width_minmax.y - width_minmax.x;
		float fit_h = height_minmax.y - height_minmax.x;
		fvec3 pos_cam = pos_center + (fit_w * 0.5f + width_minmax.x) * cam_up + (fit_h * 0.5f + height_minmax.x) * right;

		float aspect_ratio = fb_w / fb_h;
		float ip_w, ip_h;
		if (aspect_ratio < fit_w / fit_h)
		{
			ip_w = fit_w;
			ip_h = ip_w / aspect_ratio;
		}
		else
		{
			ip_h = fit_h;
			ip_w = ip_h * aspect_ratio;
		}

		mpr_cam_params.w = fb_w;
		mpr_cam_params.h = fb_h;
		*(fvec3*)mpr_cam_params.pos = pos_cam;
		*(fvec3*)mpr_cam_params.up = cam_up;
		*(fvec3*)mpr_cam_params.view = view;
		mpr_cam_params.ip_w = ip_w;
		mpr_cam_params.ip_h = ip_h;
		return true;
	}
	inline bool ComputeScreenPointsOnPanoSlicer(const int panoCamId, std::vector<glm::fvec2>& points_ss) {
		vzm::CameraParameters panoCamParams;
		vzm::GetCameraParams(panoCamId, panoCamParams);
		if (panoCamParams.projection_mode != vzm::CameraParameters::SLICER_CURVED)
			return false;

		float curvedW = panoCamParams.script_params.GetParam("CURVED_PLANE_WIDTH", 0.f);
		float curvedH = panoCamParams.script_params.GetParam("CURVED_PLANE_HEIGHT", 0.f);
		if (curvedW == 0.f || curvedH == 0.f)
			return false;

		glm::fvec3 posCWS_Cam(0);// = *(glm::fvec3*)panoCamParams.pos;
		glm::fvec3 posCWS_Up = *(glm::fvec3*)panoCamParams.up;
		glm::fvec3 posCWS_View = *(glm::fvec3*)panoCamParams.view;
		glm::fvec3 vecCWS_Right = glm::cross(posCWS_View, posCWS_Up);

		glm::fvec3 posCWS_TL = posCWS_Cam + posCWS_Up * curvedH * 0.5f - vecCWS_Right * curvedW * 0.5f;
		glm::fvec3 posCWS_TR = posCWS_Cam + posCWS_Up * curvedH * 0.5f + vecCWS_Right * curvedW * 0.5f;
		glm::fvec3 posCWS_BL = posCWS_Cam - posCWS_Up * curvedH * 0.5f - vecCWS_Right * curvedW * 0.5f;
		glm::fvec3 posCWS_BR = posCWS_Cam - posCWS_Up * curvedH * 0.5f + vecCWS_Right * curvedW * 0.5f;
		
		glm::fmat4x4 matPanoCWS2SS; // note this reflects cam pos!
		vzm::GetCamProjMatrix(panoCamId, __FP matPanoCWS2SS);

		glm::fvec3 posSS_TL = vzmutils::transformPos(posCWS_TL, matPanoCWS2SS);
		glm::fvec3 posSS_TR = vzmutils::transformPos(posCWS_TR, matPanoCWS2SS);
		glm::fvec3 posSS_BL = vzmutils::transformPos(posCWS_BL, matPanoCWS2SS);
		glm::fvec3 posSS_BR = vzmutils::transformPos(posCWS_BR, matPanoCWS2SS);

		points_ss.reserve(4);
		points_ss.push_back(posSS_TL);
		points_ss.push_back(posSS_TR);
		points_ss.push_back(posSS_BR);
		points_ss.push_back(posSS_BL);

		return true;
	}

	inline bool ComputeSrcActorOS2DstActorOS(const int srcActorId, const int dstActorId, glm::fmat4x4& matSrcOS2DstOS) {
		vzm::ActorParameters srcActorParams, dstActorParams;
		if (!vzm::GetActorParams(srcActorId, srcActorParams) || !vzm::GetActorParams(dstActorId, dstActorParams))
			return false;

		int srcSceneId, dstSceneId, tmpId;
		vzm::SceneItemType itemType;
		vzm::GetSceneRootItem(srcActorId, srcSceneId, tmpId, itemType);
		vzm::GetSceneRootItem(dstActorId, dstSceneId, tmpId, itemType);
		if (srcSceneId != dstSceneId) {
			printf("WARNNING: both scene Id of the src and dst actors must be same!!\n");
			return false;
		}
		vzm::UpdateSceneTransforms(srcSceneId);
		glm::fmat4x4 matSrcOS2WS = *(glm::fmat4x4*)srcActorParams.GetWorldTransform();
		glm::fmat4x4 matDstOS2WS = *(glm::fmat4x4*)dstActorParams.GetWorldTransform();
		glm::fmat4x4 matWS2DstOS = glm::inverse(matDstOS2WS);
		matSrcOS2DstOS = matWS2DstOS * matSrcOS2WS;
		return true;
	}

	inline bool ComputeOriginalVolPosFromPanoVolPos(glm::fvec3& posOriginVol, const glm::fvec3& posPanoVol,
		const int originVolActorId, const int panoVolActorId,
		const float curved_plane_w, const float curved_plane_h, 
		const float* curve_pos_pts, const float* curve_up_pts, const float* curve_tan_pts, 
		const int num_curve_pts, const float planeThickness){

		vzm::ActorParameters oriActorParams, panoActorParams;
		if (!vzm::GetActorParams(originVolActorId, oriActorParams) || !vzm::GetActorParams(panoVolActorId, panoActorParams))
			return false;

		int oriVolResId = oriActorParams.GetResourceID(vzm::ActorParameters::GEOMETRY);
		int panoVolResId = panoActorParams.GetResourceID(vzm::ActorParameters::GEOMETRY);
		if (oriVolResId == 0 || panoVolResId == 0)
			return false;

		// get parameters used in pano volume generation 
		float planePitch = curved_plane_w / (float)num_curve_pts;
		float planeSizeY = curved_plane_h / planePitch;
		float planeCenterY = curved_plane_h * 0.5f;

		int thickSteps = std::max(int(planeThickness / planePitch + 0.5f), (int)1); /* Think "planeThickness > minPitch"!! */
		float sampleDist = planeThickness / (float)thickSteps;

		int sliceDepth = thickSteps;
		int sliceHeight = (int)planeSizeY;
		int sliceWidth = (int)num_curve_pts;

		glm::ivec3 panoVolSize;
		vzm::GetVolumeInfo(panoVolResId, NULL, (int*)&panoVolSize, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
		if (panoVolSize.x != sliceWidth || panoVolSize.y != sliceHeight || panoVolSize.z != sliceDepth)
			return false;

		if (posPanoVol.x < 0 || posPanoVol.y < 0 || posPanoVol.z < 0
			|| posPanoVol.x >= (float)(sliceWidth - 1) || posPanoVol.y >= (float)(sliceHeight - 1) || posPanoVol.z >= (float)(sliceDepth - 1))
			return false;

		glm::fvec3* curvePosPts = (glm::fvec3*)curve_pos_pts;
		glm::fvec3* curveUpPts = (glm::fvec3*)curve_up_pts;
		glm::fvec3* curveTanPts = (glm::fvec3*)curve_tan_pts;

		// compute pano vol to world
		int posX = (int)posPanoVol.x;
		float ratioX = posPanoVol.x - (float)posX;
		glm::fvec3 posPanoCenterPosWS = (1.f - ratioX) * curvePosPts[posX] + ratioX * curvePosPts[std::min(posX + 1, sliceWidth - 1)];
		glm::fvec3 upWS = glm::normalize((1.f - ratioX) * curveUpPts[posX] + ratioX * curveUpPts[std::min(posX + 1, sliceWidth - 1)]);
		glm::fvec3 tanWS = glm::normalize((1.f - ratioX) * curveTanPts[posX] + ratioX * curveTanPts[std::min(posX + 1, sliceWidth - 1)]);
		glm::fvec3 dirWS = glm::normalize(glm::cross(tanWS, upWS));

		glm::fvec3 posPanoStartPosWS = posPanoCenterPosWS - upWS * planeCenterY - planeThickness * dirWS * 0.5f;

		glm::fvec3 dirSampleWS = dirWS * sampleDist;
		glm::fvec3 upSampleWS = upWS * planePitch;
		glm::fvec3 posPanoSampleWS = posPanoStartPosWS + upSampleWS * (float)posPanoVol.y + dirSampleWS * (float)posPanoVol.z;

		glm::fmat4x4 matOS2WS = *(glm::fmat4x4*)oriActorParams.GetWorldTransform();
		glm::fmat4x4 matWS2OS = glm::inverse(matOS2WS);

		glm::ivec3 oriVolSize; 
		glm::fmat4x4 matVS2OS;
		vzm::GetVolumeInfo(oriVolResId, NULL, (int*)&oriVolSize, NULL, NULL, NULL, NULL, __FP matVS2OS, NULL, NULL);
		glm::fmat4x4 matOS2VS = glm::inverse(matVS2OS);

		glm::fmat4x4 matWS2VS = matOS2VS * matWS2OS; // note col-major
		
		posOriginVol = transformPos(posPanoSampleWS, matWS2VS);
		
		return true;
	}

	struct GeneralMove {
	private:
		helpers::arcball aball_vr;
		helpers::slicer_drag slicer_mouse;
		glm::ivec2 pos_ss_prev;
		glm::fvec3 __scene_stage_center;
		float __scene_stage_scale = 150.f;
	public:
		inline bool Start(const int* pos_ss, const vzm::CameraParameters& cam_params, const glm::fvec3 scene_stage_center = glm::fvec3(), const float scene_stage_scale = 150.f) {
			__scene_stage_scale = scene_stage_scale;
			__scene_stage_center = scene_stage_center;
			glm::fvec2 screen_size = glm::fvec2(cam_params.w, cam_params.h);
			if (cam_params.projection_mode == vzm::CameraParameters::SLICER_PLANE
				|| cam_params.projection_mode == vzm::CameraParameters::SLICER_CURVED) 
			{
				slicer_mouse.intializer((float*)&__scene_stage_center); // https://github.com/korfriend/OsstemCoreAPIs/discussions/146#discussion-4329688
				helpers::cam_pose slicer_cam_pose;
				glm::fvec3 pos = *(glm::fvec3*)slicer_cam_pose.pos = *(glm::fvec3*)cam_params.pos;
				*(glm::fvec3*)slicer_cam_pose.up = *(glm::fvec3*)cam_params.up;
				*(glm::fvec3*)slicer_cam_pose.view = *(glm::fvec3*)cam_params.view;
				slicer_mouse.start(pos_ss, (float*)&screen_size, slicer_cam_pose, cam_params.ip_w, cam_params.ip_h, 1.0);
			}
			else {
				aball_vr.intializer((float*)&__scene_stage_center, __scene_stage_scale);
				helpers::cam_pose arc_cam_pose;
				glm::fvec3 pos = *(glm::fvec3*)arc_cam_pose.pos = *(glm::fvec3*)cam_params.pos;
				*(glm::fvec3*)arc_cam_pose.up = *(glm::fvec3*)cam_params.up;
				*(glm::fvec3*)arc_cam_pose.view = *(glm::fvec3*)cam_params.view;
				aball_vr.start(pos_ss, (float*)&screen_size, arc_cam_pose);
			}
			pos_ss_prev = *(glm::ivec2*)pos_ss;
			return true;
		}
		inline bool RotateMove(const int* pos_ss, vzm::CameraParameters& cam_params) {
			if (cam_params.projection_mode == vzm::CameraParameters::SLICER_PLANE
				|| cam_params.projection_mode == vzm::CameraParameters::SLICER_CURVED) return false;

			helpers::cam_pose arc_cam_pose;
			aball_vr.move(pos_ss, arc_cam_pose);

			*(glm::fvec3*)cam_params.pos = *(glm::fvec3*)arc_cam_pose.pos;
			*(glm::fvec3*)cam_params.up = *(glm::fvec3*)arc_cam_pose.up;
			*(glm::fvec3*)cam_params.view = *(glm::fvec3*)arc_cam_pose.view;
			pos_ss_prev = *(glm::ivec2*)pos_ss;
			return true;
		}
		inline bool PanMove(const int* pos_ss, vzm::CameraParameters& cam_params) {
			if (cam_params.projection_mode == vzm::CameraParameters::SLICER_PLANE
				|| cam_params.projection_mode == vzm::CameraParameters::SLICER_CURVED)
			{
				helpers::cam_pose slicer_cam_pose;
				slicer_mouse.pan_move(pos_ss, slicer_cam_pose);
				*(glm::fvec3*)cam_params.pos = *(glm::fvec3*)slicer_cam_pose.pos;
				*(glm::fvec3*)cam_params.up = *(glm::fvec3*)slicer_cam_pose.up;
				*(glm::fvec3*)cam_params.view = *(glm::fvec3*)slicer_cam_pose.view;
			}
			else {
				helpers::cam_pose arc_cam_pose;
				aball_vr.pan_move(pos_ss, arc_cam_pose);

				*(glm::fvec3*)cam_params.pos = *(glm::fvec3*)arc_cam_pose.pos;
				*(glm::fvec3*)cam_params.up = *(glm::fvec3*)arc_cam_pose.up;
				*(glm::fvec3*)cam_params.view = *(glm::fvec3*)arc_cam_pose.view;
			}
			pos_ss_prev = *(glm::ivec2*)pos_ss;
			return true;
		}
		inline bool ZoomMove(const int* pos_ss, vzm::CameraParameters& cam_params, const bool zoomDir_convert, const bool preserveStageCenter = false) {
			if (cam_params.projection_mode == vzm::CameraParameters::SLICER_PLANE
				|| cam_params.projection_mode == vzm::CameraParameters::SLICER_CURVED)
			{
				float ip_w, ip_h;
				slicer_mouse.zoom_move(pos_ss, zoomDir_convert, ip_w, ip_h, preserveStageCenter ? cam_params.pos : NULL);
				cam_params.ip_w = ip_w;
				cam_params.ip_h = ip_h;
			}
			else {
				bool is_ortho_proj = cam_params.script_params.GetParam("ORTHOGONAL_PROJECTION", false);
				int diff_yss = pos_ss[1] - pos_ss_prev.y;
				if (cam_params.projection_mode == vzm::CameraParameters::IMAGEPLANE_SIZE && is_ortho_proj) {
					if(diff_yss * diff_yss > 1)
						ZoomImageplane(zoomDir_convert? diff_yss < 0 : diff_yss > 0, cam_params.ip_w, cam_params.ip_h);
				}
				else {
					// by adjusting cam distance
					if (zoomDir_convert) diff_yss *= -1;
					if (diff_yss * diff_yss > 1) {
						if (diff_yss > 0)
							*(glm::fvec3*)cam_params.pos += __scene_stage_scale * 0.01f * (*(glm::fvec3*)cam_params.view);
						else
							*(glm::fvec3*)cam_params.pos -= __scene_stage_scale * 0.01f * (*(glm::fvec3*)cam_params.view);
					}
				}
			}
			pos_ss_prev = *(glm::ivec2*)pos_ss;
			return true;
		}
	};
}