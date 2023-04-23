/*
Copyright (c) 2019, Dongjoon Kim & OSSTEM 
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#pragma once
#define __dojostatic extern "C" __declspec(dllexport)
#define __dojoclass class __declspec(dllexport)
#define __dojostruct struct __declspec(dllexport)

#define __FP (float*)&

#include <string>
#include <map>
#include <vector>
#include <set>
#include <any>
#include <list>
#include <windows.h>

#define SAFE_GET_COPY(DST_PTR, SRC_PTR, TYPE, ELEMENTS) { if(DST_PTR) memcpy(DST_PTR, SRC_PTR, sizeof(TYPE)*ELEMENTS); }
#define GET_COPY(DST_PTR, SRC_PTR, TYPE, ELEMENTS) { memcpy(DST_PTR, SRC_PTR, sizeof(TYPE)*ELEMENTS); }

namespace helpers
{
	__dojostatic void TransformPoint(const float* pos_src, const float* mat, const bool is_rowMajor, float* pos_dst);
	__dojostatic void TransformVector(const float* vec_src, const float* mat, const bool is_rowMajor, float* vec_dst);
	__dojostatic void ComputeBoxTransformMatrix(const float* cube_scale, const float* pos_center, const float* y_axis, const float* z_axis, const bool is_rowMajor, float* mat_tr, float* inv_mat_tr);
}

namespace vzm
{
	template <typename NAME> struct ParamMap {
	private:
		std::string __PM_VERSION = "LIBI_1.3";
		std::map<NAME, std::any> __params;
	public:
		template <typename SRCV> bool GetParamCheck(const NAME& param_name, SRCV& param) {
			auto it = __params.find(param_name);
			if (it == __params.end()) return false;
			param = std::any_cast<SRCV&>(it->second);
			return true;
		}
		template <typename SRCV> SRCV GetParam(const NAME& param_name, const SRCV& init_v) {
			auto it = __params.find(param_name);
			if (it == __params.end()) return init_v;
			return std::any_cast<SRCV&>(it->second);
		}
		template <typename SRCV> SRCV* GetParamPtr(const NAME& param_name) {
			auto it = __params.find(param_name);
			if (it == __params.end()) return NULL;
			return (SRCV*)&std::any_cast<SRCV&>(it->second);
		}
		template <typename SRCV, typename DSTV> bool GetParamCastingCheck(const NAME& param_name, DSTV& param) {
			auto it = __params.find(param_name);
			if (it == __params.end()) return false;
			param = (DSTV)std::any_cast<SRCV&>(it->second);
			return true;
		}
		template <typename SRCV, typename DSTV> DSTV GetParamCasting(const NAME& param_name, const DSTV& init_v) {
			auto it = __params.find(param_name);
			if (it == __params.end()) return init_v;
			return (DSTV)std::any_cast<SRCV&>(it->second);
		}
		void SetParam(const NAME& param_name, const std::any& param) {
			__params[param_name] = param;
		}
		void RemoveParam(const NAME& param_name) {
			auto it = __params.find(param_name);
			if (it != __params.end()) {
				__params.erase(it);
			}
		}
		void RemoveAll() {
			__params.clear();
		}
		size_t Size() {
			return __params.size();
		}
		std::string GetPMapVersion() {
			return __PM_VERSION;
		}

		typedef std::map<NAME, std::any> MapType;
		typename typedef MapType::iterator iterator;
		typename typedef MapType::const_iterator const_iterator;
		typename typedef MapType::reference reference;
		iterator begin() { return __params.begin(); }
		const_iterator begin() const { return __params.begin(); }
		iterator end() { return __params.end(); }
		const_iterator end() const { return __params.end(); }
	};

	enum class ResObjType {
		UNDEFINED = 0,
		PRIMITIVE = 1, // resource object
		VOLUME = 2, // resource object
		BUFFER = 3, // resource object
	};
	enum class SceneItemType {
		UNDEFINED = 0,
		SCENE, 
		ACTOR,
		CAMERA,
		LIGHT,
	};
	struct BoxTr
	{
	private:
		// from unit cube centered at origin
		float __cube_size[3] = { 1.f, 1.f, 1.f };
		float __pos_center[3] = { 0, 0, 0 };
		float __y_axis[3] = { 0, 1.f, 0 };
		float __z_axis[3] = { 0, 0, 1.f };
		float __mat_tr[16] = { 1.f, 0, 0, 0, 0, 1.f, 0, 0, 0, 0, 1.f, 0, 0, 0, 0, 1.f }; // target-sized box to unit axis-aligned box
		float __inv_mat_tr[16] = { 1.f, 0, 0, 0, 0, 1.f, 0, 0, 0, 0, 1.f, 0, 0, 0, 0, 1.f }; // inverse
		bool __is_rowMajor = false;
	public:
		void SetBoxTr(const float* cube_size, const float* pos_center, const float* y_axis, const float* z_axis, const bool is_rowMajor = false) {
			SAFE_GET_COPY(__cube_size, cube_size, float, 3);
			SAFE_GET_COPY(__pos_center, pos_center, float, 3);
			SAFE_GET_COPY(__y_axis, y_axis, float, 3);
			SAFE_GET_COPY(__z_axis, z_axis, float, 3);
			helpers::ComputeBoxTransformMatrix(cube_size, pos_center, y_axis, z_axis, is_rowMajor, __mat_tr, __inv_mat_tr);
		}
		// mat_tr : to aligned unit-cube
		// inv_mat_tr : to oblique cube
		void GetMatrix(float* mat_tr, float* inv_mat_tr) {
			SAFE_GET_COPY(mat_tr, __mat_tr, float, 16);
			SAFE_GET_COPY(inv_mat_tr, __inv_mat_tr, float, 16);
		}
		void GetCubeInfo(float* cube_size, float* pos_center, float* y_axis, float* z_axis) {
			SAFE_GET_COPY(cube_size, __cube_size, float, 3);
			SAFE_GET_COPY(pos_center, __pos_center, float, 3);
			SAFE_GET_COPY(y_axis, __y_axis, float, 3);
			SAFE_GET_COPY(z_axis, __z_axis, float, 3);
		}
	};

	struct CameraParameters
	{
		// note that those lookAt parameters are used for LOCAL matrix
		float pos[3] = { 0, 0, 0 };
		float view[3] = { 0, 0, -1.f };
		float up[3] = { 0, 1.f, 0 }; // default..

		void SetLookAt(const float* look_at) { // note that pos must be set before calling this
			view[0] = look_at[0] - pos[0]; view[1] = look_at[1] - pos[1]; view[2] = look_at[2] - pos[2];
			float length = sqrt(view[0] * view[0] + view[1] * view[1] + view[2] * view[2]);
			if (length > 0) { view[0] /= length; view[1] /= length; view[2] /= length; }
		}
		void SetCameraByMatrix(const float* mat_default2lookAt, const bool is_rowMajor = false) {
			float __pos[3] = { 0, 0, 0 }, __view[3] = { 0, 0, -1.f }, __up[3] = { 0, 1.f, 0 };
			helpers::TransformPoint(__pos, mat_default2lookAt, is_rowMajor, pos);
			helpers::TransformVector(__view, mat_default2lookAt, is_rowMajor, view);
			helpers::TransformVector(__up, mat_default2lookAt, is_rowMajor, up);
		};
		void ApplyCameraByMatrix(const float* mat_tr, const bool is_rowMajor = false) {
			helpers::TransformPoint(pos, mat_tr, is_rowMajor, pos);
			helpers::TransformVector(view, mat_tr, is_rowMajor, view);
			helpers::TransformVector(up, mat_tr, is_rowMajor, up);
		};

		enum ProjectionMode {
			UNDEFINED = 0,
			IMAGEPLANE_SIZE = 1, // use ip_w, ip_h instead of fov_y
			CAMERA_FOV = 2, // use fov_y, aspect_ratio
			CAMERA_INTRINSICS = 3, // AR mode
			SLICER_PLANE = 4, // mpr sectional mode
			SLICER_CURVED = 5, // pano sectional mode
		};
		enum VolumeRayCastMode {
			OPTICAL_INTEGRATION = 0,
			OPTICAL_INTEGRATION_MULTI_OTF = 23,
			OPTICAL_INTEGRATION_TRANSPARENCY = 1,
			OPTICAL_INTEGRATION_MULTI_OTF_TRANSPARENCY = 2,
			OPTICAL_INTEGRATION_SCULPT_MASK = 22,
			OPTICAL_INTEGRATION_SCULPT_MASK_TRANSPARENCY = 25,
			VOLMASK_VISUALIZATION = 24,
			MAXIMUM_INTENSITY = 10,
			MINIMUM_INTENSITY = 11,
			AVERAGE_INTENSITY = 12
		};
		ProjectionMode projection_mode = ProjectionMode::UNDEFINED;
		VolumeRayCastMode volraycast_mode = VolumeRayCastMode::OPTICAL_INTEGRATION;
		union {
			struct { // projection_mode == 1 or 4 or 5
				float ip_w;
				float ip_h; // defined in CS
			};
			struct { // projection_mode == 2
				// fov_y should be smaller than PI
				float fov_y;
				float aspect_ratio; // ip_w / ip_h
			};
			struct { // projection_mode == 3
				// AR mode (normal camera intrinsics..)
				float fx;
				float fy;
				float sc;
				float cx;
				float cy;
			};
		};
		float np = 0.01f;
		float fp = 1000.f; // the scale difference is recommended : ~100000 (in a single precision (float))
		int w = 0;
		int h = 0; // resolution. note that the aspect ratio is recomputed w.r.t. w and h during the camera setting.
		bool is_rgba_write = false; // if false, use BGRA order
		HWND hWnd = NULL; // if NULL, offscreen rendering is performed

		ParamMap<std::string> script_params;
		std::set<int> hidden_actors;
		void SetCurvedSlicer(const float curved_plane_w, const float curved_plane_h, const float* curve_pos_pts, const float* curve_up_pts, const float* curve_tan_pts, const int num_curve_pts) {
			script_params.SetParam("CURVED_PLANE_WIDTH", curved_plane_w);
			script_params.SetParam("CURVED_PLANE_HEIGHT", curved_plane_h);
			std::vector<float> vf_curve_pos_pts(num_curve_pts * 3), vf_curve_up_pts(num_curve_pts * 3), vf_curve_tan_pts(num_curve_pts * 3);
			memcpy(&vf_curve_pos_pts[0], curve_pos_pts, sizeof(float) * 3 * num_curve_pts);
			memcpy(&vf_curve_up_pts[0], curve_up_pts, sizeof(float) * 3 * num_curve_pts);
			memcpy(&vf_curve_tan_pts[0], curve_tan_pts, sizeof(float) * 3 * num_curve_pts);
			script_params.SetParam("COUNT_INTERPOLATION_POINTS", num_curve_pts);
			script_params.SetParam("ARRAY_CURVE_INTERPOLATION_POS", vf_curve_pos_pts);
			script_params.SetParam("ARRAY_CURVE_INTERPOLATION_UP", vf_curve_up_pts);
			script_params.SetParam("ARRAY_CURVE_INTERPOLATION_TANGENT", vf_curve_tan_pts);
		}
		void SetOrthogonalProjection(const bool orthoproj_mode) {
			// only available when projection_mode == IMAGEPLANE_SIZE
			script_params.SetParam("ORTHOGONAL_PROJECTION", orthoproj_mode);
		}
		void SetSlicerThickness(const float thickness) {
			// only available when projection_mode == SLICER_PLANE or SLICER_CURVED
			script_params.SetParam("SLICER_THICKNESS", thickness);
		}
		void StoreSlicerCutLines(const bool is_store) {
			// only available when projection_mode == SLICER_PLANE or SLICER_CURVED
			script_params.SetParam("STORE_SLICERCUTLINES", is_store);
		}
		void Set2xVolumeRayCaster(const bool enable) {
			// only available when projection_mode == SLICER_PLANE or SLICER_CURVED
			script_params.SetParam("FAST_VOLRAYCASTER2X", enable);
		}
		void HideActor(const int actor_id) {
			hidden_actors.insert(actor_id);
		}
		void DeactiveHiddenActor(const int actor_id) {
			auto it = hidden_actors.find(actor_id);
			if(it != hidden_actors.end())
				hidden_actors.erase(it);
		}
	};

	// note that object id must be unique in a scene!!
	struct ActorParameters
	{
	public:
		enum RES_USAGE
		{
			GEOMETRY, // main volume or primitives
			VR_OTF,
			MPR_WINDOWING,
			COLOR_MAP,
			TEXTURE_VOLUME, 
			MASK_VOLUME, // only for volume rendering ... multi-OTF
		};
	private:
		bool is_rowMajor = false;
		bool use_localTrans = true;
		float __os2ls[16] = { 1.f, 0, 0, 0, 0, 1.f, 0, 0, 0, 0, 1.f, 0, 0, 0, 0, 1.f }; // object space to local space (used in hierarchical tree structure)
		float __os2ws[16] = { 1.f, 0, 0, 0, 0, 1.f, 0, 0, 0, 0, 1.f, 0, 0, 0, 0, 1.f }; // 4x4 matrix col-major (same as in glm::fmat4x4)
		ParamMap<RES_USAGE> associated_obj_ids; // <usage, obj_id> 

	public:
		bool IsLocalTransform() { return use_localTrans; }
		void SetWorldTransform(const float* os2ws) {
			memcpy(__os2ws, os2ws, sizeof(float) * 16); use_localTrans = false;
		};
		void UpdateWorldTransform(const float* os2ws) {
			memcpy(__os2ws, os2ws, sizeof(float) * 16);
		}
		const float* GetWorldTransform() const { return __os2ws; };

		// those local interfaces will be implemented when supporting group interfaces! //
		void SetLocalTransform(const float* os2ls) {
			memcpy(__os2ls, os2ls, sizeof(float) * 16); use_localTrans = true;
		};
		const float* GetLocalTransform() const { return __os2ls; };
		
		// materials and expressions 
		// note that those parameters are not passed on to children (of a tree node)
		float phong_coeffs[4] = { 0.3f, 0.4f, 0.4, 1000.f }; // ambient, diffuse, specular, highlight
		bool is_visible = true;
		bool is_pickable = false;
		float color[4] = {1.f, 1.f, 1.f, 1.f}; // rgba [0,1]
		
		int GetResourceID(const RES_USAGE res_usage) {
			return associated_obj_ids.GetParam(res_usage, (int)0);
		}
		void SetResourceID(const RES_USAGE res_usage, const int obj_id) {
			associated_obj_ids.SetParam(res_usage, obj_id);
		}

		// primitive 3D only
		bool use_vertex_color = true; // use vertex color instead of color[0,1,2], if vertex buffer contains color information. note that color[3] is always used for the transparency
		float point_thickness = 0; // (diameter in pixels) when the object is defined as a point cloud without surfels, using 1 pixel 
		float surfel_size = 0; // (diameter in world unit) when the object is defined as a point cloud with surfels, using 0.002 size of object boundary
		bool represent_points_to_surfels = true; // available when the object is defined as a point cloud
		float line_thickness = 0; // (pixels) available when the object is defined as line primitives and not available for wire frame lines, using 1 pixel 
		bool is_wireframe = false; // available when the object is a polygonal mesh
		bool use_vertex_wirecolor = false; // use vertex color instead of wire_color[0,1,2], if vertex buffer contains color information. note that color[3] is always used for the transparency
		float wire_color[4] = {1.f, 1.f, 1.f, 1.f}; // rgba [0,1].. only for wireframe object

		// volume 3D only
		//float sample_rate = 1.f; // NA in this version

		ParamMap<std::string> script_params;
		ParamMap<std::string> test_params; // direct mapping to VmActor
		void SetOutline(const int thick_pixs, const float depth_thres, const float* outline_color_rgb) {
			// if thick_pixs == 0, no outline
			script_params.SetParam("SILHOUETTE_THICKNESS", thick_pixs);
			script_params.SetParam("SILHOUETTE_DEPTH_THRES", depth_thres);
			if (outline_color_rgb != NULL) {
				std::vector<float> color = { outline_color_rgb[0], outline_color_rgb[1], outline_color_rgb[2] };
				script_params.SetParam("SILHOUETTE_COLOR_RGB", color);
			}
		}
		void SetClipBox(const vzm::BoxTr* clipBox) {
			if (clipBox) script_params.SetParam("CLIPSETTING_BOX", *clipBox);
			else script_params.RemoveParam("CLIPSETTING_BOX");
		}
		void SetClipPlane(const float* pos_vec_plane) {
			if (pos_vec_plane) {
				std::vector<float> _pos_vec_plane = { pos_vec_plane[0], pos_vec_plane[1], pos_vec_plane[2], pos_vec_plane[3], pos_vec_plane[4], pos_vec_plane[5] };
				script_params.SetParam("CLIPSETTING_PLANE", _pos_vec_plane);
			}
			else script_params.RemoveParam("CLIPSETTING_PLANE");
		}
		void SetGeoOS2VolOS(const float* mat) {
			std::vector<float> matValues(16);
			memcpy(&matValues[0], mat, sizeof(float) * 16);
			script_params.SetParam("MATRIX_GeoOS2VolOS", matValues);
		}
		void SetSculptIndex(const int sculptIndex) {
			script_params.SetParam("SCULPT_INDEX", sculptIndex);
		}

		bool GetClipBox(vzm::BoxTr& clipBox) {
			return script_params.GetParamCheck("CLIPSETTING_BOX", clipBox);
		}

		bool GetClipPlane(std::vector<float>& pos_vec_plane) {
			return script_params.GetParamCheck("CLIPSETTING_PLANE", pos_vec_plane);
		}
	};

	struct LightParameters
	{
		//float pos_light[3] = { 0, 0, 0 }, dir_light[3] = { 0, 0, -1.f };
		float pos[3] = { 0, 0, 0 };
		float dir[3] = { 0, 0, -1.f };
		float up[3] = { 0, 1.f, 0 }; // Local coordinates
		bool is_pointlight = false; // 'true' uses pos_light, 'false' uses dir_light
		bool is_on_camera = true; // 'true' sets cam params to light source. in this case, it is unnecessary to set pos, dir, and up (ignore these)
		//vmfloat3 ambient_color = vmfloat3(1.f);
		//vmfloat3 diffuse_color = vmfloat3(1.f);
		//vmfloat3 specular_color = vmfloat3(1.f);
		struct SSAO_Params
		{
			bool is_on_ssao = false;
			float kernel_r = 0;
			float ao_power = 1.f;
			float tangent_bias = 3.141592654f / 6.f;;
			int num_dirs = 8;
			int num_steps = 8;
			bool smooth_filter;
		}; 
		SSAO_Params effect_ssao;
	};

	__dojostatic bool InitEngineLib(const std::string& coreName = "VizMotive", const std::string& logFileName = "EngineApi.log");
	__dojostatic bool DeinitEngineLib();

	__dojostatic std::string GetEngineAPIsVer();
	__dojostatic std::string GetEngineCoreVer();
	__dojostatic ResObjType GetResObjType(const int obj_id);

	//////////////////////////////////
	// 	   Resource Objects
	// here, obj_id (without const) is [in/out].. in : when a registered object of obj_id exists, out : when there is no registered object of obj_id
	__dojostatic bool LoadModelFile(const std::string& filename, int& obj_id, const bool unify_redundancy = false);
	__dojostatic bool LoadMaskVolumeFile(const std::string& filename, const int ref_vol_obj, const std::string& data_type, int& obj_id, const bool reverse_zdir = false);
	__dojostatic bool LoadMultipleModelsFile(const std::string& filename, std::vector<int>& obj_ids, const bool unify_redundancy = false);
	// data_type "CHAR" "BYTE" "SHORT" "USHORT" "INT" "FLOAT"
	__dojostatic bool GenerateEmptyVolume(int& vol_id, const int ref_vol_id = 0, const std::string& data_type = "", const double min_v = 0, const double max_v = 0, const double fill_v = 0);
	// note that redundant safe-bnd to slices is not allowed
	// data_type "CHAR" "BYTE" "SHORT" "USHORT" "INT" "FLOAT"
	__dojostatic bool GenerateVolumeFromData(int& vol_id, const void** vol_slices_2darray, const std::string& data_type, const int* size_xyz, const float* pitch_xyz, const float* axis_x_os, const float* axis_y_os, const bool is_rhs, const bool is_safe_bnd);
	__dojostatic bool GenerateEmptyPrimitive(int& prim_id);
	__dojostatic bool GenerateArrowObject(const float* pos_s, const float* pos_e, const float radius_body, const float radius_head, int& obj_id);
	// optional : rgb_list (if NULL, this is not used)
	__dojostatic bool GenerateSpheresObject(const float* xyzr_list, const float* rgb_list, const int num_spheres, int& obj_id);
	__dojostatic bool GenerateCubesObject(const float* xyz_list, const float* whd_list, const float* rgb_list, const int num_cubes, const bool edgelines, int& obj_id);
	__dojostatic bool GenerateCylindersObject(const float* xyz_01_list, const float* radius_list, const float* rgb_list, const int num_cylinders, int& obj_id);
	__dojostatic bool GenerateAxisHelperObject(int& obj_id, const float axis_length, const float* xAxis_yAxis_zAxis = NULL);
	__dojostatic bool GenerateCurvedTubeObject(const float* xyz_list, const int num_points, const float radius, int& obj_id);
	__dojostatic bool GeneratePolygonalPlaneObject(const float* xyz_list, const float* rgb_list, const int num_points, const bool is_solid, int& obj_id);
	__dojostatic bool GeneratePanoHelperObjects(const float* xyz_list, const float* up, const float height, const int num_points, int& panoplane_obj_id, int& centerline_obj_id, int& outline_obj_id);
	// when line_thickness = 0, the line thickness is not used, depending on the line renderer of the rendering API.
	__dojostatic bool GenerateLinesObject(const float* xyz_01_list, const float* rgb_01_list, const int num_lines, int& obj_id);
	__dojostatic bool GenerateTrianglesObject(const float* xyz_012_list, const float* rgb_012_list, const int num_tris, int& obj_id);
	// optional : nrl_list, rgb_list, tex_list, idx_prims (if NULL, this is not used)
	// stride_prim_idx : 1 ==> point cloud, 2 ==> line, 3 ==> triangle
	__dojostatic bool GeneratePrimitiveObject(const float* xyz_list, const float* nrl_list, const float* rgb_list, const float* tex_list, const int num_vtx, const unsigned int* idx_prims, const int num_prims, const int stride_prim_idx, int& obj_id);
	// optional : nrl_list, rgb_list (if NULL, this is not used)
	__dojostatic bool GeneratePointCloudObject(const float* xyz_list, const float* nrl_list, const float* rgb_list, const int num_points, int& obj_id);
	__dojostatic bool GenerateIsoSurfaceObject(const int vol_id, const float iso_value, const int downsample_offset, const int mask_id, const int mask_value, const BoxTr* boxTr, int& obj_id);
	// center_aligned == false, then xyz is used for LT position of the text rect
	__dojostatic bool GenerateTextObject(const float* xyz_view_up, const std::string& text, const float font_height, const bool bold, const bool italic, int& obj_id, const bool center_aligned = false);
	__dojostatic bool GenerateMappingTable(const int table_size, const int num_alpha_ctrs, const float* ctr_alpha_idx_list, const int num_rgb_ctrs, const float* ctr_rgb_idx_list, int& tmap_id);
	__dojostatic bool GenerateMultiMappingTable(const int table_size, const int num_tables, const int target_table_idx, const int num_alpha_ctrs, const float* ctr_alpha_idx_list, const int num_rgb_ctrs, const float* ctr_rgb_idx_list, int& tmap_id);
	__dojostatic bool GenerateCopiedObject(const int obj_src_id, int& obj_id);
	__dojostatic bool DeleteResObject(const int obj_id); // the obj is deleted in memory
	__dojostatic bool GetPModelData(const int obj_id, float** pos_vtx, float** nrl_vtx, float** rgb_vtx, float** tex_vtx, int& num_vtx, unsigned int** idx_prims, int& num_prims, int& stride_prim_idx);
	__dojostatic bool GetVolumeInfo(const int obj_id, void*** vol_slices_2darray_pointer, int* size_xyz, float* pitch_xyz, int* stride_bytes, float* stored_minmax_values, float* original_minmax_values, float* mat_vs2os, bool* safe_bnd, std::vector<unsigned long long>* histogram);

	__dojostatic bool UpdateBVHTree(const int obj_id);
	//////////////////////////////

	//////////////////////////////////
	// 	   Scenes and Actors
	__dojostatic bool NewScene(const std::string& scene_title, int& scene_id); 
	__dojostatic bool NewActor(const ActorParameters& actor_params, const std::string& actor_name, int& actor_id);
	__dojostatic bool NewCamera(const CameraParameters& cam_params, const std::string& camera_name, int& cam_id);
	__dojostatic bool NewLight(const LightParameters& light_params, const std::string& light_name, int& light_id);
	__dojostatic bool GetActorParams(const int actor_id, ActorParameters& actor_params);
	__dojostatic bool GetCameraParams(const int cam_id, CameraParameters& camera_params);
	__dojostatic bool GetLightParams(const int light_id, LightParameters& light_params);
	__dojostatic bool GetSceneItemsByName(const std::string& name, std::vector<int>& scene_itemIDs);
	__dojostatic bool GetNameBySceneItemID(const int scene_item_id, std::string& name);
	__dojostatic bool SetActorParams(const int actor_id, const ActorParameters& actor_params);
	__dojostatic bool SetCameraParams(const int cam_id, const CameraParameters& camera_params);
	__dojostatic bool SetLightParams(const int light_id, const LightParameters& light_params);
	__dojostatic bool AppendSceneItemToSceneTree(const int scene_item_id, const int parent_scene_item_id); // scene_item_id:
	__dojostatic bool RemoveSceneItem(const int scene_item_id); // scene, actor, camera, or light
	__dojostatic bool UpdateSceneTransforms(const int scene_id); // if -1, all scenes are updated

	__dojostatic bool GetActorWorldBoundingBox(const int actor_id, BoxTr& boxTr); // from unitcube
	__dojostatic bool GetObjBoundingBox(const int obj_id, float* pos_min_os, float* pos_max_os);
	__dojostatic bool GetCamProjMatrix(const int cam_id, float* mat_ws2ss, float* mat_ss2ws = nullptr, bool is_col_major = true);

	__dojostatic bool RenderScene(const int scene_id, const int cam_id); // scene_id is used just for clarifying (note that cam_id has its belonging scene_id)
	__dojostatic bool GetRenderBufferPtrs(const int cam_id, unsigned char** ptr_rgba, float** ptr_zdepth, int* fbuf_w, int* fbuf_h, size_t* render_count = nullptr);
	__dojostatic bool GetSlicerLines(const int cam_id, std::vector<std::vector<float>>& objs_linelist_xy12s, std::vector<std::vector<float>>& objs_rgb_colors, std::vector<int>& objs_ids);

	__dojostatic bool RemoveResHWND(const HWND hWnd);
	__dojostatic bool PresentHWND(const HWND hWnd);
	// etc
	__dojostatic bool GetAllScenes(std::vector<int>& scene_ids);
	__dojostatic bool GetCamerasInScene(const int scene_id, std::vector<int>& cam_ids);
	__dojostatic bool GetActorsInScene(const int scene_id, std::vector<int>& actor_ids);
	__dojostatic bool GetActorsUsingResInScene(const int scene_id, const int res_obj_id, std::vector<int>& actor_ids);
	__dojostatic bool GetObjectsInScene(const int scene_id, std::vector<int>& obj_ids);
	__dojostatic bool GetAllResObjects(std::vector<int>& obj_ids);
	__dojostatic bool GetSceneItemType(const int scene_item_id, SceneItemType& sceneitem_type);
	__dojostatic bool GetSceneItemName(const int scene_item_id, std::string& sceneitem_name);
	__dojostatic bool GetSceneRootItem(const int scene_item_id, int& scene_id, int& root_item_id, SceneItemType& root_item_type);
	__dojostatic bool GetParentSceneItem(const int scene_item_id, int& scene_parent_id, SceneItemType& scene_parent_type);
	__dojostatic bool GetChildSceneItems(const int scene_item_id, std::vector<int>& scene_children_ids, std::vector<SceneItemType>& scene_children_types);

	// picking
	__dojostatic bool PickActor(int& picked_actor_id, float* pos_pick, const int x, const int y, const int cam_id, int* picked_submask_id = nullptr);
	__dojostatic bool PickActorList(std::vector<int>& picked_actor_ids, std::vector<float>& pos_picks, const int x, const int y, const int cam_id, std::vector<int>* picked_submask_ids = nullptr);
	__dojostatic bool CheckCollisionTwoMeshActors(const int actorId0, const int actorId1);

	// only for the contributor's (by DongJoon Kim) test info.
	__dojostatic void SetRenderTestParam(const std::string& _script, const std::any& value, const int scene_id, const int cam_id);
	__dojostatic bool GetRenderTestParam(const std::string& _script, std::any& value, const int scene_id, const int cam_id);
	__dojostatic void SetLogConfiguration(const bool displayLog, const int logLevel);
	__dojostatic void DisplaySceneSummary();

	// ioActors include camera and light sources
	// note: 
	// * value of ioResObjs is (VmVObject*)
	// * value of ioActors is (VmActor*)
	// * value of parameters is std::any
	__dojostatic bool ExecuteCustomModule(const std::string& module_dll_file, const std::string& dll_function, const ParamMap<std::string>& ioResObjs, const ParamMap<std::string>& ioActors, const ParamMap<std::string>& parameters);
}

namespace vzmproc
{
	// three curve-types : "uniform", "centripetal", , "chordal" 
	__dojostatic bool GenerateCurvePoints(const float* pos_ctr_points, const int num_ctr_points, const float interpolate_interval_dist, 
		std::vector<float>* curve_xyz_points, const std::string& curveType = "centripetal", std::vector<float>*curve_xyz_tangents = nullptr,
		const float* pos_ctr_ups = nullptr, std::vector<float>* curve_xyz_ups = nullptr,
		const float* pos_ctr_vecs = nullptr, std::vector<float>* curve_xyz_vecs = nullptr,
		const float* size_ctr_points = nullptr, std::vector<float>* curve_sizes = nullptr);

	__dojostatic bool SimplifyPModelByUGrid(const int obj_src_id, const float cell_width, int& obj_dst_id);
	__dojostatic bool ComputePCA(const int obj_id, float* egvals /*float3*/, float* egvecs /*three of float3*/);
	__dojostatic bool ComputePCAc(const float* xyz_list, const int num_points, float* egvals /*float3*/, float* egvecs /*three of float3*/);

	__dojostatic bool GenerateSamplePoints(const int obj_src_id, const float* pos_src, const float r, const float min_interval, int& obj_dst_id);
	// based on special-care ICP
	__dojostatic bool ComputeMatchingTransform(const int obj_from_id, const int obj_to_id, float* mat_tr /*float16*/);
	// at least 3 point-pairs are requested
	// matching based on least squares
	// assume each point pair has the same index of the point list (one-to-one);
	__dojostatic bool ComputeRigidTransform(const float* xyz_from_list, const float* xyz_to_list, const int num_pts, float* mat_tr /*float16*/);

	// if obj_dst_id == 0, then modify obj_src_id's gemometry resource!!
	__dojostatic bool ModifyRedundantGeometry(const int obj_src_id, const int obj_dst_id);

	__dojostatic bool ComputeCustomOBB(const int obj_id, const float* axis_1, const float* axis_2, const float* axis_3, vzm::BoxTr& boxTr);

	// mat_ext : glm::fmat4x3 format, conventional camera system's extrinsic parameters (y down and z as view direction)
	// fx, fy : focusing parameters
	// sc : skew coefficient
	// cx, cy : principal point position
	// w, h : screen size
	// zn, zf : near and far plane
	// api_mode : 0 => opengl, 1 => direct3d
	// mat_ws2cs (view matrix), mat_cs2ps (projection matrix), mat_ps2ss : glm::fmat4x4 format, output
	__dojostatic bool ComputeCameraRendererMatrice(const float* mat_ext,
		const float fx, const float fy, const float sc, const float cx, const float cy,
		const int w, const int h, const float zn, const float zf, const int api_mode,
		float* mat_ws2cs, float* mat_cs2ps, float* mat_ps2ss);
	__dojostatic bool ComputeCameraRendererParameters(const float* pos_xyz_ws, const float* pos_xy_ss, const int num_mks,
		float* cam_pos, float* cam_view, float* cam_up, float* fx, float* fy, float* sc, float* cx, float* cy);

	__dojostatic bool ComputeArCameraCalibrateInfo(const float* mat_rbs2ts, const float* calrb_xyz_ts, const float* calrb_xy_ss, const int num_mks,
		float* mat_camcs2rbs, vzm::CameraParameters* cam_ar_mode_params);
}

namespace helpers
{
	struct cam_pose
	{
		float pos[3], view[3], up[3]; // WS coordinates
	};

	__dojoclass arcball
	{
	public:
		arcball();
		~arcball();
		// stage_center .. fvec3
		bool intializer(const float* stage_center, const float stage_radius);
		// pos_xy .. ivec2
		bool start(const int* pos_xy, const float* screen_size, const cam_pose& cam_pose, const float np = 0.1f, const float fp = 100.f, const float sensitivity = 1.0f);
		// pos_xy .. ivec2
		// mat_r_onmove .. fmat4x4
		bool move(const int* pos_xy, cam_pose& cam_pose);	// target is camera
		bool move(const int* pos_xy, float* mat_r_onmove);	// target is object
		bool pan_move(const int* pos_xy, cam_pose& cam_pose);
	};

	__dojoclass slicer_drag
	{
	public:
		slicer_drag();
		~slicer_drag();
		bool intializer(const float* stage_center);
		// pos_xy .. ivec2
		bool start(const int* pos_xy, const float* screen_size, const cam_pose& cam_pose, const float ip_w, const float ip_h, const float zoom_sensitivity = 1.0f);
		bool pan_move(const int* pos_xy, cam_pose& cam_pose);
		// cam_pos is NULL ==> normal mode, if not.. center-oriented zoom
		bool zoom_move(const int* pos_xy, const bool convert_zoomdir, float& ip_w, float& ip_h, float* cam_pos = NULL);
	};
}