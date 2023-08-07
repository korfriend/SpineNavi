// SNaviWin.cpp : 애플리케이션에 대한 진입점을 정의합니다.
// DOJO : 자체적으로 개발한 namespace
// 엔진 : vzm, vzmutils, vzmproc
// 어플리케이션 helper : navihelpers
// 트래킹 APIs (Optirack APIs Wrapper) : optitrk 
// 위의 namespace 외엔 glm 과 같은 commonly-used 3rd party lib 이거나 C++ standard lib 또는 windows SDK lib 임

#include "framework.h"
#include "SNaviWin.h"

#include <regex>
#include <vector>
#include <windowsx.h>
#include <iostream>
#include <winsock2.h> // udp
#include <stdio.h>
#include <ctime>

#pragma comment(lib,"ws2_32.lib")

#include <objidl.h>
#include <gdiplus.h>
#include <gdipluspath.h>
#include <gdiplusgraphics.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

// math using GLM
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"
#include <glm/gtx/quaternion.hpp>

// SpineProjects
#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#include "../optitrk/optitrk.h"
#include "naviHelpers.hpp"
#include "rapidcsv/rapidcsv.h"
#include "CArmCalibration.h"

#define DESIRED_SCREEN_W 950
#define DESIRED_SCREEN_H 950
#define USE_WHND true

#define MAX_LOADSTRING 100
typedef unsigned long long u64;

std::string folder_data = "";
std::string folder_trackingInfo = "";

// 전역 변수:
HWND g_hWnd, g_hWndDialog1;
HINSTANCE hInst;                                // 현재 인스턴스입니다.
WCHAR szTitle[MAX_LOADSTRING];                  // 제목 표시줄 텍스트입니다.
WCHAR szWindowClass[MAX_LOADSTRING];            // 기본 창 클래스 이름입니다.

// 이 코드 모듈에 포함된 함수의 선언을 전달합니다:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
LRESULT CALLBACK    DiagProc1(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

#define SCANIMG_W 1296
#define SCANIMG_H 1296
#define FLIP_SCANIMAGE true
#define PHANTOM_MODE "LEGO" // "FILM"
#define EXTRINSIC_PHANTOM_IMG_INDEX 3
cv::Mat g_curScanImg(SCANIMG_W, SCANIMG_H, CV_8UC1);

std::atomic_bool download_completed{ false };

// DOJO : 특별한 기능은 아니며, 카메라 전환 시 부드러운 변화를 위해 slerp 로 카메라 정보를 저장한 구조체
int numAnimationCount = 50;
int arAnimationKeyFrame1 = -1;
std::vector<vzm::CameraParameters> cpInterCams1(numAnimationCount);
int arAnimationKeyFrame2 = -1;
std::vector<vzm::CameraParameters> cpInterCams2(numAnimationCount);

std::string g_sceneName = "Scene1"; // Scene1, Scene2
std::string g_camName = "Cam1"; // World Camera, CArm Camera
std::string g_camName2 = "Cam2"; // World Camera, CArm Camera

std::map<std::string, int> g_selectedMkNames;
std::map<int, int> g_mapAidGroupCArmCam;
#define OPTTRK_THREAD_FREE 0
#define OPTTRK_THREAD_C_ARM_REGISTER 1
#define OPTTRK_THREAD_TOOL_REGISTER 2
int g_optiMode = OPTTRK_THREAD_FREE;

std::vector<glm::fvec3> g_toolLocalPoints;
std::vector<glm::fvec3> g_testMKs;

glm::fvec3 ComputeNormalVector(std::vector<glm::fvec3>& points) {
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
		for (int i = 0; i < nunmPts; ++i) {
			A(i, 0) = points[i].x - centerPos.x;
			A(i, 1) = points[i].y - centerPos.y;
			A(i, 2) = points[i].z - centerPos.z;
		}

		//for (int i = 0; i < 3; ++i) {
		//	A(i, 0) = points[i + 1].x - points[0].x;
		//	A(i, 1) = points[i + 1].y - points[0].y;
		//	A(i, 2) = points[i + 1].z - points[0].z;
		//}

		// Compute the normal vector as the null space of A
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullV);
		normalVector = svd.matrixV().col(2);
		normalVector.normalize();
	}

	return glm::fvec3(normalVector.x(), normalVector.y(), normalVector.z());
};

namespace mystudents {
	using namespace std;

	// BlobDetector를 통해 원이 감지되었는지 확인하는 코드입니다.
	int CheckCircleDetect(const cv::Mat& inputImg, const std::vector<cv::KeyPoint>& keyPoints)
	{
		cv::Mat img = inputImg.clone();

		int r = 0;
		for (const auto& keypoint : keyPoints) {
			int x = cvRound(keypoint.pt.x);
			int y = cvRound(keypoint.pt.y);
			int s = cvRound(keypoint.size);
			r = cvRound(s / 2);
			cv::circle(img, cv::Point(x, y), r, cv::Scalar(0, 0, 256), 2);
			string text = " (" + to_string(x) + ", " + to_string(y) + ")";
			cv::putText(img, text, cv::Point(x - 25, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
		}
		cv::imwrite(folder_trackingInfo + "test_circle_detect.png", img); // cv imshow()로 보이기.

		return r; // 반지름 반환
	}

	// 2d 포지션의 index가 잘 정렬되었는지 확인하는 코드입니다.
	void CheckPositionSort(const cv::Mat& inputImg, const std::vector<cv::Point2f>& point2Ds, int r)
	{
		cv::Mat img = inputImg.clone();

		int idx = 0;
		for (const auto& center : point2Ds) {
			int x = cvRound(center.x);
			int y = cvRound(center.y);

			cv::circle(img, cv::Point(x, y), r, cv::Scalar(0, 0, 256), 2);
			string text = " (" + to_string(x) + ", " + to_string(y) + ")";
			cv::putText(img, text, cv::Point(x - 25, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);

			// index 출력
			cv::putText(img, to_string(idx), cv::Point(x - 20, y - 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 1);
			idx += 1;
		}
		cv::imwrite(folder_trackingInfo + "test_circle_sort.png", img); // cv imshow()로 보이기.
	}

	// implemented by 김민교 
	int Get2DPostionsFromLegoPhantom(const cv::Mat& inputImg, std::vector<cv::Point2f>& points2Ds)
	{
		cv::Mat imgGray;
		int chs = inputImg.channels();
		if (inputImg.channels() == 3)
			cv::cvtColor(inputImg, imgGray, cv::COLOR_BGR2GRAY);
		else if(inputImg.channels() == 1)
			imgGray = inputImg;
		else {
			cout << "not supported image!" << endl;
			assert(0);
		}
		cv::GaussianBlur(imgGray, imgGray, cv::Size(15, 15), 0);

		//Add mask
		//cv::Mat imgSrc = imgGray.clone(); // gray image(=imgSrc)로 circle detect 할 것.
		// Blob Detector Params
		cv::SimpleBlobDetector::Params params;
		params.filterByArea = true;
		params.filterByCircularity = true;
		params.filterByConvexity = false;
		params.filterByInertia = true;
		params.filterByColor = true;
		params.blobColor = 0;

		//params.minArea = 500; // The size of the blob filter to be applied.If the corresponding value is increased, small circles are not detected.
		//params.maxArea = 1000;
		//params.minCircularity = 0.01; // 1 >> it detects perfect circle.Minimum size of center angle

		params.minArea = 1000; // The size of the blob filter to be applied.If the corresponding value is increased, small circles are not detected.
		params.maxArea = 5500;
		params.minCircularity = 0.3; // 1 >> it detects perfect circle.Minimum size of center angle

		params.minInertiaRatio = 0.01; // 1 >> it detects perfect circle. short / long axis
		params.minRepeatability = 2;
		params.minDistBetweenBlobs = 0.1;

		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		vector<cv::KeyPoint> keypoints;
		detector->detect(imgGray, keypoints); // circle detect.

		// 원을 감지했는지 확인.
		int r = CheckCircleDetect(inputImg, keypoints);

		for (const auto& kp : keypoints) {
			points2Ds.push_back(cv::Point2f(kp.pt.x, kp.pt.y));
		}

		// 원의 좌표를 정렬.
		sort(points2Ds.begin(), points2Ds.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
			return a.x < b.x;
			});

		sort(points2Ds.begin(), points2Ds.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
			return a.y < b.y;
			});

		// 포지션 잘 정렬되었는지 확인.
		CheckPositionSort(inputImg, points2Ds, r);

		return (int)points2Ds.size();
	}

	// Function to calculate the normal vector of a plane given three points
	// Function to calculate the normal vector of a plane given three points
	void calc_normal_vector(const cv::Point3f& point1, const cv::Point3f& point2, const cv::Point3f& point3,
		cv::Point3f& vectorX, cv::Point3f& vectorY, cv::Point3f& normalVector) {

		cv::Point3f vector_x(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z); // x축
		cv::Point3f vector_y(point3.x - point1.x, point3.y - point1.y, point3.z - point1.z); // y축

		cv::Point3f normal_vector;
		normal_vector = vector_x.cross(vector_y); // x축과 y축 외적을 통해 두 벡터가 이루는 평면의 법선벡터를 구함.

		// Normalize
		vectorX = vector_x / norm(vector_x);
		vectorY = vector_y / norm(vector_y);
		normalVector = normal_vector / norm(normal_vector);
	}

	// Function to fit a plane to a set of 3D points and find its normal vector
	cv::Point3f fit_plane_to_points(const vector<cv::Point3f>& points) {

		cv::Point3f centroid(0.0, 0.0, 0.0);
		for (const auto& point : points) {
			centroid.x += point.x;
			centroid.y += point.y;
			centroid.z += point.z;
		}
		centroid.x /= points.size();
		centroid.y /= points.size();
		centroid.z /= points.size();

		vector<cv::Point3f> centered_points;
		for (const auto& point : points) {
			cv::Point3f centered_point(point.x - centroid.x, point.y - centroid.y, point.z - centroid.z);
			centered_points.push_back(centered_point);
		}

		cv::Point3f normal_vector;
		normal_vector.x = normal_vector.y = normal_vector.z = 0.0;

		for (size_t i = 0; i < centered_points.size(); ++i) {
			normal_vector.x += centered_points[i].y * centered_points[(i + 1) % centered_points.size()].z - centered_points[i].z * centered_points[(i + 1) % centered_points.size()].y;
			normal_vector.y += centered_points[i].z * centered_points[(i + 1) % centered_points.size()].x - centered_points[i].x * centered_points[(i + 1) % centered_points.size()].z;
			normal_vector.z += centered_points[i].x * centered_points[(i + 1) % centered_points.size()].y - centered_points[i].y * centered_points[(i + 1) % centered_points.size()].x;
		}

		double norm = std::sqrt(normal_vector.x * normal_vector.x + normal_vector.y * normal_vector.y + normal_vector.z * normal_vector.z);
		normal_vector.x /= norm;
		normal_vector.y /= norm;
		normal_vector.z /= norm;

		return normal_vector;
	}

	// implemented by 정태민
	// Function to get 3D positions of markers

	int Get3DPostionsFromLegoPhantom(const double marker_r,
		const cv::Point3f& marker1, const cv::Point3f& marker2, const cv::Point3f& marker3, const cv::Point3f& marker4,
		vector<cv::Point3f>& points3Ds) {

		const double unit = 0.0159375;

		double plane_r = 0.003; // 홈의 반지름 
		double big_h = marker_r - std::sqrt(marker_r * marker_r - plane_r * plane_r); //빠진 길이
		double small_marker_r = 0.004; // 8 / 2 / 1000;
		double small_h = small_marker_r - std::sqrt(small_marker_r * small_marker_r - plane_r * plane_r); //빠진 길이
		double h = big_h - small_h;

		cv::Point3f x, y, tempNormal;
		calc_normal_vector(marker1, marker4, marker2, x, y, tempNormal);
		cv::Point3f z = tempNormal;

		// fit_plane_to_points() 함수에서 네 개의 포인트와 SVD를 이용해 법선벡터를 보간함...
		// fit_plane_to_points({ marker1, marker2, marker3, marker4 });

		for (int i = 0; i < 8; ++i) {
			for (int j = 0; j < 8; ++j) {
				if (i == 0 && j == 0)
					continue;
				else if (i == 7 && j == 0)
					continue;
				else if (i == 0 && j == 7)
					continue;
				else if (i == 7 && j == 7)
					continue;

				cv::Point3f unit_position(float(i), float(j), 0.0f);
				cv::Point3f estimate_marker = cv::Point3f(marker1 + unit_position.x * unit * x + unit_position.y * unit * y + h * z);
				points3Ds.push_back(estimate_marker);
			}
		}

		//vector<vector<cv::Point3f>> points3Ds;
		//points3Ds.push_back(inter_points);
		return (int)points3Ds.size();
	}

	int Get3DPostionsFromLegoPhantom2(const float marker_r,
		const cv::Point3f& marker1, const cv::Point3f& marker2, const cv::Point3f& marker3, const cv::Point3f& marker4,
		vector<cv::Point3f>& points3Ds) {

		// note we are using meter metric
		using namespace glm;
		std::vector<fvec3> pts = { *(fvec3*)&marker1, *(fvec3*)&marker2, *(fvec3*)&marker3, *(fvec3*)&marker4 };
		fvec3 estimated_normal = ComputeNormalVector(pts); // return the unit length vector of the plane fitting to the input points
		// normal correction
		fvec3 v14 = pts[3] - pts[0];
		fvec3 v12 = pts[1] - pts[0];
		fvec3 nor_dir = cross(v14, v12);
		if (dot(estimated_normal, nor_dir) < 0) estimated_normal *= -1.f;

		// compute the aligned marker position fit to the sphere height
		const float h_r = 0.003f;// (float)(15.9375 * 0.001);
		const float s_r = 0.004f;
		float m_r_sq = marker_r * marker_r;
		float h_r_sq = h_r * h_r;
		float s_r_sq = s_r * s_r;
		float a = sqrt(m_r_sq - h_r_sq);
		float b = sqrt(s_r_sq - h_r_sq);
		float correctLength = a - b;
		fvec3 pts_corrected[4];
		for (int i = 0; i < 4; i++) {
			pts_corrected[i] = pts[i] - estimated_normal * correctLength;
		}

		const int rows = 8;
		const int cols = 8;

		for (int i = 0; i < rows; ++i) {
			float ratio_row = (float)i / (float)(rows - 1);
			fvec3 interPosRow0 = (1.f - ratio_row) * pts_corrected[0] + ratio_row * pts_corrected[3];
			fvec3 interPosRow1 = (1.f - ratio_row) * pts_corrected[1] + ratio_row * pts_corrected[2];

			for (int j = 0; j < cols; ++j) {
				if (i == 0 && j == 0)
					continue;
				else if (i == rows - 1 && j == 0)
					continue;
				else if (i == 0 && j == cols - 1)
					continue;
				else if (i == rows - 1 && j == cols - 1)
					continue;

				float ratio_col = (float)j / (float)(rows - 1);
				fvec3 interPos = (1.f - ratio_col) * interPosRow0 + ratio_col * interPosRow1;
				points3Ds.push_back(*(cv::Point3f*)&interPos);
			}
		}

		//vector<vector<cv::Point3f>> points3Ds;
		//points3Ds.push_back(inter_points);
		return (int)points3Ds.size();
	}
}




int main()
{
	return wWinMain(GetModuleHandle(NULL), NULL, GetCommandLine(), SW_SHOWNORMAL);
}

void GraphicsPostDrawing(const HDC hdc, const int cidCam)
{
	//std::vector<glm::fvec2> listPosSSs;
	//vzmutils::ComputeScreenPointsOnPanoSlicer(cidCam, listPosSSs);
	Graphics graphics(hdc);
	graphics.SetSmoothingMode(SmoothingModeAntiAlias);

	GraphicsPath path(FillModeAlternate);
	path.StartFigure();
	//path.AddLine(posSS_TL.x, posSS_TL.y, posSS_TR.x, posSS_TR.y);
	path.AddLine(0, 0, 200, 200);
	path.StartFigure();
	//path.AddLine(posSS_TR.x, posSS_TR.y, posSS_BR.x, posSS_BR.y);
	path.AddLine(200, 200, 200, 800);

	Pen pen(Color(255, 255, 255, 0), 2);
	graphics.DrawPath(&pen, &path);
}

void UpdateBMP(int cidCam, HWND hWnd) {
	unsigned char* ptr_rgba;
	float* ptr_zdepth;
	int w, h;
	// DOJO : 렌더링된 버퍼를 받아서 GUI window buffer 에 그리기
	if (vzm::GetRenderBufferPtrs(cidCam, &ptr_rgba, &ptr_zdepth, &w, &h))
	{
		// https://stackoverflow.com/questions/26005744/how-to-display-pixels-on-screen-directly-from-a-raw-array-of-rgb-values-faster-t
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hWnd, &ps);
		// Temp HDC to copy picture
		HDC src = CreateCompatibleDC(hdc); // hdc - Device context for window, I've got earlier with GetDC(hwnd__) or GetDC(NULL);
		// Creating temp bitmap
		HBITMAP map = CreateBitmap(w, h, 1, 8 * 4, (void*)ptr_rgba); // pointer to array
		SelectObject(src, map); // Inserting picture into our temp HDC
		// Copy image from temp HDC to window
		BitBlt(hdc, // Destination
			0,  // x and
			0,  // y - upper-left corner of place, where we'd like to copy
			w, // width of the region
			h, // height
			src, // source
			0,   // x and
			0,   // y of upper left corner  of part of the source, from where we'd like to copy
			SRCCOPY); // Defined DWORD to juct copy pixels. Watch more on msdn;
		//StretchBlt(hdc, 0, 0, w, h, hdc, w - 1, 0, -w, h, SRCCOPY);

		//GraphicsPostDrawing(hdc, cidCam);
		DeleteDC(src); // Deleting temp HDC
		DeleteObject(map);

		EndPaint(hWnd, &ps);
	}
};

void SceneInit() {
	// DOJO : Scene 은 Camera(s), Light source(s), Actor(s)로 구성, World Space 의 Coordinate System 을 정의함 
	// 
	// we set 'meter scale world to world space

	// DOJO : Actor 는 Scene 에 그려지는 Model 이며, 
	// Actor 를 구성하는 actor resource (called resource object) 는 geometry 및 material (e.g., texture) 등으로 정의
	// vzm::GenerateXXXObject 는 actor resource 를 생성 
	// vzm::LoadXXX 는 file 로부터 actor resource 를 생성
	// actor resource 의 geometry 는 기본적으로 Object Space (Model Space) 에서 정의된 것으로 봄
	
	// DOJO : Axis 를 위한 actor resource 를 생성
	// oidAxis (objId) 은 생성된 actor resource 의 ID 를 받아옴
	// objId = 0 인 경우, 항상 새로운 actor resource 를 생성하고 새로운 ID 를 받음
	// objId != 0 인 경우, 해당 ID 의 actor resource 가 있을 때, 해당 actor resource 를 update 함 (ID 는 그대로 사용)
	// objId != 0 인 경우, 해당 ID 의 actor resource 가 없을 때, 새로운 actor resource 를 생성하고 새로운 ID 를 받음
	int oidAxis = 0;
	vzm::GenerateAxisHelperObject(oidAxis, 0.5f);

	// DOJO : Actor Parameter (vzm::ActorParameters) 와 actor resource 로 actor 생성
	vzm::ActorParameters apAxis;
	// vzm::ActorParameters::SetResourceID 로 다양한 actor resource 를 actor 에 할당할 수 있음
	// 기본적으로 vzm::ActorParameters::RES_USAGE::GEOMETRY 가 있어야 함, 그 외 여러 종류의 texture 들이 올 수 있음 (ref. vzm::ActorParameters::RES_USAGE)
	apAxis.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidAxis);
	// line 으로 정의된 geometry 에 대해 렌더링에서 사용되는 parameter (c.f., triangle geometry 의 경우 해당 parameter 가 렌더링에서 사용되지 않음)
	apAxis.line_thickness = 3;
	// color 의 경우, 해당 actor 에 대해 global 로 적용됨 
	// geometry resource 가 vertex 단위로 color 가 정의되어 있다면, 여기선 alpha 값만 사용됨
	*(glm::fvec4*)apAxis.color = glm::fvec4(1, 1, 1, 1);
	// DOJO : 새로운 액터 생성
	// actor name parameter 와 actor parameter 필요
	// actor name parameter 로 actor 의 ID 를 가져올 수 있음
	int aidAxis = 0;
	vzm::NewActor(apAxis, "World Axis", aidAxis);

	// DOJO : navihelpers 는 이번 프로젝트에서 사용할 헬퍼 함수를 담고 있음
	int oidGrid, oidLines, oidTextX, oidTextZ;
	// scene 의 바닥에 그려지는 actor resources 를 생성
	navihelpers::World_GridAxis_Gen(oidGrid, oidLines, oidTextX, oidTextZ);

	std::vector<glm::fvec3> pinfo(3);
	pinfo[0] = glm::fvec3(0, 0.5, 0);
	pinfo[1] = glm::fvec3(0, 0, -1);
	pinfo[2] = glm::fvec3(0, 1, 0);
	// DOJO : "Frame" 이라는 text 를 그리는 actor resource 를 생성 (자세한 사용법은 문의 바람)
	// 여기서 int oidFrameText; 이렇게 했는데, 새롭게 생성하는 경우 oidFrameText = 0 을 권장
	// DEBUG 모드에서는 initialize 안 하면, 보통 0 값이 들어 가지만, 
	// RELEASE 모드에서는 initialize 안 하면, 임의의 값이 들어 가고, 이 때, (거의 발생하진 않지만) 다른 actor resource 의 ID 값이 들어가면 비정상 동작할 수 있음
	//int oidFrameText; // recommend "int oidFrameText = 0;"
	//vzm::GenerateTextObject((float*)&pinfo[0], "Frame", 0.1, true, false, oidFrameText);

	vzm::ActorParameters apGrid, apLines, apTextX, apTextZ, apTextFrame;
	apGrid.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidGrid);
	apLines.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidLines);
	apTextX.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidTextX);
	apTextZ.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidTextZ);
	//apTextFrame.SetResourceID(vzm::ActorParameters::RES_USAGE::GEOMETRY, oidFrameText);
	*(glm::fvec4*)apTextX.phong_coeffs = glm::fvec4(0, 1, 0, 0);
	*(glm::fvec4*)apTextZ.phong_coeffs = glm::fvec4(0, 1, 0, 0);
	//*(glm::fvec4*)apTextFrame.phong_coeffs = glm::fvec4(0, 1, 0, 0);

	int aidGrid = 0, aidLines = 0, aidTextX = 0, aidTextZ = 0;// , aidTextFrame;
	vzm::NewActor(apGrid, "World Grid", aidGrid);
	vzm::NewActor(apLines, "World Lines", aidLines);
	vzm::NewActor(apTextX, "World Text X", aidTextX);
	vzm::NewActor(apTextZ, "World Text Y", aidTextZ);
	//vzm::NewActor(apTextFrame, "Frame Text", aidTextFrame);
	
	RECT rcWorldView;
	GetClientRect(g_hWnd, &rcWorldView);

	// DOJO : (world view 에서의) Camera 를 정의하기 위한 파라미터
	vzm::CameraParameters cpCam1;
	*(glm::fvec3*)cpCam1.pos = glm::fvec3(-1.5, 1.5, -1.5);
	*(glm::fvec3*)cpCam1.up = glm::fvec3(0, 1, 0);
	*(glm::fvec3*)cpCam1.view = glm::fvec3(1, -1, 1);
	cpCam1.displayActorLabel = cpCam1.displayCamTextItem = true;

	// YAML 로 이전에 저장된 (world view 에서의) 카메라 위치 정보 읽어 들임
	cv::FileStorage fs(folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
	if (fs.isOpened()) {
		cv::Mat ocvVec3;
		fs["POS"] >> ocvVec3;
		memcpy(cpCam1.pos, ocvVec3.ptr(), sizeof(float) * 3);
		fs["VIEW"] >> ocvVec3;
		memcpy(cpCam1.view, ocvVec3.ptr(), sizeof(float) * 3);
		fs["UP"] >> ocvVec3;
		memcpy(cpCam1.up, ocvVec3.ptr(), sizeof(float) * 3);
		fs.release();
	}

	// 렌더링될 buffer 사이즈
	cpCam1.w = rcWorldView.right - rcWorldView.left;
	cpCam1.h = rcWorldView.bottom - rcWorldView.top;

	// 렌더링 파이프라인의 View Frustum 에서의 near plane, far plane distances
	cpCam1.np = 0.1f;
	cpCam1.fp = 100.f;

	float vFov = 3.141592654f / 4.f;
	// NOTE : union 으로 저장되어 있음
	// 카메라 view projection 의 자연스러운 이동 (to C-arm View) 을 위해, 
	// 카메라 설정 convention 을 camera (intrinsic) parameters 로 사용
	if (0) {
		cpCam1.fov_y = vFov;
		cpCam1.aspect_ratio = (float)cpCam1.w / (float)cpCam1.h;
		cpCam1.projection_mode = vzm::CameraParameters::CAMERA_FOV;
	}
	else {
		float aspect_ratio = (float)cpCam1.w / (float)cpCam1.h;
		float hFov = 2.f * atan(vFov / 2.f) * aspect_ratio;

		//  fy = h/(2 * tan(f_y / 2)) 
		cpCam1.fx = cpCam1.w / (2.f * tan(hFov / 2.f));
		cpCam1.fy = cpCam1.h / (2.f * tan(vFov / 2.f));
		cpCam1.sc = 0;
		cpCam1.cx = cpCam1.w / 2.f;
		cpCam1.cy = cpCam1.h / 2.f;
		cpCam1.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
	}

	//cpCam1.SetOrthogonalProjection(true);
	cpCam1.hWnd = USE_WHND ? g_hWnd : NULL;
	// DOJO : scene 에 등록될 카메라 생성, 여기선 두 개 생성 (AP, Lateral 용)
	int cidCam1 = 0, cidCam2 = 0;
	vzm::NewCamera(cpCam1, "Cam1", cidCam1);
	cpCam1.hWnd = USE_WHND ? g_hWndDialog1 : NULL;
	vzm::NewCamera(cpCam1, "Cam2", cidCam2);

	// DOJO : Light 를 정의하기 위한 파라미터
	vzm::LightParameters lpLight1;
	lpLight1.is_on_camera = true;
	lpLight1.is_pointlight = false;
	*(glm::fvec3*)lpLight1.pos = *(glm::fvec3*)cpCam1.pos;
	*(glm::fvec3*)lpLight1.dir = *(glm::fvec3*)cpCam1.view;
	*(glm::fvec3*)lpLight1.up = *(glm::fvec3*)cpCam1.up;
	int lidLight1 = 0;
	// DOJO : scene 에 등록될 Light 생성
	vzm::NewLight(lpLight1, "World Light", lidLight1);

	// DOJO : scene 생성
	int sidScene = 0;
	vzm::NewScene("Scene1", sidScene);

	// DOJO : scene 에 camera 1 (cidCam1) 을 연결 
	vzm::AppendSceneItemToSceneTree(cidCam1, sidScene);
	// DOJO : scene 에 camera 2 (cidCam2) 을 연결 
	vzm::AppendSceneItemToSceneTree(cidCam2, sidScene);
	// DOJO : scene 에 Light (lidLight1) 을 연결 
	vzm::AppendSceneItemToSceneTree(lidLight1, sidScene);
	// DOJO : scene 에 axis actor (aidAxis) 연결 
	vzm::AppendSceneItemToSceneTree(aidAxis, sidScene);
	// DOJO : scene 에 ground grid actor (aidGrid) 을 연결 
	vzm::AppendSceneItemToSceneTree(aidGrid, sidScene);
	// DOJO : scene 에 ground grid 에서의 진한 라인 actor (aidLines) 을 연결 
	vzm::AppendSceneItemToSceneTree(aidLines, sidScene);
	// DOJO : scene 에 ground grid 위에 있는 "X" 글자 actor (aidTextX) 을 연결 
	vzm::AppendSceneItemToSceneTree(aidTextX, sidScene);
	// DOJO : scene 에 ground grid 위에 있는 "Z" 글자 actor (aidTextZ) 을 연결 
	vzm::AppendSceneItemToSceneTree(aidTextZ, sidScene);
	// DOJO : scene 에 "Frame" 글자 actor (aidTextFrame) 을 연결 
	//vzm::AppendSceneItemToSceneTree(aidTextFrame, sidScene);
	// NOTE : 여기에서는 모든 actor 및 camera, light 등 scene item 들이 모두 scene (i.e., root) 에 바로 연결되었지만,
	// 특정 actor 의 자식으로 붙을 수도 있다 (e.g., scene<-aidAxis<-aidTextZ<-cidCam1 ,...)
}

// DOJO : Tracking 되는 정보를 Scene 에 반영시키는 함수
// Timer 에 등록된 CALLBACK TimerProc 에서 호출됨 (note: Timer 가 본 어플리케이션에서 rendering thread 로 사용됨)
// track_info& trackInfo : 트래킹 Thread 에서 concurrent queue (또는 async queue 라도고 함) 로 저장된 가장 최신 정보
// numRBs : rigid bodies
// numMKs : 개별 markers
void UpdateTrackInfo2Scene(navihelpers::track_info& trackInfo) 
{
	int numMKs = trackInfo.NumMarkers();
	int numRBs = trackInfo.NumRigidBodies();

	// DOJO : 최초 한번만 리소스 오브젝트 생성 (static 으로 지정된 리소스 오브젝트 ID = 0 일 때, 생성) 
	// 이를 actor 로 생성하고 scene tree 에 배치
	{
		using namespace std;
		using namespace navihelpers;
		using namespace glm;

		int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);

		{
			// tracking camera (총 세개) 를 그리는 actors
			static int aidGroupCams = 0;
			if (aidGroupCams == 0) {
				vzm::ActorParameters apGroupCams;
				vzm::NewActor(apGroupCams, "Tracking CAM Group", aidGroupCams);
				vzm::AppendSceneItemToSceneTree(aidGroupCams, sidScene);

				static int oidCamModels[3] = { 0, 0, 0 };
				for (int i = 0; i < 3; i++) {
					fmat4x4 matCam2WS;
					optitrk::GetCameraLocation(i, __FP matCam2WS);
					int oidCamTris = 0, oidCamLines = 0, oidCamLabel = 0;
					Cam_Gen(matCam2WS, "CAM" + to_string(i), oidCamTris, oidCamLines, oidCamLabel);

					vzm::ActorParameters apCamTris, apCamLines, apCamLabel;
					apCamTris.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamTris);
					apCamTris.color[3] = 0.5f;
					apCamLines.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLines);
					*(fvec4*)apCamLines.phong_coeffs = fvec4(0, 1, 0, 0);
					apCamLines.line_thickness = 2;
					apCamLabel.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLabel);
					*(fvec4*)apCamLabel.phong_coeffs = fvec4(0, 1, 0, 0);
					int aidCamTris = 0, aidCamLines = 0, aidCamLabel = 0;
					vzm::NewActor(apCamTris, "Cam" + to_string(i) + " Tris", aidCamTris);
					vzm::NewActor(apCamLines, "Cam" + to_string(i) + " Lines", aidCamLines);
					vzm::NewActor(apCamLabel, "Cam" + to_string(i) + " Label", aidCamLabel);
					vzm::AppendSceneItemToSceneTree(aidCamTris, aidGroupCams);
					vzm::AppendSceneItemToSceneTree(aidCamLines, aidGroupCams);
					vzm::AppendSceneItemToSceneTree(aidCamLabel, aidGroupCams);
				}
			}
			//
		}

		static int oidAxis = 0;
		static int oidMarker = 0;
		static int oidProbe = 0;
		//static int oidLine = 0;
		// axis 를 그리는 resource object 생성
		if (vzm::GetResObjType(oidAxis) == vzm::ResObjType::UNDEFINED) {
			vzm::GenerateAxisHelperObject(oidAxis, 0.15f);
		}
		// spherical marker 를 그리는 resource object 생성
		if (vzm::GetResObjType(oidMarker) == vzm::ResObjType::UNDEFINED) {
			glm::fvec4 pos(0, 0, 0, 1.f);
			vzm::GenerateSpheresObject(__FP pos, NULL, 1, oidMarker);
		}
		// probe 를 그리는 resource object 생성
		if (vzm::GetResObjType(oidProbe) == vzm::ResObjType::UNDEFINED) {
			//vzm::LoadModelFile(folder_data + "probe.obj", oidProbe);

			glm::fvec3 posS(0, 0, 0.25f);
			glm::fvec3 posE(0, 0, 0);
			vzm::GenerateArrowObject(__FP posS, __FP posE, 0.0025f, 0.0025f, oidProbe);
		}
		//if (oidLine == 0) {
		//	glm::fvec3 linePos[2] = { glm::fvec3(0, 0, 0), glm::fvec3(0, 0, 1) };
		//	vzm::GenerateLinesObject((float*)linePos, NULL, 1, oidLine);
		//}

		// tracking 된 rigid bodies 중, probe 와 그외 일반 rigid body frame 을 나타낼 scene actor 에 생성된 resource (oidProbe, oidAxis) 를 할당
		// 최초 한 번만 할당하며 scene 에 tracking unit 의 이름이 있는지로 확인하고 없을 때, 할당
		// rigid body 에 등록된 마커도 같이 등록
		for (int i = 0; i < numRBs; i++) {
			std::string rbName;
			float rb_error = 0; 
			map<string, map<track_info::MKINFO, std::any>> rbmkSet;
			// 여기에서 index 는 optitrack lib 에서 사용하는 index 와 다르다! track_info 구조체에서 사용하는 index 임!
			if (trackInfo.GetRigidBodyByIdx(i, &rbName, NULL, &rb_error, &rbmkSet)) {
				if (rb_error > 0.01) cout << "\n" <<"problematic error!! " << rbName << " : " << rb_error << endl;

				int aidRb = vzmutils::GetSceneItemIdByName(rbName);
				if (aidRb == 0) {
					vzm::ActorParameters apRb;
					if (rbName == "probe") {
						apRb.SetResourceID(vzm::ActorParameters::GEOMETRY, oidProbe);
						*(glm::fvec4*)apRb.color = glm::fvec4(1, 0.3, 0.2, 1);
					}
					else {
						apRb.SetResourceID(vzm::ActorParameters::GEOMETRY, oidAxis);
						apRb.line_thickness = 3;
					}
					apRb.is_visible = false;
					vzm::NewActor(apRb, rbName, aidRb);
					vzm::AppendSceneItemToSceneTree(aidRb, sidScene);

					if (rbName == "probe") {
					}
					else if (rbName == "tool") {
						vzm::ActorParameters apEmpty;
						int aidToolBody = 0, aidToolTip = 0;
						vzm::NewActor(apEmpty, rbName + ":Body", aidToolBody);
						apEmpty.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
						vzm::NewActor(apEmpty, rbName + ":Tip", aidToolTip);
						vzm::AppendSceneItemToSceneTree(aidToolBody, aidRb);
						vzm::AppendSceneItemToSceneTree(aidToolTip, aidRb);
					}
					else {
						std::vector<glm::fvec3> pinfo(3);
						pinfo[0] = glm::fvec3(0, 0, 0);
						pinfo[1] = glm::fvec3(-1, 0, 0);
						pinfo[2] = glm::fvec3(0, -1, 0);
						int oidLabelText = 0;
						vzm::GenerateTextObject((float*)&pinfo[0], rbName, 0.07, true, false, oidLabelText);
						vzm::ActorParameters apLabelText;
						apLabelText.SetResourceID(vzm::ActorParameters::GEOMETRY, oidLabelText);
						*(glm::fvec4*)apLabelText.phong_coeffs = glm::fvec4(0, 1, 0, 0);
						int aidLabelText = 0;
						vzm::NewActor(apLabelText, rbName + ":Label", aidLabelText);
						vzm::AppendSceneItemToSceneTree(aidLabelText, aidRb);
					}

					// rigid body 의 markers 들을 최초 할당
					for (auto& rbmk : rbmkSet) {
						map<track_info::MKINFO, std::any>& rbmk_v = rbmk.second;
						string rbmk_name = std::any_cast<std::string>(rbmk_v[track_info::MKINFO::MK_NAME]);
						int aidMarker = vzmutils::GetSceneItemIdByName(rbmk_name);
						if (aidMarker == 0) {
							vzm::ActorParameters apMarker;
							apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
							apMarker.is_visible = false;
							apMarker.is_pickable = false;
							*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 0.f, 0.f, 0.3f); // rgba
							vzm::NewActor(apMarker, rbmk_name, aidMarker);
							vzm::AppendSceneItemToSceneTree(aidMarker, aidRb);
						}
					}
				}
			}
		}

		// tracking 된 individual markers 를 나타낼 spherical shape 의 scene actor 에 생성된 resource (oidMarker) 를 할당
		// 최초 한 번만 할당하며 scene 에 tracking unit 의 이름이 있는지로 확인하고 없을 때, 할당
		for (int i = 0; i < numMKs; i++) {
			std::map<navihelpers::track_info::MKINFO, std::any> mk;
			if (trackInfo.GetMarkerByIdx(i, mk)) {
				std::string mkName = std::any_cast<std::string>(mk[navihelpers::track_info::MKINFO::MK_NAME]);
				int aidMarker = vzmutils::GetSceneItemIdByName(mkName);
				if (aidMarker == 0) {
					vzm::ActorParameters apMarker;
					apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
					apMarker.is_visible = false;
					*(glm::fvec4*)apMarker.color = glm::fvec4(1.f, 1.f, 1.f, 1.f); // rgba
					vzm::NewActor(apMarker, mkName, aidMarker);
					vzm::AppendSceneItemToSceneTree(aidMarker, sidScene);
				}
			}
		}

		int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
		if (aidTestGroup == 0) {
			vzm::ActorParameters apGroup;
			vzm::NewActor(apGroup, "testMK Group", aidTestGroup);
		}
		for (int i = 0; i < (int)g_testMKs.size(); i++) {
			std::string testMkName = "testMk-" + std::to_string(i);
			int aidMarkerTest = vzmutils::GetSceneItemIdByName(testMkName);
			if (aidMarkerTest == 0) {
				vzm::ActorParameters apMarker;
				apMarker.SetResourceID(vzm::ActorParameters::GEOMETRY, oidMarker);
				apMarker.is_visible = false;
				*(glm::fvec4*)apMarker.color = glm::fvec4(0, 0, 1.f, 1.f); // rgba
				vzm::NewActor(apMarker, testMkName, aidMarkerTest);
				vzm::AppendSceneItemToSceneTree(aidMarkerTest, aidTestGroup);
			}
		}
	}

	// DOJO : scene tree 에 배치된 (rigid body) actor 들의 위치를 tracking 정보를 바탕으로 변환 이동 
	for (int i = 0; i < numRBs; i++) {
		std::string rbName;
		glm::fmat4x4 matLS2WS;
		std::map<std::string, std::map<navihelpers::track_info::MKINFO, std::any>> rbmkSet;
		if (trackInfo.GetRigidBodyByIdx(i, &rbName, &matLS2WS, NULL, &rbmkSet)) {
			
			int aidRb = vzmutils::GetSceneItemIdByName(rbName);
			vzm::ActorParameters apRb;
			vzm::GetActorParams(aidRb, apRb);
			apRb.is_visible = true;

			apRb.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidRb, apRb);

			glm::fmat4x4 matWS2LS = glm::inverse(matLS2WS);

			for (auto& rbmk : rbmkSet) {
				std::map<navihelpers::track_info::MKINFO, std::any>& rbmk_v = rbmk.second;

				std::string rbmk_name = std::any_cast<std::string>(rbmk_v[navihelpers::track_info::MKINFO::MK_NAME]);
				glm::fvec3 pos = std::any_cast<glm::fvec3>(rbmk_v[navihelpers::track_info::MKINFO::POSITION]);
				glm::fvec3 posRb = vzmutils::transformPos(pos, matWS2LS);

				glm::fmat4x4 matMkLS2WS = glm::translate(posRb);
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.007f)); // set 1 cm to the marker diameter
				matMkLS2WS = matMkLS2WS * matScale;
				int aidMarker = vzmutils::GetSceneItemIdByName(rbmk_name);
				vzm::ActorParameters apMarker;
				vzm::GetActorParams(aidMarker, apMarker);
				apMarker.is_visible = true;
				apMarker.is_pickable = false;
				apMarker.SetLocalTransform(__FP matMkLS2WS);
				vzm::SetActorParams(aidMarker, apMarker);
			}

			if (rbName == "tool") {
				using namespace glm;

				const float tipMarkerRadius = (float)(0.019 * 0.5);
				int numToolPoints = (int)g_toolLocalPoints.size();
				fvec3 posCenter = fvec3(0, 0, 0);
				fvec3 posTipMk = g_toolLocalPoints[numToolPoints - 1];
				for (int i = 0; i < numToolPoints - 1; i++) {
					posCenter += g_toolLocalPoints[i];
				}
				posCenter /= (float)(numToolPoints - 1);

				// compute normal
				fvec3 nrl = ComputeNormalVector(g_toolLocalPoints);

				// correct the normal direction using cam view
				int cidCam1 = vzmutils::GetSceneItemIdByName(g_camName); // Cam1
				vzm::CameraParameters cam1Param;
				vzm::GetCameraParams(cidCam1, cam1Param);
				fvec3 camViewLS = vzmutils::transformVec(*(fvec3*)cam1Param.view, matWS2LS);
				if (dot(nrl, camViewLS) > 0) nrl *= -1.f;

				// compute line point of the tool
				fvec3 posLine = posCenter - nrl * 0.07f;
				fvec3 posLineLS = vzmutils::transformPos(posLine, matWS2LS);

				// compute the direction of a tip (using inverse of the cam up)
				fvec3 posTipMkLS = vzmutils::transformPos(posTipMk, matWS2LS);
				fvec3 camUpLS = vzmutils::transformVec(*(fvec3*)cam1Param.up, matWS2LS);
				fvec3 dirToolLS = normalize(posTipMkLS - posLineLS);
				
				// compute the line geometry in the tool's rigid body
				fvec3 posTipLS = posTipMkLS - dirToolLS * tipMarkerRadius;
				glm::fvec3 linePos[2] = { posLineLS, posTipLS };
				static int oidToolLine = 0;
				if (vzm::GetResObjType(oidToolLine) == vzm::ResObjType::UNDEFINED)
					vzm::GenerateLinesObject((float*)linePos, NULL, 1, oidToolLine);

				int aidToolBody = vzmutils::GetSceneItemIdByName(rbName + ":Body");
				int aidToolTip = vzmutils::GetSceneItemIdByName(rbName + ":Tip");
				assert(aidToolBody != 0 && aidToolTip != 0);

				vzm::ActorParameters apToolBody, apToolTip;
				vzm::GetActorParams(aidToolBody, apToolBody);
				apToolBody.SetResourceID(vzm::ActorParameters::GEOMETRY, oidToolLine);
				apToolBody.line_thickness = 3;
				*(fvec4*)apToolBody.color = fvec4(0, 0, 1, 1);
				vzm::SetActorParams(aidToolBody, apToolBody);

				// local transform to the tip sphere
				vzm::GetActorParams(aidToolTip, apToolTip);
				*(fvec4*)apToolTip.color = fvec4(1, 0, 0, 1);
				glm::fmat4x4 matLS2WS = glm::translate(posTipLS);
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.002f)); // set 1 cm to the marker diameter
				matLS2WS = matLS2WS * matScale;
				apToolTip.SetLocalTransform(__FP matLS2WS);
				vzm::SetActorParams(aidToolTip, apToolTip);
			}
		}
	}

	// DOJO : scene tree 에 배치된 (marker) actor 들의 위치를 tracking 정보를 바탕으로 변환 이동
	for (int i = 0; i < numMKs; i++) {
		std::map<navihelpers::track_info::MKINFO, std::any> mk;
		if (trackInfo.GetMarkerByIdx(i, mk)) {
			std::string mkName = std::any_cast<std::string>(mk[navihelpers::track_info::MKINFO::MK_NAME]);
			glm::fvec3 pos = std::any_cast<glm::fvec3>(mk[navihelpers::track_info::MKINFO::POSITION]);

			glm::fmat4x4 matLS2WS = glm::translate(pos);
			glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.005f)); // set 1 cm to the marker diameter
			matLS2WS = matLS2WS * matScale;
			int aidMarker = vzmutils::GetSceneItemIdByName(mkName);
			vzm::ActorParameters apMarker;
			vzm::GetActorParams(aidMarker, apMarker);
			apMarker.is_visible = true;
			apMarker.is_pickable = true;

			*(glm::fvec4*)apMarker.color = glm::fvec4(1, 1, 1, 1);
			if(g_selectedMkNames.find(mkName) != g_selectedMkNames.end()) {
				*(glm::fvec4*)apMarker.color = glm::fvec4(0, 1, 0, 1);
			}

			apMarker.SetLocalTransform(__FP matLS2WS);
			vzm::SetActorParams(aidMarker, apMarker);
		}
	}
	
	// test display
	{
		// for testing
		int test_count = 1;
		for (int i = 0; i < (int)g_testMKs.size(); i++) {
			std::string testMkName = "testMk-" + std::to_string(i);
			int aidMarkerTest = vzmutils::GetSceneItemIdByName(testMkName);
			if (aidMarkerTest != 0) {
				glm::fmat4x4 matLS2WS = glm::translate(g_testMKs[i]);
				glm::fmat4x4 matScale = glm::scale(glm::fvec3(0.002f)); // set 1 cm to the marker diameter
				matLS2WS = matLS2WS * matScale;

				vzm::ActorParameters apMarker;
				vzm::GetActorParams(aidMarkerTest, apMarker);
				apMarker.is_visible = true;

				if (test_count == 7) test_count++;
				else if (test_count == 57) test_count++;

				*(glm::fvec4*)apMarker.color = glm::fvec4((test_count % 8) / 8.f, ((test_count / 8.f) + 1.f) / 8.f, 0, 1.f); // rgba

				test_count++;

				apMarker.SetLocalTransform(__FP matLS2WS);
				vzm::SetActorParams(aidMarkerTest, apMarker);
			}
		}
	}
}

// DOJO : 현재 Scene (g_sceneName) 에서 등록된 camera (g_camName, g_camName2) 들에 대해 렌더링하는 함수
// Timer 에 등록된 CALLBACK TimerProc 에서 호출됨 (note: Timer 가 본 어플리케이션에서 rendering thread 로 사용됨)
void Render() {
	int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(g_camName);

	if (sidScene != 0 && cidCam1 != 0) {
		// DOJO : camera 의 위치 update
		// cpInterCams1 는 매 스캔 시 업데이트되며, 
		// slerp 로 인터폴레이션된 camera 정보를 받아 옴
		if (arAnimationKeyFrame1 >= 0) {
			vzm::CameraParameters cpCam = cpInterCams1[arAnimationKeyFrame1++];
			cpCam.projection_mode = vzm::CameraParameters::CAMERA_INTRINSICS;
			vzm::SetCameraParams(cidCam1, cpCam);
			if (arAnimationKeyFrame1 == numAnimationCount)
				arAnimationKeyFrame1 = -1;

			//UINT flags = SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE;
			//SetWindowPos(g_hWnd, NULL, 100, 100, cpCam.w, cpCam.h, flags);
		}

		// DOJO : sidScene 의 cidCam1 을 렌더링
		vzm::RenderScene(sidScene, cidCam1);

		// DOJO : windows SDK 에서 화면 업데이트 (QT 방식은 UpdateBMP 참조)
		// USE_WHND 가 false 로 되어 있어야 함
		if (USE_WHND)
			vzm::PresentHWND(g_hWnd);
		else {
			UpdateBMP(cidCam1, g_hWnd);
			InvalidateRect(g_hWnd, NULL, FALSE);
		}
		//Sleep(1);
	}

	int cidCam2 = vzmutils::GetSceneItemIdByName(g_camName2);
	if (sidScene != 0 && cidCam2 != 0) {
		// DOJO : camera 의 위치 update
		// cpInterCams2 는 매 스캔 시 업데이트되며, 
		// slerp 로 인터폴레이션된 camera 정보를 받아 옴
		if (arAnimationKeyFrame2 >= 0) {
			vzm::CameraParameters cpCam = cpInterCams2[arAnimationKeyFrame2++];
			vzm::SetCameraParams(cidCam2, cpCam);
			if (arAnimationKeyFrame2 == numAnimationCount)
				arAnimationKeyFrame2 = -1;

			//UINT flags = SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE;
			//SetWindowPos(g_hWnd, NULL, 100, 100, cpCam.w, cpCam.h, flags);
		}

		// DOJO : sidScene 의 cidCam1 을 렌더링
		vzm::RenderScene(sidScene, cidCam2);

		// DOJO : windows SDK 에서 화면 업데이트 (QT 방식은 UpdateBMP 참조)
		// USE_WHND 가 false 로 되어 있어야 함
		if (USE_WHND)
			vzm::PresentHWND(g_hWndDialog1);
		else {
			UpdateBMP(cidCam2, g_hWndDialog1);
			InvalidateRect(g_hWndDialog1, NULL, FALSE);
		}
	}
}

// DOJO: asynchronous (read write modify) queue of tracking info.
// tracking thread 에서 queue 에 등록 (최신 정보 insert) 및 삭제 (오래된 element push out)
// timer thread (rendering thread) 에서 queue 의 정보를 pop out (삭제) 하여 사용 
navihelpers::concurrent_queue<navihelpers::track_info>* g_track_que = NULL;

// DOJO: windows SDK 에서 호출되는 SetTimer 의 callback 함수
// 여기에서 UpdateTrackInfo2Scene() 과 Render() 이 호출된다.
// rendering thread 의 process 라고 생각해도 됨
void CALLBACK TimerProc(HWND, UINT, UINT_PTR pcsvData, DWORD)
{
	static int frameCount = 0;

	int frame = frameCount++;

	// DOJO: Scene 에 "Frame : 숫자" Text 뜨게 하는 (Actor에 사용될) 리소스(오브젝트) 생성 
	{
		//int aidFrameText = vzmutils::GetSceneItemIdByName("Frame Text");
		//vzm::ActorParameters apFrameText;
		//vzm::GetActorParams(aidFrameText, apFrameText);
		//int oidFrameText = apFrameText.GetResourceID(vzm::ActorParameters::GEOMETRY);
		//std::vector<glm::fvec3> pinfo(3);
		//pinfo[0] = glm::fvec3(0, 0.3, 0);
		//pinfo[1] = glm::fvec3(0, 0, -1);
		//pinfo[2] = glm::fvec3(0, 1, 0);
		//vzm::GenerateTextObject((float*)&pinfo[0], "Frame : " + std::to_string(frame), 0.1, true, false, oidFrameText);

		int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
		int cidCam1 = vzmutils::GetSceneItemIdByName(g_camName);
		vzm::CameraParameters cpCam1;
		vzm::GetCameraParams(cidCam1, cpCam1);
		vzm::TextItem textItem;
		textItem.textStr = "Frame : " + std::to_string(frame);
		textItem.fontSize = 30.f;
		textItem.iColor = 0xFFFFFF;
		textItem.posScreenX = 0;
		textItem.posScreenY = cpCam1.h - 40;
		cpCam1.text_items.SetParam("FRAME", textItem);
		vzm::SetCameraParams(cidCam1, cpCam1);
	}

	navihelpers::concurrent_queue<navihelpers::track_info>* track_que = 
		(navihelpers::concurrent_queue<navihelpers::track_info>*)pcsvData;
	
	// DOJO: concurrent queue 에서 가장 최신에 저장된 tracking frame 정보 pop out
	navihelpers::track_info trackInfo;
	track_que->wait_and_pop(trackInfo);

	// generate scene actors with updated trackInfo
	// DOJO : Tracking 되는 기본 세트 (individual markers + rigid body frames) 를 Scene 에 반영시키는 함수

	UpdateTrackInfo2Scene(trackInfo);

	Render();
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

	auto getdatapath = []()
	{
		using namespace std;
		char ownPth[2048];
		GetModuleFileNameA(NULL, ownPth, (sizeof(ownPth)));
		string exe_path = ownPth;
		string exe_path_;
		size_t pos = 0;
		std::string token;
		string delimiter = "\\"; // windows
		while ((pos = exe_path.find(delimiter)) != std::string::npos) {
			token = exe_path.substr(0, pos);
			if (token.find(".exe") != std::string::npos) break;
			exe_path += token + "/";
			exe_path_ += token + "/";
			exe_path.erase(0, pos + delimiter.length());
		}
		return exe_path_ + "../../data/";
	};

	folder_data = getdatapath();
	folder_trackingInfo = getdatapath() + "Tracking 2023-08-05/";

	{
		cv::Mat __curScanImg = cv::imread(folder_trackingInfo + "test0.png");
		std::vector<cv::Point2f> points2d;
		mystudents::Get2DPostionsFromLegoPhantom(__curScanImg, points2d);
		int gg = 0;
	}



	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	// engine initialization to use the core APIs of framework
	// must be paired with DeinitEngineLib()
	vzm::InitEngineLib("SpineNavi");
	vzm::SetLogConfiguration(true, 4);

    bool optitrkMode = optitrk::InitOptiTrackLib();

	optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-08-06.motive", folder_data + "System Calibration.cal");
	//optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-07-04.motive", folder_data + "System Calibration.cal");

    // 전역 문자열을 초기화합니다.
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_SNAVIWIN, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // 애플리케이션 초기화를 수행합니다:
    if (!InitInstance (hInstance, nCmdShow))
    {
        return 0;
    }

	SceneInit();

	//const int postpone = 3;
	std::vector<std::string> rbNames;
	int numAllFramesRBs = optitrk::GetRigidBodies(&rbNames);

	static navihelpers::concurrent_queue<navihelpers::track_info> track_que(10);
	g_track_que = &track_que;
    std::atomic_bool tracker_alive{true};
	std::thread tracker_processing_thread([&]() {
		using namespace std;
		using namespace navihelpers;
		using namespace glm;
		while (tracker_alive)
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

			auto GetWsMarkersFromSelectedMarkerNames = [&trk_info](const std::map<std::string, int>& selectedMkNames, std::vector<glm::fvec3>& selectedMks) {
			
				int numMks = (int)selectedMkNames.size();
				selectedMks.assign(numMks, glm::fvec3(0));
			
				int numSuccess = 0;
				for (auto it = selectedMkNames.begin(); it != selectedMkNames.end(); it++) {
					std::map<navihelpers::track_info::MKINFO, std::any> mkInfo;
					if (trk_info.GetMarkerByName(it->first, mkInfo)) {
						numSuccess++;
						glm::fvec3 pos = std::any_cast<glm::fvec3>(mkInfo[navihelpers::track_info::MKINFO::POSITION]);
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
			if (g_optiMode != OPTTRK_THREAD_FREE) {
				std::vector<glm::fvec3> selectedMks;
				GetWsMarkersFromSelectedMarkerNames(g_selectedMkNames, selectedMks);

				// if you want to use switch with g_optiMode, make sure thread-safe of g_optiMode
				if (g_optiMode == OPTTRK_THREAD_C_ARM_REGISTER) {
					optitrk::SetRigidBody("c-arm", g_selectedMkNames.size(), (float*)&selectedMks[0]);
					UpdateRigidBodiesInThread();
					g_optiMode = OPTTRK_THREAD_FREE;
				}
				else if (g_optiMode == OPTTRK_THREAD_TOOL_REGISTER) {
					if (std::time(NULL) - timeBegin > 5) {
						cout << "\nregister tool success!" << endl;
						int numSelectedMKs = (int)g_selectedMkNames.size();
						optitrk::SetRigidBody("tool", numSelectedMKs - 1, (float*)&selectedMks[0]);

						glm::fvec3 mkPivotPos = glm::fvec3(0, 0, 0);
						for (int i = 0; i < numSelectedMKs - 1; i++) {
							mkPivotPos += selectedMks[i];
						}

						g_toolLocalPoints = selectedMks;
						mkPivotPos /= (float)numSelectedMKs;
						for (int i = 0; i < numSelectedMKs; i++) {
							g_toolLocalPoints[i] -= mkPivotPos;
						}

						// use cam direction...

						UpdateRigidBodiesInThread();
						g_optiMode = OPTTRK_THREAD_FREE;
					}
				}
			}
			else {
				timeBegin = std::time(NULL);
			}

			track_que.push(trk_info);
		}
	});

	UINT_PTR customData = (UINT_PTR)&track_que;

	//UpdateFrame(csvData);
	//Render();
	SetTimer(g_hWnd, customData, 10, TimerProc);

	/////////////////////////////////////////////////////////
	// TCP routine
#define CFG_LISTEN_PORT 22222
#define CFG_SIZE_MAX_FD 1000
#define CFG_SIZE_MAX_RECV_BUFFER 5000

	std::atomic_bool network_alive{ true };
	std::thread network_processing_thread([&]() {
		WSADATA wsaData;
		WSAStartup(MAKEWORD(2, 2), &wsaData);


		/* create socket */
		SOCKET serverSocket;
		serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (serverSocket < 0) {
			printf("can not create socket\n");
			return -1;
		}

		/* bind socket */
		struct sockaddr_in serverAddr;
		memset(&serverAddr, 0, sizeof(serverAddr));
		serverAddr.sin_family = AF_INET;
		serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		serverAddr.sin_port = htons(CFG_LISTEN_PORT);

		if (bind(serverSocket, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
		{
			printf("bind error\n");
			return -1;
		}

		/* listen */
		if (listen(serverSocket, CFG_SIZE_MAX_FD) == SOCKET_ERROR) {
			printf("listen error\n");
			return -1;
		}

		printf("listen %d port...\n", CFG_LISTEN_PORT);

		/* fd_set에 서버 소켓 설정 */
		fd_set reads;
		fd_set copy_reads;
		FD_ZERO(&reads);
		FD_SET(serverSocket, &reads);
		int maxFd = serverSocket;

		/* select timeout 1초 설정 */
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		int clientSocket = -1;
		struct sockaddr_in clientAddr;
		unsigned int clientAddrSize = sizeof(clientAddr);

		const int bufferSize = CFG_SIZE_MAX_RECV_BUFFER;
		char buffer[bufferSize];

		int totalBufferNum;
		int BufferNum;
		int readBytes;
		long file_size;
		long totalReadBytes;
		uint numTimeOut = 0;

		while (network_alive) {

			while (true && !download_completed) {
				/* timeout 시간 재설정 */
				timeout.tv_sec = 1;
				timeout.tv_usec = 0;

				/* fd_set 복사 */
				copy_reads = reads;

				/* select */
				int result = select(maxFd + 1, &copy_reads, NULL, NULL, &timeout);

				/* select 결과 예외처리 */
				if (result < 0) {
					printf("\rselect error");
					fflush(stdout);
					break;
				}

				if (result == 0) {
					printf("\rtime out...%d", ++numTimeOut);
					fflush(stdout);
					break;
				}

				/* server listen socket에 input event가 있는지 확인 */
				if (FD_ISSET(serverSocket, &copy_reads)) {
					/* input event가 있으면 accept */
					clientSocket = accept(serverSocket,
						(struct sockaddr*)&clientAddr,
						(int*)&clientAddrSize);

					if (clientSocket < 0) {
						printf("\ninvalid client socket. [clientSocket:%d]\n",
							clientSocket);
						break;
					}

					printf("\naccept new client. [clientSocket:%d]\n",
						clientSocket);

					//int safeTmpBytes = 5000;
					//readBytes = recv(clientSocket, buffer, bufferSize, 0);
					//safeTmpBytes = readBytes;

					file_size = SCANIMG_W * SCANIMG_H;// atol(buffer);

					unsigned char* data = new unsigned char[file_size];
					totalReadBytes = 0;

					//bool isOffsetProcCompleted = false;

					//FILE* fp;
					//fopen_s(&fp, (folder_capture + "result.raw").c_str(), "wb");
					std::vector<char> bufferTmp(file_size * 4);
					while (file_size > totalReadBytes) {
						readBytes = recv(clientSocket, buffer, bufferSize, 0);
						//if (totalTmpBytes < 5000) continue;
						if (readBytes < 0) break;

						//fwrite(buffer, sizeof(char), readBytes, fp);
						memcpy(&bufferTmp[totalReadBytes], buffer, readBytes);

						totalReadBytes += readBytes;
						printf("\rIn progress: %d/%d byte(s) [%d]", totalReadBytes, file_size, (int)std::min((totalReadBytes * 100.f) / file_size, 100.f));
						fflush(stdout);
					}
					if (readBytes > 0) {
						download_completed = true;
						memcpy(g_curScanImg.ptr(), &bufferTmp[0], file_size);
						//cv::imshow("test", g_curScanImg);
						//cv::waitKey(1);
					}

					//fclose(fp);
					/* client socket 종료 */
					closesocket(clientSocket);
					printf("close [clientSocket:%d]\n", clientSocket);
				}
			}

		}

		if (serverSocket != -1) {
			closesocket(serverSocket);
		}
		WSACleanup();
	});


    MSG msg;
	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_SNAVIWIN));

    // 기본 메시지 루프입니다:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			if (IsDialogMessage(g_hWndDialog1, &msg) && msg.message == WM_KEYDOWN) {
				DiagProc1(g_hWndDialog1, msg.message, msg.wParam, msg.lParam);
			}
			else {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
        }
	}


	tracker_alive = false; // make the thread finishes, this setting should be located right before the thread join
	tracker_processing_thread.join();
	network_alive = false;
	network_processing_thread.join();

	optitrk::DeinitOptiTrackLib();
	vzm::DeinitEngineLib();

	//end = GetMicroCounter();
	//printf("Elapsed Time (micro seconds) : %d", end - start);
	GdiplusShutdown(gdiplusToken);

    return (int) msg.wParam;
}



//
//  함수: MyRegisterClass()
//
//  용도: 창 클래스를 등록합니다.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style          = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_SNAVIWIN));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_SNAVIWIN);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

//
//   함수: InitInstance(HINSTANCE, int)
//
//   용도: 인스턴스 핸들을 저장하고 주 창을 만듭니다.
//
//   주석:
//
//        이 함수를 통해 인스턴스 핸들을 전역 변수에 저장하고
//        주 프로그램 창을 만든 다음 표시합니다.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // 인스턴스 핸들을 전역 변수에 저장합니다.

   //WND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	//   CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);
   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	   -1915, 5, DESIRED_SCREEN_W, DESIRED_SCREEN_H, nullptr, nullptr, hInstance, nullptr); //-1915
   //HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	//   CW_USEDEFAULT, 0, DESIRED_SCREEN_W, DESIRED_SCREEN_H, nullptr, nullptr, hInstance, nullptr);
   if (!hWnd)
   {
      return FALSE;
   }
   g_hWnd = hWnd;

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   g_hWndDialog1 = CreateDialog(hInst, MAKEINTRESOURCE(IDD_FORMVIEW), hWnd, DiagProc1);

   RECT rc1;
   GetWindowRect(g_hWnd, &rc1);
   SetWindowPos(g_hWndDialog1, 0, rc1.right + 5, 10, DESIRED_SCREEN_W, DESIRED_SCREEN_H, SWP_SHOWWINDOW);
   if (!g_hWndDialog1)
   {
	   return FALSE;
   }
   ShowWindow(g_hWndDialog1, SW_SHOW);
   UpdateWindow(g_hWndDialog1);

   return TRUE;
}

// DOJO : 스캔 시 (carmIdx) 저장된 이미지(이미지 texture 가 포함된 plane actor)에 fitting 되도록 cpInterCams (by slerp) 지정 
// slerp 의 시작점은 현재 cidCam 카메라의 pose, 끝점은 스캔 시 (carmIdx) 저장된 이미지 fitting 카메라 pose
// 매 스캔 시에 호출되야 하는 SaveAndChangeViewState() 에서 호출함
void MoveCameraToCArmView(const int carmIdx, const int cidCam,
	std::vector<vzm::CameraParameters>& cpInterCams, int& arAnimationKeyFrame) {

	vzm::CameraParameters cpCam;
	if (!vzm::GetCameraParams(cidCam, cpCam))
		return;

	cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
	cv::Mat cameraMatrix;
	fs["K"] >> cameraMatrix;
	fs.release();

	int aidCArmPlane = vzmutils::GetSceneItemIdByName("CArm Plane:test" + std::to_string(carmIdx));
	if (aidCArmPlane == 0)
		return;

	vzm::ActorParameters apCArmPlane;
	vzm::GetActorParams(aidCArmPlane, apCArmPlane);
	glm::fmat4x4 matCA2WS = apCArmPlane.script_params.GetParam("matCA2WS", glm::fmat4x4(1));
	glm::ivec2 imageWH = apCArmPlane.script_params.GetParam("imageWH", glm::ivec2(0));

	vzm::CameraParameters cpNewCam = cpCam;

	glm::fvec3 posPrev = *(glm::fvec3*)cpCam.pos;
	glm::fvec3 viewPrev = *(glm::fvec3*)cpCam.view;
	glm::fvec3 upPrev = *(glm::fvec3*)cpCam.up;

	UINT widthWindow = cpCam.w;
	UINT heightWindow = cpCam.h;

	const float intrinsicRatioX = (float)imageWH.x / widthWindow;
	const float intrinsicRatioY = (float)imageWH.y / heightWindow;

	cpNewCam.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_INTRINSICS;
	glm::fvec3 pos(0, 0, 0);
	glm::fvec3 view(0, 0, 1);
	glm::fvec3 up(0, -1, 0);
	pos = *(glm::fvec3*)cpNewCam.pos = vzmutils::transformPos(pos, matCA2WS);
	view = *(glm::fvec3*)cpNewCam.view = vzmutils::transformVec(view, matCA2WS);
	up = *(glm::fvec3*)cpNewCam.up = vzmutils::transformVec(up, matCA2WS);
	cpNewCam.fx = cameraMatrix.at<double>(0, 0) / intrinsicRatioX;
	cpNewCam.fy = cameraMatrix.at<double>(1, 1) / intrinsicRatioY;
	cpNewCam.sc = cameraMatrix.at<double>(0, 1);
	cpNewCam.cx = cameraMatrix.at<double>(0, 2) / intrinsicRatioX;
	cpNewCam.cy = cameraMatrix.at<double>(1, 2) / intrinsicRatioY;
	cpNewCam.np = 0.2;

	glm::fquat q1 = glm::quatLookAtRH(glm::normalize(viewPrev), upPrev); // to world
	//glm::fvec3 _o(0, 0, 0);
	//glm::fmat4x4 m1 = glm::lookAtRH(_o, viewPrev, upPrev);
	//glm::fmat4x4 m2 = glm::toMat4(q1);
	//glm::fmat4x4 m3 = glm::inverse(m2);
	glm::fquat q2 = glm::quatLookAtRH(glm::normalize(view), up);

	float range = std::max((float)(numAnimationCount - 1), 1.f);
	for (int i = 0; i < numAnimationCount; i++) {
		float t = (float)i / range; // 0 to 1
		glm::fquat q = glm::slerp(q1, q2, t);
		glm::fmat4x4 invMatLookAt = glm::toMat4(q);
		//glm::fmat4x4 invMat = glm::inverse(matLookAt);
		glm::fvec3 _view(0, 0, -1);
		glm::fvec3 _up(0, 1, 0);
		_view = vzmutils::transformVec(_view, invMatLookAt);
		_up = vzmutils::transformVec(_up, invMatLookAt);

		vzm::CameraParameters& _cpCam = cpInterCams[i];
		_cpCam = cpNewCam;
		*(glm::fvec3*)_cpCam.pos = posPrev + (pos - posPrev) * t;
		*(glm::fvec3*)_cpCam.view = _view;
		*(glm::fvec3*)_cpCam.up = _up;

#define INTERPOLCAM(PARAM) _cpCam.PARAM = cpCam.PARAM + (cpNewCam.PARAM - cpCam.PARAM) * t;

		//INTERPOLCAM(w);
		//INTERPOLCAM(h);
		INTERPOLCAM(fx);
		INTERPOLCAM(fy);
		INTERPOLCAM(sc);
		INTERPOLCAM(cx);
		INTERPOLCAM(cy);
	}
	arAnimationKeyFrame = 0;
}

// DOJO : 스캔 시 정보를 저장할 파일
// 현 버전에서는 carm_intrinsics.txt (presetting) 에 저장된 intrinsic parameter 와
// rb2carm1.txt (presetting + 매 스캔 시 저장됨) 에 저장된 c-arm source pose (extrinsic parameter) 를 로드하여 이를 파일로 저장
// TODO : 추후 해당 부분을 file to file 이 아닌, memory to memory 로 관리해야 함
// TODO : 매 스캔마다 camera intrinsics 와 extrinsics 를 계산하도록 수정해야 함
auto StoreParams = [](const std::string& paramsFileName, const glm::fmat4x4& matRB2WS, const std::string& imgFileName) {

	cv::FileStorage __fs(folder_trackingInfo + paramsFileName, cv::FileStorage::Mode::WRITE);

	cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
	cv::Mat matK;
	fs["K"] >> matK;
	__fs << "K" << matK;
	cv::Mat distCoeffs;
	fs["DistCoeffs"] >> distCoeffs;
	__fs << "DistCoeffs" << distCoeffs;

	fs.open(folder_trackingInfo + "rb2carm1.txt", cv::FileStorage::Mode::READ);
	cv::Mat rvec, tvec;
	fs["rvec"] >> rvec;
	fs["tvec"] >> tvec;
	__fs << "rvec" << rvec;
	__fs << "tvec" << tvec;
	fs.release();

	cv::Mat ocvRb(4, 4, CV_32FC1);
	memcpy(ocvRb.ptr(), glm::value_ptr(matRB2WS), sizeof(float) * 16);
	__fs << "rb2wsMat" << ocvRb;
	__fs << "imgFile" << imgFileName;
	__fs.release();
};

// DOJO : 매 스캔 시 호출 (c-arm 스캔 정보를 scene 에 적용), network thread 에서 호출되야 함
// AP 인지 LATERAL 인지 자동으로 확인하고, 1 이나 2번 할당하는 작업 필요
auto SaveAndChangeViewState = [](const int keyParam, const int sidScene, const int cidCam,
	std::vector<vzm::CameraParameters>& cpInterCams, int& arAnimationKeyFrame) {
	using namespace std;
	static char LOADKEYS[10] = { '1' , '2', '3', '4', '5', '6', '7', '8', '9', '0' };
	for (int i = 0; i < 10; i++) {
		if (keyParam == LOADKEYS[i]) {

			if (download_completed) {
				glm::fvec3 rotAxisAvr = glm::fvec3(0, 0, 0);
				float rotAngleAvr = 0;
				glm::fvec3 trAvr = glm::fvec3(0, 0, 0);
				int captureCount = 0;
				const int maxSamples = 30;
				const float normalizedNum = 1.f / (float)maxSamples;
				while (captureCount < maxSamples) {
					navihelpers::track_info trackInfo;
					g_track_que->wait_and_pop(trackInfo);
					glm::fquat q;
					glm::fvec3 t;
					if (trackInfo.GetRigidBodyQuatTVecByName("c-arm", &q, &t)) {
						float quatMagnitude = glm::length(q);
						q /= quatMagnitude;

						float rotationAngle = 2.0f * acos(q.w);

						float sinAngle = sin(rotationAngle / 2.0f);
						glm::fvec3 rotationAxis = glm::fvec3(q.x, q.y, q.z) / sinAngle;

						rotAngleAvr += rotationAngle * normalizedNum;
						rotAxisAvr += rotationAxis * normalizedNum;

						trAvr += t * normalizedNum;
						captureCount++;
					}
					//std::cout << "\n" <<"Capturing... " << captureCount << std::endl;
					printf("\rCapturing... %d", captureCount);
					fflush(stdout);
				}
				std::cout << "\n" <<"Capturing... DONE!" << std::endl;
				rotAxisAvr = glm::normalize(rotAxisAvr);
				glm::fmat4x4 mat_r = glm::rotate(rotAngleAvr, rotAxisAvr);
				glm::fmat4x4 mat_t = glm::translate(trAvr);
				glm::fmat4x4 matRB2WS = mat_t * mat_r;

				cv::Mat downloadImg;
				cv::cvtColor(g_curScanImg, downloadImg, cv::COLOR_GRAY2RGB);
				string downloadImgFileName = folder_trackingInfo + "test" + to_string(i) + ".png";
				cv::imwrite(downloadImgFileName, downloadImg);
				StoreParams("test" + to_string(i) + ".txt", matRB2WS, downloadImgFileName);
				download_completed = false;
				std::cout << "\nSTORAGE COMPLETED!!!" << std::endl;

				auto it = g_mapAidGroupCArmCam.find(EXTRINSIC_PHANTOM_IMG_INDEX);
				if (it != g_mapAidGroupCArmCam.end()) 
					vzm::RemoveSceneItem(it->second, true);
			}
			else { // !download_completed

			}

			// load case
			auto it = g_mapAidGroupCArmCam.find(i + 1);
			if (it != g_mapAidGroupCArmCam.end())
				vzm::RemoveSceneItem(it->second, true);
			int aidGroup = RegisterCArmImage(sidScene, folder_trackingInfo + "test" + to_string(i) + ".txt", "test" + to_string(i));
			if (aidGroup == -1)
				break;
			g_mapAidGroupCArmCam[i + 1] = aidGroup;
			MoveCameraToCArmView(i, cidCam, cpInterCams, arAnimationKeyFrame);
			break;
		}
	}
};

//
//  함수: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  용도: 주 창의 메시지를 처리합니다.
//
//  WM_COMMAND  - 애플리케이션 메뉴를 처리합니다.
//  WM_PAINT    - 주 창을 그립니다.
//  WM_DESTROY  - 종료 메시지를 게시하고 반환합니다.
//
//

// DOJO IMPORTANT NOTE :
// key 1 ~ 9 : save and add the scan geometry to the scene (global key event)
// key S : save current camera state (main window only)
// key L : load the latest-stored camera state (main window only)
// key C : calibration mode for selecting markers whose position is used to compute c-arm extrinsic calibration
// key U : update c-arm marker set
// key X : compute c-arm extrinsics based on the c-arm marker set and the selected marker positions

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
	int cidCam1 = vzmutils::GetSceneItemIdByName(g_camName);
	float scene_stage_scale = 5.f;

	glm::fvec3 scene_stage_center = glm::fvec3();
	vzm::CameraParameters cpCam1;
	if (sidScene != 0 && cidCam1 != 0) {
		vzm::GetCameraParams(cidCam1, cpCam1);

		scene_stage_scale = glm::length(scene_stage_center - *(glm::fvec3*)cpCam1.pos) * 1.0f;
	}

	static vzmutils::GeneralMove general_move;
	static bool calribmodeToggle = false;

    switch (message)
	{
	case WM_KEYDOWN:
	{
		if (g_optiMode == OPTTRK_THREAD_TOOL_REGISTER) break;
		using namespace std;
		switch (wParam) {
			case char('S') :
			{
				if (calribmodeToggle) break;

				cv::FileStorage fs(folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::WRITE);
				cv::Mat ocvVec3(1, 3, CV_32FC1);
				memcpy(ocvVec3.ptr(), cpCam1.pos, sizeof(float) * 3);
				fs.write("POS", ocvVec3);
				memcpy(ocvVec3.ptr(), cpCam1.view, sizeof(float) * 3);
				fs.write("VIEW", ocvVec3);
				memcpy(ocvVec3.ptr(), cpCam1.up, sizeof(float) * 3);
				fs.write("UP", ocvVec3);
				fs.release();
				break;
			}
			case char('L') :
			{
				if (calribmodeToggle) break;

				cv::FileStorage fs(folder_data + "SceneCamPose.txt", cv::FileStorage::Mode::READ);
				if (fs.isOpened()) {
					cv::Mat ocvVec3;
					fs["POS"] >> ocvVec3;
					memcpy(cpCam1.pos, ocvVec3.ptr(), sizeof(float) * 3);
					fs["VIEW"] >> ocvVec3;
					memcpy(cpCam1.view, ocvVec3.ptr(), sizeof(float) * 3);
					fs["UP"] >> ocvVec3;
					memcpy(cpCam1.up, ocvVec3.ptr(), sizeof(float) * 3);
					fs.release();
				}
				vzm::SetCameraParams(cidCam1, cpCam1);
				break;
			}
			case char('C') :
			{
				calribmodeToggle = !calribmodeToggle;

				vzm::TextItem textItem;
				textItem.textStr = calribmodeToggle? "SELECTING MARKER MODE" : "";
				textItem.fontSize = 30.f;
				textItem.iColor = 0xFFFF00;
				textItem.posScreenX = 0;
				textItem.posScreenY = 0;

				if (!calribmodeToggle) {
					g_testMKs.clear();
					g_selectedMkNames.clear();
					int aidTestGroup = vzmutils::GetSceneItemIdByName("testMK Group");
					if (aidTestGroup != 0)
						vzm::RemoveSceneItem(aidTestGroup);
				}

				cout << "\n" << "calibration mode " << (calribmodeToggle ? "ON" : "OFF") << endl;

				using namespace glm;
				fmat4x4 matCam2WS;
				optitrk::GetCameraLocation(0, __FP matCam2WS);

				*(fvec3*)cpCam1.pos = vzmutils::transformPos(fvec3(0, 0, 0), matCam2WS);
				*(fvec3*)cpCam1.view = vzmutils::transformVec(fvec3(0, 0, -1), matCam2WS);
				*(fvec3*)cpCam1.up = vzmutils::transformVec(fvec3(0, 1, 0), matCam2WS);
				cpCam1.projection_mode = vzm::CameraParameters::ProjectionMode::CAMERA_FOV;
				cpCam1.fov_y = glm::pi<float>() / 2.f;
				cpCam1.aspect_ratio = cpCam1.ip_w / cpCam1.ip_h;
				cpCam1.np = 0.3f;

				cpCam1.text_items.SetParam("OPERATION_MODE", textItem);

				vzm::SetCameraParams(cidCam1, cpCam1);
				break;
			}
			case char('U') :
			{
				if (!calribmodeToggle)
					return 0;
				// to do
				// update "c-arm" RB
				int numSelectedMKs = (int)g_selectedMkNames.size();
				if (numSelectedMKs < 3) {
					cout << "\n" << "at least 3 points are needed to register a rigid body!! current # of points : " << numSelectedMKs << endl;
					return 0;
				}

				cout << "\n" << "rigidbody for c-arm is registered with " << numSelectedMKs << " points" << endl;

				g_optiMode = OPTTRK_THREAD_C_ARM_REGISTER;

				while (g_optiMode == OPTTRK_THREAD_C_ARM_REGISTER) { Sleep(2); }

				int aidRbCarm = vzmutils::GetSceneItemIdByName("c-arm");
				vzm::RemoveSceneItem(aidRbCarm);
				// 이 떄마다, "c-arm" 관련 scene item 모두 지우고, 해당 element 지우기..
				break;
			}
			case char('D') : {
				for (int i = 0; i++; i < 9) {
					auto it = g_mapAidGroupCArmCam.find(i + 1);
					if (it != g_mapAidGroupCArmCam.end())
						vzm::RemoveSceneItem(it->second, true);
				}
				g_mapAidGroupCArmCam.clear();
				break;
			}
			case char('T') : {
				if (!calribmodeToggle) {
					cout << "\n" << "not calibration mode!!" << endl;
					return 0;
				}
				if (g_selectedMkNames.size() < 4) {
					cout << "\n" << "at least 4 points (last one is for tool tip) are needed!!" << endl;
					return 0;
				}

				// 툴 등록하기..
				g_optiMode = OPTTRK_THREAD_TOOL_REGISTER;
				break;
			}
			//case char('E') : {
			//	if (!calribmodeToggle) {
			//		cout << "\n" << "not calibration mode!!" << endl;
			//		return 0;
			//	}
			//	// 툴 끝점 등록하기..
			//	break;
			//}
			case char('X') :
			{
				if (!calribmodeToggle) {
					cout << "\n" <<"not calibration mode!!" << endl;
					return 0;
				}
//#define MYDEFINE
#ifdef MYDEFINE
				download_completed = true;
				g_curScanImg = cv::imread(folder_trackingInfo + "test" + to_string(EXTRINSIC_PHANTOM_IMG_INDEX) + ".png");
				cv::cvtColor(g_curScanImg, g_curScanImg, cv::COLOR_RGB2GRAY);
#else
				if (!download_completed) {
					cout << "\n" <<"no download image!!" << endl;
					return 0;
				}
#endif
				cv::Mat source2detImg;
				if (FLIP_SCANIMAGE)
					cv::flip(g_curScanImg, source2detImg, 1);
				else
					source2detImg = g_curScanImg;

				if (g_selectedMkNames.size() != 4) {
					cout << "\n4 points are needed!!" << endl;
					return 0;
				}

				// now g_curScanImg is valid
				// test3.png is for extrinsic calibration image
				cv::Mat downloadImg;
				cv::cvtColor(g_curScanImg, downloadImg, cv::COLOR_GRAY2RGB);
				string downloadImgFileName = folder_trackingInfo + "test" + to_string(EXTRINSIC_PHANTOM_IMG_INDEX) + ".png";
				cv::imwrite(downloadImgFileName, downloadImg);
				// now tracking info of c-arm scan is valid.
				std::cout << "\n" << "SCAN IMAGE LOAD COMPLETED!!!" << std::endl;

				// 1st, get c-arm rb info.
				glm::fmat4x4 matRB2WS, matWS2RB;
				{
					glm::fvec3 rotAxisAvr = glm::fvec3(0, 0, 0);
					float rotAngleAvr = 0;
					glm::fvec3 trAvr = glm::fvec3(0, 0, 0);
					int captureCount = 0;
					const int maxSamples = 10;
					const float normalizedNum = 1.f / (float)maxSamples;
					while (captureCount < maxSamples) {
						navihelpers::track_info trackInfo;
						g_track_que->wait_and_pop(trackInfo);
						glm::fquat q;
						glm::fvec3 t;
						if (trackInfo.GetRigidBodyQuatTVecByName("c-arm", &q, &t)) {
							float quatMagnitude = glm::length(q);
							q /= quatMagnitude;

							float rotationAngle = 2.0f * acos(q.w);

							float sinAngle = sin(rotationAngle / 2.0f);
							glm::fvec3 rotationAxis = glm::fvec3(q.x, q.y, q.z) / sinAngle;

							rotAngleAvr += rotationAngle * normalizedNum;
							rotAxisAvr += rotationAxis * normalizedNum;

							trAvr += t * normalizedNum;
							captureCount++;
						}
						else {
							cout << "\n" << "failure to c-arm tracking!!" << endl;
							return 0;
						}
						//std::cout << "\n" <<"Capturing... " << captureCount << std::endl;
						printf("\rCapturing... %d", captureCount);
						fflush(stdout);
						Sleep(10);
					}
					std::cout << "\n" << "Capturing... DONE!" << std::endl;
					rotAxisAvr = glm::normalize(rotAxisAvr);
					glm::fmat4x4 mat_r = glm::rotate(rotAngleAvr, rotAxisAvr);
					glm::fmat4x4 mat_t = glm::translate(trAvr);
					matRB2WS = mat_t * mat_r;
					matWS2RB = glm::inverse(matRB2WS);
				}

				// get 2d mks info from x-ray img (calculImg)
				std::vector<cv::Point2f> points2d;
				mystudents::Get2DPostionsFromLegoPhantom(source2detImg, points2d);
				if (PHANTOM_MODE == "LEGO") {
					if (points2d.size() != 60) {
						cout << "\n# of circles must be 60 if PHANTOM_MODE is LEGO" << endl;
						return 0;
					}
				}
				else if (PHANTOM_MODE == "FILM") {
					if (points2d.size() != 77) {
						cout << "\n# of circles must be 77 if PHANTOM_MODE is LEGO" << endl;
						return 0;
					}
				}
				else assert(0);

				navihelpers::track_info trackInfo;
				g_track_que->wait_and_pop(trackInfo);
				std::vector<cv::Point3f> posWsMarkers(g_selectedMkNames.size());
				for (auto it = g_selectedMkNames.begin(); it != g_selectedMkNames.end(); it++) {
					std::map<navihelpers::track_info::MKINFO, std::any> mkInfo;
					trackInfo.GetMarkerByName(it->first, mkInfo);
					glm::fvec3 pos = std::any_cast<glm::fvec3>(mkInfo[navihelpers::track_info::MKINFO::POSITION]);
					posWsMarkers[it->second] = *(cv::Point3f*)&pos;
				}

				if (PHANTOM_MODE == "LEGO") {
					// check the selecting direction
					glm::fvec3 v01 = *(glm::fvec3*)&posWsMarkers[1] - *(glm::fvec3*)&posWsMarkers[0];
					glm::fvec3 v03 = *(glm::fvec3*)&posWsMarkers[3] - *(glm::fvec3*)&posWsMarkers[0];
					glm::fvec3 selDir = glm::cross(v01, v03);
					glm::fvec3 posCenterCArmRB = vzmutils::transformPos(glm::fvec3(0, 0, 0), matRB2WS);
					glm::fvec3 cal2DetDir = posCenterCArmRB - *(glm::fvec3*)&posWsMarkers[0];
					if (glm::dot(selDir, cal2DetDir) < 0) {
						std::cout << "\ncorrecting the selecting direction!" << std::endl;
						// 0123 to 1032
						std::vector<cv::Point3f> posWsMarkersTmp = posWsMarkers;
						posWsMarkers[0] = posWsMarkersTmp[1];
						posWsMarkers[1] = posWsMarkersTmp[0];
						posWsMarkers[2] = posWsMarkersTmp[3];
						posWsMarkers[3] = posWsMarkersTmp[2];
					}
				}

				// compute the estimated marker positions on the phantom (WS)
				std::vector<cv::Point3f> points3d;
				mystudents::Get3DPostionsFromLegoPhantom2(0.007f, posWsMarkers[0], posWsMarkers[1], posWsMarkers[2], posWsMarkers[3], points3d);
				g_testMKs.clear();
				g_testMKs.assign(points3d.size(), glm::fvec3(0));
				memcpy(&g_testMKs[0], &points3d[0], sizeof(glm::fvec3) * points3d.size());

				// important! the 3d points must be defined in C-Arm RB space
				// because we want to get the transform btw C-arm cam (x-ray source) space and C-Arm RB space
				for (int i = 0; i < points3d.size(); i++) {
					*(glm::fvec3*)&points3d[i] = vzmutils::transformPos(*(glm::fvec3*)&points3d[i], matWS2RB);
				}
				
				cv::Mat cameraMatrix, distCoeffs;
				{
					cv::FileStorage fs(folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
					fs["K"] >> cameraMatrix;
					fs["DistCoeffs"] >> distCoeffs;
					fs.release();
				}

				cv::Mat rvec, tvec;
				cv::solvePnP(points3d, points2d, cameraMatrix, distCoeffs, rvec, tvec);
				//std::cout << A << "\n";
				//std::cout << rvec << "\n";
				//std::cout << tvec << "\n";
				// get 3D position
				// determine ap or lateral
				// check residual 
				// get 3d mks info from c-arm rb
				// solve pnp
				// update...

				// used in SaveAndChangeViewState
				{
					cv::FileStorage fs(folder_trackingInfo + "rb2carm1.txt", cv::FileStorage::Mode::WRITE);
					fs.write("rvec", rvec);
					fs.write("tvec", tvec);
					fs.release();
				}

				{
					auto ComputeReprojErr = [](const std::vector<cv::Point2f>& pts2d, const std::vector<cv::Point2f>& pts2d_reproj, float& maxErr) {
						float avrDiff = 0;
						maxErr = 0;
						int numPts = (int)pts2d.size();
						for (int i = 0; i < (int)pts2d.size(); i++) {
							float diff = glm::distance(*(glm::fvec2*)&pts2d[i], *(glm::fvec2*)&pts2d_reproj[i]);
							maxErr = std::max(maxErr, diff);
							avrDiff += diff / (float)numPts;
						}
						return avrDiff;
					};

					// Reproject the 3D points onto the image plane using the camera calibration
					std::vector<cv::Point2f> imagePointsReprojected;
					cv::projectPoints(points3d, rvec, tvec, cameraMatrix, distCoeffs, imagePointsReprojected);
				
					// Compute the reprojection error
					float maxErr = 0;
					float reprojectionError = ComputeReprojErr(points2d, imagePointsReprojected, maxErr);
					//double reprojectionError = cv::norm(pts2ds, imagePointsReprojected, cv::NORM_L2) / pts2ds.size();
					cout << "\nreprojectionError : " << reprojectionError << ", max error : " << maxErr << endl;
				}

				vzm::RemoveSceneItem(1, true);
				vzm::RemoveSceneItem(2, true);

				// store the info.
				// note rb2carm1.txt is used in StoreParams!
				StoreParams("test" + to_string(EXTRINSIC_PHANTOM_IMG_INDEX) + ".txt", matRB2WS, downloadImgFileName);
				std::cout << "\n" << "STORAGE COMPLETED!!!" << std::endl;

				//download_completed = false; 
				wParam = char('3'); // to call download_completed = false;
				break;
			}
		}

		SaveAndChangeViewState(wParam, sidScene, cidCam1, cpInterCams1, arAnimationKeyFrame1);
		break;
	}
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            // 메뉴 선택을 구문 분석합니다:
            switch (wmId)
            {
            case IDM_ABOUT:
                DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
                break;
            case IDM_EXIT:
                DestroyWindow(hWnd);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
		break;
	case WM_SIZE:
		{
			if (cidCam1 != 0) {
				RECT rc;
				GetClientRect(hWnd, &rc);
				UINT width = rc.right - rc.left;
				UINT height = rc.bottom - rc.top;

				cpCam1.w = width;
				cpCam1.h = height;
				vzm::SetCameraParams(cidCam1, cpCam1);
			}
		}
		break;
    case WM_PAINT:
        {
			if (sidScene != 0) 
			{
				if (USE_WHND) {

					PAINTSTRUCT ps;
					HDC hdc = BeginPaint(hWnd, &ps);
					//// TODO: 여기에 hdc를 사용하는 그리기 코드를 추가합니다...
					EndPaint(hWnd, &ps);

					//vzm::PresentHWND(hWnd); // this calls WM_PAINT again...
				}
				else {
					UpdateBMP(cidCam1, hWnd);
				} // no need to call invalidate
			}
        }
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
		break;
	case WM_LBUTTONDOWN:
	case WM_RBUTTONDOWN:
	{
		if (g_optiMode == OPTTRK_THREAD_TOOL_REGISTER) break;
		int x = GET_X_LPARAM(lParam);
		int y = GET_Y_LPARAM(lParam);
		if (x == 0 && y == 0) break;

		// renderer does not need to be called
		glm::ivec2 pos_ss = glm::ivec2(x, y);

		if (calribmodeToggle) {
			int aidPicked = 0;
			glm::fvec3 posPick;
			if (vzm::PickActor(aidPicked, __FP posPick, x, y, cidCam1)) {
				std::string actorName;
				vzm::GetSceneItemName(aidPicked, actorName);
				if (g_selectedMkNames.find(actorName) == g_selectedMkNames.end()) {
					int idx = g_selectedMkNames.size();
					g_selectedMkNames[actorName] = idx;
					std::cout << "\npicking count : " << g_selectedMkNames.size() << std::endl;
				}
			}
		}
		else {
			general_move.Start((int*)&pos_ss, cpCam1, scene_stage_center, scene_stage_scale);
		}
		break;
	}
	case WM_MOUSEMOVE:
	{
		if (calribmodeToggle) break;

		int x = GET_X_LPARAM(lParam);
		int y = GET_Y_LPARAM(lParam);

		if ((wParam & MK_LBUTTON) || (wParam & MK_RBUTTON))
		{
			glm::ivec2 pos_ss = glm::ivec2(x, y);
			if (wParam & MK_LBUTTON)
				general_move.PanMove((int*)&pos_ss, cpCam1);
			else if (wParam & MK_RBUTTON)
				general_move.RotateMove((int*)&pos_ss, cpCam1);

			vzm::SetCameraParams(cidCam1, cpCam1);
			//Render();
			//vzm::RenderScene(sidScene, cidCam1);
			//UpdateWindow(hWnd);
			//InvalidateRect(hWnd, NULL, FALSE);
		}
		else {
			int aidPickedMK = 0;
			glm::fvec3 pos_picked;
			if (vzm::PickActor(aidPickedMK, __FP pos_picked, x, y, cidCam1))
			{
				std::string mkName;
				vzm::GetNameBySceneItemID(aidPickedMK, mkName);
				vzm::ActorParameters apMK;
				vzm::GetActorParams(aidPickedMK, apMK);
				glm::fmat4x4 matWorld = *(glm::fmat4x4*)apMK.GetWorldTransform();
				glm::fvec3 posOrigin(0, 0, 0);
				glm::fvec3 posMK = vzmutils::transformPos(posOrigin, matWorld);
				std::cout << "\n" <<mkName << "is picked : " << ", Pos : " << posMK.x << ", " << posMK.y << ", " << posMK.z << std::endl;

			}
			
		}
		break;
	}
	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
		break;
	case WM_MOUSEWHEEL:
	{
		int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);

		// by adjusting cam distance
			if (zDelta > 0)
				*(glm::fvec3*)cpCam1.pos += scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam1.view);
			else
				*(glm::fvec3*)cpCam1.pos -= scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam1.view);

			vzm::SetCameraParams(cidCam1, cpCam1);
		//Render();
		//vzm::RenderScene(sidScene, cidCam1);

		//InvalidateRect(hWnd, NULL, FALSE);
		//UpdateWindow(hWnd);
	}
	case WM_ERASEBKGND:
		return TRUE; // tell Windows that we handled it. (but don't actually draw anything)
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }

    return 0;
}


LRESULT CALLBACK DiagProc1(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	int sidScene = vzmutils::GetSceneItemIdByName(g_sceneName);
	int cidCam2 = vzmutils::GetSceneItemIdByName(g_camName2);
	float scene_stage_scale = 5.f;
	glm::fvec3 scene_stage_center = glm::fvec3();
	vzm::CameraParameters cpCam2;
	if (sidScene != 0 && cidCam2 != 0) {
		vzm::GetCameraParams(cidCam2, cpCam2);

		scene_stage_scale = glm::length(scene_stage_center - *(glm::fvec3*)cpCam2.pos) * 1.0f;
	}

	static vzmutils::GeneralMove general_move;
	static std::set<int> excludedMKIndices;
	static bool mouseCamFlag = false;

	switch (message)
	{
	case WM_KEYDOWN:
	{
		SaveAndChangeViewState(wParam, sidScene, cidCam2, cpInterCams2, arAnimationKeyFrame2);
		break;
	}
	case WM_SIZE:
	{
		if (cidCam2 != 0) {
			RECT rc;
			GetClientRect(hDlg, &rc);
			UINT width = rc.right - rc.left;
			UINT height = rc.bottom - rc.top;

			cpCam2.w = width;
			cpCam2.h = height;
			vzm::SetCameraParams(cidCam2, cpCam2);
		}
		break;
	}
	case WM_PAINT:
	{
		if (sidScene != 0)
		{
			if (USE_WHND) {

				PAINTSTRUCT ps;
				HDC hdc = BeginPaint(hDlg, &ps);
				//// TODO: 여기에 hdc를 사용하는 그리기 코드를 추가합니다...
				EndPaint(hDlg, &ps);

				//vzm::PresentHWND(hWnd); // this calls WM_PAINT again...
			}
			else {
				UpdateBMP(cidCam2, hDlg);
			} // no need to call invalidate
		}
		break;
	}
	case WM_LBUTTONDOWN:
	case WM_RBUTTONDOWN:
	{
		int x = GET_X_LPARAM(lParam);
		int y = GET_Y_LPARAM(lParam);
		if (x == 0 && y == 0) break;

		if (wParam & MK_CONTROL) {
			// exclude the picking mk...
			int aidPickedMK = 0;
			glm::fvec3 pos_picked;
			if (vzm::PickActor(aidPickedMK, __FP pos_picked, x, y, cidCam2))
			{
				std::string mkName;
				vzm::GetNameBySceneItemID(aidPickedMK, mkName);
				int idx = std::stoi(mkName.substr(2, mkName.size() - 1));
				excludedMKIndices.insert(idx);
				std::cout << "\n" <<"picking index : " << idx << std::endl;

				std::cout << "\n" <<"excluded index list : ";
				for (auto it : excludedMKIndices) {
					std::cout << "\n" <<it << ", ";
				}
				std::cout << "\n" <<std::endl;

				for (int i = 0; i < 2; i++) {
					int aidMarker = vzmutils::GetSceneItemIdByName((i == 0 ? "Rb" : "Ws") + std::to_string(idx));
					vzm::ActorParameters apMarker;
					vzm::GetActorParams(aidMarker, apMarker);
					apMarker.is_visible = false;
					vzm::SetActorParams(aidMarker, apMarker);
				}
			}
		}
		else {
			// renderer does not need to be called
			glm::ivec2 pos_ss = glm::ivec2(x, y);
			general_move.Start((int*)&pos_ss, cpCam2, scene_stage_center, scene_stage_scale);
			mouseCamFlag = true;
		}
		break;
	}
	case WM_MOUSEMOVE:
	{
		int x = GET_X_LPARAM(lParam);
		int y = GET_Y_LPARAM(lParam);

		if (((wParam & MK_LBUTTON) || (wParam & MK_RBUTTON)) && mouseCamFlag)
		{
			glm::ivec2 pos_ss = glm::ivec2(x, y);
			if (wParam & MK_LBUTTON)
				general_move.PanMove((int*)&pos_ss, cpCam2);
			else if (wParam & MK_RBUTTON)
				general_move.RotateMove((int*)&pos_ss, cpCam2);

			vzm::SetCameraParams(cidCam2, cpCam2);
		}
		break;
	}
	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
		mouseCamFlag = false;
		break;
	case WM_MOUSEWHEEL:
	{
		int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);

		// by adjusting cam distance
		if (zDelta > 0)
			*(glm::fvec3*)cpCam2.pos += scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam2.view);
		else
			*(glm::fvec3*)cpCam2.pos -= scene_stage_scale * 0.1f * (*(glm::fvec3*)cpCam2.view);

		vzm::SetCameraParams(cidCam2, cpCam2);
		//Render();
		//vzm::RenderScene(sidScene, cidCam1);

		//InvalidateRect(hWnd, NULL, FALSE);
		//UpdateWindow(hWnd);
		break;
	}
	//case WM_ERASEBKGND:
	//	return TRUE; // tell Windows that we handled it. (but don't actually draw anything)
	//default:
	//	return DefWindowProc(hDlg, message, wParam, lParam);
	}

	return 0;
}

// 정보 대화 상자의 메시지 처리기입니다.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}
