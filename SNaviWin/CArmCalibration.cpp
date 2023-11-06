
#include "CArmCalibration.h"

//#include "ApiUtility.hpp"
#include "naviHelpers.hpp"
#include "nanoflann.hpp"


using namespace std;
using namespace cv;

__GC* __gc = NULL;

#define __gm2__ *(cv::Point2f*)
#define __gm3__ *(cv::Point3f*)


//using namespace nanoflann;
template <typename T, typename TT>
struct PointCloud
{
	//public:
	const uint num_vtx;
	const TT* pts;
	PointCloud(const TT* _pts, const uint _num_vtx) : pts(_pts), num_vtx(_num_vtx) { }

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return num_vtx; }

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline T kdtree_distance(const T* p1, const size_t idx_p2, size_t) const
	{
		const T d0 = p1[0] - pts[idx_p2].x;
		const T d1 = p1[1] - pts[idx_p2].y;
		const T d2 = p1[2] - pts[idx_p2].z;
		return d0 * d0 + d1 * d1 + d2 * d2;
	}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim == 0) return pts[idx].x;
		else if (dim == 1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX&) const { return false; }
};
typedef nanoflann::KDTreeSingleIndexAdaptor<
	nanoflann::L2_Simple_Adaptor<float, PointCloud<float, glm::fvec3> >,
	PointCloud<float, glm::fvec3>,
	3 // dim 
> kd_tree_t;

namespace mystudents {
	using namespace std;

	// BlobDetector를 통해 원이 감지되었는지 확인하는 코드입니다.
	bool CheckCircleDetect(const cv::Mat& inputImg, const std::vector<cv::KeyPoint>& keyPoints)
	{
		cv::Mat img = inputImg.clone();

		for (const auto& keypoint : keyPoints) {
			int x = cvRound(keypoint.pt.x);
			int y = cvRound(keypoint.pt.y);
			int s = cvRound(keypoint.size);
			int r = cvRound(s / 2);
			cv::circle(img, cv::Point(x, y), r, cv::Scalar(0, 0, 255), 2);
			string text = " (" + to_string(x) + ", " + to_string(y) + ")";
			cv::putText(img, text, cv::Point(x - 25, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
		}
		//cv::imwrite(__gc->g_folder_trackingInfo + "test_circle_detect.png", img); // cv imshow()로 보이기.

		return true; // 반지름 반환
	}

	// 2d 포지션의 index가 잘 정렬되었는지 확인하는 코드입니다.
	void CheckPositionSort(const cv::Mat& inputImg, const std::vector<cv::Point2f>& point2Ds, const std::vector<float>& radiis)
	{
		cv::Mat img;
		cv::cvtColor(inputImg, img, cv::COLOR_GRAY2RGB);

		for (int i = 0; i < (int)point2Ds.size(); i++) {
			cv::Point2f center = point2Ds[i];
		//for (const auto& center : point2Ds) {
			int x = cvRound(center.x);
			int y = cvRound(center.y);
			int r = cvRound(radiis[i]);

			cv::circle(img, cv::Point(x, y), r, cv::Scalar(0, 0, 256), 2);
			string text = " (" + to_string(x) + ", " + to_string(y) + ")";
			cv::putText(img, text, cv::Point(x - 25, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);

			// index 출력
			cv::putText(img, to_string(i), cv::Point(x - 20, y - 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 1);
		}

		cv::flip(img, img, 1);
		cv::imwrite(__gc->g_folder_trackingInfo + "test_circle_sort.png", img); // cv imshow()로 보이기.
	}

	// 원의 점을 직선의 거리에 대하여 비교하는 함수.
	bool sortByDistance(const cv::Point2f& a, const cv::Point2f& b, const int m, const int n) {
		float distanceA = abs(m * a.x + n * a.y + n);//std::sqrt((a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y));
		float distanceB = abs(m * b.x + n * b.y + n);//std::sqrt((b.x - center.x) * (b.x - center.x) + (b.y - center.y) * (b.y - center.y));
		return distanceA < distanceB;
	}

	// implemented by 김민교 
	int Get2DPostionsFromLegoPhantom(const cv::Mat& inputImg, std::vector<cv::Point2f>& points2Ds)
	{
		cv::Mat imgGray;
		int chs = inputImg.channels();
		if (inputImg.channels() == 3)
			cv::cvtColor(inputImg, imgGray, cv::COLOR_BGR2GRAY);
		else if (inputImg.channels() == 1)
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

		// 벡터에 일단 원의 점을 넣는다.
		vector<cv::Point3f> circlePoints;
		vector<float> circleRadiis;

		for (const auto& kp : keypoints) {
			circlePoints.push_back(cv::Point3f(kp.pt.x, kp.pt.y, kp.size));
		}

		int col = 6; int row = 8;
		int top_bottom = 6; // 위 아래 6개
		int middle = 8;     // 가운데 8개

		for (int i = 0; i < row; i++) {
			int pointNum = middle;
			if (i == 0 || i == row - 1) // 맨 처음줄과 아랫줄이면 6개만 감지한다.
				pointNum = top_bottom;

			// 1. y축 정렬
			sort(circlePoints.begin(), circlePoints.end(), [](const cv::Point3f& a, const cv::Point3f& b) {
				return a.y < b.y;
				});

			// 2. 가장 위의 점 두개 뽑기
			cv::Point3f first = circlePoints[0];
			cv::Point3f second = circlePoints[1];

			// 3. 두점을 잇는 선
			float m = (second.y - first.y) / (second.x - first.x); // 기울기.
			float n = first.y - (m * first.x); // y절편.

			// 4. 직선의 방정식 : y = mx + n, y-mx-n =0;
			// 직선과 가까운 점 정렬.
			std::sort(circlePoints.begin(), circlePoints.end(),
				[&](const cv::Point3f& a, const cv::Point3f& b) {
					return mystudents::sortByDistance(cv::Point2f(a.x, a.y), cv::Point2f(b.x, b.y), m, n); // sortByDistance에서 직선과 점의 거리 계산 후 정렬.
				});

			// 6개 or 8개 빼기			
			vector<cv::Point3f> rowPoints; // 뺀 점이 rowPoints에 들어갑니다.
			for (int j = 0; j < pointNum; j++)
			{
				rowPoints.push_back(circlePoints[0]); // 거리가 가장 가까운 것을 뽑음. 가장 가까운건 0번째
				circlePoints.erase(circlePoints.begin()); // 뽑았으면 삭제.
			}

			// 5. x 방향으로 정렬.
			sort(rowPoints.begin(), rowPoints.end(), [](const cv::Point3f& a, const cv::Point3f& b) {
				return a.x < b.x;
				});

			// 6. point2Ds에 넣음.
			for (int j = 0; j < pointNum; j++) {
				Point3f p = rowPoints[j];
				points2Ds.push_back(Point2f(p.x, p.y));
				circleRadiis.push_back(p.z * 0.5f);
			}
		}

		// 포지션 잘 정렬되었는지 확인.
		//mystudents::CheckPositionSort(inputImg, points2Ds, circleRadiis);

		return (int)points2Ds.size();
	}

	int Get2DPostionsFromFMarkersPhantom(const cv::Mat& inputImg, const int rows, const int cols, std::vector<cv::Point2f>& points2Ds
		, const float minArea, const float maxArea, const float minInterval, std::vector<float>* circleRadius)
	{
		points2Ds.clear();
		if (circleRadius) circleRadius->clear();

		cv::Mat imgGray;
		int chs = inputImg.channels();
		if (inputImg.channels() == 3)
			cv::cvtColor(inputImg, imgGray, cv::COLOR_BGR2GRAY);
		else if (inputImg.channels() == 1)
			imgGray = inputImg.clone();
		else {
			cout << "not supported image!" << endl;
			assert(0);
		}

		cv::bitwise_not(imgGray, imgGray);
		cv::GaussianBlur(imgGray, imgGray, cv::Size(5, 5), 0);
		//Add mask
		//cv::Mat imgSrc = imgGray.clone(); // gray image(=imgSrc)로 circle detect 할 것.
		// Blob Detector Params
		cv::SimpleBlobDetector::Params params;
		params.filterByArea = true;
		params.filterByCircularity = true;
		params.filterByConvexity = false;
		params.filterByInertia = true;
		params.filterByColor = false;
		params.blobColor = 0;

		//params.minArea = 500; // The size of the blob filter to be applied.If the corresponding value is increased, small circles are not detected.
		//params.maxArea = 1000;
		//params.minCircularity = 0.01; // 1 >> it detects perfect circle.Minimum size of center angle

		params.minArea = minArea; // The size of the blob filter to be applied.If the corresponding value is increased, small circles are not detected.
		params.maxArea = maxArea;
		params.minDistBetweenBlobs = minInterval;

		if (rows * cols < 40) {
			// extrinsics
			params.minCircularity = __gc->g_extParam_minCircularity; // 1 >> it detects perfect circle.Minimum size of center angle
			params.minInertiaRatio = __gc->g_extParam_minInertiaRatio; // 1 >> it detects perfect circle. short / long axis
			params.minRepeatability = __gc->g_extParam_minRepeatability;
		}
		else {
			// intrinsics
			params.minCircularity = __gc->g_intParam_minCircularity; // 1 >> it detects perfect circle.Minimum size of center angle
			params.minInertiaRatio = __gc->g_intParam_minInertiaRatio; // 1 >> it detects perfect circle. short / long axis
			params.minRepeatability = __gc->g_intParam_minRepeatability;
		}

		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		vector<cv::KeyPoint> keypoints;
		detector->detect(imgGray, keypoints); // circle detect.
		// 원을 감지했는지 확인.
		int r = mystudents::CheckCircleDetect(inputImg, keypoints);
		if (keypoints.size() < rows * cols)
		{
			cout << "there is a missing circle!!" << endl;
			return false;
		}

		// 벡터에 일단 원의 점을 넣는다.
		vector<cv::Point3f> circlePoints;
		vector<float> circleRadiis;

		for (const auto& kp : keypoints) {
			circlePoints.push_back(cv::Point3f(kp.pt.x, kp.pt.y, kp.size));
		}

		int col = cols; int row = rows;
		int top_bottom = cols; // 위 아래 6개
		int middle = cols;     // 가운데 8개

		for (int i = 0; i < row; i++) {
			int pointNum = middle;
			if (i == 0 || i == row - 1) // 맨 처음줄과 아랫줄이면 6개만 감지한다.
				pointNum = top_bottom;

			// 1. y축 정렬
			sort(circlePoints.begin(), circlePoints.end(), [](const cv::Point3f& a, const cv::Point3f& b) {
				return a.y < b.y;
				});

			// 2. 가장 위의 점 두개 뽑기
			glm::fvec2 p0 = glm::fvec2(circlePoints[0].x, circlePoints[0].y);
			glm::fvec2 p1 = glm::fvec2(circlePoints[1].x, circlePoints[1].y);

			glm::fvec2 v = glm::normalize(p0 - p1); // note p0 != p1

			// 4. 직선의 방정식 : y = mx + n, y-mx-n =0;
			// 직선과 가까운 점 정렬.
			std::sort(circlePoints.begin(), circlePoints.end(),
				[&](const cv::Point3f& a, const cv::Point3f& b) {

					auto computeDist = [&v, &p0](const glm::fvec2 q) {
						float t = glm::dot(q - p0, v);
						glm::fvec2 p = v * t + p0;
						return glm::length2(p - q);
					};

					return computeDist(glm::fvec2(a.x, a.y)) < computeDist(glm::fvec2(b.x, b.y));
					//return mystudents::sortByDistance(cv::Point2f(a.x, a.y), cv::Point2f(b.x, b.y), m, n); // sortByDistance에서 직선과 점의 거리 계산 후 정렬.
				});

			// 6개 or 8개 빼기			
			vector<cv::Point3f> rowPoints; // 뺀 점이 rowPoints에 들어갑니다.
			for (int j = 0; j < pointNum; j++)
			{
				rowPoints.push_back(circlePoints[0]); // 거리가 가장 가까운 것을 뽑음. 가장 가까운건 0번째
				circlePoints.erase(circlePoints.begin()); // 뽑았으면 삭제.
			}

			// 5. x 방향으로 정렬.
			sort(rowPoints.begin(), rowPoints.end(), [](const cv::Point3f& a, const cv::Point3f& b) {
				return a.x < b.x;
				});

			// 6. point2Ds에 넣음.
			for (int j = 0; j < pointNum; j++) {
				cv::Point3f p = rowPoints[j];
				points2Ds.push_back(cv::Point2f(p.x, p.y));
				circleRadiis.push_back(p.z * 0.5f);
			}
		}

		if (circleRadius) *circleRadius = circleRadiis;
		// 포지션 잘 정렬되었는지 확인.
		//mystudents::CheckPositionSort(inputImg, points2Ds, circleRadiis);

		return (int)points2Ds.size();
	}

	int Get2DPostionsFromFMarkersPhantom2(const cv::Mat& inputImg, const int rows, const int cols, std::vector<cv::Point2f>& points2Ds)
	{
		cv::Mat imgGray;
		int chs = inputImg.channels();
		if (inputImg.channels() == 3)
			cv::cvtColor(inputImg, imgGray, cv::COLOR_BGR2GRAY);
		else if (inputImg.channels() == 1)
			imgGray = inputImg.clone();
		else {
			cout << "not supported image!" << endl;
			assert(0);
		}

		cv::bitwise_not(imgGray, imgGray);
		cv::GaussianBlur(imgGray, imgGray, cv::Size(15, 15), 0);


		cv::threshold(imgGray, imgGray, 210, 255, cv::THRESH_TRUNC);

		std::vector<cv::Point3f> circlePoints;
		cv::HoughCircles(imgGray, circlePoints, cv::HOUGH_GRADIENT, 1,
			300, // change this value to detect circles with different distances to each other
			110, 20, 40, 70 // change the last two parameters
			// (min_radius & max_radius) to detect larger circles
		);

		// 벡터에 일단 원의 점을 넣는다.
		vector<float> circleRadiis;

		int col = cols; int row = rows;
		int top_bottom = cols; // 위 아래 6개
		int middle = cols;     // 가운데 8개

		for (int i = 0; i < row; i++) {
			int pointNum = middle;
			if (i == 0 || i == row - 1) // 맨 처음줄과 아랫줄이면 6개만 감지한다.
				pointNum = top_bottom;

			// 1. y축 정렬
			sort(circlePoints.begin(), circlePoints.end(), [](const cv::Point3f& a, const cv::Point3f& b) {
				return a.y < b.y;
				});

			// 2. 가장 위의 점 두개 뽑기
			cv::Point3f first = circlePoints[0];
			cv::Point3f second = circlePoints[1];

			// 3. 두점을 잇는 선
			float m = (second.y - first.y) / (second.x - first.x); // 기울기.
			float n = first.y - (m * first.x); // y절편.

			// 4. 직선의 방정식 : y = mx + n, y-mx-n =0;
			// 직선과 가까운 점 정렬.
			std::sort(circlePoints.begin(), circlePoints.end(),
				[&](const cv::Point3f& a, const cv::Point3f& b) {
					return mystudents::sortByDistance(cv::Point2f(a.x, a.y), cv::Point2f(b.x, b.y), m, n); // sortByDistance에서 직선과 점의 거리 계산 후 정렬.
				});

			// 6개 or 8개 빼기			
			vector<cv::Point3f> rowPoints; // 뺀 점이 rowPoints에 들어갑니다.
			for (int j = 0; j < pointNum; j++)
			{
				rowPoints.push_back(circlePoints[0]); // 거리가 가장 가까운 것을 뽑음. 가장 가까운건 0번째
				circlePoints.erase(circlePoints.begin()); // 뽑았으면 삭제.
			}

			// 5. x 방향으로 정렬.
			sort(rowPoints.begin(), rowPoints.end(), [](const cv::Point3f& a, const cv::Point3f& b) {
				return a.x < b.x;
				});

			// 6. point2Ds에 넣음.
			for (int j = 0; j < pointNum; j++) {
				cv::Point3f p = rowPoints[j];
				points2Ds.push_back(cv::Point2f(p.x, p.y));
				circleRadiis.push_back(p.z);
			}
		}

		// 포지션 잘 정렬되었는지 확인.
		mystudents::CheckPositionSort(inputImg, points2Ds, circleRadiis);

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
		fvec3 estimated_normal = navihelpers::ComputeNormalVector(pts); // return the unit length vector of the plane fitting to the input points
		// normal correction
		fvec3 v14 = pts[3] - pts[0];
		fvec3 v12 = pts[1] - pts[0];
		fvec3 nor_dir = cross(v14, v12);
		if (dot(estimated_normal, nor_dir) < 0) estimated_normal *= -1.f;

		//estimated_normal = normalize(nor_dir);

		// compute the aligned marker position fit to the sphere height
		const double h_r = 0.0033;// (float)(15.9375 * 0.001);
		const double s_r = 0.004;
		double m_r_sq = marker_r * marker_r;
		double h_r_sq = h_r * h_r;
		double s_r_sq = s_r * s_r;
		double a = sqrt(m_r_sq - h_r_sq);
		double b = sqrt(s_r_sq - h_r_sq);
		double correctLength = a - b;// -0.003;
		std::cout << "\ncorrectLength " << correctLength << std::endl;
		fvec3 pts_corrected[4];
		for (int i = 0; i < 4; i++) {
			pts_corrected[i] = pts[i] - estimated_normal * (float)correctLength;
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

		cv::Mat ocvVec3(1, 3, CV_32FC1);
		cv::FileStorage fs(__gc->g_folder_trackingInfo + "test_tip_sphere.txt", cv::FileStorage::Mode::WRITE);
		for (int i = 0; i < 60; i++) {
			memcpy(ocvVec3.ptr(), &points3Ds[i], sizeof(float) * 3);
			fs << "inter_sphere_" + std::to_string(i) << ocvVec3;
		}
		fs.release();
		//vector<vector<cv::Point3f>> points3Ds;
		//points3Ds.push_back(inter_points);
		return (int)points3Ds.size();
	}

	int Get3DPostionsFromFMarkersPhantom(const glm::fvec3& marker0, const glm::fvec3& marker1, const glm::fvec3& marker2, const glm::fvec3& marker3, 
		const track_info* trk, const int rows, const int cols,
		std::vector<cv::Point3f>& points3Ds) {

		points3Ds.clear();

		// note we are using meter metric
		using namespace std;
		using namespace glm;

		glm::fmat4x4 matRB2WS;
		track_info& trackInfo = *(track_info*)trk;

		trackInfo.GetRigidBodyByName("c-arm", &matRB2WS, NULL, NULL, NULL);
		fvec3 carmCenter = vzmutils::transformPos(fvec3(0, 0, 0), matRB2WS);
		fvec3 dirToCarmRB = normalize(carmCenter - marker0);

		int numMarkers = trackInfo.NumMarkers();
		vector<fvec3> mkPts;
		for (int i = 0; i < numMarkers; i++) {
			std::map<track_info::MKINFO, std::any> mk;
			trackInfo.GetMarkerByIdx(i, mk);
			mkPts.push_back(std::any_cast<fvec3>(mk[track_info::MKINFO::POSITION]));
		}

		PointCloud<float, fvec3> pc(&mkPts[0], numMarkers);
		kd_tree_t kdt(3, pc, nanoflann::KDTreeSingleIndexAdaptorParams(10));
		kdt.buildIndex();

		size_t out_ids[3];
		float out_dists[3];
		//fvec3 posMkCenter = (marker0 + marker1 + marker2 + marker3) / 4.f;
		//kdt.knnSearch((float*)&marker0, 16, out_ids, out_dists);

		vector<fvec3> mkPhantomPts;

		for (int i = 0; i < rows; ++i) {
			float ratio_row = (float)i / (float)(rows - 1);
			fvec3 interPosRow0 = (1.f - ratio_row) * marker0 + ratio_row * marker3;
			fvec3 interPosRow1 = (1.f - ratio_row) * marker1 + ratio_row * marker2;

			for (int j = 0; j < cols; ++j) {

				float ratio_col = (float)j / (float)(rows - 1);
				fvec3 interPos = (1.f - ratio_col) * interPosRow0 + ratio_col * interPosRow1;
				mkPhantomPts.push_back(interPos);
			}
		}

		for (int i = 0; i < (int)mkPhantomPts.size(); i++) {
			kdt.knnSearch((float*)&mkPhantomPts[i], 1, out_ids, out_dists);
			points3Ds.push_back(__gm3__ & mkPts[out_ids[0]]);
		}


		//cv::Mat ocvVec3(1, 3, CV_32FC1);
		//cv::FileStorage fs(__gc->g_folder_trackingInfo + "test_tip_sphere_mk.txt", cv::FileStorage::Mode::WRITE);
		//for (int i = 0; i < (int)points3Ds.size(); i++) {
		//	memcpy(ocvVec3.ptr(), &points3Ds[i], sizeof(float) * 3);
		//	fs << "inter_sphere_" + std::to_string(i) << ocvVec3;
		//}
		//fs.release();
		//vector<vector<cv::Point3f>> points3Ds;
		//points3Ds.push_back(inter_points);
		return (int)points3Ds.size();
	}
}

namespace calibtask {

	void InitializeTask(__GC* gcp) {
		__gc = gcp;

		cv::FileStorage fs(__gc->g_folder_trackingInfo + "rb2carm1.txt", cv::FileStorage::Mode::READ);
		
		if (fs.isOpened()) {
			cv::Mat rvec, tvec;
			fs["rvec"] >> rvec;
			fs["tvec"] >> tvec;
			fs.release();

			cv::Mat matR;
			cv::Rodrigues(rvec, matR);
			// note, here camera frame (notation 'CA', opencv convention) is defined with
			// z axis as viewing direction
			// -y axis as up vector
			__gc->g_CArmRB2SourceCS = glm::fmat4x4(1);
			__gc->g_CArmRB2SourceCS[0][0] = (float)matR.at<double>(0, 0);
			__gc->g_CArmRB2SourceCS[0][1] = (float)matR.at<double>(1, 0);
			__gc->g_CArmRB2SourceCS[0][2] = (float)matR.at<double>(2, 0);
			__gc->g_CArmRB2SourceCS[1][0] = (float)matR.at<double>(0, 1);
			__gc->g_CArmRB2SourceCS[1][1] = (float)matR.at<double>(1, 1);
			__gc->g_CArmRB2SourceCS[1][2] = (float)matR.at<double>(2, 1);
			__gc->g_CArmRB2SourceCS[2][0] = (float)matR.at<double>(0, 2);
			__gc->g_CArmRB2SourceCS[2][1] = (float)matR.at<double>(1, 2);
			__gc->g_CArmRB2SourceCS[2][2] = (float)matR.at<double>(2, 2);
			__gc->g_CArmRB2SourceCS[3][0] = (float)((double*)tvec.data)[0];
			__gc->g_CArmRB2SourceCS[3][1] = (float)((double*)tvec.data)[1];
			__gc->g_CArmRB2SourceCS[3][2] = (float)((double*)tvec.data)[2];
		}
	}
	int RegisterCArmImage(const int sidScene, const std::string& carmScanParams, const std::string& scanName, cv::Mat* pimgCArm)
	{
		cv::FileStorage fs(carmScanParams, cv::FileStorage::Mode::READ);
		if (!fs.isOpened())
			return -1;

		cv::Mat K;
		fs["K"] >> K;
		cv::Mat DistCoeffs;
		fs["DistCoeffs"] >> DistCoeffs;
		cv::Mat rvec, tvec;
		fs["rvec"] >> rvec;
		fs["tvec"] >> tvec;
		cv::Mat rb2wsMat;
		fs["rb2wsMat"] >> rb2wsMat; // memory storage ordering in opengl
		std::string imgFileName;
		fs["imgFile"] >> imgFileName;
		fs.release();

		int aidGroupCArmCam = 0;
		vzm::ActorParameters apGroupCams;
		vzm::NewActor(apGroupCams, "Tracking CAM Group:scanName", aidGroupCArmCam);
		vzm::AppendSceneItemToSceneTree(aidGroupCArmCam, sidScene);


		///////////////////////////////////
		// preprocessing information
		// *** IMPORTANT NOTE:
		// the C-arm image (Genoray) is aligned w.r.t. detector's view
		// the calibration image must be aligned w.r.t. source's view (view frustum's origin point)
		// therefore, the position mirrored horizontally

		//if (pimgCArm)
		cv::Mat imgCArm = cv::imread(imgFileName);

		double arrayMatK[9] = {};
		double arrayDistCoeffs[5] = {};
		memcpy(arrayMatK, K.ptr(), sizeof(double) * 9);
		memcpy(arrayDistCoeffs, DistCoeffs.ptr(), sizeof(double) * 5);

		cv::Mat matR;
		cv::Rodrigues(rvec, matR);
		// note, here camera frame (notation 'CA', opencv convention) is defined with
		// z axis as viewing direction
		// -y axis as up vector
		glm::fmat4x4 matRB2CA = glm::fmat4x4(1);
		matRB2CA[0][0] = (float)matR.at<double>(0, 0);
		matRB2CA[0][1] = (float)matR.at<double>(1, 0);
		matRB2CA[0][2] = (float)matR.at<double>(2, 0);
		matRB2CA[1][0] = (float)matR.at<double>(0, 1);
		matRB2CA[1][1] = (float)matR.at<double>(1, 1);
		matRB2CA[1][2] = (float)matR.at<double>(2, 1);
		matRB2CA[2][0] = (float)matR.at<double>(0, 2);
		matRB2CA[2][1] = (float)matR.at<double>(1, 2);
		matRB2CA[2][2] = (float)matR.at<double>(2, 2);
		matRB2CA[3][0] = (float)((double*)tvec.data)[0];
		matRB2CA[3][1] = (float)((double*)tvec.data)[1];
		matRB2CA[3][2] = (float)((double*)tvec.data)[2];

		glm::fmat4x4 matCA2RB = glm::inverse(matRB2CA);
		glm::fmat4x4 matRB2WS;
		memcpy(glm::value_ptr(matRB2WS), rb2wsMat.ptr(), sizeof(float) * 16);

		glm::fmat4x4 matCA2WS = matRB2WS * matCA2RB;
		int oidCamTris = 0, oidCamLines = 0, oidCamLabel = 0;
		navihelpers::CamOcv_Gen(matCA2WS, "C-Arm Source:" + scanName, oidCamTris, oidCamLines, oidCamLabel);
		vzm::ActorParameters apCamTris, apCamLines, apCamLabel;
		apCamTris.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamTris);
		apCamTris.color[3] = 0.5f;
		apCamLines.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLines);
		*(glm::fvec4*)apCamLines.phong_coeffs = glm::fvec4(0, 1, 0, 0);
		apCamLines.line_thickness = 2;
		apCamLabel.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCamLabel);
		*(glm::fvec4*)apCamLabel.phong_coeffs = glm::fvec4(0, 1, 0, 0);
		int aidCamTris = 0, aidCamLines = 0, aidCamLabel = 0;
		vzm::NewActor(apCamTris, "C-Arm Cam Tris:" + scanName, aidCamTris);
		vzm::NewActor(apCamLines, "C-Arm Lines:" + scanName, aidCamLines);
		vzm::NewActor(apCamLabel, "C-Arm Label:" + scanName, aidCamLabel);
		vzm::AppendSceneItemToSceneTree(aidCamTris, aidGroupCArmCam);
		vzm::AppendSceneItemToSceneTree(aidCamLines, aidGroupCArmCam);
		vzm::AppendSceneItemToSceneTree(aidCamLabel, aidGroupCArmCam);

		glm::fmat4x4 matCS2PS;
		navihelpers::ComputeMatCS2PS(arrayMatK[0], arrayMatK[4], arrayMatK[1], arrayMatK[2], arrayMatK[5],
			(float)imgCArm.cols, (float)imgCArm.rows, 0.1f, 1.2f, matCS2PS);
		// here, CS is defined with
		// -z axis as viewing direction
		// y axis as up vector
		glm::fmat4x4 matPS2CS = glm::inverse(matCS2PS);
		// so, we need to convert CS to OpenCV's Camera Frame by rotating 180 deg w.r.t. x axis
		glm::fmat4x4 matCS2CA = glm::rotate(glm::pi<float>(), glm::fvec3(1, 0, 0));
		glm::fmat4x4 matPS2WS = matCA2WS * matCS2CA * matPS2CS;


		// mapping the carm image to far plane
		glm::fvec3 ptsCArmPlanes[4] = { glm::fvec3(-1, 1, 1), glm::fvec3(1, 1, 1), glm::fvec3(1, -1, 1), glm::fvec3(-1, -1, 1) };
		for (int i = 0; i < 4; i++) {
			ptsCArmPlanes[i] = vzmutils::transformPos(ptsCArmPlanes[i], matPS2WS);
		}
		//glm::fvec2 ptsTexCoords[4] = { glm::fvec2(0, 0), glm::fvec2(1, 0), glm::fvec2(1, 1), glm::fvec2(0, 1) };
		glm::fvec3 ptsTexCoords[4] = { glm::fvec3(0, 0, 0), glm::fvec3(1, 0, 0), glm::fvec3(1, 1, 0), glm::fvec3(0, 1, 0) };
		//std::vector<glm::fvec3> posBuffer(6);
		//std::vector<glm::fvec2> texBuffer(6);
		unsigned int idxList[6] = { 0, 1, 3, 1, 2, 3 };

		int oidImage = 0;
		vzm::LoadImageFile(imgFileName, oidImage, true, true);
		int oidCArmPlane = 0;
		vzm::GeneratePrimitiveObject(__FP ptsCArmPlanes[0], NULL, NULL, __FP ptsTexCoords[0], 4, idxList, 2, 3, oidCArmPlane);
		vzm::ActorParameters apCArmPlane;
		apCArmPlane.SetResourceID(vzm::ActorParameters::GEOMETRY, oidCArmPlane);
		apCArmPlane.SetResourceID(vzm::ActorParameters::TEXTURE_2D, oidImage);
		apCArmPlane.script_params.SetParam("matCA2WS", matCA2WS); // temporal storage :)
		apCArmPlane.script_params.SetParam("imageWH", glm::ivec2(imgCArm.cols, imgCArm.rows)); // temporal storage :)
		*(glm::fvec4*)apCArmPlane.phong_coeffs = glm::fvec4(1, 0, 0, 0);
		int aidCArmPlane = 0;
		vzm::NewActor(apCArmPlane, "CArm Plane:" + scanName, aidCArmPlane);
		vzm::AppendSceneItemToSceneTree(aidCArmPlane, aidGroupCArmCam);

		return aidGroupCArmCam;
	}

	// note : this function needs to be called in the render thread
	// using "carm_intrinsics.txt", store opencv's rvec and tvec to "rb2carm1.txt"
	bool CalibrationWithPhantom(glm::fmat4x4& matCArmRB2SourceCS, const cv::Mat& downloadedGrayImg, const track_info* trk, const bool useGlobal)
	{
		assert(downloadedGrayImg.channels() == 1);

		using namespace std;

		cv::Mat source2detImg;
		if (FLIP_SCANIMAGE)
			cv::flip(downloadedGrayImg, source2detImg, 1);
		else
			source2detImg = downloadedGrayImg;

		cv::FileStorage fs(__gc->g_folder_trackingInfo + "calibBoard.txt", cv::FileStorage::Mode::READ);
		std::string calib_mode;
		std::string det_mode;
		int cols, rows;
		fs["ROWS"] >> rows;
		fs["COLS"] >> cols;
		fs["MODE"] >> calib_mode;
		fs["DETECTOR"] >> det_mode;
		fs.release();

		if (calib_mode == "LEGO" || calib_mode == "FMARKERS") {
			if (__gc->g_selectedMkNames.size() != 4) {
				__gc->SetErrorCode("4 Points are Needed!");
				cout << "\n4 points are needed!!" << endl;
				return false;
			}
		}

		// 1st, get c-arm rb info.
		glm::fmat4x4 matRB2WS, matWS2RB;

		glm::fquat q;
		glm::fvec3 t;
		track_info& trackInfo = *(track_info*)trk;
		if (!trackInfo.GetRigidBodyQuatTVecByName("c-arm", &q, &t)) {
			__gc->SetErrorCode("Failure to Get C-Arm Rigidbody!");
			cout << "\nfailure to get c-arm rigid body" << endl;
			return false;
		}

		glm::fmat4x4 mat_r = glm::toMat4(q);
		glm::fmat4x4 mat_t = glm::translate(t);
		matRB2WS = mat_t * mat_r;
		matWS2RB = glm::inverse(matRB2WS);

		// get 2d mks info from x-ray img (calculImg)
		std::vector<cv::Point2f> points2d;
		if (calib_mode == "LEGO") {
			mystudents::Get2DPostionsFromLegoPhantom(source2detImg, points2d);
			if (points2d.size() != 60) {
				__gc->SetErrorCode("60 Circles must be Detected! " + std::to_string(points2d.size()));
				cout << "\n# of circles must be 60 if PHANTOM_MODE is LEGO" << endl;
				return false;
			}
		}
		else if (calib_mode == "FILM") {
			if (points2d.size() != 77) {
				__gc->SetErrorCode("77 Circles must be Detected! " + std::to_string(points2d.size()));
				cout << "\n# of circles must be 77 if PHANTOM_MODE is FILM" << endl;
				return false;
			}
		}
		else if (calib_mode == "FMARKERS") {
			if (det_mode == "SimpleBlobDetector")
				mystudents::Get2DPostionsFromFMarkersPhantom(source2detImg, rows, cols, points2d);
			else if (det_mode == "HoughCircles")
				mystudents::Get2DPostionsFromFMarkersPhantom2(source2detImg, rows, cols, points2d);
			else {
				cout << "\nInvalid Detector!" << endl;
				return false;
			}
			
			if (points2d.size() != rows * cols) {
				__gc->SetErrorCode(std::to_string(rows * cols) + " Circles must be Detected! " + std::to_string(points2d.size()));
				cout << "\n# of circles must be " << rows * cols << " if PHANTOM_MODE is FMARKERS" << endl;
				return false;
			}
		}
		else assert(0);

		std::vector<cv::Point3f> posWsMarkers(__gc->g_selectedMkNames.size());
		for (auto it = __gc->g_selectedMkNames.begin(); it != __gc->g_selectedMkNames.end(); it++) {
			std::map<track_info::MKINFO, std::any> mkInfo;
			trackInfo.GetMarkerByName(it->first, mkInfo);
			glm::fvec3 pos = std::any_cast<glm::fvec3>(mkInfo[track_info::MKINFO::POSITION]);
			posWsMarkers[it->second] = *(cv::Point3f*)&pos;
		}

		if (calib_mode == "LEGO" || calib_mode == "FMARKERS") {
			// check the selecting direction
			glm::fvec3 v01 = *(glm::fvec3*)&posWsMarkers[1] - *(glm::fvec3*)&posWsMarkers[0];
			glm::fvec3 v03 = *(glm::fvec3*)&posWsMarkers[3] - *(glm::fvec3*)&posWsMarkers[0];
			glm::fvec3 selDir = glm::cross(v01, v03);
			glm::fvec3 posCenterCArmRB = vzmutils::transformPos(glm::fvec3(0, 0, 0), matRB2WS);
			glm::fvec3 cal2DetDir = posCenterCArmRB - *(glm::fvec3*)&posWsMarkers[0];
			//if (glm::dot(selDir, cal2DetDir) < 0)
			{
				//std::cout << "\ncorrecting the selecting direction!" << std::endl;
				// 이상함... 아래것으로 해야 하는데... 이렇게 하면 카메라 위치가 반대로 감...
				// 일단은 데모를 위해 주석 처리...
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
		if (calib_mode == "LEGO") {
			mystudents::Get3DPostionsFromLegoPhantom2(0.007f, posWsMarkers[0], posWsMarkers[1], posWsMarkers[2], posWsMarkers[3], points3d);
		}
		else if (calib_mode == "FMARKERS") {
			mystudents::Get3DPostionsFromFMarkersPhantom(__cv3__&posWsMarkers[0], __cv3__&posWsMarkers[1], __cv3__&posWsMarkers[2], __cv3__&posWsMarkers[3], 
				trk, rows, cols, points3d);
		}

		//__gc->g_testMKs.clear();
		__gc->g_testMKs.assign(points3d.size(), glm::fvec3(0));
		memcpy(&__gc->g_testMKs[0], &points3d[0], sizeof(glm::fvec3) * points3d.size());

		//__gc->g_testMKs.push_back(*(glm::fvec3*)&posWsMarkers[0]);
		//__gc->g_testMKs.push_back(*(glm::fvec3*)&posWsMarkers[1]);
		//__gc->g_testMKs.push_back(*(glm::fvec3*)&posWsMarkers[2]);
		//__gc->g_testMKs.push_back(*(glm::fvec3*)&posWsMarkers[3]);

		// important! the 3d points must be defined in C-Arm RB space
		// because we want to get the transform btw C-arm cam (x-ray source) space and C-Arm RB space
		for (int i = 0; i < points3d.size(); i++) {
			*(glm::fvec3*)&points3d[i] = vzmutils::transformPos(*(glm::fvec3*)&points3d[i], matWS2RB);
		}

		cv::Mat cameraMatrix, distCoeffs;
		{
			cv::FileStorage fs(__gc->g_folder_trackingInfo + "carm_intrinsics.txt", cv::FileStorage::Mode::READ);
			fs["K"] >> cameraMatrix;
			fs["DistCoeffs"] >> distCoeffs;
			fs.release();
		}

		for (int i = 0; i < points2d.size(); i++) {

			__gc->g_homographyPairs[__cv3__ & points3d[i]] = __cv2__ & points2d[i];
		}


		cv::Mat rvec, tvec;
		if (useGlobal) {
			std::vector<cv::Point2f> global_points2d;
			std::vector<cv::Point3f> global_points3d;
			for (auto& _pair : __gc->g_homographyPairs) {
				global_points2d.push_back(__gm2__ & (_pair.second));
				global_points3d.push_back(__gm3__ & (_pair.first));
			}

			cv::solvePnP(global_points3d, global_points2d, cameraMatrix, distCoeffs, rvec, tvec);
			cout << "\n# pairs : " << __gc->g_homographyPairs.size() << endl;
		}
		else {
			cv::solvePnP(points3d, points2d, cameraMatrix, distCoeffs, rvec, tvec);
			cout << "\n# pairs : " << points2d.size() << endl;
		}

		{
			//matCArmRB2SourceCS
			cv::Mat matR;
			cv::Rodrigues(rvec, matR);
			// note, here camera frame (notation 'CA', opencv convention) is defined with
			// z axis as viewing direction
			// -y axis as up vector
			matCArmRB2SourceCS = glm::fmat4x4(1);
			matCArmRB2SourceCS[0][0] = (float)matR.at<double>(0, 0);
			matCArmRB2SourceCS[0][1] = (float)matR.at<double>(1, 0);
			matCArmRB2SourceCS[0][2] = (float)matR.at<double>(2, 0);
			matCArmRB2SourceCS[1][0] = (float)matR.at<double>(0, 1);
			matCArmRB2SourceCS[1][1] = (float)matR.at<double>(1, 1);
			matCArmRB2SourceCS[1][2] = (float)matR.at<double>(2, 1);
			matCArmRB2SourceCS[2][0] = (float)matR.at<double>(0, 2);
			matCArmRB2SourceCS[2][1] = (float)matR.at<double>(1, 2);
			matCArmRB2SourceCS[2][2] = (float)matR.at<double>(2, 2);
			matCArmRB2SourceCS[3][0] = (float)((double*)tvec.data)[0];
			matCArmRB2SourceCS[3][1] = (float)((double*)tvec.data)[1];
			matCArmRB2SourceCS[3][2] = (float)((double*)tvec.data)[2];
		}

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
			cv::FileStorage fs(__gc->g_folder_trackingInfo + "rb2carm1.txt", cv::FileStorage::Mode::WRITE);
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



		return true;
	}

}

