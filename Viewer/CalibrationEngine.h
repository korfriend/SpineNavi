#pragma once

#include<QWidget>


#include <opencv2/opencv.hpp>

#include "../SNaviWin/naviHelpers.hpp"
#include "nanoflann.hpp"
#include "Defines.h"

class Q_DECL_EXPORT CalibrationEngine : public QWidget
{

	Q_OBJECT


public:

	CalibrationEngine();
	~CalibrationEngine();

	//mystudent
	bool CheckCircleDetect(const cv::Mat& inputImg, const std::vector<cv::KeyPoint>& keyPoints);
	void CheckPositionSort(const cv::Mat& inputImg, const std::vector<cv::Point2f>& point2Ds, const std::vector<float>& radiis);
	bool sortByDistance(const cv::Point2f& a, const cv::Point2f& b, const int m, const int n);
	int Get2DPostionsFromLegoPhantom(const cv::Mat& inputImg, std::vector<cv::Point2f>& points2Ds);
	int Get2DPostionsFromFMarkersPhantom(const cv::Mat& inputImg, const int rows, const int cols, std::vector<cv::Point2f>& points2Ds);
	int Get2DPostionsFromFMarkersPhantom2(const cv::Mat& inputImg, const int rows, const int cols, std::vector<cv::Point2f>& points2Ds);
	void calc_normal_vector(const cv::Point3f& point1, const cv::Point3f& point2, const cv::Point3f& point3,
		cv::Point3f& vectorX, cv::Point3f& vectorY, cv::Point3f& normalVector);
	cv::Point3f fit_plane_to_points(const std::vector<cv::Point3f>& points);
	int Get3DPostionsFromLegoPhantom(const double marker_r,
		const cv::Point3f& marker1, const cv::Point3f& marker2, const cv::Point3f& marker3, const cv::Point3f& marker4,
		std::vector<cv::Point3f>& points3Ds);
	int Get3DPostionsFromLegoPhantom2(const float marker_r,
		const cv::Point3f& marker1, const cv::Point3f& marker2, const cv::Point3f& marker3, const cv::Point3f& marker4,
		std::vector<cv::Point3f>& points3Ds);
	int Get3DPostionsFromFMarkersPhantom(const glm::fvec3& marker0, const glm::fvec3& marker1, const glm::fvec3& marker2, const glm::fvec3& marker3,
		const track_info* trk, const int rows, const int cols,
		std::vector<cv::Point3f>& points3Ds);


	void InitializeTask(__GC* gcp);
	int RegisterCArmImage(const int sidScene, const std::string& carmScanParams, const std::string& scanName);
	bool CalibrationWithPhantom(glm::fmat4x4& matCArmRB2SourceCS, const cv::Mat& downloadedGrayImg, const track_info* trk, const bool useGlobal);

private:

	__GC* __gc;
};