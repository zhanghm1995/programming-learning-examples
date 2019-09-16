#include "ImageRectify.h"

#include<fstream>
#include<iostream>
#include<opencv2/imgproc.hpp>

ImageRectify::ImageRectify(const std::string& strSettingsFile, const std::string& prefix)
{
	Init(strSettingsFile, prefix);
}


bool ImageRectify::Init(const std::string &strSettingsFile, const std::string &prefix)
{
	cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		std::cerr << "ERROR: Wrong path to settings" << std::endl;
		return false;
	}

	cv::Mat K, P, R, D;
	fsSettings[prefix + ".K"] >> K;
	fsSettings[prefix + ".P"] >> P;
	fsSettings[prefix + ".R"] >> R;
	fsSettings[prefix + ".D"] >> D;

	int rows = fsSettings[prefix + ".height"];
	int cols = fsSettings[prefix + ".width"];

	if (K.empty() || P.empty() || R.empty() || D.empty() || rows == 0 || cols == 0)
	{
		std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
		return false;
	}

	cv::initUndistortRectifyMap(K, D, R, P.rowRange(0, 3).colRange(0, 3), cv::Size(cols, rows), CV_32F, M1, M2);

	return true;
}

void ImageRectify::doRectify(const cv::Mat &srcImage, cv::Mat &dstImage)
{
	cv::remap(srcImage, dstImage, M1, M2, cv::INTER_LINEAR);
}
