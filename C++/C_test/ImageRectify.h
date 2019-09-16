#pragma once

#include<string>
#include<opencv2/opencv.hpp>


class ImageRectify
{
public:
	ImageRectify(const std::string& strSettingsFile, const std::string& prefix);
	virtual ~ImageRectify()= default;

	bool Init(const std::string &strSettingsFile, const std::string &prefix);

	void doRectify(const cv::Mat &srcImage, cv::Mat &dstImage);

protected:
	cv::Mat M1, M2;
};

