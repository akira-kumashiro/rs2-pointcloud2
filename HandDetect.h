#pragma once

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>

class HandDetect
{
public:
	HandDetect(double nearThreshold, double farThreshold);
	std::vector<cv::Point> getTipData(cv::Mat depth, cv::Mat color);
	cv::Mat colorMarked, contourMask;
	~HandDetect();
	class MatContainer
	{
	public:
		cv::Mat _mat;
		std::string name;
		MatContainer(std::string name, cv::Mat mat) :
			name(name),
			_mat(mat)
		{

		}

		MatContainer(std::string name, IplImage* mat) :
			name(name)
		{
			_mat = cv::Mat::zeros(cv::Size(mat->width, mat->height), CV_8UC3);
			for (int y = 0; y < mat->height; y++)
			{
				cv::Vec3b* _matPtr = _mat.ptr<cv::Vec3b>(y);
				for (int x = 0; x < mat->width; x++)
				{
					for (int i = 0; i < 3; i++)
					{
						_matPtr[x][i] = mat->imageData[mat->widthStep * y + x * 3 + i];
					}
				}
			}
		}
	};

private:
	cv::Mat colorImage, depthImage, depthBinaryImage;
	double _nearThreshold, _farThreshold;
	cv::Mat getBinaryImage(void);
};

