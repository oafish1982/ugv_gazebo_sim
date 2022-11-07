#ifndef PROBOT_VISION_MANAGER
#define PROBOT_VISION_MANAGER

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/types_c.h>


class VisionManager
{
  public:
	VisionManager(float length, float breadth);
	void get2DLocation(cv::Mat img, float &x, float &y);

  private:
	void detect2DObject(float &pixel_x, float &pixel_y);
	void convertToMM(float &pixel_mm_x, float &pixel_mm_y);
	void detectQRCode();

	bool flag_detectQR;
	float pixels_permm_x;
	float pixels_permm_y;
	float curr_pixel_centre_x;
	float curr_pixel_centre_y;
	float QRCode_length;
	float QRCode_breadth;
	float img_centre_x_;
	float img_centre_y_;
	cv::Mat curr_img;
};

#endif
