#include "hunter_arm_program_planning/visionManager.h"

VisionManager::VisionManager(float length, float breadth)
{
	this->QRCode_length = length;
	this->QRCode_breadth = breadth;
	flag_detectQR = true;
	this->pixels_permm_x=1040;
	this->pixels_permm_y=1040;
}

void VisionManager::get2DLocation(cv::Mat img, float &x, float &y)
{
	this->curr_img = img;
	cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
    cv::imshow("img", curr_img);
	cv::waitKey(0);
	cv::destroyWindow("img");	
	img_centre_y_ = img.rows / 2;
	img_centre_x_ = img.cols / 2;
	std::cout << "rows: " << img.rows << std::endl;
	std::cout << "cols: " << img.cols << std::endl;
	// if(this->flag_detectQR)
	// {
	// 	detectQRCode();
	// 	this->flag_detectQR = false;
	// }
	detect2DObject(x, y);
	convertToMM(x, y);
}

void VisionManager::detectQRCode()
{
	// Extract Table from the image and assign values to pixel_per_mm fields
	cv::Mat imgHSV,Mask;
	cv::Mat image = curr_img.clone();//克隆图片
    cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);//转换到hsv色彩空间

    cv::Scalar lower(0, 0, 0);//得到色彩空间的最小阀值
	cv::Scalar upper(180, 255, 30);//得到色彩空间的最大阀值
	cv::inRange(imgHSV, lower, upper, Mask);//二值化，在这一空间内的显示白色，否则显示黑色并保存在Mask里面
 
    cv::Mat structureElement1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));//构造结构元素
	cv::erode(Mask, Mask, structureElement1, cv::Point(-1, -1), 1);//进行腐蚀操作
    cv::Mat structureElement2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));//构造结构元素
	cv::dilate(Mask, Mask, structureElement2, cv::Point(-1, -1), 1);//进行膨胀操作

	cv::namedWindow("dilate, erode", cv::WINDOW_AUTOSIZE);
    cv::imshow("dilate, erode", Mask);
	cv::waitKey(0);
	cv::destroyWindow("dilate, erode");
	cv::imwrite("/home/wky/Pictures/erode_qr.jpg", Mask);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(Mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	std::vector<cv::Point> conPoly;
	cv::Rect boundRect;
	for (int i = 0; i < contours.size(); i++)//所有轮廓遍历
	{
		float length = cv::arcLength(contours[i], true);//轮廓周长
		cv::approxPolyDP(contours[i], conPoly, 0.015 * length, true);//把一个连续光滑曲线折线化,第三个参数表示原始曲线到近似曲线的最大距离
		boundRect = cv::boundingRect(conPoly);//获得包覆该轮廓最小矩形
		int objCor = (int)conPoly.size();//获取折线化后轮廓的顶点个数
		if(objCor <= 5 && objCor > 3)
		{
			break;
		}
	}


	// std::vector<cv::Point> nonZeroPoints;
	// cv::findNonZero(Mask, nonZeroPoints);
	// cv::Rect boundRect = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pt.x = boundRect.x + boundRect.width / 2;
	pt.y = boundRect.y + boundRect.height / 2;
	cv::circle(Mask, pt, 2, cv::Scalar(0, 0, 255), -1, 8);
	// cv::namedWindow("center", cv::WINDOW_AUTOSIZE);
    // cv::imshow("center", Mask);
	// cv::waitKey(0);
	// cv::destroyWindow("center");
	cv::imwrite("/home/wky/Pictures/center——qr.jpg", Mask);
	// Update pixels_per_mm fields
	pixels_permm_y = boundRect.height / QRCode_length;
	pixels_permm_x = boundRect.width  / QRCode_breadth;

	// Test the conversion values
	std::cout << "pixels_permm_x is : " << pixels_permm_x << std::endl;
	std::cout << "pixels_permm_y is : " << pixels_permm_y << std::endl;	

}

void VisionManager::detect2DObject(float &pixel_x, float &pixel_y)
{

        // Extract Table from the image and assign values to pixel_per_mm fields
	cv::Mat imgHSV,Mask,Mask1,Mask2;
	cv::Mat image = curr_img.clone();//克隆图片
    cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);//转换到hsv色彩空间

    cv::Scalar lower(0, 43, 46);//得到色彩空间的最小阀值
	cv::Scalar upper(10, 255, 255);//得到色彩空间的最大阀值
    cv::Scalar lower1(156, 43, 46);//得到色彩空间的最小阀值
	cv::Scalar upper1(180, 255, 255);//得到色彩空间的最大阀值
    cv::Scalar lower2(100, 43, 46);//得到色彩空间的最小阀值
	cv::Scalar upper2(124, 255, 255);//得到色彩空间的最大阀值		
	cv::inRange(imgHSV, lower, upper, Mask);//二值化，在这一空间内的显示白色，否则显示黑色并保存在Mask里面
	cv::inRange(imgHSV, lower1, upper1, Mask1);
	cv::inRange(imgHSV, lower2, upper2, Mask2);	
	Mask=(Mask+Mask1+Mask2);

    cv::Mat structureElement1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));//构造结构元素
	cv::erode(Mask, Mask, structureElement1, cv::Point(-1, -1), 1);//进行腐蚀操作
	// cv::namedWindow("inrange", cv::WINDOW_AUTOSIZE);
	// cv::imshow("inrange", Mask);
	// cv::waitKey(0);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(Mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));//找出轮廓点集
	std::vector<cv::Moments> mu(contours.size());
	for (int i = 0; i < contours.size(); i++)//找出各个轮廓的矩并存放到vector容器mu
	{
		mu[i] = cv::moments(contours[i], false);
	}
	for (int i = 0; i < contours.size(); i++)//根据目标轮廓的长度，找出目标轮廓
	{
		if(cv::arcLength(contours[i],true)>0 && cv::arcLength(contours[i],true)<300)
			{
				//计算目标质心
				pixel_x = int(mu[i].m10 / mu[i].m00);
				pixel_y = int(mu[i].m01 / mu[i].m00);
				break;
			}
	}
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(0, 0, 255);
		cv::drawContours(image, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
	}
	// cv::namedWindow("contour", cv::WINDOW_AUTOSIZE);
	// cv::imshow("contour", image);
	// cv::waitKey(0);
	// cv::namedWindow("Point", cv::WINDOW_AUTOSIZE);
	cv::circle(image, cv::Point(pixel_x,pixel_y), 3, cv::Scalar(0, 255, 120), -1);//画点，其实就是实心圆
	// cv::namedWindow("Point", cv::WINDOW_AUTOSIZE);
	// cv::imshow("Point", image);
	// cv::waitKey(0);
	// cv::destroyWindow("Point");
	cv::imwrite("/home/wky/Pictures/point——box.jpg", image);
	std::cout << "obj_pixel_x: " << pixel_x << std::endl;
	std::cout << "obj_pixel_y: " << pixel_y << std::endl;

}
void VisionManager::convertToMM(float &x, float &y)
{
	x = (x - img_centre_x_) / pixels_permm_x;
	y = (y - img_centre_y_) / pixels_permm_y;
}
int main()
{
  cv::Mat img = cv::imread("/home/wky/1.png");
  float x, y;
  VisionManager vm(100,100);
  vm.get2DLocation(img,x,y);
}
