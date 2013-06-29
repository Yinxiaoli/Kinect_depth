#include <iostream>

#include <cv.h>
#include <highgui.h>


int main()
{

	cv::Mat image = cv::imread("images.jpg");
	cv::imshow("Display", image);
	cv::waitKey();

	return 0;
}