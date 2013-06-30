#include <iostream>
#include <direct.h>
#include <fstream>

#include <cv.h>
#include <highgui.h>
#include <vector>
#include "C:\opencv\modules\core\include\opencv2\core\core.hpp"

#include "D:\MyResearch\Kinect_Austin\Kinect_depth\Kinect.h"
#include "EfficientGraphSegmentation.h"

//#define USE_DEPTH_IMAGES
//#define USE_COLOR_IMAGES
//#define USE_SKELETAL_IMAGES
#define USE_COLOR_AND_DEPTH_IMAGES


//Global corner point
CvPoint leftupPt = cvPoint(0,0);
CvPoint leftDownPt = cvPoint(0,0);
CvPoint rightupPt = cvPoint(0,0);
CvPoint rightDownPt = cvPoint(0,0);

std::vector<cv::Point> ROI_Vertices; 


bool bClicked;


//---------------------------------------------------------------------------------------
// Convert 4-channel 8-bit image to 3-channel 8-bit image for
// saving to image files.
void prepareToSave(const cv::Mat& img4, cv::Mat& img3)
{
  cv::cvtColor(img4, img3, CV_BGRA2BGR);
}


void on_mouse( int event, int x, int y, int flags, void* ustc)  
{  
	CvFont font;  
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);  

	if( event == CV_EVENT_LBUTTONDOWN && bClicked == false)  
	{  
		ROI_Vertices.push_back(cv::Point(x,y));     
	}  

	if (event == CV_EVENT_RBUTTONDOWN )
	{
		printf("Done corner points!!!");
		bClicked = true;
	}

	/*
	if( event == CV_EVENT_LBUTTONDOWN )  
	{  
		leftupPt = cvPoint(x,y);     
	}  
	if (event == CV_EVENT_MBUTTONDOWN )
	{
		rightupPt = cvPoint(x,y);  
	}
	if (event == CV_EVENT_RBUTTONDOWN )
	{
		rightDownPt = cvPoint(x,y);  
	}
	if (leftupPt.x != 0 && rightupPt.x != 0 && rightDownPt.x != 0 && bClicked == false)
	{
		leftDownPt = cvPoint(leftupPt.x, rightDownPt.y);
		printf("Done corner points!!!");	
		bClicked = true;
	}
	*/
}  



//---------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  // Setup the kinect
  ethos::cpr::Kinect kinect;
  if (!kinect.Initialize())
  {
    return -1;
  }

  bClicked = false;

  // The image holders
  cv::Mat depthImg, colorImg, skeletalImg, maskOverColorImg, maskImg, maskedDepthImg, labelImg;
  bool foundSkeleton;
  cv::Mat CroppedDepth, CroppedColor;
  cv::Mat CaliDepth;
  
  cv::Size dSize = cv::Size(320, 240);

  // 3-channel versions of image holders for depth and color
  // (skeleton already 3-channels)
  cv::Mat depthImg3, colorImg3;

  // Save images?
  const bool saveDebugImages = true;
  char imgFilename[1024*1024];
  int imgFrameCounter = 0;
  if (saveDebugImages)
  {
#ifdef USE_DEPTH_IMAGES
    _mkdir("depth");
#endif
#ifdef USE_COLOR_IMAGES
    _mkdir("color");
#endif
#ifdef USE_SKELETAL_IMAGES
    _mkdir("skeleton");
#endif
  }

  // Save joint values?
  const bool saveJoints = true;
  char jointFilename[1024];
  int jointFrameCounter = 0;
  if (saveJoints)
  {
#ifdef USE_SKELETAL_IMAGES
    _mkdir("joints");
#endif
  }

  // Grab images until user says to stop
  while (true)
  {
#ifdef USE_DEPTH_IMAGES
    // Grab next frames from the kinect
    if (!kinect.GetDepthImage(depthImg))
    {
      break;
    }
    cv::imshow("Depth", depthImg);
#endif

#ifdef USE_COLOR_IMAGES
    if (!kinect.GetColorImage(colorImg))
    {
      break;
    }
    cv::imshow("Color", colorImg);
#endif

#ifdef USE_SKELETAL_IMAGES
    if (!kinect.GetSkeletalImage(skeletalImg, &foundSkeleton))
    {
      break;
    }
    //const ethos::cpr::SkeletonJoints& joints = kinect.GetSkeletalJoints();
    cv::imshow("Skeleton", skeletalImg);
#endif

#ifdef USE_COLOR_AND_DEPTH_IMAGES
	if (!kinect.GetDepthImage(depthImg))
	{
		break;
	}
	cv::imshow("Depth", depthImg);
	if (!kinect.GetColorImage(colorImg))
	{
		break;
	}
	cv::imshow("Color", colorImg);
#endif

	cvSetMouseCallback( "Depth", on_mouse, 0 );
	//Calibration
	{
		cv::Point3f depthPt;
		cv::Point3f colorPt;
		CaliDepth = depthImg;
		CaliDepth.setTo(0);

		for (int j = 0; j < depthImg.rows; j ++)
		{
			uchar* data_depth= depthImg.ptr<uchar>(j); 
			for (int i = 0; i < depthImg.cols; i ++)
			{				
				depthPt.z = data_depth[i] * 5.0f;	           //z, according to the previous convert factor: depthF /= 1300.0f; depthF *= 255.0f. Revert back;
				depthPt.x = depthPt.z * (i - 324.3) / 526.7;   //x
				depthPt.y = depthPt.z * (i - 247.8) / 525.8;   //y

				//convert to color camera 2D
				colorPt.x =  509.98f * depthPt.x - 18.124f * depthPt.y + 349.569f * depthPt.z - 11661.2f; //x
				colorPt.y = -11.765f * depthPt.x + 512.72f * depthPt.y + 273.624f * depthPt.z + 153.29f;  //y
				colorPt.z = -0.0496f * depthPt.x - 0.0502f * depthPt.y + 0.9975f  * depthPt.z + 7.4660f;  //w

				colorPt.x = (int)(colorPt.x / colorPt.z);  //normalize by w
				colorPt.y = (int)(colorPt.y / colorPt.z);  //normalize by w

				//check boundary
				if (colorPt.x > depthImg.cols || colorPt.x < 0 ||
					colorPt.y > depthImg.cols || colorPt.y < 0)
					continue;

				int ii = colorPt.y;  //ii is the transformed x in color image
				int jj = colorPt.x;  //jj is the transformed y in color image

				CaliDepth.ptr<uchar>(jj)[ii] = data_depth[i]; //assign the same depth value from the original depth image with a new position (ii,jj)
			}
		}

		cv::imshow("Cali_Depth", CaliDepth );
		
// 		//points in camera 3D
// 		std::vector<cv::Vec3b> color3D;
// 		for (int i = 0; i < depth3D.size(); i++)
// 		{
// 			cv::Vec3b temp;
// 			cv::Vec3b depth_temp = depth3D[i];
// 			depthPt.x =  0.9988 * depth_depthPt.x - 0.0035 * depth_depthPt.y + 0.0495 * depth_depthPt.z - 26.7385;
// 			depthPt.y =  0.0010 * depth_depthPt.x + 0.9987 * depth_depthPt.y + 0.0503 * depth_depthPt.z - 3.2267;
// 			depthPt.z = -0.0496 * depth_depthPt.x - 0.0502 * depth_depthPt.y + 0.9975 * depth_depthPt.z + 7.4660;
// 			color3D.push_back(temp);
// 		}

		

	}






	//set Callback function, only call once per run



	if (bClicked == true)
	{
		//We start to process the image
		
// 		maskImg = cvCreateMat(colorImg.rows, colorImg.cols, CV_8UC1);
// 		maskImg.setTo(0);
// 		maskOverColorImg = cvCreateMat(colorImg.rows, colorImg.cols, CV_8UC1);
// 		maskOverColorImg.setTo(0);

		cv::Point leftUpCorner = ROI_Vertices[0];
		cv::Point rightDownCorner = ROI_Vertices[1];

		cv::Rect myROI(leftUpCorner.x, leftUpCorner.y, rightDownCorner.x - leftUpCorner.x, rightDownCorner.y - leftUpCorner.y);

		CroppedDepth = depthImg(myROI);

		CroppedColor = colorImg(myROI);

		//cv::bilateralFilter(CroppedDepth, CroppedDepth, CV_BILATERAL, 3, 0);


		for (int i = 0; i < 2; i ++)
		{
			cv::GaussianBlur(CroppedDepth, CroppedDepth, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
		}

		cv::imshow("new_Depth", CroppedDepth );
		cv::imshow("new_Color", CroppedColor);


		
		/*

		std::vector<cv::Point> ROI_Poly;
		cv::approxPolyDP(ROI_Vertices, ROI_Poly, 1.0, true);
		cv::fillConvexPoly(maskImg, &ROI_Poly[0], ROI_Poly.size(), cv::Scalar(255,255,255));                 
		colorImg.copyTo(maskOverColorImg, maskImg);


		inRange(maskOverColorImg, cv::Scalar(15, 15, 15), cv::Scalar(124, 154, 95), maskedDepthImg);
		//inRange(maskOverColorImg, cv::Scalar(50, 65, 40), cv::Scalar(124, 154, 95), maskedDepthImg);
		cv::imshow("Mask", maskOverColorImg);

		int erosion_size = 1;   
		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
			cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
			cv::Point(erosion_size, erosion_size) );
		
		for (int j=0; j<maskedDepthImg.rows; j++)  
		{  
			uchar* data_depth= maskedDepthImg.ptr<uchar>(j);  
			uchar* data_mask = maskImg.ptr<uchar>(j);
			for (int i=0; i<maskedDepthImg.cols; i++)  
			{                   
				if (data_depth[i] == data_mask[i])
				{
					data_depth[i] = 0;
				}
				else
				{
					data_depth[i] = 255;
				}
				 
			}  
		}  
		for (int i = 0 ; i < 4; i ++)
			cv::erode(maskedDepthImg, maskedDepthImg, element);	
		for (int i = 0; i < 5; i ++)
			cv::dilate(maskedDepthImg, maskedDepthImg, element);


		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		findContours( maskedDepthImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		if( !contours.empty() && !hierarchy.empty() )
		{
			for (int idx=0;idx < contours.size();idx++)
			{
				drawContours(maskedDepthImg,contours,idx,cv::Scalar::all(255),CV_FILLED,8);
			}
		}

		cv::imshow("Depth_Mask", maskedDepthImg);
		maskImg.setTo(0);
		depthImg.copyTo(maskImg, maskedDepthImg);
		cv::imshow("Final", maskImg);

		resize(maskImg, maskImg, dSize);
		*/
	}

	
    // (Optionally) save output debugging images
    if (saveDebugImages)
    {
#ifdef USE_DEPTH_IMAGES
      sprintf(imgFilename, "depth/depthFrame_%04d.png", imgFrameCounter);
      prepareToSave(depthImg, depthImg3);
      cv::imwrite(imgFilename, depthImg3);
#endif

#ifdef USE_COLOR_IMAGES
      sprintf(imgFilename, "color/colorFrame_%04d.png", imgFrameCounter);
      prepareToSave(colorImg, colorImg3);
      cv::imwrite(imgFilename, colorImg3);
#endif

#ifdef USE_SKELETAL_IMAGES
      sprintf(imgFilename, "skeleton/skeletonFrame_%04d.png", imgFrameCounter);
      cv::imwrite(imgFilename, skeletalImg);
#endif

#ifdef USE_COLOR_AND_DEPTH_IMAGES
	  int key = cv::waitKey(1);
	  if (key == 's')
	  //if (bClicked == true)
	  {
// 		  sprintf(imgFilename, "depth/depthFrame_%04d.png", imgFrameCounter);
// 		  prepareToSave(depthImg, depthImg3);
// 		  cv::imwrite(imgFilename, depthImg3);

		  sprintf(imgFilename, "depth_mask/depthFrame_%04d.jpg", imgFrameCounter);
		  prepareToSave(CroppedDepth, depthImg3);
		  cv::imwrite(imgFilename, depthImg3);

		  sprintf(imgFilename, "color/colorFrame_%04d.png", imgFrameCounter);
		  prepareToSave(CroppedColor, colorImg3);
		  cv::imwrite(imgFilename, colorImg3);
	  }
#endif

      ++imgFrameCounter;
    }

    // (Optionally) save joint values
    //if (saveJoints && foundSkeleton)
    //{
#ifdef USE_SKELETAL_IMAGES
      sprintf(jointFilename, "joints/jointFrame_%04d.txt", jointFrameCounter);
      std::ofstream ofs(jointFilename);
      kinect.SaveSkeletalJoints(ofs);
      ofs.close();
#endif

    //  ++jointFrameCounter;
   // }

    // Check for user keyboard input to quit early
    int key = cv::waitKey(1);
    if (key == 'q')
    {
      break;
    }
  }

  // Exit application
  return 0;
}


