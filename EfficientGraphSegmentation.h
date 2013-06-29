/*=========================================================================

  Program:   Artas Treatment Planning System
  Module:    Figure Rect Detection---sysPhotoOpt
  Language:  C++
  Date:      12/05/2011
  Version:   1.0

  Copyright (c) Restoration Robotics Inc. All rights reserved.
  
  Author: Yinxiao Li

---------------------------------------------------------------------------
  Update: 

=========================================================================*/
#include <cv.h>
#include <highgui.h>

class GraphSegmentation
{
public:
	GraphSegmentation(void);
	virtual ~GraphSegmentation(void);

public:
	void FHGraphSegment(IplImage* inputImage);
	void FindHeadRect(char* pathFileName, double bottomRatio);
	void FHGraphSegmentation(
		IplImage* img, 
		float c, 
		int min_size,
		IplImage *SegLabel); 

public:
	CvRect* GetHeadRect();
	CvRect* GetImageRect() { return &imageRect; };

private:
	CvRect headRect;
	CvRect imageRect;
	IplImage* outputImage;


//////////////////////////////////////////////////////////////////////////
//Define a c++ wrapper for single-channel byte images, multi-channel byte images, and multi-channel float images:
//Direct access using a c++ wrapper: (Simple and efficient access)
//////////////////////////////////////////////////////////////////////////
	template<class T> class Image
	{
	private:
		IplImage* imgp;
	public:
		Image(IplImage* img=0) {imgp=img;}
		~Image(){imgp=0;}
		void operator=(IplImage* img) {imgp=img;}
		inline T* operator[](const int rowIndx) {
			return ((T *)(imgp->imageData + rowIndx*imgp->widthStep));}
	};  

	//Note, X, Y flipped!!!

	typedef struct{
		unsigned char b,g,r;
	} RgbPixel;

	typedef struct{
		float b,g,r;
	} RgbPixelFloat;

	typedef Image<RgbPixel>       RgbImage;
	typedef Image<RgbPixelFloat>  RgbImageFloat;
	typedef Image<unsigned char>  BwImage;
	typedef Image<float>          BwImageFloat;
};
