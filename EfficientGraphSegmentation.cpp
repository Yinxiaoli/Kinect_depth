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

#include "EfficientGraphSegmentation.h"
#include <string>
#include <math.h>
#include <ctime>
//#include <Windows.h>

GraphSegmentation::GraphSegmentation(void)
{

}

GraphSegmentation::~GraphSegmentation(void)
{

}


// ================> begin local functions (available only to this translation unit)

#define THRESHOLD(size, c) (c/size)

template <class T>
inline T square(const T &x) { return x*x; }; 

typedef struct {
	float w;
	int a, b;
} Edge;

typedef struct
{
	int rank;
	int p;
	int size;
} uni_elt;

class Universe
{
public:
	Universe(int elements)
	{
		num = elements;
		for (int i = 0; i < elements; i++)
		{
			uni_elt elt;
			elt.rank = 0;
			elt.size = 1;
			elt.p = i;
			elts.push_back(elt);
		}
	}
	~Universe(){};
	int find(int x)
	{
		int y = x;
		while (y != elts[y].p)
			y = elts[y].p;
		elts[x].p = y;
		return y;
	};  
	void join(int x, int y)
	{
		if (elts[x].rank > elts[y].rank)
		{
			elts[y].p = x;
			elts[x].size += elts[y].size;
		} 
		else
		{
			elts[x].p = y;
			elts[y].size += elts[x].size;
			if (elts[x].rank == elts[y].rank)
				elts[y].rank++;
		}
		num--;
	}
	int size(int x) const { return elts[x].size; }
	int num_sets() const { return num; }

private:
	std::vector<uni_elt>elts;
	int num;
};

/*
void convert(IplImage* fltImage, IplImage* chImage )
{
	int width  = fltImage->width;
	int height = fltImage->height;

	float* ptrf;
	uchar* p;
	for (int y = 0; y < height; y++)
	{
		ptrf = (float*)(fltImage->imageData + y * fltImage->widthStep);
		p = (uchar*)(chImage->imageData + y * chImage->widthStep);

		for (int x = 0; x < width; x++)
		{
			p[x] = (uchar)ptrf[x];
		}
	}
}

void convertColor(IplImage* fltImage, IplImage* chImage )
{
	int width  = fltImage->width;
	int height = fltImage->height;

	float* ptrf;
	uchar* p;
	for (int y = 0; y < height; y++)
	{
		ptrf = (float*)(fltImage->imageData + y * fltImage->widthStep);
		p = (uchar*)(chImage->imageData + y * chImage->widthStep);

		for (int x = 0; x < width; x++)
		{
			p[3 * x] = (uchar)ptrf[3 * x];
			p[3 * x + 1] = (uchar)ptrf[3 * x + 1];
			p[3 * x + 2] = (uchar)ptrf[3 * x + 2];
		}
	}
}
*/

float iDiff( IplImage* r,  IplImage* g, IplImage* b,
	int x1, int y1, int x2, int y2) 
{
// 	float r1 =  ((float*)(r->imageData + y1 * r->widthStep))[x1]; 
// 	float r2 =  ((float*)(r->imageData + y2 * r->widthStep))[x2];
// 	float g1 =  ((float*)(g->imageData + y1 * r->widthStep))[x1]; 
// 	float g2 =  ((float*)(g->imageData + y2 * r->widthStep))[x2]; 
// 	float b1 =  ((float*)(b->imageData + y1 * r->widthStep))[x1]; 
// 	float b2 =  ((float*)(b->imageData + y2 * r->widthStep))[x2]; 
 	return sqrt(square( ((float*)(r->imageData + y1 * r->widthStep))[x1] - ((float*)(r->imageData + y2 * r->widthStep))[x2] ) + 
 		square( ((float*)(g->imageData + y1 * r->widthStep))[x1] - ((float*)(g->imageData + y2 * r->widthStep))[x2] ) + 
 		square( ((float*)(b->imageData + y1 * r->widthStep))[x1] - ((float*)(b->imageData + y2 * r->widthStep))[x2] ));
}

void iBuildGraph(IplImage* smooth_r, 
	IplImage* smooth_g,
	IplImage* smooth_b,
	std::vector<Edge> *edges,
	int *num_edges)
{
	int width = smooth_r->width;
	int height = smooth_r->height;
	int num = 0;
	int x, y;
	edges->clear();
	for ( y = 0; y < height; y++)
	{
		for ( x = 0; x < width; x++)
		{
			if (x < width-1)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = y * width + (x+1);
				edge.w = iDiff(smooth_r, smooth_g, smooth_b, x, y, x+1, y);
				edges->push_back(edge);
				num++;
			}

			if (y < height-1)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = (y+1) * width + x;
				edge.w = iDiff(smooth_r, smooth_g, smooth_b, x, y, x, y+1);
				edges->push_back(edge);
				num++;
			}

			if ((x < width-1) && (y < height-1)) 
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = (y+1) * width + (x+1);
				edge.w = iDiff(smooth_r, smooth_g, smooth_b, x, y, x+1, y+1);
				edges->push_back(edge);
				num++;
			}

			if ((x < width-1) && (y > 0))
			{
				Edge edge;
				edge.a  = y * width + x;
				edge.b  = (y-1) * width + (x+1);
				edge.w  = iDiff(smooth_r, smooth_g, smooth_b, x, y, x+1, y-1);
				edges->push_back(edge);
				num++;
			}
		}
	}
	*num_edges = num;
}


void iExtractRGBColorSpace(IplImage* img, 
	IplImage* B, 
	IplImage* G,
	IplImage* R)
{
	const int width = img->width, height = img->height;
	uchar* ptr;
	float* ptrB;
	float* ptrG;
	float* ptrR;
	for (int y = 0; y < height; y++)
	{
		ptr = (uchar*)(img->imageData + y * img->widthStep);
		ptrB = (float*)(B->imageData + y * B->widthStep);
		ptrG = (float*)(G->imageData + y * G->widthStep);
		ptrR = (float*)(R->imageData + y * R->widthStep);
		for (int x = 0; x < width; x++)
		{
			ptrB[x] = (float)ptr[3 * x];
			ptrG[x] = (float)ptr[3 * x + 1 ];
			ptrR[x] = (float)ptr[3 * x + 2 ];
		}
	}
	
}

void iSmooth(IplImage* src, IplImage* out)
{
	cvSmooth(src, out, CV_GAUSSIAN, 3, 3);
}

bool lessThan (const Edge& a, const Edge& b) {
	return a.w < b.w;
}

void iSegment_graph(int num_vertices, int num_edges, std::vector<Edge>& edges, float c, Universe *u)
{ 
	// sort edges by weight
	std::sort(&edges[0], &edges[num_edges-1], lessThan);

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	for (i = 0; i < num_vertices; i++)
		threshold[i] = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	for (i = 0; i < num_edges; i++) 
	{
		Edge edge = edges[i];
		// components connected by this edge
		int a = u->find(edge.a);
		int b = u->find(edge.b);
		if (a != b) 
		{
			if ((edge.w <= threshold[a]) &&
				(edge.w <= threshold[b])) {
					u->join(a, b);
					a = u->find(a);
					threshold[a] = edge.w + THRESHOLD(u->size(a), c);
			}
		}
	}

	// free up
	delete threshold;
}

// ================> end local functions (available only to this translation unit)


void GraphSegmentation::FHGraphSegmentation(
	IplImage* img, 
	float c, 
	int min_size,
	IplImage *SegLabel) 
{
	int width = img->width;
	int height = img->height;
	int x, y;

	IplImage* B = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 1);
	IplImage* G = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 1);
	IplImage* R = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 1);

	iExtractRGBColorSpace(img, B, G, R);
	
	IplImage* smooth_B = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 1);
	IplImage* smooth_G = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 1);
	IplImage* smooth_R = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 1);

	iSmooth(B, smooth_B);
	iSmooth(G, smooth_G);
	iSmooth(R, smooth_R);
	
	cvReleaseImage(&B);
	cvReleaseImage(&G);
	cvReleaseImage(&R);

	std::vector<Edge> edges;
	int num_edges;
	iBuildGraph(smooth_B, smooth_G, smooth_R, &edges, &num_edges);
	Universe u(width * height);
	iSegment_graph(width * height, num_edges, edges, c, &u);

	int i;
	for (i = 0; i < num_edges; i++)
	{
		int a = u.find(edges[i].a); 
		int b = u.find(edges[i].b);
		if ((a != b) && ((u.size(a) < min_size) || (u.size(b) < min_size)))
		{
			u.join(a, b);
		}
	}

	uchar* ptr;
	for (y = 0; y < height; y++)
	{
		ptr = (uchar*)(SegLabel->imageData + y * SegLabel->widthStep);
		for ( x = 0; x < width; x++)
		{
			int comp = u.find(y * width + x);
			ptr[x] = comp % 255;
		}
	}  

	cvReleaseImage(&smooth_B);
	cvReleaseImage(&smooth_G);
	cvReleaseImage(&smooth_R);
}


void GraphSegmentation::FHGraphSegment(IplImage* imgLoad)
{
	
	CvSize newSZ;
	newSZ.height = 512;
	newSZ.width = (int)((double)512/(double)imgLoad->height*(double)imgLoad->width);
	IplImage* img = cvCreateImage(newSZ, imgLoad->depth, imgLoad->nChannels);
	cvResize(imgLoad, img, CV_INTER_LINEAR);

	imageRect.x = 0;
	imageRect.y = 0;
	imageRect.width = newSZ.width;
	imageRect.height = newSZ.height;

	outputImage = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);

	clock_t startTime, endTime;
	startTime = clock();
	FHGraphSegmentation(img, 500.0f, img->width * img->height /8, outputImage);
	endTime = clock();

	int timePeriod = 1000 * (endTime - startTime) / CLOCKS_PER_SEC;

	char time[100];
	sprintf(time, "%d", timePeriod);
	//MessageBox(0, time, "period", MB_OK);

// 	cvShowImage( "Jim_Front", outputImage);
// 	cvWaitKey(0);
// 	cvDestroyWindow("Jim_Front");

	cvReleaseImage(&img);
}

void GraphSegmentation::FindHeadRect(char* pathFileName, double bottomRatio)
{
	IplImage* imgLoad = cvLoadImage( pathFileName );

    //First do Graph segmentation
	FHGraphSegment(imgLoad);

	headRect = cvRect(0,0,0,0);
	int left = 0, right = 0, top = 0, bottom = 0;
	bool flag = false;
	const int width = outputImage->width, height = outputImage->height;
	BwImage imgLabel(outputImage);

	int headColor = imgLabel[width/2][height/2];

	//find top
	flag = false;
	for (int j = 0; j < height; j++)
		for (int i = 0; i < width; i++)
		{
			if (flag == true) break;
			if (imgLabel[j][i] == headColor)
			{
				top = j;
				flag = true;
				break;
			}
		}

		//find left
		flag = false;
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height * 0.7; j++)
			{
				if (flag == true) break;
				if (imgLabel[j][i] == headColor)
				{
					left = i;
					flag = true;
					break;
				}
			}

		//find right
		flag = false;
		for (int i = width-1; i > 0; i--)
			for (int j = 0; j < height * 0.7; j++)
			{
				if (flag == true) break;
				int color = imgLabel[j][i];
				if (imgLabel[j][i] == headColor)
				{
					right = i;
					flag = true;
					break;
				}
			}

		//add a ratio to the height of the rect
		bottom = (int)((double)top + bottomRatio * (double)(right - left));

		if (bottom >= height * 0.95) //bottom cannot exceed the boundary
		{
			bottom = (int)(height * 0.95);
		}

	headRect = cvRect(left, top, (right - left), (bottom - top));

// 	if (left < 0.05 * width ||
// 		right > 0.95 * height ||
// 		top < 0.05 * height)
// 	{
// 		headRect = cvRect(0,0,0,0);
// 	}

	cvReleaseImage(&outputImage);
	cvReleaseImage(&imgLoad);
}

CvRect* GraphSegmentation::GetHeadRect()
{
	CvRect *iHeadRect;
	iHeadRect = &headRect;
	return iHeadRect;
}