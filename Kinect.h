#ifndef ethos_cpr_kinect_h
#define ethos_cpr_kinect_h

#include <cv.h>  // <--- OpenCV Include file

#include <Windows.h>  // <--- Need this before NuiApi.h b/c windows is annoying
#include <NuiApi.h>   // <--- Main Kinect Incldue file

#include <fstream>

#include <d3d11.h>

namespace ethos
{
  namespace cpr
  {
    // A data structure holding the 3D position of each joint for a
    // skeleton
    struct SkeletonJoints
    {
      cv::Point3f head,
        shoulderCenter, shoulderLeft, shoulderRight,
        spine,
        hipCenter, hipLeft, hipRight,
        elbowLeft, elbowRight,
        wristLeft, wristRight,
        handLeft, handRight,
        kneeLeft, kneeRight,
        ankleLeft, ankleRight;

      double timestamp;
    };

    // A class which talks to a Kinect sensor and grabs depth images.  For now
    // no color images are retrieved, but this can be added later if necessary.
    class Kinect
    {

	  static const NUI_IMAGE_RESOLUTION   cDepthResolution = NUI_IMAGE_RESOLUTION_640x480;
	  static const NUI_IMAGE_RESOLUTION   cColorResolution = NUI_IMAGE_RESOLUTION_640x480;

	public:
      // The known image dimensions of a depth image from the Kinect
      static const int cDepthWidth    = 640;
      static const int cDepthHeight   = 480;
      static const int cBytesPerPixel = 4;

      // Constructor
      Kinect();

      // Destructor
      ~Kinect();

      // This is the initialization function.  Call this once to find and setup the
      // Kinect.  Returns true if successful; otherwise returns false.  Only call this ONCE!
      bool Initialize();

      // Get a depth image from the Kinect.  It is stored as a 640x480 image with
      // 4 bytes per pixel.  This is mainly for display.
      bool GetDepthImage(cv::Mat& depthImg);

      // Get a color image from the Kinect.  It is stored as a 640x480 4-channel image.
      bool GetColorImage(cv::Mat& colorImg);

      // Get the skeletal image from the Kinect (e.g., the projection of the 3D skeleton
      // into a 2D image).
      bool GetSkeletalImage(cv::Mat& skeletalImg, bool* foundSkeleton);

      // Get the most recent skeleton 3D joint positions
      inline const SkeletonJoints& GetSkeletalJoints() const
      {
        return mJoints;
      }

      // Save the most recent skeleton 3D joint positions to an output text stream
      void SaveSkeletalJoints(std::ofstream& ofs) const;

      // Cleanup and close the connection to the kinect.  If the user doesn't call this,
      // the destructor will automatically call it (and if it gets called twice there are
      // safeguards so don't worry)
      void Cleanup();

	  LONG* GetColorCoordinates();

    protected:
      // Draws a skeleton
      void Nui_DrawSkeleton( const NUI_SKELETON_DATA & skel, int windowWidth, int windowHeight, cv::Mat& canvas );

      // Draws a line between two bones
      void Nui_DrawBone( const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX bone0, NUI_SKELETON_POSITION_INDEX bone1, cv::Mat& canvas );

      // Converts a skeleton point to screen space
      cv::Point2f SkeletonToScreen( Vector4 skeletonPoint, int width, int height );

      // Determines which skeletons to track and tracks them
      void UpdateTrackedSkeletons( const NUI_SKELETON_FRAME & skel );

      // Convert a skeleton joint position to a cv::Point3f
      cv::Point3f ConvertJoint(const Vector4& vec) const;

    private:
      // The main Kinect object
      INuiSensor* m_pNuiSensor;

      // An array which holds the depth image data directly from the Kinect
      BYTE* m_depthRGBX;

      // How many bytes per depth image (width*height*4)
      const int m_numBytes;

      // Some internal handles used for the threading within the Kinect
      HANDLE m_hNextDepthFrameEvent;
      HANDLE m_hNextColorFrameEvent;
      HANDLE m_hNextSkeletonEvent;
      HANDLE m_pVideoStreamHandle;
      HANDLE m_pDepthStreamHandle;

      cv::Point2f m_Points[NUI_SKELETON_POSITION_COUNT];

      DWORD m_StickySkeletonIds[NUI_SKELETON_MAX_TRACKED_COUNT];
      int m_TrackedSkeletons;

      SkeletonJoints mJoints;

	  USHORT* m_depthD16;
	  LONG m_depthWidth;
	  LONG m_depthHeight;
	  LONG m_colorWidth;
	  LONG m_colorHeight;
	  LONG* m_colorCoordinates;
      LONG m_colorToDepthDivisor;

	  BYTE* m_colorRGBX;

	  ID3D11DeviceContext* m_pImmediateContext;
    };
  }
}


#endif

