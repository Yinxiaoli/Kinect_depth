#include "D:\MyResearch\Kinect_Austin\Kinect_depth\Kinect.h"
#include <iostream>


namespace ethos
{
  namespace cpr
  {
    static const int g_JointThickness = 3;
    static const int g_TrackedBoneThickness = 6;
    static const int g_InferredBoneThickness = 1;

    enum _SV_TRACKED_SKELETONS
    {
      SV_TRACKED_SKELETONS_DEFAULT = 0,
      SV_TRACKED_SKELETONS_NEAREST1,
      SV_TRACKED_SKELETONS_NEAREST2,
      SV_TRACKED_SKELETONS_STICKY1,
      SV_TRACKED_SKELETONS_STICKY2
    } SV_TRACKED_SKELETONS;

    //----------------------------------------------------------------------------------
    Kinect::Kinect()
      : m_depthRGBX(NULL), 
      m_pNuiSensor(NULL),
      m_numBytes(cDepthWidth*cDepthHeight*cBytesPerPixel)
    {
      m_depthRGBX = new BYTE[m_numBytes];
      ZeroMemory(m_StickySkeletonIds, sizeof(m_StickySkeletonIds));
      m_TrackedSkeletons = 0;

	  DWORD width = 0;
	  DWORD height = 0;
	  NuiImageResolutionToSize(cDepthResolution, width, height);
	  m_depthWidth  = static_cast<LONG>(width);
	  m_depthHeight = static_cast<LONG>(height);

	  NuiImageResolutionToSize(cColorResolution, width, height);
	  m_colorWidth  = static_cast<LONG>(width);
	  m_colorHeight = static_cast<LONG>(height);

	  m_colorToDepthDivisor = m_colorWidth/m_depthWidth;

	  m_depthD16 = new USHORT[m_depthWidth*m_depthHeight];
	  m_colorCoordinates = new LONG[m_depthWidth*m_depthHeight*2];
	  m_colorRGBX = new BYTE[m_colorWidth*m_colorHeight*cBytesPerPixel];

	  m_pImmediateContext = NULL;
    }


    //----------------------------------------------------------------------------------
    Kinect::~Kinect()
    {
      Cleanup();
	  if (m_pImmediateContext) 
	  {
		  m_pImmediateContext->ClearState();
	  }
    }


    //----------------------------------------------------------------------------------
    bool Kinect::Initialize()
    {
      // Find the Kinect
      INuiSensor * pNuiSensor;
      HRESULT hr;
      int iSensorCount = 0;
      hr = NuiGetSensorCount(&iSensorCount);
      if (FAILED(hr))
      {
        std::cerr << "Kinect::Initialize() -- ERROR!  Could not retrieve Kinect sensor count!\n";
        return false;
      }

      if (iSensorCount <= 0)
      {
        std::cerr << "Kinect::Initialize() -- ERROR!  No Kinect sensors detected!\n";
        return false;
      }

      // Look at each Kinect sensor
      for (int i = 0; i < iSensorCount; ++i)
      {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr))
        {
          continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
          m_pNuiSensor = pNuiSensor;
          break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
      }

      // Create an event that will be signaled when depth data is available
      m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
      m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
      m_hNextSkeletonEvent   = CreateEvent(NULL, TRUE, FALSE, NULL);

      if (NULL != m_pNuiSensor)
      {
        DWORD nuiFlags = NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON |  NUI_INITIALIZE_FLAG_USES_COLOR;

        // Initialize the Kinect
        hr = m_pNuiSensor->NuiInitialize(nuiFlags); 
        if ( E_NUI_SKELETAL_ENGINE_BUSY == hr )
        {
          nuiFlags = NUI_INITIALIZE_FLAG_USES_DEPTH |  NUI_INITIALIZE_FLAG_USES_COLOR;
          hr = m_pNuiSensor->NuiInitialize( nuiFlags ) ;
        }

        if ( HasSkeletalEngine( m_pNuiSensor ) )
        {
          hr = m_pNuiSensor->NuiSkeletonTrackingEnable( m_hNextSkeletonEvent, NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE );
          if( FAILED( hr ) )
          {
            return false;
          }
        }

        hr = m_pNuiSensor->NuiImageStreamOpen(
          NUI_IMAGE_TYPE_COLOR,
          NUI_IMAGE_RESOLUTION_640x480,
          0,
          2,
          m_hNextColorFrameEvent,
          &m_pVideoStreamHandle );

        if (FAILED(hr))
        {
          return false;
        }

        hr = m_pNuiSensor->NuiImageStreamOpen(
          HasSkeletalEngine(m_pNuiSensor) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH,
          NUI_IMAGE_RESOLUTION_640x480, //NUI_IMAGE_RESOLUTION_320x240,
          0,
          2,
          m_hNextDepthFrameEvent,
          &m_pDepthStreamHandle );

        if (FAILED(hr))
        {
          return false;
        }
      }

      // Return success
      return true;
    }


    //----------------------------------------------------------------------------------
    bool Kinect::GetDepthImage(cv::Mat& depthImg)
    {
      // First make sure depthImg is the correct dimensions, and if not, fix it:
      if ((depthImg.rows != cDepthHeight) || (depthImg.cols != cDepthWidth) || (depthImg.type() != CV_8UC4))
      {
        depthImg = cv::Mat(cDepthHeight, cDepthWidth, CV_8UC4);
      }

      // Make sure initializations have been done before proceeding!
      if (m_pNuiSensor == NULL)
      {
        std::cerr << "Kinect::GetDepthImage() -- ERROR!  Must initialize Kinect before retrieving the depth image!\n";
        return false;
      }

      // Attempt to get the depth frame
      HRESULT hr;
      NUI_IMAGE_FRAME imageFrame;
      int numAttempts = 0;
      while ( WAIT_OBJECT_0 != WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
      {
        ++numAttempts;
        if (numAttempts > 50000)
        {
          std::cerr << "Kinect::GetDepthImage() -- ERROR!  Tried too many times to wait for next frame event!\n";
        }
        Sleep(5);
        continue;
      }

      hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
      if (FAILED(hr))
      {
        std::cerr << "Kinect::GetDepthImage() -- ERROR!  Could not get image from Kinect!\n";
        return false;
      }

      INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
      NUI_LOCKED_RECT LockedRect;

      // Lock the frame data so the Kinect knows not to modify it while we're reading it
      pTexture->LockRect(0, &LockedRect, NULL, 0);

      // Make sure we've received valid data
      if (LockedRect.Pitch != 0)
      {
        BYTE * rgbrun = m_depthRGBX;
        const USHORT * pBufferRun = (const USHORT *)LockedRect.pBits;

        // end pixel is start + width*height - 1
        const USHORT * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight);

        while ( pBufferRun < pBufferEnd )
        {
          // discard the portion of the depth that contains only the player index
          USHORT depth = NuiDepthPixelToDepth(*pBufferRun);


		  float depthF = static_cast<float>(depth);  // <--- put this after "USHORT depth" is set from NuiDepthPixelToDepth
 		  depthF /= 1300.0f;  // <--- scale from max possible USHORT value to 0-1 range
 		  depthF *= 255.0f;  // <--- scale from 0-1 to 0-255

		  //depthF /= 255.0f;  // <--- instead of the depth%256 part

		  BYTE intensity = static_cast<BYTE>(depthF);  // <--- instead of the depth%256 part

		  //intensity = static_cast<BYTE>(depth % 256);

          // to convert to a byte we're looking at only the lower 8 bits
          // by discarding the most significant rather than least significant data
          // we're preserving detail, although the intensity will "wrap"
          //BYTE intensity = static_cast<BYTE>(depth % 256);

          // Write out blue byte
          *(rgbrun++) = intensity;

          // Write out green byte
          *(rgbrun++) = intensity;

          // Write out red byte
          *(rgbrun++) = intensity;

          // We're outputting BGR, the last byte in the 32 bits is unused so skip it
          // If we were outputting BGRA, we would write alpha here.
          ++rgbrun;

          // Increment our index into the Kinect's depth buffer
          ++pBufferRun;
        }
      }

      // We're done with the texture so unlock it
      pTexture->UnlockRect(0);

      // Release the frame
      m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

      // Copy to cv::Mat image
      memcpy(depthImg.data, m_depthRGBX, m_numBytes);

      // Return success
      return true;
    }

    //----------------------------------------------------------------------------------
    bool Kinect::GetColorImage(cv::Mat& colorImg)
    {
      // First make sure colorImg is the correct dimensions, and if not, fix it:
      if ((colorImg.rows != cDepthHeight) || (colorImg.cols != cDepthWidth) || (colorImg.type() != CV_8UC4))
      {
        colorImg = cv::Mat(cDepthHeight, cDepthWidth, CV_8UC4);
      }

      // Make sure initializations have been done before proceeding!
      if (m_pNuiSensor == NULL)
      {
        std::cerr << "Kinect::GetColorImage() -- ERROR!  Must initialize Kinect before retrieving the color image!\n";
        return false;
      }

      // Attempt to get the color frame
      HRESULT hr;
      NUI_IMAGE_FRAME imageFrame;
      int numAttempts = 0;
      while ( WAIT_OBJECT_0 != WaitForSingleObject(m_hNextColorFrameEvent, 0) )
      {
        ++numAttempts;
        if (numAttempts > 50000)
        {
          std::cerr << "Kinect::GetColorImage() -- ERROR!  Tried too many times to wait for next frame event!\n";
        }
        Sleep(5);
        continue;
      }

      hr = m_pNuiSensor->NuiImageStreamGetNextFrame( m_pVideoStreamHandle, 0, &imageFrame );
      if ( FAILED( hr ) )
      {
        std::cerr << "Kinect::GetColorImage() -- ERROR!  Could not get image from Kinect!\n";
        return false;
      }

	  //////////////////////////////////////////////////////////////////////////

	  m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
		  cColorResolution,
		  cDepthResolution,
		  m_depthWidth*m_depthHeight,
		  m_depthD16,
		  m_depthWidth*m_depthHeight*2,
		  m_colorCoordinates
		  );

	  /*
	  // copy to our d3d 11 color texture
	  D3D11_MAPPED_SUBRESOURCE msT;
	  //hr = m_pImmediateContext->Map(m_pColorTexture2D, NULL, D3D11_MAP_WRITE_DISCARD, NULL, &msT);
	  if ( FAILED(hr) ) { return hr; }

	  // loop over each row and column of the color
	  for (LONG y = 0; y < m_colorHeight; ++y)
	  {
		  LONG* pDest = (LONG*)((BYTE*)msT.pData + msT.RowPitch * y);
		  for (LONG x = 0; x < m_colorWidth; ++x)
		  {
			  // calculate index into depth array
			  int depthIndex = x/m_colorToDepthDivisor + y/m_colorToDepthDivisor * m_depthWidth;

			  // retrieve the depth to color mapping for the current depth pixel
			  LONG colorInDepthX = m_colorCoordinates[depthIndex * 2];
			  LONG colorInDepthY = m_colorCoordinates[depthIndex * 2 + 1];

			  // make sure the depth pixel maps to a valid point in color space
			  if ( colorInDepthX >= 0 && colorInDepthX < m_colorWidth && colorInDepthY >= 0 && colorInDepthY < m_colorHeight )
			  {
				  // calculate index into color array
				  LONG colorIndex = colorInDepthX + colorInDepthY * m_colorWidth;

				  // set source for copy to the color pixel
				  LONG* pSrc = (LONG *)m_colorRGBX + colorIndex;
				  *pDest = *pSrc;
			  }
			  else
			  {
				  *pDest = 0;
			  }

			  pDest++;
		  }
	  }

*/	  

	  //////////////////////////////////////////////////////////////////////////
      INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
      NUI_LOCKED_RECT LockedRect;
      pTexture->LockRect( 0, &LockedRect, NULL, 0 );
      if ( LockedRect.Pitch != 0 )
      {
        memcpy(colorImg.data, LockedRect.pBits, colorImg.rows*colorImg.cols*colorImg.channels());
      }

      pTexture->UnlockRect( 0 );
      m_pNuiSensor->NuiImageStreamReleaseFrame( m_pVideoStreamHandle, &imageFrame );

      // Return success
      return true;
    }

	LONG* Kinect::GetColorCoordinates()
	{
		return m_colorCoordinates;
	}

    //----------------------------------------------------------------------------------
    bool Kinect::GetSkeletalImage(cv::Mat& skeletalImg, bool* foundSkeleton)
    {
      // First make sure skeletalImg is the correct dimensions, and if not, fix it:
      if ((skeletalImg.rows != cDepthHeight) || (skeletalImg.cols != cDepthWidth) || (skeletalImg.type() != CV_8UC3))
      {
        skeletalImg = cv::Mat(cDepthHeight, cDepthWidth, CV_8UC3);
      }
      skeletalImg.setTo(cv::Scalar(0,0,0,0));

      // Make sure initializations have been done before proceeding!
      if (m_pNuiSensor == NULL)
      {
        std::cerr << "Kinect::GetSkeletalImage() -- ERROR!  Must initialize Kinect before retrieving the skeletal image!\n";
        return false;
      }

      // Attempt to get the skeleton frame
      HRESULT hr;
      NUI_SKELETON_FRAME SkeletonFrame = {0};
      int numAttempts = 0;
      while ( WAIT_OBJECT_0 != WaitForSingleObject(m_hNextSkeletonEvent, 0) )
      {
        ++numAttempts;
        if (numAttempts > 50000)
        {
          std::cerr << "Kinect::GetSkeletalImage() -- ERROR!  Tried too many times to wait for next frame event!\n";
        }
        Sleep(5);
        continue;
      }

      *foundSkeleton = false;
      if ( SUCCEEDED(m_pNuiSensor->NuiSkeletonGetNextFrame( 0, &SkeletonFrame )) )
      {
        for ( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
        {
          NUI_SKELETON_TRACKING_STATE trackingState = SkeletonFrame.SkeletonData[i].eTrackingState;

          if ( trackingState == NUI_SKELETON_TRACKED || trackingState == NUI_SKELETON_POSITION_ONLY )
          {
            *foundSkeleton = true;
          }
        }
      }

      // smooth out the skeleton data
      hr = m_pNuiSensor->NuiTransformSmooth(&SkeletonFrame,NULL);
      if ( FAILED(hr) )
      {
        return false;
      }

      for ( int i = 0 ; i < NUI_SKELETON_COUNT; i++ )
      {
        NUI_SKELETON_TRACKING_STATE trackingState = SkeletonFrame.SkeletonData[i].eTrackingState;

        // Save time stamp
        mJoints.timestamp = static_cast<double>(SkeletonFrame.liTimeStamp.QuadPart);

        if ( trackingState == NUI_SKELETON_TRACKED )
        {
          // We're tracking the skeleton, draw it
          Nui_DrawSkeleton( SkeletonFrame.SkeletonData[i], skeletalImg.cols, skeletalImg.rows, skeletalImg );
        }
        else if ( trackingState == NUI_SKELETON_POSITION_ONLY )
        {
          // we've only received the center point of the skeleton, draw that
          cv::circle(skeletalImg, 
            SkeletonToScreen( SkeletonFrame.SkeletonData[i].Position, skeletalImg.cols, skeletalImg.rows ), 
            g_JointThickness, cv::Scalar( 68, 192, 68 ), 1 );
        }
      }

      UpdateTrackedSkeletons( SkeletonFrame );

      // Return success
      return true;
    }

    //----------------------------------------------------------------------------------
    void Kinect::Nui_DrawSkeleton( const NUI_SKELETON_DATA & skel, int windowWidth, int windowHeight, cv::Mat& canvas )
    {
      int i;

      for (i = 0; i < NUI_SKELETON_POSITION_COUNT; i++)
      {
        m_Points[i] = SkeletonToScreen( skel.SkeletonPositions[i], windowWidth, windowHeight );
      }

      // Save 3D positions of joints
      mJoints.head           = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_HEAD]);
      mJoints.shoulderCenter = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER]);
      mJoints.shoulderLeft   = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT]);
      mJoints.shoulderRight  = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT]);
      mJoints.spine          = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_SPINE]);
      mJoints.hipCenter      = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_HIP_CENTER]);
      mJoints.hipLeft        = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_HIP_LEFT]);
      mJoints.hipRight       = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_HIP_RIGHT]);
      mJoints.elbowLeft      = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_LEFT]);
      mJoints.elbowRight     = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT]);
      mJoints.wristLeft      = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_LEFT]);
      mJoints.wristRight     = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT]);
      mJoints.handLeft       = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT]);
      mJoints.handRight      = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT]);
      mJoints.kneeLeft       = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_LEFT]);
      mJoints.kneeRight      = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_RIGHT]);
      mJoints.ankleLeft      = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_LEFT]);
      mJoints.ankleRight     = ConvertJoint(skel.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_RIGHT]);

			// Also save the corresponding 2D positions of the joints

      // Render Torso
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT, canvas );

      // Left Arm
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT, canvas );

      // Right Arm
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT, canvas );

      // Left Leg
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT, canvas );

      // Right Leg
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, canvas );
      Nui_DrawBone( skel, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT, canvas );

      // Draw the joints in a different color
      for ( i = 0; i < NUI_SKELETON_POSITION_COUNT; i++ )
      {
        if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_INFERRED )
        {
          cv::circle(canvas, m_Points[i], g_JointThickness, cv::Scalar(0, 255, 255 ), 1); 
        }
        else if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED )
        {
          cv::circle(canvas, m_Points[i], g_JointThickness, cv::Scalar( 68, 192, 68 ), 1);
        }
      }
    }

    //----------------------------------------------------------------------------------
    void Kinect::Nui_DrawBone( const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX bone0, NUI_SKELETON_POSITION_INDEX bone1, cv::Mat& canvas )
    {
      NUI_SKELETON_POSITION_TRACKING_STATE bone0State = skel.eSkeletonPositionTrackingState[bone0];
      NUI_SKELETON_POSITION_TRACKING_STATE bone1State = skel.eSkeletonPositionTrackingState[bone1];

      // If we can't find either of these joints, exit
      if ( bone0State == NUI_SKELETON_POSITION_NOT_TRACKED || bone1State == NUI_SKELETON_POSITION_NOT_TRACKED )
      {
        return;
      }

      // Don't draw if both points are inferred
      if ( bone0State == NUI_SKELETON_POSITION_INFERRED && bone1State == NUI_SKELETON_POSITION_INFERRED )
      {
        return;
      }

      // We assume all drawn bones are inferred unless BOTH joints are tracked
      if ( bone0State == NUI_SKELETON_POSITION_TRACKED && bone1State == NUI_SKELETON_POSITION_TRACKED )
      {
        cv::line(canvas, m_Points[bone0], m_Points[bone1], cv::Scalar(0, 128, 0), g_TrackedBoneThickness);
      }
      else
      {
        cv::line(canvas, m_Points[bone0], m_Points[bone1], cv::Scalar( 128, 128, 128 ), g_InferredBoneThickness);
      }
    }

    //----------------------------------------------------------------------------------
    cv::Point2f Kinect::SkeletonToScreen( Vector4 skeletonPoint, int width, int height )
    {
      LONG x, y;
      USHORT depth;

      // calculate the skeleton's position on the screen
      // NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
      NuiTransformSkeletonToDepthImage( skeletonPoint, &x, &y, &depth );

      float screenPointX = static_cast<float>(x * width) / (float)320;
      float screenPointY = static_cast<float>(y * height) / (float)240;

      return cv::Point2f(screenPointX, screenPointY);
    }

    //----------------------------------------------------------------------------------
    void Kinect::UpdateTrackedSkeletons( const NUI_SKELETON_FRAME & skel )
    {
      DWORD nearestIDs[2] = { 0, 0 };
      USHORT nearestDepths[2] = { NUI_IMAGE_DEPTH_MAXIMUM, NUI_IMAGE_DEPTH_MAXIMUM };

      // Purge old sticky skeleton IDs, if the user has left the frame, etc
      bool stickyID0Found = false;
      bool stickyID1Found = false;
      for ( int i = 0 ; i < NUI_SKELETON_COUNT; i++ )
      {
        NUI_SKELETON_TRACKING_STATE trackingState = skel.SkeletonData[i].eTrackingState;

        if ( trackingState == NUI_SKELETON_TRACKED || trackingState == NUI_SKELETON_POSITION_ONLY )
        {
          if ( skel.SkeletonData[i].dwTrackingID == m_StickySkeletonIds[0] )
          {
            stickyID0Found = true;
          }
          else if ( skel.SkeletonData[i].dwTrackingID == m_StickySkeletonIds[1] )
          {
            stickyID1Found = true;
          }
        }
      }

      if ( !stickyID0Found && stickyID1Found )
      {
        m_StickySkeletonIds[0] = m_StickySkeletonIds[1];
        m_StickySkeletonIds[1] = 0;
      }
      else if ( !stickyID0Found )
      {
        m_StickySkeletonIds[0] = 0;
      }
      else if ( !stickyID1Found )
      {
        m_StickySkeletonIds[1] = 0;
      }

      // Calculate nearest and sticky skeletons
      for ( int i = 0 ; i < NUI_SKELETON_COUNT; i++ )
      {
        NUI_SKELETON_TRACKING_STATE trackingState = skel.SkeletonData[i].eTrackingState;

        if ( trackingState == NUI_SKELETON_TRACKED || trackingState == NUI_SKELETON_POSITION_ONLY )
        {
          // Save SkeletonIds for sticky mode if there's none already saved
          if ( 0 == m_StickySkeletonIds[0] && m_StickySkeletonIds[1] != skel.SkeletonData[i].dwTrackingID )
          {
            m_StickySkeletonIds[0] = skel.SkeletonData[i].dwTrackingID;
          }
          else if ( 0 == m_StickySkeletonIds[1] && m_StickySkeletonIds[0] != skel.SkeletonData[i].dwTrackingID )
          {
            m_StickySkeletonIds[1] = skel.SkeletonData[i].dwTrackingID;
          }

          LONG x, y;
          USHORT depth;

          // calculate the skeleton's position on the screen
          NuiTransformSkeletonToDepthImage( skel.SkeletonData[i].Position, &x, &y, &depth );

          if ( depth < nearestDepths[0] )
          {
            nearestDepths[1] = nearestDepths[0];
            nearestIDs[1] = nearestIDs[0];

            nearestDepths[0] = depth;
            nearestIDs[0] = skel.SkeletonData[i].dwTrackingID;
          }
          else if ( depth < nearestDepths[1] )
          {
            nearestDepths[1] = depth;
            nearestIDs[1] = skel.SkeletonData[i].dwTrackingID;
          }
        }
      }

      if ( SV_TRACKED_SKELETONS_NEAREST1 == m_TrackedSkeletons || SV_TRACKED_SKELETONS_NEAREST2 == m_TrackedSkeletons )
      {
        // Only track the closest single skeleton in nearest 1 mode
        if ( SV_TRACKED_SKELETONS_NEAREST1 == m_TrackedSkeletons )
        {
          nearestIDs[1] = 0;
        }
        m_pNuiSensor->NuiSkeletonSetTrackedSkeletons(nearestIDs);
      }

      if ( SV_TRACKED_SKELETONS_STICKY1 == m_TrackedSkeletons || SV_TRACKED_SKELETONS_STICKY2 == m_TrackedSkeletons )
      {
        DWORD stickyIDs[2] = { m_StickySkeletonIds[0], m_StickySkeletonIds[1] };

        // Only track a single skeleton in sticky 1 mode
        if ( SV_TRACKED_SKELETONS_STICKY1 == m_TrackedSkeletons )
        {
          stickyIDs[1] = 0;
        }
        m_pNuiSensor->NuiSkeletonSetTrackedSkeletons(stickyIDs);
      }
    }

    //----------------------------------------------------------------------------------
    cv::Point3f Kinect::ConvertJoint(const Vector4& vec) const
    {
      return cv::Point3f(vec.x, vec.y, vec.z);
    }

    //----------------------------------------------------------------------------------
    void Kinect::SaveSkeletalJoints(std::ofstream& ofs) const
    {
			// Write out 3D position followed by 2d-projection of joint positon
      ofs << mJoints.head.x           << " " << mJoints.head.y           << " " << mJoints.head.z           << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_HEAD].x << " " << m_Points[NUI_SKELETON_POSITION_HEAD].y << std::endl;
      ofs << mJoints.shoulderCenter.x << " " << mJoints.shoulderCenter.y << " " << mJoints.shoulderCenter.z << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_SHOULDER_CENTER].x << " " << m_Points[NUI_SKELETON_POSITION_SHOULDER_CENTER].y << std::endl;
      ofs << mJoints.shoulderLeft.x   << " " << mJoints.shoulderLeft.y   << " " << mJoints.shoulderLeft.z   << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_SHOULDER_LEFT].x << " " << m_Points[NUI_SKELETON_POSITION_SHOULDER_LEFT].y << std::endl;
      ofs << mJoints.shoulderRight.x  << " " << mJoints.shoulderRight.y  << " " << mJoints.shoulderRight.z  << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x << " " << m_Points[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y << std::endl;
      ofs << mJoints.spine.x          << " " << mJoints.spine.y          << " " << mJoints.spine.z          << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_SPINE].x << " " << m_Points[NUI_SKELETON_POSITION_SPINE].y << std::endl;
      ofs << mJoints.hipCenter.x      << " " << mJoints.hipCenter.y      << " " << mJoints.hipCenter.z      << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_HIP_CENTER].x << " " << m_Points[NUI_SKELETON_POSITION_HIP_CENTER].y << std::endl;
      ofs << mJoints.hipLeft.x        << " " << mJoints.hipLeft.y        << " " << mJoints.hipLeft.z        << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_HIP_LEFT].x << " " << m_Points[NUI_SKELETON_POSITION_HIP_LEFT].y << std::endl;
      ofs << mJoints.hipRight.x       << " " << mJoints.hipRight.y       << " " << mJoints.hipRight.z       << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_HIP_RIGHT].x << " " << m_Points[NUI_SKELETON_POSITION_HIP_RIGHT].y << std::endl;
      ofs << mJoints.elbowLeft.x      << " " << mJoints.elbowLeft.y      << " " << mJoints.elbowLeft.z      << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_ELBOW_LEFT].x << " " << m_Points[NUI_SKELETON_POSITION_ELBOW_LEFT].y << std::endl;
      ofs << mJoints.elbowRight.x     << " " << mJoints.elbowRight.y     << " " << mJoints.elbowRight.z     << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_ELBOW_RIGHT].x << " " << m_Points[NUI_SKELETON_POSITION_ELBOW_RIGHT].y << std::endl;
      ofs << mJoints.wristLeft.x      << " " << mJoints.wristLeft.y      << " " << mJoints.wristLeft.z      << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_WRIST_LEFT].x << " " << m_Points[NUI_SKELETON_POSITION_WRIST_LEFT].y << std::endl;
      ofs << mJoints.wristRight.x     << " " << mJoints.wristRight.y     << " " << mJoints.wristRight.z     << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_WRIST_RIGHT].x << " " << m_Points[NUI_SKELETON_POSITION_WRIST_RIGHT].y << std::endl;
      ofs << mJoints.handLeft.x       << " " << mJoints.handLeft.y       << " " << mJoints.handLeft.z       << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_HAND_LEFT].x << " " << m_Points[NUI_SKELETON_POSITION_HAND_LEFT].y << std::endl;
      ofs << mJoints.handRight.x      << " " << mJoints.handRight.y      << " " << mJoints.handRight.z      << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_HAND_RIGHT].x << " " << m_Points[NUI_SKELETON_POSITION_HAND_RIGHT].y << std::endl;
      ofs << mJoints.kneeLeft.x       << " " << mJoints.kneeLeft.y       << " " << mJoints.kneeLeft.z       << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_KNEE_LEFT].x << " " << m_Points[NUI_SKELETON_POSITION_KNEE_LEFT].y << std::endl;
      ofs << mJoints.kneeRight.x      << " " << mJoints.kneeRight.y      << " " << mJoints.kneeRight.z      << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_KNEE_RIGHT].x << " " << m_Points[NUI_SKELETON_POSITION_KNEE_RIGHT].y << std::endl;
      ofs << mJoints.ankleLeft.x      << " " << mJoints.ankleLeft.y      << " " << mJoints.ankleLeft.z      << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_ANKLE_LEFT].x << " " << m_Points[NUI_SKELETON_POSITION_ANKLE_LEFT].y << std::endl;
      ofs << mJoints.ankleRight.x     << " " << mJoints.ankleRight.y     << " " << mJoints.ankleRight.z     << " ";
			ofs << m_Points[NUI_SKELETON_POSITION_ANKLE_RIGHT].x << " " << m_Points[NUI_SKELETON_POSITION_ANKLE_RIGHT].y << std::endl;
    }

    //----------------------------------------------------------------------------------
    void Kinect::Cleanup()
    {
      if (m_depthRGBX != NULL)
      {
        // Delete image data
        delete [] m_depthRGBX;
        m_depthRGBX = NULL;
      }

      // Shutdown the kinect
      if (m_pNuiSensor)
      {
        m_pNuiSensor->NuiShutdown();

        if ( m_hNextSkeletonEvent && ( m_hNextSkeletonEvent != INVALID_HANDLE_VALUE ) )
        {
          CloseHandle( m_hNextSkeletonEvent );
          m_hNextSkeletonEvent = NULL;
        }
        if ( m_hNextDepthFrameEvent && ( m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE ) )
        {
          CloseHandle( m_hNextDepthFrameEvent );
          m_hNextDepthFrameEvent = NULL;
        }
        if ( m_hNextColorFrameEvent && ( m_hNextColorFrameEvent != INVALID_HANDLE_VALUE ) )
        {
          CloseHandle( m_hNextColorFrameEvent );
          m_hNextColorFrameEvent = NULL;
        }

        m_pNuiSensor->Release();
        m_pNuiSensor = NULL;

		delete[] m_depthD16;
      }
    }
  }
}
