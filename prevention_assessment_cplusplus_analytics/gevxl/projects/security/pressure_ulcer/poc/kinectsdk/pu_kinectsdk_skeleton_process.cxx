// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.


/// \file 
/// \author Yi (Jessica) Yao
/// \date 6/5/2012
/// \par Modifications: 
/// - Original versioin



#include <vcl_utility.h>
#include <vil/vil_image_view.h>
#include <vnl/vnl_double_2.h>
//#include <vul/vul_sprintf.h>

#include <util/time/global_time.h>
//#include <util/time/highres_time_manager.h>
#include <util/string.h>
//#include <img/connected_components_scanline.h>
//#include <img/filters.h>
//#include <img/histogram_1d.h>

//#include <macros/print_macros.h>


#include "person_skeleton_process.h"

using namespace gesec::smartroom;


/// Default constructor
person_skeleton_process::person_skeleton_process(const char *name) : 
 //gevxl::detectors::gp_3d_detector(name), 
 gevxl::vid::frame_process<vxl_byte>(name),
 visualize_(false),
 enabled_(false)
{
}

/// Default destructor
person_skeleton_process::~person_skeleton_process(void)
{
}


bool person_skeleton_process::configure(gevxl::util::config_file& config )
{
  //if(!gevxl::detectors::gp_3d_detector::configure(config)) return false;

  //config_=config;


  sample_xml_path_="";
  config.get_string(name()+"::sample_xml_path",sample_xml_path_);

  visualize_=false;
  config.get_bool(name()+"::visualize",visualize_);

  enabled_=false;
  config.get_bool(name()+"::enabled",enabled_);
  
  return true;
}


//initialize
bool person_skeleton_process::initialize(void)
{ 
  if (!enabled_) return true;
    //NuiStatusProc temp;
    //void * temp2;
    //NuiSetDeviceStatusCallback(temp, temp2 );

    INuiSensor * pNuiSensor;

    int iSensorCount = 0;
    HRESULT hr = NuiGetSensorCount(&iSensorCount);
    if (hr<0)
    {
        return hr;
    }

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (hr<0)
        {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (hr == 0)
        {
            pNuiSensor_ = pNuiSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }

    if (NULL != pNuiSensor_)
    {
        // Initialize the Kinect and specify that we'll be using skeleton
        hr = pNuiSensor_->NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON); 
        if (SUCCEEDED(hr))
        {
            // Create an event that will be signaled when skeleton data is available
            hNextSkeletonEvent_ = CreateEventW(NULL, TRUE, FALSE, NULL);

            // Open a skeleton stream to receive skeleton data
            hr = pNuiSensor_->NuiSkeletonTrackingEnable(hNextSkeletonEvent_, 0); 

            points_.resize(NUI_SKELETON_POSITION_COUNT);
        }
    }

    if (NULL == pNuiSensor_ || hr<0)
    {
        vcl_cerr << "No ready Kinect found!\n";
        return false;
    }

  //depth_generator_.GetMetaData(depth_metadata_);
  //frame_width_=depth_metadata_.XRes();
  //frame_height_=depth_metadata_.YRes();
  //frame_ = vil_image_view<vxl_byte>(frame_width_, frame_height_,1,3);
  ////frame_.set_size(frame_width_, frame_height_);
  return true;
}

void person_skeleton_process::uninitialize(void)
{
  if (!enabled_) return;
    if (pNuiSensor_)
    {
        pNuiSensor_->NuiShutdown();
    }

    if (hNextSkeletonEvent_ && (hNextSkeletonEvent_ != INVALID_HANDLE_VALUE))
    {
        CloseHandle(hNextSkeletonEvent_);
    }


    pNuiSensor_->Release();
    pNuiSensor_=NULL;
  //gevxl::detectors::gp_3d_detector::uninitialize();
  frame_.clear();
}

bool person_skeleton_process::step(void)
{
  //if(!gevxl::detectors::gp_3d_detector::step()) return false;
  if(!enabled_) return true;

  if (NULL == pNuiSensor_) return false;


  // Wait for 0ms, just quickly test if it is time to process a skeleton
  if ( WAIT_OBJECT_0 == WaitForSingleObject(hNextSkeletonEvent_, 0) )
  {
      process_skeleton();
  }



  //visuualize results
  visualize(viz_);

  //vil_save(*frame_,"c:/dump/test.png");

  return true;
}

void person_skeleton_process::visualize(gevxl::img::visualizer_2d *viz)
{
  if (!enabled_ || !visualize_)
    return;

  viz_->lock();
  viz_->set_image( frame_ );
  viz_->unlock();
  //vil_save(*frame_,"c:/dump/test.png");

  return;


}

/// <summary>
/// Handle new skeleton data
/// </summary>
void person_skeleton_process::process_skeleton(void)
{
    NUI_SKELETON_FRAME skeletonFrame = {0};

    HRESULT hr = pNuiSensor_->NuiSkeletonGetNextFrame(0, &skeletonFrame);
    if ( FAILED(hr) )
    {
        return;
    }

    // smooth out the skeleton data
    pNuiSensor_->NuiTransformSmooth(&skeletonFrame, NULL);
    
    viz_->lock();

    for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i)
    {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            // We're tracking the skeleton, draw it
            draw_skeleton(skeletonFrame.SkeletonData[i], 2, 2);
            smoker_detection(skeletonFrame.SkeletonData[i]);
        }
        else if (NUI_SKELETON_POSITION_ONLY == trackingState)
        {
            // we've only received the center point of the skeleton, draw that
          vnl_double_2 point = skeleton_to_screen(skeletonFrame.SkeletonData[i].Position,2, 2);
          viz_->add_filled_circle(point(0), point(1), 3);
            //D2D1_ELLIPSE ellipse = D2D1::Ellipse(
            //    SkeletonToScreen(skeletonFrame.SkeletonData[i].Position, width, height),
            //    g_JointThickness,
            //    g_JointThickness
            //    );

        //    m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointTracked);
        }
    }


    viz_->unlock();

    //hr = m_pRenderTarget->EndDraw();

    //// Device lost, need to recreate the render target
    //// We'll dispose it now and retry drawing
    //if (D2DERR_RECREATE_TARGET == hr)
    //{
    //    hr = S_OK;
    //    DiscardDirect2DResources();
    //}
}
void person_skeleton_process::smoker_detection(const NUI_SKELETON_DATA & skel)
{
  Vector4 head_pos = skel.SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
  Vector4 neck_pos = skel.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER];
  Vector4 left_hand_pos = skel.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_LEFT];
  Vector4 right_hand_pos = skel.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT];
  Vector4 mouth_pos; 
  mouth_pos.x=(head_pos.x+3*neck_pos.x)/4;
  mouth_pos.y=(head_pos.y+3*neck_pos.y)/4;
  mouth_pos.z=(head_pos.z+3*neck_pos.z)/4;
  float distx=left_hand_pos.x - mouth_pos.x;
  float disty=left_hand_pos.y - mouth_pos.y;
  float distz=left_hand_pos.z - mouth_pos.z;

  float dist1=sqrt(distx*distx+disty*disty+distz*distz);

  vcl_cerr << "left hand to head dist: " << dist1 << vcl_endl;

  distx=right_hand_pos.x - mouth_pos.x;
  disty=right_hand_pos.y - mouth_pos.y;
  distz=right_hand_pos.z - mouth_pos.z;

  float dist2=sqrt(distx*distx+disty*disty+distz*distz);

  vcl_cerr << "left hand to head dist: " << dist2 << vcl_endl;

  if (dist1<0.3 || dist2<0.3)
  {
    viz_->lock();
    viz_->add_text(0,0,"Smoking Event Detected!");
    viz_->unlock();
  }

}

void person_skeleton_process::draw_skeleton(const NUI_SKELETON_DATA & skel, int windowWidth, int windowHeight)
{      
    int i;

    for (i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
    {
        points_[i] = skeleton_to_screen(skel.SkeletonPositions[i], windowWidth, windowHeight);
    }

    // Render Torso
    draw_bone(skel, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);
    draw_bone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
    draw_bone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
    draw_bone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
    draw_bone(skel, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);
    draw_bone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT);
    draw_bone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT);

    // Left Arm
    draw_bone(skel, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
    draw_bone(skel, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
    draw_bone(skel, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);

    // Right Arm
    draw_bone(skel, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
    draw_bone(skel, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
    draw_bone(skel, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);

    // Left Leg
    draw_bone(skel, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT);
    draw_bone(skel, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
    draw_bone(skel, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);

    // Right Leg
    draw_bone(skel, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT);
    draw_bone(skel, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
    draw_bone(skel, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);
    
    // Draw the joints in a different color
    for (i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
    {
        //D2D1_ELLIPSE ellipse = D2D1::Ellipse( m_Points[i], g_JointThickness, g_JointThickness );

        if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_INFERRED )
        {
          viz_->add_circle(points_[i](0), points_[i](1), 2);
            
            //add_circle(points_[i]);
            //m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointInferred);
        }
        else if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED )
        {
          viz_->add_circle(points_[i](0), points_[i](1), 4);
            //m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointTracked);
        }
    }
}


vnl_double_2 person_skeleton_process::skeleton_to_screen(Vector4 skeletonPoint, int width, int height)
{
    LONG x, y;
    USHORT depth;

    // Calculate the skeleton's position on the screen
    // NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
    NuiTransformSkeletonToDepthImage(skeletonPoint, &x, &y, &depth);

    float screenPointX = static_cast<float>(x * width);
    float screenPointY = static_cast<float>(y * height);

    return vnl_double_2(screenPointX, screenPointY);
}


void person_skeleton_process::draw_bone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0, NUI_SKELETON_POSITION_INDEX joint1)
{
    NUI_SKELETON_POSITION_TRACKING_STATE joint0State = skel.eSkeletonPositionTrackingState[joint0];
    NUI_SKELETON_POSITION_TRACKING_STATE joint1State = skel.eSkeletonPositionTrackingState[joint1];

    // If we can't find either of these joints, exit
    if (joint0State == NUI_SKELETON_POSITION_NOT_TRACKED || joint1State == NUI_SKELETON_POSITION_NOT_TRACKED)
    {
        return;
    }
    
    // Don't draw if both points are inferred
    if (joint0State == NUI_SKELETON_POSITION_INFERRED && joint1State == NUI_SKELETON_POSITION_INFERRED)
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if (joint0State == NUI_SKELETON_POSITION_TRACKED && joint1State == NUI_SKELETON_POSITION_TRACKED)
    {
      viz_->set_line_width(4);
      viz_->add_line(points_[joint0], points_[joint1]);

        //m_pRenderTarget->DrawLine(points_[joint0], points_[joint1], m_pBrushBoneTracked, g_TrackedBoneThickness);
    }
    else
    {
       viz_->set_line_width(2);
       viz_->add_line(points_[joint0], points_[joint1]);
       //m_pRenderTarget->DrawLine(points_[joint0], points_[joint1], m_pBrushBoneInferred, g_InferredBoneThickness);
    }
}