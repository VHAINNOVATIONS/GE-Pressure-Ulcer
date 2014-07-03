// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.


/// \file 
/// \author Yi (Jessica) Yao
/// \date 6/8/2012
/// \par Modifications: 
/// - Original versioin



#include <vcl_utility.h>
#include <vil/vil_image_view.h>
#include <vil/vil_plane.h>
#include <vil/vil_save.h>
#include <vnl/vnl_double_2.h>
//#include <vul/vul_sprintf.h>

#include <util/time/global_time.h>
//#include <util/time/highres_time_manager.h>
#include <util/string.h>
//#include <img/connected_components_scanline.h>
//#include <img/filters.h>
//#include <img/histogram_1d.h>

//#include <macros/print_macros.h>

#include <img/integral_image.h>

#include "person_face_process.h"

using namespace gesec::smartroom;

DWORD s_ColorCode[] = {0x00FFFF00, 0x00FF0000,  0x0000FF00, 0x0000FFFF, 0x00FF00FF, 0x000000FF};
/// Default constructor
person_face_process::person_face_process(const char *name) : 
 //gevxl::detectors::gp_3d_detector(name), 
 gevxl::vid::frame_process<vxl_byte>(name),
 visualize_(false),
 enabled_(false)
{
    user_context_ = 0;
    color_image_ = NULL;
    depth_image_ = NULL;
    depth_type_ = NUI_IMAGE_TYPE_DEPTH;
    depth_res_ = NUI_IMAGE_RESOLUTION_320x240;
    near_mode_flag_ = false;
    color_type_ = NUI_IMAGE_TYPE_COLOR;
    color_res_ = NUI_IMAGE_RESOLUTION_640x480;
	  seated_skeleton_flag_ = false;
    draw_mask_=true;
    frame_set_=false;
    num_users_=1;

    disguise_flag_=false;

    mouth_indx_.resize(33);
    mouth_indx_[0]=6;mouth_indx_[1]=7;mouth_indx_[2]=8;mouth_indx_[3]=9;mouth_indx_[4]=10;
    mouth_indx_[5]=26;mouth_indx_[6]=27;mouth_indx_[7]=32;mouth_indx_[8]=34;mouth_indx_[9]=39;
    mouth_indx_[10]=40;mouth_indx_[11]=41;mouth_indx_[12]=42;mouth_indx_[13]=43;mouth_indx_[14]=59;
    mouth_indx_[15]=60;mouth_indx_[16]=65;mouth_indx_[17]=67;mouth_indx_[18]=76;mouth_indx_[19]=77;
    mouth_indx_[20]=80;mouth_indx_[21]=81;mouth_indx_[22]=82;mouth_indx_[23]=83;mouth_indx_[24]=84;
    mouth_indx_[25]=85;mouth_indx_[26]=86;mouth_indx_[27]=87;mouth_indx_[28]=88;mouth_indx_[29]=89;
    mouth_indx_[30]=90;mouth_indx_[31]=112;mouth_indx_[32]=113;

}

/// Default destructor
person_face_process::~person_face_process(void)
{
}


bool person_face_process::configure(gevxl::util::config_file& config )
{
  //if(!gevxl::detectors::gp_3d_detector::configure(config)) return false;

  //config_=config;


  if (!config.get_string(name()+"::sample_xml_path",sample_xml_path_))
    sample_xml_path_="";

  if (!config.get_bool(name()+"::visualize",visualize_))
    visualize_=false;

  if (!config.get_bool(name()+"::enabled",enabled_))
    enabled_=false;

  if (!config.get_bool(name()+"::near_mode_flag", near_mode_flag_))
    near_mode_flag_=false;

  if (!config.get_bool(name()+"::seated_skeleton_flag", seated_skeleton_flag_))
    seated_skeleton_flag_=false;

  double temp;
  if (config.get_double(name()+"::detection_distance_threshold",temp))
    kinect_sensor_.set_detection_distance_threshold(temp);

  if (!config.get_bool(name()+"::disguise_flag",disguise_flag_))
    disguise_flag_=false;
    

  return true;
}


//initialize
bool person_face_process::initialize(void)
{ 
    FT_CAMERA_CONFIG video_config;
    FT_CAMERA_CONFIG depth_config;
    FT_CAMERA_CONFIG* depth_config_ptr = NULL;


    INuiSensor * pNuiSensor;

    int iSensorCount = 0;
    HRESULT hr = NuiGetSensorCount(&iSensorCount);
    if (hr<0)
    {
        return hr;
    }

    // Look at each Kinect sensor
    int i=0;
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (hr<0)
        //{
        //    continue;
        //}

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (hr == 0)
        {
          kinect_sensor_.set_sensor(pNuiSensor);
          //break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        else
          pNuiSensor->Release();
    }

    // Try to get the Kinect camera to work
    hr = kinect_sensor_.Init(depth_type_, depth_res_, near_mode_flag_, FALSE, 
                                     color_type_, color_res_, seated_skeleton_flag_);
    if (SUCCEEDED(hr))
    {
        kinect_sensor_flag_ = TRUE;
        kinect_sensor_.GetVideoConfiguration(&video_config);
        kinect_sensor_.GetDepthConfiguration(&depth_config);
        depth_config_ptr = &depth_config;
    }
    else
    {
        kinect_sensor_flag_ = FALSE;
        vcl_cerr << "Could not initialize the Kinect sensor.\n";
        return false;
    }
    
    user_context_ = new FTHelperContext[num_users_];
    if (user_context_ != 0)
    {
        memset(user_context_, 0, sizeof(FTHelperContext)*num_users_);
    }
    else
    {
      vcl_cerr << "Could not allocate user context array.\n";
      return false;
    }

    for (unsigned i=0; i<num_users_;i++)
    {
        // Try to start the face tracker.
        user_context_[i].m_pFaceTracker = FTCreateFaceTracker(NULL);
        if (!user_context_[i].m_pFaceTracker)
        {
          vcl_cerr << "Could not create the face tracker.\n";
          return false;
        }

        hr = user_context_[i].m_pFaceTracker->Initialize(&video_config, depth_config_ptr, NULL, NULL); 
        if (FAILED(hr))
        {
          vcl_cerr << "Could not initialize face tracker for user " << i << ".\n";
          return false;
        }
        user_context_[i].m_pFaceTracker->CreateFTResult(&user_context_[i].m_pFTResult);
        if (!user_context_[i].m_pFTResult)
        {
          vcl_cerr << "Could not initialize the face tracker result for " << i << ".\n";
          return false;
        }
        user_context_[i].m_LastTrackSucceeded = false;
    }

    // Initialize the RGB image.
    color_image_ = FTCreateImage();
    //if (!color_image_ || FAILED(hr = color_image_->Allocate(video_config.Width, video_config.Height, FTIMAGEFORMAT_UINT8_R8G8B8)))
    if (!color_image_ || FAILED(hr = color_image_->Allocate(video_config.Width, video_config.Height, FTIMAGEFORMAT_UINT8_B8G8R8X8))){
        return false;
    }
    
    if (depth_config_ptr)
    {
        depth_image_ = FTCreateImage();
        if (!depth_image_ || FAILED(hr = depth_image_->Allocate(depth_config.Width, depth_config.Height, FTIMAGEFORMAT_UINT16_D13P3)))
        {
            return false;
        }
    }

    //SetCenterOfImage(NULL);


  return true;
}

void person_face_process::uninitialize(void)
{

  if (user_context_ != 0)
  {
    for (unsigned i=0; i<num_users_; i++)
    {
      if (user_context_[i].m_pFTResult != 0)
      {
        user_context_[i].m_pFTResult->Release();
        user_context_[i].m_pFTResult = 0;
      }
      if (user_context_[i].m_pFaceTracker != 0)
      {
        user_context_[i].m_pFaceTracker->Release();
        user_context_[i].m_pFaceTracker = 0;
      }
    }
    delete[] user_context_;
    user_context_ = 0;
  }
  //gevxl::detectors::gp_3d_detector::uninitialize();
  frame_.clear();
}

bool person_face_process::step(void)
{
  //if(!gevxl::detectors::gp_3d_detector::step()) return false;
  if(!enabled_) return true;

  step_face_tracking();

  step_frame();

  //visuualize results
  visualize(viz_);

  //vil_save(*frame_,"c:/dump/test.png");

  return true;
}

void person_face_process::visualize(gevxl::img::visualizer_2d *viz)
{
  if (!enabled_ || !visualize_)
    return;


  viz_->lock();
  if (disguise_flag_)
  {
    blob_filter(frame_,5,disguised_frame_);
    viz_->set_image( disguised_frame_ );
  }
  else
    viz_->set_image( frame_ );
  viz_->unlock();
  kinect_sensor_.visualize(viz);
  //vil_save(*frame_,"c:/dump/test.png");

  return;


}

void person_face_process::step_face_tracking(void)
{
    HRESULT hrFT = E_FAIL;

    if (kinect_sensor_flag_ && kinect_sensor_.GetVideoBuffer())
    {
        HRESULT hrCopy = kinect_sensor_.GetVideoBuffer()->CopyTo(color_image_, NULL, 0, 0);


        if (SUCCEEDED(hrCopy) && kinect_sensor_.GetDepthBuffer())
        {
            hrCopy = kinect_sensor_.GetDepthBuffer()->CopyTo(depth_image_, NULL, 0, 0);
        }

        // Do face tracking
        if (SUCCEEDED(hrCopy))
        {
            FT_SENSOR_DATA sensorData(color_image_, depth_image_, kinect_sensor_.GetZoomFactor(), kinect_sensor_.GetViewOffSet());
            select_user_to_track(&kinect_sensor_, num_users_, user_context_);
            //if (m_UserSelectCallBack != NULL)
            //{
            //    (*m_UserSelectCallBack)(m_UserSelectCallBackParam, &m_KinectSensor, m_nbUsers, m_UserContext);
            //}
            //else
            //{
            //    SelectUserToTrack(&m_KinectSensor, m_nbUsers, m_UserContext);
            //}
            for (unsigned i=0; i<num_users_; i++)
            {
                if (user_context_[i].m_CountUntilFailure == 0 ||
                    !kinect_sensor_.IsTracked(user_context_[i].m_SkeletonId))
                {
                    user_context_[i].m_LastTrackSucceeded = false;
                    continue;
                }
                FT_VECTOR3D hint[2];
                hint[0] =  kinect_sensor_.NeckPoint(user_context_[i].m_SkeletonId);
                hint[1] =  kinect_sensor_.HeadPoint(user_context_[i].m_SkeletonId);
                
                //if (user_context_[i].m_LastTrackSucceeded)
                //{
                //    hrFT = user_context_[i].m_pFaceTracker->ContinueTracking(&sensorData, hint, user_context_[i].m_pFTResult);
                //}
                //else
                //{
                //    hrFT = user_context_[i].m_pFaceTracker->StartTracking(&sensorData, NULL, hint, user_context_[i].m_pFTResult);
                //}
                if (user_context_[i].m_LastTrackSucceeded)
                {
                    hrFT = user_context_[i].m_pFaceTracker->ContinueTracking(&sensorData, NULL, user_context_[i].m_pFTResult);
                }
                else
                {
                    hrFT = user_context_[i].m_pFaceTracker->StartTracking(&sensorData, NULL, NULL, user_context_[i].m_pFTResult);
                }
                HRESULT temp =user_context_[i].m_pFTResult->GetStatus();
                vcl_cerr << temp << vcl_endl;
                //if (temp == FT_ERROR_HEAD_SEARCH_FAILED) 
                //  vcl_cerr << "error! " << FT_ERROR_HEAD_SEARCH_FAILED << "\n";
                user_context_[i].m_LastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(user_context_[i].m_pFTResult->GetStatus());
                if (user_context_[i].m_LastTrackSucceeded)
                {
                    submit_frace_tracking_result(user_context_[i].m_pFTResult, i);
                }
                else
                {
                    user_context_[i].m_pFTResult->Reset();
                }
                //submit_frace_tracking_result(user_context_[i].m_pFTResult);
            }
        }
    }
}

bool person_face_process::submit_frace_tracking_result(IFTResult* pResult, UINT userId)
{
    if (pResult != NULL && SUCCEEDED(pResult->GetStatus()))
    {

        if (draw_mask_)
        {
            FLOAT* pSU = NULL;
            UINT numSU;
            BOOL suConverged;
            user_context_[userId].m_pFaceTracker->GetShapeUnits(NULL, &pSU, &numSU, &suConverged);
            POINT viewOffset = {0, 0};
            FT_CAMERA_CONFIG cameraConfig;
            if (kinect_sensor_flag_)
            {
                kinect_sensor_.GetVideoConfiguration(&cameraConfig);
            }
            else
            {
                cameraConfig.Width = 640;
                cameraConfig.Height = 480;
                cameraConfig.FocalLength = 500.0f;
            }
            IFTModel* ftModel;
            HRESULT hr = user_context_[userId].m_pFaceTracker->GetFaceModel(&ftModel);
            if (SUCCEEDED(hr))
            {
                DWORD color = s_ColorCode[userId%6];
                hr = visualize_face_model(color_image_, ftModel, &cameraConfig, pSU, 1.0, viewOffset, pResult, color);
                ftModel->Release();
            }
        }
    }
    return TRUE;
}

HRESULT person_face_process::visualize_face_model(IFTImage* pColorImg, IFTModel* pModel, FT_CAMERA_CONFIG const* pCameraConfig, FLOAT const* pSUCoef, 
    FLOAT zoomFactor, POINT viewOffset, IFTResult* pAAMRlt, UINT32 color)
{
    if (!pColorImg || !pModel || !pCameraConfig || !pSUCoef || !pAAMRlt)
    {
        return E_POINTER;
    }

    HRESULT hr = S_OK;
    UINT vertexCount = pModel->GetVertexCount();
    FT_VECTOR2D* pPts2D = reinterpret_cast<FT_VECTOR2D*>(_malloca(sizeof(FT_VECTOR2D) * vertexCount));
    FT_VECTOR3D* pPts3D = reinterpret_cast<FT_VECTOR3D*>(_malloca(sizeof(FT_VECTOR3D) * vertexCount));
    if (pPts2D)
    {
        FLOAT *pAUs;
        UINT auCount;
        hr = pAAMRlt->GetAUCoefficients(&pAUs, &auCount);
        if (SUCCEEDED(hr))
        {
            FLOAT scale, rotationXYZ[3], translationXYZ[3];
            hr = pAAMRlt->Get3DPose(&scale, rotationXYZ, translationXYZ);
            if (SUCCEEDED(hr))
            {
              hr = pModel->Get3DShape(pSUCoef, pModel->GetSUCount(), pAUs, auCount, scale, rotationXYZ, translationXYZ, 
                pPts3D, vertexCount);
              if (SUCCEEDED(hr))
              {
                float mx=0,my=0,mz=0;
                for (unsigned i=0; i<mouth_indx_.size(); i++)
                {
                  unsigned indx=mouth_indx_[i];
                  mx+=pPts3D[indx].x;
                  my+=pPts3D[indx].y;
                  mz+=pPts3D[indx].z;
                }
                mx/=(float)mouth_indx_.size();
                my/=(float)mouth_indx_.size();
                mz/=(float)mouth_indx_.size();
                kinect_sensor_.set_mouth_pos(mx, my, mz);

                //kinect_sensor_.set_mouth_pos(translationXYZ[0], translationXYZ[1], translationXYZ[2]);
                //vcl_ofstream output("C:/dump/facepoints.txt");
                //for (unsigned i=0; i<vertexCount; i++)
                //{
                //  output << pPts3D[i].x << "  " << pPts3D[i].y << "  " << pPts3D[i].z << vcl_endl; 
                //}
                //output.close();
              }
                hr = pModel->GetProjectedShape(pCameraConfig, zoomFactor, viewOffset, pSUCoef, pModel->GetSUCount(), pAUs, auCount, 
                    scale, rotationXYZ, translationXYZ, pPts2D, vertexCount);
                if (SUCCEEDED(hr))
                {
                    POINT* p3DMdl   = reinterpret_cast<POINT*>(_malloca(sizeof(POINT) * vertexCount));
                    if (p3DMdl)
                    {
                        for (UINT i = 0; i < vertexCount; ++i)
                        {
                            p3DMdl[i].x = LONG(pPts2D[i].x + 0.5f);
                            p3DMdl[i].y = LONG(pPts2D[i].y + 0.5f);
                        }

                        FT_TRIANGLE* pTriangles;
                        UINT triangleCount;
                        hr = pModel->GetTriangles(&pTriangles, &triangleCount);
                        if (SUCCEEDED(hr))
                        {
                            struct EdgeHashTable
                            {
                                UINT32* pEdges;
                                UINT edgesAlloc;

                                void Insert(int a, int b) 
                                {
                                    UINT32 v = (min(a, b) << 16) | max(a, b);
                                    UINT32 index = (v + (v << 8)) * 49157, i;
                                    for (i = 0; i < edgesAlloc - 1 && pEdges[(index + i) & (edgesAlloc - 1)] && v != pEdges[(index + i) & (edgesAlloc - 1)]; ++i)
                                    {
                                    }
                                    pEdges[(index + i) & (edgesAlloc - 1)] = v;
                                }
                            } eht;

                            eht.edgesAlloc = 1 << UINT(log(2.f * (1 + vertexCount + triangleCount)) / log(2.f));
                            eht.pEdges = reinterpret_cast<UINT32*>(_malloca(sizeof(UINT32) * eht.edgesAlloc));
                            if (eht.pEdges)
                            {
                                ZeroMemory(eht.pEdges, sizeof(UINT32) * eht.edgesAlloc);
                                for (UINT i = 0; i < triangleCount; ++i)
                                { 
                                    eht.Insert(pTriangles[i].i, pTriangles[i].j);
                                    eht.Insert(pTriangles[i].j, pTriangles[i].k);
                                    eht.Insert(pTriangles[i].k, pTriangles[i].i);
                                }
                                for (UINT i = 0; i < eht.edgesAlloc; ++i)
                                {
                                    if(eht.pEdges[i] != 0)
                                    {
                                        pColorImg->DrawLine(p3DMdl[eht.pEdges[i] >> 16], p3DMdl[eht.pEdges[i] & 0xFFFF], color, 1);
                                    }
                                }
                                _freea(eht.pEdges);
                            }

                            // Render the face rect in magenta
                            RECT rectFace;
                            hr = pAAMRlt->GetFaceRect(&rectFace);
                            if (SUCCEEDED(hr))
                            {
                                POINT leftTop = {rectFace.left, rectFace.top};
                                POINT rightTop = {rectFace.right - 1, rectFace.top};
                                POINT leftBottom = {rectFace.left, rectFace.bottom - 1};
                                POINT rightBottom = {rectFace.right - 1, rectFace.bottom - 1};
                                UINT32 nColor = 0xff00ff;
                                SUCCEEDED(hr = pColorImg->DrawLine(leftTop, rightTop, nColor, 1)) &&
                                    SUCCEEDED(hr = pColorImg->DrawLine(rightTop, rightBottom, nColor, 1)) &&
                                    SUCCEEDED(hr = pColorImg->DrawLine(rightBottom, leftBottom, nColor, 1)) &&
                                    SUCCEEDED(hr = pColorImg->DrawLine(leftBottom, leftTop, nColor, 1));
                            }
                        }

                        _freea(p3DMdl); 
                    }
                    else
                    {
                        hr = E_OUTOFMEMORY;
                    }
                }
            }
        }
        _freea(pPts2D);
    }
    else
    {
        hr = E_OUTOFMEMORY;
    }
    return hr;
}


void person_face_process::step_frame(void)
{

    BOOL ret = TRUE;

    // Now, copy a fraction of the camera image into the screen.
    
    if (color_image_)
    {
      if (!frame_set_)
      {
        frame_width_ = color_image_->GetWidth();
        frame_height_ = color_image_->GetHeight();
        if (frame_width_ > 0 && frame_height_ > 0)
        {
          frame_ = vil_image_view<vxl_byte>(frame_width_,frame_height_,1,3);
          depth_frame_ =vil_image_view<vxl_byte>(depth_image_->GetWidth(), depth_image_->GetHeight(), 1,1);
          disguised_frame_=vil_image_view<vxl_byte>(frame_width_,frame_height_,1,3);
        }
      }
      
      vxl_byte * image_buffer=color_image_->GetBuffer();
      vxl_byte * depth_buffer=depth_image_->GetBuffer();

      for (unsigned j=0; j<frame_height_; j++)
      {
        for (unsigned i=0; i<frame_width_; i++)
        {
          frame_(i,j,0)=image_buffer[2];
          frame_(i,j,1)=image_buffer[1];
          frame_(i,j,2)=image_buffer[0];
          image_buffer += 4;
        }
      }
      for (unsigned j=0; j<depth_image_->GetHeight(); j++)
      {
        for (unsigned i=0; i<depth_image_->GetWidth(); i++)
        {
          depth_frame_(i,j)=depth_buffer[1];
          depth_buffer +=2;
        }
      }
      //frame_.set_to_memory(image_buffer, frame_width_, frame_height_, 3, 
      //                     frame_.istep(), frame_.jstep(), frame_.planestep());
      //frame_.set_to_memory(image_buffer, frame_width_, frame_height_, 3, 
      //                     1, frame_width_, frame_width_*frame_height_);
      //vil_save(depth_frame_,"c:/dump/temp.png");
    }
    return;
}
void person_face_process::select_user_to_track(kinect_sensor * pKinectSensor,
    unsigned nbUsers, FTHelperContext* pUserContexts)
{
    // Initialize an array of the available skeletons
    bool SkeletonIsAvailable[NUI_SKELETON_COUNT];
    for (UINT i=0; i<NUI_SKELETON_COUNT; i++)
    {
        SkeletonIsAvailable[i] = pKinectSensor->IsTracked(i);
    }
    // If the user's skeleton is still tracked, mark it unavailable
    // and make sure we will keep associating the user context to that skeleton
    // If the skeleton is not track anaymore, decrease a counter until we 
    // deassociate the user context from that skeleton id.
    for (UINT i=0; i<nbUsers; i++)
    {
        if (pUserContexts[i].m_CountUntilFailure > 0)
        {
            if (SkeletonIsAvailable[pUserContexts[i].m_SkeletonId])
            {
                SkeletonIsAvailable[pUserContexts[i].m_SkeletonId] = false;
                pUserContexts[i].m_CountUntilFailure++;
                if (pUserContexts[i].m_CountUntilFailure > 5)
                {
                    pUserContexts[i].m_CountUntilFailure = 5;
                }
            }
            else
            {
                pUserContexts[i].m_CountUntilFailure--;
            }
        }
    }

    // Try to find an available skeleton for users who do not have one
    for (UINT i=0; i<nbUsers; i++)
    {
        if (pUserContexts[i].m_CountUntilFailure == 0)
        {
            for (UINT j=0; j<NUI_SKELETON_COUNT; j++)
            {
                if (SkeletonIsAvailable[j])
                {
                    pUserContexts[i].m_SkeletonId = j;
                    pUserContexts[i].m_CountUntilFailure = 1;
                    SkeletonIsAvailable[j] = false;
                    break;
                }
            }
        }
    }
}



void person_face_process::blob_filter(const vil_image_view<vxl_byte> & proc_frame, 
                                      int blob_half_size, 
                                      vil_image_view<vxl_byte> &blob_frame)
{
  unsigned x,y,k;
  unsigned nrows = proc_frame.nj();
  unsigned ncols = proc_frame.ni();
  int row_start_ctr, row_end_ctr, col_start_ctr,col_end_ctr,winarea_ctr;

  int ctr_half_width = blob_half_size;

  int ctr_gen_area = (2*ctr_half_width+1)*(2*ctr_half_width+1);

  blob_frame.set_size(ncols,nrows);

  int sum_ctr;
  
  if (proc_frame)
  {
    for (k=0; k<proc_frame.nplanes(); k++)
    {
      //generate integral image
      vil_image_view<int> int_image;
      gevxl::img::integral_image<vxl_byte,int>(vil_plane(proc_frame,k),int_image);

      //compute the average within the sliding window over the matrix
	    for (y=0;y<nrows;y++)
		    for (x=0;x<ncols;x++)
		    {
			    //estimate window position, considering border effect
			    get_sliding_window(y,x, nrows, ncols, ctr_half_width, ctr_gen_area, row_start_ctr, row_end_ctr, 
                           col_start_ctr, col_end_ctr, winarea_ctr);
    			
			    sum_ctr = int_image(col_start_ctr, row_start_ctr)+int_image(col_end_ctr, row_end_ctr)
                   -int_image(col_end_ctr, row_start_ctr)-int_image(col_start_ctr, row_end_ctr);


          //compute the difference of the avarage
          blob_frame(x,y,k)= sum_ctr/winarea_ctr;

		    }
    }
  }
    
}

void person_face_process::get_sliding_window
        (int rowidx, int colidx, int nrows, int ncols, int winsidewidth, int win_gen_area,
        int &rowstart, int &rowend, int &colstart, int &colend, int &winarea)  
{
  bool recomp_area = false;

  rowstart = rowidx-winsidewidth-1;	
	if ( rowstart < 0 )
  {
    rowstart = 0;
    recomp_area = true;
  }
	
	rowend = rowidx + winsidewidth;
	if (rowend >= nrows) 
  {
    rowend = nrows-1;
    recomp_area = true;
  }

	colstart = colidx - winsidewidth-1;
	if (colstart < 0 )
  {
    colstart = 0;
    recomp_area = true;
  }

	colend = colidx + winsidewidth;
	if (colend >= ncols)
  {
    colend = ncols-1;
    recomp_area = true;
  }
	
  if (recomp_area)
    winarea = (rowend - rowstart + 1)*(colend - colstart+1);
  else
    winarea = win_gen_area;

	return;
}



