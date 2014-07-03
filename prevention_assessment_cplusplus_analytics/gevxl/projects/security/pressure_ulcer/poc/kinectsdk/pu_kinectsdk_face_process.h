// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef   gesec_smartroom_person_face_process_h_
#define   gesec_smartroom_person_face_process_h_

/// \file 
/// \author Yi (Jessica) Yao
/// \date 6/8/2012
/// \par Modifications: 
/// - Original versioin


#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>
#include <img/visualizer_2d.h>
#include <util/config_file.h> 
#include <vcl_string.h>
#include <vcl_vector.h>
//activemq stuff
#include <netcom/activemq_connection.h>
#include <netcom/activemq_message_sender.h>
#include <detectors/gp_3d_detector.h>
#include <vil/vil_rgb.h>

#include <util/time/highres_time.h>


//#include <XnOpenNI.h>
//#include <XnCodecIDs.h>
//#include <XnCppWrapper.h>
#include <windows.h>
//#include <WinNT.h>
#include <NuiApi.h>
//#include <WinBase.h>
#include <smartroom/depth/kinect_sensor.h>

#define MAX_DEPTH 10000


namespace gesec { namespace smartroom {
struct FTHelperContext
{
    IFTFaceTracker*     m_pFaceTracker;
    IFTResult*          m_pFTResult;
    FT_VECTOR3D         m_hint3D[2];
    bool                m_LastTrackSucceeded;
    int                 m_CountUntilFailure;
    UINT                m_SkeletonId;
};


/// \brief This class is a process that detects retro-reflective badges on caregivers.
class person_face_process :  
  public gevxl::vid::frame_process<vxl_byte>
  //public gevxl::detectors::gp_3d_detector
{


  public: 

    //////////////////////////////////////////////////////////////////
    // Standard Methods
    //////////////////////////////////////////////////////////////////

    /// Default constructor.
    person_face_process(char const *name="smartroom::person_face_process");

    /// Default destructor.
    virtual ~person_face_process(void);

    //////////////////////////////////////////////////////////////////
    // Methods
    //////////////////////////////////////////////////////////////////

    virtual bool configure(gevxl::util::config_file& config );
    ///  get configure
    //gevxl::util::config_file& get_config_file() { return config_; }

    /// Return the current frame
    vil_image_view<vxl_byte> const & cur_frame() const 
    {
      return frame_;
    }


    void enable_visualize(bool flag) {visualize_ = flag;}
    bool is_visualize(void) const {return visualize_;}
    void set_visualizer(gevxl::img::visualizer_2d *viz) { viz_=viz; kinect_sensor_.set_visualizer(viz); }

    //void set_gen_frame_process(gevxl::vid::generic_frame_process<vxl_byte> *gfp) 
    //{
    //  gen_frame_proc_ = gfp;
    //  source_frame_proc_ = gen_frame_proc_->get_frame_process();
    //}
        
    //virtual bool configure(const gevxl::util::config_file& config );
    virtual bool initialize(void);
    virtual void uninitialize(void);
    virtual bool step(void);
    virtual void visualize(gevxl::img::visualizer_2d *viz);

  protected:
    void step_face_tracking(void);
    void step_frame(void);
    bool submit_frace_tracking_result(IFTResult* pResult, UINT userId);
    HRESULT visualize_face_model(IFTImage* pColorImg, IFTModel* pModel, FT_CAMERA_CONFIG const* pCameraConfig, FLOAT const* pSUCoef, 
        FLOAT zoomFactor, POINT viewOffset, IFTResult* pAAMRlt, UINT32 color);
    void select_user_to_track(kinect_sensor * pKinectSensor,
        UINT nbUsers, FTHelperContext* pUserContexts);
    void blob_filter(const vil_image_view<vxl_byte> & proc_frame, 
                                      int blob_half_size, 
                                      vil_image_view<vxl_byte> &blob_frame);

    void get_sliding_window
          (int rowidx, int colidx, int nrows, int ncols, int winsidewidth, int win_gen_area,
          int &rowstart, int &rowend, int &colstart, int &colend, int &winarea);

    //int check_rc(int nRetVal, vcl_string what)	
    //{
	   // if (nRetVal != XN_STATUS_OK)									
	   // {																
		  //  printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));
		  //  return nRetVal;												
	   // }
    //  return nRetVal;
    //}
    //void compute_chist(void);
  //void process_skeleton(void);
  //void smoker_detection(const NUI_SKELETON_DATA & skel);
  //vnl_double_2 skeleton_to_screen(Vector4 skeletonPoint, int width, int height);
  //void draw_skeleton(const NUI_SKELETON_DATA & skel, int windowWidth, int windowHeight);
  //void draw_bone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0, NUI_SKELETON_POSITION_INDEX joint1);




  protected: 

    //xn::Context context_;
    //xn::DepthGenerator depth_generator_;
    //xn::UserGenerator user_generator_;

    vcl_string sample_xml_path_;

    gevxl::img::visualizer_2d *viz_;
    bool visualize_;

    bool enabled_;

    //gevxl::vid::generic_frame_process<vxl_byte> *gen_frame_proc_;

    /// Current frame
    bool frame_set_;
    vil_image_view<vxl_byte> frame_;
    unsigned frame_width_;
    unsigned frame_height_;
    vil_image_view<vxl_byte> depth_frame_;

    bool disguise_flag_;
    vil_image_view<vxl_byte> disguised_frame_;


	  //xn::SceneMetaData scene_metadata_;
	  //xn::DepthMetaData depth_metadata_;

    ///// ActiveMQ connection
    //gevxl::netcom::activemq_connection_sptr activemq_connection_sptr_;
    ///// ActiveMQ message sender
    //gevxl::netcom::activemq_message_sender_sptr activemq_message_sender_sptr_;

    //set time when no time tag is available
    double global_start_time_; 
    gevxl::util::time::highres_time hires_time_;     
    double system_start_time_;

    //vcl_vector< vil_rgb<double> > colors_;

    //float g_pDepthHist[MAX_DEPTH];

    //// Current Kinect
    kinect_sensor                kinect_sensor_;
    bool                         kinect_sensor_flag_;
    unsigned                     num_users_;
    FTHelperContext*             user_context_;
    IFTImage*                    color_image_;
    IFTImage*                    depth_image_;
    NUI_IMAGE_TYPE               depth_type_;
    NUI_IMAGE_RESOLUTION         depth_res_;
    bool                         near_mode_flag_;
    NUI_IMAGE_TYPE               color_type_;
    NUI_IMAGE_RESOLUTION         color_res_;
	  bool             						 seated_skeleton_flag_;
    bool                         draw_mask_;

    vcl_vector<unsigned> mouth_indx_;

};

typedef vbl_shared_pointer<person_face_process> person_face_process_sptr;

}} //  namespace gesec, smartroom

#endif //   person_face_process_h
