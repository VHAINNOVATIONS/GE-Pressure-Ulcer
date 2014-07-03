// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef   gesec_smartroom_person_skeleton_process_h_
#define   gesec_smartroom_person_skeleton_process_h_

/// \file 
/// \author Yi (Jessica) Yao
/// \date 6/5/2012
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

#define MAX_DEPTH 10000


namespace gesec { namespace smartroom {


/// \brief This class is a process that detects retro-reflective badges on caregivers.
class person_skeleton_process :  
  public gevxl::vid::frame_process<vxl_byte>
  //public gevxl::detectors::gp_3d_detector
{


  public: 

    //////////////////////////////////////////////////////////////////
    // Standard Methods
    //////////////////////////////////////////////////////////////////

    /// Default constructor.
    person_skeleton_process(char const *name="smartroom::person_skeleton_process");

    /// Default destructor.
    virtual ~person_skeleton_process(void);

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
    void set_visualizer(gevxl::img::visualizer_2d *viz) { viz_=viz; }

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
  void process_skeleton(void);
  void smoker_detection(const NUI_SKELETON_DATA & skel);
  vnl_double_2 skeleton_to_screen(Vector4 skeletonPoint, int width, int height);
  void draw_skeleton(const NUI_SKELETON_DATA & skel, int windowWidth, int windowHeight);
  void draw_bone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0, NUI_SKELETON_POSITION_INDEX joint1);




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
    vil_image_view<vxl_byte> frame_;
    unsigned frame_width_;
    unsigned frame_height_;


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

    // Current Kinect
    INuiSensor*             pNuiSensor_;
    HANDLE                  hNextSkeletonEvent_;

    vcl_vector<vnl_double_2> points_;

};

typedef vbl_shared_pointer<person_skeleton_process> person_skeleton_process_sptr;

}} //  namespace gesec, smartroom

#endif //   person_skeleton_process_h
