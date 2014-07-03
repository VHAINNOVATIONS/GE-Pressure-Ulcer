// Copyright (C) 2013 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 11/07/2013
/// \par Modifications:


#include "pu_motion_feature_analysis_proc.h"

#include <vcl_iostream.h>
#include <vil/vil_save.h>
#include <vil/vil_convert.h>

#include <vid/openni2_frame_process.h>

using namespace gevxl;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::util::time;

pu_motion_feature_analysis_proc::pu_motion_feature_analysis_proc(char const *name)
: gevxl::framework::process(name), 	
  source_proc_("vid::source_process"), 
  writer_proc_("vid::writer_process"),
  viz_(NULL),
	visualize_to_image_(false),
	viz_video_out_openned_(false),
  frame_nr_(0)
{
	highres_timer_.reset();
}

pu_motion_feature_analysis_proc::~pu_motion_feature_analysis_proc(void)
{
	if(viz_video_out_openned_) {
		viz_video_out_.close();
		viz_video_out_openned_ = false;
	}
}

bool pu_motion_feature_analysis_proc::configure(util::config_file &config)
{
	config_ = config;

  // ---- source configuration 
	if( !source_proc_.configure(config) ) {
		vcl_cerr << "pu_motion_feature_analysis_proc::configure, Error configuring source_proc_." << vcl_endl;
		return false;
	}
  // ensure the output is the depth view
  source_proc_.set_output_type("depth");

  // configure the frame tag
  if(!frame_tag_proc_.configure(config)) {
    vcl_cerr << "pu_motion_feature_analysis_proc::configure, Error configuring frame_tag_proc_." << vcl_endl;
    return false;
  }
  
  // connect to the spatio-temporal filter
  if(!fgbg_seg_motion_extraction_proc_.configure(config)) {
    vcl_cerr << "pu_motion_feature_analysis_proc::configure, Error configuring fgbg_seg_motion_extraction_proc_." << vcl_endl;		
    return false;
  }

  thresh_sal_frac_ = 0.05;
  config_.get_float(name()+"::thresh_sal_frac", thresh_sal_frac_);
  
  thresh_sal_ = 0.01;
  config_.get_float(name()+"::thresh_sal", thresh_sal_);
  
	visualize_to_image_ = false;
	config.get_bool(name()+"::visualize_to_image", visualize_to_image_);

	viz_video_out_filename_ = "";
	config.get_string(name()+"::viz_video_out_filename", viz_video_out_filename_);
	
	return true;
}

bool pu_motion_feature_analysis_proc::initialize(void)
{
  if(!source_proc_.initialize()) {
    vcl_cerr << "pu_motion_feature_analysis_proc::initialize, Error initializing source_proc_." << vcl_endl;		
    return false;
  }

  const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
  frame_tag_proc_.set_source_tagged_frame_process(openni2_source);
  if(!frame_tag_proc_.initialize()) {
    vcl_cerr << "pu_motion_feature_analysis_proc::initialize, Error initializing frame_tag_proc_." << vcl_endl;		
    return false;
  }

  // ---- writer configuration
  writer_proc_.set_frame_process(source_proc_.get_frame_process());
  writer_proc_.set_tag_source(&frame_tag_proc_);
  if( !writer_proc_.configure(config_) ) {
    vcl_cerr << "pu_motion_feature_analysis_proc::configure, Error configuring writer_proc_." << vcl_endl;
		return false;
  }
  if( !writer_proc_.initialize() ) {
    vcl_cerr << "pu_motion_feature_analysis_proc::configure, Error initializing writer_proc_." << vcl_endl;
		return false;
  }

  if(fgbg_seg_motion_extraction_proc_.get_enabled_flag() == true) {
    const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
    fgbg_seg_motion_extraction_proc_.set_source_frame_process(openni2_source);
    
    if(!fgbg_seg_motion_extraction_proc_.initialize()) {
      vcl_cerr << "pu_motion_feature_analysis_proc::initialize, Error initializing fgbg_seg_motion_extraction_proc_." << vcl_endl;    
      return false;
    }

    fgbg_seg_motion_extraction_proc_.set_tag_source(&frame_tag_proc_);
  }



	return true;
}

void pu_motion_feature_analysis_proc::uninitialize(void)
{
  writer_proc_.uninitialize();

	if(viz_video_out_openned_) {
		viz_video_out_.close();
		viz_video_out_openned_ = false;
	}  
}

bool pu_motion_feature_analysis_proc::step(vil_image_view<vxl_byte> img)
{
  curr_frame_.deep_copy(img);

  if(viz_) {
		gevxl::threading::scoped_lock lock( viz_ );
		viz_->initialize();
		if (viz_->is_initialized()) { 
			viz_->set_image(curr_frame_);
		}
	}

  if(visualize_to_image_) {
		// set up the visualize_to_image stuff using the original color_img
		viz_img_output_.deep_copy(curr_frame_);
		viz_img_ = new gevxl::img::visualizer_image();
		viz_img_->set_output_image(&viz_img_output_);

		if(!viz_video_out_openned_ && viz_video_out_filename_ != "") {
			vid::io::ffmpeg_writer_param p;
			viz_video_out_.open( viz_video_out_filename_.c_str(), 
									 p.encoder(vid::io::ffmpeg_writer_param::MSMPEG4V2).frame_rate(10).bit_rate(2000).size(viz_img_output_.ni(),viz_img_output_.nj()) );
			viz_video_out_openned_ = true;
		}
	}

	// deal with the frame rate issue.
	double frame_rate = 1000/highres_timer_.elapsed();	
	highres_timer_.reset();

	gevxl::util::time::highres_timer state_timer;
	state_timer.reset();

  // start to compute the motion feature from the depth image
  
	////////////////////////////////////////////////////////////////////////
	// Visualization of the process
  visualize();

  // save the visualization image to video
  if(visualize_to_image_ && viz_video_out_openned_) {
		viz_video_out_.write_frame(viz_img_output_);		
	}

  prev_frame_.deep_copy(curr_frame_);
	
  return true;
}


bool pu_motion_feature_analysis_proc::step(void)
{
  const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
  if(!openni2_source) {
    vcl_cerr << "pu_motion_feature_analysis_proc::step, the source proc is not openni2 process." << vcl_endl;
		return false;
  }

  if(!source_proc_.step()) {
    vcl_cerr << "pu_motion_feature_analysis_proc::step, stepping openni2 source error." << vcl_endl;
		return false;
  }

  if(!frame_tag_proc_.step()) {
    vcl_cerr << "pu_motion_feature_analysis_proc::step, stepping frame_tag_proc_ error." << vcl_endl;
		return false;
  }
  
  frame_tag_proc_.set_frame_id(frame_nr_);
  frame_tag_proc_.set_frame_nr(frame_nr_);
  frame_tag_proc_.set_time_code((1000.0/30.0)*frame_nr_);
  frame_nr_++;

  if(fgbg_seg_motion_extraction_proc_.get_enabled_flag() == true) {
    
    fgbg_seg_motion_extraction_proc_.set_id(0);
    if(!fgbg_seg_motion_extraction_proc_.step()) {
      vcl_cerr << "pu_motion_feature_analysis_proc::step(): motion extraction process failed.\n";
      return false;
    }
  }

  vil_image_view<vxl_byte> depth_img = openni2_source->cur_frame();

	return step(depth_img);	
}

void pu_motion_feature_analysis_proc::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;

	if(visualize_to_image_) {
		
	}
}

void pu_motion_feature_analysis_proc::visualize(void)
{
	// Visualization stuff of each processes    	
	
	IF_CAN_VISUALIZE( viz_ ) {

	  if(fgbg_seg_motion_extraction_proc_.get_enabled_flag() == true) {

      sal_pts_.clear();
      fgbg_seg_motion_extraction_proc_.get_salient_points(sal_pts_, thresh_sal_frac_, thresh_sal_);

      for (unsigned i = 0; i < sal_pts_.size(); i++) {
        //get the points
        unsigned x = sal_pts_[i][0];
        unsigned y = sal_pts_[i][1];

        viz_->set_foreground(0, 0, 1);
        viz_->add_circle(x, y, 1);

        //if(visualize_to_image_ && viz_video_out_openned_) {
        //  viz_img_->set_foreground(0, 0, 1);
        //  viz_img_->add_circle(x, y, 1);
        //}
      }
    
    }

    viz_->flush();
	}
}

