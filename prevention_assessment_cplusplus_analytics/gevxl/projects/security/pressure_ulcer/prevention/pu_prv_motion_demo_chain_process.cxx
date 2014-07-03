// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_prv_motion_demo_chain_process.h"
#include <vcl_iostream.h>
#include <vil/vil_save.h>
#include <vil/vil_convert.h>
#include <vil/vil_flip.h>
#include <vid/openni2_frame_process.h>
#include <sstream> // for converting float to string
#include <img/tile_image.h>

using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer::prevention;

pu_prv_motion_demo_chain_process::pu_prv_motion_demo_chain_process(char const *name)
: gevxl::framework::process(name)
, source_proc_("vid::source_process") 
, writer_proc_("vid::writer_process")
, viz_(NULL)
, viz_video_out_openned_(false)
, frame_nr_(0)
{
  highres_timer_.reset();
}

pu_prv_motion_demo_chain_process::~pu_prv_motion_demo_chain_process(void)
{
  if(viz_video_out_openned_) 
  {
    viz_video_out_.close();
    viz_video_out_openned_ = false;
  }
}

bool pu_prv_motion_demo_chain_process::configure(util::config_file &config)
{
  config_ = config;

  viz_video_out_filename_ = "";
  config.get_string(name()+"::viz_video_out_filename", viz_video_out_filename_);

  // ---- Configure source process 
  if( !source_proc_.configure(config) ) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::configure, Error configuring source_proc_." << vcl_endl;
    return false;
  }
  source_proc_.set_output_type("depth"); // ensure the output is the depth view

  // ---  Configure the frame tag process
  if(!frame_tag_proc_.configure(config)) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::configure, Error configuring frame_tag_proc_." << vcl_endl;
    return false;
  }

  // -- Configure the rectify_kinect process
  if (!rectify_kinect_proc_.configure(config)) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::configure, Error configuring rectify_kinect_proc_." << vcl_endl;
    return false;
  }

  // 
  if (!motion_estimate_proc_.configure(config)) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::configure, Error configuring motion_estimate_proc_." << vcl_endl;
    return false;
  }

  return true;
}

bool pu_prv_motion_demo_chain_process::initialize(void)
{
  if(!source_proc_.initialize()) 
  {
    vcl_cerr << "pu_prv_motion_demo_chain_process::initialize, Error initializing source_proc_." << vcl_endl;        
    return false;
  }

  const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
  frame_tag_proc_.set_source_tagged_frame_process(openni2_source);
  if(!frame_tag_proc_.initialize()) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::initialize, Error initializing frame_tag_proc_." << vcl_endl;        
    return false;
  }


  // ---- writer configuration
  writer_proc_.set_frame_process(source_proc_.get_frame_process());
  writer_proc_.set_tag_source(&frame_tag_proc_);
  if( !writer_proc_.configure(config_) ) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::configure, Error configuring writer_proc_." << vcl_endl;
    return false;
  }
  if( !writer_proc_.initialize() ) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::configure, Error initializing writer_proc_." << vcl_endl;
    return false;
  }

  return true;
}

void pu_prv_motion_demo_chain_process::uninitialize(void)
{
  writer_proc_.uninitialize();

  if(viz_video_out_openned_) {
    viz_video_out_.close();
    viz_video_out_openned_ = false;
  }  
}

vil_image_view<vxl_byte> colorize_yellow_scheme( const vil_image_view<vxl_byte> &img ) 
{
  assert( img.nplanes() == 1 ); // grayscale 

  vil_image_view<vxl_byte> out_img( img.ni(), img.nj(), 1, 3 );
  vil_image_view<vxl_byte>::const_iterator in_itr = img.begin();
  vil_image_view<vxl_byte>::iterator       out_itr = out_img.begin();
  while ( img.end() != in_itr ) 
  {
    *(out_itr      ) = *in_itr;
    *(out_itr  + 1 ) = min(255, *in_itr+85);
    *(out_itr  + 2 ) = min(255, *in_itr+85);

    in_itr  += 1;
    out_itr += 3;
  }
  return out_img;
}


bool pu_prv_motion_demo_chain_process::step(void)
{
  // Stepping through individual process in the chain
  if(!source_proc_.step()) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::step, stepping openni2 source error." << vcl_endl;
    return false;
  }

  if(!frame_tag_proc_.step()) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::step, stepping frame_tag_proc_ error." << vcl_endl;
    return false;
  }

  // --- Rectify
  const vid::openni2_frame_process *openni2_proc = dynamic_cast<const vid::openni2_frame_process *>(source_proc_.get_frame_process());
  if(!openni2_proc ) {
    vcl_cerr << "pu_prv_motion_demo_chain_process::step(), Error executing rectify_kinect_proc_.step()." << vcl_endl;
    return false;
  }
  rectify_kinect_proc_.step( openni2_proc->cur_xyz_rgb_frame());


  //-- Motion Estimate 
  motion_estimate_proc_.step( openni2_proc->cur_depth_byte_frame() );

  //-- Deal with the frame rate issue.
  double frame_rate = 1000/highres_timer_.elapsed();    
  highres_timer_.reset();

  gevxl::util::time::highres_timer state_timer;
  state_timer.reset();

  frame_tag_proc_.set_frame_id(frame_nr_);
  frame_tag_proc_.set_frame_nr(frame_nr_);
  frame_tag_proc_.set_time_code((1000.0/30.0)*frame_nr_);
  frame_nr_++;

  //-- Visualize the process
  visualize();

  return true;
}

void pu_prv_motion_demo_chain_process::set_visualizer( gevxl::img::visualizer_2d *viz)
{
  viz_ = viz;
}

void pu_prv_motion_demo_chain_process::visualize(void)
{
  // Give each process in the chain a chance to draw to the visualizer
  vid::openni2_frame_process* openni2_proc = dynamic_cast<gevxl::vid::openni2_frame_process*>(source_proc_.get_frame_process());
  vil_image_view<vxl_byte> gray_img = openni2_proc->cur_frame();

  if( viz_ && openni2_proc && gray_img.size()>0 ) {
    gevxl::threading::scoped_lock lock( viz_ );
    viz_->initialize();
    if (viz_->is_initialized() ) { 
      // Tile a large image and visualize it.  Encountered problem with viz_->paste(..) function as it
      // does not seem to centralize the image correctly. I guess it is a problem with computing the 
      // origin values for OpenGL correctly within the viz_->paste() function.
      vcl_vector< vil_image_view<vxl_byte> > img_vec;
      img_vec.push_back(colorize_yellow_scheme(gray_img)); // depth image from kinect
      img_vec.push_back( motion_estimate_proc_.get_visualization_image());
      img_vec.push_back( motion_estimate_proc_.get_motion_histogram_image());
      vil_image_view<vxl_byte> tiled_img = img::tile_horizontal( img_vec );
      viz_->set_image( tiled_img );

      // Show motion magnitude information
      std::ostringstream buf;
      buf << motion_estimate_proc_.get_scalar_motion_magnitude();
      viz_->add_text( 640, 8, buf.str() );
    }
  }

  IF_CAN_VISUALIZE( viz_ ) 
  {
    viz_->flush();
  }
}

