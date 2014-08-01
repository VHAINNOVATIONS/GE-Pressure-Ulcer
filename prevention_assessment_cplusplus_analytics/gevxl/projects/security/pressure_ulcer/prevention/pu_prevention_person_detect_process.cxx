// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_prevention_person_detect_process.h"

#include <vcl_iostream.h>
#include <vcl_cmath.h>

#include <vil/vil_save.h>
#include <vil/vil_load.h>

#include <vgl/vgl_convex.h>
#include <vgl/vgl_polygon.h>

#include <vnl/vnl_random.h>

#include <geom/proj_utils.h>

#include <vul/vul_file.h>

using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer::prevention;
using namespace gevxl::geom;

pu_prevention_person_detect_process::pu_prevention_person_detect_process(char const *name)
: gevxl::framework::process(name), 
  viz_(NULL), 
  is_process_busy_(false),
  verbose_(0),
  precomp_initialized_(false),
  persons_detected_(false),
	total_persons_detected_flag_count_(0),
	total_time_elapse_count_(0),
	nr_of_no_persons_detected_frames_(0)
{
  enable(false);
}

pu_prevention_person_detect_process::~pu_prevention_person_detect_process(void)
{

}

bool pu_prevention_person_detect_process::configure(util::config_file &config)
{
	config_ = config;

  bool enabled = false;
  config.get_bool(name()+"::enabled", enabled);
  enable(enabled);

	verbose_ = 0;
	config.get_integer(name()+"::verbose", verbose_);

  // the minimum spatial distance between two detection hypotheses on the ground plane
  min_spatial_distance_thresh_ = 0.2;
  config.get_double(name()+"::min_spatial_distance_thresh", min_spatial_distance_thresh_);

  // the half size of person body for one detection hypothesis
  radius_of_person_body_ = 0.3;
  config.get_double(name()+"::radius_of_person_body", radius_of_person_body_);

  // the cuboid height for person detection hypothesis
  height_of_person_body_ = 1.8;
  config.get_double(name()+"::height_of_person_body", height_of_person_body_);

  // the ground plane z value tolerance
  gp_z_tolerance_thresh_ = 0.1;
  config.get_double(name()+"::gp_z_tolerance_thresh", gp_z_tolerance_thresh_);

  // the number of randomly sampled correspondence positions for camera projection calibration
  num_of_random_samples_for_calibration_ = 100;
  config.get_integer(name()+"::num_of_random_samples_for_calibration", num_of_random_samples_for_calibration_);

  // the height threshold for the person detection
  person_detection_height_thresh_ = 1.40;
  config.get_double(name()+"::person_detection_height_thresh", person_detection_height_thresh_);

  // the score threshold for the person detection
  person_detection_score_thresh_ = 20*20; // 400 pixels have to be falling within the groundplane bounding box to support the hypothesis
	config.get_double(name()+"::person_detection_score_thresh", person_detection_score_thresh_);

	// the precomputed hypothes
	precomputed_hypotheses_full_filename_ = "";
	config.get_string(name()+"::precomputed_hypotheses_full_filename", precomputed_hypotheses_full_filename_);

	return true;
}

bool pu_prevention_person_detect_process::initialize(void)
{
	return true;
}

void pu_prevention_person_detect_process::uninitialize(void)
{

}

bool pu_prevention_person_detect_process::step(const gevxl::vid::frame_tag &tag, 
                                               const vil_image_view<vxl_uint_16> &raw_depth_frame, 
                                               const vil_image_view<float> &rectified_xyz_frame)
{
  persons_detected_ = false;

  if(!is_enabled()) {
    vcl_cout << "pu_prevention_person_detect_process::step, the process is not enabled." << vcl_endl;
    return true;
  }

  if(!precomp_initialized_) {
		vcl_cout << "pu_prevention_person_detect_process::step, begin initialize_precomputations." << vcl_endl;
    precomp_initialized_ = initialize_precomputations(raw_depth_frame, rectified_xyz_frame);
		vcl_cout << "pu_prevention_person_detect_process::step, end initialize_precomputations." << vcl_endl;
  }

  if(!precomp_initialized_) {
    vcl_cout << "pu_prevention_person_detect_process::step, the initialize_precomputations returns false, skip this step." << vcl_endl;
    return true;
  }

  persons_detected_ = detect_persons(raw_depth_frame, rectified_xyz_frame);

	if(persons_detected_) {
		total_persons_detected_flag_count_++;
		nr_of_no_persons_detected_frames_ = 0;
	}
	else {
		nr_of_no_persons_detected_frames_++;
	}
	total_time_elapse_count_++;

	//-- Visualize the process
	//visualize();

	return true;
}

// get the person detection time window count
void pu_prevention_person_detect_process::get_persons_detection_time_window_count(double &total_persons_detected_flag_count, 
																																									double &total_time_elapse_count,
																																									double &nr_of_no_persons_detected_frames)
{
	total_persons_detected_flag_count = total_persons_detected_flag_count_;
	total_time_elapse_count = total_time_elapse_count_;
	nr_of_no_persons_detected_frames = nr_of_no_persons_detected_frames_;
}

void pu_prevention_person_detect_process::reset_persons_detection_time_window_count(void)
{
	total_persons_detected_flag_count_ = 0;
	total_time_elapse_count_ = 0;
	nr_of_no_persons_detected_frames_ = 0;
}

// perform the person detection 
bool pu_prevention_person_detect_process::detect_persons(const vil_image_view<vxl_uint_16> &raw_depth_frame,
                                                         const vil_image_view<float> &rectified_xyz_frame)
{
  // clean up the hypotheses_ scoring
  for(unsigned k = 0; k < hypotheses_.size(); k++) {
    hypotheses_[k].score_ = 0;  
  }

  // start the detection process
  unsigned img_width, img_height;
  img_width = rectified_xyz_frame.ni();
  img_height = rectified_xyz_frame.nj();

  double gp_x, gp_y, gp_z;
  for(unsigned j = 0; j < img_height; j++) {
    for(unsigned i = 0; i < img_width; i++) {

      if(raw_depth_frame(i, j) == 0) {
        // this pixel doesn't have reliable depth sensing measure, ignore.
        continue;
      }

      gp_x = rectified_xyz_frame(i, j, 0);
      gp_y = rectified_xyz_frame(i, j, 1);
      gp_z = rectified_xyz_frame(i, j, 2);

      if(gp_z < person_detection_height_thresh_) {
        // this height of this pixel does not pass the person_detection_height_thresh_, ignore.
        continue;
      }

      // this is a valid pixel with reasonable height, now let's which person hypothesis it belongs to
      for(unsigned k = 0; k < hypotheses_.size(); k++) {
        if(hypotheses_[k].gp_bounding_box_.inside(gp_x, gp_y)) {
          // this pixel falls into this hypothesis because it is inside the hypothesis's groundplane bounding box
          hypotheses_[k].score_++;
        }
      }
    }
  }

  int num_of_detected_hypotheses = 0;
  for(unsigned k = 0; k < hypotheses_.size(); k++) {
    if(hypotheses_[k].score_ > person_detection_score_thresh_) {
      num_of_detected_hypotheses++;
    }
  }

	if(num_of_detected_hypotheses > 0) {
		vcl_cout << "pu_prevention_person_detect_process::detect_persons, num_of_detected_hypotheses = " << num_of_detected_hypotheses << vcl_endl;
	}

  if(num_of_detected_hypotheses > 0) {    
    return true;
  }
  else {
    return false;
  }
}

// initialize the various precomputations
bool pu_prevention_person_detect_process::initialize_precomputations(const vil_image_view<vxl_uint_16> &raw_depth_frame, 
                                                                     const vil_image_view<float> &rectified_xyz_frame)
{
  hypotheses_.clear();

	bool hypotheses_loaded = false;
	if( precomputed_hypotheses_full_filename_ != "" && vul_file::exists(precomputed_hypotheses_full_filename_) ) {

		vsl_b_ifstream precomp_istr(precomputed_hypotheses_full_filename_.c_str());
		hypotheses_loaded = vsl_binary_read( precomp_istr );
		precomp_istr.close();
	}
	if(hypotheses_loaded) return true;

  unsigned img_width, img_height;
  img_width = rectified_xyz_frame.ni();
  img_height = rectified_xyz_frame.nj();

  const float *xyz_ptr = rectified_xyz_frame.top_left_ptr();
  
  // given the rectified_xyz_frame, return all the ground plane (x,y) points using the z threshold
  // and find the min_gp_x, max_gp_x, min_gp_y, max_gp_y
  vcl_vector<vgl_point_2d<double> > gp_locs;
  gp_locs.reserve(img_height*img_width);

  double min_gp_x, max_gp_x, min_gp_y, max_gp_y;
  min_gp_x = 1e+10;
  max_gp_x = -1e+10;
  min_gp_y = 1e+10;
  max_gp_y = -1e+10;

  double gp_x, gp_y, gp_z;
  for(unsigned j = 0; j < img_height; j++) {
    for(unsigned i = 0; i < img_width; i++) {

      gp_x = *(xyz_ptr);
      gp_y = *(xyz_ptr+1);
      gp_z = *(xyz_ptr+2);

      if( vcl_fabs(gp_z) < gp_z_tolerance_thresh_ ) {
        // this is a ground plane point
        gp_locs.push_back( vgl_point_2d<double>(gp_x, gp_y) );
        
        if(gp_x < min_gp_x) min_gp_x = gp_x;
        if(gp_x > max_gp_x) max_gp_x = gp_x;

        if(gp_y < min_gp_y) min_gp_y = gp_y;
        if(gp_y > max_gp_y) max_gp_y = gp_y;
      }
      xyz_ptr += 3;
    }
  }
  if(min_gp_x == 0.0 && max_gp_x == 0.0 && min_gp_y == 0.0 && max_gp_y == 0.0) {
    vcl_cout << "pu_prevention_person_detect_process::initialize_precomputations, the input rectified_xyz_frame is still invalid." << vcl_endl;
    return false;
  }

  // calibrate the camera's projection matrix based on a randomly selected 2d image pixels and 3d world xyz coordinates
  vcl_vector<vnl_double_2> image_pts;
  vcl_vector<vnl_double_3> world_pts;

  int num_of_samples = 0;
  int num_of_groundplane_samples = 0;
  vnl_random rand_generator(9667566);
  int img_x, img_y;
  double wld_x, wld_y, wld_z;
  int max_num_of_groundplane_samples = (int)( (double)(num_of_random_samples_for_calibration_*3.0)/(double)4.0 );  
  while(num_of_samples < num_of_random_samples_for_calibration_) {
    img_x = rand_generator.lrand32(0, img_width);
    img_y = rand_generator.lrand32(0, img_height);

    if(img_x < 0 || img_x >= img_width || img_y < 0 || img_y >= img_height) {
      // the randomly generated image position is outside of the image window, ignore.
      continue;
    }
    if(raw_depth_frame(img_x, img_y) == 0) {
      // the randomly generated image position holds no reliable depth sensing measure, ignore.
      continue;
    }

    wld_x = rectified_xyz_frame(img_x, img_y, 0);
    wld_y = rectified_xyz_frame(img_x, img_y, 1);
    wld_z = rectified_xyz_frame(img_x, img_y, 2);

    if( vcl_fabs(wld_z) < gp_z_tolerance_thresh_ && 
        num_of_groundplane_samples < max_num_of_groundplane_samples) {
      
      // this is a groundplane sample
      image_pts.push_back(vnl_double_2(img_x, img_y));
      world_pts.push_back(vnl_double_3(wld_x, wld_y, wld_z));
      num_of_groundplane_samples++;
      num_of_samples++;
    }
    else if(vcl_fabs(wld_z) > gp_z_tolerance_thresh_) {
      
      // this is not a groundplane sample
      image_pts.push_back(vnl_double_2(img_x, img_y));
      world_pts.push_back(vnl_double_3(wld_x, wld_y, wld_z));
      num_of_samples++;
    }
  }

  bool success = proj_utils::estimate_projection(world_pts, image_pts, proj_mat_);
  if(success) {
    vcl_cout << "pu_prevention_person_detect_process::initialize_precomputations, successfully computed the camera projection matrix = " << vcl_endl;
    vcl_cout << proj_mat_ << vcl_endl;
  }
  else {
    vcl_cout << "pu_prevention_person_detect_process::initialize_precomputations, failed to compute the camera projection matrix" << vcl_endl;
    return false;
  }

	vcl_cout << "pu_prevention_person_detect_process::initialize_precomputations, start to compute the person detection hypotheses" << vcl_endl;

  // find the convex hull of the all the ground plane (x,y) points
  vgl_polygon<double> convex_hull = vgl_convex_hull(gp_locs);

  // based on the min_spatial_distance_thresh_, create the hypotheses
  vcl_vector<vgl_point_2d<double> > final_gp_locs;
  final_gp_locs.reserve(gp_locs.size());

  bool inside = false;
  for(gp_y = min_gp_y; gp_y < max_gp_y; gp_y += min_spatial_distance_thresh_) {
    for(gp_x = min_gp_x; gp_x < max_gp_x; gp_x += min_spatial_distance_thresh_) {
      inside = convex_hull.contains(gp_x, gp_y);
      if(inside) {
        final_gp_locs.push_back( vgl_point_2d<double>(gp_x, gp_y) );
      }
    }
  }

  // start to generate the hypotheses
  vnl_double_2 projected_pt; 
  hypotheses_.reserve(final_gp_locs.size());
  for(unsigned i = 0; i < final_gp_locs.size(); i++) {
    
    pu_prevention_person_detect_hypothesis hypothesis;
    
    // the groundplane center of the person standing
    hypothesis.gp_center_pt_ = final_gp_locs[i];
    // the groundplane bounding box of the person body
    hypothesis.gp_bounding_box_.x0 = hypothesis.gp_center_pt_.x() - radius_of_person_body_;
    hypothesis.gp_bounding_box_.y0 = hypothesis.gp_center_pt_.y() - radius_of_person_body_;
    hypothesis.gp_bounding_box_.x1 = hypothesis.gp_center_pt_.x() + radius_of_person_body_;
    hypothesis.gp_bounding_box_.y1 = hypothesis.gp_center_pt_.y() + radius_of_person_body_;

    // the image pixel position of the groundplane center
    projected_pt = proj_utils::project(proj_mat_, hypothesis.gp_center_pt_.x(), hypothesis.gp_center_pt_.y(), 0);
    hypothesis.gp_center_img_pt_ = vnl_int_2( (int)(projected_pt(0)+0.5), (int)(projected_pt(1)+0.5) );

    // the 8 cuboid corners of the person body in world 3d
    hypothesis.cuboid_3d_corners_.resize(8);
    hypothesis.cuboid_3d_corners_[0] = vgl_point_3d<double>(hypothesis.gp_bounding_box_.x0, hypothesis.gp_bounding_box_.y0, 0);
    hypothesis.cuboid_3d_corners_[1] = vgl_point_3d<double>(hypothesis.gp_bounding_box_.x1, hypothesis.gp_bounding_box_.y0, 0);
    hypothesis.cuboid_3d_corners_[2] = vgl_point_3d<double>(hypothesis.gp_bounding_box_.x1, hypothesis.gp_bounding_box_.y1, 0);
    hypothesis.cuboid_3d_corners_[3] = vgl_point_3d<double>(hypothesis.gp_bounding_box_.x0, hypothesis.gp_bounding_box_.y1, 0);
    
    hypothesis.cuboid_3d_corners_[4] = vgl_point_3d<double>(hypothesis.gp_bounding_box_.x0, hypothesis.gp_bounding_box_.y0, height_of_person_body_);
    hypothesis.cuboid_3d_corners_[5] = vgl_point_3d<double>(hypothesis.gp_bounding_box_.x1, hypothesis.gp_bounding_box_.y0, height_of_person_body_);
    hypothesis.cuboid_3d_corners_[6] = vgl_point_3d<double>(hypothesis.gp_bounding_box_.x1, hypothesis.gp_bounding_box_.y1, height_of_person_body_);
    hypothesis.cuboid_3d_corners_[7] = vgl_point_3d<double>(hypothesis.gp_bounding_box_.x0, hypothesis.gp_bounding_box_.y1, height_of_person_body_);

    // the 8 cuboid corners of the person body in image 2d
    hypothesis.cuboid_2d_corners_.resize(8);
    
    projected_pt = proj_utils::project(proj_mat_, hypothesis.cuboid_3d_corners_[0].x(), hypothesis.cuboid_3d_corners_[0].y(), hypothesis.cuboid_3d_corners_[0].z());
    hypothesis.cuboid_2d_corners_[0] = vnl_int_2( (int)(projected_pt(0)+0.5), (int)(projected_pt(1)+0.5) );
    
    projected_pt = proj_utils::project(proj_mat_, hypothesis.cuboid_3d_corners_[1].x(), hypothesis.cuboid_3d_corners_[1].y(), hypothesis.cuboid_3d_corners_[1].z());
    hypothesis.cuboid_2d_corners_[1] = vnl_int_2( (int)(projected_pt(0)+0.5), (int)(projected_pt(1)+0.5) );
    
    projected_pt = proj_utils::project(proj_mat_, hypothesis.cuboid_3d_corners_[2].x(), hypothesis.cuboid_3d_corners_[2].y(), hypothesis.cuboid_3d_corners_[2].z());
    hypothesis.cuboid_2d_corners_[2] = vnl_int_2( (int)(projected_pt(0)+0.5), (int)(projected_pt(1)+0.5) );
    
    projected_pt = proj_utils::project(proj_mat_, hypothesis.cuboid_3d_corners_[3].x(), hypothesis.cuboid_3d_corners_[3].y(), hypothesis.cuboid_3d_corners_[3].z());
    hypothesis.cuboid_2d_corners_[3] = vnl_int_2( (int)(projected_pt(0)+0.5), (int)(projected_pt(1)+0.5) );
    
    projected_pt = proj_utils::project(proj_mat_, hypothesis.cuboid_3d_corners_[4].x(), hypothesis.cuboid_3d_corners_[4].y(), hypothesis.cuboid_3d_corners_[4].z());
    hypothesis.cuboid_2d_corners_[4] = vnl_int_2( (int)(projected_pt(0)+0.5), (int)(projected_pt(1)+0.5) );
    
    projected_pt = proj_utils::project(proj_mat_, hypothesis.cuboid_3d_corners_[5].x(), hypothesis.cuboid_3d_corners_[5].y(), hypothesis.cuboid_3d_corners_[5].z());
    hypothesis.cuboid_2d_corners_[5] = vnl_int_2( (int)(projected_pt(0)+0.5), (int)(projected_pt(1)+0.5) );
    
    projected_pt = proj_utils::project(proj_mat_, hypothesis.cuboid_3d_corners_[6].x(), hypothesis.cuboid_3d_corners_[6].y(), hypothesis.cuboid_3d_corners_[6].z());
    hypothesis.cuboid_2d_corners_[6] = vnl_int_2( (int)(projected_pt(0)+0.5), (int)(projected_pt(1)+0.5) );
    
    projected_pt = proj_utils::project(proj_mat_, hypothesis.cuboid_3d_corners_[7].x(), hypothesis.cuboid_3d_corners_[7].y(), hypothesis.cuboid_3d_corners_[7].z());
    hypothesis.cuboid_2d_corners_[7] = vnl_int_2( (int)(projected_pt(0)+0.5), (int)(projected_pt(1)+0.5) );

    hypotheses_.push_back(hypothesis);
  }

	if( precomputed_hypotheses_full_filename_ != "" ) {
		
		if( vul_file::exists(precomputed_hypotheses_full_filename_) ) {
			vul_file::delete_file_glob(precomputed_hypotheses_full_filename_);		
		}
		
		vsl_b_ofstream precomp_ostr(precomputed_hypotheses_full_filename_.c_str());
		vsl_binary_write(precomp_ostr);
		precomp_ostr.close();
	}

	vcl_cout << "pu_prevention_person_detect_process::initialize_precomputations, finish to compute the person detection hypotheses" << vcl_endl;

  return true;
}

void pu_prevention_person_detect_process::run_thread(void)
{ 
	do {
		
	}
	while(!get_time_to_stop_flag());
}

const vil_image_view<vxl_byte> &pu_prevention_person_detect_process::cur_frame(void) const
{
  return vil_image_view<vxl_byte>();
}

void pu_prevention_person_detect_process::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;
}

void pu_prevention_person_detect_process::visualize(void)
{	
	/*
	vil_image_view<vxl_byte> viz_img;
	viz_img = cur_frame();
	
	if( viz_ && viz_img.size() > 0 ) {
		gevxl::threading::scoped_lock lock( viz_ );
		viz_->initialize();
		if (viz_->is_initialized() ) { 
			
			viz_->set_image( viz_img );
		}
	}

	IF_CAN_VISUALIZE( viz_ ) {
		viz_->flush();
	}
	*/
}

void pu_prevention_person_detect_process::visualize_overlay(void)
{
	// show whether the persons are detected
	if( viz_ ) {
		if(persons_detected_) {
			viz_->set_foreground(0, 0, 1);
			viz_->add_text( 0, 8, "caregiver(s) detected" );
		}
	}
}

/// Deserialize from a stream.
bool pu_prevention_person_detect_process::vsl_binary_read( vsl_b_istream &bistr )
{
	hypotheses_.clear();

	int size = 0;
	int x_int = 0, y_int = 0;
	int val_int = 0;
	double x_double = 0.0, y_double = 0.0, z_double = 0.0;
	double val_double = 0.0;

	// read the number of hypotheses
	vsl_b_read(bistr, size);
	hypotheses_.reserve(size);

	for(unsigned i = 0; i < size; i++) {
		pu_prevention_person_detect_hypothesis hypothesis;

		// the groundplane center of the person standing    
		vsl_b_read(bistr, x_double);
		vsl_b_read(bistr, y_double);
		hypothesis.gp_center_pt_ = vgl_point_2d<double>(x_double, y_double);

		// the groundplane bounding box of the person body
		vsl_b_read(bistr, val_double);
		hypothesis.gp_bounding_box_.x0 = val_double;
		vsl_b_read(bistr, val_double);
		hypothesis.gp_bounding_box_.y0 = val_double;
		vsl_b_read(bistr, val_double);
		hypothesis.gp_bounding_box_.x1 = val_double;
		vsl_b_read(bistr, val_double);
		hypothesis.gp_bounding_box_.y1 = val_double;

		// the image pixel position of the groundplane center
		vsl_b_read(bistr, x_int);
		vsl_b_read(bistr, y_int);
		hypothesis.gp_center_img_pt_ = vnl_int_2( x_int, y_int );
		
		// the 8 cuboid corners of the person body in world 3d    
		hypothesis.cuboid_3d_corners_.resize(8);
		for(unsigned k = 0; k < 8; k++) {
			vsl_b_read(bistr, x_double);
			vsl_b_read(bistr, y_double);
			vsl_b_read(bistr, z_double);
			hypothesis.cuboid_3d_corners_[k] = vgl_point_3d<double>(x_double, y_double, z_double);			
		}
    
    // the 8 cuboid corners of the person body in image 2d    
		hypothesis.cuboid_2d_corners_.resize(8);
    for(unsigned k = 0; k < 8; k++) {
			vsl_b_read(bistr, x_int);
			vsl_b_read(bistr, y_int);
			hypothesis.cuboid_2d_corners_[k] = vnl_int_2( x_int, y_int );			
		}    
	}

	return true;
}

/// Serialize to a stream.
void pu_prevention_person_detect_process::vsl_binary_write( vsl_b_ostream &bostr )
{
	// write the number of hypotheses
  vsl_b_write(bostr, hypotheses_.size());

  for(unsigned i = 0; i < hypotheses_.size(); i++) {
    
    pu_prevention_person_detect_hypothesis &hypothesis = hypotheses_[i];
    
    // the groundplane center of the person standing    
		vsl_b_write(bostr, hypothesis.gp_center_pt_.x());
		vsl_b_write(bostr, hypothesis.gp_center_pt_.y());

    // the groundplane bounding box of the person body
		vsl_b_write(bostr, hypothesis.gp_bounding_box_.x0);
		vsl_b_write(bostr, hypothesis.gp_bounding_box_.y0);
		vsl_b_write(bostr, hypothesis.gp_bounding_box_.x1);
		vsl_b_write(bostr, hypothesis.gp_bounding_box_.y1);

    // the image pixel position of the groundplane center    
		vsl_b_write(bostr, hypothesis.gp_center_img_pt_(0));
		vsl_b_write(bostr, hypothesis.gp_center_img_pt_(1));

    // the 8 cuboid corners of the person body in world 3d    
		for(unsigned k = 0; k < 8; k++) {
			vsl_b_write(bostr, hypothesis.cuboid_3d_corners_[k].x());
			vsl_b_write(bostr, hypothesis.cuboid_3d_corners_[k].y());
			vsl_b_write(bostr, hypothesis.cuboid_3d_corners_[k].z());
		}
    
    // the 8 cuboid corners of the person body in image 2d    
    for(unsigned k = 0; k < 8; k++) {
			vsl_b_write(bostr, hypothesis.cuboid_2d_corners_[k](0));
			vsl_b_write(bostr, hypothesis.cuboid_2d_corners_[k](1));
		}    
  }
}
