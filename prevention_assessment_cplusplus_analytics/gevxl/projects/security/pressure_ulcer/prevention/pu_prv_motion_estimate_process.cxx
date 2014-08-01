// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include <pressure_ulcer/prevention/pu_prv_motion_estimate_process.h>
#include <vil/vil_copy.h>
#include <vil/vil_color_table.h>
#include <vil/vil_convert.h>
#include <vcl_algorithm.h> // std::fill
#include <vil/algo/vil_binary_opening.h>

#include <vgl/vgl_polygon.h>

using namespace gevxl;
using namespace gevxl::pressure_ulcer::prevention;

pu_prv_motion_estimate_process::pu_prv_motion_estimate_process( char const * name )
: gevxl::framework::process(name),
	mh_history_size_( 15 ),
	mh_img_buf_( mh_history_size_ ),
	mh_scalar_magnitude_( 0.0f ),
	mh_viz_width_( 200 ),
	mh_threshold_( 20 ),
	mh_morphology_se_radius_( 3.0 ),
	enable_morphology_noise_removal_( false ),
	viz_(NULL),
	total_motion_detected_flag_count_(0),
	total_time_elapse_count_(0),
	nr_of_no_motion_detected_frames_(0)
{
}

pu_prv_motion_estimate_process::~pu_prv_motion_estimate_process( void )
{
}

bool pu_prv_motion_estimate_process::configure( gevxl::util::config_file &config ) 
{
  mh_viz_width_ = 200;
  config.get_integer(name()+"::motion_history_viz_width", mh_viz_width_);

  mh_threshold_ = 20;
  config.get_integer(name()+"::motion_history_threshold", mh_threshold_);

  enable_morphology_noise_removal_ = false;
  config.get_bool(name()+"::enable_morphology_noise_removal", enable_morphology_noise_removal_);
  
  mh_morphology_se_radius_ = 3.0;
  config.get_double(name()+"::motion_histogram_morphology_se_radius", mh_morphology_se_radius_);

	mh_scalar_magnitude_high_thresh_ = 0.2;
	config.get_double(name()+"::mh_scalar_magnitude_high_thresh", mh_scalar_magnitude_high_thresh_);

	mh_scalar_magnitude_low_thresh_ = 0.1;
	config.get_double(name()+"::mh_scalar_magnitude_low_thresh", mh_scalar_magnitude_low_thresh_);

	// the roi quadrilateral defined in the input raw depth frame
	vcl_vector<double> vec;
	vec.clear();
	config.get_vcl_vector_double(name()+"::roi_quadrilateral", vec);
	if(vec.size() != 8) {
		vcl_cerr << "pu_prv_motion_estimate_process::configure, the roi_quadrilateral is not configured properly." << vcl_endl;
		return false;
	}

	double x[4];
	double y[4];	
	x[0] = vec[0];
	y[0] = vec[1];
	x[1] = vec[2];
	y[1] = vec[3];
	x[2] = vec[4];
	y[2] = vec[5];
	x[3] = vec[6];
	y[3] = vec[7];

	vgl_polygon<double> roi_polygon(x, y, 4);
	
	// create the roi_mask_img_
	roi_mask_img_ = vil_image_view<vxl_byte>(640, 480, 1, 1);
	roi_mask_img_.fill(0);
	roi_mask_area_ = 0;

	for(unsigned y = 0; y < roi_mask_img_.nj(); y++) {
		for(unsigned x = 0; x < roi_mask_img_.ni(); x++) {
			if(roi_polygon.contains(x, y)) {
				roi_mask_img_(x, y) = 255;
				roi_mask_area_++;
			}
		}
	}

  return true;
}

// initialize the process
bool pu_prv_motion_estimate_process::initialize(void)
{
  return true;
}

// uninitialize the process
void pu_prv_motion_estimate_process::uninitialize(void)
{

}

// compute the motion history image buffer
void pu_prv_motion_estimate_process::compute_motion_history(// input params
                            const vil_image_view<float>                               &bg_img,
                            const boost::circular_buffer<vil_image_view<vxl_byte> >   &img_buf,
                            unsigned int                                              motion_histogram_img_width, // visualization of motion histogram
                            int                                                       motion_histogram_threshold, // larger value => less noisy
                            bool                                                      enable_morphology_noise_removal,
                            double                                                    motion_histogram_morphology_se_radius,
                            // output params
                            float                     &motion_magnitude,
                            vcl_vector<int>           &motion_histogram,
                            vil_image_view<vxl_byte>  &motion_history_img,
                            vil_image_view<vxl_byte>  &motion_histogram_img
                            )
{

  // Sanity check
  assert( img_buf.size() > 0 );
  const unsigned int ni = img_buf[0].ni(); //width
  const unsigned int nj = img_buf[0].nj(); //height

  // Initialization
  motion_history_img = vil_image_view<vxl_byte>( ni, nj, 1, 3);
  motion_history_img.fill(0);

  vil_image_view<int> count_img( ni, nj, 1 ); // for keeping tracking of the number of increase or decrease
  count_img.fill(0); 

  // Let us compute the visualization for the motion history! 
  unsigned int num_increase = 0;
  unsigned int num_decrease = 0;
  for(unsigned int i = 0; i < img_buf.size(); ++i) 
  {
    const vxl_byte mono_color = (vxl_byte)( (255.0*i)/img_buf.capacity() );
    const vil_rgb<vxl_byte> neg_color = color_value(REDTEMP, mono_color); 
    const vil_rgb<vxl_byte> pos_color = color_value(BWLIN2,  mono_color); 
    const vxl_byte neg_color_R = neg_color.R();  // cache the values, save one function call
    const vxl_byte neg_color_G = neg_color.G();
    const vxl_byte neg_color_B = neg_color.B();
    const vxl_byte pos_color_R = pos_color.R();
    const vxl_byte pos_color_G = pos_color.G();
    const vxl_byte pos_color_B = pos_color.B();

    vil_image_view<vxl_byte>                 cur_img    = img_buf[i];

    vil_image_view<float>::const_iterator    bg_itr     = bg_img.begin();
    vil_image_view<vxl_byte>::const_iterator cur_itr    = cur_img.begin();

    vil_image_view<vxl_byte>::iterator       c_itr      = motion_history_img.begin();
    vil_image_view<int>::iterator            count_itr  = count_img.begin();
    
    //while( cur_itr != cur_img.end() ) 
		for(unsigned y = 0; y < cur_img.nj(); y++) {
			for(unsigned x = 0; x < cur_img.ni(); x++) {
				
				if ( *cur_itr && *bg_itr) { // process those valid depth values
					const int diff  = (int)(*cur_itr) - (int)(*bg_itr);
					if(diff > motion_histogram_threshold)  // remove spurious signals
					{
						*(c_itr    ) = neg_color_R; 
						*(c_itr + 1) = neg_color_G; 
						*(c_itr + 2) = neg_color_B; 
						*count_itr += 1;
						
						if(roi_mask_img_(x, y) == 255) {
							num_increase++;
						}
					} 
					else if(diff < -motion_histogram_threshold ) 
					{
						*(c_itr    ) = pos_color_R; 
						*(c_itr + 1) = pos_color_G; 
						*(c_itr + 2) = pos_color_B; 
						*count_itr -= 1;

						if(roi_mask_img_(x, y) == 255) {
							num_decrease++;
						}
					}
				}
				cur_itr   += 1;
				bg_itr    += 1;
				count_itr += 1;
				c_itr     += 3;
			}
		}

  }

	// Quantify motion using a single number
  const float epsilon = 0.00001f; // avoid division by zero
  //motion_magnitude    = (1.0f * num_increase ) / (num_increase + num_decrease + epsilon);
	motion_magnitude = (float)(num_increase + num_decrease)/(float)(roi_mask_area_*img_buf.size());

  // Morphological operation on the motion histogram image to remove spurious foreground
  if(enable_morphology_noise_removal)
  {
    // Create the binary image from the motion history image (which is in interleaved rgb)
    vil_image_view<bool> src_binary_img( motion_history_img.ni(), motion_history_img.nj(), 1 );
    vil_image_view<vxl_byte>::iterator c_itr  = motion_history_img.begin();
    vil_image_view<bool>::iterator     sb_itr = src_binary_img.begin();
    while( c_itr != motion_history_img.end() ) 
    {
      ( *c_itr > 0 || *(c_itr+1) > 0 || *(c_itr+2) > 0 ) ? *sb_itr = true : *sb_itr = false;
      sb_itr += 1;
      c_itr  += 3; // interleaved rgb
    }

    // Morphologize away!
    vil_structuring_element SE;
    SE.set_to_disk( motion_histogram_morphology_se_radius );
    vil_image_view<bool> dest_binary_img;
    vil_binary_opening( src_binary_img, dest_binary_img, SE );

    // Translate the morphology result back to rgb image
    vil_image_view<bool>::iterator db_itr = dest_binary_img.begin();
    sb_itr = src_binary_img.begin();
    c_itr = motion_history_img.begin();
    while( db_itr != dest_binary_img.end() ) 
    {        
      if(!(*db_itr)) // background
      {
        *(c_itr    ) = 0;
        *(c_itr + 1) = 0;
        *(c_itr + 2) = 0;
      }
      db_itr += 1;
      c_itr  += 3;
    }
  }

  // Quantify motion using a histogram
  motion_histogram.resize( nj, 0 );
  vcl_fill( motion_histogram.begin(), motion_histogram.end(), 0 );
  for (unsigned int j=0; j<nj; ++j ) { //per row
    for (unsigned int i=0; i<ni; ++i ) { //per col
      motion_histogram[j] += count_img(i,j);
    }
  }

  // Create the visualization associated with the histogram
  const float max_count = (float) ni * img_buf.size(); 
  motion_histogram_img  = vil_image_view<vxl_byte>( motion_histogram_img_width, nj, 1, 3);
  motion_histogram_img.fill(0);
  const unsigned int mid_pt = motion_histogram_img_width/2; 
  for (unsigned int bin=0; bin<nj; ++bin) { // for each histogram bin
    const          int hist_count       = motion_histogram[bin];
    const unsigned int normalized_count = (unsigned int) (mid_pt * vcl_abs(hist_count)/max_count);
    assert( normalized_count < mid_pt );
    if ( hist_count > 0 ) // draw to the left of midpoint 
    {
      for (unsigned int c=1; c<normalized_count; ++c) {
        const unsigned int      x         = mid_pt - c;
        vxl_byte                gray_col  = (vxl_byte) vcl_min( 255.0f, 180+(255.0f*c)/mid_pt );
        const vil_rgb<vxl_byte> pos_color = color_value(REDTEMP,  gray_col); 
        motion_histogram_img(x,bin,0)     = pos_color.R();
        motion_histogram_img(x,bin,1) 	  = pos_color.G();
        motion_histogram_img(x,bin,2) 	  = pos_color.B();
      }

    }
    else if (hist_count < 0 ) // draw to the right of midpoint
    {
      for (unsigned int c=1; c<normalized_count; ++c) {
        const unsigned int      x         = mid_pt + c;
        vxl_byte                gray_col  = (vxl_byte) vcl_min( 255.0f, 150+(255.0f*c*1.5f)/mid_pt );
        const vil_rgb<vxl_byte> neg_color = color_value(BWLIN2, gray_col); 
        motion_histogram_img(x,bin,0)     = neg_color.R();
        motion_histogram_img(x,bin,1)     = neg_color.G();
        motion_histogram_img(x,bin,2)     = neg_color.B();
      }
    }
  }
}

// Control the rate at which background is updated 
void pu_prv_motion_estimate_process::update_background(const vil_image_view<vxl_byte> &img, vil_image_view<float> &bg_img)
{
  // Input sanity check
  assert( img.ni() > 0 );
  assert( img.nj() > 0 );
  assert( bg_img.ni() > 0 );
  assert( bg_img.nj() > 0 );
  assert( img.ni() == bg_img.ni() );
  assert( img.nj() == bg_img.nj() );
  assert( img.nplanes() == bg_img.nplanes() );

  // Lets do the real work
  const float                              fg_weight = 0.95f;// larger => faster adaptation 
  const float                              bg_weight = 1 - fg_weight; 
  vil_image_view<vxl_byte>::const_iterator itr       = img.begin();
  vil_image_view<float>::iterator          bg_itr    = bg_img.begin();
  while ( img.end() != itr ) 
  {
    *bg_itr = bg_weight*(*bg_itr) + fg_weight *(*itr); 
    itr ++;
    bg_itr ++;
  }
}


bool pu_prv_motion_estimate_process::step( const gevxl::vid::frame_tag &tag, const vil_image_view<vxl_byte> &depth_byte_img ) 
{
  if( !(depth_byte_img.ni() > 0  && depth_byte_img.nj() > 0) ) 
  {
    // in the initial stages of intialization for the whole app
    // OpenNI2 api may not have successfully obtain an image from
    // the kinect
    return true;
  }

  assert( 1 == depth_byte_img.nplanes() ); // 8 bit depth values 

  // Cache the images
  vil_image_view<vxl_byte> incomming_img; 
  incomming_img.deep_copy( depth_byte_img );  // within the OpenNI2 api, the image memory chunk is always the same 
                                              // hence that memory chunk always get overwritten with the
                                              // motion recent image at each time step. 
  prev_img_ = cur_img_;
  cur_img_  = incomming_img;

  //
  // Motion history  
  //
  if (mh_img_buf_.size() == 0) { 
    // probably first time step() is invoked. 
    // Use whatever we have on hand as background
    vil_convert_cast( incomming_img, mh_bg_img_); // copy over to float type
  }

  mh_img_buf_.push_back( incomming_img );
  update_background( incomming_img, mh_bg_img_ );

  compute_motion_history( mh_bg_img_, mh_img_buf_, mh_viz_width_, mh_threshold_, 
                          enable_morphology_noise_removal_, mh_morphology_se_radius_, 
                          mh_scalar_magnitude_, mh_histogram_, mh_viz_img_, mh_histogram_img_ );

  //// optical flow computation
  // optical_flow_estimator_.set_image_dimensions( cur_img_.ni(), cur_img_.nj() );
  // optical_flow_estimator_.set_previous_frame( prev_img_ );
  // optical_flow_estimator_.set_current_frame( cur_img_ );
  // optical_flow_estimator_.compute_lucas_kanade_motion();
	
	if(mh_scalar_magnitude_ > mh_scalar_magnitude_high_thresh_) {
		total_motion_detected_flag_count_++;		
	}
	
	if(mh_scalar_magnitude_ < mh_scalar_magnitude_low_thresh_) {
		nr_of_no_motion_detected_frames_++;
	}
	else {
		nr_of_no_motion_detected_frames_ = 0;
	}

	total_time_elapse_count_++;

  return true;
}

void pu_prv_motion_estimate_process::get_body_motion_detection_time_window_count(double &total_motion_detected_flag_count, 
																																								 double &total_time_elapse_count,
																																								 double &nr_of_no_motion_detected_frames)
{
	total_motion_detected_flag_count = total_motion_detected_flag_count_;
	total_time_elapse_count = total_time_elapse_count_;
	nr_of_no_motion_detected_frames = nr_of_no_motion_detected_frames_;
}

void pu_prv_motion_estimate_process::reset_body_motion_detection_time_window_count(void)
{
	total_motion_detected_flag_count_ = 0;
	total_time_elapse_count_ = 0;
	nr_of_no_motion_detected_frames_ = 0;
}

vil_image_view<vxl_byte> compute_diff_image(const vil_image_view<vxl_byte> &img_1,
                                            const vil_image_view<vxl_byte> &img_2)
{
  vil_image_view<vxl_byte> out_img( img_1.ni(), img_2.nj(), 1, 3 );
  out_img.fill(0);

  // compute diff image
  vil_image_view<vxl_byte>::const_iterator itr_1   = img_1.begin();
  vil_image_view<vxl_byte>::const_iterator itr_2   = img_2.begin();
  vil_image_view<vxl_byte>::iterator       itr_out = out_img.begin();

  const vxl_byte threshold = 3;

  while( itr_1 != img_1.end() ) 
  {
    if (*itr_1 && *itr_2) { // avoid those uncertain depth measurments
      if ( *itr_1 > *itr_2 ) 
      {
        const vxl_byte d =  *itr_1 - *itr_2;
        if (d > threshold) {
          *(itr_out     )  =  vcl_min( 255, d+80);
          *(itr_out + 1 )  =  vcl_min( 255, d+80);
          *(itr_out + 2 )  =  vcl_min( 255, d*2+100);
        }
      }
      else if ( *itr_1 < *itr_2 ) 
      {
        const vxl_byte d = *itr_2 - *itr_1;
        if ( d > threshold ) 
        {
          *(itr_out     ) =  vcl_min( 255, d*2+100);
          *(itr_out + 1 ) =  vcl_min( 255, d+80);
          *(itr_out + 2 ) =  vcl_min( 255, d+80);
        }
      }
    }
    itr_1   += 1;
    itr_2   += 1;
    itr_out += 3;
  }
  assert( itr_2 == img_2.end() ); // just to ensure img_1 and img_2 has same num pixels

  return out_img;
}

vil_image_view<vxl_byte> colorize_flow(const vil_image_view<vxl_byte> &depth_img,
                                       const vil_image_view<double>   &vx,
                                       const vil_image_view<double>   &vy)
{
  vil_image_view<vxl_byte> out_img( vx.ni(), vx.nj(), 1, 3 );
  out_img.fill(0);

  vil_image_view<double>::const_iterator   vx_itr = vx.begin();
  vil_image_view<double>::const_iterator   vy_itr = vy.begin();
  vil_image_view<vxl_byte>::const_iterator d_itr  = depth_img.begin();
  vil_image_view<vxl_byte>::iterator       c_itr  = out_img.begin();
  assert( vx.size() == vy.size() );
  const double eps = 0.001;
  while( vx_itr!=vx.end() )  // should also check that vy_itr and c_itr is not at end
  {
    if ( *d_itr > 0 )  { // mask out the uncertain depth measurements
      if ( *vy_itr > eps ) // positive
      {
        *(c_itr     ) =  80;
        *(c_itr + 1 ) =  80;
        *(c_itr + 2 ) = 250;
      }
      else if ( *vy_itr < -eps )  // negative
      {
        *(c_itr     ) = 255;
        *(c_itr + 1 ) =  80;
        *(c_itr + 2 ) =  80;
      }
      // We have already done a out_img.fill(0)
      // else // zero
      // {
      //     *(c_itr     ) = 0;
      //     *(c_itr + 1 ) = 0;
      //     *(c_itr + 2 ) = 0;
      // }
    }

    vx_itr += 1;
    vy_itr += 1;
    d_itr  += 1;
    c_itr  += 3;
  }
  // vcl_cerr << vcl_endl;

  return out_img;
}

// get the body motion detected flag
bool pu_prv_motion_estimate_process::get_body_motion_detected_flag(void)
{
	if(mh_scalar_magnitude_ > mh_scalar_magnitude_high_thresh_) {
		return true;
	}
	else {
		return false;
	}
}

// get the no body motion flag
bool pu_prv_motion_estimate_process::get_no_body_motion_flag(void)
{
	if(mh_scalar_magnitude_ < mh_scalar_magnitude_low_thresh_) {
		return true;
	}
	else {
		return false;
	}
}

const vil_image_view<vxl_byte> &pu_prv_motion_estimate_process::cur_frame(void) const
{
  return vil_image_view<vxl_byte>();
}

void pu_prv_motion_estimate_process::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;
}

void pu_prv_motion_estimate_process::visualize(void)
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

void pu_prv_motion_estimate_process::visualize_overlay(void)
{
	// show whether the pose estimate info
	if( viz_ ) {
		// show motion magnitude information
		std::ostringstream buf;
		buf << "motion = " << mh_scalar_magnitude_;
		
		viz_->set_foreground(1, 0, 1);
		viz_->add_text( 0, 48, buf.str() );			
	}
}
