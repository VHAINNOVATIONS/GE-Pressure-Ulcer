// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include <pressure_ulcer/prevention/pu_prv_rectify_kinect_process.h>

#include <vgl/vgl_homg_point_3d.h>
#include <vgl/algo/vgl_fit_plane_3d.h>
#include <vgl/vgl_closest_point.h>

#include <vnl/vnl_random.h>

#include <vcl_limits.h>

#include <vid/openni2_frame_process.h>

using namespace gevxl;
using namespace gevxl::pressure_ulcer::prevention;

// Constructor
pu_prv_rectify_kinect_process::pu_prv_rectify_kinect_process( char const * name ) 
: gevxl::framework::process(name),
  source_proc_(NULL),
  orthogonal_basis_found_(false)
{
}

// Destructor
pu_prv_rectify_kinect_process::~pu_prv_rectify_kinect_process(void) 
{
}

// Configure this proc
bool pu_prv_rectify_kinect_process::configure(gevxl::util::config_file &config)
{
  min_height_thresh_ = 0.10;  // minimum height to the ground plane to be 0.10 meter = 10 centimeter
  config.get_float(name()+"::min_height_thresh", min_height_thresh_);

  max_height_thresh_ = 2.50;  // maximum height to the ground plane to be 2.50 meter = 250 centimeter
  config.get_float(name()+"::max_height_thresh", max_height_thresh_);

  min_depth_thresh_ = 0.50;  // minimum depth to the camera to be 0.50 meter = 50 centimeter
  config.get_float(name()+"::min_depth_thresh", min_depth_thresh_);

  max_depth_thresh_ = 3.00;  // maximum depth to the camera to be 3.00 meter = 300 centimeter
  config.get_float(name()+"::max_depth_thresh", max_depth_thresh_);

  plane_fitting_three_img_points_coordinates_.clear();
  config.get_vcl_vector_int(name()+"::plane_fitting_three_img_points_coordinates", plane_fitting_three_img_points_coordinates_);
  if(plane_fitting_three_img_points_coordinates_.size() != 6) plane_fitting_three_img_points_coordinates_.clear();

  return true;
}

 bool pu_prv_rectify_kinect_process::get_orthogonal_basis_from_plane2( 
    /* Input params*/
    const vil_image_view<float> & xyzrgb_img
    , int m0i  // pixel coords on depth image chosen for computing the basis
    , int m0j
    , int m1i  // pixel coords on depth image chosen for computing the basis
    , int m1j
    , int m2i  // pixel coords on depth image chosen for computing the basis
    , int m2j 
    /* Output params */
    , vgl_point_3d<float>  & pt0  // origin
    , vgl_vector_3d<float> & vx   // orthonormal basis vectors
    , vgl_vector_3d<float> & vy 
    , vgl_vector_3d<float> & vz )
{
  if(xyzrgb_img(m0i,m0j,2) == 0 || xyzrgb_img(m1i,m1j,2) == 0 || xyzrgb_img(m2i,m2j,2) == 0) {
    return false;
  }

  pt0 = vgl_point_3d<float>( xyzrgb_img(m0i,m0j,0), xyzrgb_img(m0i,m0j,1), xyzrgb_img(m0i,m0j,2));
  vgl_point_3d<float> pt1(   xyzrgb_img(m1i,m1j,0), xyzrgb_img(m1i,m1j,1), xyzrgb_img(m1i,m1j,2));
  vgl_point_3d<float> pt2(   xyzrgb_img(m2i,m2j,0), xyzrgb_img(m2i,m2j,1), xyzrgb_img(m2i,m2j,2));

  vx = normalize(pt1 - pt0);
  vy = normalize(pt2 - pt0);
  vz = cross_product( vx, vy );   // vx cross product vy -> vz
  
  vy = cross_product( vz, vx ); // ensure vy is orthogonal to vy, vz cross product vx -> vy

  return true;  
}

bool pu_prv_rectify_kinect_process::get_orthogonal_basis_from_plane( 
  /* Input params*/
  const vil_image_view<float> & xyzrgb_img
  /* Output params */
  , vgl_point_3d<float>  & pt0  // origin
  , vgl_vector_3d<float> & vx   // orthonormal basis vectors
  , vgl_vector_3d<float> & vy 
  , vgl_vector_3d<float> & vz 
  , int                  & m0i  // pixel coords on depth image chosen for computing the basis
  , int                  & m0j
  , int                  & m1i  // pixel coords on depth image chosen for computing the basis
  , int                  & m1j
  , int                  & m2i  // pixel coords on depth image chosen for computing the basis
  , int                  & m2j
  )
{
  //----------------------------------------------------------------------
  // Strategy 1 to construct an orthogonal basis, i.e., sample 3 points
  //----------------------------------------------------------------------
  // pick the point on the planar surface used for rectification and 
  // recover a set of orthonormal basis

  //             v02
  //       pt 0 ---> pt 2
  //       |
  // v01   |
  //       V
  //     pt 1
  // 
  const int ni = xyzrgb_img.ni(); // width 
  const int nj = xyzrgb_img.nj(); // height 

  // Initializing pt 0
  // The kinect depth image is padded with additional rows and columns of zeros.
  // Scan row by row unti we hit the first non-zero depth value 
  m0i = m0j = 0;
  bool all3_found = false; // used for terminating the double for loop
  for (int j=30; j<nj && !all3_found; ++j) {
    for (int i=30; i<ni && !all3_found; ++i) {
      if (xyzrgb_img(i,j,2) > 0) {
        m0i = i; 
        m0j = j;

        // Initializing pt 1 (scan along vertical column col=m0i, i.e., changing j values )
        m1i = m1j = 0;
        bool m1_found = false; // used for terminating the for loop
        for (int j=m0j+15; j<nj && !m1_found; ++j) { // 15 seems arbitrary enough
          if (xyzrgb_img(m0i,j,2) > 0) {
            m1_found = true;
            m1i      = m0i; 
            m1j 	 = j;
          }
        } // for loop searching for m1

        // Initializing pt 2 (scan along horizontal row=m0j, i.e., changing i values )
        m2i = m2j = 0;
        bool m2_found = false; // used for terminating the for loop
        for (int i=m0i+15; i<ni && !m2_found; ++i) { // 15 seems arbitrary enough
          if (xyzrgb_img(i,m0j,2) > 0) {
            m2_found   = true;
            m2i        = i; 
            m2j        = m0j;
          }
        } // for loop searching for m2

        all3_found = m1_found && m2_found;
      } 
    }
  } // double for loop searching for m0
  if (!all3_found) {
    return false; // since we can't find suitable points for constructing the basis 
  }

  //----------------------------------------------------------------------
  // Strategy 2 to construct an orthogonal basis, i.e., extract normal 
  // from plane fitting 
  //----------------------------------------------------------------------
  // Collect set of candidate points for plane fitting 
  const int i_last = vcl_min( m0i+20, ni);
  const int j_last = vcl_min( m0j+20, nj);
  const int require_candidate_pool_size = 200;
  vcl_vector< vgl_homg_point_3d<float> > candidate_pool;
  candidate_pool.reserve( require_candidate_pool_size );
  for (int j=m0j; j<xyzrgb_img.nj(); j+=3) {
    for (int i=0; i<xyzrgb_img.ni(); i+=10) {
      if (xyzrgb_img(i,j,2) > 0) { // valid z value
        candidate_pool.push_back( vgl_homg_point_3d<float>( xyzrgb_img(i,j,0), xyzrgb_img(i,j,1),xyzrgb_img(i,j,2) ));
      }
    }
  }

  // 10 Iterations of fitting (with point sets selected randomly 
  // and hopefully we can succeed to fit within the tolerance level)
  vnl_random rand_generator( 12345 );
  const float tolerance = 0.05f;
  vgl_homg_plane_3d<float> plane; 
  bool plane_fitting_is_good = false;
  for (int num_iter=0; num_iter<10; ++num_iter){
    // select a random set of points (resevoir sampling)
    const int num_pts_required= 15;
    vcl_vector< vgl_homg_point_3d<float> > pts;
    pts.reserve( num_pts_required );
    for (unsigned i=0; i<candidate_pool.size(); ++i) {
      if (i<num_pts_required) { // fill up first n position
        pts.push_back(candidate_pool[i] );
      } else { // start replacing
        const int j = rand_generator(i);
        if ( j < num_pts_required ) {
          pts[j] = candidate_pool[i];
        }
      }
    }
    // fit and record
    vgl_fit_plane_3d<float> plane_fitter( pts );
    plane_fitting_is_good = plane_fitter.fit( tolerance );
    if (plane_fitting_is_good) {
      plane = plane_fitter.get_plane();
      break;
    }
  }

  pt0 = vgl_point_3d<float>( xyzrgb_img(m0i,m0j,0), xyzrgb_img(m0i,m0j,1), xyzrgb_img(m0i,m0j,2));
  vgl_point_3d<float> pt1(   xyzrgb_img(m1i,m1j,0), xyzrgb_img(m1i,m1j,1), xyzrgb_img(m1i,m1j,2));
  vgl_point_3d<float> pt2(   xyzrgb_img(m2i,m2j,0), xyzrgb_img(m2i,m2j,1), xyzrgb_img(m2i,m2j,2));

  // Construct the orthonormal basis -- using plane fitted normal
  if (plane_fitting_is_good) {
    vz = normalize(vgl_vector_3d<float>( plane.a(), plane.b(), plane.c() ));
    vgl_point_3d<float> prj_pt1 = vgl_closest_point( plane, vgl_homg_point_3d<float>(pt1) ); 
    vgl_point_3d<float> prj_pt2 = vgl_closest_point( plane, vgl_homg_point_3d<float>(pt2) ); 
    vx = normalize(prj_pt2 - pt0);
    vy = normalize(prj_pt1 - pt0);
  } else {
    // this is the less robust way to estimate an orthogonal basis, using 3 points
    vx = normalize(pt1 - pt0);
    vy = normalize(pt2 - pt0);
    vz = cross_product( vx, vy );   // vx cross product vy -> vz
  }
  vy = cross_product( vz, vx ); // ensure vy is orthogonal to vy, vz cross product vx -> vy

  return true;
}

bool pu_prv_rectify_kinect_process::rectify_height(  
  const vil_image_view<float> & xyzrgb_img
  , const vgl_point_3d<float>   & pt0  // origin
  , const vgl_vector_3d<float>  & vx  // orthonormal basis vectors
  , const vgl_vector_3d<float>  & vy 
  , const vgl_vector_3d<float>  & vz
  , vil_image_view<float> & out_xyz_img
  , vil_image_view<vxl_byte> & out_height_filtered_img)
{
  assert( xyzrgb_img.size() > 0 );

  // assume the images are allocated on contiguous space so that we can use iterators
  vil_image_view<float>::iterator out_xyz_itr = out_xyz_img.begin();
  vil_image_view<vxl_byte>::iterator out_height_filtered_itr = out_height_filtered_img.begin();
  vil_image_view<float>::const_iterator xyzrgb_itr  = xyzrgb_img.begin();

  float height_range = max_height_thresh_ - min_height_thresh_;

  // Now start rectifying the height
  const float fNAN = std::numeric_limits<float>::quiet_NaN();
  out_xyz_img.fill( 0 ); // always initialize, otherwise, prev result will appear in the "black spots"
  while (xyzrgb_itr != xyzrgb_img.end() ) {

    const vgl_point_3d<float>  p( *xyzrgb_itr, *(xyzrgb_itr + 1), *(xyzrgb_itr + 2));
    const vgl_vector_3d<float> v = p - pt0;
    
    if ( *(xyzrgb_itr+2) > 0 ) {  
      *(out_xyz_itr    ) = dot_product( v, vx );  
      *(out_xyz_itr + 1) = dot_product( v, vy ); 
      *(out_xyz_itr + 2) = dot_product( v, vz ); 
    } 
    else {
      // kinect did not compute a depth value
      *(out_xyz_itr    ) = dot_product( v, vx );  
      *(out_xyz_itr + 1) = dot_product( v, vy ); 
      *(out_xyz_itr + 2) = fNAN;
    }

    //--- debugging
    if (*(out_xyz_itr + 2) > 0.05) {
      float z_val = *(out_xyz_itr + 2);
    }
    else if (*(out_xyz_itr + 2) > 0.04) {
      float z_val = *(out_xyz_itr + 2);
    }
    else if (*(out_xyz_itr + 2) > 0.03) {
      float z_val = *(out_xyz_itr + 2);
    }
    else if (*(out_xyz_itr + 2) > 0.02) {
      float z_val = *(out_xyz_itr + 2);
    }
    else if (*(out_xyz_itr + 2) > 0.01) {
      float z_val = *(out_xyz_itr + 2);
    }
    //--- end of debuggging

    if(*(out_xyz_itr + 2) < min_height_thresh_ || 
       *(out_xyz_itr + 2) > max_height_thresh_ ||
       *(xyzrgb_itr+2) == 0) {
      
      // outside of the height threshold range or no depth value
      (*out_height_filtered_itr) = 0;
    }
    else {
      // the valid height filtered grayscale image output will be in the range from [1 to 255];
      (*out_height_filtered_itr) = (int)( ((*(out_xyz_itr + 2) - min_height_thresh_)/height_range)*254 ) + 1; 
      (*out_height_filtered_itr) = vcl_min<int>((*out_height_filtered_itr), 255);
      (*out_height_filtered_itr) = vcl_max<int>((*out_height_filtered_itr), 1);
    }

    out_xyz_itr += 3;
    out_height_filtered_itr += 1;

    xyzrgb_itr  += 6; 
  }

  return true;
}

void pu_prv_rectify_kinect_process::perform_depth_filtering(void)
{
  if(!source_proc_) return;

  const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_->get_frame_process());
  if(!openni2_source) return;

  
  const vil_image_view<vxl_uint_16> depth_frame = openni2_source->cur_depth_frame();
	
  vil_image_view<vxl_byte>::iterator out_depth_filtered_itr = depth_filtered_img_.begin();
  
	vil_image_view<vxl_uint_16>::const_iterator depth_itr  = depth_frame.begin();
	vil_image_view<vxl_byte>::const_iterator height_filtered_itr = height_filtered_img_.begin();

  float depth_val = 0.0;
  float depth_range = max_depth_thresh_ - min_depth_thresh_;

  while(depth_itr != depth_frame.end()) {
    
    depth_val = (*depth_itr);
    depth_val = depth_val / 1000; // convert to meter

		if(depth_val < min_depth_thresh_ || 
       depth_val > max_depth_thresh_) {
      
      // outside of the height threshold range or no depth value
      (*out_depth_filtered_itr) = 0;
    }
    else {
      // the valid height filtered grayscale image output will be in the range from [1 to 255];
      (*out_depth_filtered_itr) = (int)( ((depth_val - min_depth_thresh_)/depth_range)*254 ) + 1; 
      (*out_depth_filtered_itr) = vcl_min<int>((*out_depth_filtered_itr), 255);
      (*out_depth_filtered_itr) = vcl_max<int>((*out_depth_filtered_itr), 1);
    }

		if((*height_filtered_itr) == 0) {
			// the height filtered image returns 0 in this pixel, so let's filter out the depth pixel here too.
			(*out_depth_filtered_itr) = 0;
		}

    out_depth_filtered_itr++;
    depth_itr++;
		height_filtered_itr++;
  }
}

// The main step function
bool pu_prv_rectify_kinect_process::step( const vil_image_view<float> &xyzrgb_img )
{
  if ( rectified_xyz_img_.ni() != xyzrgb_img.ni() // ensure the image we are writing out to is initialized
    || rectified_xyz_img_.nj() != xyzrgb_img.nj() ) {
    rectified_xyz_img_ = vil_image_view<float>(xyzrgb_img.ni(), xyzrgb_img.nj(), 1, 3);
  }
  rectified_xyz_img_.fill(0);

  if ( height_filtered_img_.ni() != xyzrgb_img.ni() // ensure the image we are writing out to is initialized
    || height_filtered_img_.nj() != xyzrgb_img.nj() ) {
    height_filtered_img_ = vil_image_view<vxl_byte>(xyzrgb_img.ni(), xyzrgb_img.nj(), 1, 1);
  }
  height_filtered_img_.fill(0);

  if ( depth_filtered_img_.ni() != xyzrgb_img.ni() // ensure the image we are writing out to is initialized
    || depth_filtered_img_.nj() != xyzrgb_img.nj() ) {
    depth_filtered_img_ = vil_image_view<vxl_byte>(xyzrgb_img.ni(), xyzrgb_img.nj(), 1, 1);
  }
  depth_filtered_img_.fill(0);

  // Calculate the orthonormal basis used for rectification
  if(!orthogonal_basis_found_) {
    if(plane_fitting_three_img_points_coordinates_.size() != 6) {
      // use the automatic plane fitting points finding method
      orthogonal_basis_found_ = get_orthogonal_basis_from_plane( xyzrgb_img, pt0_, vx_, vy_, vz_, m0i_, m0j_, m1i_, m1j_, m2i_, m2j_);
    }
    else {
      // use the user given plane fitting three points
      m0i_ = plane_fitting_three_img_points_coordinates_[0];
      m0j_ = plane_fitting_three_img_points_coordinates_[1];

      m1i_ = plane_fitting_three_img_points_coordinates_[2];
      m1j_ = plane_fitting_three_img_points_coordinates_[3];
      
      m2i_ = plane_fitting_three_img_points_coordinates_[4];
      m2j_ = plane_fitting_three_img_points_coordinates_[5];
      
      orthogonal_basis_found_ = get_orthogonal_basis_from_plane2( xyzrgb_img, m0i_, m0j_, m1i_, m1j_, m2i_, m2j_, pt0_, vx_, vy_, vz_);
    }
  }

	if(!orthogonal_basis_found_) return true;

	// Rectification
	rectify_height(xyzrgb_img, pt0_, vx_, vy_, vz_, rectified_xyz_img_, height_filtered_img_);

	// Perform depth pixel filtering based on the rectified height and 
	perform_depth_filtering();

  return true;
}

// ---- Color function
inline void depth2rgb( float depth_val, float max_depth, vil_image_view<vxl_byte>::iterator itr) 
{
  *(itr     ) =  0;
  *(itr + 1 ) =  0;
  *(itr + 2 ) =  0;

  const float fNAN = std::numeric_limits<float>::quiet_NaN();
  if (fNAN == depth_val) {
    *(itr     ) =  0;
    *(itr + 1 ) =  0;
    *(itr + 2 ) =  0;
  }
  else if ( -0.01 < depth_val && depth_val < 0.01) {
    *(itr     ) =     0;
    *(itr + 1 ) =   250;
    *(itr + 2 ) =   250; 
  }
  else if ( -0.02 <= depth_val  && depth_val < 0.02) 
  {
    *(itr     ) =   50;
    *(itr + 1 ) =  200;
    *(itr + 2 ) =  200; 
  }
  else if ( -0.03 <= depth_val  && depth_val < 0.03) 
  {
    *(itr     ) =  100;
    *(itr + 1 ) =  150;
    *(itr + 2 ) =  150; 
  }
  else if ( -0.04 <= depth_val  && depth_val < 0.04) 
  {
    *(itr     ) =  200;
    *(itr + 1 ) =  100;
    *(itr + 2 ) =  100; 
  }
  else if ( -0.05 <= depth_val  && depth_val < 0.05) 
  {
    *(itr     ) =  255;
    *(itr + 1 ) =   0;
    *(itr + 2 ) =   0; 
  }
  else {
    *(itr     ) =  255;
    *(itr + 1 ) =  255;
    *(itr + 2 ) =  255; 
  }


}


vil_image_view<vxl_byte> pu_prv_rectify_kinect_process::xyz2rgb( const vil_image_view<float> & xyz )
{
  assert( xyz.size() > 0 ); // this could happen in the first few frames when kinect is not initalized
  assert( xyz.nplanes() == 3 ); // x,y,z values in 3 planes

  vil_image_view<vxl_byte> rgb_img( xyz.ni(), xyz.nj(), 1, 3);
  rgb_img.fill( 0 ); // always initialize, otherwise, prev result will appear in the "black spots"

  // assume the images are allocated on contiguous space so that we can use iterators
  const float max_depth = 2.0; // meters
  vil_image_view<vxl_byte>::iterator rgb_itr = rgb_img.begin();
  vil_image_view<float>::const_iterator xyz_itr  = xyz.begin();
  while (xyz_itr != xyz.end() ) {
    depth2rgb( *(xyz_itr+2), max_depth, rgb_itr ); 
    rgb_itr += 3;
    xyz_itr += 3; 
  }


  // debugging
  // -- draw out the point chosen as base point
  const int ni = xyz.ni();
  const int nj = xyz.nj();
  for (int i=0; i<5; i++) {
    int tmp_m0i = vcl_min( m0i_+i, ni-1 );  // avoid idx out of bounds
    int tmp_m1i = vcl_min( m1i_+i, ni-1 ); 
    int tmp_m2i = vcl_min( m2i_+i, ni-1 ); 
    for (int j=0; j<5; j++) {
      int tmp_m0j = vcl_min( m0j_+j, nj-1 ); 
      int tmp_m1j = vcl_min( m1j_+j, nj-1 ); 
      int tmp_m2j = vcl_min( m2j_+j, nj-1 ); 
      rgb_img( tmp_m0i, tmp_m0j, 0 ) = 255;
      rgb_img( tmp_m0i, tmp_m0j, 1 ) =   0;
      rgb_img( tmp_m0i, tmp_m0j, 2 ) =   0;
      rgb_img( tmp_m1i, tmp_m1j, 0 ) = 255; 
      rgb_img( tmp_m1i, tmp_m1j, 1 ) =   0;
      rgb_img( tmp_m1i, tmp_m1j, 2 ) =   0;
      rgb_img( tmp_m2i, tmp_m2j, 0 ) = 255;
      rgb_img( tmp_m2i, tmp_m2j, 1 ) =   0;
      rgb_img( tmp_m2i, tmp_m2j, 2 ) =   0;
    }
  }
  //--- end of debuggin

  return rgb_img;
}

void pu_prv_rectify_kinect_process::visualize( int start_x, int start_y, gevxl::img::visualizer_2d *viz) 
{
  //-- Rectify Kinect process
  if( viz ) {
    gevxl::threading::scoped_lock lock( viz );
    viz->initialize();
    if (viz->is_initialized()) { 
      viz->paste_image( start_x, start_y, xyz2rgb(rectified_xyz_img_) );
    }
  }

}

bool pu_prv_rectify_kinect_process::get_transformed_coordinate_system(vgl_point_3d<float>  &origin,  // origin
                                                                      vgl_vector_3d<float> &vx,    // orthonormal basis vector vx
                                                                      vgl_vector_3d<float> &vy,    // orthonormal basis vector vy
                                                                      vgl_vector_3d<float> &vz )   // orthonormal basis vector vz
{
  if(orthogonal_basis_found_) {
    origin = pt0_;
    vx = vx_;
    vy = vy_;
    vz = vz_;
  }

  return orthogonal_basis_found_;
}
