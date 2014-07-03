// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef gevxl_pu_prv_rectify_kinect_process_h
#define gevxl_pu_prv_rectify_kinect_process_h

#include <framework/process.h>

#include <vid/frame_process.h>

#include <img/visualizer_2d.h>
#include <img/visualizer_image.h>

#include <util/on_off_mixin.h>

#include <vgl/vgl_point_3d.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {

class pu_prv_rectify_kinect_process : public gevxl::framework::process, public gevxl::util::on_off_mixin
{
public:

  // Constructor
  pu_prv_rectify_kinect_process(char const *name="gevxl::pressure_ulcer::prevention::pu_prv_rectify_kinect_process"); 

  // Destructor
  virtual ~pu_prv_rectify_kinect_process(void); 

  // Configure this proc
  virtual bool configure(gevxl::util::config_file &config);

  //: stepping function
  bool step() { vcl_cerr << "Use step(xyzrgb_img)" << vcl_endl; return false; }

  //: stepping function
  bool step( const vil_image_view<vxl_uint_16> depth_img, const vil_image_view<float> &xyzrgb_img );

  //: output the rectified height image 
  const vil_image_view<float> &rectified_xyz_frame() const { return rectified_xyz_img_; }

  //: output the transformed origin and the x,y,z coordinate axis.
  bool get_transformed_coordinate_system(vgl_point_3d<float>  &origin,  // origin
                                         vgl_vector_3d<float> &vx,      // orthonormal basis vector vx
                                         vgl_vector_3d<float> &vy,      // orthonormal basis vector vy
                                         vgl_vector_3d<float> &vz );    // orthonormal basis vector vz

  //: output the height filtered image
  const vil_image_view<vxl_byte> &height_filtered_frame() const { return height_filtered_img_; }

  //: output the depth filtered image
  const vil_image_view<vxl_byte> &depth_filtered_frame() const { return depth_filtered_img_; }

  //: convert an xyz to an rgb image
  vil_image_view<vxl_byte> xyz2rgb( const vil_image_view<float> & xyz_img );

  //: visualization method
  void visualize( int start_x, int start_y, gevxl::img::visualizer_2d *viz);

private:

  // the configuration file that the generator needs in order to configure its own parameters.
  gevxl::util::config_file config_;

  // Input raw 16bit depth image
  vil_image_view<vxl_uint_16> depth_img_;

  // A rectified height image 
  vil_image_view<float> rectified_xyz_img_;

  // A filtered image based on the rectified height ranging criterion
  vil_image_view<vxl_byte> height_filtered_img_;

  // A filtered image based on the original depth ranging criterion
  vil_image_view<vxl_byte> depth_filtered_img_;

  // Basis vectors and new origin used for rectifying the depth image
  vgl_point_3d<float> pt0_;  // origin
  vgl_vector_3d<float> vx_, vy_, vz_;
  int m0i_, m0j_, m1i_, m1j_, m2i_, m2j_; // pixel coord chosen on the original depth image to calculate the basis vectors 

  bool orthogonal_basis_found_;

  bool get_orthogonal_basis_from_plane( 
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
    , int                  & m2j );

  bool get_orthogonal_basis_from_plane2( 
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
    , vgl_vector_3d<float> & vz );

  bool rectify_height(  
    /* Input Params */
    const vil_image_view<float> & xyzrgb_img
    , const vgl_point_3d<float>   & pt0  // origin
    , const vgl_vector_3d<float>  & vx  // orthonormal basis vectors
    , const vgl_vector_3d<float>  & vy 
    , const vgl_vector_3d<float>  & vz 
    /* Output Params */
    , vil_image_view<float> & rectified_xyz_img
    , vil_image_view<vxl_byte> & out_height_filtered_img
    );

  void perform_depth_filtering(void);

  float min_height_thresh_;
  float max_height_thresh_;

  float min_depth_thresh_;
  float max_depth_thresh_;

  vcl_vector<int> plane_fitting_three_img_points_coordinates_;

};  // end of class pu_prv_rectify_kinect_process

typedef vbl_shared_pointer<pu_prv_rectify_kinect_process> pu_prv_rectify_kinect_process_sptr;

		} // end of prevention namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace


#endif  gevxl_pu_prv_rectify_kinect_process_h
