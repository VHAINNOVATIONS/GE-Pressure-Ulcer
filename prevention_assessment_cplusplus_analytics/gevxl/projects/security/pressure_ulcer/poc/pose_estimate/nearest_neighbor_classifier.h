//GE

#ifndef nearest_neighbor_classifier_h
#define nearest_neighbor_classifier_h



// \file
// \author Peter Tu
// \date 2/12/2014
// \par Modifications:

// The concept here is that we will have successfully segmented
// a large number of images which will have a label (back_side, left_side, right_side)
// The goal here is to generate a match score. In this way each labelled sample could be
// viewed as an expert. So the thought here is that we can try to make a set of comparisons
// and then make a weighted decision. 

// Match socre algorithm 1:
//
//         Labeled Sample:
//               compute segmentation image from depth image.
//               compute hosdorf image from segmentation image.
//
//         Input Image:
//               compute segmentation image.
//               get list of boundary points.
//               start by alligning centers of mass.
//               attempt a range of translations and compute distance score.
//               The normalized distance score then becomes the distance measure.


#include <vnl/vnl_vector.h>
#include <vil/vil_image_view.h>
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vnl/vnl_int_2.h>
#include <vnl/vnl_int_3.h>
#include <framework/process.h>
#include <vid/frame_process.h>
#include <vid/ffmpeg_frame_process.h>
#include <img/visualizer_2d.h>
#include <detectors/classifier.h>
#include <detectors/edgel_code/edgel_code_sample.h>
#include <vid/external_frame_process.h>
#include <vid/io/ffmpeg_writer.h>
#include <vid/ffmpeg_writer_process.h>
#include <util/variable_passer.h>

namespace gesec { 
namespace detectors {

 // we want a generic structure to represent a pixel

  // forward decclare the patient_pose class
  class patient_pose;

class nearest_neighbor_classifier
{
  
 public: 
  nearest_neighbor_classifier();
  
  ~nearest_neighbor_classifier();
  
  //: we can define a file structure that is of the form
  //  num_samples
  //  file_name label 
  //  file_name label 
  //  file_name label
  //  ...
  //  where file_name is the name of the depth image used for processing
  //  and label is the name of the file as defined by left_side, back_side or right_side
  void read_labelled_samples(vcl_string filename);

  // given a new image can we compute the set of scores between 0 and 1 where 
  // 0 is a low match and 1 is a high match. We send the file_name so that
  // when we comapre against a list of file names, we don't inadvertantly
  // compare the same image to the same image.

  void compute_match_scores(vil_image_view<vxl_byte> depth_img,
			    vcl_vector<double> &scores,
			    vcl_vector<vcl_string> &labels,
			    vcl_string file_name);
 
  // wraps code needed for comparing against 
  vil_image_view<vxl_byte> compute_segmentation_image(vil_image_view<vxl_byte> depth_img);

  // given a sementation image, compute a distance image. A distance image is known as the hoffsdorf image
  // which encodes the distance of any pixel to a boundary point
  vil_image_view<vxl_byte> compute_distance_image(vil_image_view<vxl_byte> segmentation_image);
  
  // for convinience we may want to contrast stretch the distance image
  vil_image_view<vxl_byte> constrast_stretch_distance_image(vil_image_view<vxl_byte> dist_img);
  
  // we may also want to visualize a set of boundary points on a distance image 
  vil_image_view<vxl_byte> merge_boundary_pixels_with_distance_image(vcl_vector<vcl_vector<double> > &boundary, 
								     int dx, int dy, vil_image_view<vxl_byte> dist_img);

  // for genearal testing process we can expose the raw image to image comparison 
  // this assumes that all initial image processing has been done.
  double compute_match_score(vil_image_view<vxl_byte> segmentation_image, vil_image_view<vxl_byte> distance_img);

  // for genearal testing process we can expose the raw image to image comparison 
  // this assumes that all initial image processing has been done.
  double compute_match_score(vcl_vector<vcl_vector<double> > &boundary_points, 
			     vil_image_view<vxl_byte> distance_img,
			     vcl_string match_img_file_name,
			     bool save_match_img_file_name);
  
  // this version performs the actual score associated with a given delta which would be applied to
  // the boundary_points
  double compute_match_score(vcl_vector<vcl_vector<double> > &boundary_points, 
			     vil_image_view<vxl_byte> distance_img,
			     int delta_x,
			     int delta_y);
  
  
  // set the patient pose class
  void set_patient_pose(patient_pose *pp);
  
 protected:
  
  // for each sample image we will compute the distance images
  vcl_vector<vil_image_view<vxl_byte> > sample_distance_images_;
  
  // these are the labels used for each sample image
  vcl_vector<vcl_string> sample_labels_;

  // we also want to keep track of the sample_file_names
  vcl_vector<vcl_string> sample_file_names_;


  // In order to make a comparison more efficient, we can extract from
  // the image a set of boundary points. Given such a list we can quickly
  // compare compute the average distance of each point to a second image
  // by using a distance image. 
  vcl_vector<vcl_vector<double> > compute_boundary_points(vil_image_view<vxl_byte> segmentation_image);
  
  // we are going to have to use the patient pose mehtods for the purposes of computing a body segmentation image
  patient_pose *patient_pose_;


  //: this function will perform a bubble sort on residules going from the smallest to the largest,
  //  this is a basic function needed for robust estimators.
  vcl_vector<double> bubble_sort_residules(vcl_vector<double> &residules);
  
  
};
  
}} // end namespaces

#endif 

