//GE

#ifndef patient_pose_h
#define patient_pose_h



// \file
// \author Peter Tu
// \date 2/12/2014
// \par Modifications:

// This is a class that takes an image and attempts to classify patient pose

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
#include "saliency.h"
#include "nearest_neighbor_classifier.h"

namespace gesec { 
namespace detectors {

// an enumeration of the number of sides 
enum patient_position{left_side, right_side, back_side, empty, num_positions};
  
class patient_pose
{
  
 public: 
  patient_pose();
  
  ~patient_pose();
  
  //: allows for visualization of various features and structures
  void set_visualizer(gevxl::img::visualizer_2d *viz);

  //: be able to configure this proc - at this point we know what
  //  the proc_choice should be
  bool configure( gevxl::util::config_file& config );

  //: set the depth image
 
  void set_depth_image(vil_image_view<vxl_byte> img);

  //: get the depth image
  vil_image_view<vxl_byte> get_depth_image()
    {return depth_img_;}

  //:  get height image
  vil_image_view<vxl_byte> get_height_image();

  //: get the processed image
  vil_image_view<vxl_byte> get_processed_image();


  //: perform a number of opperations on the raw image and then produce a classification
  //: set the confidence score to between 0 and 1, use the vis_flag to determine whether or not
  //  intermediary resutls should be displayed
  patient_position compute_patient_pose(double &confidence, bool vis_flag = true);

  //: be able to get the display image
  vil_image_view<vxl_byte> get_display_image()
    {return display_img_;}

  //: get the intensity distribution image
  vil_image_view<vxl_byte> get_intensity_distribution_image()
    {return intensity_distribution_img_;}

  //: ***** methods to test various methods *****
 
  //  test the local maxima
  void test_compute_local_maxima();

  // test the search for local maxima int the intensity distribution 
  void test_compute_intensity_distribution_maxima();

  // test body segmentation
  void test_body_segmentation();

  // test the saliency image generated from an intensity distribution image
  void test_intensity_saliency();
  
  // test a thinning approach
  void test_intensity_thinning();

  // get the saliceny image
  vil_image_view<vxl_byte>  get_saliency_img();

  
  // gather statistics for image a column or the whole image
  void save_intensity_distribution(vil_image_view<vxl_byte> &img, int column, vcl_string file_name);
  void save_intensity_distribution(vil_image_view<vxl_byte> &img, vcl_string file_name);
  void save_intensity_distribution_image(vil_image_view<vxl_byte> &img, vcl_string file_name);

  //: save the current bed thresholds since they seem to be pretty good.
  void save_as_a_clean_bed(vcl_string filename);

  //: best fit the observed bed to the current set of clean beds
  void best_fit_observed_bed_to_clean_beds();
  
  //: perform the end to end segmentation of a raw depth image
  vil_image_view<vxl_byte> depth_to_body_image(vil_image_view<vxl_byte> depth_img);
  vil_image_view<vxl_byte> get_body_image()
    {return body_img_;}
						 

 protected:

   vcl_vector<patient_position> test;

  //: ******  objects *******
  //: the depth image width and height
  int depth_img_ni_;
  int depth_img_nj_;

  //: the visualizer
  gevxl::img::visualizer_2d *viz_;
  //: the depth image
  vil_image_view<vxl_byte> depth_img_;

  //: the height image
  vil_image_view<vxl_byte> height_img_;

  //: a processed image that will be used for analysis
  vil_image_view<vxl_byte> processed_img_;

  //: the intensity distribution image which can be used to pick thresholds 
  vil_image_view<vxl_byte> intensity_distribution_img_;

  //: a thinned and cleaner version of the intensity distribution image
  vil_image_view<vxl_byte> thinned_img_;

  //: the results of body segmentation, the body image
  vil_image_view<vxl_byte> body_img_;

  //: for better or for worse we will always have a designated display_img
  vil_image_view<vxl_byte> display_img_;

  //: a set of x values and associated floor depths. 
  //: we will compute these during the configuration process.
  //  you can read this as floor_depth = floor_depths_[x]
 
  vcl_vector<double> floor_depths_;

  //: ****** methods *******
 
  //: -------------------- geometric conversion methods ----------------------
  
  // going from a range image, we can make an approximate height_img_ where the 
  // value of each pixel is proporti)onal to the height of the point off of the floor
  void generate_height_image(vil_image_view<vxl_byte> depth_img,
			     vil_image_view<vxl_byte> &height_img);

  // this function computes the depth of the floor for a given column x, this will be
  // called during the configuration process. All the depths will be stored exaustively in 
  // the structure floor_depths_
  void compute_floor_depths(int image_ni,vcl_string floor_depths_file);
    
  //:  --------------------   basic image processing -------------------------
  
  //: comute a negative image
  void negative_image(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img);

  //: apply a boxcar filter to the image 
  void boxcar_x_boxcar_y(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img,
			 double dim_x, double dim_y);

  //: compute a smoothed image with a gausian filter
  void gausian_x_gausian_y(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img, 
			   double sig_x, double sig_y);
  
  //: comoute a smoothed image with mexican_hat in the y and a gausian in the x
  void gausian_x_mexican_hat_y(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img,
			  double sig_x, double sig_y);

  //: perform a median filter with median disk radius
  void median_filter(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img,
		     double disk_radius);

  //: use linear interpolation to fill gaps by going up and down the columns.
  void fill_gaps(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img);

  //: a chain of processes that can be used to compute a processed image
  void generate_processed_image();

  //: -------------------- Segmentation Methods ---------------------------------

  //: OK we should be able to figure out what is person, bed and floor based on height and proximity. Given a labelling
  //  of person we should be able to determine what is a left back and right....
  
  //: we may want to look at the distribution of heights for a given region of an image this 
  //  will be produced as a vector with 255 values. Each bin will have the number of pixels
  //  that contain a that value.
  vcl_vector<double> compute_intensity_distribution(vil_image_view<vxl_byte> &img, gevxl::util::rectangle<int> &box);

  //: this computes the distribution for the entire image
  vcl_vector<double> compute_intensity_distribution(vil_image_view<vxl_byte> &img);

  //: we may want to save an intensity distribution to disk so that we can visualize it using xl...
  void save_intensity_distribution(vcl_vector<double> int_dist, vcl_string filename);
  
  //: make an intensity distribution image which has for the y axis the intensity values
  //  the x axis indiates the column number. The intensity is the number of pixels in each bin
  vil_image_view<vxl_byte> construct_intensity_distribution_image(vil_image_view<vxl_byte> img);


  // compute a cleaned up thnned image from the intensity distribution image 
  vil_image_view<vxl_byte> compute_intensity_thinning_image(vil_image_view<vxl_byte> intensity_distribution_img);

  //: find local maxima from intensity distribution image this will be a vector of vector of vector
  //  to be indexted as maxima[columnumber][maxima_index][0]= image intensity value
  //  to be indexted as maxima[columnumber][maxima_index][1]= strenth associated with this value
  //
  //  note that we will throw out maxima associated with image intensity values less then 5 (this is noise
  //  We will also suppress intensity values below 0.5 times the max value
  //  We will also apply some level of local non maximal suppression. 
  vcl_vector<vcl_vector<vcl_vector<double> > > compute_intensity_distribution_maxima(vil_image_view<vxl_byte> int_dist_img);
  
  // this method us used to perform saliency analysis on the intensity distribution image
  saliency saliency_;

  // this method will take the saliency methods and compute the floor (green), bed (blue) and body (red) threshold
  // values for each column. 
  vcl_vector<int> floor_thresholds_;
  vcl_vector<int> bed_thresholds_;
  vcl_vector<int> body_thresholds_;

  // Since the bed represents a somewhat static surface, when ever we see a clean bed, we can save it to disk
  // We can then try to do some sort of best fit function that would be used to a new bed using a robust estimator...
  void save_clean_bed(vcl_vector<int> &clean_bed, vcl_string filename);
  
  // load a clean bed 
  vcl_vector<int> load_clean_bed(vcl_string filename);
  
  // a list of clean bed thresholds_
  vcl_vector<vcl_vector<int> > clean_beds_;

  // given the current measured bed thresholds, do a best fit to 
  // the clean beds that we have observed, this will use the list of clean beds.
  vcl_vector<int> best_fit_clean_bed(vcl_vector<int> &observed_bed, vcl_vector<vcl_vector<int> > &clean_beds);
  
  // compute the scene thresholds
  void compute_scene_thresholds(vil_image_view<vxl_byte> chain_img,vcl_vector<int> &floor, vcl_vector<int> &bed, vcl_vector<int> &body);
  
  // we want to remove outliers from the bed threshold detections. We will call this a clean your bed method
  vcl_vector<int> clean_your_bed(vcl_vector<int> &bed_thresholds);


  // now we can compute the body_image which is a segmentation for the depth image
  vil_image_view<vxl_byte> compute_body_image(vil_image_view<vxl_byte> depth_img,
					      vcl_vector<int> floor_thresholds,
					      vcl_vector<int> bed_thresholds,
					      vcl_vector<int> body_thresholds);
  
  // we should store this in the body image
  vil_image_view<vxl_byte> body_image_;

  //: -------------------- low level features -------------------------------------
  
  //: compute local maxima by processing along each column y with will result in a list of list 
  //  vcl_vector<vcl_vector<int> > local_maxima which can be indexed as local_maxima[x][i] where i is the ith maxima 
  //  in the y position. We let nms = the range of non maximal suppression. We could consider some sort of hill climbing
  //  method as well...
  void compute_local_maxima(vil_image_view<vxl_byte> in_img,
			    vcl_vector<vcl_vector<int> > &local_maxima,
			    double nms);
  
  //: ------------------ mid level features ---------------------------------------  
  //: we may want to derive set of chains for local maxima. A chain will be a list so this will be vcl_vector<vcl_vector<vcl_vector<int> > > chains 
  //: So you index by chains[chain_number][element_number][0] = x position
  //                  chains[chain_number][element_number][1] = y position
  // For now we will compute the list of chains by connecting the local maxima
  void compute_chains(vcl_vector<vcl_vector<int> > &local_maxima,
		      vcl_vector<vcl_vector<vcl_vector<int> > >  &chains);
  
  //: ----------------- classifier methods ---------------------------------------
  
  // classify the chains to determine the patient position 
  patient_position classify_chains(vcl_vector<vcl_vector<vcl_vector<int> > >  &chains);

  // this approach to classification will make use of a nearest neighbor classifier
  nearest_neighbor_classifier nnc_;
  
  //: ---------------------- visualization methods -------------------------------
  
  //: visualize an image
  void visualize_image(vil_image_view<vxl_byte> img);

  //: visualize a set of local maxima see local maxima for structure defenition
  void visualize_local_maxima(vcl_vector<vcl_vector<int> > &local_maxima,
			      double r, double g, double b, double radius);
  
  //: visualize the chains 
  void visualize_chains(vcl_vector<vcl_vector<vcl_vector<int> > >  &chains,
			double r, double g, double b, double radius);

  //: visuzlize the patient position 
  void visuzalize_patient_position(patient_position pose,
				   double r, double g, double b, double radius);
  
  //: visualize the intensity distrivurion maxima
  void visualize_intensity_distribution_maxima(vcl_vector<vcl_vector<vcl_vector<double> > > &int_dist_maxima,
					       double r, double g, double b);
  
  //: ------------------------------- convinience functions  ------------------------

  //: this function will perform a bubble sort on residules going from the smallest to the largest,
  //  this is a basic function needed for robust estimators.
  vcl_vector<double> bubble_sort_residules(vcl_vector<double> &residules);

  // for each point on a binar

  
};
  
}} // end namespaces

#endif 

