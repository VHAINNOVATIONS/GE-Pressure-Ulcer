// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_prevention_pose_estimate_process.h"

#include <vcl_iostream.h>

#include <vil/vil_save.h>
#include <vil/vil_load.h>

#include <vgl/vgl_homg_point_2d.h>
#include <rrel/rrel_homography2d_est.h>
#include <rrel/rrel_trunc_quad_obj.h>
#include <rrel/rrel_irls.h>

#include <vcl_limits.h>

#include <vul/vul_file.h>

using namespace cv;

using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer::prevention;

pu_prevention_pose_estimate_process::pu_prevention_pose_estimate_process(char const *name)
: gevxl::framework::process(name), 
  viz_(NULL),
  is_process_busy_(false),
  verbose_(0),
	motion_estimate_proc_sptr_(NULL),
	person_detect_proc_sptr_(NULL)
{
  enable(false);
}

pu_prevention_pose_estimate_process::~pu_prevention_pose_estimate_process(void)
{

}

bool pu_prevention_pose_estimate_process::configure(util::config_file &config)
{
	config_ = config;

  bool enabled = false;
  config.get_bool(name()+"::enabled", enabled);
  enable(enabled);

	verbose_ = 0;
	config.get_integer(name()+"::verbose", verbose_);

	vcl_vector<double> vec;

	// the roi quadrilateral defined in the input raw depth frame
	//vcl_vector<vnl_double_2> roi_quadrilateral_;
	vec.clear();
	config.get_vcl_vector_double(name()+"::roi_quadrilateral", vec);
	if(vec.size() != 8) {
		vcl_cerr << "pu_prevention_pose_estimate_process::configure, the roi_quadrilateral_ is not configured properly." << vcl_endl;
		return false;
	}
	roi_quadrilateral_.resize(4);
	roi_quadrilateral_[0](0) = vec[0];
	roi_quadrilateral_[0](1) = vec[1];
	roi_quadrilateral_[1](0) = vec[2];
	roi_quadrilateral_[1](1) = vec[3];
	roi_quadrilateral_[2](0) = vec[4];
	roi_quadrilateral_[2](1) = vec[5];
	roi_quadrilateral_[3](0) = vec[6];
	roi_quadrilateral_[3](1) = vec[7];

	// the re-scaled rectangle defined in the canonical space
	//int roi_canonical_rectangle_width_;
	//int roi_canonical_rectangle_height_;

	roi_canonical_rectangle_width_ = 240;
	config.get_integer(name()+"::roi_canonical_rectangle_width", roi_canonical_rectangle_width_);
	
	roi_canonical_rectangle_height_ = 240;
	config.get_integer(name()+"::roi_canonical_rectangle_height", roi_canonical_rectangle_height_);

	vcl_vector<vnl_double_2> roi_canonical_rectangle;
	roi_canonical_rectangle.resize(4);
	roi_canonical_rectangle[0](0) = 0;
	roi_canonical_rectangle[0](1) = 0;
	roi_canonical_rectangle[1](0) = roi_canonical_rectangle_width_;
	roi_canonical_rectangle[1](1) = 0;
	roi_canonical_rectangle[2](0) = roi_canonical_rectangle_width_;
	roi_canonical_rectangle[2](1) = roi_canonical_rectangle_height_;
	roi_canonical_rectangle[3](0) = 0;
	roi_canonical_rectangle[3](1) = roi_canonical_rectangle_height_;

	// the inverse of the homography that maps the canonical rectangle to the roi quadrilateral
	//vnl_float_3x3 inverse_homography_;
	if( !estimate_homography(roi_canonical_rectangle, roi_quadrilateral_, inverse_homography_) ) {
		vcl_cerr << "pu_prevention_pose_estimate_process::configure, the estimate_homography failed." << vcl_endl;
		return false;
	}
  
	// the inverse homography map
	//vil_image_view<int> inverse_homography_map_;
	inverse_homography_map_ = vil_image_view<int>(roi_canonical_rectangle_width_, roi_canonical_rectangle_height_, 1, 2);
	inverse_homography_map_.fill(0);
	create_homography_map(inverse_homography_, 640, 480, inverse_homography_map_); 

	// allocate the memory for the canonical z frame
	canonical_z_frame_ = vil_image_view<float>(roi_canonical_rectangle_width_, roi_canonical_rectangle_height_, 1, 1);
	canonical_z_frame_.fill(0);

	// the number of x (along the bed head direction) and y (along the bed side direction) grids 
	num_of_x_grids_ = 12;
	config.get_integer(name()+"::num_of_x_grids", num_of_x_grids_);

	num_of_y_grids_ = 12;
	config.get_integer(name()+"::num_of_y_grids", num_of_y_grids_);

	// allocate the memory for the grid z frame
	grid_z_frame_ = vil_image_view<float>(num_of_x_grids_, num_of_y_grids_, 1, 1);
	grid_z_frame_.fill(0);

	// allocate the memory for the z_diff_feature_
	z_diff_feature_.resize( num_of_y_grids_ * (num_of_x_grids_*(num_of_x_grids_ - 1)/2) );

	// the video full filename used for collecting the training data
	training_video_data_full_filename_ = "";
	config.get_string(name()+"::training_video_data_full_filename", training_video_data_full_filename_);
	if(training_video_data_full_filename_ != "") {
		training_data_feature_label_ofstream_.open(training_video_data_full_filename_.c_str());
	}

	// classifier's configurable parameters
	classifier_choice_ = "decision_tree";	// "svm", "k_nearest_neighbor"
	config.get_string(name()+"::classifier_choice", classifier_choice_);

	decision_tree_max_depth_ = 8;
	config.get_integer(name()+"::decision_tree_max_depth", decision_tree_max_depth_);

	num_of_cross_validations_ = 4;
	config.get_integer(name()+"::num_of_cross_validations", num_of_cross_validations_);

	svm_C_ = 1;
	config.get_integer(name()+"::svm_C", svm_C_);

	k_nearest_neighbor_ = 5;
	config.get_integer(name()+"::k_nearest_neighbor", k_nearest_neighbor_);

  decision_tree_classifier_full_filename_ = "";
  config.get_string(name()+"::decision_tree_classifier_full_filename", decision_tree_classifier_full_filename_);

  svm_classifier_full_filename_ = "";
  config.get_string(name()+"::svm_classifier_full_filename", svm_classifier_full_filename_);

  knearest_classifier_full_filename_ = "";
  config.get_string(name()+"::knearest_classifier_full_filename", knearest_classifier_full_filename_);

  // load the previously trained classifiers
  bool succeeded = load_pose_classifier();
  if(!succeeded) {
	  // the pose label groundtruth data full filename and then train the classifiers
	  all_training_data_feature_label_full_filename_ = "";
	  config.get_string(name()+"::all_training_data_feature_label_full_filename", all_training_data_feature_label_full_filename_);
	  if(all_training_data_feature_label_full_filename_ != "") {
		  all_training_data_feature_label_ifstream_.open(all_training_data_feature_label_full_filename_.c_str());
		  learn_pose_classifier();
	  }
    else {
      vcl_cerr << "pu_prevention_pose_estimate_process::configure, can not train the classifiers because the all_training_data_feature_label_full_filename_ is not set." << vcl_endl;
		  return false;
    }
  }

  // perform the majority voting based final pose estimate decision making
	reset_classifier_labels_count();

	return true;
}

bool pu_prevention_pose_estimate_process::initialize(void)
{
	return true;
}

void pu_prevention_pose_estimate_process::uninitialize(void)
{

}

bool pu_prevention_pose_estimate_process::step(const gevxl::vid::frame_tag &tag,                     // the frame tag
                                               const vil_image_view<vxl_uint_16> &raw_depth_frame,   // the raw depth frame
                                               const vil_image_view<vxl_byte> &filtered_depth_frame, // the filtered depth frame
                                               const vil_image_view<float> &rectified_xyz_frame,     // the rectified xyz frame
                                               const vgl_point_3d<float>  &origin,                   // origin
                                               const vgl_vector_3d<float> &vx,                       // orthonormal basis vector vx
                                               const vgl_vector_3d<float> &vy,                       // orthonormal basis vector vy
                                               const vgl_vector_3d<float> &vz)                       // orthonormal basis vector vz 
{
	//-- Visualize the process
	//visualize();
	canonical_z_frame_.fill(0);
	grid_z_frame_.fill(0);

	//const float fNAN = std::numeric_limits<float>::quiet_NaN();
	const float fNAN = 0;

	// create the canonical_z_frame_ by cropping from the rectified_xyz_frame
	int x_to, y_to, x_from, y_from;
	float mean_z = 0;
	int mean_z_count = 0;

	for(y_from = 0; y_from < canonical_z_frame_.nj(); y_from++) {
		for(x_from = 0; x_from < canonical_z_frame_.ni(); x_from++) {
			x_to = inverse_homography_map_(x_from, y_from, 0);
			y_to = inverse_homography_map_(x_from, y_from, 1);

			canonical_z_frame_(x_from, y_from) = rectified_xyz_frame(x_to, y_to, 2);

			if( canonical_z_frame_(x_from, y_from) == fNAN ) {
				continue;
			}
			
			mean_z_count++;
			mean_z += canonical_z_frame_(x_from, y_from);
		}
	}
	float overall_mean_z = mean_z / mean_z_count;

	// compute the grid_z_frame_
	int y_spacing = canonical_z_frame_.nj() / grid_z_frame_.nj();
	int x_spacing = canonical_z_frame_.ni() / grid_z_frame_.ni();
	int x, y, x_start, y_start, x1, y1, x2, y2;
	
	for(y = 0; y < grid_z_frame_.nj(); y++) {
		for(x = 0; x < grid_z_frame_.ni(); x++) {
			
			y_start = y*y_spacing;
			x_start = x*x_spacing;

			mean_z = 0;
			mean_z_count = 0;
			for(y1 = 0; y1 < y_spacing; y1++) {
				for(x1 = 0; x1 < x_spacing; x1++) {
					x2 = x_start + x1;
					y2 = y_start + y1;

					if( canonical_z_frame_(x2, y2) == fNAN ) {
						continue;
					}

					mean_z_count++;
					mean_z += canonical_z_frame_(x2, y2);
				}
			}

			if(mean_z_count == 0) {
				mean_z = overall_mean_z;
			}
			else {
				mean_z = mean_z / mean_z_count;
			}

			grid_z_frame_(x, y) = mean_z;
		}
	}

	// generate the feature vector
	float z_diff = 0;
	int idx = 0;
	for(y = 0; y < grid_z_frame_.nj(); y++) {

		for(x = 0; x < grid_z_frame_.ni(); x++) {
			for(x1 = x + 1; x1 < grid_z_frame_.ni(); x1++) {
				z_diff = grid_z_frame_(x, y) - grid_z_frame_(x1, y);
				z_diff_feature_[idx] = z_diff;
				idx++;
			}
		}

	}

	// classify the sample vector
	Mat testSample(1, z_diff_feature_.size(), CV_32FC1, &(z_diff_feature_[0]) );

	if(classifier_choice_ == "decision_tree") {
		label_ = (int)(decision_tree_classifier_.predict( testSample )->value);
	}
	else if(classifier_choice_ == "svm") {
		label_ = (int)(svm_classifier_.predict( testSample ));
	}
	else if(classifier_choice_ == "k_nearest_neighbor") {
		label_ = (int)(knearest_classifier_.find_nearest( testSample, k_nearest_neighbor_ ));
	}

	// perform the majority voting based final pose estimate decision making
	if(motion_estimate_proc_sptr_ && person_detect_proc_sptr_ && 
		motion_estimate_proc_sptr_->get_no_body_motion_flag() == true && person_detect_proc_sptr_->get_persons_detected_flag() == false) {

		classifier_labels_count_[label_] = classifier_labels_count_[label_] + 1;
	}

	return true;
}

// get the majority voting based pose estimate
int pu_prevention_pose_estimate_process::get_majority_voting_based_pose_estimate(void)
{
	int max_pose_label = -1;
	int max_pose_label_count = -1;
	
	for(unsigned label = 0; label < body_pose::num_positions; label++) {
		if(max_pose_label_count < classifier_labels_count_[label]) {
			max_pose_label_count = classifier_labels_count_[label];
			max_pose_label = label;
		}
	}

	return max_pose_label;
}

void pu_prevention_pose_estimate_process::run_thread(void)
{ 
	do {
		
	}
	while(!get_time_to_stop_flag());
}

const vil_image_view<vxl_byte> &pu_prevention_pose_estimate_process::cur_frame(void) const
{
  return vil_image_view<vxl_byte>();
}

bool pu_prevention_pose_estimate_process::get_pose_changed_flag(vcl_string &current_pose)
{
  return true;
}

void pu_prevention_pose_estimate_process::set_pose_sample_label(const vcl_string &label)
{
	//enum body_pose{left_side, right_side, back_side, face_side, empty, num_positions};
	//const vcl_string body_pose_str[] = {"left_side", "right_side", "back_side", "face_side", "empty", "num_positions"};

	label_str_ = label;
	
	if(label_str_ == body_pose_str[body_pose::left_side]) {
		label_ = body_pose::left_side;
	}
	else if(label_str_ == body_pose_str[body_pose::right_side]) {
		label_ = body_pose::right_side;
	}
	else if(label_str_ == body_pose_str[body_pose::back_side]) {
		label_ = body_pose::back_side;
	}
	else if(label_str_ == body_pose_str[body_pose::face_side]) {
		label_ = body_pose::face_side;
	}
	else if(label_str_ == body_pose_str[body_pose::empty]) {		
		label_ = body_pose::empty;
	}
	else {
		label_str_ = "";
		label_ = -1;
	}

	if(label_ != -1 && training_data_feature_label_ofstream_.is_open()) {
		for(unsigned i = 0; i < z_diff_feature_.size(); i++) {
			training_data_feature_label_ofstream_ << z_diff_feature_[i] << " ";
		}
		training_data_feature_label_ofstream_ << label_ << vcl_endl;
		training_data_feature_label_ofstream_.flush();
	}
}

void pu_prevention_pose_estimate_process::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;
}

void pu_prevention_pose_estimate_process::visualize(void)
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

void pu_prevention_pose_estimate_process::visualize_overlay(void)
{
	// show whether the pose estimate info
	if( viz_ ) {
		
		viz_->set_foreground(0, 1, 0);
		viz_->add_text( 0, 28, body_pose_str[label_] );		
		viz_->add_polygon(roi_quadrilateral_);
	}
}


//estimate the homography using the 4 corners of the ROI
bool pu_prevention_pose_estimate_process::estimate_homography(const vcl_vector<vnl_double_2> &from_pts, const vcl_vector<vnl_double_2> &to_pts, vnl_float_3x3 &H)
{
	vcl_vector< vgl_homg_point_2d<double> > from, to;
  for( unsigned i = 0; i < 4; ++i ) {
    vgl_homg_point_2d<double> fp( from_pts[i][0], from_pts[i][1] );
    vgl_homg_point_2d<double> tp( to_pts[i][0], to_pts[i][1] );
    from.push_back( fp );
    to.push_back( tp );
  }
  rrel_homography2d_est est_prob( from, to );
  rrel_irls search;
  rrel_trunc_quad_obj obj;
  if( ! search.estimate( &est_prob, &obj ) ) {
    vcl_cerr << "Homography estimate failed!\n";
    return false;
  }
  
  vnl_double_3x3 h;
  est_prob.params_to_homog( search.params(), h.as_ref().non_const() );
  h /= h(2,2);
 
  //copy to H
	for (unsigned i=0;i<3;i++) {
		for (unsigned j=0;j<3;j++) {
      H(i,j) = (float) h(i,j);
		}
	}

  return true;
}

// create the homography map image, and constrain it within [x0 y0 x1 y1], and if the mapped coordinate is outside of this constraint, set the mapped value to the closed pixel within the constrained region.
void pu_prevention_pose_estimate_process::create_homography_map(const vnl_float_3x3 &H, const int img_width, const int img_height, vil_image_view<int> &map)
{
	int x_to, y_to;
	float d;

	map.fill(0);
	for(int y_from = 0; y_from < map.nj(); y_from++) {
		for(int x_from = 0; x_from < map.ni(); x_from++) {
			 
			d = H(2,0)*x_from + H(2,1)*y_from + H(2,2);
			x_to = (int)( (H(0,0)*x_from + H(0,1)*y_from + H(0,2))/d + 0.5 );
			y_to = (int)( (H(1,0)*x_from + H(1,1)*y_from + H(1,2))/d + 0.5 );

			if(x_to < 0) x_to = 0;
			if(x_to >= img_width) x_to = img_width - 1;

			if(y_to < 0) y_to = 0;
			if(y_to >= img_height) y_to = img_height - 1;

			map(x_from, y_from, 0) = x_to;
			map(x_from, y_from, 1) = y_to;
		}
	}
}

// load the previously trained classifiers
bool pu_prevention_pose_estimate_process::load_pose_classifier(void)
{
  // load back the pre-learned decision_tree_classifier_
	if(decision_tree_classifier_full_filename_ != "" && vul_file::exists(decision_tree_classifier_full_filename_) == true) {
		decision_tree_classifier_.clear();
		decision_tree_classifier_.load(decision_tree_classifier_full_filename_.c_str());
	}

  // load back the pre-learned svm_classifier_
	if(svm_classifier_full_filename_ != "" && vul_file::exists(svm_classifier_full_filename_) == true) {
		svm_classifier_.clear();
		svm_classifier_.load(svm_classifier_full_filename_.c_str());
	}

  // load back the pre-learned knearest_classifier_
	if(knearest_classifier_full_filename_ != "" && vul_file::exists(knearest_classifier_full_filename_) == true) {
		knearest_classifier_.clear();
		knearest_classifier_.load(knearest_classifier_full_filename_.c_str());
	}

  return true;
}

// train the classifiers using the loaded training feature data with the ground truth label
void pu_prevention_pose_estimate_process::learn_pose_classifier(void)
{
	vcl_vector<int> num_of_samples_per_category(body_pose::num_positions, 0);
	
	vcl_vector<vcl_vector<float> > samples;
	vcl_vector<int> labels;

	int label;
	float val;	
	while(all_training_data_feature_label_ifstream_ >> val) {
		z_diff_feature_[0] = val;
		for(unsigned i = 1; i < z_diff_feature_.size(); i++) {
			all_training_data_feature_label_ifstream_ >> val;
			z_diff_feature_[i] = val;
		}
		all_training_data_feature_label_ifstream_ >> label;

		samples.push_back(z_diff_feature_);
		labels.push_back(label);

		num_of_samples_per_category[label]++;
	}

	for(unsigned i = 0; i < num_of_samples_per_category.size(); i++) {
		vcl_cout << "learn_pose_classifier, category " << body_pose_str[i] << " has " << num_of_samples_per_category[i] << " samples." << vcl_endl;
	}

	// start the training
	unsigned num_of_samples = labels.size();
	unsigned dim_of_sample = z_diff_feature_.size();
	
	float **samples_pt = new float*[num_of_samples];
	for(unsigned i = 0; i < num_of_samples; i++) {
		samples_pt[i] = new float[dim_of_sample];
	}

	// assign the training data values and start the classifier training
	for(unsigned i = 0; i < num_of_samples; i++) {		
		for(unsigned k = 0; k < dim_of_sample; k++) {
			samples_pt[i][k] = samples[i][k];
		}
	}

	Mat labelsMat(num_of_samples, 1, CV_32FC1);
	Mat trainingDataMat(num_of_samples, dim_of_sample, CV_32FC1);

	for(unsigned i = 0; i < num_of_samples; i++) {
		labelsMat.at<float>(i,0) = labels[i];		
		for(unsigned k = 0; k < dim_of_sample; k++) {
			trainingDataMat.at<float>(i,k) = samples[i][k];
		}
	}

	int wrongly_classified = 0;
	double true_positive_rate = 0.0;

	// learn the decision_tree_classifier_
	CvDTreeParams dt_params;
	dt_params.max_depth = decision_tree_max_depth_;
	dt_params.min_sample_count = 2;
	dt_params.use_surrogates = false;
	dt_params.cv_folds = num_of_cross_validations_; // the number of cross-validation folds
	dt_params.use_1se_rule = false;
	dt_params.truncate_pruned_tree = false;

	bool learning_success = decision_tree_classifier_.train( trainingDataMat, CV_ROW_SAMPLE, labelsMat, Mat(), Mat(), Mat(), Mat(), dt_params );

	if(learning_success) {
		vcl_vector<int> num_of_samples_per_category(body_pose::num_positions, 0);

		// test and validate the classifier
		wrongly_classified = 0;
		for(unsigned i = 0; i < num_of_samples; i++) {
			Mat testSample(1, dim_of_sample, CV_32FC1, samples_pt[i] );
			int response = (int)decision_tree_classifier_.predict( testSample )->value;

			if(response != labels[i]) {
				wrongly_classified++;
			}

			num_of_samples_per_category[response]++;
		}
		vcl_cout << "decision_tree_classifier_, wrongly classified samples = " << wrongly_classified << " out of the total samples = " << num_of_samples << "." << vcl_endl;
		true_positive_rate = (double)(num_of_samples-wrongly_classified)/(double)(num_of_samples);
		vcl_cout << "the classification accuracy = " << true_positive_rate << "." << vcl_endl;

		for(unsigned i = 0; i < num_of_samples_per_category.size(); i++) {
			vcl_cout << "decision_tree_classifier_ validation, category " << body_pose_str[i] << " has " << num_of_samples_per_category[i] << " samples." << vcl_endl;
		}
		vcl_cout << "finished learning and validating the decision_tree_classifier_." << vcl_endl;

    // save out the learned decision_tree_classifier_
    if(decision_tree_classifier_full_filename_ != "") {
      decision_tree_classifier_.save(decision_tree_classifier_full_filename_.c_str());
    }
	}
	else {
		vcl_cout << "decision_tree_classifier_, learning classifier failed." << vcl_endl;
	}

	// learn the svm_classifier_
	CvSVMParams svm_params;
	svm_params.svm_type = CvSVM::C_SVC;
  svm_params.kernel_type = CvSVM::POLY; //CvSVM::LINEAR;
  svm_params.degree = 0.5;
  svm_params.gamma = 1;
  svm_params.coef0 = 1;
  svm_params.C = svm_C_;
  svm_params.nu = 0.5;
  svm_params.p = 0;
  svm_params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 1000, 0.01);

	learning_success = svm_classifier_.train( trainingDataMat, labelsMat, Mat(), Mat(), svm_params );

	if(learning_success) {
		vcl_vector<int> num_of_samples_per_category(body_pose::num_positions, 0);

		// test and validate the classifier
		wrongly_classified = 0;
		for(unsigned i = 0; i < num_of_samples; i++) {
			Mat testSample(1, dim_of_sample, CV_32FC1, samples_pt[i] );
			int response = (int)(svm_classifier_.predict( testSample ));

			if(response != labels[i]) {
				wrongly_classified++;
			}

			num_of_samples_per_category[response]++;
		}
		vcl_cout << "svm_classifier_, wrongly classified samples = " << wrongly_classified << " out of the total samples = " << num_of_samples << "." << vcl_endl;
		true_positive_rate = (double)(num_of_samples-wrongly_classified)/(double)(num_of_samples);
		vcl_cout << "the classification accuracy = " << true_positive_rate << "." << vcl_endl;

		for(unsigned i = 0; i < num_of_samples_per_category.size(); i++) {
			vcl_cout << "svm_classifier_ validation, category " << body_pose_str[i] << " has " << num_of_samples_per_category[i] << " samples." << vcl_endl;
		}
		vcl_cout << "finished learning and validating the svm_classifier_." << vcl_endl;

    // save out the learned svm_classifier_
    if(svm_classifier_full_filename_ != "") {
      svm_classifier_.save(svm_classifier_full_filename_.c_str());
    }
	}
	else {
		vcl_cout << "svm_classifier_, learning classifier failed." << vcl_endl;
	}

	// learn the knearest_classifier_
	learning_success = knearest_classifier_.train( trainingDataMat, labelsMat, Mat(), false, k_nearest_neighbor_ );

	if(learning_success) {
		vcl_vector<int> num_of_samples_per_category(body_pose::num_positions, 0);

		// test and validate the classifier
		wrongly_classified = 0;
		for(unsigned i = 0; i < num_of_samples; i++) {
			Mat testSample(1, dim_of_sample, CV_32FC1, samples_pt[i] );
			int response = (int)(knearest_classifier_.find_nearest( testSample, k_nearest_neighbor_ ));

			if(response != labels[i]) {
				wrongly_classified++;
			}

			num_of_samples_per_category[response]++;
		}
		vcl_cout << "knearest_classifier_, wrongly classified samples = " << wrongly_classified << " out of the total samples = " << num_of_samples << "." << vcl_endl;
		true_positive_rate = (double)(num_of_samples-wrongly_classified)/(double)(num_of_samples);
		vcl_cout << "the classification accuracy = " << true_positive_rate << "." << vcl_endl;

		for(unsigned i = 0; i < num_of_samples_per_category.size(); i++) {
			vcl_cout << "knearest_classifier_ validation, category " << body_pose_str[i] << " has " << num_of_samples_per_category[i] << " samples." << vcl_endl;
		}
		vcl_cout << "finished learning and validating the knearest_classifier_." << vcl_endl;

    // save out the learned knearest_classifier_
    if(knearest_classifier_full_filename_ != "") {
      knearest_classifier_.save(knearest_classifier_full_filename_.c_str());
    }
	}
	else {
		vcl_cout << "knearest_classifier_, learning classifier failed." << vcl_endl;
	}

	// clean up the memory space for the training data	
	for(unsigned i = 0; i < num_of_samples; i++) {
		delete [](samples_pt[i]);
	}
	delete []samples_pt;
}
