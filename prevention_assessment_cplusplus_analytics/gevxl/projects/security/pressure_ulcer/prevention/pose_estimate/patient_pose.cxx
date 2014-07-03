//GE

#include "patient_pose.h"
#include <vid/gray_frame_process.h>
#include <threading/scoped_lock.h>
#include <vul/vul_timer.h>
#include <img/visualizer_image.h>
#include <img/filters.h>
#include <img/connected_components_scanline.h>
#include <vul/vul_sprintf.h>
#include <vil/vil_save.h>


#include <vcl_fstream.h>
#include <util/config_file.h> 
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <util/rectangle.h>
#include <vil/vil_image_view.h>
#include <vil/vil_save.h>
#include <img/visualizer_image.h>
#include <vul/vul_sprintf.h>


using namespace gesec::detectors;
using namespace gesec;


patient_pose::patient_pose()
{
  viz_ = 0;
}

patient_pose::~patient_pose()
{

}

//: allows for visualization of the regions
void patient_pose::set_visualizer(gevxl::img::visualizer_2d *viz)
{
  viz->lock();
  viz_ = viz;
  viz->unlock();
}


//: be able to configure this proc - at this point we know what the proc_choice should be
bool patient_pose::configure( gevxl::util::config_file& config )
{
  // get the depth img dimensions - depth_img_ni_, depth_img_nj_
  depth_img_ni_ = 640;
  depth_img_nj_ = 480;
  double val;
 
 if(config.get_double("patient_pose::depth_img_ni",val)){
    depth_img_ni_ = val;
  }
 
   if(config.get_double("patient_pose::depth_img_nj",val)){
    depth_img_nj_ = val;
  }

  // get the name of the floor_measurement file and then call the make measurement file
   vcl_string fmf;
   if(config.get_string("pateint_pose::floor_measurement_file",fmf)){
     // compute the floor depths
     this->compute_floor_depths(depth_img_ni_,fmf);
   }

   // load the clean beds;
   vcl_vector<vcl_string> clean_bed_names;
   int i;
   clean_beds_.clear();
   
   // load the clean
   if(config.get_vcl_vector_string("patient_pose::clean_bed_files",clean_bed_names)){
     for(i=0;i<clean_bed_names.size();i++){
       vcl_string clean_bed_name = clean_bed_names[i];
       vcl_vector<int> clean_bed;
       clean_bed = load_clean_bed(clean_bed_name);
       clean_beds_.push_back(clean_bed);
     }
     vcl_cout << clean_beds_.size() << " clean beds were loaded" << vcl_endl;
   }


   // now read the samples for the nearest neighbor classifier
   vcl_string nnc_sample_file;
   if(config.get_string("patient_pose::nearest_neighbor_classifier_sample_file",nnc_sample_file)){
     // first make sure that the nnc_ is linked to this
     nnc_.set_patient_pose(this);
     nnc_.read_labelled_samples(nnc_sample_file);
   }



  return true;
}

//: perform a number of opperations on the depth image and then produce a classification
//: set the confidence score to between 0 and 1, use the vis_flag to determine whether or not
//  intermediary resutls should be displayed
patient_position patient_pose::compute_patient_pose(double &confidence, bool vis_flag)
{
  // get a set of scores using the nearest neighbor classifiers
  vcl_vector<double> scores;
  vcl_vector<vcl_string> labels;
  vcl_string file_name("live_image");
  nnc_.compute_match_scores(depth_img_,scores,labels,file_name);
  // print the match scores and labels and find the max score;
  double max_score = 0.0 - 10000;
  vcl_string best_label = vcl_string("empty");
  
  int i;
  for(i=0;i<scores.size();i++){
    vcl_cout << "Sample " << i << " Score= " << scores[i] << " " << labels[i] << vcl_endl;
    if(scores[i] > max_score){
      max_score = scores[i];
      best_label = labels[i];
    }
  }

  vcl_string text;
  
  if(vis_flag){
    
    if(viz_){
      viz_->lock();
      
      if(!viz_->is_initialized()){
	viz_->initialize();
      }
      viz_->clear();
      viz_->set_image(depth_img_);
      viz_->set_foreground(1,0,0);
      
      viz_->add_text(30,30,best_label);
      viz_->unlock();
    }
  }
  if(best_label == vcl_string("left_side")){
    return left_side;
  }
  if(best_label == vcl_string("right_side")){
    return right_side;
  }
  if(best_label == vcl_string("back_side")){
    return back_side;
  }
  return empty;
}


//:  --------------------   basic image processing -------------------------

//: compute a smoothed image with a gausian filter
void patient_pose::gausian_x_gausian_y(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img, 
			 double sig_x, double sig_y)
{

}

  
  //: comoute a smoothed image with mexian_hat in the y and a gausian in the x
void patient_pose::gausian_x_mexican_hat_y(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img,
					   double sig_x, double sig_y)
{

}


//: perform a median filter with median disk radius
void patient_pose::median_filter(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img,
		   double disk_radius)
{

}


//: use linear interpolation to fill gaps by going up and down the columns.
void patient_pose::fill_gaps(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img)
{
  


}

//: apply a boxcar filter to the image 
void patient_pose::boxcar_x_boxcar_y(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img,
		       double dim_x, double dim_y)
{
  int ni = in_img.ni();
  int nj = in_img.nj();
  out_img.set_size(ni,nj);
  out_img.fill(0);

  double sum;
  int i,j;
  double num_val;
  for(i=dim_x;i<ni-dim_x;i++){
    for(j=dim_y;j<nj-dim_y;j++){
      int m,n;
      num_val = 0;
      sum = 0;
      for(m=i-dim_x;m<i+dim_x;m++){
	for(n=j-dim_y;n<j+dim_y;n++){
	  sum=sum+in_img(m,n);
	  num_val++;
	}
      }
      sum=sum/num_val;
      out_img(i,j)=sum;
    }
  }
}



//: comute a negative image
void patient_pose::negative_image(vil_image_view<vxl_byte> &in_img, vil_image_view<vxl_byte> &out_img)
{
  gevxl::img::filter::negative(in_img,out_img);
}



//: -------------------- low level features -------------------------------------

//: compute local maxima by processing along each column y with will result in a list of list 
//  vcl_vector<vcl_vectr<int> > local_maxima which can be indexed as local_maxima[x][i] where i is the ith maxima 
//  in the y position. We let nms = the range of non maximal suppression. We could consider some sort of hill climbing
//  method as well...
void patient_pose::compute_local_maxima(vil_image_view<vxl_byte> in_img,
					vcl_vector<vcl_vector<int> > &local_maxima,
					double nms)
{
  // 1) for each column x in the image, make a list of maxmia which defines the set of maxima along that column
  // 2) make a list of new maxima and populate with maxima with apropriate suppression
  // assume the there is only one plane
  
  
  // process each column
  // clear the current list
  local_maxima.clear();
  int i,j,k;
  double val;
  int img_ni = in_img.ni();
  int img_nj = in_img.nj();
  for(i=0;i<img_ni;i++){
    vcl_vector<int> tmp_loc_max;
    // search for maxima
    
    for(j=nms;j<img_nj-nms;j++){
      val = in_img(i,j);
      bool flag = true;
      for(k= j-nms;k<j+nms;k++){
	if(in_img(i,k) > val){
	  flag = false;
	  k=j+nms;
	}
      }
      if(flag){
	// we have a maxima
	tmp_loc_max.push_back(j);
	// pump up the value of in_img by 1 to keep from the plateau problem
	in_img(i,j)= in_img(i,j) + 1;
      }
    }
    // all local maxima have been recorded so repair the damage done by the plateau trick
    for(k=0;k<tmp_loc_max.size();k++){
      in_img(i,tmp_loc_max[k]) = in_img(i,tmp_loc_max[k]) - 1;
    }
    // now copy tmp_loc_max into the local_maxima structure
    local_maxima.push_back(tmp_loc_max);
  }
  // we are done
}


//: ------------------ mid level features ---------------------------------------

//: we may want to derive set of chains for local maxima. A chain will be a list so this will be vcl_vector<vcl_vector<vcl_vector<int> > > chains 
//: So you index by chains[chain_number][element_number][0] = x position
//                  chains[chain_number][element_number][1] = y position
// For now we will compute the list of chains by connecting the local maxima
void patient_pose::compute_chains(vcl_vector<vcl_vector<int> > &local_maxima,
	      vcl_vector<vcl_vector<vcl_vector<int> > >  &chains)
{

}


//: ----------------- classifier methods ---------------------------------------

// classify the chains to determine the patient position 
patient_position patient_pose::classify_chains(vcl_vector<vcl_vector<vcl_vector<int> > >  &chains)
{
  return back_side;
}


//: we may want to take a list of chains and produce a classification - todo 

//: ---------------------- visualization methods -------------------------------

//: visualize an image
void patient_pose::visualize_image(vil_image_view<vxl_byte> img)
{

}


//: visualize a set of local maxima see local maxima for structure defenition
void patient_pose::visualize_local_maxima(vcl_vector<vcl_vector<int> > &local_maxima,
					  double r, double g, double b, double radius)
{
  int i,j;

  if(viz_){
    viz_->lock();
    
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    // viz_->clear();
    // viz_->set_image(depth_img_);
    
    viz_->set_point_radius(radius);
    viz_->set_foreground(r,g,b);
    for(i=0;i<local_maxima.size();i++){
      for(j=0;j<local_maxima[i].size();j++){
	int y_value = local_maxima[i][j];
	viz_->add_point(i,y_value);
      }
    }
    //   viz_->flush();
    viz_->unlock();
    
  }
  
}


//: visualize the chains 
void patient_pose::visualize_chains(vcl_vector<vcl_vector<vcl_vector<int> > >  &chains,
		      double r, double g, double b, double radius)
{

}


//: visuzlize the patient position 
void patient_pose::visuzalize_patient_position(patient_position pose,
				 double r, double g, double b, double radius)
{

}

void patient_pose::set_depth_image(vil_image_view<vxl_byte> img)
{
  depth_img_ = img;
  display_img_ = depth_img_;
}

// this function computes the depth of the floor for a given column x, this will be
// called during the configuration process. All the depths will be stored exaustively in 
// the structure floor_depths_

void patient_pose::compute_floor_depths(int image_ni,vcl_string floor_depths_file)
{
  // read in the floor depths which is of the form:
  // num_points
  // x depth
  // x depth
  // ...
  //  Where x is the x coordinate of a point on the raw depth image
  //  depth is the intensity value of the depth image
  
  vcl_ifstream infile(floor_depths_file.c_str());
  int num_val;
  infile >> num_val;
  int i;
  vcl_vector<double> x_vals;
  vcl_vector<double> depth_vals;
  double x_low=10000000;
  double x_hi = 0 - x_low;
  double depth_low =0;
  double depth_hi = 0;
  
  for(i=0;i<num_val;i++){
    double x,depth;
    infile >> x >> depth;
    x_vals.push_back(x);
    depth_vals.push_back(depth);
    if(x<x_low){
      x_low = x;
      depth_low = depth;
    }
    if(x>x_hi){
      x_hi=x;
      depth_hi = depth;
    }
  }
  // now fill in the floor_depths_ structure
 
  floor_depths_.clear();
  for(i=0;i<image_ni;i++){
    // check if i < all the x_vals 
    bool flag_done = false;
    if(i<=x_low){
      floor_depths_.push_back(depth_low);
      flag_done = true;
    }
    if(i>=x_hi){
      floor_depths_.push_back(depth_hi);
      flag_done = true;
    }
    if(!flag_done){
      // we need to interpolate 
      double x0,d0,x1,d1,x,d;
      x=i;
      int j;
      for(j=0;j<x_vals.size()-1;j++){
	if(x_vals[j]<= x && x_vals[j+1] >=x && !flag_done){
	  // we have it braketted
	  x0 = x_vals[j];
	  d0 = depth_vals[j];
	  x1 = x_vals[j+1];
	  d1 = depth_vals[j+1];
	  
	  // interpolate
	  d= d0 + (d1-d0)*(x-x0)/(x1-x0);
	  floor_depths_.push_back(d);
	  flag_done = true;
	}
      }
    }
    if(!flag_done){
      vcl_cout << "Warning we could not find a depth for x value " << i << vcl_endl;
      vcl_cout << "Check logic" << vcl_endl;
    }
  }
}



//: get the processed image
vil_image_view<vxl_byte> patient_pose::get_processed_image()
{
  // generate the processed image
  this->generate_processed_image();

  // return this value
  return processed_img_;
  
}


//: a chain of processes that can be used to compute a processed image
void patient_pose::generate_processed_image()
{
  vil_image_view<vxl_byte> tmp1,tmp2;

  // make a negateive image of the depth image
  this->negative_image(depth_img_,tmp1);

  // perform a boxcar function
  this->boxcar_x_boxcar_y(tmp1,tmp2,25,3);
  
  
  // try some histogram equalization here
  gevxl::img::filter::histogram_equalization(tmp2,tmp1);

  // set the proceswed img
  processed_img_ = tmp1;
  

  // set the display img;
  display_img_ = processed_img_;
}


// going from a range image, we can make an approximate height_img_ where the 
// value of each pixel is proportional to the height of the point off of the floor
void patient_pose::generate_height_image(vil_image_view<vxl_byte> depth_img,
					 vil_image_view<vxl_byte> &height_img)
{
  // for each point in the scene, we will attempt to compute the heiht of each pixel
  // with respect to the floor. We will do this for each pixel, the key would be
  // to do this for each column...
  
  // make sure that the height image has the right dimensions
  int i,j;
  int img_ni = depth_img.ni();
  int img_nj = depth_img.nj();
  height_img.set_size(img_ni,img_nj,1);
  
  // for each column x process the height values
  for(i=0;i<img_ni;i++){
    // set the last height to 0
    double last_height = 0;
   
    // get the floor_depth for this column in the image
    double floor_depth = floor_depths_[i];
    double last_image_depth = floor_depth;

    // get the camera height which we assume will be 
    // the floor depth at column 0 
    // 
    // *** since the geometry of our arrangement is a little
    //     different, we have to assume a height position of the camera
    //     lets make is the point furthest away = img_ni - 1  

    double camera_height = floor_depths_[img_ni-1];
    
    // now process each pixel along this column i
    for(j=0;j<img_nj;j++){
      
      
      // get the image_depth for this point
      double image_depth = depth_img(i,j);
      if(image_depth < 10){
	// this is a whole in the depth image so 
	// fill it in with last valid image depth
	image_depth = last_image_depth;
      }
      last_image_depth = image_depth;
      
      // now compute the height of this pixel
      double val1 = floor_depth*floor_depth - image_depth*image_depth;
      double val2 = camera_height*camera_height;
      if(val2 < val1){
	// trouble...
	double t = val2 - val1;
      }

      double image_height = (2.0*camera_height - vcl_sqrt(4.0*val2 - 4.0*val1))/2.0;
      if(image_height < 0){
	image_height = 0;
      }
      if(image_height > 255){
	image_height  = 255;
      }

      // now set value of the height image
      height_img(i,j) = image_height;
    }
  }
  display_img_ = height_img;
}



vil_image_view<vxl_byte> patient_pose::get_height_image()
{
  generate_height_image(depth_img_,height_img_);
  return height_img_;
}

void patient_pose::test_compute_local_maxima()
{
  vcl_vector<vcl_vector<int> > local_maxima;
  compute_local_maxima(display_img_,local_maxima,100);
  visualize_local_maxima(local_maxima,1,0,0,5);
}

//: -------------------- Segmentation Methods ---------------------------------

//: OK we should be able to figure out what is person, bed and floor based on height and proximity. Given a labelling
//  of person we should be able to determine what is a left back and right....

//: we may want to look at the distribution of heights for a given region of an image this 
//  will be produced as a vector with 255 values. Each bin will have the number of pixels
//  that contain a that value.

vcl_vector<double> patient_pose::compute_intensity_distribution(vil_image_view<vxl_byte> &img, gevxl::util::rectangle<int> &box)
{

  // clear a vector
  vcl_vector<double> int_dist(256);
  int i,j;
  for(i=0;i<int_dist.size();i++){
    int_dist[i]=0;
  }
  
  int index;
  // populate the distribution
  for(i=box.x0;i<box.x1;i++){
    for(j=box.y0;j<box.y1;j++){
      index = img(i,j);
      int_dist[index] = int_dist[index] + 1;
    }
  }
  return int_dist;
  
}


//: this computes the distribution for the entire image
vcl_vector<double> patient_pose::compute_intensity_distribution(vil_image_view<vxl_byte> &img)
{
  gevxl::util::rectangle<int> box;
  box.update(0,0);
  box.update(img.ni(),img.nj());
  return compute_intensity_distribution(img,box);
}


//: we may want to save an intensity distribution to disk so that we can visualize it using xl...
void patient_pose::save_intensity_distribution(vcl_vector<double> int_dist, vcl_string filename)
{
  vcl_ofstream outfile(filename.c_str());
  int i;
  for(i=0;i<int_dist.size();i++){
    outfile << i << " " << int_dist[i] << vcl_endl;
  }
  outfile.close();
}

// gather statistics for image a column or the whole image
void patient_pose::save_intensity_distribution(vil_image_view<vxl_byte> &img, int column, vcl_string file_name)
{
  gevxl::util::rectangle<int> box;
  box.update(column,0);
  box.update(column+1,img.nj());
  vcl_vector<double> int_dist = this->compute_intensity_distribution(img,box);
  this->save_intensity_distribution(int_dist,file_name);
}


void patient_pose::save_intensity_distribution(vil_image_view<vxl_byte> &img, vcl_string file_name)
{
  vcl_vector<double> int_dist = this->compute_intensity_distribution(img);
  this->save_intensity_distribution(int_dist,file_name);
}

void patient_pose::save_intensity_distribution_image(vil_image_view<vxl_byte> &img, vcl_string file_name)
{
  vil_image_view<vxl_byte> int_dist_img = this->construct_intensity_distribution_image(img);
  vil_save(int_dist_img,file_name.c_str());
}

//: make an intensity distribution image which has for the y axis the intensity values
//  the x axis indiates the column number. The intensity is the number of pixels in each bin
vil_image_view<vxl_byte> patient_pose::construct_intensity_distribution_image(vil_image_view<vxl_byte> img)
{
  vcl_vector<vcl_vector<double> > int_dist_list;
  int i,j;
  double max=0;
  for(i=0;i<img.ni();i++){
    gevxl::util::rectangle<int> box;
    box.update(i,0);
    box.update(i+1,img.nj());
    vcl_vector<double> int_dist = this->compute_intensity_distribution(img,box);
    for(j=0;j<int_dist.size();j++){
      if(int_dist[j] > max){
	max = int_dist[j];
      }
    }
    int_dist_list.push_back(int_dist);
  }
  // make sure that max does not equal zero
  if(max ==0){
    max = 1;
  }

  // make the image;
  vil_image_view<vxl_byte> int_dist_img(img.ni(),256);
  int_dist_img.fill(0);
  for(i=0;i<int_dist_list.size();i++){
    for(j=0;j<int_dist_list[i].size();j++){
      double val = 250.0 * int_dist_list[i][j];
      val = val / max;
      
      int_dist_img(i,j)= val;
    }
  }
  // a little bit of image processing in terms of a laplacian of gausians seems to do a nice job here
  // make a float image from the int_dist_img
  vil_image_view<float> float_image, float_output;
  gevxl::img::filter::byte_to_float(int_dist_img,float_image);
  gevxl::img::filter::laplacian_of_gauss(float_image,float_output,3.0);
  gevxl::img::filter::float_to_byte(float_output,int_dist_img);
  
  // we want to floor make a floor function that basically
  // sets to zero anything that is below a certian threshold.
  
  // the best apporach might be for us to simply look for the threshold where
  // the majority of pixels are then lost. This plus 5 might be a good value
  // for now we simply set the threshold to 175.
  
  // we want to compute the cumulative distribution 
  vcl_vector<double> int_dist(256);
  for(i=0;i<256;i++){
    int_dist[i]=0;
  }
  int val;
  for(i=0;i<int_dist_img.ni();i++){
    for(j=0;j<int_dist_img.nj();j++){
      val=int_dist_img(i,j);
      int_dist[val] = int_dist[val] + 1;
    }
  }
  // now find the mode of int_dist and this plus 3 should be the threshold
  double mode = 0;
  double threshold = 0;
  for(i=0;i<int_dist.size();i++){
    if(int_dist[i]>mode){
      mode = int_dist[i];
      threshold = i;
    }
  }
  
  threshold = threshold + 3;
  
  for(i=0;i<int_dist_img.ni();i++){
    for(j=0;j<int_dist_img.nj();j++){
      if(int_dist_img(i,j)<threshold){
	int_dist_img(i,j)=0;
      }
    }
  }
  

  intensity_distribution_img_ = int_dist_img;
  return int_dist_img;
}

//: find local maxima from intensity distribution image this will be a vector of vector of vector
//  to be indexted as maxima[columnumber][maxima_index][0]= image intensity value
//  to be indexted as maxima[columnumber][maxima_index][1]= strenth associated with this value
//
//  note that we will throw out maxima associated with image intensity values less then 5 (this is noise
//  We will also suppress intensity values below 0.5 times the max value
//  We will also apply some level of local non maximal suppression. 
vcl_vector<vcl_vector<vcl_vector<double> > > patient_pose::compute_intensity_distribution_maxima(vil_image_view<vxl_byte> int_dist_img)
{
  // create the list of maxima
  vcl_vector<vcl_vector<vcl_vector<double> > > all_maxima;
  
  int i,j,k;
  // process each column i;
  for(i=0;i<int_dist_img.ni();i++){
    // make a list of maxima for column i
    vcl_vector<vcl_vector<double> > maxima_list;
    // don't consider the first five intensity values - they are holes
    // and find the maximum value
    double max = 0;
    for(j=5;j<int_dist_img.nj();j++){
      if(int_dist_img(i,j) > max){
	max = int_dist_img(i,j);
      }
    }
    
    // now find maxima with supression w = 3 intensity values
    int min_sup = 3; 
    for(j=5+min_sup;j<int_dist_img(i,j)-min_sup;j++){
      // lets see if this a local maxima
      if(int_dist_img(i,j) - 162 > 0.1 * (max - 162)){
	bool flag = true;
	for(k=j-min_sup;k<j+min_sup;k++){
	  if(int_dist_img(i,k) > int_dist_img(i,j)){
	    // this is not a local maxima
	    flag = false;
	  }
	}
	
	if(flag){
	  // this looks like a local maxima - hurray
	  vcl_vector<double> maxima;
	  maxima.push_back(j);
	  maxima.push_back(int_dist_img(i,j));
	  maxima_list.push_back(maxima);
	}
      }
    }
    all_maxima.push_back(maxima_list);
  }
  return all_maxima;
}



//: visualize the intensity distrivurion maxima
void patient_pose::visualize_intensity_distribution_maxima(vcl_vector<vcl_vector<vcl_vector<double> > > &int_dist_maxima,
							   double r, double g, double b)
{
   int i,j;
   int radius = 3;
  if(viz_){
    viz_->lock();
    
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(intensity_distribution_img_);
    
    viz_->set_point_radius(radius);
    viz_->set_foreground(r,g,b);
    for(i=0;i<int_dist_maxima.size();i++){
      for(j=0;j<int_dist_maxima[i].size();j++){
	int y_value = int_dist_maxima[i][j][0];
	viz_->add_point(i,y_value);
      }
    }
    //   viz_->flush();
    viz_->unlock();
    
  }
}

// test the search for local maxima int the intensity distribution 
void patient_pose::test_compute_intensity_distribution_maxima()
{
  vcl_vector<vcl_vector<vcl_vector<double> > > int_dist_maxima;
  int_dist_maxima = this->compute_intensity_distribution_maxima(intensity_distribution_img_);
  this->visualize_intensity_distribution_maxima(int_dist_maxima,0,1,0);
  
}


// test the saliency image generated from an intensity distribution image
void patient_pose::test_intensity_saliency()
{
  saliency_.compute_saliency_map(intensity_distribution_img_);
  vil_image_view<vxl_byte> saliency_img = saliency_.get_saliency_image();
  
  // find the chains 
  saliency_.compute_chain_img();
  vil_image_view<vxl_byte> chain_img = saliency_.get_chain_image();
  
  // fuse the chains with the saliency image
  int ni = chain_img.ni();
  int nj = chain_img.nj();
  int i,j;
  vil_image_view<vxl_byte> joint_img(ni,nj,1,3);
  for(i=0;i<ni;i++){
    for(j=0;j<nj;j++){
      joint_img(i,j,0)=saliency_img(i,j);
      joint_img(i,j,1)=saliency_img(i,j);
      joint_img(i,j,2)=saliency_img(i,j);
      if(chain_img(i,j) > 0){
	joint_img(i,j,0)=255;
	joint_img(i,j,1)=255;
	joint_img(i,j,2)=0;
      }
    }
  }
  
  // OK now lets test the thresholds and see how they perform
  this->compute_scene_thresholds(chain_img,floor_thresholds_, bed_thresholds_, body_thresholds_);
  
  // show these values using bed body and floor
  for(i=0;i<joint_img.ni();i++){
    int floor_j = floor_thresholds_[i];
    int bed_j = bed_thresholds_[i];
    int body_j = body_thresholds_[i];
    
    joint_img(i,body_j,0)=255;
    joint_img(i,body_j,1)=0;
    joint_img(i,body_j,2)=0;

    joint_img(i,bed_j,0)=0;
    joint_img(i,bed_j,1)=255;
    joint_img(i,bed_j,2)=0;

    joint_img(i,floor_j,0)=0;
    joint_img(i,floor_j,1)=0;
    joint_img(i,floor_j,2)=255;
  }
    

  display_img_ = joint_img;
  
}



// test the thinning process as applied to the intensity distribution image 
void patient_pose::test_intensity_thinning()
{

  // make a thinned image out of the intensity_distribution_img_;
  vil_image_view<vxl_byte> thinned_img;
  
  thinned_img = this->compute_intensity_thinning_image(intensity_distribution_img_);

  // OK now lets test the thresholds and see how they perform
  this->compute_scene_thresholds(thinned_img,floor_thresholds_, bed_thresholds_, body_thresholds_);
  
  // lets clean up the bed 
  int k;


  bed_thresholds_ = this->clean_your_bed(bed_thresholds_);


  int ni = thinned_img.ni();
  int nj = thinned_img.nj();


  vil_image_view<vxl_byte> joint_img(ni,nj,1,3);
  int i,j;
  for(i=0;i<ni;i++){
    for(j=0;j<nj;j++){
      joint_img(i,j,0)=intensity_distribution_img_(i,j);
      joint_img(i,j,1)=intensity_distribution_img_(i,j);
      joint_img(i,j,2)=intensity_distribution_img_(i,j);
      if(thinned_img(i,j) > 0){
	joint_img(i,j,0)=255;
	joint_img(i,j,1)=255;
	joint_img(i,j,2)=0;
      }
    }
  }

  
 // show these values using bed body and floor
  for(i=0;i<joint_img.ni();i++){
    int floor_j = floor_thresholds_[i];
    int bed_j = bed_thresholds_[i];
    int body_j = body_thresholds_[i];
  

    joint_img(i,body_j,0)=255;
    joint_img(i,body_j,1)=0;
    joint_img(i,body_j,2)=0;

    joint_img(i,bed_j,0)=0;
    joint_img(i,bed_j,1)=255;
    joint_img(i,bed_j,2)=0;

    joint_img(i,floor_j,0)=0;
    joint_img(i,floor_j,1)=0;
    joint_img(i,floor_j,2)=255;
  }
    

  display_img_ = joint_img;
  
}


// test the thinning process as applied to the intensity distribution image 
vil_image_view<vxl_byte>  patient_pose::compute_intensity_thinning_image(vil_image_view<vxl_byte> intensity_distribution_img)
{
  
  // make a thinned image out of the intensity_distribution_img_;
  vil_image_view<vxl_byte> thinned_img;
  gevxl::img::filter::thinning_process(intensity_distribution_img,thinned_img);
  
  // now perform a connected component analysis on the thinned img
  gevxl::img::connected_components_scanline cc;
  vil_image_view<int> label_img;
  int num_labels = cc.label(thinned_img,label_img);
  
  // for each label compute the number of pixels with that label;
  vcl_vector<double> label_counts(num_labels);
  int i,j;

  for(i=0;i<num_labels;i++){
    label_counts[i]=0;
  }

  int val;
  for(i=0;i<label_img.ni();i++){
    for(j=0;j<label_img.nj();j++){
      if(label_img(i,j)>=0){
	// this is a label 
	val = label_img(i,j);
	label_counts[val] = label_counts[val]+1;
      }
    }
  }
  
  // print out the size of the label counts
  /*
    vcl_cout << "label counts: " << vcl_endl;
    for(i=0;i<label_counts.size();i++){
    vcl_cout << label_counts[i] << vcl_endl;
    }
  */
  

  // now remove connected regions that are below a certain threshold 
  
  for(i=0;i<label_img.ni();i++){
    for(j=0;j<label_img.nj();j++){
      if(label_img(i,j)>=0){
	// this is a label 
	val = label_img(i,j);
	if(label_counts[val] < 30){
	  thinned_img(i,j)=0;
	}
      }
    }
  }
  thinned_img_ = thinned_img;
  return thinned_img;
}
  


// get the saliceny image
vil_image_view<vxl_byte>  patient_pose::get_saliency_img()
{
  return saliency_.get_saliency_image();
}

// this method will take the saliency methods and compute the floor (green), bed (blue) and body (red) threshold
// values for each column.

void patient_pose::compute_scene_thresholds(vil_image_view<vxl_byte> chain_img, vcl_vector<int> &floor, vcl_vector<int> &bed, vcl_vector<int> &body)
{
  
  // get the range of the chaing img
  int ni=chain_img.ni();
  int nj=chain_img.nj();
  
  // allocate the floor bed and body values
  floor = vcl_vector<int>(ni);
  bed = vcl_vector<int>(ni);
  body = vcl_vector<int>(ni);
  
  // initialize all values
  int i,j,k;
  for(i=0;i<ni;i++){
    floor[i]=0;
    bed[i]=0;
    body[i]=nj-1;
  }

  // now search from the floor from the bottom on the chain_img
  for(i=0;i<ni;i++){
    for(j=nj-1;j>=0;j=j-1){
      if(chain_img(i,j) > 0){
	floor[i]=j;
	j= -1;
      }
    }
  }
  
  // now search from the bed starting from the floor
  for(i=0;i<ni;i++){
    for(j=floor[i]-5;j>=0;j=j-1){
      if(chain_img(i,j) > 0){
	bed[i]=j;
	j= -1;
      }
    }
  }

  // now search for the body starting from the top of the image
  for(i=0;i<ni;i++){
    for(j=0;j<nj;j++){
      if(chain_img(i,j) > 0){
	body[i]=j;
	j=nj;
      }
    }
  }
}

// now we can compute the body_image which is a segmentation for the depth image
vil_image_view<vxl_byte> patient_pose::compute_body_image(vil_image_view<vxl_byte> depth_img,
							  vcl_vector<int> floor,
							  vcl_vector<int> bed,
							  vcl_vector<int> body)
{
  int ni = depth_img.ni();
  int nj = depth_img.nj();
  
  vil_image_view<vxl_byte> body_image(ni,nj);
  body_image.fill(0);
  int i,j;

  double delta;
  double max_delta;
  double min_delta;
  double alpha;
 
  // this method assumes a clean bed image that we can use to segment everything about a few (maybe 2 or 3 pixel values below the threshold) 

  for(i=0;i<ni;i++){

    // if the body is closer then the floor and the bed we are in business

    if(bed[i] > 0){
      // lets set the threshold
      double thresh = bed[i]-3;
      
      
      // now we can colour code difference in depth;
      for(j=0;j<nj;j++){
	if(depth_img(i,j) < thresh && depth_img(i,j) > 10){
	  // this is the body
	  body_image(i,j)=255;
	}
      }

    }
  }
  
  // OK here is the cocktail needed to segment the body...
  // we will do:
  // 
  // errosian
  // dilation
  // connected components
  // keep the largest bit 
  // keep big bits that are in the y values of the big bit... lets see how this works
  
  vil_image_view<vxl_byte> img_tmp1, img_tmp2;
  
  // errosian
  double r = 3;
  vil_structuring_element el;
  el.set_to_disk(r);

  gevxl::img::filter::erode(body_image,img_tmp1,el);
  gevxl::img::filter::dilate(img_tmp1,img_tmp2,el);
  
  // now perform connected components on the image
  // and find the largest components

  gevxl::img::connected_components_scanline cc;
  
  vil_image_view<int> label_img(img_tmp2.ni(),img_tmp2.nj());
  int num_labels = cc.label(img_tmp2,label_img);
  
  if(num_labels == 0){
    vcl_cout << "Warning, there is no body... " << vcl_endl;
    return img_tmp2;
  }

  // for each label compute the number of pixels with that label;
  vcl_vector<double> label_counts(num_labels);
  

  for(i=0;i<num_labels;i++){
    label_counts[i]=0;
  }
  
  int val;
  for(i=0;i<label_img.ni();i++){
    for(j=0;j<label_img.nj();j++){
      if(label_img(i,j)>=0){
	// this is a label 
	val = label_img(i,j);
	label_counts[val] = label_counts[val]+1;
      }
    }
  }
  
  // get the bounding boxes of each cluster
  vcl_vector< gevxl::util::rectangle<int> > bb;
  cc.get_bounding_boxes(bb);
  
  // make a list of all labels that we will keep;
  vcl_vector<bool> keepers(num_labels);
  for(i=0;i<num_labels;i++){
    keepers[i]=false;
  }
  
  // find the largest cluster
  int largest_cluster =0;
  int max_cluster_size = label_counts[0];
  for(i=0;i<label_counts.size();i++){
    if(label_counts[i] > max_cluster_size){
      max_cluster_size = label_counts[i];
      largest_cluster = i;
    }
  }
  keepers[largest_cluster]=true;
  
  // find any clusters that are inline with the largest cluster
  for(i=0;i<bb.size();i++){
    if(bb[i].y1 >= bb[largest_cluster].y0 && bb[i].y0 <= bb[largest_cluster].y1){
      keepers[i] = true;
    }
  }

  // OK lets transfer all the keepers  to the body image
  body_image.fill(0);
  for(i=0;i<label_img.ni();i++){
    for(j=0;j<label_img.nj();j++){
      if(label_img(i,j) >= 0){
	// this is a cluster;
	if(keepers[label_img(i,j)]){
	  // this is a keeper
	  body_image(i,j) = 255;
	}
      }
    }
  }
  

  return body_image;


 
  /* this approach uses the idea of a body a bed and a floor set of thresholds 
  for(i=0;i<ni;i++){
    // if the body is closer then the floor and the bed we are in business
    if(floor[i] > body[i] && bed[i] > body[i]+5){
      // lets set the threshold
      double thresh = 0.5 * (bed[i] + body[i]);

      // lets find the range of depth differences so we can colour code the image
      min_delta = 10000000;
      max_delta = 0.0 - min_delta;
      
      for(j=0;j<nj;j++){
	if(depth_img(i,j) < thresh && depth_img(i,j) > 10){
	  delta = thresh - depth_img(i,j);
	  if(delta > max_delta){
	    max_delta = delta;
	  }
	  if(delta < min_delta){
	    min_delta = delta;
	  }
	}
      }

      double max_minus_min = max_delta - min_delta;
      if(max_minus_min == 0){
	max_minus_min =1;
      }
      // now we can colour code difference in depth;
      for(j=0;j<nj;j++){
	if(depth_img(i,j) < thresh && depth_img(i,j) > 10){
	  delta = thresh - depth_img(i,j);
	  alpha = (delta - min_delta)/max_minus_min;
	  // this is the body
	  body_image(i,j,0)=255*alpha;
	  body_image(i,j,1)=0;
	  body_image(i,j,2)=255*(1.0-alpha);	  
	}
      }
    }
  }

  
  return body_image;
  */


}

void patient_pose::test_body_segmentation()
{
  // compute the body segmentation
  body_image_ = this->compute_body_image(depth_img_,floor_thresholds_,bed_thresholds_,body_thresholds_);
  
  // now make a joint image
  display_img_ = body_image_;
}

// Since the bed represents a somewhat static surface, when ever we see a clean bed, we can save it to disk
// We can then try to do some sort of best fit function that would be used to a new bed using a robust estimator...
void patient_pose::save_clean_bed(vcl_vector<int> &clean_bed, vcl_string filename)
{
  vcl_ofstream outfile(filename.c_str());
  int i;
  outfile << clean_bed.size() << vcl_endl;
  for(i=0;i<clean_bed.size();i++){
    outfile << clean_bed[i] << vcl_endl;
  }
  outfile.close();
}


// load a clean bed 
vcl_vector<int> patient_pose::load_clean_bed(vcl_string filename)
{
  vcl_ifstream infile(filename.c_str());
  vcl_vector<int> clean_bed;
  int num_val;
  infile >> num_val;
  int i;
  for(i=0;i<num_val;i++){
    int val;
    infile >> val;
    clean_bed.push_back(val);
  }
  return clean_bed;
}

// given the current measured bed thresholds, do a best fit to 
// the clean beds that we have observed, this will use the list of clean beds.
vcl_vector<int> patient_pose::best_fit_clean_bed(vcl_vector<int> &observed_bed, vcl_vector<vcl_vector<int> > &clean_beds)
{
  // for now we will consider a range of possible fits to the bed and then consider 
  // 50 percent of the residules. Remember that a 0 value for an obeseved bed or a clean bed means no residule...
  
  if(clean_beds.size()==0){
    // nothing to be done here
    return observed_bed;
  }

  // if the observed_bed is all zero
  // return it;
  bool fl=false;
  int i;
  for(i=0;i<observed_bed.size();i++){
    if(observed_bed[i] > 0){
      fl=true;
    }
  }
  if(!fl){
    // there was no bed
    return observed_bed;
  }


  // for now lets consider a vertical adjustment of plus or minus 20
  double delta,res;
  double best_delta;
  double best_residule;
  int j,k,l;

  int best_bed = 0;
  
  best_delta = 0;
  best_residule = 100000000;
  for(i=0;i<clean_beds.size();i++){
    for(delta = 0-20;delta < 20;delta++){
      vcl_vector<double> residules;
      residules.clear();
      vcl_vector<double> sorted_residules;
      for(j=0;j<observed_bed.size();j++){
	if(observed_bed[j] > 0 && clean_beds[i][j] >0){
	  res = clean_beds[i][j] + delta - observed_bed[j];
	  if(res < 0){
	    res = 0 - res;
	  }
	  residules.push_back(res);
	}	
      }
      // sort the residules
      sorted_residules = bubble_sort_residules(residules);
      
      // find the 70 percentile residule 
      int index = 0.7 * sorted_residules.size();
      
      double robust_residule = sorted_residules[index];
      if(robust_residule < best_residule){
	best_residule = robust_residule;
	best_delta = delta;
	best_bed = i;
      }
    }
  }
 
  // OK we now have the best bed and the best residule 
  // vcl_cout << "The best residule is " << best_residule << " this was for a delta of " << best_delta << vcl_endl;
  
  vcl_vector<int> best_fit_bed;
  for(i=0;i<clean_beds[best_bed].size();i++){
    double val;
    if(clean_beds[best_bed][i] > 0){
      val = clean_beds[best_bed][i] + best_delta;
    }
    else{
      val = 0;
    }
    best_fit_bed.push_back(val);
  }
  return best_fit_bed;
}

//: this function will perform a bubble sort on residules going from the smallest to the largest,
//  this is a basic function needed for robust estimators.
vcl_vector<double> patient_pose::bubble_sort_residules(vcl_vector<double> &residules)
{
  int i,j,k;
  double min_value;
  vcl_vector<double> sorted_list = residules;
  for(i=0;i<sorted_list.size();i++){
    min_value = sorted_list[i];
    k=i;
    for(j=i+1;j<sorted_list.size();j++){
      if(sorted_list[j] < min_value){
	min_value = sorted_list[j];
	k=j;
      }
    }
    // make the swap
    sorted_list[k] = sorted_list[i];
    sorted_list[i] = min_value;
  }
  return sorted_list;
}

//: save the current bed thresholds since they seem to be pretty good.
void patient_pose::save_as_a_clean_bed(vcl_string filename)
{
  save_clean_bed(bed_thresholds_,filename);
}


//: best fit the observed bed to the current set of clean beds
void patient_pose::best_fit_observed_bed_to_clean_beds()
{
  vcl_vector<int> new_bed_thresholds;
  new_bed_thresholds = best_fit_clean_bed(bed_thresholds_,clean_beds_);
  bed_thresholds_ = new_bed_thresholds;

  // lets assume that the display image shows the current thresholds 
  // we will get it and store the new results
  vil_image_view<vxl_byte> joint_img = display_img_;

  int i;
  
  if(display_img_.nplanes()==3){
    for(i=0;i<joint_img.ni();i++){
      int bed_j = bed_thresholds_[i];
      
      joint_img(i,bed_j,0)=0;
      joint_img(i,bed_j,1)=255;
      joint_img(i,bed_j,2)=255;
      
    }
  }

  display_img_ = joint_img;
  
}

// we want to remove outliers from the bed threshold detections. We will call this a clean your bed method
vcl_vector<int> patient_pose::clean_your_bed(vcl_vector<int> &bed_thresholds)
{
  // at this point we simply want to get rid of stragelers...
  vcl_vector<int> new_bed = bed_thresholds;
  
  // we want long continuous chains, not small stragelly bits. so lets sort this out.
  int i,i1,i2;
  int i_ok;
  for(i=0;i<bed_thresholds.size();i++){
    if(bed_thresholds[i] > 0){
      // we have a real threshold;
      // now count the number of contiguous points
      int count = 1;
      bool flag = true;
      i1=i;
      i2=i+1;
      while(i2<bed_thresholds.size() && flag){
	if(bed_thresholds[i2]==0){
	  flag = false;
	}
	if(vcl_fabs((double)(bed_thresholds[i2] - bed_thresholds[i1])) > 5){
	  flag = false;
	}
	if(flag){
	  i1=i2;
	  i2=i2+1;
	  count++;
	}
      }
      if(count < 30){
	// lets set these thresholds to 0
	for(i1=i;i1<i2;i1++){
	  new_bed[i1]=0;
	}
      }
      i=i2;
    }
  }


  return new_bed;
}

//: perform the end to end segmentation of a raw depth image
vil_image_view<vxl_byte> patient_pose::depth_to_body_image(vil_image_view<vxl_byte> depth_img)
{
  vil_save(depth_img,"test_depth_img.jpg");

  // step 1 compute the intenstity distribution
  vil_image_view<vxl_byte> intensity_distribution_img = this->construct_intensity_distribution_image(depth_img);
  vil_save(intensity_distribution_img,"test_int_dist_img.jpg");


  // step 2 perform the thinning process
  vil_image_view<vxl_byte> thinned_img = this->compute_intensity_thinning_image(intensity_distribution_img);
  vil_save(thinned_img,"test_thinned_img.jpg");

  // step 3 compute the bed thresholds
  this->compute_scene_thresholds(thinned_img,floor_thresholds_, bed_thresholds_, body_thresholds_);
  // lets clean up the bed
  bed_thresholds_ = this->clean_your_bed(bed_thresholds_);

  // step 3 fit the bed thresholds to the clean beds
  vcl_vector<int> new_bed_thresholds;
  new_bed_thresholds = best_fit_clean_bed(bed_thresholds_,clean_beds_);
  bed_thresholds_ = new_bed_thresholds;
  
  // step 4 compute the segmentation image
  vil_image_view<vxl_byte> body_img = this->compute_body_image(depth_img,floor_thresholds_, bed_thresholds_, body_thresholds_);
  vil_save(body_img,"test_body_img.jpg");

  display_img_ = body_img;
  return body_img;
}

