//GE

#include "nearest_neighbor_classifier.h"
#include <vid/gray_frame_process.h>
#include <threading/scoped_lock.h>
#include <vul/vul_timer.h>
#include <img/visualizer_image.h>
#include <img/filters.h>
#include <vul/vul_sprintf.h>
#include <vil/vil_save.h>
#include <vul/vul_sprintf.h>

#include <vcl_fstream.h>
#include <util/config_file.h> 
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <util/rectangle.h>
#include <vil/vil_image_view.h>
#include <vil/vil_save.h>
#include <img/visualizer_image.h>
#include <vul/vul_sprintf.h>
#include <vil/vil_load.h>
#include "patient_pose.h"

using namespace gesec::detectors;
using namespace gesec;


nearest_neighbor_classifier::nearest_neighbor_classifier()
{

}

nearest_neighbor_classifier::~nearest_neighbor_classifier()
{

}


//: we can define a file structure that is of the form
//  num_samples
//  file_name label 
//  file_name label 
//  file_name label
//  ...
//  where file_name is the name of the depth image used for processing
//  and label is the name of the file as defined by left_side, back_side or right_side
void nearest_neighbor_classifier::read_labelled_samples(vcl_string filename)
{
  int num_samples;
  vcl_string depth_file;
  vcl_string label_str;
  vcl_ifstream infile(filename.c_str());
  infile >> num_samples;
  vcl_cout << "Reading " << num_samples << " samples " << vcl_endl;
  int i;
  for(i=0;i<num_samples;i++){
    infile >> depth_file >> label_str;

    // save the label
    sample_labels_.push_back(label_str);
    
    // save the depth file
    sample_file_names_.push_back(depth_file);
    
    // read in the depth file;
    vil_image_view<vxl_byte> depth_img = vil_load(depth_file.c_str());

    // compute a segmentation image, this will use the patient pose methods
    vil_image_view<vxl_byte> seg_img = this->compute_segmentation_image(depth_img);

    // compute a distance image
    vil_image_view<vxl_byte> dist_img = this->compute_distance_image(seg_img);

    // save the distance image
    sample_distance_images_.push_back(dist_img);
    
  }
}


// given a new image can we compute the set of scores between 0 and 1 where 
// 0 is a low match and 1 is a high match. We send the file_name so that
// when we comapre against a list of file names, we don't inadvertantly
// compre the same image to the same image.

void nearest_neighbor_classifier::compute_match_scores(vil_image_view<vxl_byte> depth_img,
						       vcl_vector<double> &scores,
						       vcl_vector<vcl_string> &labels,
						       vcl_string file_name)
{
  // compute the segmentation image from the depth img;
  vil_image_view<vxl_byte> seg_img = this->compute_segmentation_image(depth_img);
  
  // get the boundary points;
  vcl_vector<vcl_vector<double> > boundary_points;
  boundary_points = this->compute_boundary_points(seg_img);
  
  // now compute the scores and associated labels
  int i;
  labels.clear();
  scores.clear();
  
  for(i=0;i<sample_distance_images_.size();i++){
    labels.push_back(sample_labels_[i]);
    
    vcl_string match_file_name = vul_sprintf("test_match_%03d_.jpg",i);
    double score = compute_match_score(boundary_points,sample_distance_images_[i],match_file_name,true);
    if(file_name == sample_file_names_[i]){
      // These are the same file, so this sample does not get to vote,
      // we therefore set the score to zero
      score=0;
    }
    // now save the score
    scores.push_back(score);
  }
}


// wraps code needed for comparing against 
vil_image_view<vxl_byte> nearest_neighbor_classifier::compute_segmentation_image(vil_image_view<vxl_byte> depth_img)
{
  // One idea here is that we will get some sort of binary image. We want to normalize in a way 
  // that would allow for efficient search. So we could do somehting like make sure that the bounding
  // box of the image is centered on the center of the image.

  vil_image_view<vxl_byte> seg_img;
  
  if(!patient_pose_){
    vcl_cout << "Warning, the nearest neighbor classifier needs its patient_pose member to be set" << vcl_endl;
    return seg_img;
  }

  // have the patient_pose_ do this
  seg_img = patient_pose_->depth_to_body_image(depth_img);
  
  // to make search a little easier, we will start by computing the bounding box of the segmentation
  // and then centering the bounding box;
  
  gevxl::util::rectangle<int> bb;
  int i,j;
  vil_image_view<vxl_byte> cen_seg_img(seg_img.ni(),seg_img.nj());
  cen_seg_img.fill(0);
  
  for(i=0;i<seg_img.ni();i++){
    for(j=0;j<seg_img.nj();j++){
      if(seg_img(i,j) >0){
	bb.update(i,j);
      }
    }
  }
  
  // now figure out the deltas needed to shift
  int delta_i = 0.5*( seg_img.ni() - (bb.x1 - bb.x0)) - bb.x0;
  int delta_j = 0.5*( seg_img.nj() - (bb.y1 - bb.y0)) - bb.y0;
  
  
  // now center everything
  int di,dj;
  for(i=0;i<seg_img.ni();i++){
    for(j=0;j<seg_img.nj();j++){
      if(seg_img(i,j)>0){
	di = i + delta_i;
	if(di < 0){
	  di = 0;
	}
	if(di > seg_img.ni()-1){
	  di = seg_img.ni()-1;
	}
    
	dj = j + delta_j;
	if(dj < 0){
	  dj = 0;
	}
	if(dj > seg_img.nj()-1){
	  dj = seg_img.nj()-1;
	}
	
	cen_seg_img(di,dj)=seg_img(i,j);
      }
    }
  }
  return seg_img;
}


// given a sementation image, compute a distance image. A distance image is known as the hoffsdorf image
// which encodes the distance of any pixel to a boundary point
vil_image_view<vxl_byte> nearest_neighbor_classifier::compute_distance_image(vil_image_view<vxl_byte> segmentation_image)
{
  int ni = segmentation_image.ni();
  int nj = segmentation_image.nj();
  vil_image_view<vxl_byte> dist_img(ni,nj);
  dist_img.fill(100);
  vcl_vector<vcl_vector<double> > boundary_points = compute_boundary_points(segmentation_image);

  int i,j,m,n,r,t,v;
  double dist;

  for(v=0;v<boundary_points.size();v++){
    i=boundary_points[v][0];
    j=boundary_points[v][1];

    for(m = -100;m<=100;m++){
      for(n= -100;n<=100;n++){
	r=i+m;
	t=j+n;
	if(r>=0 && r<ni){
	  if(t>=0 && t<nj){
	    dist = vcl_sqrt(double(m*m + n*n));
	    if(dist < dist_img(r,t)){
	      dist_img(r,t)=dist;
	    }
	  }
	}
      }
    }
  }
    

  return dist_img;
}


// for genearal testing process we can expose the raw image to image comparison 
// this assumes that all initial image processing has been done.
double nearest_neighbor_classifier::compute_match_score(vil_image_view<vxl_byte> segmentation_img, vil_image_view<vxl_byte> distance_img)
{
  double match_score;
  
  vcl_vector<vcl_vector<double> > boundary_points = this->compute_boundary_points(segmentation_img);
  match_score = compute_match_score(boundary_points,distance_img,vcl_string("match_img.jpg"),false);
  
  return match_score;
}

// for genearal testing process we can expose the raw image to image comparison 
// this assumes that all initial image processing has been done.
double nearest_neighbor_classifier::compute_match_score(vcl_vector<vcl_vector<double> > &boundary_points, 
							vil_image_view<vxl_byte> distance_img,
							vcl_string match_image_file_name, 
							bool save_match_image)
{
  // start by looking for a match with plus or minus 5, up to 25 pixels away
  int big_delta_x;
  int big_delta_y; 
  double best_big_delta_x = 0;
  double best_big_delta_y = 0;
  double best_big_score = 0;
  double score;

  // search plus or minus 25 pixels 
  for(big_delta_x = -5; big_delta_x <= 5; big_delta_x=big_delta_x+5){
    for(big_delta_y = -5; big_delta_y <= 5; big_delta_y = big_delta_y+5){
      score = this->compute_match_score(boundary_points,distance_img,big_delta_x,big_delta_y);
      if(score > best_big_score){
	best_big_score = score;
	best_big_delta_x = big_delta_x;
	best_big_delta_y = big_delta_y;
      }
    }  
  }
  
  // now refine the search
  int small_delta_x;
  int small_delta_y; 
  double best_small_delta_x = 0;
  double best_small_delta_y = 0;
  double best_small_score = 0;

  for(small_delta_x = best_big_delta_x - 3; small_delta_x <= best_big_delta_x + 3; small_delta_x++){
    for(small_delta_y = best_big_delta_y -3; small_delta_y <= best_big_delta_y + 3; small_delta_y++){
      score = this->compute_match_score(boundary_points,distance_img,small_delta_x,small_delta_y);
      if(score > best_small_score){
	best_small_score = score;
	best_small_delta_x = small_delta_x;
	best_small_delta_y = small_delta_y;
      }
    }  
  }
    
  if(save_match_image){
    vil_image_view<vxl_byte> match_img = this->merge_boundary_pixels_with_distance_image(boundary_points,
											 best_small_delta_x,
											 best_small_delta_y,
											 distance_img);
    vil_save(match_img,match_image_file_name.c_str());
  }


  return best_small_score;
}


// this version performs the actual score associated with a given delta which would be applied to
// the boundary_points
double nearest_neighbor_classifier::compute_match_score(vcl_vector<vcl_vector<double> > &boundary_points, 
							vil_image_view<vxl_byte> distance_img,
							int delta_x,
							int delta_y)
{
  double match_score= 0;
  double dist;
  int i;
  int m,n;
  double num_points = 0;
  // make list of residules
  vcl_vector<double> residules;
  int ni,nj;
  ni = distance_img.ni();
  nj = distance_img.nj();

  for(i=0;i<boundary_points.size();i++){
    m=boundary_points[i][0] + delta_x;
    n=boundary_points[i][1] + delta_y;
    if(m>=0 && m < ni && n>=0 && n < nj){
      dist= distance_img(m,n);
      residules.push_back(dist);
    }
  }
  
  // get the mean 80 percentile residule
  vcl_vector<double> new_residules = bubble_sort_residules(residules);

  double num_residules = new_residules.size();
  
  int index = 0.8*num_residules;
  double base_distance = new_residules[index];
  
  // lets assume that 100 is the largest possible distance
  // we want the score to be one when there is a tight fit
  // and 0 when there is a poor fit
  double score = 1.0 - base_distance/100.0;
  return score;
}


//: this function will perform a bubble sort on residules going from the smallest to the largest,
//  this is a basic function needed for robust estimators.
vcl_vector<double> nearest_neighbor_classifier::bubble_sort_residules(vcl_vector<double> &residules)
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




// In order to make a comparison more efficient, we can extract from
// the image a set of boundary points. Given such a list we can quickly
// compare compute the average distance of each point to a second image
// by using a distance image. 
vcl_vector<vcl_vector<double> > nearest_neighbor_classifier::compute_boundary_points(vil_image_view<vxl_byte> segmentation_image)
{
  vcl_vector<vcl_vector<double> > boundary_points;
  
  // We will just go through each pixel in the segemenation image 
  // and determine whether or not it is a boundary point
  int i,j;
  int m,n;
  bool flag;
  for(i=1;i<segmentation_image.ni()-1;i++){
    for(j=1;j<segmentation_image.nj()-1;j++){
     
      if(segmentation_image(i,j)>0){
	flag = false;
	for(m=i-1;m<i+2;m++){
	  for(n=j-1;n<j+2;n++){
	    if(segmentation_image(m,n)==0){
	      // pixel i,j is a boudnary pixel
	      flag = true;
	    }
	  }
	}
	if(flag){
	  // this is a boundary pixel
	  vcl_vector<double> p(2);
	  p[0]=i;
	  p[1]=j;
	  boundary_points.push_back(p);
	}
      }
    }
  }
  return boundary_points;
}


// set the patient pose class
void nearest_neighbor_classifier::set_patient_pose(patient_pose *pp)
{
  patient_pose_ = pp;
}

// for convinience we may want to contrast stretch the distance image
vil_image_view<vxl_byte> nearest_neighbor_classifier::constrast_stretch_distance_image(vil_image_view<vxl_byte> dist_img)
{
  vil_image_view<vxl_byte> cs_dist_img;
  gevxl::img::filter::contrast_stretch(dist_img,cs_dist_img);
  return cs_dist_img;
}


// we may also want to visualize a set of boundary points on a distance image 
vil_image_view<vxl_byte> nearest_neighbor_classifier::merge_boundary_pixels_with_distance_image(vcl_vector<vcl_vector<double> > &boundary, 
												int dx, 
												int dy, 
												vil_image_view<vxl_byte> dist_img)
{
  vil_image_view<vxl_byte> cs_img = this->constrast_stretch_distance_image(dist_img);
  int ni = cs_img.ni();
  int nj = cs_img.nj();
  vil_image_view<vxl_byte> merge_img(ni,nj,1,3);
  int i,j;
  int val;
  for(i=0;i<ni;i++){
    for(j=0;j<nj;j++){
      val = cs_img(i,j);
      merge_img(i,j,0)=val;
      merge_img(i,j,1)=val;
      merge_img(i,j,2)=val;
    }
  }
  // now insert the boundary pixels
  int di,dj;
  int k;
  for(k=0;k<boundary.size();k++){
    di = dx + boundary[k][0];
    dj = dy + boundary[k][1];
    if(di >= 0 && di <ni && dj >=0 && dj < nj){
      merge_img(di,dj,0)=255;
      merge_img(di,dj,1)=0;
      merge_img(di,dj,2)=0;
    }
  }
  return merge_img;
}

