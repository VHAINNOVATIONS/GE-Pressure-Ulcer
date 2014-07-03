//GE

#include "saliency.h"
#include <vid/gray_frame_process.h>
#include <threading/scoped_lock.h>
#include <vul/vul_timer.h>
#include <img/visualizer_image.h>
#include <img/filters.h>
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


saliency::saliency()
{

}

saliency::~saliency()
{

}

//: compute the saliency map, where num_iterations tells you how far to look ahead and max_j defines how far up and 
//  down we are willing to look.
void saliency::compute_saliency_map(vil_image_view<vxl_byte> intensity_img, double max_j,  int num_iterations)
{
  int ni = intensity_img.ni();
  int nj = intensity_img.nj();
  saliency_img_.set_size(ni,nj);
  in_chain_links_.set_size(ni,nj);
  vil_image_view<float> tmp_saliency_img(ni,nj);
  out_chain_links_.set_size(ni,nj);
  
  int i,j,k,m,n;

  // Initialize the saliency image 
  for(i=0;i<ni;i++){
    for(j=0;j<nj;j++){
      in_chain_links_(i,j)=j;
      out_chain_links_(i,j)=j;
      saliency_img_(i,j)=intensity_img(i,j);
      tmp_saliency_img(i,j)=intensity_img(i,j);
    }
  }
  
  
  

  // now start the dynamic program

  double in_max;
  double out_max;
  double link_cost;
  double val_in;
  double val_out;

  for(k=0;k<num_iterations;k++){
    

    // figure out the new saliency levels 
    for(i=1;i<ni-1;i++){
      for(j=max_j;j<nj-max_j;j++){
	
	// figure out the new saliency

	in_max = 0;
	out_max = 0;
	in_chain_links_(i,j)=j;
	out_chain_links_(i,j)=j;
	
	for(m=j-max_j;m<j+max_j;m++){
	  link_cost = 2.0 - vcl_sqrt(((double)(m-j)*(m-j))/((double)(max_j*max_j)));
	  
	  val_in = saliency_img_(i-1,m);
	  val_in = val_in*(link_cost);
	  if(val_in > in_max){
	    in_max = val_in;
	    in_chain_links_(i,j)=m;
	  }
	  
	  val_out = saliency_img_(i+1,m);
	  val_out = val_out*link_cost;
	  if(val_out > out_max){
	    out_max = val_out;
	    out_chain_links_(i,j)=m;
	  }
	}
	// We can now set the new saliency
	tmp_saliency_img(i,j)=intensity_img(i,j)+in_max + out_max;
      }
    }
    
    // copy the tmp_saliency into saliency
    for(i=0;i<ni;i++){
      for(j=0;j<nj;j++){
	saliency_img_(i,j) = tmp_saliency_img(i,j);
      }
    }
  }
}


//: get the saliency map
vil_image_view<vxl_byte> saliency::get_saliency_image()
{
  vil_image_view<vxl_byte> sal_img;
  gevxl::img::filter::float_to_byte(saliency_img_,sal_img);
  return sal_img;
}



//: pixel i,j get the j value for pixel (i-1) that is the in pixel
int saliency::get_in_chain_link(int i, int j)
{
  return in_chain_links_(i,j);
}


//: pixel i,j get the j value for pixel (i+1) that is the in pixel
int saliency::get_out_chain_link(int i, int j)
{
  return out_chain_links_(i,j);
}

 
//: Find the pixels that are chain 
void saliency::compute_chain_img()
{
  int ni = saliency_img_.ni();
  int nj = saliency_img_.nj();
  chain_img_.set_size(ni,nj);
  chain_img_.fill(0);
  int i,j;
  for(i=1;i<ni-1;i++){
    for(j=0;j<nj;j++){
      if(saliency_img_(i,j)>0){
	// now lets see if their is a mutual link
	int in_j = in_chain_links_(i,j);
	int out_j = out_chain_links_(i,j);
	if(out_chain_links_(i-1,in_j)== j && in_chain_links_(i+1,out_j)==j){
	  // check the chain size;
	  double chain_length = compute_chain_length(i,j);
	  if(chain_length > 50){
	    chain_img_(i,j)=255;
	  }
	}
      }
    }
  }
  
  // lets try some thinning
  vil_image_view<vxl_byte> thin_chain_img;
  gevxl::img::filter::thinning_process(chain_img_,thin_chain_img);
  chain_img_ = thin_chain_img;
  
}



double saliency::compute_chain_length(int pi, int pj)
{
  double chain_length;
  int i,j;
  int c_i, c_j, n_i, n_j;


  int ni = saliency_img_.ni();
  int nj = saliency_img_.nj();
  chain_length = 0;


  // look to the left
  bool flag = true;
  c_i = pi;
  c_j = pj;
  while(flag && c_i > 0 && c_i < ni -1 && c_j >0 && c_j < nj-1){
    n_i = c_i-1;
    n_j = in_chain_links_(c_i,c_j);
    // check to see if the out point links at n_i, n_j point to c_j
    if(out_chain_links_(n_i,n_j) != c_j){
      // this is the end of the chain
      flag=false;
    }
    else{
      chain_length++;
      c_i = n_i;
      c_j = n_j;
    }
  }
 
 // look to the right
  flag = true;
  c_i = pi;
  c_j = pj;
  while(flag && c_i > 0 && c_i < ni -1 && c_j >0 && c_j < nj-1){
    n_i = c_i+1;
    n_j = out_chain_links_(c_i,c_j);
    // check to see if the out point links at n_i, n_j point to c_j
    if(in_chain_links_(n_i,n_j) != c_j){
      // this is the end of the chain
      flag=false;
    }
    else{
      chain_length++;
      c_i = n_i;
      c_j = n_j;
    }
  }
  

  return chain_length;
    


}
