//GE

#ifndef saliency_h
#define saliency_h



// \file
// \author Peter Tu
// \date 2/12/2014
// \par Modifications:

// The conept here is that we want to take in an intensity image and the chain the image under the assumption
// that we are going from left to right in a conservative manner. There are all sorts of methods that one could 
// use to enforce horizontal continuity. But for now we will do this in a simple fashion. At the end of this is 
// a type of dynamic programming. But for the sake of reusability we will make this algorithm as general as possible
// with the idea that one can use inheritance to modify the approach. 


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



class saliency
{
  
 public: 
  saliency();
  
  ~saliency();

  //: compute the saliency map, where num_iterations tells you how far to look ahead and max_j defines how far up and 
  //  down we are willing to look.
  void compute_saliency_map(vil_image_view<vxl_byte> intensity_img, double max_j=5,  int num_iterations=10);

  //: get the saliency map
  vil_image_view<vxl_byte> get_saliency_image();
  
  //: pixel i,j get the j value for pixel (i-1) that is the in pixel
  int get_in_chain_link(int i, int j);
  
  //: pixel i,j get the j value for pixel (i+1) that is the in pixel
  int get_out_chain_link(int i, int j);

 
  //: Find the pixels that are chain 
  void compute_chain_img();
  
  vil_image_view<vxl_byte> get_chain_image()
    {return chain_img_;}

  // given that a pixel is part of a chain, compute the length off the chain...
  double compute_chain_length(int i, int j);

 protected:
  
  //: The saliency iamge
  vil_image_view<float> saliency_img_;
 
  //: a chain image are those pixels that are linked to pixels that recipricate the relationhsip
  vil_image_view<vxl_byte> chain_img_;

  //: for now lets assume the the in pixels are on the left of pixel (i,j) 
  //  and the out pixels are on the right of pixel (i,j) and so to keep
  //  track of the optimal pixel chain both in and out we need only keep
  //  track of the delta j.
  
  //: the in_chain_links
  vil_image_view<int> in_chain_links_;
  vil_image_view<int> out_chain_links_;

};
  
}} // end namespaces

#endif 

