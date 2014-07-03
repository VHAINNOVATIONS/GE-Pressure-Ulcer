//GE

#include "pose_estimate_proc.h"
#include <vid/gray_frame_process.h>
#include <threading/scoped_lock.h>
#include <vul/vul_timer.h>
#include <img/visualizer_image.h>
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


va_proc::va_proc(const char *name):
gevxl::framework::process(name)
{
  viz_ = 0;

  true_frame_number_=0;

  // initilialize the variable passer
  initialize_variable_passer();
}


va_proc::~va_proc()
{

}

void va_proc::initialize_variable_passer()
{
  // the key here is that if we restrict variables to those defined by the enumerations
  // then we can access the variables via the enumerations as indexes into the variable passer.
  // This makes things managable from a configuration and a gui maintenance point of view

  // for example you can get the value for bool flag bv1 by
  // variable_passer_.get_bool_varialbe(bv1);
  // sinse bv1 can be viewed as an index via the enumeration...

  // do the bools
  variable_passer_.add_bool_variable(vcl_string("bv1"),true);
  variable_passer_.add_bool_variable(vcl_string("bv2"),false);

  // do the ints 
  variable_passer_.add_int_variable(vcl_string("iv1"),0);
  variable_passer_.add_int_variable(vcl_string("iv2"),1);

  // do the floats 
  variable_passer_.add_float_variable(vcl_string("fv1"),0);
  variable_passer_.add_float_variable(vcl_string("fv2"),1);

  // do the doubles 
  variable_passer_.add_double_variable(vcl_string("dv1"),0);
  variable_passer_.add_double_variable(vcl_string("dv2"),1);

  // do the strings
  variable_passer_.add_vcl_string_variable(vcl_string("vsv1"),vcl_string("check1"));
  variable_passer_.add_vcl_string_variable(vcl_string("vsv2"),vcl_string("check2"));
}


//: allows for visualization of the regions
void va_proc::set_visualizer(gevxl::img::visualizer_2d *viz)
{
  viz->lock();
  viz_ = viz;
  viz->unlock();

  patient_pose_.set_visualizer(viz_);
}




//: initiazlize the process
bool va_proc::initialize(void)
{
  return true;
}

//: be able to configure this proc - at this point we know what the proc_choice should be
bool va_proc::configure( gevxl::util::config_file& config )
{
  // configure the variable passer
  vcl_string prefix = this->name() + vcl_string("::");
  variable_passer_.configure(config,prefix);

  // configure the patient_proc
  patient_pose_.configure(config);

  return true;
}


//: print the current configuration
void va_proc::print_configuration(vcl_string filename)
{
  vcl_string prefix = this->name() + vcl_string("::");
  variable_passer_.print_configuration(prefix,filename);
}


// the magical step function
bool va_proc::step()
{

  // get the image from the frame process
  if(!frame_proc_){
    return false;
  }

  const vil_image_view<vxl_byte> &frame=frame_proc_->cur_frame();

  // vil_save(frame,"test_proc_depth_image.jpg");
  // now we will essentially attempt the image classification process
  patient_pose_.set_depth_image(frame);

  // compute the patient pose and allow for visualization
  patient_position label;
  double confidence;
  vcl_cout << "Processing frame " << true_frame_number_ << vcl_endl;

  label = patient_pose_.compute_patient_pose(confidence,true);


  if(!viz_){
    return true;
  }

  gevxl::threading::scoped_lock lock(viz_);
  if(!viz_->is_initialized()){
    viz_->initialize();
  }

  if(viz_ && viz_->is_initialized()){

    //  if(frame_proc_ ){
    //  viz_->set_image(frame);
    // }

    // embed the true frame number
    viz_->set_foreground(0,1,0);
    vcl_string text = vul_sprintf("Frame Number %06d ",true_frame_number_);
    viz_->add_text(400,30,text);

    viz_->flush();   
  }
  return true;
}


int va_proc::get_ni()
{
  // const vil_image_view<vxl_byte> &frame=frame_proc_->cur_frame();
  // int ni = frame.ni();
  int ni = 640;
  return ni;
}


int va_proc::get_nj()
{
  // const vil_image_view<vxl_byte> &frame=frame_proc_->cur_frame();
  // int nj = frame.nj();
  int nj = 480;
  return nj;
}

