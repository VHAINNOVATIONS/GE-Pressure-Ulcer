// Copyright (C) 2006 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.
#include <img/canny_wrapper.h>
#include "va_image_client.h"
#include <gui/wx/direct_visualizer.h>
#include <vil/vil_save.h>
#include <vil/vil_load.h>
#include <vil/algo/vil_gauss_filter.h>
#include <gui/wx/settings_dialog.h>
#include <wx/filename.h>
#include <wx/filedlg.h>
#include <gui/wx/wxid.h>
#include <gui/wx/canvas.h>
#include <img/visualizer_2d.h>
#include <vul/vul_sprintf.h> 

using namespace gevxl::gui::wx;
using gevxl::gui::wx::wxid;
using namespace gesec::detectors;
using namespace gevxl::gui;
using namespace gevxl::img;

// Default constructor.
va_image_client::va_image_client(void)
{
  key_flip_ = false;
  make_floor_measurement_flag_ = false;
}


// Default destructor.
va_image_client::~va_image_client(void)
{

}

void va_image_client::update_menu(wxMenu *menu)
{
 
  append(menu,"Load Depth Image",
	 (wxCommandEventFunction)&va_image_client::load_callback);
  
  append(menu,"Depth to Body Image",
	 (wxCommandEventFunction)&va_image_client::depth_to_body_image_callback);

  append(menu,"Classify Depth Image",
	 (wxCommandEventFunction)&va_image_client::compute_patient_pose_callback);
  
  //  append(menu,"Make Floor Measurements",
  //	 (wxCommandEventFunction)&va_image_client::make_floor_measurement_callback);
  
  // append(menu,"Compute Height Image",
  //	 (wxCommandEventFunction)&va_image_client::height_image_callback);

  //  append(menu,"Compute Processed Image",
  //	  (wxCommandEventFunction)&va_image_client::processed_image_callback);
 
  // append(menu,"Compute Patient Pose",
  //	 (wxCommandEventFunction)&va_image_client::patient_pose_callback);
  
 
  //   append(menu,"Test Local Maxima",
  //       (wxCommandEventFunction)&va_image_client::test_local_maxima_callback);

  append(menu,"Image Intensity Distribution",
         (wxCommandEventFunction)&va_image_client::test_image_intensity_distribution_callback);

  //  append(menu,"Test Intensity Distribution Maxima",
  //         (wxCommandEventFunction)&va_image_client::test_intensity_distribution_maxima_callback);

  //  append(menu,"Test Intensity Saliency",
  //	 (wxCommandEventFunction)&va_image_client::test_intensity_saliency_callback);

  append(menu,"Thinning",
	 (wxCommandEventFunction)&va_image_client::test_intensity_thinning_callback);

  append(menu,"Fit to clean beds",
	 (wxCommandEventFunction)&va_image_client::best_fit_observed_bed_to_clean_beds_callback);
 
  append(menu,"Body Segmentation",
	 (wxCommandEventFunction)&va_image_client::test_body_segmentation_callback);

  append(menu,"Save as a clean bed",
	 (wxCommandEventFunction)&va_image_client::save_as_a_clean_bed_callback);

  append(menu,"Help",
         (wxCommandEventFunction)&va_image_client::help_callback);
 

}




// handle key events
void va_image_client::on_key(gevxl::gui::event_source *source, wxKeyEvent &event)
{
  if(!key_flip_){
    key_flip_ = true;
    event.Skip();
    return;
  }
  key_flip_ = false;

  if(event.MetaDown()) 
    { event.Skip(); 
      return; }
  
  // vcl_cout << "The event key code is " << event.GetKeyCode() << vcl_endl;
  switch(event.GetKeyCode())
    {
    case ',': 
      {
	break;
      }
      
    case '.': 
      {
	break;
      }
     
    default: event.Skip();
    }
  event.Skip();
}

// a help function
void va_image_client::help_callback(wxCommandEvent &event)
{
  
}

 

void va_image_client::on_mouse(gevxl::gui::event_source *source, 
				     wxMouseEvent &event)
{
  canvas *c=dynamic_cast<canvas *>(source);
  if(c==0)
    {
      event.Skip();
      return;
    }
  
  visualizer_2d *viz=source->get_visualizer();
  if(!viz) return;

  if(event.LeftDown()){
    // we should be able to create a part box using the pb_source
    // get the landmark point
    
    double x =c->get_mouse_x();
    double y =c->get_mouse_y();
    
    // we may be trying to make a floor measurement
    if(make_floor_measurement_flag_){
      make_floor_measurement_flag_ = false;
      vil_image_view<vxl_byte> depth_img = patient_pose_.get_depth_image();
      this->make_floor_measurement(x,y,floor_measurement_file_name_,depth_img);
      vcl_cout << "finished making the floor measurements" << vcl_endl;
    }
    
    // a va method for painting the point
    if(viz == viz_){
      gevxl::threading::scoped_lock ev_lock(viz_);
      viz_->clear();
      vil_image_view<vxl_byte> display_img = patient_pose_.get_display_image();
      viz_->set_image(display_img);
      viz_->set_foreground(0,1,0);
      viz_->set_point_radius(2);
      viz_->add_point(x,y);
      int inten = display_img(x,y);
	  int int_x = int(x);
	  int int_y = int(y);
      vcl_string img_string = vul_sprintf("X %03d Y %03d Intensity %03d",int_x,int_y,inten);
      vcl_cout << img_string << vcl_endl;
      viz_->add_text(20,20,img_string);		
      canvas_->post_redraw_event();
      
      // now make save the image statistics for the this column x
      patient_pose_.save_intensity_distribution(display_img,x,vcl_string("column_intensity_distribution.dat"));
      
    }
  }
}

// define where the client can pull start images from
void va_image_client::set_canvas(gevxl::gui::wx::canvas *c)
{
  canvas_ = c;
  viz_ = canvas_->get_visualizer();
  patient_pose_.set_visualizer(viz_);
}

//: configure this client
bool va_image_client::configure(gevxl::util::config_file &config)
{
  patient_pose_.configure(config);
   return true;
}
 
//: load an image for processing
void va_image_client::load_callback(wxCommandEvent &event)
{

 wxFileName fname;

  wxString new_filename = wxFileSelector(wxT("Load image as..."),fname.GetPath(),fname.GetName(),
					 wxT("*.jpg"),wxT("PNG files (*.png)|*.png|JPEG files (*.jpg)|*.jpg|All files (*.*)|*.*"),
					 wxOPEN|wxFILE_MUST_EXIST);

  if(!new_filename.empty() )
  {
    raw_img_=vil_load(new_filename.mb_str());
  }
  else{
    vcl_cout << "Could not load image" << vcl_endl;
    return;
  }
  // for now just load a specific image
  // raw_img_ = vil_load("Images/left_side/Peter_01_self_turn_00898.jpg");
 
  // show the image
  
  if(viz_){
    viz_->lock();
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(raw_img_);
    viz_->unlock();
    // viz_->flush();
  }
  canvas_->post_redraw_event();

  // tell the patient_pose_ object about the raw image
  patient_pose_.set_depth_image(raw_img_);

}

//: apply patient_pose analysis methods
void va_image_client::patient_pose_callback(wxCommandEvent &event)
{
  // set the raw image
  patient_pose_.set_depth_image(raw_img_);

  
  // compute the patient pose
  patient_position pp;
  double confidence;
  pp = patient_pose_.compute_patient_pose(confidence,true);
 
  canvas_->post_redraw_event();

}


//: generate height image callback
void va_image_client::height_image_callback(wxCommandEvent &event)
{
  vil_image_view<vxl_byte> height_img = patient_pose_.get_height_image();

  if(viz_){
    viz_->lock();
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(height_img);  
    viz_->unlock();
    // viz_->flush();
  }
  canvas_->post_redraw_event();
}



//: generate height image callback
void va_image_client::processed_image_callback(wxCommandEvent &event)
{
  vil_image_view<vxl_byte> processed_img = patient_pose_.get_processed_image();

  if(viz_){
    viz_->lock();
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(processed_img);  
    viz_->unlock();
    // viz_->flush();
  }
  canvas_->post_redraw_event();
}

void va_image_client::make_floor_measurement_callback(wxCommandEvent &event)
{
  vcl_cout << "Once you type the filename click with the left button on a row of pixels that are all on the floor" << vcl_endl;



  gevxl::gui::wx::settings_dialog *d=new gevxl::gui::wx::settings_dialog(parent());
  vcl_string file_name("floor_measurements.dat");
  d->field("File Name",file_name);
  
  if(d->show_modal()!=wxID_OK){
    // don't do anything
    return;
  }
  
  floor_measurement_file_name_ = file_name;

  make_floor_measurement_flag_ = true;

  
}

//:  this function is called from the mouse click and starts the collection of floor measurements
void va_image_client::make_floor_measurement(int x, int y, vcl_string file_name, vil_image_view<vxl_byte> depth_img)
{
  // make the floor measurements...
  vcl_vector<double> x_vals;
  vcl_vector<double> depth_values;
  double initial_depth_value = depth_img(x,y);
  int i;
  for(i=0;i<depth_img.ni();i++){
    double dv = depth_img(i,y);
    if(dv > initial_depth_value*0.5){
      // this is a valid depth
      x_vals.push_back(i);
      depth_values.push_back(dv);
    }
  }
  
  // open the file
  vcl_ofstream outfile(file_name.c_str());
  outfile << x_vals.size() << vcl_endl;
  for(i=0;i<x_vals.size();i++){
    outfile << x_vals[i] << " " << depth_values[i] << vcl_endl;
  }
  


}


//: test local maxima callback
void va_image_client::test_local_maxima_callback(wxCommandEvent &event)
{
  // test the local maxima
  patient_pose_.test_compute_local_maxima();
  canvas_->post_redraw_event();

}


//: save the total image intensity distribution 
void va_image_client::test_intensity_distribution_maxima_callback(wxCommandEvent &event)
{
  patient_pose_.test_compute_intensity_distribution_maxima();
  canvas_->post_redraw_event();
}
  



//: save the total image intensity distribution 
void va_image_client::test_image_intensity_distribution_callback(wxCommandEvent &event)
{
  // test the local maxima
  vil_image_view<vxl_byte> display_img = patient_pose_.get_display_image();
  patient_pose_.save_intensity_distribution(display_img,vcl_string("image_intensity_distribution.dat"));
  patient_pose_.save_intensity_distribution_image(display_img,vcl_string("intensity_distribution_image.jpg"));
  // push this image onto the canvass
  // get the intensity distribution image and put it onto the image space
  vil_image_view<vxl_byte> int_dist_img = patient_pose_.get_intensity_distribution_image();
  
  if(viz_){
    viz_->lock();
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(int_dist_img);  
    viz_->unlock();
    // viz_->flush();
  }
  canvas_->post_redraw_event();

}


//: save the total image intensity distribution 
void va_image_client::test_intensity_saliency_callback(wxCommandEvent &event)
{
  // test the local maxima
  
  patient_pose_.test_intensity_saliency();
  vil_image_view<vxl_byte> display_img = patient_pose_.get_display_image();
  
  if(viz_){
    viz_->lock();
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(display_img);  
    viz_->unlock();
    // viz_->flush();
  }
  canvas_->post_redraw_event();

}


//: save the total image intensity distribution 
void va_image_client::test_intensity_thinning_callback(wxCommandEvent &event)
{
  // test the local maxima
  
  patient_pose_.test_intensity_thinning();
  vil_image_view<vxl_byte> display_img = patient_pose_.get_display_image();
  
  if(viz_){
    viz_->lock();
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(display_img);  
    viz_->unlock();
    // viz_->flush();
  }
  canvas_->post_redraw_event();

}

//: test the body segmentation methods 
void va_image_client::test_body_segmentation_callback(wxCommandEvent &event)
{
  // test the local maxima
  
  patient_pose_.test_body_segmentation();
  vil_image_view<vxl_byte> display_img = patient_pose_.get_display_image();
  
  if(viz_){
    viz_->lock();
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(display_img);  
    viz_->unlock();
    // viz_->flush();
  }
  canvas_->post_redraw_event();

}
  
//: save the current bed thresholds as a clean bed 
void va_image_client::save_as_a_clean_bed_callback(wxCommandEvent &event)
{
  gevxl::gui::wx::settings_dialog *d=new gevxl::gui::wx::settings_dialog(parent());
  vcl_string file_name("clean_bed_0.dat");
  d->field("Clean bed filename",file_name);
  
  if(d->show_modal()!=wxID_OK){
    // don't do anything
    return;
  }
  patient_pose_.save_as_a_clean_bed(file_name);
}

//: fit the current bed to a clean bed
void va_image_client::best_fit_observed_bed_to_clean_beds_callback(wxCommandEvent &event)
{
  patient_pose_.best_fit_observed_bed_to_clean_beds();

  vil_image_view<vxl_byte> display_img = patient_pose_.get_display_image();
  
  if(viz_){
    viz_->lock();
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(display_img);  
    viz_->unlock();
    // viz_->flush();
  }
  canvas_->post_redraw_event();
}

//: fit the current bed to a clean bed
void va_image_client::depth_to_body_image_callback(wxCommandEvent &event)
{
  vil_image_view<vxl_byte> depth_img =  patient_pose_.get_depth_image();
  vil_image_view<vxl_byte> body_img = patient_pose_.depth_to_body_image(depth_img);
  vil_image_view<vxl_byte> display_img = patient_pose_.get_display_image();
  
  if(viz_){
    viz_->lock();
    if(!viz_->is_initialized()){
      viz_->initialize();
    }
    viz_->clear();
    viz_->set_image(display_img);  
    viz_->unlock();
    // viz_->flush();
  }
  canvas_->post_redraw_event();
}

void va_image_client::compute_patient_pose_callback(wxCommandEvent &event)
{
  double confidence;
  patient_position best_label;
  best_label = patient_pose_.compute_patient_pose(confidence,true);
  canvas_->post_redraw_event();
}
