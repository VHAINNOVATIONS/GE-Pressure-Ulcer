// Copyright (C) 2006 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef img_gui_va_image_client_h_
#define img_gui_va_image_client_h_

/// \file
/// \author Peter Tu
/// \date 07/23/2009
/// \par Modifications:

// Put includes here.
#include <gui/wx/handler_client.h>
#include <gui/wx/canvas.h>
#include <vil/vil_image_view.h>
#include <img/filters.h>
#include <img/visualizer_image.h>
#include <vid/external_frame_process.h>
#include <detectors/va/patient_pose.h>


namespace gesec { 
  namespace detectors {
  
class va_image_client : public gevxl::gui::wx::handler_client
{
public:

  //////////////////////////////////////////////////////////////////
  // Standard Methods
  //////////////////////////////////////////////////////////////////

  /// Default constructor.
  va_image_client(void);

  /// Default destructor.
  virtual ~va_image_client(void);

  //////////////////////////////////////////////////////////////////
  // Methods
  //////////////////////////////////////////////////////////////////
  
  // define where the client can pull start images from
  void set_canvas(gevxl::gui::wx::canvas *c);
   
  // update the menu system 
  void update_menu(wxMenu *menu);

  // handle key events
  void on_key(gevxl::gui::event_source *source, wxKeyEvent &event);
  
  //: do something if and when the mouse key is hit
  virtual void on_mouse( gevxl::gui::event_source *source, wxMouseEvent &event); 
    
  //: configure this client
  virtual bool configure(gevxl::util::config_file &config); 
 
  
private: 
  /* member variables */
  // stupid method needed to stop double clicking
  bool key_flip_;

 
  // allows for the user to go back and forth through the 
  // indes and selects an image using the N and M keys.
  int index_choice_;
   // the vector of canvases
  gevxl::gui::wx::canvas* canvas_;
   
  // the vizualizer
  gevxl::img::visualizer_2d *viz_;

  /* member functions */

  void help_callback(wxCommandEvent &event);

  // save callback
  void save_callback(wxCommandEvent &event);

  //: load an image for processing
  void load_callback(wxCommandEvent &event);

  //: generate height image callback
  void height_image_callback(wxCommandEvent &event);

  //: generate a processed image callback
  void processed_image_callback(wxCommandEvent &event);

  //: test local maxima callback
  void test_local_maxima_callback(wxCommandEvent &event);
  
  // this is the current raw image
  vil_image_view<vxl_byte> raw_img_;
  
  //: apply patient_pose analysis methods
  void patient_pose_callback(wxCommandEvent &event);
  
  // this is the patient poise class used to classify an image
  patient_pose patient_pose_; 
  
  //: set the make floor measurements flag
  bool make_floor_measurement_flag_;
  void make_floor_measurement_callback(wxCommandEvent &event);
  vcl_string floor_measurement_file_name_;

  //:  this function is called from the mouse click and starts the collection of floor measurements
  void make_floor_measurement(int x, int y, vcl_string file_name, vil_image_view<vxl_byte> depth_img);

  //: save the total image intensity distribution 
  void test_image_intensity_distribution_callback(wxCommandEvent &event);

  //: save the total image intensity distribution 
  void test_intensity_distribution_maxima_callback(wxCommandEvent &event);

  //: test image intesity distribution saliency
  void test_intensity_saliency_callback(wxCommandEvent &event);

  //: test image intesity distribution thinning
  void test_intensity_thinning_callback(wxCommandEvent &event);

  //: test image intesity distribution saliency
  void test_body_segmentation_callback(wxCommandEvent &event);

  //: save the current bed thresholds as a clean bed 
  void save_as_a_clean_bed_callback(wxCommandEvent &event);
 
  //: fit the current bed to a clean bed
  void best_fit_observed_bed_to_clean_beds_callback(wxCommandEvent &event);

  //: test the end to end segmenation 
  void depth_to_body_image_callback(wxCommandEvent &event);
  
  //: classify the patient pose 
  void compute_patient_pose_callback(wxCommandEvent &event);

};

}} // end namespaces

#endif
