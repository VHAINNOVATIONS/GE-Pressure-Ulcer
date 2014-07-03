// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_snapshot_taking_client.h"

#include <vcl_fstream.h>
#include <vcl_set.h>
#include <vcl_cstdlib.h>

#include <util/string.h>

#include <wx/filedlg.h>
#include <wx/filename.h>

#include <gui/wx/settings_dialog.h>
#include <gui/wx/canvas.h>
#include <gui/wx/wxid.h>

#include <img/style.h>

#ifdef VCL_WIN32
#include <wx/msw/registry.h>
#define HAS_REGISTRY
#endif

using namespace gevxl::gui;
using namespace gevxl::gui::wx;
using namespace gevxl::vid;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::assessment;

pu_snapshot_taking_client::pu_snapshot_taking_client(void) 
: proc_control_(NULL),
  assessment_system_proc_(NULL),
  menu_(NULL)
{
  rgb_file_directory_ = "c:/dump/rgb";
  depth_file_directory_ = "c:/dump/depth";
  thermal_file_directory_ = "c:/dump/thermal";

  view_pt_idx_ = 4;
}

/// Default destructor.
pu_snapshot_taking_client::~pu_snapshot_taking_client(void)
{

}

void pu_snapshot_taking_client::update_menu(wxMenu *menu)
{  
	menu_ = menu;
	
	append(menu_, "Take RGB/Depth/Thermal Snapshot", (wxCommandEventFunction)&pu_snapshot_taking_client::on_rgb_depth_thermal_take_snapshot);
	
	append(menu_, "Take Hyperspectral Snapshot", (wxCommandEventFunction)&pu_snapshot_taking_client::on_hyperspectral_take_snapshot);

  append(menu_, "Start to Record RGB/Depth/Thermal Video", (wxCommandEventFunction)&pu_snapshot_taking_client::on_rgb_depth_thermal_start_recording);
  
  append(menu_, "Stop to Record RGB/Depth/Thermal Video", (wxCommandEventFunction)&pu_snapshot_taking_client::on_rgb_depth_thermal_stop_recording);
  
  append(menu_, "Start to Playback RGB/Depth/Thermal Video", (wxCommandEventFunction)&pu_snapshot_taking_client::on_rgb_depth_thermal_playback_recording);
} 

void pu_snapshot_taking_client::on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev)
{
  //vcl_cout << "pu_snapshot_taking_client::on_mouse." <<  vcl_endl;

	if(source == NULL) return;

  gevxl::gui::wx::canvas *c = dynamic_cast<gevxl::gui::wx::canvas *>(source);
  if(c == NULL) return;

	if(assessment_system_proc_ == NULL) return;
	
  ev.Skip();
}

void pu_snapshot_taking_client::on_rgb_depth_thermal_take_snapshot(wxCommandEvent &ev)
{
  if(proc_control_ == NULL) return;
	if(assessment_system_proc_ == NULL) return;
	
  proc_control_->execute_one_step();

	vcl_cout << "pu_snapshot_taking_client::on_rgb_depth_thermal_take_snapshot." << vcl_endl;
  
  settings_dialog *dlg = new settings_dialog(parent());

	dlg->add_page("RGB/Depth/Thermal Take Snapshot");

  dlg->field("RGB File Directory", rgb_file_directory_);
  dlg->field("Depth File Directory", depth_file_directory_);
  dlg->field("Thermal File Directory", thermal_file_directory_);

  dlg->field("View Point Idx", view_pt_idx_);

	if(dlg->show_modal()==wxID_OK) {

    assessment_system_proc_->set_rgb_file_directory(rgb_file_directory_);
    assessment_system_proc_->set_depth_file_directory(depth_file_directory_);
    assessment_system_proc_->set_thermal_file_directory(thermal_file_directory_);
   
    assessment_system_proc_->take_snapshot(view_pt_idx_);
  }

  dlg->Destroy();

  proc_control_->execute_continously();
}

void pu_snapshot_taking_client::on_hyperspectral_take_snapshot(wxCommandEvent &ev)
{
  if(proc_control_ == NULL) return;
	if(assessment_system_proc_ == NULL) return;
	
  proc_control_->execute_one_step();

  vcl_cout << "pu_snapshot_taking_client::on_hyperspectral_take_snapshot." << vcl_endl;

  settings_dialog *dlg = new settings_dialog(parent());

	dlg->add_page("Hyperspectral Take Snapshot");

  vcl_string hyperspectral_file_directory;
  
  hyperspectral_file_directory = "c:/dump/hyperspectral";
  
  dlg->field("Hyperspectral File Directory", hyperspectral_file_directory);
  
	if(dlg->show_modal()==wxID_OK) {

    assessment_system_proc_->set_hyperspectral_file_directory(hyperspectral_file_directory);
    
    assessment_system_proc_->take_hyperspectral_snapshot();
  }

  dlg->Destroy();

  proc_control_->execute_continously();
}

void pu_snapshot_taking_client::on_rgb_depth_thermal_start_recording(wxCommandEvent &ev)
{
  if(proc_control_ == NULL) return;
	if(assessment_system_proc_ == NULL) return;
	
  // step the process control once, so that all individual processes inside the process control properly finish one stepping.
  proc_control_->execute_one_step();

	vcl_cout << "pu_snapshot_taking_client::on_rgb_depth_thermal_start_recording." << vcl_endl;
  
  settings_dialog *dlg = new settings_dialog(parent());

	dlg->add_page("RGB/Depth/Thermal Start Recording");

  dlg->field("RGB File Directory", rgb_file_directory_);
  dlg->field("Depth File Directory", depth_file_directory_);
  dlg->field("Thermal File Directory", thermal_file_directory_);

  dlg->field("View Point Idx", view_pt_idx_);

	if(dlg->show_modal()==wxID_OK) {

    assessment_system_proc_->set_rgb_file_directory(rgb_file_directory_);
    assessment_system_proc_->set_depth_file_directory(depth_file_directory_);
    assessment_system_proc_->set_thermal_file_directory(thermal_file_directory_);
   
    assessment_system_proc_->start_recording(view_pt_idx_);
  }

  dlg->Destroy();

  proc_control_->execute_continously();
}

void pu_snapshot_taking_client::on_rgb_depth_thermal_stop_recording(wxCommandEvent &ev)
{
  if(proc_control_ == NULL) return;
	if(assessment_system_proc_ == NULL) return;
	
  // step the process control once, so that all individual processes inside the process control properly finish one stepping.
  proc_control_->execute_one_step();

	vcl_cout << "pu_snapshot_taking_client::on_rgb_depth_thermal_stop_recording." << vcl_endl;
  
  assessment_system_proc_->stop_recording();
  
  proc_control_->execute_continously();
}

void pu_snapshot_taking_client::on_rgb_depth_thermal_playback_recording(wxCommandEvent &ev)
{
  if(proc_control_ == NULL) return;
	if(assessment_system_proc_ == NULL) return;
	
  // step the process control once, so that all individual processes inside the process control properly finish one stepping.
  proc_control_->execute_one_step();

	vcl_cout << "pu_snapshot_taking_client::on_rgb_depth_thermal_playback_recording." << vcl_endl;
  
  settings_dialog *dlg = new settings_dialog(parent());

	dlg->add_page("RGB/Depth/Thermal Playback Recording");

  dlg->field("RGB File Directory", rgb_file_directory_);
  dlg->field("Depth File Directory", depth_file_directory_);
  dlg->field("Thermal File Directory", thermal_file_directory_);

  dlg->field("View Point Idx", view_pt_idx_);

	if(dlg->show_modal()==wxID_OK) {

    assessment_system_proc_->set_rgb_file_directory(rgb_file_directory_);
    assessment_system_proc_->set_depth_file_directory(depth_file_directory_);
    assessment_system_proc_->set_thermal_file_directory(thermal_file_directory_);
   
    assessment_system_proc_->playback_recording(view_pt_idx_);
  }

  dlg->Destroy();

  proc_control_->execute_continously();
}
