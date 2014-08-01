// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_pose_estimate_client.h"

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
using namespace gevxl::pressure_ulcer::prevention;

pu_pose_estimate_client::pu_pose_estimate_client(void) 
: proc_control_(NULL),
  prevention_chaining_proc_(NULL),
  menu_(NULL),
	pose_estimate_labeling_menu_(NULL)
{

}

/// Default destructor.
pu_pose_estimate_client::~pu_pose_estimate_client(void)
{

}

void pu_pose_estimate_client::update_menu(wxMenu *menu)
{  
	menu_ = menu;

	pose_estimate_labeling_menu_ = append_check(menu_, "Pose Estimate Labeling", "Pose Estimate Labeling.");

	separator(menu_);

	append(menu_, "Left Side Label", (wxCommandEventFunction)&pu_pose_estimate_client::on_left_side_label);
	append(menu_, "Right Side Label", (wxCommandEventFunction)&pu_pose_estimate_client::on_right_side_label);
	append(menu_, "Back Side Label", (wxCommandEventFunction)&pu_pose_estimate_client::on_back_side_label);
	append(menu_, "Face Side Label", (wxCommandEventFunction)&pu_pose_estimate_client::on_face_side_label);
	append(menu_, "Empty Label", (wxCommandEventFunction)&pu_pose_estimate_client::on_empty_label);	
}

void pu_pose_estimate_client::on_left_side_label(wxCommandEvent &ev)
{
	if(prevention_chaining_proc_ == NULL) return;
	
	prevention_chaining_proc_->set_pose_estimate_label("left_side");
}

void pu_pose_estimate_client::on_right_side_label(wxCommandEvent &ev)
{
	if(prevention_chaining_proc_ == NULL) return;
	
	prevention_chaining_proc_->set_pose_estimate_label("right_side");
}

void pu_pose_estimate_client::on_back_side_label(wxCommandEvent &ev)
{
	if(prevention_chaining_proc_ == NULL) return;
	
	prevention_chaining_proc_->set_pose_estimate_label("back_side");
}

void pu_pose_estimate_client::on_face_side_label(wxCommandEvent &ev)
{
	if(prevention_chaining_proc_ == NULL) return;
	
	prevention_chaining_proc_->set_pose_estimate_label("face_side");
}

void pu_pose_estimate_client::on_empty_label(wxCommandEvent &ev)
{
	if(prevention_chaining_proc_ == NULL) return;
	
	prevention_chaining_proc_->set_pose_estimate_label("empty");
}

void pu_pose_estimate_client::on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev)
{
  //vcl_cout << "pu_pose_estimate_client::on_mouse." <<  vcl_endl;

	if(source == NULL) return;

  gevxl::gui::wx::canvas *c = dynamic_cast<gevxl::gui::wx::canvas *>(source);
  if(c == NULL) return;

	if(prevention_chaining_proc_ == NULL) return;
	
  ev.Skip();
}

void pu_pose_estimate_client::on_key( gevxl::gui::event_source *source, wxKeyEvent &ev)
{
	if(prevention_chaining_proc_ == NULL) return;

	switch(ev.GetKeyCode())
	{
	case 'L': 
		{
			prevention_chaining_proc_->set_pose_estimate_label("left_side");
			vcl_cout << "L - left_side" << vcl_endl;
			break;
		}

	case 'R': 
		{
			prevention_chaining_proc_->set_pose_estimate_label("right_side");
			vcl_cout << "R - right_side" << vcl_endl;
			break;	
		}

	case 'B': 
		{
			prevention_chaining_proc_->set_pose_estimate_label("back_side");
			vcl_cout << "B - back_side" << vcl_endl;
			break;
		}

	case 'F': 
		{
			prevention_chaining_proc_->set_pose_estimate_label("face_side");
			vcl_cout << "F - face_side" << vcl_endl;
			break;
		}

	default: ev.Skip();
	}
	
	ev.Skip();
	return; 
}
