// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_thermal_analysis_client.h"

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

pu_thermal_analysis_client::pu_thermal_analysis_client(void) 
: assessment_system_proc_(NULL),
  menu_(NULL),
	roi_rect_menu_(NULL)
{

}

/// Default destructor.
pu_thermal_analysis_client::~pu_thermal_analysis_client(void)
{

}

void pu_thermal_analysis_client::update_menu(wxMenu *menu)
{  
	menu_ = menu;
	
	roi_rect_menu_ = append_check(menu_, "Draw Rectangle ROI for Thermal Analysis", "Draw Rectangle ROI for Thermal Analysis.");
	
	append(menu_, "Perform ROI Thermal Analysis", (wxCommandEventFunction)&pu_thermal_analysis_client::on_roi_thermal_analysis_perform);
	
	append(menu_, "Clear ROI", (wxCommandEventFunction)&pu_thermal_analysis_client::on_roi_thermal_analysis_clear);

	separator(menu_);

	append(menu_, "Parameter Setting", (wxCommandEventFunction)&pu_thermal_analysis_client::on_thermal_analysis_params_setting);	
} 

void pu_thermal_analysis_client::on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev)
{
  //vcl_cout << "pu_thermal_analysis_client::on_mouse." <<  vcl_endl;

	if(source == NULL) return;

  gevxl::gui::wx::canvas *c = dynamic_cast<gevxl::gui::wx::canvas *>(source);
  if(c == NULL) return;

	if(assessment_system_proc_ == NULL) return;

	
  ev.Skip();
}

void pu_thermal_analysis_client::on_roi_thermal_analysis_perform(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_thermal_analysis_proc() == NULL) return;

	assessment_system_proc_->get_thermal_analysis_proc()->perform_roi_thermal_analysis();
	assessment_system_proc_->visualize();
}

void pu_thermal_analysis_client::on_roi_thermal_analysis_clear(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_thermal_analysis_proc() == NULL) return;

	assessment_system_proc_->get_thermal_analysis_proc()->clear_roi_thermal_analysis();
	assessment_system_proc_->visualize();
}

void pu_thermal_analysis_client::on_thermal_analysis_params_setting(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_thermal_analysis_proc() == NULL) return;

	float skin_temperature_lower_range_in_C = 0.0, skin_temperature_higher_range_in_C = 0.0;
	float delta_temperature_in_heat_map = 0.0, hot_spot_temperature_diff_thresh = 0.0;
	int num_of_deviations_as_outlier = 3;

	assessment_system_proc_->get_thermal_analysis_proc()->get_gui_configurable_params(skin_temperature_lower_range_in_C,
																																										skin_temperature_higher_range_in_C,
																																										delta_temperature_in_heat_map,
																																										hot_spot_temperature_diff_thresh,
																																										num_of_deviations_as_outlier);
	
	int viz_mode_selection = assessment_system_proc_->get_thermal_analysis_proc()->get_viz_mode_selection();
	const vcl_vector<vcl_string> &viz_modes = assessment_system_proc_->get_thermal_analysis_proc()->get_viz_modes();
	
	bool viz_hot_spot_flag = assessment_system_proc_->get_thermal_analysis_proc()->get_viz_hot_spot_flag();

	vcl_cout << "pu_thermal_analysis_client::on_thermal_analysis_params_setting." << vcl_endl;

  settings_dialog *dlg = new settings_dialog(parent());

	dlg->add_page("Thermal Analysis Params");

	double val1, val2, val3, val4;

	val1 = skin_temperature_lower_range_in_C;
	val2 = skin_temperature_higher_range_in_C;
	val3 = delta_temperature_in_heat_map;
	val4 = hot_spot_temperature_diff_thresh;

	dlg->field("Skin Temperature Lower Range in C", val1);
	dlg->field("Skin Temperature Higher Range in C", val2);
	dlg->field("Delta Temperature in Heat Map", val3);
	dlg->field("Hot Spot Temperature Diff Threshold", val4);

	dlg->field("Hot Spot Num of Deviations as Outlier", num_of_deviations_as_outlier);

	dlg->choice("Viz Modes", viz_modes, viz_mode_selection);

	dlg->checkbox("Viz Hot Spot", viz_hot_spot_flag);
	
  if(dlg->show_modal()==wxID_OK) {

		skin_temperature_lower_range_in_C = float(val1);
		skin_temperature_higher_range_in_C = float(val2);
		delta_temperature_in_heat_map = float(val3);
		hot_spot_temperature_diff_thresh = float(val4);

		assessment_system_proc_->get_thermal_analysis_proc()->set_gui_configurable_params(skin_temperature_lower_range_in_C,
																																											skin_temperature_higher_range_in_C,
																																											delta_temperature_in_heat_map,
																																											hot_spot_temperature_diff_thresh,
																																											num_of_deviations_as_outlier);

		assessment_system_proc_->get_thermal_analysis_proc()->set_viz_mode_selection(viz_mode_selection);

		assessment_system_proc_->get_thermal_analysis_proc()->set_viz_hot_spot_flag(viz_hot_spot_flag);
	}

  dlg->Destroy();
}