// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_tissue_analysis_client.h"

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

#include <vil/vil_load.h>
#include <vil/vil_save.h>

#ifdef VCL_WIN32
#include <wx/msw/registry.h>
#define HAS_REGISTRY
#endif

using namespace gevxl::gui;
using namespace gevxl::gui::wx;
using namespace gevxl::vid;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::assessment;

pu_tissue_analysis_client::pu_tissue_analysis_client(void) 
: assessment_system_proc_(NULL),
  wound_segment_menu_(NULL),
	wound_segment_rect_menu_(NULL), 
  wound_segment_foreground_pts_menu_(NULL),
	wound_segment_background_pts_menu_(NULL),
	tissue_segment_menu_(NULL),
	tissue_segment_labeling_menu_(NULL),
	wound_3D_measurement_menu_(NULL)
{

}

/// Default destructor.
pu_tissue_analysis_client::~pu_tissue_analysis_client(void)
{

}

void pu_tissue_analysis_client::update_menu(wxMenu *menu)
{  
	menu_ = menu;

  // wound segmentation menu section
  wound_segment_menu_ = new wxMenu;
  append(wound_segment_menu_, menu_, "Wound Segmentation", "Wound Segmentation Submenu Bar.");
  
	wound_segment_rect_menu_ = append_check(wound_segment_menu_, "Draw Rectangle To Select Wound ROI", "Draw Rectangle To Select Wound ROI.");
  wound_segment_foreground_pts_menu_ = append_check(wound_segment_menu_, "Draw Points to Select Wound Foreground Points", "Draw Points to Select Wound Foreground Points.");
	wound_segment_background_pts_menu_ = append_check(wound_segment_menu_, "Draw Points to Select Wound Background Points", "Draw Points to Select Wound Background Points.");
	
	separator(wound_segment_menu_);
	
	append(wound_segment_menu_, "Wound Segmentation Start", (wxCommandEventFunction)&pu_tissue_analysis_client::on_wound_segmentation_start);
	append(wound_segment_menu_, "Wound Segmentation Cancel", (wxCommandEventFunction)&pu_tissue_analysis_client::on_wound_segmentation_cancel);
	append(wound_segment_menu_, "Wound Segmentation End", (wxCommandEventFunction)&pu_tissue_analysis_client::on_wound_segmentation_end);

  separator(menu_);

	// tissue segmentation menu section
	tissue_segment_menu_ = new wxMenu;
	append(tissue_segment_menu_, menu_, "Tissue Segmentation", "Tissue Segmentation Submenu Bar.");

	append(tissue_segment_menu_, "Tissue Segmentation Setting", (wxCommandEventFunction)&pu_tissue_analysis_client::on_tissue_segmentation_setting);
	append(tissue_segment_menu_, "Tissue Segmentation Start", (wxCommandEventFunction)&pu_tissue_analysis_client::on_tissue_segmentation_start);
	append(tissue_segment_menu_, "Tissue Segmentation End", (wxCommandEventFunction)&pu_tissue_analysis_client::on_tissue_segmentation_end);

	separator(tissue_segment_menu_);

	tissue_segment_labeling_menu_ = append_check(tissue_segment_menu_, "Tissue Segment Labeling", "Tissue Segment Labeling.");

	append(tissue_segment_menu_, "Tissue Type Histogram Model Saving", (wxCommandEventFunction)&pu_tissue_analysis_client::on_tissue_type_histogram_model_save);
	append(tissue_segment_menu_, "Tissue Type Histogram Model Loading", (wxCommandEventFunction)&pu_tissue_analysis_client::on_tissue_type_histogram_model_load);	

	separator(tissue_segment_menu_);

	append(tissue_segment_menu_, "Tissue Type Classifying", (wxCommandEventFunction)&pu_tissue_analysis_client::on_tissue_type_classifying);

	separator(menu_);

	// wound 3D measurement menu items
	wound_3D_measurement_menu_ = new wxMenu;
	append(wound_3D_measurement_menu_, menu_, "Wound 3D Measurement", "Wound 3D Measurement Submenu Bar.");

	append(wound_3D_measurement_menu_, "Wound 3D Length, Width, Depth Measurement Start", (wxCommandEventFunction)&pu_tissue_analysis_client::on_wound_3D_length_width_depth_measurement_start);
	
	separator(menu_);

	append(menu_, "Wound and Tissue Analysis End", (wxCommandEventFunction)&pu_tissue_analysis_client::on_wound_tissue_analysis_end);

	//separator(menu_);
} 

void pu_tissue_analysis_client::on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev)
{
  //vcl_cout << "pu_tissue_analysis_client::on_mouse." <<  vcl_endl;

	if(source == NULL) return;

  gevxl::gui::wx::canvas *c = dynamic_cast<gevxl::gui::wx::canvas *>(source);
  if(c == NULL) return;

	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	if( (ev.LeftIsDown()) && (tissue_segment_labeling_menu_->IsChecked()) ) {
		// the tissue segment labeling menu item is checked and the left mouse button is clicked.

		int x = c->get_mouse_x();
    int y = c->get_mouse_y();

		if(assessment_system_proc_->get_tissue_analysis_proc()->is_tissue_segmentation_labeling_mouse_clicking_valid(x, y) == false) {
			assessment_system_proc_->visualize();
			return;
		}

		// pop up a dialog window for user typing the tissue labeling.
		const vcl_vector<vcl_string> &tissue_types = assessment_system_proc_->get_tissue_analysis_proc()->get_tissue_types();
		if(tissue_types.size() == 0) return;

		int idx = 0;

		settings_dialog *dlg = new settings_dialog(parent());

		dlg->add_page("Tissue Type Labeling");
		dlg->choice("Tissue Type Choice", tissue_types, idx);
		
		if(dlg->show_modal()==wxID_OK) {
			assessment_system_proc_->get_tissue_analysis_proc()->set_tissue_segmentation_label(x, y, tissue_types[idx]);
			assessment_system_proc_->visualize();
		}

		dlg->Destroy();
	}
	
  ev.Skip();
}

void pu_tissue_analysis_client::on_wound_segmentation_start(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	assessment_system_proc_->get_tissue_analysis_proc()->start_wound_segmentation();
	assessment_system_proc_->visualize();
}
	
void pu_tissue_analysis_client::on_wound_segmentation_cancel(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	assessment_system_proc_->get_tissue_analysis_proc()->cancel_wound_segmentation();
	assessment_system_proc_->visualize();
}

void pu_tissue_analysis_client::on_wound_segmentation_end(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	assessment_system_proc_->get_tissue_analysis_proc()->end_wound_segmentation();
	assessment_system_proc_->visualize();
}

void pu_tissue_analysis_client::on_tissue_segmentation_setting(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	tissue_segmentation_spatial_radius_ = assessment_system_proc_->get_tissue_analysis_proc()->get_tissue_segmentation_spatial_radius();
	tissue_segmentation_color_radius_ = assessment_system_proc_->get_tissue_analysis_proc()->get_tissue_segmentation_color_radius();;
	tissue_segmentation_max_pyramid_level_ = assessment_system_proc_->get_tissue_analysis_proc()->get_tissue_segmentation_max_pyramid_level();

	vcl_cout << "pu_tissue_analysis_client::on_tissue_segmentation_setting is clicked." << vcl_endl;

  settings_dialog *dlg = new settings_dialog(parent());

	dlg->add_page("Mean-Shift Segmentation Params");

	dlg->field("Spatial Radius", tissue_segmentation_spatial_radius_);
  dlg->field("Color Radius", tissue_segmentation_color_radius_);
  dlg->field("Max Pyramid Level", tissue_segmentation_max_pyramid_level_);

  if(dlg->show_modal()==wxID_OK) {

		vcl_cout << "pu_tissue_analysis_client::on_tissue_segmentation_setting, Spatial Radius = " << tissue_segmentation_spatial_radius_ << vcl_endl;
		vcl_cout << "pu_tissue_analysis_client::on_tissue_segmentation_setting, Color Radius = " << tissue_segmentation_color_radius_ << vcl_endl;
		vcl_cout << "pu_tissue_analysis_client::on_tissue_segmentation_setting, Max Pyramid Level = " << tissue_segmentation_max_pyramid_level_ << vcl_endl;

    assessment_system_proc_->get_tissue_analysis_proc()->set_tissue_segmentation_spatial_radius(tissue_segmentation_spatial_radius_);
		assessment_system_proc_->get_tissue_analysis_proc()->set_tissue_segmentation_color_radius(tissue_segmentation_color_radius_);
		assessment_system_proc_->get_tissue_analysis_proc()->set_tissue_segmentation_max_pyramid_level(tissue_segmentation_max_pyramid_level_);
	}

  dlg->Destroy();
}
	
void pu_tissue_analysis_client::on_tissue_segmentation_start(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	// call assessment_system_proc_->get_tissue_analysis_proc() to start the tissue segmentation algorithm.
	assessment_system_proc_->get_tissue_analysis_proc()->start_tissue_segmentation();
	assessment_system_proc_->visualize();
}

void pu_tissue_analysis_client::on_tissue_segmentation_end(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	// call assessment_system_proc_->get_tissue_analysis_proc() to stop the tissue segmentation algorithm.
	assessment_system_proc_->get_tissue_analysis_proc()->end_tissue_segmentation();
	assessment_system_proc_->visualize();
}

void pu_tissue_analysis_client::on_tissue_type_histogram_model_save(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	wxFileName fname;
	wxString new_filename = wxFileSelector(wxT("Save tissue type histogram models as..."), fname.GetPath(), fname.GetName(), wxT("*.*"), wxT("TXT files (*.txt)|*.txt|All files (*.*)|*.*"),0);

	if(!new_filename.empty()) {
		vcl_ofstream ofs(new_filename.mb_str());
		if(ofs) {
			assessment_system_proc_->get_tissue_analysis_proc()->save_to_file_tissue_type_histogram_model(ofs);
			ofs.close();
			wxLogStatus(wxT("Saved tissue type histogram models to file '%s'."), new_filename.mb_str());
		}
		else {
			wxLogSysError(wxT("Error, failed to open file for saving."));
		}
	}
	else {
		wxLogStatus(wxT("Cancelled. Tissue type histogram models not saved."));
	}
}

void pu_tissue_analysis_client::on_tissue_type_histogram_model_load(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	wxFileName fname;
  wxFileDialog dlg(0,wxT("Load tissue type histogram models "), fname.GetPath(), fname.GetName(), wxT("TXT files (*.txt)|*.txt|All files (*.*)|*.*"), wxOPEN|wxMULTIPLE);
  
  if(dlg.ShowModal()==wxID_OK) {

    wxArrayString filenames;
    wxArrayString paths;

    dlg.GetFilenames(filenames);
    dlg.GetPaths(paths);

    if(filenames.GetCount() > 0) {
      
			for(unsigned i = 0; i < filenames.GetCount(); i++) {
        
				vcl_string name = paths[i].mb_str();
        vcl_ifstream ifs(name.c_str());

        if(ifs) {
					assessment_system_proc_->get_tissue_analysis_proc()->load_from_file_tissue_type_histogram_model(ifs);
          ifs.close();

          wxLogStatus(wxT("Loaded tissue type histogram models from '%s'."), name.c_str());
          vcl_cout << "Loaded tissue type histogram models from " << name << vcl_endl;        
        }
        else {
          wxLogSysError(wxT("Error, failed to open file '%s'."), name.c_str());
          vcl_cerr << "Error, loading tissue type histogram models " << name << vcl_endl;
        }
      }
    }
    else {
      wxLogStatus(wxT("Cancelled. tissue type histogram models not loaded."));
    }
  }
}

void pu_tissue_analysis_client::on_tissue_type_classifying(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	// call assessment_system_proc_->get_tissue_analysis_proc() to classify the wound segment into a few tissue types
	vil_image_view<vxl_byte> image_label_map;
	double granulation_percentage = 0, slough_percentage = 0, eschar_percentage = 0, bone_percentage = 0;

	assessment_system_proc_->get_tissue_analysis_proc()->classify_wound_segment_into_tissue_types(image_label_map,
																																																granulation_percentage,
																																																slough_percentage,
																																																eschar_percentage,
																																																bone_percentage);
	assessment_system_proc_->visualize();

	// to be continued to add the tissue classification result into the database.
	if(!assessment_database_writer_) return;

	settings_dialog *dlg = new settings_dialog(parent());

	dlg->add_page("Tissue Profiling Results Database Entry Submission");

	int session_id = 0, experiment_id = 0;
	vcl_string image_label_map_filename = "c:/image_label_map_filename_" + gevxl::util::to_str(session_id) + "_" + gevxl::util::to_str(experiment_id) + ".jpg";
	dlg->field("Session ID", session_id);
  dlg->field("Experiment ID", experiment_id);
	dlg->field("Image Label Map Filename", image_label_map_filename);
  if(dlg->show_modal()==wxID_OK) {

		vcl_cout << "pu_tissue_analysis_client::on_tissue_type_classifying, Session ID = " << session_id << vcl_endl;
		vcl_cout << "pu_tissue_analysis_client::on_tissue_type_classifying, Experiment ID = " << experiment_id << vcl_endl;
		vcl_cout << "pu_tissue_analysis_client::on_tissue_type_classifying, Image Label Map Filename = " << image_label_map_filename << vcl_endl;
	}

  dlg->Destroy();

	vil_save(image_label_map, image_label_map_filename.c_str());
	assessment_database_writer_->insert_segment_database_entry(session_id, experiment_id, image_label_map_filename, 
																															granulation_percentage, slough_percentage, eschar_percentage);
}

void pu_tissue_analysis_client::on_wound_3D_length_width_depth_measurement_start(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	// call assessment_system_proc_->get_tissue_analysis_proc() to stop the tissue segmentation algorithm.
	assessment_system_proc_->get_tissue_analysis_proc()->start_wound_3D_length_width_depth_measurement();
	assessment_system_proc_->visualize();
}

// wound and tissue analysis end
void pu_tissue_analysis_client::on_wound_tissue_analysis_end(wxCommandEvent &ev)
{
	if(assessment_system_proc_ == NULL) return;
	if(assessment_system_proc_->get_tissue_analysis_proc() == NULL) return;

	// call assessment_system_proc_->get_tissue_analysis_proc() to end the wound and tissue analysis for the current frame.
	assessment_system_proc_->get_tissue_analysis_proc()->end_wound_and_tissue_analysis_for_current_frame();
	assessment_system_proc_->visualize();
}

