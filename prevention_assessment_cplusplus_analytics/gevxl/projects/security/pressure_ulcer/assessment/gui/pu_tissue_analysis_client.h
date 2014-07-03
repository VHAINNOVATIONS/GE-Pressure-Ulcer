// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 2/18/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#ifndef gevxl_pressure_ulcer_pu_tissue_analysis_client_h
#define gevxl_pressure_ulcer_pu_tissue_analysis_client_h

#include <gui/wx/handler_client.h>

#include <pressure_ulcer/assessment/pu_assessment_system_proc.h>
#include <pressure_ulcer/assessment/database/assessment_database_writer.h>

namespace gevxl { 
namespace pressure_ulcer {
	namespace assessment {

/// \brief Client that handles the pixel picking behavior to visualize the pixel's attributes in the canvas
class pu_tissue_analysis_client : public gevxl::gui::wx::handler_client
{
public:

  //////////////////////////////////////////////////////////////////
  // Standard Methods
  //////////////////////////////////////////////////////////////////

  /// Default constructor.
  pu_tissue_analysis_client(void);

  /// Default destructor.
  virtual ~pu_tissue_analysis_client(void);

  //////////////////////////////////////////////////////////////////
  // Methods
  //////////////////////////////////////////////////////////////////

	void set_assessment_system_proc(gevxl::pressure_ulcer::assessment::pu_assessment_system_proc *p) { assessment_system_proc_ = p; }

	void set_assessment_database_writer(gevxl::pressure_ulcer::assessment::database::assessment_database_writer_sptr writer) { assessment_database_writer_ = writer; }

  void update_menu(wxMenu *menu);

  void on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev);

	// status of wound segmentation menu items
	bool is_wound_segment_rect_menu_checked(void) { return wound_segment_rect_menu_->IsChecked(); }
	bool is_wound_segment_foreground_pts_menu_checked(void) { return wound_segment_foreground_pts_menu_->IsChecked(); }
	bool is_wound_segment_background_pts_menu_checked(void) { return wound_segment_background_pts_menu_->IsChecked(); }

private:

  pu_tissue_analysis_client &operator=(const pu_tissue_analysis_client &other);
  pu_tissue_analysis_client(const pu_tissue_analysis_client &other);

	gevxl::pressure_ulcer::assessment::pu_assessment_system_proc *assessment_system_proc_;
	
	gevxl::pressure_ulcer::assessment::database::assessment_database_writer_sptr assessment_database_writer_;

	// the menu pointer
  wxMenu *menu_;

	// wound segmentation menu items
	wxMenu *wound_segment_menu_;

  wxMenuItem *wound_segment_rect_menu_;
  wxMenuItem *wound_segment_foreground_pts_menu_;
	wxMenuItem *wound_segment_background_pts_menu_;

	void on_wound_segmentation_start(wxCommandEvent &ev);
	void on_wound_segmentation_cancel(wxCommandEvent &ev);
	void on_wound_segmentation_end(wxCommandEvent &ev);

	// tissue segmentation menu items
	wxMenu *tissue_segment_menu_;

	void on_tissue_segmentation_setting(wxCommandEvent &ev);
	void on_tissue_segmentation_start(wxCommandEvent &ev);
	void on_tissue_segmentation_end(wxCommandEvent &ev);

	int tissue_segmentation_spatial_radius_;
	int tissue_segmentation_color_radius_;
	int tissue_segmentation_max_pyramid_level_;

	wxMenuItem *tissue_segment_labeling_menu_;

	void on_tissue_type_histogram_model_save(wxCommandEvent &ev);
	void on_tissue_type_histogram_model_load(wxCommandEvent &ev);

	void on_tissue_type_classifying(wxCommandEvent &ev);

	// wound 3D measurement menu items
	wxMenu *wound_3D_measurement_menu_;
	
	void on_wound_3D_length_width_depth_measurement_start(wxCommandEvent &ev);

	// wound and tissue analysis end
	void on_wound_tissue_analysis_end(wxCommandEvent &ev);
};

}}} // end namespaces

#endif
