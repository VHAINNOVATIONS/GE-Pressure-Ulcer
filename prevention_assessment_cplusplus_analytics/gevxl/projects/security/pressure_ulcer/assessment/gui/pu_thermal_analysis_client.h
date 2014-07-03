// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 3/9/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#ifndef gevxl_pressure_ulcer_pu_thermal_analysis_client_h
#define gevxl_pressure_ulcer_pu_thermal_analysis_client_h

#include <gui/wx/handler_client.h>

#include <pressure_ulcer/assessment/pu_assessment_system_proc.h>
#include <pressure_ulcer/assessment/database/assessment_database_writer.h>

namespace gevxl { 
namespace pressure_ulcer {
	namespace assessment {

/// \brief Client that handles the pixel picking behavior to visualize the pixel's attributes in the canvas
class pu_thermal_analysis_client : public gevxl::gui::wx::handler_client
{
public:

  //////////////////////////////////////////////////////////////////
  // Standard Methods
  //////////////////////////////////////////////////////////////////

  /// Default constructor.
  pu_thermal_analysis_client(void);

  /// Default destructor.
  virtual ~pu_thermal_analysis_client(void);

  //////////////////////////////////////////////////////////////////
  // Methods
  //////////////////////////////////////////////////////////////////

	void set_assessment_system_proc(gevxl::pressure_ulcer::assessment::pu_assessment_system_proc *p) { assessment_system_proc_ = p; }

	void set_assessment_database_writer(gevxl::pressure_ulcer::assessment::database::assessment_database_writer_sptr writer) { assessment_database_writer_ = writer; }

  void update_menu(wxMenu *menu);

  void on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev);

	bool is_roi_rect_menu_checked(void) { return roi_rect_menu_->IsChecked(); }

private:

  pu_thermal_analysis_client &operator=(const pu_thermal_analysis_client &other);
  pu_thermal_analysis_client(const pu_thermal_analysis_client &other);

	gevxl::pressure_ulcer::assessment::pu_assessment_system_proc *assessment_system_proc_;

	gevxl::pressure_ulcer::assessment::database::assessment_database_writer_sptr assessment_database_writer_;

	// the menu pointer
  wxMenu *menu_;

	wxMenuItem *roi_rect_menu_;

	void on_roi_thermal_analysis_perform(wxCommandEvent &ev);
	void on_roi_thermal_analysis_clear(wxCommandEvent &ev);

	void on_thermal_analysis_params_setting(wxCommandEvent &ev);	
};

}}} // end namespaces

#endif
