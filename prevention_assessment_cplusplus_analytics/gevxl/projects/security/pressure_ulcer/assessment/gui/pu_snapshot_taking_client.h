// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 5/25/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#ifndef gevxl_pressure_ulcer_pu_snapshot_taking_client_h
#define gevxl_pressure_ulcer_pu_snapshot_taking_client_h

#include <gui/wx/handler_client.h>

#include <framework/process_control.h>
#include <pressure_ulcer/assessment/pu_assessment_system_proc.h>

namespace gevxl { 
namespace pressure_ulcer {
	namespace assessment {

/// \brief Client that handles the pixel picking behavior to visualize the pixel's attributes in the canvas
class pu_snapshot_taking_client : public gevxl::gui::wx::handler_client
{
public:

  //////////////////////////////////////////////////////////////////
  // Standard Methods
  //////////////////////////////////////////////////////////////////

  /// Default constructor.
  pu_snapshot_taking_client(void);

  /// Default destructor.
  virtual ~pu_snapshot_taking_client(void);

  //////////////////////////////////////////////////////////////////
  // Methods
  //////////////////////////////////////////////////////////////////

  void set_process_control(gevxl::framework::process_control *ctrl) { proc_control_ = ctrl; }

	void set_assessment_system_proc(gevxl::pressure_ulcer::assessment::pu_assessment_system_proc *p) { assessment_system_proc_ = p; }

	void update_menu(wxMenu *menu);

  void on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev);

private:

  pu_snapshot_taking_client &operator=(const pu_snapshot_taking_client &other);
  pu_snapshot_taking_client(const pu_snapshot_taking_client &other);

  gevxl::framework::process_control *proc_control_;
	gevxl::pressure_ulcer::assessment::pu_assessment_system_proc *assessment_system_proc_;

	// the menu pointer
  wxMenu *menu_;

	void on_rgb_depth_thermal_take_snapshot(wxCommandEvent &ev);
	void on_hyperspectral_take_snapshot(wxCommandEvent &ev);

  void on_rgb_depth_thermal_start_recording(wxCommandEvent &ev);
  void on_rgb_depth_thermal_stop_recording(wxCommandEvent &ev);
  void on_rgb_depth_thermal_playback_recording(wxCommandEvent &ev);

  vcl_string rgb_file_directory_;
  vcl_string depth_file_directory_;
  vcl_string thermal_file_directory_;

  int view_pt_idx_;
};

}}} // end namespaces

#endif
