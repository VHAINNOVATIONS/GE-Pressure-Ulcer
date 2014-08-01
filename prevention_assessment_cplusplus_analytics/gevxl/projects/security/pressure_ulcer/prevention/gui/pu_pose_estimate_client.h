// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 5/25/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#ifndef gevxl_pressure_ulcer_pu_pose_estimate_client_h
#define gevxl_pressure_ulcer_pu_pose_estimate_client_h

#include <gui/wx/handler_client.h>

#include <framework/process_control.h>
#include <pressure_ulcer/prevention/pu_prevention_chaining_process.h>

namespace gevxl { 
namespace pressure_ulcer {
	namespace prevention {

/// \brief Client that handles the pixel picking behavior to visualize the pixel's attributes in the canvas
class pu_pose_estimate_client : public gevxl::gui::wx::handler_client
{
public:

  //////////////////////////////////////////////////////////////////
  // Standard Methods
  //////////////////////////////////////////////////////////////////

  /// Default constructor.
  pu_pose_estimate_client(void);

  /// Default destructor.
  virtual ~pu_pose_estimate_client(void);

  //////////////////////////////////////////////////////////////////
  // Methods
  //////////////////////////////////////////////////////////////////

  void set_process_control(gevxl::framework::process_control *ctrl) { proc_control_ = ctrl; }

	void set_prevention_chaining_proc(gevxl::pressure_ulcer::prevention::pu_prevention_chaining_process *p) { prevention_chaining_proc_ = p; }

	virtual void update_menu(wxMenu *menu);

	//: do something if and when the mouse key is hit
  virtual void on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev);

	//: do something when a key has been hit
  virtual void on_key( gevxl::gui::event_source *source, wxKeyEvent &ev);

private:

  pu_pose_estimate_client &operator=(const pu_pose_estimate_client &other);
  pu_pose_estimate_client(const pu_pose_estimate_client &other);

	void on_left_side_label(wxCommandEvent &ev);
	void on_right_side_label(wxCommandEvent &ev);
	void on_back_side_label(wxCommandEvent &ev);
	void on_face_side_label(wxCommandEvent &ev);
	void on_empty_label(wxCommandEvent &ev);

  gevxl::framework::process_control *proc_control_;
	gevxl::pressure_ulcer::prevention::pu_prevention_chaining_process *prevention_chaining_proc_;

	// the menu pointer
  wxMenu *menu_;

	// the check menu for the pose estimate labeling
	wxMenuItem *pose_estimate_labeling_menu_;
};

}}} // end namespaces

#endif
