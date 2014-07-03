// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/30/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#ifndef gevxl_pressure_ulcer_pu_pixel_picking_client_h
#define gevxl_pressure_ulcer_pu_pixel_picking_client_h

#include <gui/wx/handler_client.h>
#include <pressure_ulcer/pu_camera_source_proc.h>

namespace gevxl { 
namespace pressure_ulcer {

/// \brief Client that handles the pixel picking behavior to visualize the pixel's attributes in the canvas
class pu_pixel_picking_client : public gevxl::gui::wx::handler_client
{
public:

  //////////////////////////////////////////////////////////////////
  // Standard Methods
  //////////////////////////////////////////////////////////////////

  /// Default constructor.
  pu_pixel_picking_client(void);

  /// Default destructor.
  virtual ~pu_pixel_picking_client(void);

  //////////////////////////////////////////////////////////////////
  // Methods
  //////////////////////////////////////////////////////////////////

  void set_proc(gevxl::pressure_ulcer::pu_camera_source_proc *p) { proc_ = p; }

  void update_menu(wxMenu *menu);

  void on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev);

private:

  pu_pixel_picking_client &operator=(const pu_pixel_picking_client &other);
  pu_pixel_picking_client(const pu_pixel_picking_client &other);

  // the proc_ to be updated from the client
  gevxl::pressure_ulcer::pu_camera_source_proc *proc_;

  // the menu pointer
  wxMenu *menu_;

	// thermal camera menu items
  wxMenuItem *thermal_vis_in_F_menu_;
  wxMenuItem *thermal_vis_in_C_menu_;

	// intel 3D camera menu items
  wxMenuItem *intel_3d_vis_xyz_menu_;
  wxMenuItem *intel_3d_vis_xyz_rgb_menu_;

	// openni2 3D camera menu items
	wxMenuItem *openni2_3d_vis_xyz_menu_;
  wxMenuItem *openni2_3d_vis_xyz_rgb_menu_;
};

}} // end namespaces

#endif
