// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 2/18/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_assessment_frame_h
#define gevxl_pressure_ulcer_pu_assessment_frame_h

#include <vcl_cstdlib.h>
#include <vcl_fstream.h>

#include <vbl/vbl_shared_pointer.h>
#include <img/visualizer_2d_buffered.h>

#include <gui/wx/app.h>
#include <gui/wx/frame.h>
#include <gui/wx/wxid.h>
#include <gui/wx/canvas.h>
#include <gui/wx/direct_visualizer.h>
#include <gui/wx/show_splash.h>
#include <gui/wx/settings_dialog.h>
#include <gui/wx/frame_record_client.h>

#include <gui/utils.h>

// assessment system proc
#include <pressure_ulcer/assessment/pu_assessment_system_proc.h>

// tissue analysis client
#include <pressure_ulcer/assessment/gui/pu_tissue_analysis_client.h>

// thermal analysis client
#include <pressure_ulcer/assessment/gui/pu_thermal_analysis_client.h>

// snapshot taking client
#include <pressure_ulcer/assessment/gui/pu_snapshot_taking_client.h>

// shape manipulation classes
#include <shape/point.h>
#include <shape/line.h>
#include <shape/rectangle.h>
#include <shape/polygon.h>

#include <shape/shape_handle_set.h>
#include <shape/point_handle_set.h>
#include <shape/line_handle_set.h>
#include <shape/rectangle_handle_set.h>
#include <shape/polygon_handle_set.h>

#include <shape/point_creator.h>
#include <shape/line_creator.h>
#include <shape/rectangle_creator.h>
#include <shape/polygon_creator.h>

#include <shape/shape_manager.h>
#include <shape/shape_creation_manager.h>

#include <shape/gui/shape_manager_client.h>
#include <shape/gui/shape_creation_manager_client.h>

#include <util/config_file.h>
#include <pressure_ulcer/assessment/database/assessment_database_writer.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace assessment {

/////////////////////////////////////////////////////////////////////////
//
// The implementation of the frame.
//
// This frame essentially visualizes a process that is controlled by
// a process controller and which supports the following
// functions:
//
// set_visualizer(visualizer *viz) -- tells process how to visualize
//
// vil_image_view<vxl_byte> &cur_frame(void) -- returns current frame
// unsigned cur_frame_num(void) -- returns current frame that is processed
// unsigned length(void) -- returns length of video
// seek(unsigned n) -- tells process to go to given frame
//
/////////////////////////////////////////////////////////////////////////
class pu_assessment_frame : public gevxl::gui::wx::frame  
{
public:

  /// Default constructor.
  pu_assessment_frame(const vcl_string &title)
    : frame(NULL, title, wxDefaultPosition, wxSize(640,480)), 
      canvas_(NULL),
      record_client_(NULL),
			assessment_system_proc_(NULL),
			tissue_analysis_client_(NULL),
			thermal_analysis_client_(NULL),
      snapshot_taking_client_(NULL),
			shape_manager_client_(NULL),
			shape_creation_manager_client_(NULL),
			assessment_database_writer_(NULL)
  { };

  virtual ~pu_assessment_frame(void);

	/// Configure the assessment frame from the config_file
	bool configure(gevxl::util::config_file &config);

	/// Tell this GUI about the assessment system process.
	void set_assessment_system_process(gevxl::pressure_ulcer::assessment::pu_assessment_system_proc *p) { assessment_system_proc_ = p; } 

  // get the shape manager
	gevxl::shape::shape_manager &get_shape_manager(void) { return shape_manager_; }
			
	// get the shape creation manager
	gevxl::shape::shape_creation_manager &get_shape_creation_manager(void) { return shape_creation_manager_; }

protected:

  bool initialize(void); 

  bool is_scroll_bar_active(void) { return true; }

  /// The canvas that we allocate.
  gevxl::gui::wx::canvas *canvas_;

  /// The main OpenGL visualizer. Will be owned by process.
  gevxl::gui::wx::direct_visualizer viz_;

  /// The overlay visualizer. Will be owned by GUI.
  gevxl::img::visualizer_2d_buffered overlay_viz_;

	//gevxl::gui::wx::canvas_record_client *canvas_record_client_;
  gevxl::gui::wx::frame_record_client *record_client_;

	/// This is where we hold the process to the assessment system proc.
	gevxl::pressure_ulcer::assessment::pu_assessment_system_proc *assessment_system_proc_;

  /// client to handle the GUI behavior for tissue analysis
  pu_tissue_analysis_client *tissue_analysis_client_;

	/// client to handle the GUI behavior for thermal analysis
  pu_thermal_analysis_client *thermal_analysis_client_;

  /// client to take snapshot
  pu_snapshot_taking_client *snapshot_taking_client_;

	/// the database access object
	gevxl::util::config_file config_;
	gevxl::pressure_ulcer::assessment::database::assessment_database_writer_sptr assessment_database_writer_;

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// the followings are the shape GUI stuff
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// the followings are the shape GUI stuff
	// This is the shape manager
	gevxl::shape::shape_manager shape_manager_;

	// This is the shape creation manager
	gevxl::shape::shape_creation_manager shape_creation_manager_;

	// callbacks for the shape creator
	void add_handable_point( gevxl::shape::point_sptr pt );
	void add_handable_line( gevxl::shape::line_sptr ln );
	void add_handable_rectangle( gevxl::shape::rectangle_sptr rect );
	void add_handable_polygon( gevxl::shape::polygon_sptr ply );

	// print new location callback after shape movement event
	void print_new_location_callback( gevxl::shape::shape_handle_set *shape );

	gevxl::shape::shape_manager_client *shape_manager_client_;
	gevxl::shape::shape_creation_manager_client *shape_creation_manager_client_;
};
typedef vbl_shared_pointer<pu_assessment_frame> pu_assessment_frame_sptr;

}}}  // end of the namespace

#endif