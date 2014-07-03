// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_assessment_frame.h"

// Our namespace.
using namespace gevxl;
using namespace gevxl::gui;
using namespace gevxl::gui::wx;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::assessment;
using namespace gevxl::shape;

pu_assessment_frame::~pu_assessment_frame(void) 
{
  if(!get_process_control()->is_terminated()) {
    get_process_control()->terminate_execution();
    get_process_control()->wait_until_terminated();
  }

  viz_.set_output_canvas(NULL);

	assessment_system_proc_->set_visualizer(NULL);

  canvas_->set_proc_visualizer(NULL);
  
  canvas_->remove_client(tissue_analysis_client_);

	canvas_->remove_client(thermal_analysis_client_);

  canvas_->remove_client(snapshot_taking_client_);

  if(NULL != record_client_) {
    RemoveEventHandler(record_client_);
    delete record_client_;
    record_client_ = NULL;
  }

  if(NULL != tissue_analysis_client_) {
    RemoveEventHandler(tissue_analysis_client_);
    delete tissue_analysis_client_;
    tissue_analysis_client_ = NULL;
  }

	if(NULL != thermal_analysis_client_) {
    RemoveEventHandler(thermal_analysis_client_);
    delete thermal_analysis_client_;
    thermal_analysis_client_ = NULL;
  }

  if(NULL != snapshot_taking_client_) {
    RemoveEventHandler(snapshot_taking_client_);
    delete snapshot_taking_client_;
    snapshot_taking_client_ = NULL;
  }
  
  if(NULL != canvas_) {
    delete canvas_;
    canvas_ = NULL;
  }
}

/// Configure the assessment frame from the config_file
bool pu_assessment_frame::configure(gevxl::util::config_file &config)
{
	config_ = config;
	return true;
}

bool pu_assessment_frame::initialize(void)
{
  int gl_attrib[10] = { WX_GL_RGBA,WX_GL_BUFFER_SIZE,32,WX_GL_DOUBLEBUFFER, 0 };

  // Allocate canvas  
	canvas_ = new wx::canvas(get_panel(), wxid("Canvas"), wxDefaultPosition, wxSize(1280, 720), 0, ("PUAssessmentSystemCanvas"), gl_attrib );

  canvas_->set_process_control(get_process_control());
 
  // Canvas on top.
  top_sizer_->Add(canvas_, 1, wxGROW, 0);

  // Visualizer visualizes into canvas.
  viz_.set_output_canvas(canvas_);

  // Tell canvas about visualizer (needed for resizing when proc is paused).
  canvas_->set_proc_visualizer(&viz_);

  // Tell OpenGL visualizer owned by proc about a secondary visualizer 
  // used by clients for overlays.
  viz_.add_visualizer(&overlay_viz_);

  // Tell canvas about this overlay visualizer. This visualizer can be 
  // used by clients during events such as on_mouse().
  canvas_->set_visualizer(&overlay_viz_);

  // Don't use swap buffer if we don't double buffer.
  viz_.use_swap_buffer_on_flush(true);

	// Add a canvas view client.
  // This is actually a handler client.
  canvas_->add_canvas_view_client();
  
  record_client_ = new gevxl::gui::wx::frame_record_client;
  record_client_->set_visualizer(&viz_);
  add_client(record_client_);

	assessment_database_writer_ = gevxl::pressure_ulcer::assessment::database::assessment_database_writer_sptr(new gevxl::pressure_ulcer::assessment::database::assessment_database_writer());
	assessment_database_writer_->configure(config_);
	if(!assessment_database_writer_->connect()) {
		vcl_cerr << "pu_assessment_frame::initialize: Error, failed to create database connection." << vcl_endl;
		return false;
	}
	
  tissue_analysis_client_ = new pu_tissue_analysis_client();
	tissue_analysis_client_->set_assessment_system_proc(assessment_system_proc_);
	tissue_analysis_client_->set_assessment_database_writer(assessment_database_writer_);
  add_client(tissue_analysis_client_);
  canvas_->add_client(tissue_analysis_client_);

	thermal_analysis_client_ = new pu_thermal_analysis_client();
	thermal_analysis_client_->set_assessment_system_proc(assessment_system_proc_);
	thermal_analysis_client_->set_assessment_database_writer(assessment_database_writer_);
  add_client(thermal_analysis_client_);
  canvas_->add_client(thermal_analysis_client_);

  snapshot_taking_client_ = new pu_snapshot_taking_client();
  snapshot_taking_client_->set_process_control(get_process_control());
  snapshot_taking_client_->set_assessment_system_proc(assessment_system_proc_);
  add_client(snapshot_taking_client_);
  canvas_->add_client(snapshot_taking_client_);

  if(get_menu_bar()) {
    record_client_->update_menu_as_submenu(get_menu_bar(), "&Record");    
    tissue_analysis_client_->update_menu_as_submenu(get_menu_bar(), "&Tissue Analysis");
    thermal_analysis_client_->update_menu_as_submenu(get_menu_bar(), "&Thermal Analysis");
    snapshot_taking_client_->update_menu_as_submenu(get_menu_bar(), "&Snapshot Taking");
  }

  // Tell process about visualizer.
	assessment_system_proc_->set_visualizer(&viz_);

  //////////////////////////////////////////////////////////////////////////////////////
	// shape manager and shape creation manager stuff
	
	{
		// Add a client to interface to the shape manager. This client is
    // what links the GUI to the shape manager. (Remember, a client is
    // a "client for GUI events".)
    //
    shape_manager_client_ = new shape::shape_manager_client();
		shape_manager_client_->set_shape_manager( &(shape_manager_) );

    // By giving the visualizer to the client, the client can instruct
    // the shape manager to re-visualize the shapes that the shape
    // manager knows about. These requests for re-visualization
    // (on_paint() events) can occur for two reasons: (1) the user
    // moves, hides, or otherwise changes the windows, triggering a
    // redraw; or (2) the user modifies a shape, and the shape manager
    // requests a redraw.
    //
    // It is not required that the client get a visualizer. If it
    // doesn't then you need to have some other mechanism in place to
    // redraw the shapes when necessary. As you will see below (in the
    // OnInit function), the shape manager will call a function of
    // your choosing when it thinks a redraw is necessary. What you do
    // with that callback is up to you.
    //
    shape_manager_client_->set_visualizer( &viz_ );

    // The client also provides a couple of menus that can change the
    // way the shape manager behaves. We choose to add those menus to
    // the GUI, thus making them available to the user.
    //
    if( get_menu_bar() ) {
      shape_manager_client_->update_menu_as_submenu( get_menu_bar(), ("&Shapes") );
    }

    // This next call is critical. Tells the canvas about the shape
    // manager client, and requests that the canvas notify the client
    // when events (mouse movements, etc) occur. Without this, the
    // client would never see any events. Also, the shape manager
    // client must be added to the *canvas*, because it is the canvas
    // that listens to the mouse.
    //
    canvas_->add_client( shape_manager_client_ );
	}

	{
		// first set the shape creation manager client
		// we add a client to tie the shape creation manager to the GUI.
    shape_creation_manager_client_ = new shape::shape_creation_manager_client();
    shape_creation_manager_client_->set_shape_creation_manager( &(shape_creation_manager_) );

    // Again, similar to the shape manager client above, setting a
    // visualizer to the shape creation manager client allows the
    // client to pass the visualization requests to the creation
    // manager.  The shape creation manager will request redraws quite
    // often: typically on every mouse movement as a shape is being
    // created.
    //
    // As with the shape manager client, the visualizer does not have
    // to be provided.  You can use the callback mechanism (in the
    // OnInit method below) to perform the appropriate updates in some
    // other way.
    //
    shape_creation_manager_client_->set_visualizer( &viz_ );

    // The shape creation manager client will create a menu that will
    // allow you to create any shape that the shape creation manager
    // knows about.  This menu is dynamically mantained, so as you add
    // possible shapes to the shape creation manager, appropriate
    // items will be added to the menu.
    //
    if( get_menu_bar() ) {
      shape_creation_manager_client_->update_menu_as_submenu( get_menu_bar(), ("&Shape Creation") );
    }

    // Again, this is critial.  This is what allows the mouse events,
    // etc, to propagate to the client.
    //
    canvas_->add_client( shape_creation_manager_client_ );
	}

	shape_manager_.set_redraw_function( boost::bind( &gui::wx::canvas::post_redraw_event, canvas_ ) );

	// This is the default drawing style
  {
    img::style_sptr st( new img::style );
    st->set_line_width( 1 );
    st->set_point_radius( 4 );
    st->set_foreground( 1, 1, 1 );
    shape_manager_.set_normal_style( st );
  }

  // This is the style used to draw shapes that are selected
  {
    img::style_sptr st( new img::style );
    st->set_line_width( 1 );
    st->set_point_radius( 4 );
    st->set_foreground( 1, 0, 0 );
    shape_manager_.set_selected_style( st );
  }

  // This is the style used to draw shapes that are
  // highlighted. Highlighting is a temporary condition (mouse
  // nearby); selection is a more permanent condition.
  {
    img::style_sptr st( new img::style );
    st->set_foreground( 1, 1, 1 );
    st->set_point_radius( 6 );
    st->set_line_width( 4 );
    shape_manager_.set_highlighted_style( st );
  }

	// Now we start setting up the creation manager.  Like the shape
  // manager, the redraw requests happen through callbacks.
	shape_creation_manager_.set_redraw_function( boost::bind( &gui::wx::canvas::post_redraw_event, canvas_ ) );

	{
    // Create a point called "Managed Point".  When such a point is created, we
    // add a handable version of it to the shape manager.  To do this
    // (without dynamic casting) we need to know that is it a point
    // being created, so we register a shape-specific callback.

    shape::point_creator_sptr pc( new shape::point_creator );
    pc->add_complete_specific_callback( boost::bind( &pu_assessment_frame::add_handable_point, this, _1 ) );
    shape_creation_manager_.add_creator( "Managed Point", pc );
  }

  {
    // Create a line called "Managed Line".  When such a line is created, we
    // add a handable version of it to the shape manager.  To do this
    // (without dynamic casting) we need to know that is it a line
    // being created, so we register a shape-specific callback.

    shape::line_creator_sptr lc( new shape::line_creator );
    lc->add_complete_specific_callback( boost::bind( &pu_assessment_frame::add_handable_line, this, _1 ) );
    shape_creation_manager_.add_creator( "Managed Line", lc );
  }

	{
    // Create a rectangle called "Managed Rectangle".  When such a rectangle is created, we
    // add a handable version of it to the shape manager.  To do this
    // (without dynamic casting) we need to know that is it a rectangle
    // being created, so we register a shape-specific callback.

    shape::rectangle_creator_sptr rc( new shape::rectangle_creator );
    rc->add_complete_specific_callback( boost::bind( &pu_assessment_frame::add_handable_rectangle, this, _1 ) );
    shape_creation_manager_.add_creator( "Managed Rectangle", rc );
  }

	{
    // Create a polygon called "Managed Polygon".  When such a polygon is created, we
    // add a handable version of it to the shape manager.  To do this
    // (without dynamic casting) we need to know that is it a polygon
    // being created, so we register a shape-specific callback.

    shape::polygon_creator_sptr plyc( new shape::polygon_creator );
    plyc->add_complete_specific_callback( boost::bind( &pu_assessment_frame::add_handable_polygon, this, _1 ) );
    shape_creation_manager_.add_creator( "Managed Polygon", plyc );
  }

  return true;
}

void pu_assessment_frame::add_handable_point( gevxl::shape::point_sptr pt )
{
	shape::point_handle_set_sptr h( new shape::point_handle_set() );
	h->set_shape(pt);
	h->set_moved_callback( boost::bind( &pu_assessment_frame::print_new_location_callback, this, _1 ) );
  //shape_manager_.add_handlable_shape( h );
}

void pu_assessment_frame::add_handable_line( gevxl::shape::line_sptr ln )
{
	shape::line_handle_set_sptr h( new shape::line_handle_set() );
	h->set_shape(ln);
	h->set_moved_callback( boost::bind( &pu_assessment_frame::print_new_location_callback, this, _1 ) );
  //shape_manager_.add_handlable_shape( h );
}

void pu_assessment_frame::add_handable_rectangle( gevxl::shape::rectangle_sptr rect )
{
	shape::rectangle_handle_set_sptr h( new shape::rectangle_handle_set( rect ) );
	h->set_moved_callback( boost::bind( &pu_assessment_frame::print_new_location_callback, this, _1 ) );
  //shape_manager_.add_handlable_shape( h );

	if(tissue_analysis_client_->is_wound_segment_rect_menu_checked()) {
		// wound segmentation rectangle roi
		assessment_system_proc_->get_tissue_analysis_proc()->set_wound_segment_rect_handle_set(h);
	}

	// add the thermal_analysis_client_ handler to the rectangle shape 
	if(thermal_analysis_client_->is_roi_rect_menu_checked()) {
		// thermal skin region analysis rectangle roi
		assessment_system_proc_->get_thermal_analysis_proc()->set_roi_rect_handle_set(h);
	}
}

void pu_assessment_frame::add_handable_polygon( gevxl::shape::polygon_sptr ply )
{
	shape::polygon_handle_set_sptr h( new shape::polygon_handle_set( ply ) );
	h->set_moved_callback( boost::bind( &pu_assessment_frame::print_new_location_callback, this, _1 ) );
  //shape_manager_.add_handlable_shape( h );

	if(tissue_analysis_client_->is_wound_segment_foreground_pts_menu_checked()) {
		// wound segmentation foreground pixels
		assessment_system_proc_->get_tissue_analysis_proc()->add_wound_segment_foreground_polygon_handle_set(h);
	}
	else if(tissue_analysis_client_->is_wound_segment_background_pts_menu_checked()) {
		// wound segmentation background pixels
		assessment_system_proc_->get_tissue_analysis_proc()->add_wound_segment_background_polygon_handle_set(h);
	}
}

void pu_assessment_frame::print_new_location_callback( gevxl::shape::shape_handle_set *shape )
{
	vcl_cout << "\n\n\nMOVEMENT COMPLETE\n" << *shape->get_shape() << vcl_endl;
}
