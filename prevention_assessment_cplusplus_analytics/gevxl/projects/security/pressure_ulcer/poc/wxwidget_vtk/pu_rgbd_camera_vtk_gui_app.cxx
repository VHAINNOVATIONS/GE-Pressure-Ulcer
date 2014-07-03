// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 5/12/2014
/// \par Modifications:

#include <vcl_cstdlib.h>
#include <vcl_fstream.h>

#include <vbl/vbl_shared_pointer.h>
#include <vul/vul_arg.h>

#include <gui/wx/app.h>
#include <gui/wx/frame.h>
#include <gui/wx/wxid.h>
#include <gui/wx/canvas.h>
#include <gui/wx/show_splash.h>
#include <gui/wx/settings_dialog.h>

#include <gui/wx/vtk_canvas.h>
#include <gui/wx/visualizer_vtk_buffered.h>

#include <gui/utils.h>

#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>
#include <vid/openni2_frame_process.h>
#include <vid/pxc_frame_process.h>

#include <framework/process.h>
#include <framework/process_control.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <threading/sleep.h>

// Our namespace.
using namespace gevxl::gui;
using namespace gevxl::gui::wx;

namespace gevxl {
	namespace pressure_ulcer {

class pu_rgbd_camera_vtk_proc : public gevxl::framework::process,
														    public gevxl::util::on_off_mixin
{
public:

	// constructor
	pu_rgbd_camera_vtk_proc(char const *name="gevxl::pressure_ulcer::pu_rgbd_camera_vtk_proc") 
		: gevxl::framework::process(name),
			source_proc_("vid::source_process"), 
			vtk_viz_(NULL),
			frame_nr_(0),
			source_process_type_("openni2"),
			source_output_type_("depth")
	{
		highres_timer_.reset();  
	}

	// destructor
	virtual ~pu_rgbd_camera_vtk_proc(void)
	{

	}

	// configure this proc
	virtual bool configure(gevxl::util::config_file &config)
	{
		config_ = config;

		source_process_type_ = "openni2";
    //source_process_type_ = "pcsdk";
		config_.get_string(name()+"::source_process_type", source_process_type_);

		source_output_type_ = "depth";  // for 3D depth camera, output type = "rgb" and "depth" and "rgb_in_depth_view"
		config_.get_string(name()+"::source_output_type", source_output_type_);

		// ---- source configuration 
		if( !source_proc_.configure(config_) ) {
			vcl_cerr << "pu_rgbd_camera_vtk_proc::configure, Error configuring source_proc_." << vcl_endl;
			return false;
		}

		if(source_process_type_ == "openni2") {
			if(source_output_type_ != "rgb" && source_output_type_ != "depth") {
				// ensure the output is the depth view
				source_proc_.set_output_type("depth");
			}
			else {
				source_proc_.set_output_type(source_output_type_);
			}
		}
    else if(source_process_type_ == "pcsdk") {
      if(source_output_type_ != "rgb" && source_output_type_ != "depth" && source_output_type_ != "rgb_in_depth_view") {
        // ensure the output is the depth view
        source_proc_.set_output_type("depth");
      }
      else {
        source_proc_.set_output_type(source_output_type_);
      }
    }
		
		return true;
	}

	// initialize the process
	virtual bool initialize(void)
	{
		if(!source_proc_.initialize()) {
			vcl_cerr << "pu_rgbd_camera_vtk_proc::initialize, Error initializing source_proc_." << vcl_endl;		
			return false;
		}

		if(source_process_type_ == "openni2") {
			const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
			if(!openni2_source) {
				vcl_cerr << "pu_rgbd_camera_vtk_proc::initialize, Error the source camera process has to be openni2_frame_process." << vcl_endl;		
				return false;
			}
		}
    else if(source_process_type_ == "pcsdk") {
      const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(source_proc_.get_frame_process());
      if(!pxc_source) {
				vcl_cerr << "pu_rgbd_camera_vtk_proc::initialize, Error the source camera process has to be pxc_frame_process." << vcl_endl;		
				return false;
			}
    }
  	
		return true;
	}

	// uninitialize the process
	virtual void uninitialize(void) { }

	// the main step function
	virtual bool step(void)
	{
		if(!source_proc_.step()) {
			vcl_cerr << "pu_rgbd_camera_vtk_proc::step, stepping source_proc_ error." << vcl_endl;
			return false;
		}

    
		// start to visualize the point cloud frame
		if(vtk_viz_ == NULL ) return true;

		vtk_viz_->lock();
		
    if(source_process_type_ == "openni2") {
		  const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());	
		  const vil_image_view<float> &xyz_rgb_frame = openni2_source->cur_xyz_rgb_frame();
		  vtk_viz_->add_point_cloud(xyz_rgb_frame, -1);
    }
    else if(source_process_type_ == "pcsdk") {
      const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(source_proc_.get_frame_process());
      const vil_image_view<float> &xyz_rgb_frame = pxc_source->cur_xyz_rgb_frame();
		  vtk_viz_->add_point_cloud(xyz_rgb_frame, -1);
    }

		//vtk_viz_->reset_view();
		vtk_viz_->flush();
		
		vtk_viz_->unlock();

		gevxl::threading::sleep(500);

		return true;
	}

	// set the process's visualizer
	virtual void set_visualizer(gevxl::gui::wx::visualizer_vtk_buffered *viz)
	{
		vtk_viz_ = viz;
	}

	// visualization function
	virtual void visualize(void)
	{
		///
	}

  // get the current frame from the source process
  const vil_image_view<vxl_byte> &cur_frame(void) const { return source_proc_.cur_frame(); }

  // get the source process type
  vcl_string get_source_process_type(void) { return source_process_type_; }

  // get the source process
  gevxl::vid::generic_frame_process<vxl_byte> &get_source_process(void) { return source_proc_; }
      
private:
						
	// input source
	gevxl::vid::generic_frame_process<vxl_byte> source_proc_;

  // frame nr
  int frame_nr_;

  // high resolution timer
  gevxl::util::time::highres_timer highres_timer_; 

  // visualizer 	
	gevxl::gui::wx::visualizer_vtk_buffered *vtk_viz_;

	// the configuration file that the generator needs in order to configure its own parameters.
	gevxl::util::config_file config_;

  // input source process type
  vcl_string source_process_type_;

  // source process's output type
  vcl_string source_output_type_;

};  // end of class pu_rgbd_camera_vtk_proc
typedef vbl_shared_pointer<pu_rgbd_camera_vtk_proc> pu_rgbd_camera_vtk_proc_sptr;

class pu_rgbd_camera_vtk_frame : public gevxl::gui::wx::frame  
{
public:

  /// Default constructor.
  pu_rgbd_camera_vtk_frame(const vcl_string &title)
    : frame(NULL, title, wxDefaultPosition, wxSize(640,480)), 
      vtk_canvas_(NULL),
      proc_(NULL)      
  { };

  virtual ~pu_rgbd_camera_vtk_frame(void)
	{
		if(!get_process_control()->is_terminated()) {
			get_process_control()->terminate_execution();
			get_process_control()->wait_until_terminated();
		}

		vtk_viz_.set_canvas(NULL);

		proc_->set_visualizer(NULL);

		// the proc_ is owned by the MyApp class, so the frame shouldn't do anything to clean up this pointer
		if(NULL != vtk_canvas_) {
			delete vtk_canvas_;
			vtk_canvas_ = NULL;
		}
	}

  /// Tell this GUI about the process.
  void set_process(gevxl::pressure_ulcer::pu_rgbd_camera_vtk_proc *p) { proc_ = p; }   

protected:

  bool initialize(void)
	{
		int gl_attrib[10] = { WX_GL_RGBA,WX_GL_BUFFER_SIZE,32,WX_GL_DOUBLEBUFFER, 0 };

		// Allocate canvas
		vtk_canvas_ = new gevxl::gui::wx::vtk_canvas(get_panel(), wxid("Canvas"), wxDefaultPosition, wxSize(proc_->cur_frame().ni(), proc_->cur_frame().nj()), 0, ("PUDepthCanvas"), gl_attrib );

		// Canvas on top.
		top_sizer_->Add(vtk_canvas_, 1, wxGROW, 0);

		// Visualizer visualizes into canvas.
		vtk_viz_.set_canvas(vtk_canvas_);

		// Tell process about visualizer.
		proc_->set_visualizer(&vtk_viz_);

		return true;
	}

  bool is_scroll_bar_active(void) { return true; }

  /// The canvas that we allocate.
	gevxl::gui::wx::vtk_canvas *vtk_canvas_;

  /// The main vtk buffered visualizer. Will be owned by process.  
	gevxl::gui::wx::visualizer_vtk_buffered vtk_viz_;

  /// This is where we do all the processing.
  gevxl::pressure_ulcer::pu_rgbd_camera_vtk_proc *proc_;

};
typedef vbl_shared_pointer<pu_rgbd_camera_vtk_frame> pu_rgbd_camera_vtk_frame_sptr;

	} // end of pressure_ulcer namespace
} // end of gevxl namespace

/////////////////////////////////////////////////////////////////////////
//
// The main application.
//
/////////////////////////////////////////////////////////////////////////
class MyApp : public wx::app
{
  virtual bool OnInit();

  virtual int OnExit();

  /// Example process.
	gevxl::pressure_ulcer::pu_rgbd_camera_vtk_proc proc_;  

  /// Process control that controls above process.
  gevxl::framework::process_control control_;

  // The main frame window.
	gevxl::pressure_ulcer::pu_rgbd_camera_vtk_frame_sptr frame_; 
};

// This is a wxWidgets construct. It hides main().
IMPLEMENT_APP(MyApp)

// This function can be viewed as the traditional main().
bool MyApp::OnInit()
{  
   vul_arg<vcl_string> a_config_file( "-c", "Config file", "C:/GEVXL/gevxl/projects/security/pressure_ulcer/config/pu_camera_source.tconf" );
   gevxl::util::config_file config;

#if wxUSE_UNICODE 
  convert_unicode_to_ansi(argc, argv);
  vul_arg_parse(argc, argv_, false);  
  config.parse_arguments(argc, argv_); // process remaining command-line args
#else
  vul_arg_parse(argc, argv, false);
  config.parse_arguments(argc, argv); // process remaining command-line args
#endif

 // if( a_config_file.set() )
  config.read( a_config_file().c_str() );  // process entire config file

  utils::ensure_terminal_output();

  if(!proc_.configure(config) || !proc_.initialize())
  {
    vcl_cerr << "Error, failed to initialize or configure process.\n";
    return EXIT_FAILURE;
  }

  control_.set_process(&proc_);
  control_.initialize_control(control_.ASYNCHRONOUS);
  control_.execute_one_step(true);
  control_.execute_continously();
 
  // Create the main frame window.
	frame_ = gevxl::pressure_ulcer::pu_rgbd_camera_vtk_frame_sptr(new gevxl::pressure_ulcer::pu_rgbd_camera_vtk_frame("Pressure Ulcer RGB-D Camera VTK Demo"));
  frame_->set_process(&proc_);
  frame_->set_process_control(&control_);
  frame_->enable_scroll_bar(true);
  frame_->init(); // Main initialization (which is the actual GUI).
  frame_->activate_scroll_bar(false);
  frame_->Show(true);// Show the frame (i.e., GUI).

  // Return and start the wxWidgets event loop.
  return true;
}

int MyApp::OnExit()
{
  vcl_cout << "Shutting down process and process control.\n";

  vcl_cout << "Main done. \n";

  vcl_exit(EXIT_SUCCESS);

  return 1;
}
