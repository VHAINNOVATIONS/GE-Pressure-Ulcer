// Copyright (C) 2012 General Electric Company
//  
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.
// 
// 

#include <vcl_cstdlib.h>
#include <vcl_fstream.h>

#include <vbl/vbl_shared_pointer.h>

#include <vul/vul_arg.h>

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

#include <framework/process_control.h>

#include <pressure_ulcer/pu_motion_feature_analysis_proc.h>

// Our namespace.
using namespace gevxl::gui;
using namespace gevxl::gui::wx;
using namespace gevxl::pressure_ulcer;

/////////////////////////////////////////////////////////////////////////
//
// A client to handle modify parameters of processes the application.
//
/////////////////////////////////////////////////////////////////////////
class options_handler_client : public gevxl::gui::wx::handler_client
{
public:
  //:
  options_handler_client( void ) { };

  //:
  virtual ~options_handler_client() { };

  //:
  virtual void update_menu (wxMenu *menu );

  //:
  void on_mouse( gevxl::gui::event_source *source, wxMouseEvent &event) {}

  virtual void on_key( gevxl::gui::event_source *source, wxKeyEvent &event) {}

  virtual bool configure(gevxl::util::config_file &config) { return true; };

  // process that containts the options we are interested in toggling
  void set_proc(gevxl::pressure_ulcer::pu_motion_feature_analysis_proc *p) { proc_ = p; }

  void on_configure(wxCommandEvent &evt);

private:

  gevxl::pressure_ulcer::pu_motion_feature_analysis_proc *proc_;

};

void options_handler_client::update_menu( wxMenu *menu )
{
  append(menu,"&Configure",(wxCommandEventFunction)&options_handler_client::on_configure, "Change Process Prameters.");
}

void options_handler_client::on_configure(wxCommandEvent &evt)
{
  gevxl::gui::wx::settings_dialog dlg(parent_);
  dlg.add_page("Settings");  
}


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
//
// seek(unsigned n) -- tells process to go to given frame
//
/////////////////////////////////////////////////////////////////////////

class demo_frame : public gevxl::gui::wx::frame  
{
public:

  /// Default constructor.
  demo_frame(const vcl_string &title)
    : frame(NULL, title, wxDefaultPosition, wxSize(640,480)), 
      canvas_(NULL),
      proc_(NULL),
      record_client_(NULL),
      options_handler_(NULL)
  { };

  virtual ~demo_frame(void);

  /// Tell this GUI about the process.
  void set_process(gevxl::pressure_ulcer::pu_motion_feature_analysis_proc *p) { proc_ = p; } 

protected:

  bool initialize(void); 

  bool is_scroll_bar_active(void) { return true; }

  /// The canvas that we allocate.
  wx::canvas *canvas_;

  /// The main OpenGL visualizer. Will be owned by process.
  wx::direct_visualizer viz_;

  /// The overlay visualizer. Will be owned by GUI.
  gevxl::img::visualizer_2d_buffered overlay_viz_;

  /// This is where we do all the processing.
  gevxl::pressure_ulcer::pu_motion_feature_analysis_proc *proc_;

  //gevxl::gui::wx::canvas_record_client *canvas_record_client_;
  gevxl::gui::wx::frame_record_client *record_client_;

  // Client to handle the toggling of options in the program
  options_handler_client *options_handler_;

};
typedef vbl_shared_pointer<demo_frame> demo_frame_sptr;

demo_frame::~demo_frame(void) 
{
  if(!get_process_control()->is_terminated()) {
    if(NULL != proc_) {
      proc_->set_visualizer(NULL);
    }
		//control_.execute_one_step();
    get_process_control()->terminate_execution();
    get_process_control()->wait_until_terminated();
  }

  if(NULL != canvas_) {
    delete canvas_;
    canvas_ = NULL;
  }

  if(NULL != record_client_) {
    delete record_client_;
    record_client_ = NULL;
  }

  if(NULL != options_handler_) {
    delete options_handler_;
    options_handler_ = NULL;
  }
}

bool demo_frame::initialize(void)
{
  int gl_attrib[10] = { WX_GL_RGBA,WX_GL_BUFFER_SIZE,32,WX_GL_DOUBLEBUFFER, 0 };

  // Allocate canvas
  canvas_ = new wx::canvas(get_panel(), wxid("Canvas"), wxDefaultPosition, wxSize(proc_->cur_frame().ni(), proc_->cur_frame().nj()), 0, ("PUDepthCanvas"), gl_attrib );

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
  
  record_client_ = new wx::frame_record_client;
  record_client_->set_visualizer(&viz_);
  add_client(record_client_);   

  options_handler_ = new options_handler_client();
  options_handler_->set_proc(proc_);
  add_client(options_handler_);

  if(get_menu_bar()) {
    record_client_->update_menu_as_submenu(get_menu_bar(),"&Record");
    options_handler_->update_menu_as_submenu(get_menu_bar(),"&Modify Params");
  }

  // Tell process about visualizer.
  proc_->set_visualizer(&viz_);

  return true;
}

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
  gevxl::pressure_ulcer::pu_motion_feature_analysis_proc proc_;

  /// Process control that controls above process.
  gevxl::framework::process_control control_;

  // The main frame window.
  demo_frame_sptr frame_; 
};

// This is a wxWidgets construct. It hides main().
IMPLEMENT_APP(MyApp)

// This function can be viewed as the traditional main().
bool MyApp::OnInit()
{  
   vul_arg<vcl_string> a_config_file( "-c", "Config file", "C:/GEVXL/gevxl/projects/security/pressure_ulcer/config/pu_motion_feature_analysis.tconf" );
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
  frame_ = demo_frame_sptr(new demo_frame("Pressure Ulcer Motion Feature Analysis Demo"));
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

  if(!control_.is_terminated()) {
		proc_.set_visualizer(NULL);
		//control_.execute_one_step();
    control_.terminate_execution();
    control_.wait_until_terminated();
  }

  vcl_cout << "Main done. \n";

  vcl_exit(EXIT_SUCCESS);

  return 1;
}
