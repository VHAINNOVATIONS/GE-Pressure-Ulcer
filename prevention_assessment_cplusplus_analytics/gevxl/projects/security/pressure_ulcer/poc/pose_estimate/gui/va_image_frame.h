#ifndef va_image_frame_h_
#define va_image_frame_h_

//GE


/// \file
/// \author Peter Tu
/// \date Aug 23, 2010

// This frame is used for an executable that can exercize the 
// image processing filters wrapped in the imager filtering client
#include <wx/wx.h>
#include <wx/frame.h>

#include <gui/utils.h>
#include <gui/wx/canvas.h>
#include <gui/wx/client.h>
#include <gui/wx/handler_client.h>
#include <gui/wx/direct_visualizer.h>
#include <gui/wx/canvas_view_client.h>
#include <img/visualizer_2d_buffered.h>
#include <img/gui/image_io_client.h>
#include <img/gui/image_filter_client.h>
#include <util/config_file.h>
#include <detectors/va/gui/va_image_client.h>


namespace gesec { 
namespace detectors {



/// \brief Brief description of this class.
///
/// More details on this class.
class va_image_frame: public wxFrame
{
public:
  va_image_frame(wxFrame *frame, const wxString& title,
              const wxPoint& pos, const wxSize& size, 
              long style = wxDEFAULT_FRAME_STYLE);

  virtual ~va_image_frame();

  virtual void init(bool double_buffer=false);

  // we need to specify the config file so that it can 
  // be used to configure the clients...
  void configure(gevxl::util::config_file *config) {config_ = config;}

protected:
  gevxl::util::config_file *config_;

  void on_tick(wxTimerEvent &event);
  void setup_canvas(void);
  void setup_layout(void);
  void setup_visualization(void);
  void setup_clients(void);
  void setup_menus(void);

  bool double_buffer_;

  /// The visualizer that does the OpenGL rendering into the canvas.
  gevxl::gui::wx::direct_visualizer viz_[1];

  /// The visualizer that buffers draw commands from clients.
  /// The content of this visualizer is extracted by viz_ and 
  /// rendered into the canvas.
  gevxl::img::visualizer_2d_buffered overlay_viz_[1];

  /// Pointer to canvas (OpenGL rendering area).
  gevxl::gui::wx::canvas *canvas_[1];

  /// A client that deals with zooming and panning of the frame in the canvas.
  gevxl::gui::wx::canvas_view_client *view_client_[1];

  /// An example handler client that waits for mouse clicks and draws line into the
  /// overlay visualizer.
  
  //: an  image io client
  gevxl::img::image_io_client *io_client_[1];

  //: also make a va 
  va_image_client *va_image_client_;

  //: an image filter client
  gevxl::img::image_filter_client *ifm_client_;


  //: this seems to be needed for embedding multiple canvases...
  wxPanel *panel_;
};




}} // end namespaces

#endif
