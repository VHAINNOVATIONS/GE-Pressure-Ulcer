//GE

#ifndef va_proc_h
#define va_proc_h



// \file
// \author Peter Tu
// \date 1/31/2006
// \par Modifications:

// This is a va video process

#include <vnl/vnl_vector.h>
#include <vil/vil_image_view.h>
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vnl/vnl_int_2.h>
#include <vnl/vnl_int_3.h>
#include <framework/process.h>
#include <vid/frame_process.h>
#include <vid/ffmpeg_frame_process.h>
#include <img/visualizer_2d.h>
#include <detectors/classifier.h>
#include <detectors/edgel_code/edgel_code_sample.h>
#include <vid/external_frame_process.h>
#include <vid/io/ffmpeg_writer.h>
#include <vid/ffmpeg_writer_process.h>
#include <util/variable_passer.h>

#include "patient_pose.h"

namespace gesec { 
  namespace detectors {

    // enumerations for bool, int float, double and vcl_string varibles
    enum bv{bv1,bv2,num_bv};
    enum iv{iv1,iv2,num_iv};
    enum fv{fv1,fv2,num_fv};
    enum dv{dv1,dv2,num_dv};
    enum vsv{vsv1,vsv2,num_vsv};

    class va_proc: public gevxl::framework::process
    {

    public: 
      va_proc(const char *name="va_proc");

      ~va_proc();

      //: allows for visualization of the regions
      virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

      //: initiazlize the process
      virtual bool initialize(void);

      //: be able to configure this proc - at this point we know what the proc_choice should be
      virtual bool configure( gevxl::util::config_file& config );

      //: print the current configuration
      void print_configuration(vcl_string filename);

      // the magical step function
      virtual bool step();

      //: Specify the source images
      void set_source_frame_process( gevxl::vid::frame_process<vxl_byte> const* source ) 
      { frame_proc_=source; }

      //: we may need to know the size of the input image
      int get_ni();
      int get_nj();

      //: get the variable passer
      gevxl::util::variable_passer* get_variable_passer()
      {return &variable_passer_;}

      //: inorder to keep track of the acutal frame, we will have the main app 
      //  tell us what the current frame number is. This will help with selecting new exemplars
      void set_true_frame_number(int tfn)
      {true_frame_number_ = tfn;}

    protected:
      //: the visualizer
      gevxl::img::visualizer_2d *viz_;

      //: the frame process
      const gevxl::vid::frame_process<vxl_byte> *frame_proc_;

      //: keeps track of variables that you want to be configured and updated via the gui
      gevxl::util::variable_passer variable_passer_;

      //: initilize the variable passer based on the enumerations of variables 
      void initialize_variable_passer();


      //: ---------- methods associated with the patient_pose code ---------------
      patient_pose patient_pose_;

      //: the true frame number 
      int true_frame_number_;

    };

  }} // end namespaces

#endif 

