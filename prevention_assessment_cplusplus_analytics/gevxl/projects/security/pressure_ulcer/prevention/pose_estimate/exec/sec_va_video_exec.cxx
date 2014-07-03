//GE


///
/// \file sec_va_video_exec.cxx
/// \brief This file will process each frame of a video with a proc, visualize it and then save the 
///        results to an output video
///

/// \author peter.tu at research.ge.com
///


#include <vcl_fstream.h>
#include <vcl_cstdlib.h>
#include <crowdseg/foreground_estimation_process.h>
#include <util/config_file.h> 

#include <vid/io/ffmpeg_reader.h>
#include <vid/ffmpeg_frame_process.h>
#include <vid/ffmpeg_writer_process.h>
#include <vid/code_stamp_frame_tag_process.h>
#include <vid/gray_frame_process.h>
#include <vid/external_frame_process.h>

#include <vid/io/ffmpeg_writer.h>
#include <framework/async_wrapping_process.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <detectors/va/va_proc.h>

#include <vil/vil_save.h>
#include <detectors/detection_writer_process.h>
#include <vid/code_stamp_frame_tag_process.h>

using namespace gevxl;
using namespace gesec;
using namespace gesec::detectors;

int main(int argc, char **argv)
{ 
	vcl_cout << "usage: sec_va_video_exec -c config_file -v input_vid -r output_vid" << vcl_endl;
  // *** get the config file ***
  // look for a -c then read the configuration file name,  if you 
  // don't see it then use config.tconf as the default value
  vul_arg<char *> 
    config_file_name("-c", "<config file name>", "config.tconf");
  vul_arg<vcl_string> video_input_name("-v", "<input video>", vcl_string("nothing"));
  vul_arg<vcl_string> video_output_name("-r", "<output video>", vcl_string("nothing"));
  
  // now parse the input parameters 
  vul_arg_parse(argc, argv);

  // now make a stream to the config file, load the config file 
  // and then close the stream

  util::config_file config;
  vcl_ifstream config_file(config_file_name());

  vcl_cout << "Trying to use the following configuration file" << config_file_name() << vcl_endl;

  if(config_file){
    config.read(config_file);
    config_file.close();
  }

  if(video_input_name() != vcl_string("nothing")){
    // change the input video name associated with the video input file
    vcl_cout << "Chaning input video to " << video_input_name() << vcl_endl;
    config.set_param(vcl_string("vid::generic_frame_process::filename"),video_input_name());
    config.set_param(vcl_string("vid::ffmpeg_frame_process::filename"),video_input_name());
  }
  

  if(video_output_name() != vcl_string("nothing")){
    // change the output video name associated with the video output file
    vcl_cout << "Changing output video to " << video_output_name() << vcl_endl;
    config.set_param(vcl_string("vid::ffmpeg_writer_process::filename"),video_output_name());
  }
    

  // this config file might need to be dynamically changed
  config.dynamic_content_update();


  // **** now configure an ffmpeg reader ****
  vid::ffmpeg_frame_process video_in_proc;



  if(!video_in_proc.configure(config)){ 
    vcl_cout << "Could not open video input file" << vcl_endl;
    exit(EXIT_FAILURE);
  }
  vcl_cout << "Opened video.\n"; 

  video_in_proc.step();


  // **** we want to be using grey level images instead of rgb
  // so this is the next step in the process
  vid::gray_frame_process<vxl_byte> gray_proc;
  gray_proc.set_source_frame_process(&video_in_proc);


  // **** Create va procss - you need to change this accordingly****
  va_proc the_proc;

  // set the input file process
  the_proc.set_source_frame_process(&gray_proc);
   
  // set up the visualizer
  gevxl::img::visualizer_image vis;
  vil_image_view<vxl_byte> vis_img(video_in_proc.width(),
				   video_in_proc.height(),
				   1,3);
  

  vis.set_output_image(&vis_img);
  
  the_proc.set_visualizer(&vis);  
  the_proc.configure(config);
  

  // set up an external frame process
  vid::external_frame_process<vxl_byte> external_proc;
  external_proc.set_view(vis_img);

  // **** now generate an ffmpeg writer ****


  vid::ffmpeg_writer_process video_out_proc;
  if(!video_out_proc.configure(config)){
    vcl_cout << "Could not open video output file" << vcl_endl;
    exit(EXIT_FAILURE);
  }


  // hook up the last process into video_out_proc
  video_out_proc.set_source_frame_process(&external_proc);


  // ****** start the loop for the entire process ******

  // check to see if there are any start and stop frames
  int start_frame = 0;
  int stop_frame = 10000000;
  int val;

  if(config.get_integer("edgel_code_proc::start_frame",val)){
    start_frame = val;
  }

  if(config.get_integer("edgel_code_proc::stop_frame",val)){
    stop_frame = val;
  }


  int skip_factor=1;
  if(config.get_integer("edgel_code_proc::skip_factor",val)){
    skip_factor = val;
  }

  int proc_frames = 0;
  int num_frames =0;
  while(1){
    num_frames++;
     if(!video_in_proc.step()){
      break;
    }
    if(num_frames > start_frame && num_frames < stop_frame){
      if(!gray_proc.step()){
        break;
      }
      // sets the image to be visualized into
      // note that this results in a deep copy into
      // vis_img, which is held by external_proc.
      // the video_out_proc just gets its current
      // frame from external_proc. 

      proc_frames++;
      if(proc_frames == skip_factor){

        proc_frames=0;

        vis.set_image(video_in_proc.cur_frame());

        // this causes part detection code to work 
        // visualized into vis_img.

	the_proc.set_true_frame_number(num_frames);
        if(!the_proc.step()){
          break;
        }

        
        // this gets vis_img from the visualizer via the 
        // external proc. The img is then pushed into the 
        // video
        video_out_proc.step();
      }
    }
  }
  video_out_proc.close();     
  

}
