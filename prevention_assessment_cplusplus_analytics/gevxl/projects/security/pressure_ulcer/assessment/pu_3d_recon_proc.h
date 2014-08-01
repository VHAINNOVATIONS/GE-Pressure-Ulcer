// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 01/27/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_3d_recon_proc_h_
#define gevxl_pressure_ulcer_pu_3d_recon_proc_h_

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>

#include <img/visualizer_2d.h>
#include <img/visualizer_image.h>

#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>
#include <vid/code_stamp_frame_tag_process.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <threading/mutex.h>

#include <vnl/vnl_int_2.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace assessment {
  
			class pu_3d_recon_proc : public gevxl::framework::process,
															 public gevxl::util::on_off_mixin
			{
			public:

				// Constructor
				pu_3d_recon_proc(char const *name="gevxl::pressure_ulcer::pu_3d_recon_proc"); 

				// Destructor
				virtual ~pu_3d_recon_proc(void); 

				// Configure this proc
				virtual bool configure(gevxl::util::config_file &config);

				// Initialize the process
				virtual bool initialize(void);

				// Uninitialize the process
				virtual void uninitialize(void);

				// The main step function
				virtual bool step(void);
	      
				// set the process's visualizer
				virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

				// set the viz offset
				virtual void set_viz_offset(int viz_offset_i, int viz_offset_j);

				// visualization function
				virtual void visualize(void);

				// visualizing to the viz_canvas_frame
				virtual void visualize_canvas(vil_image_view<vxl_byte> &viz_canvas_frame);

				// visualizing to the overlay visualizer
				virtual void visualize_overlay();

				// get the current frame from the source process
				const vil_image_view<vxl_byte> &cur_frame(void) const { return source_proc_->cur_frame(); }

				// set the source process
				void set_source_process(gevxl::vid::generic_frame_process<vxl_byte> *src_proc) { source_proc_ = src_proc; }

				// get the source process
				gevxl::vid::generic_frame_process<vxl_byte> &get_source_process(void) { return *source_proc_; }				

			private:
				
				// preprocess the point cloud
				//void preprocess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output);

				// align two point clouds
				//void align(pcl::PointCloud<pcl::PointXYZ>::Ptr &input1, pcl::PointCloud<pcl::PointXYZ>::Ptr &input2, pcl::PointCloud<pcl::PointXYZ>::Ptr &Final);

				// surface reconstruction
				//void surfacerecon(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PolygonMesh & triangles);

				// Input Source
				gevxl::vid::generic_frame_process<vxl_byte> *source_proc_;

				// Input point cloud xyz_rgb frame
				vil_image_view<float> xyz_rgb_frame_;

				// high resolution timer
				gevxl::util::time::highres_timer highres_timer_; 

				// visualizer 	
				gevxl::img::visualizer_2d *viz_;

				// visualizer offset
				int viz_offset_i_;
				int viz_offset_j_;

				// visualizer image
				gevxl::img::visualizer_image *viz_img_;

				// the configuration file that the generator needs in order to configure its own parameters.
				gevxl::util::config_file config_;
			
				// frame number count
				int frame_nr_;

				// debug info
				vcl_string pcd_filename_;
	
				pcl::visualization::PCLVisualizer::Ptr pcl_viewer_;

			};  // end of class pu_3d_recon_proc

			typedef vbl_shared_pointer<pu_3d_recon_proc> pu_3d_recon_proc_sptr;

		} // end of assessment namespace
  } //end namespace pressure_ulcer
} //end namespace gevxl


#endif
