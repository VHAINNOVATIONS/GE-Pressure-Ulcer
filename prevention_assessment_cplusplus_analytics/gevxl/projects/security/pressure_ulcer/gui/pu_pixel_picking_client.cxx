// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_pixel_picking_client.h"

#include <vcl_fstream.h>
#include <vcl_set.h>
#include <vcl_cstdlib.h>

#include <util/string.h>

#include <wx/filedlg.h>
#include <wx/filename.h>

#include <gui/wx/settings_dialog.h>
#include <gui/wx/canvas.h>
#include <gui/wx/wxid.h>

#include <img/style.h>

#include <vid/openni2_frame_process.h>
#include <vid/micro_epsilon_socket_frame_process.h>
#include <vid/pxc_frame_process.h>

#ifdef GEVXL_PUGUI_HAS_PCL_SDK
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#endif


#ifdef VCL_WIN32
#include <wx/msw/registry.h>
#define HAS_REGISTRY
#endif

using namespace gevxl::vid;
using namespace gevxl::pressure_ulcer;

pu_pixel_picking_client::pu_pixel_picking_client(void) 
: proc_(NULL), 
  menu_(NULL),
  thermal_vis_in_F_menu_(NULL),
  thermal_vis_in_C_menu_(NULL),
	intel_3d_vis_xyz_menu_(NULL),
  intel_3d_vis_xyz_rgb_menu_(NULL),
	openni2_3d_vis_xyz_menu_(NULL),
  openni2_3d_vis_xyz_rgb_menu_(NULL)  
{

}

/// Default destructor.
pu_pixel_picking_client::~pu_pixel_picking_client(void)
{

}

void pu_pixel_picking_client::update_menu(wxMenu *menu)
{
  menu_ = menu;

  // thermal Camera Menu section
  wxMenu *thermal_camera_menu = new wxMenu;
  
  thermal_vis_in_F_menu_ = append_check(thermal_camera_menu, "Visualize Temperature in Fahrenheit\tF", "Visualize Temperature in Fahrenheit.");
  thermal_vis_in_C_menu_ = append_check(thermal_camera_menu, "Visualize Temperature in Celsius\tC", "Visualize Temperature in Celsius.");

  append(thermal_camera_menu, menu_, "Thermal Camera", "Visualization Setting for Thermal Camera.");

  separator(menu_);

	// Intel 3D Camera Menu section
  wxMenu *intel_3d_camera_menu = new wxMenu;
  
  intel_3d_vis_xyz_menu_ = append_check(intel_3d_camera_menu, "Visualize Pixel's (X,Y,Z) Coordinates\tD", "Visualize Pixel's (X,Y,Z) Coordinates.");
  intel_3d_vis_xyz_rgb_menu_ = append_check(intel_3d_camera_menu, "Visualize Pixel's (X,Y,Z) Coordinates and (R,G,B) Color\tR", "Visualize Pixel's (X,Y,Z) Coordinates and (R,G,B) Color.");

  append(intel_3d_camera_menu, menu_, "Intel 3D Camera", "Visualization Setting for Intel 3D Camera.");

  separator(menu_);

  // openni2 3D Camera Menu section
  wxMenu *openni2_3d_camera_menu = new wxMenu;
  
  openni2_3d_vis_xyz_menu_ = append_check(openni2_3d_camera_menu, "Visualize Pixel's (X,Y,Z) Coordinates\tS", "Visualize Pixel's (X,Y,Z) Coordinates.");
  openni2_3d_vis_xyz_rgb_menu_ = append_check(openni2_3d_camera_menu, "Visualize Pixel's (X,Y,Z) Coordinates and (R,G,B) Color\tE", "Visualize Pixel's (X,Y,Z) Coordinates and (R,G,B) Color.");

  append(openni2_3d_camera_menu, menu_, "Openni2 3D Camera", "Visualization Setting for Openni2 3D Camera.");

  separator(menu_);
} 

void pu_pixel_picking_client::on_mouse(gevxl::gui::event_source *source, wxMouseEvent &ev)
{
  //vcl_cout << "pu_pixel_picking_client::on_mouse." <<  vcl_endl;

#ifdef GEVXL_PUGUI_HAS_PCL_SDK
  if ( ev.RightIsDown() && proc_->get_source_process_type() == "pcsdk") 
  {
     const gevxl::vid::pxc_frame_process *pxc_source 
       = dynamic_cast<const gevxl::vid::pxc_frame_process *>(proc_->get_source_process().get_frame_process());

      const vil_image_view<float> &cur_xyz_rgb_frame = pxc_source->cur_xyz_rgb_frame();
      if(cur_xyz_rgb_frame.nplanes() != 6) return;

      const float nan_value = std::numeric_limits<float>::quiet_NaN ();
   
      // save the vil_image into a point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (cur_xyz_rgb_frame.ni(), cur_xyz_rgb_frame.nj()));
      for (int yy=0, k=0; yy<cur_xyz_rgb_frame.nj(); yy++)
      {
        for (int xx=0; xx<cur_xyz_rgb_frame.ni(); xx++, k++)
        {
          if (cur_xyz_rgb_frame(xx,yy, 2)==5000.0f) 
          {
            cloud->points[k].x = nan_value;
            cloud->points[k].y = nan_value;
            cloud->points[k].z = nan_value;
          }
          else
          {
            cloud->points[k].x = cur_xyz_rgb_frame(xx,yy, 0);
            cloud->points[k].y = cur_xyz_rgb_frame(xx,yy, 1);
            cloud->points[k].z = cur_xyz_rgb_frame(xx,yy, 2);
          }
          //cloud->points[k].r = cur_xyz_rgb_frame(xx,yy, 3);
          //cloud->points[k].g = cur_xyz_rgb_frame(xx,yy, 4);
          //cloud->points[k].b = cur_xyz_rgb_frame(xx,yy, 5);
        }
      }
      //cloud->is_dense = false;
       
      pcl::io::savePCDFile("D:/temp/test_pcd.pcd", *cloud);
      vcl_cout << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << vcl_endl;
  }
#endif

  if(thermal_vis_in_F_menu_->IsChecked() == false && thermal_vis_in_C_menu_->IsChecked() == false && 
    intel_3d_vis_xyz_menu_->IsChecked() == false && intel_3d_vis_xyz_rgb_menu_->IsChecked() == false &&
		openni2_3d_vis_xyz_menu_->IsChecked() == false && openni2_3d_vis_xyz_rgb_menu_->IsChecked() == false) {
    return;
  }

  if(source == NULL) return;

  gevxl::gui::wx::canvas *c = dynamic_cast<gevxl::gui::wx::canvas *>(source);
  if(c == NULL) return;



  if( (ev.LeftIsDown()) && 
      (intel_3d_vis_xyz_menu_->IsChecked() || intel_3d_vis_xyz_rgb_menu_->IsChecked() ||
			 openni2_3d_vis_xyz_menu_->IsChecked() || openni2_3d_vis_xyz_rgb_menu_->IsChecked() ||
       thermal_vis_in_F_menu_->IsChecked() || thermal_vis_in_C_menu_->IsChecked()) ) {
    
    int img_x = c->get_mouse_x();
    int img_y = c->get_mouse_y();

    if(proc_->get_source_process_type() == "openni2") {
      const gevxl::vid::openni2_frame_process *openni2_source 
        = dynamic_cast<const gevxl::vid::openni2_frame_process *>(proc_->get_source_process().get_frame_process());

			if(openni2_source->is_point_cloud_frame_available()) {
				// visualize pixel's point cloud x, y, z, and r, g, b
				const vil_image_view<float> &cur_xyz_rgb_frame = openni2_source->cur_xyz_rgb_frame();
				if(cur_xyz_rgb_frame.nplanes() != 6) return;

				if(img_x < 0 || img_x >= cur_xyz_rgb_frame.ni() || 
					 img_y < 0 || img_y >= cur_xyz_rgb_frame.nj()) return;

				float x,y,z,r,g,b;
				x = cur_xyz_rgb_frame(img_x, img_y, 0)*100; // in centi-meter unit
				y = cur_xyz_rgb_frame(img_x, img_y, 1)*100; // in centi-meter unit
				z = cur_xyz_rgb_frame(img_x, img_y, 2)*100; // in centi-meter unit
				r = cur_xyz_rgb_frame(img_x, img_y, 3);
				g = cur_xyz_rgb_frame(img_x, img_y, 4);
				b = cur_xyz_rgb_frame(img_x, img_y, 5);

				int text_height = 20;
				// visualize the extracted pixel attributes into the canvas source using the visualizer_2d.
				gevxl::img::visualizer_2d *viz = c->get_visualizer();
				if(viz) {
					viz->lock();
					if(!viz->is_initialized()) {
						vcl_cout << "Viz is not initialized." << vcl_endl;
					}
					else {
						viz->clear();

						viz->set_foreground(0,0,0);
						viz->set_background(0.5,0.5,0.5,0.5);
	      
						vcl_string str_prefix;
						if(openni2_3d_vis_xyz_menu_->IsChecked()) {
              str_prefix = "(i,j)=";
							viz->add_text(float(img_x), float(img_y-text_height), str_prefix+"("+util::to_str(img_x)+","+util::to_str(img_y)+")", true, true, "below");
							str_prefix = "x=";
							viz->add_text(float(img_x), float(img_y), str_prefix+util::to_str(x), true, true, "below");
							str_prefix = "y=";
							viz->add_text(float(img_x), float(img_y+text_height), str_prefix+util::to_str(y), true, true, "below");
							str_prefix = "z=";
							viz->add_text(float(img_x), float(img_y+2*text_height), str_prefix+util::to_str(z), true, true, "below");
						}
						else if(openni2_3d_vis_xyz_rgb_menu_->IsChecked()) {
              str_prefix = "(i,j)=";
							viz->add_text(float(img_x), float(img_y-text_height), str_prefix+"("+util::to_str(img_x)+","+util::to_str(img_y)+")", true, true, "below");
							str_prefix = "x=";
							viz->add_text(float(img_x), float(img_y), str_prefix+util::to_str(x), true, true, "below");
							str_prefix = "y=";
							viz->add_text(float(img_x), float(img_y+text_height), str_prefix+util::to_str(y), true, true, "below");
							str_prefix = "z=";
							viz->add_text(float(img_x), float(img_y+2*text_height), str_prefix+util::to_str(z), true, true, "below");
							str_prefix = "r=";
							viz->add_text(float(img_x), float(img_y+3*text_height), str_prefix+util::to_str(r), true, true, "below");
							str_prefix = "g=";
							viz->add_text(float(img_x), float(img_y+4*text_height), str_prefix+util::to_str(g), true, true, "below");
							str_prefix = "b=";
							viz->add_text(float(img_x), float(img_y+5*text_height), str_prefix+util::to_str(b), true, true, "below");
						}         
					}
					c->post_redraw_event();
					viz->unlock();              
				}
				else {
					vcl_cout << "can't get viz" << vcl_endl;
				}
			}
    }
    else if (proc_->get_source_process_type() == "micro_epsilon_socket"){
      const gevxl::vid::micro_epsilon_socket_frame_process *micro_epsilon_socket_source 
        = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(proc_->get_source_process().get_frame_process());      

      // visualize pixel's temperature from the Micro-Epsilon's thermal camera
      const vil_image_view<float> &cur_thermal_frame_in_C = micro_epsilon_socket_source->cur_thermal_frame_in_C();
      const vil_image_view<float> &cur_thermal_frame_in_F = micro_epsilon_socket_source->cur_thermal_frame_in_F();
      
      if(cur_thermal_frame_in_C.nplanes() != 1 || cur_thermal_frame_in_F.nplanes() != 1) return;

      if(img_x < 0 || img_x >= cur_thermal_frame_in_C.ni() || 
         img_y < 0 || img_y >= cur_thermal_frame_in_C.nj() || 
         img_x >= cur_thermal_frame_in_F.ni() || img_y >= cur_thermal_frame_in_F.nj()) return;

      float temperature_in_C, temperature_in_F;
      temperature_in_C = cur_thermal_frame_in_C(img_x, img_y);
      temperature_in_F = cur_thermal_frame_in_F(img_x, img_y);

      int text_height = 20;

      // visualize the extracted pixel attributes into the canvas source using the visualizer_2d.
      gevxl::img::visualizer_2d *viz = c->get_visualizer();
      if(viz) {
        viz->lock();
        if(!viz->is_initialized()) {
          vcl_cout << "Viz is not initialized." << vcl_endl;
        }
        else {
          viz->clear();

          viz->set_foreground(0,0,0);
          viz->set_background(0.5,0.5,0.5,0.5);
      
          vcl_string str_prefix;
          if(thermal_vis_in_C_menu_->IsChecked()) {
            str_prefix = "(i,j)=";
						viz->add_text(float(img_x), float(img_y-text_height), str_prefix+"("+util::to_str(img_x)+","+util::to_str(img_y)+")", true, true, "below");
            str_prefix = "temperature in C = ";
            viz->add_text(float(img_x), float(img_y), str_prefix+util::to_str(temperature_in_C), true, true, "below");
          }
          else if(thermal_vis_in_F_menu_->IsChecked()) {
            str_prefix = "(i,j)=";
						viz->add_text(float(img_x), float(img_y-text_height), str_prefix+"("+util::to_str(img_x)+","+util::to_str(img_y)+")", true, true, "below");
            str_prefix = "temperature in F = ";
            viz->add_text(float(img_x), float(img_y), str_prefix+util::to_str(temperature_in_F), true, true, "below");
          }
          
        }
        c->post_redraw_event();
        viz->unlock();              
      }
      else {
        vcl_cout << "can't get viz" << vcl_endl;
      }
    }
    else if(proc_->get_source_process_type() == "pcsdk") {
      const gevxl::vid::pxc_frame_process *pxc_source 
        = dynamic_cast<const gevxl::vid::pxc_frame_process *>(proc_->get_source_process().get_frame_process());

      // visualize pixel's point cloud x, y, z, and r, g, b
      const vil_image_view<float> &cur_xyz_rgb_frame = pxc_source->cur_xyz_rgb_frame();
      if(cur_xyz_rgb_frame.nplanes() != 6) return;

      if(img_x < 0 || img_x >= cur_xyz_rgb_frame.ni() || 
         img_y < 0 || img_y >= cur_xyz_rgb_frame.nj()) return;

      float x,y,z,r,g,b;
      x = cur_xyz_rgb_frame(img_x, img_y, 0)*100; // in centi-meter unit
      y = cur_xyz_rgb_frame(img_x, img_y, 1)*100; // in centi-meter unit
      z = cur_xyz_rgb_frame(img_x, img_y, 2)*100; // in centi-meter unit
      r = cur_xyz_rgb_frame(img_x, img_y, 3);
      g = cur_xyz_rgb_frame(img_x, img_y, 4);
      b = cur_xyz_rgb_frame(img_x, img_y, 5);

      int text_height = 20;
      // visualize the extracted pixel attributes into the canvas source using the visualizer_2d.
      gevxl::img::visualizer_2d *viz = c->get_visualizer();
      if(viz) {
        viz->lock();
        if(!viz->is_initialized()) {
          vcl_cout << "Viz is not initialized." << vcl_endl;
        }
        else {
          viz->clear();

          viz->set_foreground(0,0,0);
          viz->set_background(0.5,0.5,0.5,0.5);
      
          vcl_string str_prefix;
          if(intel_3d_vis_xyz_menu_->IsChecked()) {
            str_prefix = "(i,j)=";
						viz->add_text(float(img_x), float(img_y-text_height), str_prefix+"("+util::to_str(img_x)+","+util::to_str(img_y)+")", true, true, "below");
            str_prefix = "x=";
            viz->add_text(float(img_x), float(img_y), str_prefix+util::to_str(x), true, true, "below");
            str_prefix = "y=";
            viz->add_text(float(img_x), float(img_y+text_height), str_prefix+util::to_str(y), true, true, "below");
            str_prefix = "z=";
            viz->add_text(float(img_x), float(img_y+2*text_height), str_prefix+util::to_str(z), true, true, "below");
          }
          else if(intel_3d_vis_xyz_rgb_menu_->IsChecked()) {
            str_prefix = "(i,j)=";
						viz->add_text(float(img_x), float(img_y-text_height), str_prefix+"("+util::to_str(img_x)+","+util::to_str(img_y)+")", true, true, "below");
            str_prefix = "x=";
            viz->add_text(float(img_x), float(img_y), str_prefix+util::to_str(x), true, true, "below");
            str_prefix = "y=";
            viz->add_text(float(img_x), float(img_y+text_height), str_prefix+util::to_str(y), true, true, "below");
            str_prefix = "z=";
            viz->add_text(float(img_x), float(img_y+2*text_height), str_prefix+util::to_str(z), true, true, "below");
            str_prefix = "r=";
            viz->add_text(float(img_x), float(img_y+3*text_height), str_prefix+util::to_str(r), true, true, "below");
            str_prefix = "g=";
            viz->add_text(float(img_x), float(img_y+4*text_height), str_prefix+util::to_str(g), true, true, "below");
            str_prefix = "b=";
            viz->add_text(float(img_x), float(img_y+5*text_height), str_prefix+util::to_str(b), true, true, "below");
          }         
        }
        c->post_redraw_event();
        viz->unlock();              
      }
      else {
        vcl_cout << "can't get viz" << vcl_endl;
      }
    }     
  }

  ev.Skip();
}
