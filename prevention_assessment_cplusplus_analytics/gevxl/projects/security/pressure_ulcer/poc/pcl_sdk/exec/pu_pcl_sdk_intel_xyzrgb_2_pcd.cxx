#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>

#include <vcl_string.h>
#include <util/string.h>
#include <img/image_raw_file_io.h>

#include <vul/vul_arg.h>
#include <util/config_file.h>

#include <vcl_cstdio.h>
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vcl_algorithm.h>
#include <vcl_cstdlib.h>

#include <vul/vul_sprintf.h>
#include <vul/vul_file.h>
#include <vul/vul_file_iterator.h>

int main (int argc, char** argv)
{
	// read in the config file
	vul_arg<vcl_string> a_config_file( "-c", "Config file", "E:/projects_cpp/gevxl/projects/security/pressure_ulcer/poc/pcl_sdk/exec/intel_xyzrgb_2_pcd.tconf" );
	gevxl::util::config_file config;

	vul_arg_parse(argc, argv, false);
  config.parse_arguments(argc, argv); // process remaining command-line args
	
	if( a_config_file.set() ) {
		config.read( a_config_file().c_str() );  // process entire config file
	}
	else {
		vcl_cout << "config file is not set, the program can not continue." << vcl_endl;
		return 0;
	}
	
	vcl_string file_path = "";
	config.get_string("pu_pcl_sdk_intel_xyzrgb_2_pcd::file_path", file_path);
	if(file_path == "") {
		vcl_cout << "file_path is not set, the program can not continue." << vcl_endl;
		return 0;
	}

	// start to load the intel 3d xyz_rgb frames
	vcl_string file_path_scan = file_path + "/*";

  vcl_cout << "file_path_scan = " << file_path_scan << vcl_endl;
  for( vul_file_iterator fn = file_path_scan; fn; ++fn ) {
    
    vcl_string filename(fn());
    if(filename.substr(filename.size()-3, filename.size()) != "raw") continue;

		// load one new xyz_rgb raw frame and convert it into the pcd file		
		vil_image_view<float> xyz_rgb_frame;
		gevxl::img::image_raw_load(xyz_rgb_frame, filename);

		// convert to the pcl cloud data
		const float nan_value = std::numeric_limits<float>::quiet_NaN();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(xyz_rgb_frame.ni(), xyz_rgb_frame.nj()));

		for(int yy = 0, k = 0; yy < xyz_rgb_frame.nj(); yy++) {
			for(int xx = 0; xx < xyz_rgb_frame.ni(); xx++, k++) {

				if(xyz_rgb_frame(xx, yy, 0) == 0.0f && xyz_rgb_frame(xx, yy, 1) == 0.0f && xyz_rgb_frame(xx, yy, 2) == 0.0f) {
					cloud->points[k].x = nan_value;
					cloud->points[k].y = nan_value;
					cloud->points[k].z = nan_value;
				}
				else {
					cloud->points[k].x = xyz_rgb_frame(xx, yy, 0);
					cloud->points[k].y = xyz_rgb_frame(xx, yy, 1);
					cloud->points[k].z = xyz_rgb_frame(xx, yy, 2);
				}

				cloud->points[k].r = xyz_rgb_frame(xx, yy, 3);
				cloud->points[k].g = xyz_rgb_frame(xx, yy, 4);
				cloud->points[k].b = xyz_rgb_frame(xx, yy, 5);
			}
		}
		cloud->is_dense = false;

		filename = filename.substr(0, filename.size()-3) + "pcd";

		// save out the generated point cloud data.		
		pcl::io::savePCDFile(filename.c_str(), *cloud);

		// visualize the raw point cloud
		vcl_cout << "visualize the raw point cloud " << filename << " with rgb info." << vcl_endl;
		pcl::visualization::PCLVisualizer xyz_rgb_pc_viewer("xyz_rgb_pc_viewer");
		vcl_string cloud_name = "xyz_rgb_point_cloud";
    
    //xyz_rgb_pc_viewer.addPointCloud<pcl::PointXYZRGB> (cloud, cloud_name.c_str());    
		//xyz_rgb_pc_viewer.spin();
		
		//xyz_rgb_pc_viewer.setCameraPosition(0,0,2,0,1,0,0);
		//xyz_rgb_pc_viewer.removeAllPointClouds();
		//xyz_rgb_pc_viewer.removeAllShapes();
		xyz_rgb_pc_viewer.setBackgroundColor (0, 0, 0);
		
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		xyz_rgb_pc_viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name.c_str());
		xyz_rgb_pc_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name.c_str());
		//xyz_rgb_pc_viewer.addCoordinateSystem (1.0, 0);
		//xyz_rgb_pc_viewer.initCameraParameters ();
	  
		xyz_rgb_pc_viewer.spin();
		//xyz_rgb_pc_viewer.spinOnce();
		
	}
  // Finish
  return 0;
}
