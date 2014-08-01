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

int main (int argc, char** argv)
{
	// read in the config file
	vul_arg<vcl_string> a_config_file( "-c", "Config file", "E:/projects_cpp/gevxl/projects/security/pressure_ulcer/poc/pcl_sdk/exec/darpa_pc_fusion.tconf" );
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
	config.get_string("pu_pcl_sdk_darpa_pc_fusion::file_path", file_path);
	if(file_path == "") {
		vcl_cout << "file_path is not set, the program can not continue." << vcl_endl;
		return 0;
	}

	int num_of_frames = 0;
	config.get_integer("pu_pcl_sdk_darpa_pc_fusion::num_of_frames", num_of_frames);
	if(num_of_frames == 0) {
		vcl_cout << "num_of_frames is not set, the program can not continue." << vcl_endl;
		return 0;
	}

	vcl_string cloud_choice = "world";
	config.get_string("pu_pcl_sdk_darpa_pc_fusion::cloud_choice", cloud_choice);

	// start to load the kinect clouds and the world clouds
	vcl_string filename;
	vil_image_view<float> kinect_xyz_rgb_frame, world_xyz_rgb_frame;
  
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr fused_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for(int i = 0; i < num_of_frames; i++) {

		vcl_cout << "start to load the xyz_rgb frame " << i << " data." << vcl_endl;

		filename = file_path + "/kinect_xyz_rgb_" + gevxl::util::to_str(i) + ".raw";
		gevxl::img::image_raw_load(kinect_xyz_rgb_frame, filename);
		
		filename = file_path + "/world_xyz_rgb_" + gevxl::util::to_str(i) + ".raw";
		gevxl::img::image_raw_load(world_xyz_rgb_frame, filename);

		// generate the pcl point cloud based on the input and visualize the cloud
		const float nan_value = std::numeric_limits<float>::quiet_NaN ();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(world_xyz_rgb_frame.ni(), world_xyz_rgb_frame.nj()));
		for(int yy = 0, k = 0; yy < world_xyz_rgb_frame.nj(); yy++) {
			for(int xx = 0; xx < world_xyz_rgb_frame.ni(); xx++, k++) {
				
				if(kinect_xyz_rgb_frame(xx, yy, 2) == 0.0f) {
					cloud->points[k].x = nan_value;
					cloud->points[k].y = nan_value;
					cloud->points[k].z = nan_value;
				}
				else {
					if(cloud_choice == "world") {
						cloud->points[k].x = world_xyz_rgb_frame(xx, yy, 0);
						cloud->points[k].y = world_xyz_rgb_frame(xx, yy, 1);
						cloud->points[k].z = world_xyz_rgb_frame(xx, yy, 2);
					}
					else if(cloud_choice == "kinect") {
						cloud->points[k].x = kinect_xyz_rgb_frame(xx, yy, 0);
						cloud->points[k].y = kinect_xyz_rgb_frame(xx, yy, 1);
						cloud->points[k].z = kinect_xyz_rgb_frame(xx, yy, 2);
					}
				}

				if(cloud_choice == "world") {
					cloud->points[k].r = world_xyz_rgb_frame(xx, yy, 3);
					cloud->points[k].g = world_xyz_rgb_frame(xx, yy, 4);
					cloud->points[k].b = world_xyz_rgb_frame(xx, yy, 5);
				}
				else if(cloud_choice == "kinect") {
					cloud->points[k].r = kinect_xyz_rgb_frame(xx, yy, 3);
					cloud->points[k].g = kinect_xyz_rgb_frame(xx, yy, 4);
					cloud->points[k].b = kinect_xyz_rgb_frame(xx, yy, 5);
				}
			}
		}
		cloud->is_dense = false;

		*fused_cloud += *cloud;

		// save out the generated point cloud data.
		filename = file_path + "/" + cloud_choice + "_xyz_rgb_pcd_" + gevxl::util::to_str(i) + ".pcd";
		pcl::io::savePCDFile(filename.c_str(), *cloud);

		// visualize the raw point cloud
		vcl_cout << "visualize the raw point cloud " << i << " with rgb info." << vcl_endl;
		pcl::visualization::PCLVisualizer xyz_rgb_pc_viewer("xyz_rgb_pc_viewer");
		vcl_string cloud_name = "xyz_rgb_point_cloud_" + gevxl::util::to_str(i);
    
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

	// visualize the fused point cloud
	vcl_cout << "visualize the fused point cloud with rgb info." << vcl_endl;
	pcl::visualization::PCLVisualizer fused_xyz_rgb_pc_viewer("fused_xyz_rgb_pc_viewer");
	vcl_string fused_cloud_name = "fused_xyz_rgb_point_cloud";
    
	//fused_xyz_rgb_pc_viewer.addPointCloud<pcl::PointXYZRGB> (fused_cloud, fused_cloud_name.c_str());    
	//fused_xyz_rgb_pc_viewer.spin();
		
	//fused_xyz_rgb_pc_viewer.setCameraPosition(0,0,2,0,1,0,0);
	//fused_xyz_rgb_pc_viewer.removeAllPointClouds();
	//fused_xyz_rgb_pc_viewer.removeAllShapes();
	fused_xyz_rgb_pc_viewer.setBackgroundColor (0, 0, 0);
		
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> fused_rgb(fused_cloud);
	fused_xyz_rgb_pc_viewer.addPointCloud<pcl::PointXYZRGB>(fused_cloud, fused_rgb, fused_cloud_name.c_str());
	fused_xyz_rgb_pc_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, fused_cloud_name.c_str());
	//fused_xyz_rgb_pc_viewer.addCoordinateSystem (1.0, 0);
	//fused_xyz_rgb_pc_viewer.initCameraParameters ();
	  
	fused_xyz_rgb_pc_viewer.spin();
	//fused_xyz_rgb_pc_viewer.spinOnce();

  // Finish
  return 0;
}
