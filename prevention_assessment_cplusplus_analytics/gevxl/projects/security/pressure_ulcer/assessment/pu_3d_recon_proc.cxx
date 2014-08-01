// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 01/27/2014
/// \par Modifications:

#include "pu_3d_recon_proc.h"

#include <vcl_cstdio.h>
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vcl_algorithm.h>
#include <vcl_cstdlib.h>

#include <vil/vil_save.h>
#include <vil/vil_convert.h>

#include <vid/pxc_frame_process.h>

#include <shape/rectangle.h>

#include <util/rectangle.h>
#include <util/string.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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

using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::assessment;
  
pu_3d_recon_proc::pu_3d_recon_proc(char const *name)
: gevxl::framework::process(name), 	
  source_proc_(NULL), 
  viz_(NULL),
	viz_img_(NULL),
  frame_nr_(0)
{
	
}

pu_3d_recon_proc::~pu_3d_recon_proc(void)
{

}

bool pu_3d_recon_proc::configure(util::config_file &config)
{
	config_ = config;

	bool enabled = false;
	config_.get_bool(name()+"::enabled", enabled);
	enable(enabled);

	pcd_filename_ = "D:/data/pressure_ulcer/dump/intel_pcd.pcd";
	config_.get_string(name()+"::pcd_filename", pcd_filename_);

	//pcl_viewer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("pcl_3d_viewer"));

	return true;
}


bool pu_3d_recon_proc::initialize(void)
{
	if(is_enabled() == false) return true;

  return true;
}

void pu_3d_recon_proc::uninitialize(void)
{
	if(is_enabled() == false) return;
}


bool pu_3d_recon_proc::step(void)
{
	if(is_enabled() == false) return true;

  frame_nr_++;

	if(source_proc_ == NULL) {
		vcl_cerr << "pu_3d_recon_proc::step, source process is not set." << vcl_endl;
		return false;
	}

	const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(source_proc_->get_frame_process());
	if(pxc_source == NULL) {
		vcl_cerr << "pu_3d_recon_proc::step, error the video source process is not the pxc_frame_process." << vcl_endl;
		return false;
	}

	if(frame_nr_ < 10) return true;

	vil_image_view<float> cur_xyz_rgb_frame = pxc_source->cur_xyz_rgb_frame();

	const float nan_value = std::numeric_limits<float>::quiet_NaN ();

	// save the vil_image into a point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB> (cur_xyz_rgb_frame.ni(), cur_xyz_rgb_frame.nj()));
	for(int yy = 0, k = 0; yy < cur_xyz_rgb_frame.nj(); yy++) {
		for(int xx = 0; xx < cur_xyz_rgb_frame.ni(); xx++, k++) {
			if(cur_xyz_rgb_frame(xx, yy, 2) == 5.0f) {
				cloud->points[k].x = nan_value;
				cloud->points[k].y = nan_value;
				cloud->points[k].z = nan_value;
			}
			else {
				cloud->points[k].x = cur_xyz_rgb_frame(xx, yy, 0);
				cloud->points[k].y = cur_xyz_rgb_frame(xx, yy, 1);
				cloud->points[k].z = cur_xyz_rgb_frame(xx, yy, 2);
			}
			cloud->points[k].r = cur_xyz_rgb_frame(xx, yy, 3);
			cloud->points[k].g = cur_xyz_rgb_frame(xx, yy, 4);
			cloud->points[k].b = cur_xyz_rgb_frame(xx, yy, 5);
		}
	}	
	cloud->is_dense = false;

	//pcl::io::savePCDFile(pcd_filename_.c_str(), *cloud);
	//vcl_cout << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << vcl_endl;

	/*
	pcl_viewer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("pcl_3d_viewer"));
	
	pcl_viewer_->setCameraPosition(0,0,2,0,1,0,0);
	pcl_viewer_->removeAllPointClouds();
	pcl_viewer_->removeAllShapes();
	pcl_viewer_->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	pcl_viewer_->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample xyzrgb cloud");
	pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample xyzrgb cloud");
	//pcl_viewer_->addCoordinateSystem (1.0, 0);
  //pcl_viewer_->initCameraParameters ();
  
  //pcl_viewer_->spin();
	pcl_viewer_->spinOnce();
	*/

	return true;
}


void pu_3d_recon_proc::set_visualizer(gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;
}


void pu_3d_recon_proc::set_viz_offset(int viz_offset_i, int viz_offset_j)
{
	// visualizer offset
	viz_offset_i_ = viz_offset_i;
	viz_offset_j_ = viz_offset_j;
}


void pu_3d_recon_proc::visualize(void)
{	
	if(is_enabled() == false) return;
}

// visualizing to the viz_canvas_frame
void pu_3d_recon_proc::visualize_canvas(vil_image_view<vxl_byte> &viz_canvas_frame)
{
	if(is_enabled() == false) return;		
}

// visualizing to the overlay visualizer
void pu_3d_recon_proc::visualize_overlay()
{
	if(is_enabled() == false) return;

	if(viz_) {
		if(viz_->is_initialized()) { 

			int i_offset = 10, j_offset = 20;
			viz_->set_foreground(0,1,0);
		}
	}
}

//void pu_3d_recon_proc::preprocess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
void preprocess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
	// fixing nan values, converting XYZRGB cloud to XYZ cloud
	// some code has trouble with XYZRGB cloud

	////////////////////////////////////////////////////////////////////
	// According to internnet search, some PCL code crashes using XYZRGB cloud
	////////////////////////////////////////////////////////////////////
	const float nan_value = std::numeric_limits<float>::quiet_NaN ();

	//cout<<"Cloud size "<<input->size()<<endl;
	//cout<<"Nan value "<<nan_value<<endl;
	//cout<<input->width<<" "<<input->height<<endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (input->width, input->height));
	for (int k=0; k<input->size(); k++)
	{
		if (   input->points[k].x == nan_value
			|| input->points[k].y == nan_value
			|| input->points[k].z == nan_value )
		{
			cloud->points[k].x = 0;
			cloud->points[k].y = 0;
			cloud->points[k].z = 0;
		}
		else
		{
			cloud->points[k].x = input->points[k].x;
			cloud->points[k].y = input->points[k].y;
			cloud->points[k].z = input->points[k].z;
		}
	}

# if 1
	cout<<"Smoothing\n";
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (false); // crash when turn this parameter on
	mls.setPolynomialOrder(1);
	mls.setSearchRadius (0.01); // 1cm radius for local MLS
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed(new pcl::PointCloud<pcl::PointXYZ>);
	mls.process(*smoothed);
#endif

#if 0
	// Crashing...
	cout<<"Smoothing by bilateral filtering"<<endl;
	pcl::FastBilateralFilter<pcl::PointXYZ> bilateral_filter;

	bilateral_filter.setInputCloud (cloud1);
	bilateral_filter.setSigmaS (0.005);
	bilateral_filter.setSigmaR (0.005);
	bilateral_filter.applyFilter(*output);
#endif

#if 1
	// estimate normals 
	cout<<"Estimating normals\n";
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setInputCloud (smoothed);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::PointNormal>::Ptr input_normals (new pcl::PointCloud<pcl::PointNormal>);
	ne.setKSearch (15);
	ne.compute (*input_normals);

	cout<<"Filtering shadow points\n";
	// shaodow points filter does not work on pass-through filtered point cloud
	// Maybe because shadow point filter requires ORGNIZED point cloud
	// While pass-through filter generates UNORGNIZED point cloud
	pcl::ShadowPoints <pcl::PointXYZ, pcl::PointNormal> sp;
	sp.setInputCloud (smoothed);
	sp.setThreshold (0.1f);
	sp.setNormals (input_normals);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr shadow_point_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	sp.filter (*output);
#endif 

#if 0
	// Code below tries out different outlier removal
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> stat;
	stat.setInputCloud (shadow_point_filtered);
	stat.setRadiusSearch(0.02);
	stat.setMinNeighborsInRadius(10);
	stat.filter (*output);

	// pass through filter
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(shadow_point_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.1,1.0);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_filtered;
	pass.filter(*output);
#endif
}

//void pu_3d_recon_proc::align(pcl::PointCloud<pcl::PointXYZ>::Ptr &input1, pcl::PointCloud<pcl::PointXYZ>::Ptr &input2, pcl::PointCloud<pcl::PointXYZ>::Ptr &Final)
void align(pcl::PointCloud<pcl::PointXYZ>::Ptr &input1, pcl::PointCloud<pcl::PointXYZ>::Ptr &input2, pcl::PointCloud<pcl::PointXYZ>::Ptr &Final)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(input1);
  icp.setInputTarget(input2);
  icp.setUseReciprocalCorrespondences(true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final_raw(new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final_raw);

  std::cout << "ICP has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  //std::cout << icp.getFinalTransformation() << std::endl;

  *Final_raw += *input2;
  
  cout<<"Resampling & smoothing \n";
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  mls.setInputCloud (Final_raw);
  mls.setPolynomialFit (false); // crash when turn this parameter on
  mls.setPolynomialOrder(1);
  //mls.setSearchRadius (0.005);
  mls.setSearchRadius (0.01);
  pcl::PointCloud<pcl::PointXYZ>::Ptr FinalSmoothed(new pcl::PointCloud<pcl::PointXYZ>);
  mls.process(*FinalSmoothed);
  //mls.process(*Final);

  cout<<"Downsampling\n";
  // Otherwise, we will be acculumating points
  ///////////////////////////////////////////////
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (FinalSmoothed);
  sor.setLeafSize (0.002f, 0.002f, 0.002f);
  //sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*Final);

  
  //std::cout<<FinalSmoothed->size()<<std::endl;
  //std::cout<<Final->size()<<std::endl;
}

//void pu_3d_recon_proc::surfacerecon(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PolygonMesh & triangles)
void surfacerecon(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PolygonMesh & triangles)
{
  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setInputCloud (input);
  ne.setSearchMethod (tree);
  ne.setKSearch (15);
  ne.compute (*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*input, *normals, *cloud_with_normals);

  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.1);

  // Set typical values for the parameters
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors (200);
  gp3.setMaximumSurfaceAngle(M_PI);
  gp3.setMinimumAngle(M_PI/36); 
  gp3.setMaximumAngle(M_PI); 
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  
  //pcl::io::saveVTKFile ("D:/temp/test_mesh.vtk", triangles);
  //pcl::io::savePolygonFileSTL("D:/temp/test_mesh.stl", triangles);
}
