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

#define N 9
// Smoothing and filtering outliers
void preprocess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
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

void align(pcl::PointCloud<pcl::PointXYZ>::Ptr &input1,
           pcl::PointCloud<pcl::PointXYZ>::Ptr &input2,
           pcl::PointCloud<pcl::PointXYZ>::Ptr &Final)
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

void surfacerecon(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
                  pcl::PolygonMesh & triangles)
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

int main (int argc, char** argv)
{
  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
  for (int i=0;i<N;i++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 cloud_blob;
    char filename[256];
    sprintf(filename,"D:/temp/test_pcd%d.pcd", i);
    pcl::io::loadPCDFile (filename, cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud1);

		// visualize the raw point cloud
		cout << "visualize the raw point cloud " << i << " with rgb info.\n";
		pcl::visualization::PCLVisualizer raw_point_cloud_viewer("raw_point_cloud_viewer");
		char cloudname[256];
    sprintf(cloudname,"raw_point_cloud%d", i);
    raw_point_cloud_viewer.addPointCloud<pcl::PointXYZRGB> (cloud1, cloudname);    
		raw_point_cloud_viewer.spin();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    preprocess(cloud1, cloud1_filtered);
    clouds.push_back(cloud1_filtered);
  }

  /////////////////////////////////////////////
  // 1. Run PCL between two data sets
  /////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
  align(clouds[0], clouds[1], Final);
  for (int i=2;i<N;i++)
    align(Final, clouds[i], Final);

  // 2. Creatiing mesh
  //////////////////////////////////////////////
  pcl::PolygonMesh triangles;
  surfacerecon(Final, triangles);

  // 3. visualization
  /////////////////////////////////////////////////////////////////////////
  pcl::visualization::PCLVisualizer viewer2("viewer2");

  for (int i=0;i<N;i++)
  {
    char cloudname[256];
    sprintf(cloudname,"cloud%d",i);
    viewer2.addPointCloud<pcl::PointXYZ> (clouds[i], cloudname);
  }
  
  viewer2.spin();

  pcl::visualization::PCLVisualizer viewerF("viewerF");
  viewerF.addPointCloud<pcl::PointXYZ> (Final,  "Final");

  viewerF.spin();

  pcl::visualization::PCLVisualizer viewerT("viewer mesh");
  viewerT.addPolygonMesh(triangles);
  viewerT.spin();


  // Finish
  return (0);
}
