#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <numeric>

int main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("D:/temp/test_plane.pcd",cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *raw_cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  // segmentation
  seg.setInputCloud (raw_cloud);
  seg.segment (*inliers, *coefficients);


  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  else
  {
    double dis_sum = 0;
    for (int i=0;i<inliers->indices.size();i++)
    {
      int ind = inliers->indices.at(i);
      
      raw_cloud->points[ind].r = 255;
      raw_cloud->points[ind].g = 0;
      raw_cloud->points[ind].b = 0;
      double dis = pcl::pointToPlaneDistance(raw_cloud->points[ind], coefficients->values[0],
                                coefficients->values[1],coefficients->values[2],
                                coefficients->values[3]);
      dis_sum += dis;
    }

    std::cout<<"Average distance to plane "<<dis_sum/inliers->indices.size()<<std::endl;
  }

  //pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  pcl::visualization::PCLVisualizer viewer("viewer");
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(raw_cloud);
  viewer.addPointCloud<pcl::PointXYZRGB> (raw_cloud, rgb, "sample cloud");

  viewer.spin();

  // Finish
  return (0);
}