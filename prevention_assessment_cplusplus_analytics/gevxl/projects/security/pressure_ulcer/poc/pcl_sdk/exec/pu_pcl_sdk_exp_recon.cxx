#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("D:/temp/test_pcd.pcd",cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *raw_cloud);

#if 0
  // Failed code for MLS smoothing
  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (raw_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (10, 1000.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter(*cloud);

  pcl::io::savePCDFile("D:/temp/test_pcd_filtered.pcd", *cloud);

  std::cout<<"Cloud loaded and filtered\n";

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    
  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);

  // Set parameters
  // tree->setInputCloud(cloud);

  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  // mls.setSearchMethod (tree);
  mls.setSearchRadius (10);

  // Reconstruct
  //mls.process(mls_points);

  pcl::io::savePCDFile("D:/temp/test_pcd_smoothed.pcd", *cloud);
  std::cout<<"Cloud smoothed\n";
#endif

#if 0
  // Failed code for meshing from unordered point cloud
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.01);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (200);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();


  //pcl::io::saveVTKFile ("D:/temp/test_mesh.vtk", triangles);
  pcl::io::savePolygonFileSTL("D:/temp/test_mesh.stl", triangles);
  
#endif

  pcl::OrganizedFastMesh<pcl::PointXYZ> ofm;
  pcl::PolygonMesh triangles;

  ofm.setMaxEdgeLength(0.01);
  ofm.setTrianglePixelSize(1);
  ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_ADAPTIVE_CUT);
  ofm.storeShadowedFaces(true);

  ofm.setInputCloud(raw_cloud);
  ofm.reconstruct (triangles);

  std::cout<<"Mesh created\n";


  pcl::MeshSmoothingLaplacianVTK smoother;
  pcl::PolygonMeshConstPtr meshin(&triangles);
  smoother.setInputMesh(meshin);
  smoother.setNumIter(2000); 
  smoother.setConvergence(0.001); 
  smoother.setRelaxationFactor(0.001); 
  smoother.setFeatureEdgeSmoothing(false); 
  smoother.setFeatureAngle(M_PI/2); 
  smoother.setBoundarySmoothing(true); 
  pcl::PolygonMesh smooth_triangles;
  smoother.process(smooth_triangles);
  std::cout<<"Mesh smoothed\n";

  pcl::io::saveVTKFile ("D:/temp/test_mesh.vtk", smooth_triangles);
  
  //pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  pcl::visualization::PCLVisualizer viewer("viewer");
  //viewer.addPointCloud<pcl::PointXYZ> (raw_cloud, "sample cloud");
  viewer.addPolygonMesh(smooth_triangles);
  viewer.spin();

  // Finish
  return (0);
}