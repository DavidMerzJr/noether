#include <noether_edge_paths/edge_paths.h>

#include <pcl/conversions.h>

namespace noether_edge_paths
{

EdgePathGenerator::EdgePathGenerator()
{
  input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

  resolution_ = 0.01;
  num_neighbors_ = 20;
  search_radius_ = 0.05;
  interior_path_angle_ = 0.0;
  exterior_path_angle_ = 0.0;
  config_.pt_spacing = 0.1;
  config_.tool_offset = 0.0;
}

void EdgePathGenerator::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud)
{
  input_cloud_ = input_cloud;
  return;
}

void EdgePathGenerator::setInputMesh(const pcl::PolygonMesh& input_mesh)
{
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  pcl::fromPCLPointCloud2(input_mesh.cloud, input_cloud);
  input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(input_cloud));
  return;
}

void EdgePathGenerator::computeEdgePaths(noether_msgs::ToolRasterPath& paths)
{
  paths.paths.clear();

  // Calculate Paths
  // Do the PCA
  edge_detector_.setInputCloud(input_cloud_, resolution_);
  edge_detector_.solve(search_radius_, num_neighbors_);
  edge_results_ = edge_detector_.getEdgeResults();

  // Evaluate the PCA results to identify edges
  // Extract the edges from the mesh/cloud
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr edginess_cloud;
  evaluateEdginess(edge_results_, edginess_cloud);

  // Separate the continuous edge segments
    // - some manner of region growing?
    // Use connectivity in half-edge mesh to add to list before calling next one
  // Identify the external edge (longest / most points?)
    // Connectivity of half-edge mesh - which has the most half-edge segments in it?
    // Or just most points.
    // Either way assumes even point spacing, which may not be correct.
    // Could actually calculate the distance of each segment to get more accurate number.
    // Could calculate the distance from each point to 3-4 points back in order to smooth the distance number.
  // Separate each individual edge 'side'

  // Fit B-Splines to each edge group
    // Collection of points / submesh
    // calculate general/average normal
  // Place evenly spaced waypoints along each B-Spline
    // It's parameterized, so that's cool.
  // Rotate the b-spline points, then apply standoff
    // rotate relative to existing e

  return;
}

} // end namespace noether_edge_paths
