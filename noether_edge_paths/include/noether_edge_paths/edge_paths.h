#ifndef EDGE_PATHS_H
#define EDGE_PATHS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include <noether_msgs/ToolPathConfig.h>
#include <noether_msgs/ToolRasterPath.h>

#include <robust_edge_extraction/robust_edge_extraction.h>
#include <pcl_bspline_reconstruction/bspline_reconstruction.h>

namespace noether_edge_paths
{

class EdgePathGenerator
{
public:
  EdgePathGenerator();

  void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud);
  void setInputMesh(const pcl::PolygonMesh& input_mesh);

  void setResolution(const double resolution) { resolution_ = resolution; }
  void setNumberNeighbors(const int num_neighbors) { num_neighbors_ = num_neighbors; }
  void setInteriorPathAngle(const double in_degrees_from_normal) { interior_path_angle_ = in_degrees_from_normal; }
  void setExteriorPathAngle(const double in_degrees_from_normal) { exterior_path_angle_ = in_degrees_from_normal; }
  void setToolPathParams(const noether_msgs::ToolPathConfig& config) { config_ = config; }

  /**
   * @brief computeEdgePaths - computes paths which roughly follow the
   * outline of the supplied mesh/point cloud.  They break on hard corners
   * so that speed constraints may be more easily met.
   * @param paths - output - PoseArray[], an array of arrays of poses.
   * Each PoseArray will represent one stroke along one continuous line.
   * They will break on hard corners.
   */
  void computeEdgePaths(noether_msgs::ToolRasterPath& paths);

private:
  // Sorts points into not-edges, edges, and corners
  void evaluateEdginess(const pcl::PointCloud<pcl::PointXYZHSV>::ConstPtr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::ConstPtr output_cloud);

  robust_edge_extraction::RobustEdgeExtraction edge_detector_;
  bspline_reconstruction::BSplineReconstruction bspline_generator_;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud_;
  pcl::PointCloud<pcl::PointXYZHSV>::ConstPtr edge_results_;

  double resolution_;
  double search_radius_;
  int num_neighbors_;
  double interior_path_angle_;
  double exterior_path_angle_;
  noether_msgs::ToolPathConfig config_;

}; // end class EdgePathGenerator

} // end namespace noether_edge_paths

// How to decide to break one edge from the next?

#endif // EDGE_PATHS_H
