#ifndef ROBUST_EDGE_EXTRACTION_H
#define ROBUST_EDGE_EXTRACTION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <omp.h>
#include <memory>

namespace robust_edge_extraction
{

class RobustEdgeExtraction
{
public:

  RobustEdgeExtraction() = default;
  virtual ~RobustEdgeExtraction() = default;

  /**
   * @brief Set the input cloud
   * @param input_cloud The input point cloud
   * @param resolution The resolution at which to construct he octree for radius searches
   */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const double resolution);

  /**
   * @brief Generate eigen value analysis for edge extraction
   * @param search_radius The PCA search radius for calculating eigen values
   * @param num_threads The number of threads to use, default omp_get_max_threads()
   * @return If true successfully, otherwise false
   */
  bool solve(const double search_radius, const int num_threads = omp_get_max_threads());

  /**
   * @brief Get the results where
   *
   *   The H  = sigma1 = lambda(0)/(lambda(0) + lambda(1) + lambda(2))
   *   The S  = sigma2 = lambda(1)/(lambda(0) + lambda(1) + lambda(2))
   *   The V  = sigma3 = lambda(2)/(lambda(0) + lambda(1) + lambda(2))
   *
   * @return Point cloud where intensity represents sigma1
   */
  pcl::PointCloud<pcl::PointXYZHSV>::ConstPtr getEdgeResults() const { return edge_results_; }

protected:
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud_;
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr edge_results_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_search_;
};

};
#endif // ROBUST_EDGE_EXTRACTION_H
