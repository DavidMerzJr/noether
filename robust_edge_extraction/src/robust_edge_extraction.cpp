
#include <robust_edge_extraction/robust_edge_extraction.h>

#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iostream>

namespace robust_edge_extraction
{

void RobustEdgeExtraction::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const double resolution)
{
  input_cloud_ = input_cloud;
  octree_search_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution));
  octree_search_->setInputCloud (input_cloud_);
  octree_search_->addPointsFromInputCloud ();
};

bool RobustEdgeExtraction::solve(const double search_radius, const int num_threads)
{

  edge_results_.reset (new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::copyPointCloud(*input_cloud_, *edge_results_);

  long cnt = 0;
  long percentage = 0;
  long num_points = static_cast<long>(input_cloud_->size());

#pragma omp parallel for num_threads(num_threads)
  for (std::size_t idx = 0; idx < input_cloud_->points.size(); ++idx)
  {
    std::vector<int> point_idx;
    std::vector<float> point_squared_distance;
    octree_search_->radiusSearch (static_cast<int>(idx), search_radius, point_idx, point_squared_distance);

    Eigen::Vector4d centroid;
    Eigen::Matrix3d covariance_matrix;

    pcl::computeMeanAndCovarianceMatrix (*input_cloud_,
                                         point_idx,
                                         covariance_matrix,
                                         centroid);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.compute(covariance_matrix, false);
    auto eigen_values = solver.eigenvalues();
    double sigma1 = eigen_values(0)/(eigen_values(0) + eigen_values(1) + eigen_values(2));
    double sigma2 = eigen_values(1)/(eigen_values(0) + eigen_values(1) + eigen_values(2));
    double sigma3 = eigen_values(2)/(eigen_values(0) + eigen_values(1) + eigen_values(2));
    edge_results_->points[idx].h = static_cast<float>(sigma1);
    edge_results_->points[idx].s = static_cast<float>(sigma2);
    edge_results_->points[idx].v = static_cast<float>(sigma3);

#pragma omp critical
    {
      ++cnt;
      if (static_cast<long>(100.0f * static_cast<float>(cnt)/num_points) > percentage)
      {
        percentage = static_cast<long>(100.0f * static_cast<float>(cnt)/num_points);
        std::stringstream ss;
        ss << "\rPoints Processed: " << percentage << "% of " << num_points;
        std::cout << ss.str() << std::flush;
      }
    }
  }

  return true;
}

}
