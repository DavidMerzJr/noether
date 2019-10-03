#ifndef BSPLINE_RECONSTRUCTION_H
#define BSPLINE_RECONSTRUCTION_H

#include <functional>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>

namespace bspline_reconstruction
{

struct BSplineReconstructionParams
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // parameters
  int order = 3;
  int refinement = 3;
  unsigned iterations = 3;
  unsigned mesh_resolution = 20;
  bool curve_fit_boundary = true;
  Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0);

  pcl::on_nurbs::FittingSurface::Parameter surface_params;
  pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;

  std::function<void(const pcl::on_nurbs::FittingSurface&, const BSplineReconstructionParams&)> surface_fit_callback;
  std::function<void(const pcl::on_nurbs::FittingCurve2dASDM&, const pcl::on_nurbs::FittingSurface&, const BSplineReconstructionParams&)> curve_fit_callback;

  BSplineReconstructionParams()
  {
    surface_params.interior_smoothness = 1.0;
    surface_params.interior_weight = 1.0;
    surface_params.boundary_smoothness = 1.0;
    surface_params.boundary_weight = 0.0;

    curve_params.addCPsAccuracy = 5e-2;
    curve_params.addCPsIteration = 3;
    curve_params.maxCPs = 100;
    curve_params.accuracy = 1e-3;
    curve_params.iterations = 100;

    curve_params.param.closest_point_resolution = 0;
    curve_params.param.closest_point_weight = 1.0;
    curve_params.param.closest_point_sigma2 = 0.01;
    curve_params.param.interior_sigma2 = 0.00001;
    curve_params.param.smooth_concavity = 1.0;
    curve_params.param.smoothness = 1.0;

    surface_fit_callback = nullptr;
    curve_fit_callback = nullptr;
  }
};

class BSplineReconstruction
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BSplineReconstruction() = default;
  virtual ~BSplineReconstruction() = default;

  /**
   * @brief Get BSpline reconstruction parameters
   * @return reference to parameters.
   */
  BSplineReconstructionParams& getParameters() { return params_; }

  /**
   * @brief Get BSpline reconstruction parameters const
   * @return const reference to parameters.
   */
  const BSplineReconstructionParams& getParameters() const { return params_; }

  /**
   * @brief Set the input cloud to be reconstructed
   * @param input_cloud The input point cloud
   */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud);

  /**
   * @brief Reconstruct the surface. It will initialize the surface usign PCA and Point Cloud bounding box
   * @return If true the surface was successfully reconstructed otherwise false
   */
  bool solve();

  /**
   * @brief Reconstruct the surface using initial_surface as the starting point
   * @param initial_surface Initial surface to start the reconstruction with.
   * @return
   */
  bool solve(ON_NurbsSurface initial_surface);

  /**
   * @brief Get the NURBS model. This will only exist if solve was successful
   * @return NURBS Model
   */
  ONX_Model& getModel() { return model_; }

  /**
   * @brief Get a polygon mesh of the NURBS surface. Only exists if solve was successful
   * @return Polygon Mesh of surface.
   */
  const pcl::PolygonMesh& getMesh() const { return mesh_; }

protected:
  BSplineReconstructionParams params_;
  pcl::on_nurbs::NurbsDataSurface data_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud_;
  ONX_Model model_;
  pcl::PolygonMesh mesh_;
};

};

#endif // BSPLINE_RECONSTRUCTION_H
