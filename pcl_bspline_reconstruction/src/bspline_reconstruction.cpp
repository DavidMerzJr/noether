#include <pcl_bspline_reconstruction/bspline_reconstruction.h>
#include <pcl/surface/on_nurbs/triangulation.h>

namespace bspline_reconstruction
{

void PointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::on_nurbs::vector_vec3d &data)
{
  for (unsigned i = 0; i < cloud->size (); i++)
  {
    const pcl::PointXYZ &p = cloud->at (i);
    if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z))
      data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
  }
}

void BSplineReconstruction::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud)
{
  input_cloud_ = input_cloud;
  data_ = pcl::on_nurbs::NurbsDataSurface();
  PointCloud2Vector3d (input_cloud_, data_.interior);
}

bool BSplineReconstruction::solve()
{
  ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (params_.order, &data_, params_.z);
  return solve(nurbs);
}

bool BSplineReconstruction::solve(ON_NurbsSurface initial_surface)
{
  // initialize
  printf ("  surface fitting ...\n");
  pcl::on_nurbs::FittingSurface fit (&data_, initial_surface);
  fit.setQuiet (false); // enable/disable debug output

  if (params_.surface_fit_callback != nullptr)
    params_.surface_fit_callback(fit, params_);

  // surface refinement
  for (int i = 0; i < params_.refinement; i++)
  {
    fit.refine (0);
    fit.refine (1);
    fit.assemble (params_.surface_params);
    fit.solve ();
    if (params_.surface_fit_callback != nullptr)
      params_.surface_fit_callback(fit, params_);
  }

  // surface fitting with final refinement level
  for (unsigned i = 0; i < params_.iterations; i++)
  {
    fit.assemble (params_.surface_params);
    fit.solve ();
    if (params_.surface_fit_callback != nullptr)
      params_.surface_fit_callback(fit, params_);
  }

  // ############################################################################
  // fit B-spline curve
  std::shared_ptr<pcl::on_nurbs::FittingCurve2dASDM> curve_fit;
  if (params_.curve_fit_boundary)
  {
    // initialisation (circular)
    printf ("  curve fitting ...\n");
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = data_.interior_param;
    curve_data.interior_weight_function.push_back (true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (params_.order, curve_data.interior);

    // curve fitting
    curve_fit = std::make_shared<pcl::on_nurbs::FittingCurve2dASDM>(&curve_data, curve_nurbs);
    curve_fit->setQuiet (false); // enable/disable debug output
    curve_fit->fitting (params_.curve_params);
    if (params_.curve_fit_callback != nullptr)
      params_.curve_fit_callback(*curve_fit, fit, params_);

    printf ("  triangulate trimmed surface ...\n");
  }

  // save trimmed B-spline surface
  if ( fit.m_nurbs.IsValid() )
  {
    model_.Destroy();
    ONX_Model_Object& surf = model_.m_object_table.AppendNew();
    surf.m_object = new ON_NurbsSurface(fit.m_nurbs);
    surf.m_bDeleteObject = true;
    surf.m_attributes.m_layer_index = 1;
    surf.m_attributes.m_name = "surface";

    if (params_.curve_fit_boundary)
    {
      ONX_Model_Object& curv = model_.m_object_table.AppendNew();
      curv.m_object = new ON_NurbsCurve(curve_fit->m_nurbs);
      curv.m_bDeleteObject = true;
      curv.m_attributes.m_layer_index = 2;
      curv.m_attributes.m_name = "trimming curve";

      pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (fit.m_nurbs, curve_fit->m_nurbs, mesh_,
                                                                       params_.mesh_resolution);
    }
    else
    {
      pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh_, params_.mesh_resolution);
    }
  }

  return fit.m_nurbs.IsValid();
}

}
