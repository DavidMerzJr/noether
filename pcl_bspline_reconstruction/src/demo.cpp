#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

#include <mutex>
#include <pcl_bspline_reconstruction/bspline_reconstruction.h>

static std::string pcd_file, file_3dm;
static pcl::PolygonMesh mesh;
static pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
static std::vector<pcl::Vertices> mesh_vertices;
static std::string mesh_id = "mesh_nurbs";
static ON_NurbsCurve curve;
static ON_NurbsSurface surface;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps (new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::visualization::CloudViewer viewer ("B-spline surface fitting");
static std::mutex mutex;

void visualizeClear (pcl::visualization::PCLVisualizer &viewer)
{
  for (std::size_t i = 0; i < curve_cloud->size () - 1; i++)
  {
    pcl::PointXYZRGB &p1 = curve_cloud->at (i);
    pcl::PointXYZRGB &p2 = curve_cloud->at (i + 1);
    std::ostringstream os;
    os << "line" << i;
    viewer.removeShape (os.str ());
    viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1.0, 0.0, 0.0, os.str ());
  }

  viewer.removePointCloud ("cloud_cps");
  viewer.removePolygonMesh (mesh_id);
}

void visualizeCurve (pcl::visualization::PCLVisualizer &viewer)
{
  mutex.lock();

  for (std::size_t i = 0; i < curve_cloud->size () - 1; i++)
  {
    pcl::PointXYZRGB &p1 = curve_cloud->at (i);
    pcl::PointXYZRGB &p2 = curve_cloud->at (i + 1);
    std::ostringstream os;
    os << "line" << i;
    viewer.removeShape (os.str ());
    viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1.0, 0.0, 0.0, os.str ());
  }

  viewer.removePointCloud ("cloud_cps");
  viewer.addPointCloud (curve_cps, "cloud_cps");
  mutex.unlock();
}

void updateCurve(const pcl::on_nurbs::FittingCurve2dASDM& curve_fit, const pcl::on_nurbs::FittingSurface& surface_fit, const bspline_reconstruction::BSplineReconstructionParams& params)
{
  mutex.lock();
  curve_cloud->clear();

  pcl::on_nurbs::Triangulation::convertCurve2PointCloud (curve_fit.m_nurbs, surface_fit.m_nurbs, curve_cloud, 4);

  curve_cps->clear();
  for (int i = 0; i < curve.CVCount (); i++)
  {
    ON_3dPoint p1;
    curve.GetCV (i, p1);

    double pnt[3];
    surface.Evaluate (p1.x, p1.y, 0, 3, pnt);
    pcl::PointXYZRGB p2;
    p2.x = float (pnt[0]);
    p2.y = float (pnt[1]);
    p2.z = float (pnt[2]);

    p2.r = 255;
    p2.g = 0;
    p2.b = 0;

    curve_cps->push_back (p2);
  }
  mutex.unlock();

  viewer.runOnVisualizationThreadOnce (visualizeCurve);
}

void visualizeMesh(pcl::visualization::PCLVisualizer& viewer)
{
  mutex.lock();
  viewer.removePolygonMesh (mesh_id);
  viewer.addPolygonMesh (mesh, mesh_id);
  viewer.spinOnce ();
  mutex.unlock();
}

void updateMesh(const pcl::on_nurbs::FittingSurface& fit, const bspline_reconstruction::BSplineReconstructionParams& params)
{
  mutex.lock();
  pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, params.mesh_resolution);
  mutex.unlock();

  viewer.runOnVisualizationThreadOnce (visualizeMesh);
  sleep(3);
}

int main (int argc, char *argv[])
{
  if (argc < 3)
  {
    printf ("\nUsage: pcl_example_nurbs_fitting_surface pcd<PointXYZ>-in-file 3dm-out-file\n\n");
    exit (0);
  }
  pcd_file = argv[1];
  file_3dm = argv[2];


  // ############################################################################
  // load point cloud

  printf ("  loading %s\n", pcd_file.c_str ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud2;
  pcl::on_nurbs::NurbsDataSurface data;

  if (pcl::io::loadPCDFile (pcd_file, cloud2) == -1)
    throw std::runtime_error ("  PCD file not found.");

  fromPCLPointCloud2 (cloud2, *cloud);
  viewer.showCloud(cloud, "cloud_cylinder");
  printf ("  %lu points in data set\n", cloud->size ());


  // ############################################################################
  // fit B-spline surface
  bool user_defined = true;
  if (!user_defined)
  {
    bspline_reconstruction::BSplineReconstruction surf_recon;

    auto& params = surf_recon.getParameters();
    params.surface_fit_callback = updateMesh;
    params.curve_fit_callback = updateCurve;
    params.z = Eigen::Vector3d(0, 0, 1);

    surf_recon.setInputCloud(cloud);

    if (surf_recon.solve())
    {
      printf ("Successfully Reconstructed Surface!\n");
      surf_recon.getModel().Write(file_3dm.c_str());
      printf("  model saved: %s\n", file_3dm.c_str());

      mesh = surf_recon.getMesh();
      viewer.runOnVisualizationThreadOnce (visualizeMesh);
    }
  }
  else
  {
    bspline_reconstruction::BSplineReconstruction surf_recon;

    auto& params = surf_recon.getParameters();
    params.surface_fit_callback = updateMesh;
    params.curve_fit_callback = updateCurve;
    params.refinement = 3;
    params.iterations = 1;
    params.mesh_resolution = 10;
    params.curve_params.iterations = 20;
    surf_recon.setInputCloud(cloud);

    ON_3dPoint ll, lr, ur, ul;
    double y = -0.5;
    double x = 1.0;
    double z = 0.5;
    ll.Set(-x, y, -z);
    lr.Set(x, y, -z);
    ur.Set(x, y, z);
    ul.Set(-x, y, z);
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbs4Corners(params.order, ll, lr, ur, ul);

    if (surf_recon.solve(nurbs))
    {
      printf ("Successfully Reconstructed Surface!\n");
      surf_recon.getModel().Write(file_3dm.c_str());
      printf("  model saved: %s\n", file_3dm.c_str());

      mesh = surf_recon.getMesh();
      viewer.runOnVisualizationThreadOnce (visualizeMesh);
    }
  }

  while (!viewer.wasStopped ()) {}

  return 0;
}
