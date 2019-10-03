#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <mutex>

#include <robust_edge_extraction/robust_edge_extraction.h>

static std::string pcd_file, output_file;
static pcl::PolygonMesh mesh;
static pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
static std::vector<pcl::Vertices> mesh_vertices;
static std::string edge_id = "edge_cloud";
//static pcl::visualization::CloudViewer viewer ("robust_edge_extraction");
//static pcl::visualization::PCLVisualizer pcl_viewer ("robust_edge_extraction");
static std::mutex mutex;

//void visualizeClear (pcl::visualization::PCLVisualizer &viewer)
//{
//  for (std::size_t i = 0; i < curve_cloud->size () - 1; i++)
//  {
//    pcl::PointXYZRGB &p1 = curve_cloud->at (i);
//    pcl::PointXYZRGB &p2 = curve_cloud->at (i + 1);
//    std::ostringstream os;
//    os << "line" << i;
//    viewer.removeShape (os.str ());
//    viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1.0, 0.0, 0.0, os.str ());
//  }

//  viewer.removePointCloud ("cloud_cps");
//  viewer.removePolygonMesh (mesh_id);
//}

//void visualizeCurve (pcl::visualization::PCLVisualizer &viewer)
//{
//  mutex.lock();

//  for (std::size_t i = 0; i < curve_cloud->size () - 1; i++)
//  {
//    pcl::PointXYZRGB &p1 = curve_cloud->at (i);
//    pcl::PointXYZRGB &p2 = curve_cloud->at (i + 1);
//    std::ostringstream os;
//    os << "line" << i;
//    viewer.removeShape (os.str ());
//    viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1.0, 0.0, 0.0, os.str ());
//  }

//  viewer.removePointCloud ("cloud_cps");
//  viewer.addPointCloud (curve_cps, "cloud_cps");
//  mutex.unlock();
//}

//void updateCurve(const pcl::on_nurbs::FittingCurve2dASDM& curve_fit, const pcl::on_nurbs::FittingSurface& surface_fit, const bspline_reconstruction::BSplineReconstructionParams& params)
//{
//  mutex.lock();
//  curve_cloud->clear();

//  pcl::on_nurbs::Triangulation::convertCurve2PointCloud (curve_fit.m_nurbs, surface_fit.m_nurbs, curve_cloud, 4);

//  curve_cps->clear();
//  for (int i = 0; i < curve.CVCount (); i++)
//  {
//    ON_3dPoint p1;
//    curve.GetCV (i, p1);

//    double pnt[3];
//    surface.Evaluate (p1.x, p1.y, 0, 3, pnt);
//    pcl::PointXYZRGB p2;
//    p2.x = float (pnt[0]);
//    p2.y = float (pnt[1]);
//    p2.z = float (pnt[2]);

//    p2.r = 255;
//    p2.g = 0;
//    p2.b = 0;

//    curve_cps->push_back (p2);
//  }
//  mutex.unlock();

//  viewer.runOnVisualizationThreadOnce (visualizeCurve);
//}

//void visualizeMesh(pcl::visualization::PCLVisualizer& viewer)
//{
//  mutex.lock();
//  viewer.removePolygonMesh (mesh_id);
//  viewer.addPolygonMesh (mesh, mesh_id);
//  viewer.spinOnce ();
//  mutex.unlock();
//}

//void updateMesh(const pcl::on_nurbs::FittingSurface& fit, const bspline_reconstruction::BSplineReconstructionParams& params)
//{
//  mutex.lock();
//  pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, params.mesh_resolution);
//  mutex.unlock();

//  viewer.runOnVisualizationThreadOnce (visualizeMesh);
//  sleep(3);
//}

int main (int argc, char *argv[])
{
  if (argc < 3)
  {
    printf ("\nUsage: robust_edge_extraction pcd<PointXYZ>-in-file 3dm-out-file\n\n");
    exit (0);
  }

  pcd_file = argv[1];
  output_file = argv[2];

  printf ("  loading %s\n", pcd_file.c_str ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud2;

  if (pcl::io::loadPCDFile (pcd_file, cloud2) == -1)
    throw std::runtime_error ("  PCD file not found.");

  fromPCLPointCloud2 (cloud2, *input_cloud);

  double search_radius = 0.03;
  double resolution = 0.001;

  robust_edge_extraction::RobustEdgeExtraction edge_extraction;
  edge_extraction.setInputCloud(input_cloud, resolution);
  edge_extraction.solve(search_radius);

  auto edge_data = edge_extraction.getEdgeResults();
  pcl::io::savePCDFile("/home/larmstrong/test_pcd.pcd", *edge_data);

//  pcl_viewer.addPointCloud<pcl::PointXYZHSV>(edge_data, edge_id);
//  pcl_viewer.setBackgroundColor (0, 0, 0);
//  pcl_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, edge_id);
//  pcl_viewer.addCoordinateSystem (1.0);
//  pcl_viewer.initCameraParameters ();

//  printf ("... Done!\n");

//  while (!pcl_viewer.wasStopped ())
//  {
//    pcl_viewer.spinOnce (100);
//  }

  return 0;
}
