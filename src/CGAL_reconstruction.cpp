#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <boost/lexical_cast.hpp>
#include <boost/iterator/function_output_iterator.hpp>
#include <cstdlib>
#include <vector>
#include <fstream>
// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Parallel_if_available_tag Concurrency_tag;
typedef CGAL::Scale_space_surface_reconstruction_3<Kernel> Reconstruction;
typedef CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel> Mesher;
typedef CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel> Smoother;

typedef Reconstruction::Facet_const_iterator Facet_iterator;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Sphere_3 Sphere_3;
typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef std::pair<Point_3, Vector_3> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;

typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Mesh>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;

struct halfedge2edge
{
  halfedge2edge(const Mesh &m, std::vector<edge_descriptor> &edges)
      : m_mesh(m), m_edges(edges)
  {
  }
  void operator()(const halfedge_descriptor &h) const
  {
    m_edges.push_back(edge(h, m_mesh));
  }
  const Mesh &m_mesh;
  std::vector<edge_descriptor> &m_edges;
};


int main(int argc, char *argv[])
{
  CGAL::Timer t;
  t.start();
  ///////////////Read Pointset///////////////
  Point_set points;
  std::string fname = argc == 1 ? CGAL::data_file_path("points_3/kitten.xyz") : argv[1];
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " [input.xyz/off/ply/las]" << std::endl;
    std::cerr << "Running " << argv[0] << " data/kitten.xyz -1\n";
  }
  std::ifstream stream(fname, std::ios_base::binary);
  if (!stream)
  {
    std::cerr << "Error: cannot read file " << fname << std::endl;
    return EXIT_FAILURE;
  }
  // Move the file stream to the 12nd line
  for (int i = 0; i < 12; ++i)
  {
    std::string dummy;
    std::getline(stream, dummy);
  }
  Point_3 point;
  while (stream >> point)
  {
    points.insert(point);
  }

  // stream >> points;
  std::cout << "Read " << points.size() << " point(s)" << std::endl;
  if (points.empty())
    return EXIT_FAILURE;

  ///////////////outlier_remove///////////////
  typename Point_set::iterator rout_it = CGAL::remove_outliers<CGAL::Sequential_tag>(points,
                                                                                     24,                                          // Number of neighbors considered for evaluation
                                                                                     points.parameters().threshold_percent(5.0)); // Percentage of points to remove
  points.remove(rout_it, points.end());
  std::cout << points.number_of_removed_points()
            << " point(s) are outliers." << std::endl;
  // Applying point set processing algorithm to a CGAL::Point_set_3
  // object does not erase the points from memory but place them in
  // the garbage of the object: memory can be freed by the user.
  points.collect_garbage();
  if (!CGAL::IO::write_XYZ("../mesh/fenda/2/fenda_outlier.xyz", points))
    return EXIT_FAILURE;

  ///////////////grid_simplify///////////////

  // Compute average spacing using neighborhood of 6 points
  double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 6);
  // Simplify using a grid of size 2 * average spacing
  typename Point_set::iterator gsim_it = CGAL::grid_simplify_point_set(points, 2. * spacing);
  points.remove(gsim_it, points.end());
  std::cout << points.number_of_removed_points()
            << " point(s) removed after simplification." << std::endl;
  points.collect_garbage();

  if (!CGAL::IO::write_XYZ("../mesh/fenda/2/fenda_outlier_grid.xyz", points))
    return EXIT_FAILURE;

  ///////////////surface fitting///////////////
  /////////////////// Advancing front///////////////////

  /////////////////// Estimates normals/////////////////
  PointList point_list;
  if (!CGAL::IO::read_points("../mesh/fenda/2/fenda_outlier_grid.xyz", std::back_inserter(point_list),
                             CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())))
  {
    std::cerr << "Error: cannot read file "
              << "../mesh/fenda/2/fenda_outlier_grid.xyz" << std::endl;
    return EXIT_FAILURE;
  }

  CGAL::jet_estimate_normals<Concurrency_tag>(point_list,
                                              18,
                                              CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
  std::cerr << "Write file "
            << "../mesh/fenda/2/fenda_outlier_grid_normal.xyz" << std::endl
            << std::endl;

  if (!CGAL::IO::write_points("../mesh/fenda/2/fenda_outlier_grid_normal.xyz", point_list,
                              CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
                                  .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
                                  .stream_precision(17)))
  {
    std::cerr << "Error: cannot write file "
              << "../mesh/fenda/2/fenda_outlier_grid_normal.xyz" << std::endl;
    return EXIT_FAILURE;
  }

  /////////////////bilateral_smoothing/////////////////

  std::vector<PointVectorPair> points_smooth;
  if (!CGAL::IO::read_points("../mesh/fenda/2/fenda_outlier_grid_normal.xyz", std::back_inserter(points_smooth),
                             CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
                                 .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())))
  {
    std::cerr << "Error: cannot read file "
              << "../mesh/fenda/2/fenda_outlier_grid_normal.xyz" << std::endl;
    return EXIT_FAILURE;
  }

  // Algorithm parameters
  int k = 200;                 // size of neighborhood. The bigger the smoother the result will be.
                               // This value should bigger than 1.
  double sharpness_angle = 25; // control sharpness of the result.
                               // The bigger the smoother the result will be
  int iter_number = 5;         // number of times the projection is applied

  for (int i = 0; i < iter_number; ++i)
  {
    /* double error = */
    CGAL::bilateral_smooth_point_set<Concurrency_tag>(
        points_smooth,
        k,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
            .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
            .sharpness_angle(sharpness_angle));
  }

  if (!CGAL::IO::write_XYZ("../mesh/fenda/2/fenda_outlier_grid_normal_smooth.xyz", points_smooth,
                           CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
                               .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
                               .stream_precision(17)))
    return EXIT_FAILURE;

  /////////////////reconstruction/////////////////

  Point_set points_final;
  if (!CGAL::IO::read_point_set("../mesh/fenda/2/fenda_outlier_grid_normal_smooth.xyz", points_final))
  {
    std::cerr << "Error: cannot read file" << std::endl;
    return EXIT_FAILURE;
  }
  std::cerr << "done: " << points_final.size() << " points." << std::endl;

  std::cerr << "Reconstruction ";

  // Construct the mesh in a scale space.
  Reconstruction reconstruct(points_final.points().begin(), points_final.points().end());
  reconstruct.increase_scale<Smoother>(4);
  reconstruct.reconstruct_surface(Mesher(0.5));

  std::ofstream out("../mesh/fenda/2/out_advancingfont.off");
  out << "OFF" << std::endl
      << points_final.size() << " " << reconstruct.number_of_facets() << " 0" << std::endl;

  for (Point_set::iterator it = points_final.begin(); it != points_final.end(); ++it)
    out << points_final.point(*it) << std::endl;

  for (Reconstruction::Facet_iterator it = reconstruct.facets_begin();
       it != reconstruct.facets_end(); ++it)
    out << "3 " << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << std::endl;

  ///////////////isotropic remeshing///////////////

  Mesh mesh;
  if (!PMP::IO::read_polygon_mesh("../mesh/fenda/2/out_advancingfont.off", mesh) || !CGAL::is_triangle_mesh(mesh))
  {
    std::cerr << "Invalid input." << std::endl;
    return 1;
  }

  double target_edge_length = (argc > 2) ? std::stod(std::string(argv[2])) : 0.04;
  unsigned int nb_iter = (argc > 3) ? std::stoi(std::string(argv[3])) : 10;
  std::cout << "Split border...";

  std::vector<edge_descriptor> border;
  PMP::border_halfedges(faces(mesh), mesh, boost::make_function_output_iterator(halfedge2edge(mesh, border)));
  PMP::split_long_edges(border, target_edge_length, mesh);

  std::cout << "done." << std::endl;
  std::cout << "Start remeshing of "
            << "fenda"
            << " (" << num_faces(mesh) << " faces)..." << std::endl;

  PMP::isotropic_remeshing(faces(mesh), target_edge_length, mesh,
                           CGAL::parameters::number_of_iterations(nb_iter)
                               .protect_constraints(true)); // i.e. protect border, here

  CGAL::IO::write_polygon_mesh("../mesh/fenda/2/final.off", mesh, CGAL::parameters::stream_precision(17));

  std::cout << "Remeshing done." << std::endl;
  std::cerr << "done in " << t.time() << " sec." << std::endl;
  t.reset();

  return EXIT_SUCCESS;
}