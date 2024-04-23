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
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Sphere_3 Sphere_3;
typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;

typedef std::pair<Point_3, Vector_3> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;

// mode explanation
// 0: Poissson: outlier_remove->grid_simplify->pca_normal_estimation->bilateral smoothing
// 1: Advancing Font: outlier_remove->grid_simplify->jet smoothing
// 2: Scale space: outlier_remove->grid_simplify->jet smoothing
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
  stream >> points;
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

  ///////////////grid_simplify///////////////
  // Compute average spacing using neighborhood of 6 points
  double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 6);
  // Simplify using a grid of size 2 * average spacing
  typename Point_set::iterator gsim_it = CGAL::grid_simplify_point_set(points, 2. * spacing);
  points.remove(gsim_it, points.end());
  std::cout << points.number_of_removed_points()
            << " point(s) removed after simplification." << std::endl;
  points.collect_garbage();

  if (!CGAL::IO::write_XYZ("../mesh/plate/plate_outlier_grid.xyz", points))
    return EXIT_FAILURE;

  ///////////////surface fitting///////////////
  int reconstruction_choice = argc == 1 ? -1 : (argc < 3 ? 0 : atoi(argv[2]));
  if (reconstruction_choice == 0 || reconstruction_choice == -1) //////////////////////////////////////////////////////////////////////// Poisson
  {

    // Estimates normals direction.
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(points, 24); // Use 24 neighbors
    // Orientation of normals, returns iterator to first unoriented point
    typename Point_set::iterator unoriented_points_begin =
        CGAL::mst_orient_normals(points, 24); // Use 24 neighbors
    points.remove(unoriented_points_begin, points.end());
    CGAL::Surface_mesh<Point_3> output_mesh;
    CGAL::poisson_surface_reconstruction_delaunay(points.begin(), points.end(),
                                                  points.point_map(), points.normal_map(),
                                                  output_mesh, spacing);
    std::ofstream f("../mesh/plate/out_poisson.ply", std::ios_base::binary);
    CGAL::IO::set_binary_mode(f);
    CGAL::IO::write_PLY(f, output_mesh);
    f.close();
  }

  if (reconstruction_choice == 1 || reconstruction_choice == -1) //////////////////////////////////////////////////////////////////Advancing front
  {

    /////////////////// Estimates normals/////////////////
    PointList point_list;
    if (!CGAL::IO::read_points("../mesh/plate/plate_outlier_grid.xyz", std::back_inserter(point_list),
                               CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())))
    {
      std::cerr << "Error: cannot read file "
                << "../mesh/plate/plate_outlier_grid.xyz" << std::endl;
      return EXIT_FAILURE;
    }

    CGAL::jet_estimate_normals<Concurrency_tag>(point_list,
                                                18,
                                                CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
    std::cerr << "Write file "
              << "../mesh/plate/plate_outlier_grid_normal.xyz" << std::endl
              << std::endl;

    if (!CGAL::IO::write_points("../mesh/plate/plate_outlier_grid_normal.xyz", point_list,
                                CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
                                    .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
                                    .stream_precision(17)))
    {
      std::cerr << "Error: cannot write file "
                << "../mesh/plate/plate_outlier_grid_normal.xyz" << std::endl;
      return EXIT_FAILURE;
    }

    /////////////////bilateral_smoothing/////////////////

    std::vector<PointVectorPair> points_smooth;
    if (!CGAL::IO::read_points("../mesh/plate/plate_outlier_grid_normal.xyz", std::back_inserter(points_smooth),
                               CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
                                   .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())))
    {
      std::cerr << "Error: cannot read file "
                << "../mesh/plate/plate_outlier_grid_normal.xyz" << std::endl;
      return EXIT_FAILURE;
    }

    // Algorithm parameters
    int k = 120;                 // size of neighborhood. The bigger the smoother the result will be.
                                 // This value should bigger than 1.
    double sharpness_angle = 25; // control sharpness of the result.
                                 // The bigger the smoother the result will be
    int iter_number = 3;         // number of times the projection is applied

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

    if (!CGAL::IO::write_XYZ("../mesh/plate/plate_outlier_grid_normal_smooth.xyz", points_smooth,
                             CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
                                 .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
                                 .stream_precision(17)))
      return EXIT_FAILURE;

    /////////////////reconstruction/////////////////

    Point_set points_final;
    if (!CGAL::IO::read_point_set("../mesh/plate/plate_outlier_grid_normal_smooth.xyz", points_final))
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

    std::ofstream out("../mesh/plate/out_advancingfont.off");
    out << "OFF" << std::endl
        << points_final.size() << " " << reconstruct.number_of_facets() << " 0" << std::endl;

    for (Point_set::iterator it = points_final.begin(); it != points_final.end(); ++it)
      out << points_final.point(*it) << std::endl;

    for (Reconstruction::Facet_iterator it = reconstruct.facets_begin();
         it != reconstruct.facets_end(); ++it)
      out << "3 " << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << std::endl;

    std::cerr << "Writing result in " << t.time() << " sec." << std::endl;

    std::cerr << "Done." << std::endl;
    std::cerr << "done in " << t.time() << " sec." << std::endl;

    t.reset();
  }

  if (reconstruction_choice == 2 || reconstruction_choice == -1) ////////////////////////////////////////////////////////////////////////// Scale space
  {

    ///////////////jet_smoothing///////////////
    CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(points, 24);
    CGAL::Scale_space_surface_reconstruction_3<Kernel> reconstruct(points.points().begin(), points.points().end());
    // Smooth using 4 iterations of Jet Smoothing
    reconstruct.increase_scale(4, CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel>());
    // Mesh with the Advancing Front mesher with a maximum facet length of 0.5
    reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>(0.5));
    std::ofstream f("../pcd/registrated_pcd/plate/out_scalespace.off");
    f << "OFF" << std::endl
      << points.size() << " "
      << reconstruct.number_of_facets() << " 0" << std::endl;
    for (Point_set::Index idx : points)
      f << points.point(idx) << std::endl;
    for (const auto &facet : CGAL::make_range(reconstruct.facets_begin(), reconstruct.facets_end()))
      f << "3 " << facet[0] << " " << facet[1] << " " << facet[2] << std::endl;
    f.close();
  }
  return EXIT_SUCCESS;
}