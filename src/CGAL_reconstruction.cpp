#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// basic
#include <boost/lexical_cast.hpp>
#include <boost/iterator/function_output_iterator.hpp>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <iostream>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Timer.h>
// outlier
#include <CGAL/remove_outliers.h>
// smoothing
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/compute_average_spacing.h>
// estimate normal
#include <CGAL/jet_estimate_normals.h>
// Simplification
#include <CGAL/Simple_cartesian.h>
#include <CGAL/grid_simplify_point_set.h>
// surface fitting
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Parallel_if_available_tag Concurrency_tag;

typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;

typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef std::pair<Point_3, Vector_3> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;
typedef std::vector<Point_3> Point_container;

typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Mesh>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;

Point_set read_points_from_file(const std::string &fname)
{
  Point_set points;
  std::ifstream stream(fname, std::ios_base::binary);
  if (!stream)
  {
    throw std::runtime_error("Error: cannot read file " + fname);
  }
  // Move the file stream to the 12th line
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
  std::cout << "Read " << points.size() << " point(s)" << std::endl;
  if (points.empty())
  {
    throw std::runtime_error("Error: no points read from file " + fname);
  }
  return points;
}

Point_set outlier_remove(Point_set &points)
{
  typename Point_set::iterator rout_it = CGAL::remove_outliers<CGAL::Sequential_tag>(points,
                                                                                     24,                                          // Number of neighbors considered for evaluation
                                                                                     points.parameters().threshold_percent(2.0)); // Percentage of points to remove
  points.remove(rout_it, points.end());
  std::cout << points.number_of_removed_points()
            << " point(s) are outliers." << std::endl;
  points.collect_garbage();

  return points;
}

Point_set grid_simplify(Point_set &points)
{
  double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 20);
  typename Point_set::iterator gsim_it = CGAL::grid_simplify_point_set(points, 1.2 * spacing);
  points.remove(gsim_it, points.end());
  std::cout << points.number_of_removed_points()
            << " point(s) removed after simplification." << std::endl;

  return points;
}

PointList convert_to_point_list(Point_set &points)
{
  PointList point_list;
  for (const auto &point : points.points())
  {
    point_list.push_back(std::make_pair(point, Vector_3(0, 0, 0))); // Initialize normals to zero
  }
  return point_list;
}

PointList estimate_normal(PointList &points)
{
  CGAL::jet_estimate_normals<Concurrency_tag>(points,
                                              18,
                                              CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
  return points;
}

PointList bilateral_smooth(PointList &points)
{
  int k = 200;                 // size of neighborhood. The bigger the smoother the result will be.
                               // This value should bigger than 1.
  double sharpness_angle = 25; // control sharpness of the result.
                               // The bigger the smoother the result will be
  int iter_number = 5;         // number of times the projection is applied

  for (int i = 0; i < iter_number; ++i)
  {
    CGAL::bilateral_smooth_point_set<Concurrency_tag>(
        points,
        k,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
            .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
            .sharpness_angle(sharpness_angle));
  }
  return points;
}

void generate_mesh(const PointList &point_list, const std::string &output_filename, double relative_alpha, double relative_offset)
{
  Point_container pointwrap;
  for (const auto &point : point_list)
  {
    pointwrap.push_back(point.first);
  }

  Mesh meshwrap;

  CGAL::Bbox_3 bbox = CGAL::bbox_3(std::cbegin(pointwrap), std::cend(pointwrap));
  const double diag_length = std::sqrt(CGAL::square(bbox.xmax() - bbox.xmin()) +
                                       CGAL::square(bbox.ymax() - bbox.ymin()) +
                                       CGAL::square(bbox.zmax() - bbox.zmin()));
  const double alpha = diag_length / relative_alpha;
  const double offset = diag_length / relative_offset;
  CGAL::alpha_wrap_3(pointwrap, alpha, offset, meshwrap);

  CGAL::IO::write_polygon_mesh(output_filename, meshwrap, CGAL::parameters::stream_precision(17));
}

int main(int argc, char *argv[])
{
  CGAL::Timer t;
  t.start();

  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0] << " [input.xyz] [relative_alpha] [relative_offset]" << std::endl;
    return EXIT_FAILURE;
  }

  std::string fname = argv[1];
  const double relative_alpha = std::stod(argv[2]);
  const double relative_offset = std::stod(argv[3]);

  // Read Pointset
  Point_set points = read_points_from_file(fname);

  // Remove outliers
  points = outlier_remove(points);

  // Grid simplify
  points = grid_simplify(points);

  // Convert Point_Set to PointList
  PointList point_list = convert_to_point_list(points);

  // Estimate normals
  point_list = estimate_normal(point_list);

  // Bilateral smoothing
  point_list = bilateral_smooth(point_list);

  // Generate mesh and save to STL
  generate_mesh(point_list, "../mesh/lid/l515/2/lid_wrap.stl", relative_alpha, relative_offset);

  std::cout << "Remeshing done." << std::endl;
  std::cerr << "done in " << t.time() << " sec." << std::endl;
  t.reset();

  return EXIT_SUCCESS;
}
