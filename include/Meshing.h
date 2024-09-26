#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Surface_mesh.h>
#include <vector>
#include <string>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/alpha_wrap_3.h>
#include <fstream>
#include <iostream>

namespace CGAL
{

    class Meshing
    {
    public:
        // types
        typedef Exact_predicates_inexact_constructions_kernel Kernel;
        typedef Parallel_if_available_tag Concurrency_tag;

        typedef Kernel::Point_3 Point_3;
        typedef Kernel::Vector_3 Vector_3;

        typedef Point_set_3<Point_3, Vector_3> Point_set;
        typedef Surface_mesh<Point_3> Mesh;
        typedef std::pair<Point_3, Vector_3> PointVectorPair;
        typedef std::vector<PointVectorPair> PointList;
        typedef std::vector<Point_3> Point_container;

        Point_set read_points_from_file(const std::string &fname);
        Point_set outlier_remove(Point_set &points);
        Point_set grid_simplify(Point_set &points);
        PointList convert_to_point_list(Point_set &points);
        PointList estimate_normal(PointList &points);
        PointList bilateral_smooth(PointList &points);
        void generate_mesh(const PointList &point_list, const std::string &output_filename, double relative_alpha, double relative_offset);
    };

}