#include "Meshing.h"

namespace CGAL {
    Meshing::Point_set Meshing::read_points_from_file(const std::string &fname)
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

    Meshing::Point_set Meshing::outlier_remove(Point_set &points)
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

    Meshing::Point_set Meshing::grid_simplify(Point_set &points)
    {
        double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 20);
        typename Point_set::iterator gsim_it = CGAL::grid_simplify_point_set(points, 1.2 * spacing);
        points.remove(gsim_it, points.end());
        std::cout << points.number_of_removed_points()
                  << " point(s) removed after simplification." << std::endl;

        return points;
    }

    Meshing::PointList Meshing::convert_to_point_list(Point_set &points)
    {
        PointList point_list;
        for (const auto &point : points.points())
        {
            point_list.push_back(std::make_pair(point, Vector_3(0, 0, 0))); // Initialize normals to zero
        }
        return point_list;
    }

    Meshing::PointList Meshing::estimate_normal(PointList &points)
    {
        CGAL::jet_estimate_normals<Concurrency_tag>(points,
                                                    18,
                                                    CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
        return points;
    }

    Meshing::PointList Meshing::bilateral_smooth(PointList &points)
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

    void Meshing::generate_mesh(const PointList &point_list, const std::string &output_filename, double relative_alpha, double relative_offset)
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
}