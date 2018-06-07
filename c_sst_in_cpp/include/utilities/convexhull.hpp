#ifndef CONVEXHULL_HPP
#define CONVEXHULL_HPP

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <vector>
#include <fstream>
typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Point_3                                Point_3;
typedef K::Vector_3                               Vector_3;
typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;
typedef Polyhedron_3::Facet_iterator              Facet_iterator;
typedef Polyhedron_3::Halfedge_handle       HalfedgeHandle;
typedef Polyhedron_3::Halfedge_around_facet_const_circulator HalfEdgeConstCirculator;

class ConvexHull
{
public:
    
    double ConvexHull_Volume(std::vector<double*> &raw_points); 
    
private:
    
    double facet_area(Facet_iterator &f);

    Vector_3 facet_normal(Facet_iterator &f);

    double dot_product(Point_3 &p, Vector_3 &facet_normal);
};

#endif // MAPPING_OCCUPANCY_GRID_HPP
