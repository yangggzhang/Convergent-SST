#include "utilities/convexhull.hpp"

double ConvexHull::facet_area(Facet_iterator &f)
{
      double area = K::Compute_area_3()(
      f->halfedge()->vertex()->point(),
      f->halfedge()->next()->vertex()->point(),
      f->halfedge()->opposite()->vertex()->point() );
      return area;
}

Vector_3 ConvexHull::facet_normal(Facet_iterator &f)
{
  Vector_3 n;
  HalfedgeHandle h  = f->halfedge();
  Point_3              p1 = h->vertex()->point();
  Point_3              p2 = h->next()->vertex()->point();
  Point_3              p3 = h->next()->next()->vertex()->point();
  n  = CGAL::cross_product(p2-p1, p3-p1);
  n = n / std::sqrt(n*n);
  return n;
}

double ConvexHull::dot_product(Point_3 &p, Vector_3 &facet_normal){
  double ans;
  ans = p.x() * facet_normal.x() + p.y() * facet_normal.y() + p.z() * facet_normal.z();
  return ans;
}

 double ConvexHull::ConvexHull_Volume(std::vector<double*> &raw_points)
 {
    double volume;
    Polyhedron_3 poly;
    std::vector<Point_3> points;
    points.resize(raw_points.size());
    for (size_t i = 0; i < raw_points.size();i++)
    {
      points.push_back(Point_3(raw_points[i][0],raw_points[i][1],raw_points[i][2]));
    }
    // compute convex hull of non-collinear points
    CGAL::convex_hull_3(points.begin(), points.end(), poly);
    //std::cout << "The convex hull contains " << poly.size_of_vertices() << " vertices" << std::endl;

    for (Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); f++)
    {
        Vector_3 n = facet_normal(f); 
        double area = facet_area(f);
        Point_3 local_point =  f->halfedge()->vertex()->point();
        volume = volume + dot_product(local_point,n)*area;
    }
    volume = 1.0/3.0*abs(volume);
    return volume;
 }
