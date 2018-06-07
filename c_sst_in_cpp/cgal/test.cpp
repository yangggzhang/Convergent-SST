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

double facet_area(Facet_iterator &f)
{
  double area = K::Compute_area_3()(
      f->halfedge()->vertex()->point(),
      f->halfedge()->next()->vertex()->point(),
      f->halfedge()->opposite()->vertex()->point() );
  return area;
}

Vector_3 facet_normal(Facet_iterator &f)
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

double dot_product(Point_3 &p, Vector_3 &facet_normal){
  double ans;
  ans = p.x() * facet_normal.x() + p.y() * facet_normal.y() + p.z() * facet_normal.z();
  return ans;
}
       

int main(int argc, char* argv[])
{
  std::vector<Point_3> points;
  Point_3 p1(0,0,0);
  Point_3 p2(0,1,0);
  Point_3 p3(1,1,0);
  Point_3 p4(1,0,0);
  Point_3 p5(0,0,1);
  Point_3 p6(0,1,1);
  Point_3 p7(1,1,1);
  Point_3 p8(1,0,1);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);
  points.push_back(p6);
  points.push_back(p7);
  points.push_back(p8);
  // define polyhedron to hold convex hull
  Polyhedron_3 poly;
  
  // compute convex hull of non-collinear points
  CGAL::convex_hull_3(points.begin(), points.end(), poly);
  std::cout << "The convex hull contains " << poly.size_of_vertices() << " vertices" << std::endl;
  double volume = 0;
  for (Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); f++)
  {
     Vector_3 n = facet_normal(f); 
     double area = facet_area(f);
     Point_3 local_point =  f->halfedge()->vertex()->point();
     volume = volume + dot_product(local_point,n)*area;
   }
   volume = 1.0/3.0*abs(volume);
   std::cout<<"Volume : "<<volume<<std::endl;
  return 0;
}