#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include "convexhull.hpp"
#include <vector>
#include <fstream>
typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Point_3                                Point_3;

int main(int argc, char* argv[])
{
    // 57    34    32    61    69     9    16   100
    // 47    17    53    27    75    23    83     8
    //  2    80    17    66    46    92    54    45


  std::vector<Point_3> points;
  Point_3 p1(57,47,2);
  Point_3 p2(34,17,80);
  Point_3 p3(32,53,17);
  Point_3 p4(61,27,66);
  Point_3 p5(69,75,46);
  Point_3 p6(9,23,92);
  Point_3 p7(16,83,54);
  Point_3 p8(100,8,45);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);
  points.push_back(p6);
  points.push_back(p7);
  points.push_back(p8);
  // define polyhedron to hold convex hull
  ConvexHull h;
  double volume = h.ConvexHull_Volume(points);
  std::cout<<"Volume : "<<volume<<std::endl;
  return 0;
}