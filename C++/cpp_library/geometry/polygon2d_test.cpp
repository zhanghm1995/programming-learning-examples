#include "polygon2d.h"
#include <iostream>

using namespace geometry;
using namespace std;

int main()
{
    Polygon2d poly1({{45.140747,0.872014},{45.015877,-0.888845}, {44.665958,-0.864031}, {44.790829,0.896828}});
    Polygon2d poly2({{47.492493,2.692316},{42.517471,2.193155},{43.016632,-2.781867},{47.991653,-2.282706}});

    double area = poly2.ComputeIoU(poly1);

    Polygon2d overlap;
    poly2.ComputeOverlap(poly1, &overlap);

    cout<<area<<endl;

    cout<<overlap.area()<<endl;


    return 0;
}