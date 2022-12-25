//
// Created by Dominik Krupke on 11.12.22.
// This file provides some simple CGAL definitions to ease the use of CGAL.
//

#ifndef SAMPLNS_CGAL_KERNEL_H
#define SAMPLNS_CGAL_KERNEL_H
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Segment_2.h>
namespace cetsp {
namespace details {
using Kernel = CGAL::Cartesian<double>;
using Point = CGAL::Point_2<Kernel>;
using Segment = CGAL::Segment_2<Kernel>;
using Polygon = CGAL::Polygon_2<Kernel>;
} // namespace details
} // namespace cetsp
#endif // SAMPLNS_CGAL_KERNEL_H
