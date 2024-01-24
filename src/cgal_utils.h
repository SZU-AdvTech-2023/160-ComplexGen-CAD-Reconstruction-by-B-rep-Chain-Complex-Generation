#pragma once
#ifndef MY_CGAL_UTILS_H
#define MY_CGAL_UTILS_H

#include <iostream>
#include <fstream>
#include <cassert>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/IO/read_ply_points.h>

#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Periodic_2_Delaunay_triangulation_traits_2.h>
#include <CGAL/Periodic_2_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>


#include <CGAL/algorithm.h>

//#include "patch.h"


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef CGAL::IO::Color Color;
typedef Kernel::Triangle_2 Triangle_2;
typedef Kernel::Triangle_3 Triangle_3;
typedef CGAL::Point_set_3<Point_3> Point_set;
typedef Point_set::Property_map<Color> Color_map;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;

typedef CGAL::Alpha_shape_vertex_base_2<Kernel>                   Vb;
typedef CGAL::Alpha_shape_face_base_2<Kernel>                     Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>          Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;
typedef Alpha_shape_2::Alpha_shape_edges_iterator            Alpha_shape_edges_iterator;

//// Traits
//typedef CGAL::Periodic_2_Delaunay_triangulation_traits_2<Kernel>     Gt;
//// Vertex type
//typedef CGAL::Periodic_2_triangulation_vertex_base_2<Gt>        Vb;
//typedef CGAL::Alpha_shape_vertex_base_2<Gt, Vb>                 AsVb;
//// Cell type
//typedef CGAL::Periodic_2_triangulation_face_base_2<Gt>          Cb;
//typedef CGAL::Alpha_shape_face_base_2<Gt, Cb>                   AsCb;
//typedef CGAL::Triangulation_data_structure_2<AsVb, AsCb>        Tds;
//typedef CGAL::Periodic_2_Delaunay_triangulation_2<Gt, Tds>      P2DT2;
//typedef CGAL::Alpha_shape_2<P2DT2>                              Alpha_shape_2;
//typedef Gt::Point_2                                             Point;
//typedef Gt::Segment_2                                           Segment;
//typedef Alpha_shape_2::Alpha_shape_edges_iterator               Alpha_shape_edges_iterator;

typedef CGAL::Exact_predicates_tag                                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, CGAL::Default, Itag>    CDT;

//occ
#include <gp_Pnt.hxx>

bool save_gp_points(std::vector<gp_Pnt>& points, std::string file_name);

bool save_triangles_3(std::vector<Triangle_3>& triangles, std::string file_name);

void CDTriangulation(CDT& cdt, std::map<Point_2, CDT::Vertex_handle>& p_index, Alpha_shape_2& as);

#endif
