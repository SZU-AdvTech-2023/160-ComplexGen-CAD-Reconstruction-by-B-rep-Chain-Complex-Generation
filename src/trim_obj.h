#pragma once
#ifndef TRIM_OBJ_H
#define TRIM_OBJ_H

#include "complexgen_object.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

class TrimObj
{
public:
	TrimObj(const std::string name, const toml::table tbl);

	void set_polyhedron(const Polyhedron polyhedron) { m_polyhedron = polyhedron; }
	Polyhedron get_polyhedron() { return m_polyhedron; }
	void set_name(const std::string name) { m_name = name; }
	std::string get_name() { return m_name; }
	void read_original_point_cloud();
	void trim();
	double get_epsilon() { return m_tbl["epsilon"].value<double>().value(); }
private:
	Point_set m_original_point_cloud;
	Polyhedron m_polyhedron;
	std::string m_name;
	toml::table m_tbl;
};

#endif