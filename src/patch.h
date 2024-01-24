#pragma once
#ifndef PATCH_H
#define PATCH_H

#include <vector>
#include <chrono>
#include <json/json.h>

#include "cgal_utils.h"
#include <toml++/toml.h>

#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <gp_Cone.hxx>
#include <gp_Pln.hxx>
#include <Geom_BSplineSurface.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Extrema_ExtPS.hxx>
#include <ElSLib.hxx>

// #include <ELCLib.hxx>
// #include <Geom_CylindricalSurface.hxx>

class Patch
{
private:
	inline static int next_id = 0;
	int m_id;
	std::vector<gp_Pnt> m_grid_points;
	std::vector<gp_Pnt> m_inner_original_points;
	std::vector<gp_Pnt> m_inner_unique_original_points;
	std::vector<gp_Pnt2d> m_uv_points;
	std::vector<gp_Pnt2d> m_unique_uv_points;
	std::vector<Triangle_3> m_alpha_shape_triangles;
	std::vector<Triangle_3> m_unique_alpha_shape_triangles;
	std::vector<int> m_adjacent_curve_ids;
	toml::table m_tbl;
public:
	Patch(toml::table tbl) { m_tbl = tbl, m_id = next_id++; }
	void set_id(int id) { m_id = id; }
	int get_id() { return m_id; }
	// return predicted points on complexgen, not original point cloud
	double get_epsilon() { return m_tbl["epsilon"].value<double>().value(); }
	std::vector<gp_Pnt> get_grid_points() { return m_grid_points; }
	std::vector<gp_Pnt> get_inner_points() { return m_inner_original_points; }
	std::vector<gp_Pnt> get_unique_inner_points() { return m_inner_unique_original_points; }
	std::vector<gp_Pnt2d> get_uv_points() { return m_uv_points; }
	std::vector<gp_Pnt2d> get_unique_uv_points() { return m_unique_uv_points; }
	std::vector<Triangle_3> get_alpha_shape_triangles() { return m_alpha_shape_triangles; }
	std::vector<Triangle_3> get_unique_alpha_shape_triangles() { return m_unique_alpha_shape_triangles; }
	std::vector<int> get_adjacent_curve_ids() { return m_adjacent_curve_ids; }
	void add_grid_point(const gp_Pnt& point) { m_grid_points.emplace_back(point); }
	void add_inner_point(const gp_Pnt& point) { m_inner_original_points.push_back(point); }
	void add_unique_inner_point(const gp_Pnt& point) { m_inner_unique_original_points.push_back(point); }
	void add_uv_points(const gp_Pnt2d& point) { m_uv_points.push_back(point); }
	void add_unique_uv_points(const gp_Pnt2d& point) { m_unique_uv_points.push_back(point); }
	void add_alpha_shape_triangles(const Triangle_3& triangle) { m_alpha_shape_triangles.push_back(triangle); }
	void add_unique_alpha_shape_triangles(const Triangle_3& triangle) { m_unique_alpha_shape_triangles.push_back(triangle); }
	void add_adjacent_curve_id(int id) { m_adjacent_curve_ids.push_back(id); }
	std::vector<Triangle_2> triangles_of_alpha_shape();
	std::vector<Triangle_2> triangles_of_unique_alpha_shape();
	//std::vector<Triangle_2> triangles_of_periodic_alpha_shape();
	virtual void select_inner_original_points(Point_set original_point_cloud) = NULL;
	virtual std::string get_patch_type() = NULL;
	virtual void project_inner_points_to_2d() = NULL;
	virtual void project_unique_inner_points_to_2d() = NULL;
	virtual Point_3 get_point_3_by_uv(double u, double v) = NULL;
	virtual double calculate_distance(const gp_Pnt& point) = NULL;
};

class MyCylinder : public Patch
{
public:
	MyCylinder(const Json::Value& patch, toml::table tbl);
	void select_inner_original_points(Point_set original_point_cloud) override;
	std::string get_patch_type() override { return "Cylinder"; }
	void project_inner_points_to_2d() override;
	void project_unique_inner_points_to_2d() override;
	Point_3 get_point_3_by_uv(double u, double v) override;
	double calculate_distance(const gp_Pnt& point) override;
private:
	gp_Cylinder m_cylinder;
};


class MySphere : public Patch
{
public:
	MySphere(const Json::Value& patch, toml::table tbl);
	void select_inner_original_points(Point_set original_point_cloud) override;
	std::string get_patch_type() override { return "Sphere"; }
	void project_inner_points_to_2d() override;
	void project_unique_inner_points_to_2d() override;
	Point_3 get_point_3_by_uv(double u, double v) override;
	double calculate_distance(const gp_Pnt& point) override;
private:
	gp_Sphere m_sphere;
};


class MyCone : public Patch
{
public:
	MyCone(const Json::Value& patch, toml::table tbl);
	void select_inner_original_points(Point_set original_point_cloud) override;
	std::string get_patch_type() override { return "Cone"; }
	void project_inner_points_to_2d() override;
	void project_unique_inner_points_to_2d() override;
	Point_3 get_point_3_by_uv(double u, double v) override;
	double calculate_distance(const gp_Pnt& point) override;
private:
	gp_Cone m_cone;
};


class MyPlane : public Patch
{
public:
	MyPlane(const Json::Value& patch, toml::table tbl);
	void select_inner_original_points(Point_set original_point_cloud) override;
	std::string get_patch_type() override { return "Plane"; }
	void project_inner_points_to_2d() override;
	void project_unique_inner_points_to_2d() override;
	Point_3 get_point_3_by_uv(double u, double v) override;
	double calculate_distance(const gp_Pnt& point) override;
private:
	gp_Pln m_plane;
};

class MyBSpline : public Patch
{
public:
	MyBSpline(const Json::Value& patch, toml::table tbl);
	void select_inner_original_points(Point_set original_point_cloud) override;
	std::string get_patch_type() override { return "BSpline"; }
	void project_inner_points_to_2d() override;
	void project_unique_inner_points_to_2d() override;
	Point_3 get_point_3_by_uv(double u, double v) override;
	double calculate_distance(const gp_Pnt& point) override;
private:
	Handle(Geom_BSplineSurface) m_b_spline_surface;
};


#endif