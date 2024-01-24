#include "patch.h"


// If the patch is a 'Cylinder': params[0:3] = axis_direction, params[3:6] = point_on_axis, params[6] = radius
MyCylinder::MyCylinder(const Json::Value& patch, toml::table tbl) : Patch(tbl)
{
	Json::Value param = patch["param"];
	// set grid points on the cylinder
	Json::Value grid = patch["grid"];
	int patch_sample_number = static_cast<int>(grid.size()) / 3;
	for (int i = 0; i < patch_sample_number; i++)
		add_grid_point(gp_Pnt(grid[i * 3].asDouble(), grid[i * 3 + 1].asDouble(), grid[i * 3 + 2].asDouble()));

	gp_Ax3 theA3(gp_Pnt(param[3].asDouble(), param[4].asDouble(), param[5].asDouble()),
		gp_Dir(param[0].asDouble(), param[1].asDouble(), param[2].asDouble()));
	m_cylinder = gp_Cylinder(theA3, param[6].asDouble());
}

void MyCylinder::select_inner_original_points(Point_set original_point_cloud)
{
	for (auto it = original_point_cloud.begin(); it != original_point_cloud.end(); it++)
	{
		auto& point = original_point_cloud.point_map()[*it];
		auto& gp_point = gp_Pnt(point.x(), point.y(), point.z());

		// check if the point is on the cylinder
		gp_Dir axis_direction = m_cylinder.Axis().Direction();
		gp_Vec point_to_location = gp_Vec(m_cylinder.Axis().Location(), gp_point);
		gp_Vec projection_to_axis = point_to_location.Dot(axis_direction) * axis_direction;
		// distance_to_patch = distance_to_axis - radius
		double distance_to_patch = (point_to_location - projection_to_axis).Magnitude() - m_cylinder.Radius();

		if (std::fabs(distance_to_patch) < get_epsilon())
		{
			add_inner_point(gp_point);
		}
	}
	// std::cout << get_inner_points().size() << " of " << original_point_cloud.size() << " points in " << get_patch_type() << " #" << get_id() << std::endl;
}

void MyCylinder::project_inner_points_to_2d()
{
	for (auto& point: get_inner_points())
	{
		double u, v;
		ElSLib::Parameters(m_cylinder, point, u, v);
		add_uv_points(gp_Pnt2d(u, v));
	}
}

void MyCylinder::project_unique_inner_points_to_2d()
{
	for (auto& point : get_unique_inner_points())
	{
		double u, v;
		ElSLib::Parameters(m_cylinder, point, u, v);
		add_unique_uv_points(gp_Pnt2d(u, v));
	}
}

Point_3 MyCylinder::get_point_3_by_uv(double u, double v)
{
	gp_Pnt point = ElSLib::Value(u, v, m_cylinder);
	return Point_3(point.X(), point.Y(), point.Z());
}

double MyCylinder::calculate_distance(const gp_Pnt& point)
{
	gp_Dir axis_direction = m_cylinder.Axis().Direction();
	gp_Vec point_to_location = gp_Vec(m_cylinder.Axis().Location(), point);
	gp_Vec projection_to_axis = point_to_location.Dot(axis_direction) * axis_direction;
	double distance_to_patch = (point_to_location - projection_to_axis).Magnitude() - m_cylinder.Radius();
	return distance_to_patch;
}

#pragma optimize("", off)
// If the patch is a 'Cone': params[0:3] = axis_direction, params[3:6] = apex, params[6] = angle
MyCone::MyCone(const Json::Value& patch, toml::table tbl) : Patch(tbl)
{
	Json::Value param = patch["param"];

	// set grid points on the cone
	Json::Value grid = patch["grid"];
	int patch_sample_number = static_cast<int>(grid.size()) / 3;
	for (int i = 0; i < patch_sample_number; i++)
		add_grid_point(gp_Pnt(grid[i * 3].asDouble(), grid[i * 3 + 1].asDouble(), grid[i * 3 + 2].asDouble()));

	gp_Dir axis_direction(param[0].asDouble(), param[1].asDouble(), param[2].asDouble());
	gp_Pnt apex(param[3].asDouble(), param[4].asDouble(), param[5].asDouble());

	gp_Ax3 theA3(apex, axis_direction);
	m_cone = gp_Cone(theA3, param[6].asDouble(), 0.0000001);
}

void MyCone::select_inner_original_points(Point_set original_point_cloud)
{
	for (auto it = original_point_cloud.begin(); it != original_point_cloud.end(); it++)
	{
		auto& point = original_point_cloud.point_map()[*it];
		auto& gp_point = gp_Pnt(point.x(), point.y(), point.z());

		// check if the point is on the cone
		double semi_angle = m_cone.SemiAngle();
		gp_Pnt apex = m_cone.Apex();
		gp_Dir axis_direction = m_cone.Axis().Direction();
		gp_Vec apex_to_point = gp_Vec(apex, gp_point);
		double angle_diff = std::acos(apex_to_point.Dot(axis_direction) / apex_to_point.Magnitude()) - semi_angle;

		if (std::fabs(angle_diff) < get_epsilon())
		{
			add_inner_point(gp_point);
		}
	}
	// std::cout << get_inner_points().size() << " of " << original_point_cloud.size() << " points in " << get_patch_type() << " #" << get_id() << std::endl;
}

void MyCone::project_inner_points_to_2d()
{
	for (auto& point : get_inner_points())
	{
		double u, v;
		ElSLib::Parameters(m_cone, point, u, v);
		add_uv_points(gp_Pnt2d(u, v));
	}
}

void MyCone::project_unique_inner_points_to_2d()
{
	for (auto& point : get_unique_inner_points())
	{
		double u, v;
		ElSLib::Parameters(m_cone, point, u, v);
		add_unique_uv_points(gp_Pnt2d(u, v));
	}
}

Point_3 MyCone::get_point_3_by_uv(double u, double v)
{
	gp_Pnt point = ElSLib::Value(u, v, m_cone);
	return Point_3(point.X(), point.Y(), point.Z());
}

double MyCone::calculate_distance(const gp_Pnt& point)
{
	return 0.0;
}
#pragma optimize("", on)

// If the patch is a 'Sphere': params[0:3] = center, params[3] = radius
MySphere::MySphere(const Json::Value& patch, toml::table tbl) : Patch(tbl)
{
	Json::Value param = patch["param"];
	// set grid points on the sphere
	Json::Value grid = patch["grid"];
	int patch_sample_number = static_cast<int>(grid.size()) / 3;
	for (int i = 0; i < patch_sample_number; i++)
		add_grid_point(gp_Pnt(grid[i * 3].asDouble(), grid[i * 3 + 1].asDouble(), grid[i * 3 + 2].asDouble()));

	gp_Ax3 theA3(gp_Pnt(param[0].asDouble(), param[1].asDouble(), param[2].asDouble()),
		gp_Dir(param[0].asDouble(), param[1].asDouble(), param[2].asDouble()));
	m_sphere = gp_Sphere(theA3,param[3].asDouble());
}

void MySphere::select_inner_original_points(Point_set original_point_cloud)
{
	std::vector<gp_Pnt> save_points;
	for (auto it = original_point_cloud.begin(); it != original_point_cloud.end(); it++)
	{
		auto& point = original_point_cloud.point_map()[*it];
		gp_Pnt gp_point(point.x(), point.y(), point.z());
		// if point is in m_grid_points, then it is on the cylinder
		double distance_to_center = std::fabs(m_sphere.Location().Distance(gp_point));
		if (distance_to_center < get_epsilon())
		{
			add_inner_point(gp_point);
		}
	}
}

void MySphere::project_inner_points_to_2d()
{
	for (auto& point : get_inner_points())
	{
		double u, v;
		ElSLib::Parameters(m_sphere, point, u, v);
		add_uv_points(gp_Pnt2d(u, v));
	}
}

void MySphere::project_unique_inner_points_to_2d()
{
	for (auto& point : get_unique_inner_points())
	{
		double u, v;
		ElSLib::Parameters(m_sphere, point, u, v);
		add_unique_uv_points(gp_Pnt2d(u, v));
	}
}

Point_3 MySphere::get_point_3_by_uv(double u, double v)
{
	gp_Pnt point = ElSLib::Value(u, v, m_sphere);
	return Point_3(point.X(), point.Y(), point.Z());
}

double MySphere::calculate_distance(const gp_Pnt& point)
{
	return std::fabs(m_sphere.Location().Distance(point));
}

// If the patch is a 'Plane' of form dot(normal, x) - d = 0: params[0:3] stores the normal direction, params[3] = d
MyPlane::MyPlane(const Json::Value& patch, toml::table tbl) : Patch(tbl)
{
	Json::Value param = patch["param"];

	// set grid points on the plane
	Json::Value grid = patch["grid"];
	int patch_sample_number = static_cast<int>(grid.size()) / 3;
	for (int i = 0; i < patch_sample_number; i++)
		add_grid_point(gp_Pnt(grid[i * 3].asDouble(), grid[i * 3 + 1].asDouble(), grid[i * 3 + 2].asDouble()));

	m_plane = gp_Pln(param[0].asDouble(), param[1].asDouble(), param[2].asDouble(), -param[3].asDouble());
}

void MyPlane::select_inner_original_points(Point_set original_point_cloud)
{
	for (auto it = original_point_cloud.begin(); it != original_point_cloud.end(); it++)
	{
		auto& point = original_point_cloud.point_map()[*it];
		auto& gp_point = gp_Pnt(point.x(), point.y(), point.z());

		// check if the point is on the plame
		double distance_to_patch = std::fabs(m_plane.Distance(gp_point));

		if (distance_to_patch < get_epsilon())
		{
			add_inner_point(gp_point);
		}
	}
	// std::cout << get_inner_points().size() << " of " << original_point_cloud.size() << " points in " << get_patch_type() << " #" << get_id() << std::endl;
}

void MyPlane::project_inner_points_to_2d()
{
	for (auto& point : get_inner_points())
	{
		double u, v;
		ElSLib::Parameters(m_plane, point, u, v);
		add_uv_points(gp_Pnt2d(u, v));
	}
}

void MyPlane::project_unique_inner_points_to_2d()
{
	for (auto& point : get_unique_inner_points())
	{
		double u, v;
		ElSLib::Parameters(m_plane, point, u, v);
		add_unique_uv_points(gp_Pnt2d(u, v));
	}
}

Point_3 MyPlane::get_point_3_by_uv(double u, double v)
{
	gp_Pnt point = ElSLib::Value(u, v, m_plane);
	return Point_3(point.X(), point.Y(), point.Z());
}

double MyPlane::calculate_distance(const gp_Pnt& point)
{
	return std::fabs(m_plane.Distance(point));
}

MyBSpline::MyBSpline(const Json::Value& patch, toml::table tbl) : Patch(tbl)
{
	Json::Value param = patch["param"];

	// set grid points on the plane
	Json::Value grid = patch["grid"];
	int patch_sample_number = static_cast<int>(grid.size()) / 3;
	for (int i = 0; i < patch_sample_number; i++)
		add_grid_point(gp_Pnt(grid[i * 3].asDouble(), grid[i * 3 + 1].asDouble(), grid[i * 3 + 2].asDouble()));

	int u_dim = patch["u_dim"].asInt(), v_dim = patch["v_dim"].asInt();
	TColgp_Array2OfPnt array(1, u_dim, 1, v_dim);
	for (int i = 1; i <= u_dim; i++)
		for (int j = 1; j <= v_dim; j++)
		{
			int index = (i - 1) * v_dim + j - 1; // Calculate the 1D index
			gp_Pnt p(grid[index * 3].asDouble(), grid[index * 3 + 1].asDouble(), grid[index * 3 + 2].asDouble());
			array.SetValue(i, j, p);
		}
	m_b_spline_surface = GeomAPI_PointsToBSplineSurface(array).Surface();
	if (patch["u_closed"].asBool())
		m_b_spline_surface->SetUPeriodic();

	if (patch["v_closed"].asBool())
		m_b_spline_surface->SetVPeriodic();
}

void MyBSpline::select_inner_original_points(Point_set original_point_cloud)
{
	std::vector<gp_Pnt> points;

	for (auto it = original_point_cloud.begin(); it != original_point_cloud.end(); it++)
	{
		auto point = original_point_cloud.point_map()[*it];
		points.push_back(gp_Pnt(point.x(), point.y(), point.z()));
	}

	std::vector<bool> is_inner(points.size(), false);
	std::atomic<int> progress(0);

	auto start = std::chrono::system_clock::now();
#pragma omp parallel for
	for (int i = 0; i < points.size(); i++)
	{
		progress.fetch_add(1);
		if (progress.load() % 10000 == 0)
			std::cout << "Select inner points progress: " << progress.load() << "/" << points.size() << std::endl;

		gp_Pnt gp_point = points[i];
		// check if the point is on the b-spline
		auto projector = GeomAPI_ProjectPointOnSurf(gp_point, m_b_spline_surface);
		projector.Perform(gp_point);
		if (projector.IsDone())
		{
			auto distance_to_patch = std::fabs(projector.LowerDistance());

			if (distance_to_patch < get_epsilon())
				is_inner[i] = true;
		}
		else
		{
			std::cout << "Projection failed !!" << std::endl;
		}
	}

	for (int i = 0; i < points.size(); i++)
		if (is_inner[i])
			add_inner_point(points[i]);

	auto end = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

	std::cout << "#BSpline" << get_id() << " select inner points success, take " << 
		static_cast<double>(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
	<< " seconds!" << std::endl;
}

void MyBSpline::project_inner_points_to_2d()
{
	std::vector<gp_Pnt> inner_points = get_inner_points();
	std::vector<gp_Pnt2d> inner_uv_points(inner_points.size());
	std::atomic<int> progress(0);
#pragma omp parallel for
	for (int i = 0; i < inner_points.size(); i++)
	{
		progress.fetch_add(1);
		if (progress.load() % 100 == 0)
			std::cout << "Project to get uv Progress: " << progress.load() << "/" << inner_points.size() << std::endl;

		auto& point = inner_points[i];
		double u, v;
		auto projector = GeomAPI_ProjectPointOnSurf(point, m_b_spline_surface);
		projector.Perform(point);
		projector.LowerDistanceParameters(u, v);
		inner_uv_points[i] = gp_Pnt2d(u, v);
	}
	for (auto& p : inner_uv_points)
		add_uv_points(p);
}

void MyBSpline::project_unique_inner_points_to_2d()
{
	std::vector<gp_Pnt> inner_points = get_unique_inner_points();
	std::vector<gp_Pnt2d> inner_uv_points(inner_points.size());
	std::atomic<int> progress(0);
#pragma omp parallel for
	for (int i = 0; i < inner_points.size(); i++)
	{
		progress.fetch_add(1);
		if (progress.load() % 100 == 0)
			std::cout << "Project to get uv Progress: " << progress.load() << "/" << inner_points.size() << std::endl;

		auto& point = inner_points[i];
		double u, v;
		auto projector = GeomAPI_ProjectPointOnSurf(point, m_b_spline_surface);
		projector.Perform(point);
		projector.LowerDistanceParameters(u, v);
		inner_uv_points[i] = gp_Pnt2d(u, v);
	}
	for (auto& p : inner_uv_points)
		add_unique_uv_points(p);
}

Point_3 MyBSpline::get_point_3_by_uv(double u, double v)
{
	gp_Pnt point = m_b_spline_surface->Value(u, v);
	return Point_3(point.X(), point.Y(), point.Z());
}

double MyBSpline::calculate_distance(const gp_Pnt& point)
{
	auto projector = GeomAPI_ProjectPointOnSurf(point, m_b_spline_surface);
	projector.Perform(point);
	if (projector.IsDone())
	{
		auto distance_to_patch = std::fabs(projector.LowerDistance());

		return distance_to_patch;
	}
	std::cout << "Projection failed !!" << std::endl;
	return 0.0;
}

template <class OutputIterator>
void alpha_edges(const Alpha_shape_2& A, OutputIterator out)
{
	Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin(),
		end = A.alpha_shape_edges_end();
	for (; it != end; ++it)
		*out++ = A.segment(*it);
}

template <class OutputIterator>
void alpha_faces(const Alpha_shape_2& A, OutputIterator out)
{
	auto it = A.all_face_handles().begin(),
		end = A.all_face_handles().end();
	for (; it != end; ++it) {
		switch (A.classify(*it)) {
		case Alpha_shape_2::INTERIOR:
			break;
		default:
			continue;
		}
		if (A.classify(*it) != Alpha_shape_2::EXTERIOR)
			*out++ = A.triangle(*it);
	}
}

//template <class OutputIterator>
//void periodic_alpha_edges(const Alpha_shape_2& A, OutputIterator out)
//{
//	Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin(),
//		end = A.alpha_shape_edges_end();
//	for (; it != end; ++it)
//		*out++ = A.segment(*it);
//}


std::vector<Triangle_2> Patch::triangles_of_alpha_shape()
{
	// construct alpha shape
	std::vector<Point_2> points;
	for (auto& point : get_uv_points())
	{
		points.push_back(Point_2(point.X(), point.Y()));
	}
	Alpha_shape_2 as(points.begin(), points.end(), FT(m_tbl["alpha_value"].value<double>().value()), Alpha_shape_2::REGULARIZED);

	// output triangles of alpha shape
	std::vector<Triangle_2> triangles_2;
	alpha_faces(as, std::back_inserter(triangles_2));

	if (get_patch_type() == "BSpline")
	{
		std::cout << "Alpha Shape computed" << std::endl;
		std::cout << triangles_2.size() << " alpha shape triangles" << std::endl;
	}

	return triangles_2;
}

std::vector<Triangle_2> Patch::triangles_of_unique_alpha_shape()
{
	// construct alpha shape
	std::vector<Point_2> points;
	for (auto& point : get_unique_uv_points())
	{
		points.push_back(Point_2(point.X(), point.Y()));
	}
	Alpha_shape_2 as(points.begin(), points.end(), FT(m_tbl["alpha_value"].value<double>().value()), Alpha_shape_2::REGULARIZED);

	// output triangles of alpha shape
	std::vector<Triangle_2> triangles_2;
	alpha_faces(as, std::back_inserter(triangles_2));

	if (get_patch_type() == "BSpline")
	{
		std::cout << "Alpha Shape computed" << std::endl;
		std::cout << triangles_2.size() << " alpha shape triangles" << std::endl;
	}

	return triangles_2;
}


//std::vector<Triangle_2> Patch::triangles_of_periodic_alpha_shape()
//{
//	// construct alpha shape
//	std::vector<Point_2> points;
//	for (auto& point : get_uv_points())
//	{
//		points.push_back(Point_2(point.X(), point.Y()));
//	}
//	// Define the periodic square
//	//P2DT2 pdt(Gt::Iso_rectangle_2(0, -10, 2 * std::acos(-1), 10));
//
//	P2DT2 pdt(Gt::Iso_rectangle_2(-10, -10, 700, 700));
//	// Heuristic for inserting large point sets (if pts is reasonably large)
//	pdt.insert(points.begin(), points.end(), true);
//
//
//	// As pdt won't be modified anymore switch to 1-sheeted cover if possible
//	if (pdt.is_triangulation_in_1_sheet())
//		pdt.convert_to_1_sheeted_covering();
//	std::cout << "Periodic Delaunay computed." << std::endl;
//	// compute alpha shape
//	Alpha_shape_2 as(pdt);
//	std::cout << "Alpha shape computed in REGULARIZED mode by default." << std::endl;
//
//	as.set_alpha(0.0008);
//	std::vector<Triangle_2> triangles_2;
//	auto it = as.finite_faces_begin(), end = as.finite_faces_end();
//	for (; it != end; it++)
//	{
//		if (as.classify(it) == Alpha_shape_2::INTERIOR)
//			triangles_2.push_back(as.triangle(it));
//	}
//	std::cout << "Alpha Shape computed" << std::endl;
//	std::cout << triangles_2.size() << " alpha shape triangles" << std::endl;
//	std::vector<Segment> segments;
//	periodic_alpha_edges(as, std::back_inserter(segments));
//	std::cout << segments.size() << " alpha shape edges" << std::endl;
//	
//	//CDT cdt;
//	//std::map<CDT::Point_2, CDT::Vertex_handle> p_index;
//	//CDTriangulation(cdt, p_index, as);
//
//	//std::vector<Triangle_2> triangles_2;
//
//	//auto fit = cdt.finite_faces_begin();
//	//while (fit != cdt.finite_faces_end()) {
//
//	//	Point_2 p1 = fit->vertex(0)->point();
//	//	Point_2 p2 = fit->vertex(1)->point();
//	//	Point_2 p3 = fit->vertex(2)->point();
//
//	//	if (CGAL::collinear(p1, p2, p3))
//	//		continue;
//
//	//	Point_2 center = Point_2(0, 0) + (((p1 - Point_2(0, 0)) + (p2 - Point_2(0, 0)) + (p3 - Point_2(0, 0))) * 1.0 / 3.0);
//	//	int res = as.classify(center);
//
//	//	if (res == Alpha_shape_2::INTERIOR) {
//	//		triangles_2.push_back(Triangle_2(p1, p2, p3));
//	//	}
//
//	//	fit++;
//	//}
//	return triangles_2;
//}
