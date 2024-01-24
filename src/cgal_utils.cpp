#include "cgal_utils.h"


bool save_gp_points(std::vector<gp_Pnt>& points, std::string full_path)
{
	Point_set point_set;

	// convert gp_Pnt to Point_set
	for (auto& point: points)
	{
		auto it = point_set.insert(Point_3(point.X(), point.Y(), point.Z()));
	}

	if (!CGAL::IO::write_PLY(full_path, point_set, CGAL::parameters::stream_precision(17)))
		return false;
	return true;
}

bool save_triangles_3(std::vector<Triangle_3>& triangles, std::string full_path)
{
	Surface_mesh mesh;
	auto face_color = mesh.add_property_map<Surface_mesh::Face_index, Color>("f:color", Color(0, 0, 0)).first;
	// auto v_normal = mesh.add_property_map<Surface_mesh::Vertex_index, Vector_3>("v:normal", Vector_3(0, 0, 0)).first;

	for (auto& tri: triangles)
	{
		CGAL::SM_Vertex_index v1 = mesh.add_vertex(tri.vertex(0));
		CGAL::SM_Vertex_index v2 = mesh.add_vertex(tri.vertex(1));
		CGAL::SM_Vertex_index v3 = mesh.add_vertex(tri.vertex(2));
		CGAL::SM_Face_index f = mesh.add_face(v1, v2, v3);
		CGAL::Random random(static_cast<unsigned int>(triangles.size()));
		// Assign a random color to each region
		const unsigned char r = static_cast<unsigned char>(random.get_int(0, 192) + 64);
		const unsigned char g = static_cast<unsigned char>(random.get_int(0, 192) + 64);
		const unsigned char b = static_cast<unsigned char>(random.get_int(0, 192) + 64);
		face_color[f] = Color(r, g, b);
	}

	if (!CGAL::IO::write_PLY(full_path, mesh))
		return false;
	return true;
}


void CDTriangulation(CDT& cdt, std::map<Point_2, CDT::Vertex_handle>& p_index, Alpha_shape_2& as)
{
	//insert vertex
	auto vit = as.alpha_shape_vertices_begin();
	while (vit != as.alpha_shape_vertices_end()) {
		CDT::Vertex_handle v = cdt.insert((*vit)->point());
		p_index[(*vit)->point()] = v;
		vit++;
	}

	//insert edge
	auto eit = as.alpha_shape_edges_begin();
	while (eit != as.alpha_shape_edges_end()) {
		switch (as.classify(*eit))
		{
		case Alpha_shape_2::SINGULAR:
			eit++;
			continue;
		default:
			break;
		}
		CDT::Vertex_handle v1 = p_index[as.segment(*eit).source()];
		CDT::Vertex_handle v2 = p_index[as.segment(*eit).target()];

		if (!v1->is_valid() || !v2->is_valid())
			std::cout << "invalid!" << std::endl;

		cdt.insert_constraint(v1, v2);
		eit++;
	}
}