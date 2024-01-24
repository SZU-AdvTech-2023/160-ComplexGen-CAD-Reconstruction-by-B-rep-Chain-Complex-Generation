#include "trim_obj.h"

TrimObj::TrimObj(const std::string name, const toml::table tbl) : m_name(name), m_tbl(tbl)
{
	std::string file_path = m_tbl["obj_input_dir"].value<std::string>().value() + m_name + "_geom_refine.obj";
	std::ifstream ifs(file_path);

	if (CGAL::IO::read_OBJ(ifs, m_polyhedron))
	{
        std::cout << "Successfully read the mesh with " << m_polyhedron.size_of_vertices() << " vertices." << std::endl;
        std::cout << "Successfully read the mesh with " << m_polyhedron.size_of_facets() << " faces." << std::endl;

        // Now you can access and iterate through the mesh's faces (polyhedron facets)
        //for (Polyhedron::Facet_iterator facet_iter = m_polyhedron.facets_begin(); facet_iter != m_polyhedron.facets_end(); ++facet_iter) {
        //    // Access the vertices of the current face (facet)
        //    Polyhedron::Halfedge_around_facet_circulator edge_iter(facet_iter->facet_begin());
        //    do {
        //        Point_3 vertex = edge_iter->vertex()->point();
        //        std::cout << "Vertex: " << vertex << std::endl;
        //    } while (++edge_iter != facet_iter->facet_begin());
        //}
	}
	else
	{
		std::cout << "load obj error!" << std::endl;
	}
}

void TrimObj::read_original_point_cloud()
{
    std::string file_path = m_tbl["obj_trim_point_cloud_input_dir"].value<std::string>().value() + m_name + "_10000.ply";
    // Reading input
    if (!CGAL::IO::read_PLY(file_path, m_original_point_cloud))
    {
        std::cerr << "Can't read input file: " << file_path << std::endl;
        return;
    }
    std::cout << "Read " << m_original_point_cloud.size() << " points from " << file_path << std::endl;
}

void TrimObj::trim()
{
    for (Polyhedron::Vertex_handle v = m_polyhedron.vertices_begin(); v != m_polyhedron.vertices_end(); v++)
    {
        Point_3 point = v->point();

        double min_distance = std::numeric_limits<double>::max();
        for (auto& other_it : m_original_point_cloud)
        {
            auto other_point = m_original_point_cloud.point(other_it);
            auto distance = std::sqrt(CGAL::squared_distance(point, other_point));
            min_distance = std::fmin(min_distance, distance);
        }
        if (min_distance > get_epsilon())
        {
            std::vector<Polyhedron::Halfedge_around_vertex_circulator> facets_to_delete;

            Polyhedron::Halfedge_around_vertex_circulator he = v->vertex_begin();
            do
            {
	            if (!he->is_border())
	            	facets_to_delete.push_back(he);
                he = next(he);
            } while (he != v->vertex_begin());

            for (auto& facet: facets_to_delete)
                m_polyhedron.erase_facet(facet);
        }
	}

    std::string full_path = m_tbl["save_dir"].value<std::string>().value() + m_name + "_trim_refinement_obj.ply";
    CGAL::IO::write_PLY(full_path, m_polyhedron);
}