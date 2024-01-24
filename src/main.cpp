#include "complexgen_object.h"
#include "patch.h"
#include "curve.h"
#include "cgal_utils.h"
#include "trim_obj.h"
// OCCT
//#include <IMeshData_Status.hxx>
//#include <IMeshTools_Parameters.hxx>
//#include <BRepMesh_IncrementalMesh.hxx>
//
//#include <gp_Pnt.hxx>
//#include <gp_Cylinder.hxx>
//#include <Geom_CylindricalSurface.hxx>
//
//#include <IntAna_IntQuadQuad.hxx>
//#include <IntAna_Quadric.hxx>

const double epsilon = 0.007;

void save_shapes(std::vector<Patch*> patches, std::string full_path)
{
    Surface_mesh mesh;
    auto face_color = mesh.add_property_map<Surface_mesh::Face_index, Color>("f:color", Color(0, 0, 0)).first;

    int seed = patches.size();
    for (auto& patch : patches)
    {
        // if (patch->get_patch_type() == "Cone" || patch->get_patch_type() == "BSpline") continue;
        CGAL::Random random(static_cast<unsigned int>(seed--));
        // Assign a random color to each region
        const unsigned char r = static_cast<unsigned char>(random.get_int(0, 192) + 64);
        const unsigned char g = static_cast<unsigned char>(random.get_int(0, 192) + 64);
        const unsigned char b = static_cast<unsigned char>(random.get_int(0, 192) + 64);
        // std::cout << static_cast<int>(r) << " " << static_cast<int>(g) << " " << static_cast<int>(b) << std::endl;
        for (auto& tri : patch->get_alpha_shape_triangles())
        {
            CGAL::SM_Vertex_index v1 = mesh.add_vertex(tri.vertex(0));
            CGAL::SM_Vertex_index v2 = mesh.add_vertex(tri.vertex(1));
            CGAL::SM_Vertex_index v3 = mesh.add_vertex(tri.vertex(2));
            CGAL::SM_Face_index f = mesh.add_face(v1, v2, v3);
            face_color[f] = Color(r, g, b);
        }
    }

    CGAL::IO::write_PLY(full_path, mesh);
}

void save_unique_shapes(std::vector<Patch*> patches, std::string full_path)
{
    Surface_mesh mesh;
    auto face_color = mesh.add_property_map<Surface_mesh::Face_index, Color>("f:color", Color(0, 0, 0)).first;

    int seed = patches.size();
    for (auto& patch : patches)
    {
        // if (patch->get_patch_type() == "Cone" || patch->get_patch_type() == "BSpline") continue;
        CGAL::Random random(static_cast<unsigned int>(seed--));
        // Assign a random color to each region
        const unsigned char r = static_cast<unsigned char>(random.get_int(0, 192) + 64);
        const unsigned char g = static_cast<unsigned char>(random.get_int(0, 192) + 64);
        const unsigned char b = static_cast<unsigned char>(random.get_int(0, 192) + 64);
        // std::cout << static_cast<int>(r) << " " << static_cast<int>(g) << " " << static_cast<int>(b) << std::endl;
        for (auto& tri : patch->get_unique_alpha_shape_triangles())
        {
            CGAL::SM_Vertex_index v1 = mesh.add_vertex(tri.vertex(0));
            CGAL::SM_Vertex_index v2 = mesh.add_vertex(tri.vertex(1));
            CGAL::SM_Vertex_index v3 = mesh.add_vertex(tri.vertex(2));
            CGAL::SM_Face_index f = mesh.add_face(v1, v2, v3);
            face_color[f] = Color(r, g, b);
        }
    }

    CGAL::IO::write_PLY(full_path, mesh);
}


void save_curves(const std::vector<Curve*> curves, const std::string full_path)
{
    Surface_mesh mesh;
    auto color_map = mesh.add_property_map<CGAL::SM_Vertex_index, Color>("v:color", Color(0, 0, 0)).first;

    int seed = curves.size();
    for (auto& curve: curves)
    {
        // if (patch->get_patch_type() == "Cone" || patch->get_patch_type() == "BSpline") continue;
        CGAL::Random random(static_cast<unsigned int>(seed--));
        // Assign a random color to each region
        const unsigned char r = static_cast<unsigned char>(random.get_int(0, 192) + 64);
        const unsigned char g = static_cast<unsigned char>(random.get_int(0, 192) + 64);
        const unsigned char b = static_cast<unsigned char>(random.get_int(0, 192) + 64);
        for (auto& point: curve->get_points())
        {
            auto v_idx = mesh.add_vertex(Point_3(point.X(), point.Y(), point.Z()));
            color_map[v_idx] = Color(r, g, b);
        }
    }

    CGAL::IO::write_PLY(full_path, mesh);
}

void complexgen_trim(const std::string id)
{
    // init ComplexGen object
    ComplexGenObject cgo(id);
    cgo.read_original_point_cloud();

    std::vector<Patch*> patches;

    // construct occt object from cgo
    for (auto& patch : cgo.get_patches())
    {
        Json::Value param = patch["param"];
        std::string type = patch["type"].asString();
        if (type == "Cylinder")
        {
            patches.push_back(new MyCylinder(patch, cgo.get_table()));
        }
        else if (type == "Sphere")
        {
            patches.push_back(new MySphere(patch, cgo.get_table()));
        }
        else if (type == "Cone")
        {
            patches.push_back(new MyCone(patch, cgo.get_table()));
        }
        else if (type == "Plane")
        {
            patches.push_back(new MyPlane(patch, cgo.get_table()));
        }
        else if (type == "BSpline")
        {
            patches.push_back(new MyBSpline(patch, cgo.get_table()));
        }
    }

    // select inner original points on shape 
    for (auto& patch : patches)
    {
    	// std::cout << "Select Point in the range of #" << patch->get_patch_type() << " " << patch->get_id() << std::endl;
        patch->select_inner_original_points(cgo.get_original_point_cloud());
        if (cgo.get_save_intermediate())
        {
            save_gp_points(patch->get_inner_points(), cgo.get_save_dir() + "/" + cgo.get_name() + "_" + patch->get_patch_type() + "_" + std::to_string(patch->get_id()) + ".ply");
            save_gp_points(patch->get_grid_points(), cgo.get_save_dir() + "/" + cgo.get_name() + "_grid_" + patch->get_patch_type() + "_" + std::to_string(patch->get_id()) + ".ply");
        }
    }

  //  // each point can only own to one patch
  //  auto original_point_cloud = cgo.get_original_point_cloud();
  //  for (auto it = original_point_cloud.begin(); it != original_point_cloud.end(); ++it)
  //  {
  //      auto& point = original_point_cloud.point(*it);
  //      auto& gp_point = gp_Pnt(point.x(), point.y(), point.z());
  //      std::pair<double, int> dis(std::numeric_limits<double>::max(), -1);
  //      for (int i_patch = 0; i_patch < patches.size(); i_patch++)
  //      { 
  //      	auto& patch = patches[i_patch];
  //          double distance = patch->calculate_distance(gp_point);
  //          if (distance < epsilon && distance < dis.first)
  //          {
  //              dis.first = distance;
  //              dis.second = i_patch;
  //          }
		//}
  //      if (dis.second != -1)
  //          patches[dis.second]->add_unique_inner_point(gp_point);
  //  }

    //// alpha shape for unique inner point
    //for (auto& patch : patches)
    //{
    //    patch->project_unique_inner_points_to_2d();
    //    auto triangles_2 = patch->triangles_of_unique_alpha_shape();
    //    //auto triangles_2 = patch->triangles_of_periodic_alpha_shape();
    //    for (auto& triangle_2 : triangles_2)
    //    {
    //        Point_2 p1 = triangle_2.vertex(0);
    //        Point_2 p2 = triangle_2.vertex(1);
    //        Point_2 p3 = triangle_2.vertex(2);
    //        Point_3 p1_3d = patch->get_point_3_by_uv(p1.x(), p1.y());
    //        Point_3 p2_3d = patch->get_point_3_by_uv(p2.x(), p2.y());
    //        Point_3 p3_3d = patch->get_point_3_by_uv(p3.x(), p3.y());
    //        patch->add_unique_alpha_shape_triangles(Triangle_3(p1_3d, p2_3d, p3_3d));
    //    }
    //    if (cgo.get_save_intermediate())
    //        save_triangles_3(patch->get_unique_alpha_shape_triangles(), cgo.get_save_dir() + "/" + cgo.get_name() + "_triangles_of_as_" + patch->get_patch_type() + "_" + std::to_string(patch->get_id()) + ".ply");
    //}


    // alpha shape for inner point
    for (auto& patch : patches)
    {
        patch->project_inner_points_to_2d();
        auto triangles_2 = patch->triangles_of_alpha_shape();
        //auto triangles_2 = patch->triangles_of_periodic_alpha_shape();
        for (auto& triangle_2 : triangles_2)
        {
            Point_2 p1 = triangle_2.vertex(0);
            Point_2 p2 = triangle_2.vertex(1);
            Point_2 p3 = triangle_2.vertex(2);
            Point_3 p1_3d = patch->get_point_3_by_uv(p1.x(), p1.y());
            Point_3 p2_3d = patch->get_point_3_by_uv(p2.x(), p2.y());
            Point_3 p3_3d = patch->get_point_3_by_uv(p3.x(), p3.y());
            patch->add_alpha_shape_triangles(Triangle_3(p1_3d, p2_3d, p3_3d));
        }
        if (cgo.get_save_intermediate())
			save_triangles_3(patch->get_alpha_shape_triangles(), cgo.get_save_dir() + "/" + cgo.get_name() + "_triangles_of_as_" + patch->get_patch_type() + "_" + std::to_string(patch->get_id()) + ".ply");
    }

    save_shapes(patches, cgo.get_save_dir() + "/" + cgo.get_name() + "_complexgen_trim.ply");
    //save_unique_shapes(patches, cgo.get_save_dir() + "/" + cgo.get_name() + "_unique_complexgen_trim.ply");

    std::vector<Curve*> curves;
    for (auto& curve : cgo.get_curves())
    {
        curves.push_back(new Curve(curve));
        if (cgo.get_save_intermediate())
			save_gp_points(curves.back()->get_points(), cgo.get_save_dir() + "/" + cgo.get_name() + "_curves_" + curves.back()->get_type() + "_" + std::to_string(curves.back()->get_id()) + ".ply");
    }

    if (cgo.get_save_curve())
		save_curves(curves, cgo.get_save_dir() + "/" + cgo.get_name() + "_curves.ply");

    //assert(patches.size() == cgo.get_patch2curve().size());
    //assert(curves.size() == cgo.get_patch2curve().size()[0].size());

    //for (auto& patch : patches)
    //{
    //    int patch_id = patch->get_id();
    //    auto patch2curve = cgo.get_patch2curve();
    //    int curve_number = static_cast<int>(patch2curve[patch_id].size());
    //    std::cout << "#patch " << patch_id << " is " << patch->get_patch_type() << ": ";
    //    for (int curve_id = 0; curve_id < curve_number; curve_id++)
    //        if (patch2curve[patch_id][curve_id] == 1)
    //        {
    //            patch->add_adjacent_curve_id(curve_id);
    //            std::cout << curve_id << " ";
    //        }
    //    std::cout << std::endl;
    //}
}


int main()
{
    toml::table tbl;
    try
    {
        tbl = toml::parse_file("../config.toml");
    }
    catch (const toml::parse_error& err)
    {
        std::cerr
            << "Error parsing file '" << err.source().path
            << "':\n" << err.description()
            << "\n (" << err.source().begin << ")\n";
        return -1;
    }

    const std::string trim_type = tbl["trim_type"].value<std::string>().value();
    toml::array& test_ids = *tbl.get_as<toml::array>("test_ids");
    if (trim_type == "trim_obj")
    {
		for (int i = 0; i < test_ids.size(); i++)
		{
			auto id = test_ids[i].value<std::string>().value();
			TrimObj trim_obj(id, tbl);
            trim_obj.read_original_point_cloud();
            trim_obj.trim();
		}
	    
    }
    else if (trim_type == "trim_json")
    {
        for (int i = 0; i < test_ids.size(); i++)
        {
            auto id = test_ids[i].value<std::string>().value();
            complexgen_trim(id);
        }
    }

    return 0;
}




