#include "complexgen_object.h"


ComplexGenObject::ComplexGenObject(std::string name) : m_name(name)
{
    try
    {
        m_tbl = toml::parse_file("../config.toml");
    }
    catch (const toml::parse_error& err)
    {
        std::cerr
            << "Error parsing file '" << err.source().path
            << "':\n" << err.description()
            << "\n (" << err.source().begin << ")\n";
        return;
    }

    std::string file_path = m_tbl["json_input_dir"].value<std::string>().value() + m_name + "_geom_refine.json";
    std::ifstream ifs(file_path);
    ifs.open(file_path);

    assert(ifs.is_open());

    Json::Reader reader;
    Json::Value root;
    if (reader.parse(ifs, root))
    {
        // parse patches info
        Json::Value patches = root["patches"];
        for (Json::Value& patch : patches)
            m_patches.push_back(patch);

        // parse curves info
        Json::Value curves = root["curves"];
        for (Json::Value& curve : curves)
            m_curves.push_back(curve);

        // parse corners info
        Json::Value corners = root["corners"];
        for (Json::Value& corner : corners)
            m_corners.push_back(corner);

        // parse curve2corner info
        Json::Value curve2corner = root["curve2corner"];
        for (Json::Value& c2c : curve2corner)
        {
            std::vector<int> tmp;
            for (Json::Value& c : c2c)
                tmp.push_back(c.asInt());
            m_curve2corner.push_back(tmp);
        }

        // parse patch2corner info
        Json::Value patch2corner = root["patch2corner"];
        for (Json::Value& p2c : patch2corner)
        {
            std::vector<int> tmp;
            for (Json::Value& p : p2c)
                tmp.push_back(p.asInt());
            m_patch2corner.push_back(tmp);
        }

        // parse patch2curve info
        Json::Value patch2curve = root["patch2curve"];
        for (Json::Value& p2c : patch2curve)
        {
            std::vector<int> tmp;
            for (Json::Value& p : p2c)
                tmp.push_back(p.asInt());
            m_patch2curve.push_back(tmp);
        }

    }
    else
    {
        std::cout << "parse error!" << std::endl;
    }
}


void ComplexGenObject::read_original_point_cloud()
{
    std::string file_path = m_tbl["point_cloud_input_dir"].value<std::string>().value() + m_name + "_more_points.ply";
    // Reading input
    if (!CGAL::IO::read_PLY(file_path, m_original_point_cloud))
    {
        std::cerr << "Can't read input file: " << file_path << std::endl;
        return;
    }
    std::cout << "Read " << m_original_point_cloud.size() << " points from " << file_path << std::endl;
}
