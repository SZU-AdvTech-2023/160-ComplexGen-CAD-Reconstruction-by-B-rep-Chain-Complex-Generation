#pragma once
#ifndef COMPLEXGEN_OBJECT_H
#define COMPLEXGEN_OBJECT_H


#include <iostream>
#include <json/json.h>
#include <fstream>
#include <cassert>
#include <toml++/toml.h>

#include "cgal_utils.h"


class ComplexGenObject
{
private:
    std::string m_name;
    std::vector<Json::Value> m_patches;
    std::vector<Json::Value> m_curves;
    std::vector<Json::Value> m_corners;
    std::vector<std::vector<int>> m_curve2corner;
    std::vector<std::vector<int>> m_patch2corner;
    std::vector<std::vector<int>> m_patch2curve;
    Point_set m_original_point_cloud;
    toml::table m_tbl;
public:
    ComplexGenObject(std::string name);
    void read_original_point_cloud();
    std::vector<Json::Value> get_patches() { return m_patches; }
    std::vector<Json::Value> get_curves() { return m_curves; }
    std::vector<std::vector<int>> get_curve2corner() { return m_curve2corner; }
	std::vector<std::vector<int>> get_patch2corner() { return m_patch2corner; }
	std::vector<std::vector<int>> get_patch2curve() { return m_patch2curve; }
    Point_set get_original_point_cloud() { return m_original_point_cloud; }
    std::string get_name() { return m_name; }
    std::string get_save_dir() { return m_tbl["save_dir"].value<std::string>().value(); }
    toml::table get_table() { return m_tbl; }
    bool get_save_intermediate() { return m_tbl["save_intermediate"].value<bool>().value(); }
    bool get_save_curve() { return m_tbl["save_curve"].value<bool>().value(); }
};


#endif