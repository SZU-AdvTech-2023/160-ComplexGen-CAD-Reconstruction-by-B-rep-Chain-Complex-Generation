#include "curve.h"

Curve::Curve(Json::Value curve)
{
	m_id = next_id++;
	m_closed = curve["closed"].asBool();
	m_type = curve["type"].asString();
	int point_number = static_cast<int>(curve["pts"].size()) / 3;
	for (int i = 0; i < point_number; i++)
		m_points.emplace_back(gp_Pnt(curve["pts"][i * 3].asDouble(), curve["pts"][i * 3 + 1].asDouble(), curve["pts"][i * 3 + 2].asDouble()));
	// std::cout << "#" << m_type << m_id << " created: " << m_points.size() << " points and ";
}
