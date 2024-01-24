#pragma once
#ifndef CURVE_H
#define CURVE_H

#include <json/json.h>
#include <gp_Pnt.hxx>

class Curve
{
private:
	inline static int next_id = 0;
	int m_id;

	// exact position of 34 points uniformly sampled on the curve
	std::vector<gp_Pnt> m_points;
	// geometric type of the curve, including 'Line', 'Circle', 'Ellipse', 'BSpline'
	std::string m_type;
	// whether the curve is closed
	bool m_closed;
public:
	Curve(Json::Value curve);
	void set_id(int id) { m_id = id; }
	int get_id() { return m_id; }
	std::vector<gp_Pnt> get_points() { return m_points; }
	std::string get_type() { return m_type; }
	bool is_closed() { return m_closed; }
};


#endif