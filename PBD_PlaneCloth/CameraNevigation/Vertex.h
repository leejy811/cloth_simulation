#ifndef __VERTEX_H__
#define __VERTEX_H__
#pragma once
#include "Vec3.h"
#include "Quaternion.h"
#include <vector>

using namespace std;

class Face;
class Vertex
{
public:
	int				_index;
	Vec3<double>	_pos; // X,Y,Z
	Vec3<double>	_pos1; // X,Y,Z
	Vec3<double>	_normal;
	Vec3<double> _vel;
	Vec3<double> _angVel;
	Quaternion _orientation;
	Quaternion _orientation1;
	Vec3<double> _volGradC;
	double	_invMass;
	double	_airDrag;
	vector<Face*>	_nbFaces; // Neighbor face
	vector<Vertex*>	_nbVertices; // Neighbor vertex
public:
	Vertex();
	Vertex(int index, Vec3<double> pos, Vec3<double> vel, double invMass)
	{
		_index = index;
		_pos = pos;
		_vel = vel;
		_invMass = invMass;
	}
	~Vertex();
public:
	inline double x(void) { return _pos.x(); }
	inline double y(void) { return _pos.y(); }
	inline double z(void) { return _pos.z(); }
public:
	bool	hasNbVertex(Vertex* v);
};

#endif