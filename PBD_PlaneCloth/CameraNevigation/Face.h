#ifndef __FACE_H__
#define __FACE_H__

#pragma once
#include "Vertex.h"
#include <vector>

using namespace std;

class Face
{
public:
	Vec3<double>	_normal;
	vector<Vertex*>	_vertices; // Triangle : num. vertex -> 3
public:
	Face();
	Face(Vertex* v0, Vertex* v1, Vertex* v2)
	{
		_vertices.push_back(v0);
		_vertices.push_back(v1);
		_vertices.push_back(v2);
	}
	~Face();
public:
	int		getIndex(Vertex* v);
};

#endif
