#ifndef __PBD_PLANE_CLOTH_H__
#define __PBD_PLANE_CLOTH_H__

#pragma once
#include "Vec3.h"
#include "Vertex.h"
#include "Face.h"
#include <vector>

using namespace std;

class PBD_PlaneCloth
{
public:
	int				_res[2]; // width and height
	//vector<double>	_bendingForce;
	//vector<double>	_dihedralAngle;
	vector<Vertex*> _vertices;
	vector<Face*> _faces;
public:
	double			_structuralLength0Horiz;			// horizontal structural
	double			_structuralLength0Verti;			// vertical structural
	double			_structuralLength0Diago;		// vertical structural
	double			_bendLength0Short;				// horizontal bend
	double			_bendLength0LongHoriz;		// horizontal bend
	double			_bendLength0LongVerti;			// vertical bend
	double			_shearLength0Horiz;				// horizontal shear
	double			_shearLength0Verti;				// Vertical shear 
public:
	PBD_PlaneCloth();
	PBD_PlaneCloth(int width, int height);
	~PBD_PlaneCloth();
public:
	inline vec3	pos(int i, int j) { return _vertices[j * _res[0] + i]->_pos; }
public:
	void	init(void);
	void	reset(void);
	void	buildAdjacency(void);
	void	integrate(double dt);
	void	simulation(double dt);
	void	computeRestLength(void);
	void	computeDihedralAngle(void);
	void	updateBendSprings(void);
	void	updateShearSprings(void);
	void	updateStructuralSprings(void);
	void	applyWind(vec3 wind);
	void	computeWindForTriangle(vec3 wind, int index0, int index1, int index2);
	void	applyExtForces(double dt);
	void	solveDistanceConstraint(int index0, int index1, double restLength);
	void solvePressureConstraint(double restVolume);
	void	solveDihedralConstraint(int index0, int index1, int index2, int index3, double restAngle);
public:
	void	draw(void);
	void	drawSpring(void);
	void	drawOutline(void);
};

#endif
