#ifndef __PBD_OBJECT_CLOTH_H__
#define __PBD_OBJECT_CLOTH_H__

#pragma once
#include "Vec3.h"
#include "Vertex.h"
#include "Face.h"
#include <vector>

using namespace std;

class PBD_ObjectCloth
{
public:
	//temp
	Vec3<double>	_minBoundary;
	Vec3<double>	_maxBoundary;
	vector<Vertex*> _vertices;
	vector<Face*> _faces;
	vector<int> _fixIndex;
public:
	double			_restVolume;
	double			_addVolume;
	double			_iteration = 5.0;
	double			_springK = 0.7;
	double			_volumeK = 0.99;
	double			_pressure;
	double			_bernoulliConst;
	bool				_isAirRelease = false;
public:
	PBD_ObjectCloth();
	PBD_ObjectCloth(char* filename)
	{
		loadObj(filename);
	}
	~PBD_ObjectCloth();
public:
	void reset(void);
	void	loadObj(char* filename);
	void moveToCenter(double scale);
	void	buildAdjacency(void);
	void selectFixVertex(void);
	void	integrate(double dt);
	void	simulation(double dt);
	void	computeRestLength(void);
	void computeRestVolume(void);
	void	computeNormal(void);
	void	computeBernoulliConst(void);
	void	updateBendSprings(void);
	void	updateStructuralSprings(void);
	void updateMass(void);
	void updatePressure(void);
	void updateNormal(void);
	void	applyWind(vec3 wind);
	void	applyBallon(void);
	void	applyAirRelease(void);
	void	onAirRelease(void);
	void	computeWindForTriangle(vec3 wind, Face* f);
	void	applyExtForces(double dt);
	void	solveDistanceConstraint(int index0, int index1, double restLength);
	void solvePressureConstraint(double restVolume);
public:
	void	drawWire(void);
	void	drawSolid(void);
	void	drawPoint(void);
};

#endif
