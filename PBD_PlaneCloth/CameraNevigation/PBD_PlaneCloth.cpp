#include "PBD_PlaneCloth.h"
#include "GL\glut.h"
#include <Windows.h>

//#define BEND_DIHEDRAL_ANGLE

PBD_PlaneCloth::PBD_PlaneCloth()
{
}

PBD_PlaneCloth::PBD_PlaneCloth(int width, int height)
{
	_res[0] = width;
	_res[1] = height;

	init();
	computeRestLength();
	//computeDihedralAngle();
}

PBD_PlaneCloth::~PBD_PlaneCloth()
{
}

/*void PBD_PlaneCloth::computeDihedralAngle(void)
{
	// m_DihedralAngle
	for (int i = 0; i < _res[0] - 1; i++) {
		for (int j = 0; j < _res[1] - 1; j++) {
			int index0 = (j + 1) * _res[0] + i;
			int index1 = j * _res[0] + (i + 1);
			int index2 = (j + 1) * _res[0] + (i + 1);
			int index3 = j * _res[0] + i;

			auto p0 = _pos[index0];
			auto p1 = _pos[index1];
			auto p2 = _pos[index2];
			auto p3 = _pos[index3];

			vec3 e = p3 - p2;
			double length = e.length();
			if (length < 1e-6) {
				return;
			}
			double invlength = 1.0 / length;
			vec3 n1 = (p2 - p0).cross(p3 - p0);
			vec3 n2 = (p3 - p1).cross(p2 - p1);
			n1 /= n1.lengthSquared();
			n2 /= n2.lengthSquared();

			vec3 d0 = n1 * length;
			vec3 d1 = n2 * length;
			vec3 d2 = n1 * ((p0 - p3).dot(e) * invlength) + n2 * ((p1 - p3).dot(e) * invlength);
			vec3 d3 = n1 * ((p2 - p0).dot(e) * invlength) + n2 * ((p2 - p1).dot(e) * invlength);

			n1.normalize();
			n2.normalize();
			double dot = n1.dot(n2);

			if (dot < -1.0) {
				dot = -1.0;
			}
			if (dot > 1.0) {
				dot = 1.0;
			}
			double restAngle = acos(dot);
			_dihedralAngle.push_back(restAngle);
		}
	}
}*/

void PBD_PlaneCloth::computeRestLength(void)
{
	_structuralLength0Horiz = 2.0 / (double)_res[0];
	_structuralLength0Verti = 2.0 / (double)_res[1];
	_shearLength0Horiz = sqrt(2.0) * _structuralLength0Horiz;
	_shearLength0Verti = sqrt(2.0) * _structuralLength0Verti;
	_bendLength0Horiz = _structuralLength0Horiz * 2.0;
	_bendLength0Verti = _structuralLength0Verti * 2.0;
}

void PBD_PlaneCloth::init(void)
{
	for (int i = 0; i < _res[0]; i++) {
		for (int j = 0; j < _res[1]; j++) {
			int index = j * _res[0] + i;
			_vertices.push_back(new Vertex(index, Vec3<double>(2.0 * i / (double)_res[0], 0.0, 2.0 * j / (double)_res[1]), Vec3<double>(0.0, 0.0, 0.0), 1.0));
		}
	}
}

void PBD_PlaneCloth::solveDistanceConstraint(int index0, int index1, double restlength)
{
	double c_p1p2 = (_vertices[index0]->_pos1 - _vertices[index1]->_pos1).length() - restlength;
	vec3 dp1 = (_vertices[index0]->_pos1 - _vertices[index1]->_pos1);
	vec3 dp2 = (_vertices[index0]->_pos1 - _vertices[index1]->_pos1);
	dp1.normalize();
	dp2.normalize();
	dp1 *= -_vertices[index0]->_invMass / (_vertices[index0]->_invMass + _vertices[index1]->_invMass) * c_p1p2;
	dp2 *= _vertices[index1]->_invMass / (_vertices[index0]->_invMass + _vertices[index1]->_invMass) * c_p1p2;
	_vertices[index0]->_pos1 += dp1;
	_vertices[index1]->_pos1 += dp2;
}

void PBD_PlaneCloth::applyExtForces(double dt)
{
	vec3 gravity(0.0, -9.8, 0.0);
	double damping = 0.99;
	for (int i = 0; i < _res[0]; i++) {
		for (int j = 0; j < _res[1]; j++) {
			int index = j * _res[0] + i;
			_vertices[index]->_vel += gravity * dt * _vertices[index]->_invMass;
			_vertices[index]->_vel *= damping;
			_vertices[index]->_pos1 = _vertices[index]->_pos + (_vertices[index]->_vel * dt);
		}
	}
}

void PBD_PlaneCloth::updateStructuralSprings(void)
{
	for (int i = 0; i < _res[1]; i++) {
		for (int j = 0; j < _res[0] - 1; j++) {
			int index0 = j * _res[0] + i;
			int index1 = (j + 1) * _res[0] + i;
			solveDistanceConstraint(index0, index1, _structuralLength0Horiz);
		}
	}
	for (int i = 0; i < _res[1] - 1; i++) {
		for (int j = 0; j < _res[0]; j++) {
			int index0 = j * _res[0] + i;
			int index1 = j * _res[0] + (i + 1);
			solveDistanceConstraint(index0, index1, _structuralLength0Verti);
		}
	}
}

void PBD_PlaneCloth::updateShearSprings(void)
{
	for (int i = 0; i < _res[1] - 1; i++) {
		for (int j = 0; j < _res[0] - 1; j++) {
			int index0 = j * _res[0] + i;
			int index1 = (j + 1) * _res[0] + (i + 1);
			int index2 = (j + 1) * _res[0] + i;
			int index3 = j * _res[0] + (i + 1);
			solveDistanceConstraint(index0, index1, _shearLength0Horiz);
			solveDistanceConstraint(index2, index3, _shearLength0Verti);
		}
	}
}

void PBD_PlaneCloth::updateBendSprings(void)
{
#ifdef BEND_DIHEDRAL_ANGLE
	for (int i = 0; i < _bendingForce.size(); i++) {
		_bendingForce[i] = 0.0;
	}
	int id = 0;
	for (int i = 0; i < _res[0] - 1; i++) {
		for (int j = 0; j < _res[1] - 1; j++) {
			int index0 = (j + 1) * _res[0] + i;
			int index1 = j * _res[0] + (i + 1);
			int index2 = (j + 1) * _res[0] + (i + 1);
			int index3 = j * _res[0] + i;
			solveDihedralConstraint(index0, index1, index2, index3, _dihedralAngle[id++]);
		}
	}
#else
	for (int i = 0; i < _res[1]; i++) {
		for (int j = 0; j < _res[0] - 2; j++) {
			int index0 = j * _res[0] + i;
			int index1 = (j + 2) * _res[0] + i;
			solveDistanceConstraint(index0, index1, _bendLength0Horiz);
		}
	}
	for (int i = 0; i < _res[1] - 2; i++) {
		for (int j = 0; j < _res[0]; j++) {
			int index0 = j * _res[0] + i;
			int index1 = j * _res[0] + (i + 2);
			solveDistanceConstraint(index0, index1, _bendLength0Verti);
		}
	}
#endif
}

void PBD_PlaneCloth::integrate(double dt)
{
	for (int i = 1; i < _res[1]; i++) {
		for (int j = 0; j < _res[0]; j++) {
			int index = j * _res[0] + i;
			_vertices[index]->_vel = (_vertices[index]->_pos1 - _vertices[index]->_pos) / dt;
			_vertices[index]->_pos = _vertices[index]->_pos1;
		}
	}
}

void PBD_PlaneCloth::simulation(double dt)
{
	applyExtForces(dt);

	int iter = 5;
	for (int k = 0; k < iter; k++) {
		updateStructuralSprings();
		updateShearSprings();
		updateBendSprings();
	}	

	integrate(dt);
}

void PBD_PlaneCloth::computeWindForTriangle(vec3 wind, int index0, int index1, int index2)
{
	auto p0 = _vertices[index0]->_pos1;
	auto p1 = _vertices[index1]->_pos1;
	auto p2 = _vertices[index2]->_pos1;
	auto normal = (p1 - p0).cross(p2 - p0);
	normal.normalize();
	auto force = normal * (normal.dot(wind));
	_vertices[index0]->_vel += force;
	_vertices[index1]->_vel += force;
	_vertices[index2]->_vel += force;
}

/*void PBD_PlaneCloth::solveDihedralConstraint(int index0, int index1, int index2, int index3, double restAngle)
{
	double stiffness = 0.05;
	auto p0 = _vertices[index0]->_pos1;
	auto p1 = _vertices[index1]->_pos1;
	auto p2 = _vertices[index2]->_pos1;
	auto p3 = _vertices[index3]->_pos1;

	vec3 e = p3 - p2;
	double length = e.length();
	if (length < 0.001) {
		return;
	}
	double invlength = 1.0 / length;
	vec3 n1 = (p2 - p0).cross(p3 - p0);
	vec3 n2 = (p3 - p1).cross(p2 - p1);
	n1 /= n1.lengthSquared();
	n2 /= n2.lengthSquared();
	
	vec3 d0 = n1 * length;
	vec3 d1 = n2 * length;
	vec3 d2 = n1 * ((p0 - p3).dot(e) * invlength) + n2 * ((p1 - p3).dot(e) * invlength);
	vec3 d3 = n1 * ((p2 - p0).dot(e) * invlength) + n2 * ((p2 - p1).dot(e) * invlength);

	n1.normalize();
	n2.normalize();
	double dot = n1.dot(n2);
	
	if (dot < -1.0) {
		dot = -1.0;
	}
	if (dot > 1.0) {
		dot = 1.0;
	}
	double phi = acos(dot);

	double lambda = _invMass[index0] * d0.lengthSquared() +
		_invMass[index1] * d1.lengthSquared() +
		_invMass[index2] * d2.lengthSquared() +
		_invMass[index3] * d3.lengthSquared();

	if (lambda == 0.0) {
		return;
	}

	lambda = (phi - restAngle) / lambda * stiffness;

	if (n1.cross(n2).dot(e) > 0.0) {
		lambda = -lambda;
	}

	_pos1[index0] += d0 * (-_invMass[index0] * lambda);
	_pos1[index1] += d1 * (-_invMass[index1] * lambda);
	_pos1[index2] += d2 * (-_invMass[index2] * lambda);
	_pos1[index3] += d3 * (-_invMass[index3] * lambda);

	double visLambda = fabs(lambda*1000000.0);
	_bendingForce[index0] += visLambda;
	_bendingForce[index1] += visLambda;
	_bendingForce[index2] += visLambda;
	_bendingForce[index3] += visLambda;
}*/

void PBD_PlaneCloth::applyWind(vec3 wind)
{
	for (int i = 0; i < _res[0] - 1; i++) {
		for (int j = 0; j < _res[1] - 1; j++) {
			computeWindForTriangle(wind, j * _res[0] + i, (j + 1) * _res[0] + (i + 1), (j+1) * _res[0] + i);
			computeWindForTriangle(wind, j * _res[0] + i, (j + 1) * _res[0] + (i + 1), j * _res[0] + (i + 1));
		}
	}
}

void PBD_PlaneCloth::drawSpring(void)
{
	glDisable(GL_LIGHTING);
	glColor3f(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < _res[1]; i++) {
		for (int j = 0; j < _res[0] - 1; j++) {
			int index0 = j * _res[0] + i;
			int index1 = (j + 1) * _res[0] + i;
			auto p1 = _vertices[index0]->_pos;
			auto p2 = _vertices[index1]->_pos;
			glBegin(GL_LINES);
			glVertex3f(p1.x(), p1.y(), p1.z());
			glVertex3f(p2.x(), p2.y(), p2.z());
			glEnd();
		}
	}

	for (int i = 0; i < _res[1] - 1; i++) {
		for (int j = 0; j < _res[0]; j++) {
			int index0 = j * _res[0] + i;
			int index1 = j * _res[0] + (i + 1);
			auto p1 = _vertices[index0]->_pos;
			auto p2 = _vertices[index1]->_pos;
			glBegin(GL_LINES);
			glVertex3f(p1.x(), p1.y(), p1.z());
			glVertex3f(p2.x(), p2.y(), p2.z());
			glEnd();
		}
	}
	glEnable(GL_LIGHTING);
}

void PBD_PlaneCloth::draw(void)
{
	glDisable(GL_LIGHTING);
	for (int i = 0; i < _res[0] - 1; i++) {
		for (int j = 0; j < _res[1] - 1; j++) {
			auto p00 = pos(i, j);
			auto p10 = pos(i + 1, j);
			auto p11 = pos(i + 1, j + 1);
			auto p01 = pos(i, j + 1);
			int c = ((((i & 0x8) == 0) ^ ((j & 0x8)) == 0)) * 255;
			glColor3f((float)c, (float)c, (float)c);
			glBegin(GL_QUADS);
			glVertex3f(p00.x(), p00.y(), p00.z());
			glVertex3f(p10.x(), p10.y(), p10.z());
			glVertex3f(p11.x(), p11.y(), p11.z());
			glVertex3f(p01.x(), p01.y(), p01.z());
			glEnd();
		}
	}
	drawOutline();
}

void PBD_PlaneCloth::drawOutline(void)
{
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for (int i = 0; i < _res[0] - 1; i++) {
		auto p0 = pos(i, 0);
		auto p1 = pos(i + 1, 0);
		auto p2 = pos(i, _res[1] - 1);
		auto p3 = pos(i + 1, _res[1] - 1);
		auto p4 = pos(0, i);
		auto p5 = pos(0, i + 1);
		auto p6 = pos(_res[1] - 1, i);
		auto p7 = pos(_res[1] - 1, i + 1);
		glVertex3f(p0.x(), p0.y(), p0.z());
		glVertex3f(p1.x(), p1.y(), p1.z());
		glVertex3f(p2.x(), p2.y(), p2.z());
		glVertex3f(p3.x(), p3.y(), p3.z());
		glVertex3f(p4.x(), p4.y(), p4.z());
		glVertex3f(p5.x(), p5.y(), p5.z());
		glVertex3f(p6.x(), p6.y(), p6.z());
		glVertex3f(p7.x(), p7.y(), p7.z());
	}
	glEnd();
	glLineWidth(1.0f);
}