#include "PBD_ObjectCloth.h"
#include "GL\glut.h"
#include <Windows.h>
#include <stdio.h>
#include <iostream>

PBD_ObjectCloth::PBD_ObjectCloth()
{
}

PBD_ObjectCloth::~PBD_ObjectCloth()
{
}

void PBD_ObjectCloth::reset(void) 
{
}

void PBD_ObjectCloth::loadObj(char* filename)
{
	FILE* fp;
	char header[256] = { 0 };
	double pos[3];
	int v_index[3];
	int index = 0;

	_minBoundary.set(100000.0);
	_maxBoundary.set(-100000.0);

	fopen_s(&fp, filename, "r");
	while (fscanf(fp, "%s %lf %lf %lf", header, &pos[0], &pos[1], &pos[2]) != EOF) {
		if (header[0] == 'v' && header[1] == NULL) {
			if (_minBoundary[0] > pos[0])	_minBoundary[0] = pos[0];
			if (_minBoundary[1] > pos[1])	_minBoundary[1] = pos[1];
			if (_minBoundary[2] > pos[2])	_minBoundary[2] = pos[2];
			if (_maxBoundary[0] < pos[0])	_maxBoundary[0] = pos[0];
			if (_maxBoundary[1] < pos[1])	_maxBoundary[1] = pos[1];
			if (_maxBoundary[2] < pos[2])	_maxBoundary[2] = pos[2];
			_vertices.push_back(new Vertex(index++, Vec3<double>(pos[0], pos[1], pos[2]), Vec3<double>(0.0, 0.0, 0.0), 1.0));
		}
	}
	printf("num. vertices : %d\n", _vertices.size());
	printf("min boundary : %f, %f, %f\n", _minBoundary.x(), _minBoundary.y(), _minBoundary.z());
	printf("max boundary : %f, %f, %f\n", _maxBoundary.x(), _maxBoundary.y(), _maxBoundary.z());

	index = 0;
	fseek(fp, 0, SEEK_SET);
	while (fscanf(fp, "%s %d %d %d", header, &v_index[0], &v_index[1], &v_index[2]) != EOF) {
		if (header[0] == 'f' && header[1] == NULL) {
			auto v0 = _vertices[v_index[0] - 1];
			auto v1 = _vertices[v_index[1] - 1];
			auto v2 = _vertices[v_index[2] - 1];
			_faces.push_back(new Face(index++, v0, v1, v2));
		}
	}
	printf("num. faces : %d\n", _faces.size());
	moveToCenter(3.0);
	buildAdjacency();
	//selectFixVertex();
	computeRestLength();
	computeRestVolume();
	computeNormal();
	computeInverseTensor();
	fclose(fp);
}

void PBD_ObjectCloth::moveToCenter(double scale)
{
	double max_length = fmax(fmax(_maxBoundary[0] - _minBoundary[0], _maxBoundary[1] - _minBoundary[1]), _maxBoundary[2] - _minBoundary[2]);
	auto center = (_maxBoundary + _minBoundary) / 2.0;
	Vec3<double> new_center(0.0, 0.0, 0.0);

	for (auto v : _vertices) {
		auto pos = v->_pos;
		auto grad = pos - center;
		grad /= max_length;
		grad *= scale;
		pos = new_center;
		pos += grad;
		v->_pos = pos;
	}
	printf("move to center\n");
}

void PBD_ObjectCloth::buildAdjacency(void)
{
	for (auto v : _vertices) {
		v->_nbFaces.clear();
		v->_nbVertices.clear();
	}

	// v-f
	for (auto f : _faces) {
		for (int j = 0; j < 3; j++) {
			f->_vertices[j]->_nbFaces.push_back(f);
		}
	}

	// v-v
	for (auto v : _vertices) {
		for (auto nf : v->_nbFaces) {
			auto pivot = nf->getIndex(v); // 0 : 1,2, 1 : 2,0, 2: 0,1
			int other_id0 = (pivot + 1) % 3;
			int other_id1 = (pivot + 2) % 3;
			if (!v->hasNbVertex(nf->_vertices[other_id0])) {
				v->_nbVertices.push_back(nf->_vertices[other_id0]);
			}
			if (!v->hasNbVertex(nf->_vertices[other_id1])) {
				v->_nbVertices.push_back(nf->_vertices[other_id1]);
			}
		}
	}
}

void PBD_ObjectCloth::selectFixVertex(void) {
	double centerY = 0.0;

	for (auto v : _vertices) {
		centerY += v->y();
	}
	centerY /= (double)_vertices.size();
	for (auto v : _vertices) {
		if (v->_index < 100) {
			_fixIndex.push_back(v->_index);
		}
	}
}

void PBD_ObjectCloth::computeRestLength(void)
{
	for (auto f : _faces) {
		//init structural Length
		for (int i = 0; i < 3; i++) {
			f->_structuralLength.push_back((f->_vertices[i]->_pos - f->_vertices[(i + 1) % 3]->_pos).getNorm());
		}

		//init bend Length
		for (int i = 0; i < 3; i++) {
			int id0 = (i + 1) % 3;
			int id1 = (i + 2) % 3;
			for (auto nv : f->_vertices[id0]->_nbVertices) {
				if (f->_vertices[id1]->hasNbVertex(nv) && nv != f->_vertices[i]) {
					f->_bendLength.push_back((f->_vertices[i]->_pos - nv->_pos).getNorm());
					break;
				}
			}
		}
	}
}

void PBD_ObjectCloth::computeRestVolume(void)
{
	_restVolume = 0.0;

	for (auto f : _faces) {
		_restVolume += f->_vertices[0]->_pos.cross(f->_vertices[1]->_pos).dot(f->_vertices[2]->_pos);
	}
}

void	PBD_ObjectCloth::computeNormal(void) {
	// f-normal
	for (auto f : _faces) {
		auto a = f->_vertices[0]->_pos;
		auto b = f->_vertices[1]->_pos;
		auto c = f->_vertices[2]->_pos;
		auto normal = (a - b).cross(a - c);
		normal.normalize();
		f->_normal = normal;
	}

	// v-normal
	for (auto v : _vertices) {
		v->_normal.clear();
		for (auto nf : v->_nbFaces) {
			v->_normal += nf->_normal;
		}
		v->_normal /= v->_nbFaces.size();
	}
}

void	PBD_ObjectCloth::computeBernoulliConst(void) {
	_pressure = ((_addVolume + 20) * 4 * 2077 * 294.15) / _restVolume;

	_bernoulliConst = _pressure + 1.205 * 9.8 * abs(_vertices[_vertices.size() / 5]->y());
}

void	PBD_ObjectCloth::computeInverseTensor(void) {
	double ix = 0.0, iy = 0.0, iz = 0.0, ixy = 0.0, ixz = 0.0, iyz = 0.0;

	vec3 massCenter(0.0, 0.0, 0.0);
	Quaternion orientation(0.0, 0.0, 0.0, 0.0);
	for (auto v : _vertices)
	{
		massCenter += v->_pos;
		orientation += v->_orientation;
	}
	massCenter /= _vertices.size();
	orientation /= _vertices.size();

	for (auto v : _vertices)
	{
		ix += (1 / v->_invMass) * ((v->_pos - massCenter).dot(vec3(1, 0, 0))) * ((v->_pos - massCenter).dot(vec3(1, 0, 0)));
		iy += (1 / v->_invMass) * ((v->_pos - massCenter).dot(vec3(0, 1, 0))) * ((v->_pos - massCenter).dot(vec3(0, 1, 0)));
		iz += (1 / v->_invMass) * ((v->_pos - massCenter).dot(vec3(0, 0, 1))) * ((v->_pos - massCenter).dot(vec3(0, 0, 1)));
		ixy += (1 / v->_invMass) * ((v->_pos - massCenter).dot(vec3(1, 0, 0))) * ((v->_pos - massCenter).dot(vec3(0, 1, 0)));
		ixz += (1 / v->_invMass) * ((v->_pos - massCenter).dot(vec3(1, 0, 0))) * ((v->_pos - massCenter).dot(vec3(0, 0, 1)));
		iyz += (1 / v->_invMass) * ((v->_pos - massCenter).dot(vec3(0, 1, 0))) * ((v->_pos - massCenter).dot(vec3(0, 0, 1)));
	}

	_inverseTensor.setInertiaTensorCoeffs(ix, iy, iz, ixy, ixz, iyz);
	_inverseTensor.invert();

	orientation.normalise();
	Matrix4 transformMatrix;
	CalculateTransformMatrix(transformMatrix, massCenter, orientation);
	TransformInertiaTensor(_inverseTensorWorld, orientation, _inverseTensor, transformMatrix);
}

void PBD_ObjectCloth::solveDistanceConstraint(int index0, int index1, double restlength)
{
	double c_p1p2 = (_vertices[index0]->_pos1 - _vertices[index1]->_pos1).length() - restlength;
	vec3 dp1 = (_vertices[index0]->_pos1 - _vertices[index1]->_pos1);
	vec3 dp2 = (_vertices[index0]->_pos1 - _vertices[index1]->_pos1);
	dp1.normalize();
	dp2.normalize();
	dp1 *= -_vertices[index0]->_invMass / (_vertices[index0]->_invMass + _vertices[index1]->_invMass) * c_p1p2;
	dp2 *= _vertices[index1]->_invMass / (_vertices[index0]->_invMass + _vertices[index1]->_invMass) * c_p1p2;
	dp1 *= 1.0 - pow((1.0 - _springK), 1.0 / _iteration);
	dp2 *= 1.0 - pow((1.0 - _springK), 1.0 / _iteration);
	_vertices[index0]->_pos1 += dp1;
	_vertices[index1]->_pos1 += dp2;
}

void PBD_ObjectCloth::solvePressureConstraint(double restVolume)
{
	double c = -1.0 * restVolume * _volumeK;

	for (auto f : _faces) {
		c += f->_vertices[0]->_pos1.cross(f->_vertices[1]->_pos1).dot(f->_vertices[2]->_pos1);
	}

	for (auto v : _vertices) {
		v->_volGradC = vec3(0.0, 0.0, 0.0);
		for (auto nf : v->_nbFaces) {
			int idx = nf->getIndex(v);
			int id0 = (idx + 1) % 3;
			int id1 = (idx + 2) % 3;
			v->_volGradC += nf->_vertices[id0]->_pos1.cross(nf->_vertices[id1]->_pos1);
		}
	}

	double s = 0;
	for (auto v : _vertices) {
		s += v->_invMass * pow(v->_volGradC.getNorm(), 2.0);
	}

	s = c / s;

	for (auto v : _vertices) {
		vec3 dp = v->_volGradC * s * -1.0;
		v->_pos1 += dp * v->_invMass;
	}
}

void PBD_ObjectCloth::applyExtForces(double dt)
{
	double volume = 0.0;
	double area = 0.0;
	vec3 reaction(0.0, 0.0, 0.0);

	for (auto f : _faces) {
		volume += f->_vertices[0]->_pos.cross(f->_vertices[1]->_pos).dot(f->_vertices[2]->_pos);
		auto a = f->_vertices[0]->_pos;
		auto b = f->_vertices[1]->_pos;
		auto c = f->_vertices[2]->_pos;
		auto normal = (a - b).cross(a - c);
		area += normal.getNorm() / 2.0;
	}

	if (_isAirRelease) {
		reaction = sqrt(abs((_bernoulliConst - _pressure - (1.205 * 9.8 * abs(_vertices[_vertices.size() / 5]->y()))) * 2 / 1.205));
		reaction *= _vertices[_vertices.size() / 5]->_normal * -0.03;
	}

	vec3 gravity(0.0, -9.8, 0.0);
	vec3 buoyancy = gravity * volume * (0.02) * -1.0;
	double linearDamping = 0.99;

	vec3 massCenter(0.0, 0.0, 0.0);
	for (auto v : _vertices) {
		massCenter += v->_pos;
	}
	massCenter /= _vertices.size();

	//printf("mass Center : (%f, %f, %f)\n", massCenter.x(), massCenter.y(), massCenter.z());

	vec3 torque = (_vertices[_vertices.size() / 5]->_pos - massCenter).cross(reaction);
	double angularDamping = 0.99;

	for (auto v : _vertices) {
		vec3 airDrag = v->_vel * -0.5 * 0.5 * 1.205 * area;
		double dot = v->_vel.dot(v->_normal) / v->_vel.getNorm();
		airDrag *= dot >= 0 ? dot : 0;

		if (_isAirRelease) {
			v->_vel += reaction * dt;
		}
		v->_vel += gravity * dt;
		v->_vel += buoyancy * dt * v->_invMass;
		v->_vel += airDrag * dt * v->_invMass;
		v->_vel *= linearDamping;
		v->_pos1 = v->_pos + (v->_vel * dt);

		v->_angVel += _inverseTensorWorld * torque * dt;
		v->_angVel *= angularDamping;
		v->_orientation += Quaternion(v->_angVel) * v->_orientation * dt * 0.5;
		v->_orientation.normalise();
		Matrix3 rotateMatrix;
		rotateMatrix.setOrientation(v->_orientation);
		v->_pos1 = rotateMatrix * v->_pos1;
	}
}

void PBD_ObjectCloth::TransformInertiaTensor(Matrix3& iitWorld, const Quaternion& q, const Matrix3& iitBody, const Matrix4& rotmat)
{
	double t4 = rotmat.data[0] * iitBody.data[0] +
		rotmat.data[1] * iitBody.data[3] +
		rotmat.data[2] * iitBody.data[6];
	double t9 = rotmat.data[0] * iitBody.data[1] +
		rotmat.data[1] * iitBody.data[4] +
		rotmat.data[2] * iitBody.data[7];
	double t14 = rotmat.data[0] * iitBody.data[2] +
		rotmat.data[1] * iitBody.data[5] +
		rotmat.data[2] * iitBody.data[8];
	double t28 = rotmat.data[4] * iitBody.data[0] +
		rotmat.data[5] * iitBody.data[3] +
		rotmat.data[6] * iitBody.data[6];
	double t33 = rotmat.data[4] * iitBody.data[1] +
		rotmat.data[5] * iitBody.data[4] +
		rotmat.data[6] * iitBody.data[7];
	double t38 = rotmat.data[4] * iitBody.data[2] +
		rotmat.data[5] * iitBody.data[5] +
		rotmat.data[6] * iitBody.data[8];
	double t52 = rotmat.data[8] * iitBody.data[0] +
		rotmat.data[9] * iitBody.data[3] +
		rotmat.data[10] * iitBody.data[6];
	double t57 = rotmat.data[8] * iitBody.data[1] +
		rotmat.data[9] * iitBody.data[4] +
		rotmat.data[10] * iitBody.data[7];
	double t62 = rotmat.data[8] * iitBody.data[2] +
		rotmat.data[9] * iitBody.data[5] +
		rotmat.data[10] * iitBody.data[8];

	iitWorld.data[0] = t4 * rotmat.data[0] +
		t9 * rotmat.data[1] +
		t14 * rotmat.data[2];
	iitWorld.data[1] = t4 * rotmat.data[4] +
		t9 * rotmat.data[5] +
		t14 * rotmat.data[6];
	iitWorld.data[2] = t4 * rotmat.data[8] +
		t9 * rotmat.data[9] +
		t14 * rotmat.data[10];
	iitWorld.data[3] = t28 * rotmat.data[0] +
		t33 * rotmat.data[1] +
		t38 * rotmat.data[2];
	iitWorld.data[4] = t28 * rotmat.data[4] +
		t33 * rotmat.data[5] +
		t38 * rotmat.data[6];
	iitWorld.data[5] = t28 * rotmat.data[8] +
		t33 * rotmat.data[9] +
		t38 * rotmat.data[10];
	iitWorld.data[6] = t52 * rotmat.data[0] +
		t57 * rotmat.data[1] +
		t62 * rotmat.data[2];
	iitWorld.data[7] = t52 * rotmat.data[4] +
		t57 * rotmat.data[5] +
		t62 * rotmat.data[6];
	iitWorld.data[8] = t52 * rotmat.data[8] +
		t57 * rotmat.data[9] +
		t62 * rotmat.data[10];
}

void PBD_ObjectCloth::CalculateTransformMatrix(Matrix4& transformMatrix, const vec3& position, const Quaternion& orientation)
{
	transformMatrix.data[0] = 1 - 2 * orientation.j * orientation.j -
		2 * orientation.k * orientation.k;
	transformMatrix.data[1] = 2 * orientation.i * orientation.j -
		2 * orientation.r * orientation.k;
	transformMatrix.data[2] = 2 * orientation.i * orientation.k +
		2 * orientation.r * orientation.j;
	transformMatrix.data[3] = position._value[0];

	transformMatrix.data[4] = 2 * orientation.i * orientation.j +
		2 * orientation.r * orientation.k;
	transformMatrix.data[5] = 1 - 2 * orientation.i * orientation.i -
		2 * orientation.k * orientation.k;
	transformMatrix.data[6] = 2 * orientation.j * orientation.k -
		2 * orientation.r * orientation.i;
	transformMatrix.data[7] = position._value[1];

	transformMatrix.data[8] = 2 * orientation.i * orientation.k -
		2 * orientation.r * orientation.j;
	transformMatrix.data[9] = 2 * orientation.j * orientation.k +
		2 * orientation.r * orientation.i;
	transformMatrix.data[10] = 1 - 2 * orientation.i * orientation.i -
		2 * orientation.j * orientation.j;
	transformMatrix.data[11] = position._value[2];
}

void PBD_ObjectCloth::updateStructuralSprings(void)
{
	for (auto f : _faces) {
		for (int i = 0; i < 3; i++) {
			int id0 = (i + 1) % 3;
			solveDistanceConstraint(f->_vertices[i]->_index, f->_vertices[id0]->_index, f->_structuralLength[i]);
		}
	}
}

void PBD_ObjectCloth::updateBendSprings(void)
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
	for (auto f : _faces) {
		for (int i = 0; i < 3; i++) {
			int id0 = (i + 1) % 3;
			int id1 = (i + 2) % 3;
			for (auto nv : f->_vertices[id0]->_nbVertices) {
				if (f->_vertices[id1]->hasNbVertex(nv) && nv != f->_vertices[i]) {
					solveDistanceConstraint(f->_vertices[i]->_index, nv->_index, f->_bendLength[i]);
					break;
				}
			}
		}
	}
#endif
}

void PBD_ObjectCloth::updateMass(void) {
	double volume = 0.0;

	for (auto f : _faces) {
		volume += f->_vertices[0]->_pos.cross(f->_vertices[1]->_pos).dot(f->_vertices[2]->_pos);
	}

	double mass = volume * 1.205;

	for (auto v : _vertices) {
		v->_invMass = 1.0 / (1.0 + (mass / _vertices.size()));
	}
}

void PBD_ObjectCloth::updatePressure(void) {
	double volume = 0.0;

	for (auto f : _faces) {
		volume += f->_vertices[0]->_pos.cross(f->_vertices[1]->_pos).dot(f->_vertices[2]->_pos);
	}

	_pressure = ((_addVolume + 20) * 4 * 2077 * 294.15) / volume;
}

void PBD_ObjectCloth::updateNormal(void) {
	for (auto f : _faces) {
		auto a = f->_vertices[0]->_pos;
		auto b = f->_vertices[1]->_pos;
		auto c = f->_vertices[2]->_pos;
		auto normal = (a - b).cross(a - c);
		normal.normalize();
		f->_normal = normal;
	}

	// v-normal
	for (auto v : _vertices) {
		v->_normal.clear();
		for (auto nf : v->_nbFaces) {
			v->_normal += nf->_normal;
		}
		v->_normal /= v->_nbFaces.size();
	}
}

void PBD_ObjectCloth::integrate(double dt)
{
	for (int i = 0; i < _vertices.size();i++) {
		bool fix = false;

		for (auto j : _fixIndex) {
			if (_vertices[i] == _vertices[j]) {
				fix = true;
				break;
			}
		}

		if (fix)
			continue;

		_vertices[i]->_vel = (_vertices[i]->_pos1 - _vertices[i]->_pos) / dt;
		_vertices[i]->_pos = _vertices[i]->_pos1;
	}
}

void PBD_ObjectCloth::simulation(double dt)
{
	updateNormal();
	updateMass();
	updatePressure();
	computeInverseTensor();
	applyExtForces(dt);
	applyAirRelease();

	for (int k = 0; k < _iteration; k++) {
		updateStructuralSprings();
		updateBendSprings();
		solvePressureConstraint(_restVolume + _addVolume);
	}

	integrate(dt);
}

void PBD_ObjectCloth::computeWindForTriangle(vec3 wind, Face* f)
{
	auto p0 = f->_vertices[0]->_pos1;
	auto p1 = f->_vertices[1]->_pos1;
	auto p2 = f->_vertices[2]->_pos1;
	auto normal = (p1 - p0).cross(p2 - p0);
	normal.normalize();
	auto force = normal * (normal.dot(wind));
	f->_vertices[0]->_vel += force;
	f->_vertices[0]->_vel += force;
	f->_vertices[0]->_vel += force;
}

void PBD_ObjectCloth::applyWind(vec3 wind)
{
	for (auto f : _faces) {
		computeWindForTriangle(wind, f);
	}
}

void	PBD_ObjectCloth::applyBallon(void) {
	_isAirRelease = false;
	_addVolume += 1.0;
}

void	PBD_ObjectCloth::onAirRelease(void) {
	_isAirRelease = !_isAirRelease;
}

void	PBD_ObjectCloth::applyAirRelease(void) {
	if (!_isAirRelease) return;

	if (_pressure < 1013 * 1000) {
		_isAirRelease = false;
		return;
	}

	_addVolume -= _pressure / 1013000;
}

void PBD_ObjectCloth::drawPoint(void)
{
	glDisable(GL_LIGHTING);
	glPointSize(2.0f);
	glBegin(GL_POINTS);
	for (auto v : _vertices) {
		glVertex3f(v->x(), v->y(), v->z());
	}
	glEnd();
	glPointSize(1.0f);
	glEnable(GL_LIGHTING);
}

void PBD_ObjectCloth::drawSolid(void)
{
	glEnable(GL_LIGHTING);
	for (auto f : _faces) {
		glBegin(GL_POLYGON);
		glNormal3f(f->_normal.x(), f->_normal.y(), f->_normal.z());
		for (int j = 0; j < 3; j++) {
			glVertex3f(f->_vertices[j]->x(), f->_vertices[j]->y(), f->_vertices[j]->z());
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);
}

void PBD_ObjectCloth::drawWire(void)
{
	glDisable(GL_LIGHTING);
	glColor3f(1.0f, 1.0f, 1.0f);
	for (auto f : _faces) {
		glBegin(GL_LINES);
		for (int j = 0; j < 3; j++) {
			int s = j % 3; // 0,1,2
			int e = (j + 1) % 3; // 0,1,2
			glVertex3f(f->_vertices[s]->x(), f->_vertices[s]->y(), f->_vertices[s]->z());
			glVertex3f(f->_vertices[e]->x(), f->_vertices[e]->y(), f->_vertices[e]->z());
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);
}