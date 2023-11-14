#ifndef __MATRIX3_H__
#define __MATRIX3_H__
#pragma once

#include "Vec3.h"
#include "Quaternion.h"

class Matrix3
{
public:
    double data[9];

    Matrix3()
    {
        data[0] = data[1] = data[2] = data[3] = data[4] = data[5] =
            data[6] = data[7] = data[8] = 0;
    }

    Matrix3(const vec3& compOne, const vec3& compTwo,
        const vec3& compThree)
    {
        setComponents(compOne, compTwo, compThree);
    }

    Matrix3(double c0, double c1, double c2, double c3, double c4, double c5,
        double c6, double c7, double c8)
    {
        data[0] = c0; data[1] = c1; data[2] = c2;
        data[3] = c3; data[4] = c4; data[5] = c5;
        data[6] = c6; data[7] = c7; data[8] = c8;
    }

    void setDiagonal(double a, double b, double c)
    {
        setInertiaTensorCoeffs(a, b, c);
    }

    void setInertiaTensorCoeffs(double ix, double iy, double iz,
        double ixy = 0, double ixz = 0, double iyz = 0)
    {
        data[0] = ix;
        data[1] = data[3] = -ixy;
        data[2] = data[6] = -ixz;
        data[4] = iy;
        data[5] = data[7] = -iyz;
        data[8] = iz;
    }

    void setBlockInertiaTensor(vec3& halfSizes, double mass)
    {
        vec3 squares = halfSizes.dot(halfSizes);
        setInertiaTensorCoeffs(0.3f * mass * (squares.y() + squares.z()),
            0.3f * mass * (squares.x() + squares.z()),
            0.3f * mass * (squares.x() + squares.y()));
    }

    void setSkewSymmetric(const vec3 vector)
    {
        data[0] = data[4] = data[8] = 0;
        data[1] = -vector._value[2];
        data[2] = vector._value[1];
        data[3] = vector._value[2];
        data[5] = -vector._value[0];
        data[6] = -vector._value[1];
        data[7] = vector._value[0];
    }

    void setComponents(const vec3& compOne, const vec3& compTwo,
        const vec3& compThree)
    {
        data[0] = compOne._value[0];
        data[1] = compTwo._value[0];
        data[2] = compThree._value[0];
        data[3] = compOne._value[1];
        data[4] = compTwo._value[1];
        data[5] = compThree._value[1];
        data[6] = compOne._value[2];
        data[7] = compTwo._value[2];
        data[8] = compThree._value[2];

    }

    vec3 operator*(const vec3& vector) const
    {
        return vec3(
            vector._value[0] * data[0] + vector._value[1] * data[1] + vector._value[2] * data[2],
            vector._value[0] * data[3] + vector._value[1] * data[4] + vector._value[2] * data[5],
            vector._value[0] * data[6] + vector._value[1] * data[7] + vector._value[2] * data[8]
        );
    }

    vec3 transform(const vec3& vector) const
    {
        return (*this) * vector;
    }

    vec3 transformTranspose(const vec3& vector) const
    {
        return vec3(
            vector._value[0] * data[0] + vector._value[1] * data[3] + vector._value[2] * data[6],
            vector._value[0] * data[1] + vector._value[1] * data[4] + vector._value[2] * data[7],
            vector._value[0] * data[2] + vector._value[1] * data[5] + vector._value[2] * data[8]
        );
    }

    vec3 getRowVector(int i) const
    {
        return vec3(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
    }

    vec3 getAxisVector(int i) const
    {
        return vec3(data[i], data[i + 3], data[i + 6]);
    }

    void setInverse(const Matrix3& m)
    {
        double t4 = m.data[0] * m.data[4];
        double t6 = m.data[0] * m.data[5];
        double t8 = m.data[1] * m.data[3];
        double t10 = m.data[2] * m.data[3];
        double t12 = m.data[1] * m.data[6];
        double t14 = m.data[2] * m.data[6];

        // Calculate the determinant
        double t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
            t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

        // Make sure the determinant is non-zero.
        if (t16 == (double)0.0f) return;
        double t17 = 1 / t16;

        data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
        data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
        data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
        data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
        data[4] = (m.data[0] * m.data[8] - t14) * t17;
        data[5] = -(t6 - t10) * t17;
        data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
        data[7] = -(m.data[0] * m.data[7] - t12) * t17;
        data[8] = (t4 - t8) * t17;
    }

    Matrix3 inverse() const
    {
        Matrix3 result;
        result.setInverse(*this);
        return result;
    }

    void invert()
    {
        setInverse(*this);
    }

    void setTranspose(const Matrix3& m)
    {
        data[0] = m.data[0];
        data[1] = m.data[3];
        data[2] = m.data[6];
        data[3] = m.data[1];
        data[4] = m.data[4];
        data[5] = m.data[7];
        data[6] = m.data[2];
        data[7] = m.data[5];
        data[8] = m.data[8];
    }

    Matrix3 transpose() const
    {
        Matrix3 result;
        result.setTranspose(*this);
        return result;
    }

    Matrix3 operator*(const Matrix3& o) const
    {
        return Matrix3(
            data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
            data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
            data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

            data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
            data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
            data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

            data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
            data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
            data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]
        );
    }

    void operator*=(const Matrix3& o)
    {
        double t1;
        double t2;
        double t3;

        t1 = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
        t2 = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
        t3 = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];
        data[0] = t1;
        data[1] = t2;
        data[2] = t3;

        t1 = data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
        t2 = data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
        t3 = data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];
        data[3] = t1;
        data[4] = t2;
        data[5] = t3;

        t1 = data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
        t2 = data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
        t3 = data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];
        data[6] = t1;
        data[7] = t2;
        data[8] = t3;
    }

    void operator*=(const double scalar)
    {
        data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
        data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
        data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
    }

    void operator+=(const Matrix3& o)
    {
        data[0] += o.data[0]; data[1] += o.data[1]; data[2] += o.data[2];
        data[3] += o.data[3]; data[4] += o.data[4]; data[5] += o.data[5];
        data[6] += o.data[6]; data[7] += o.data[7]; data[8] += o.data[8];
    }

    void setOrientation(const Quaternion& q)
    {
        data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
        data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
        data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
        data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
        data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
        data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
        data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
        data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
        data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
    }

    static Matrix3 linearInterpolate(const Matrix3& a, const Matrix3& b, double prop) {
        Matrix3 result;
        for (unsigned i = 0; i < 9; i++) {
            result.data[i] = a.data[i] * (1 - prop) + b.data[i] * prop;
        }
        return result;
    }
};

#endif