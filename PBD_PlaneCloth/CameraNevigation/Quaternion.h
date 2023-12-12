#ifndef __QUATERNION_H__
#define __QUATERNION_H__
#pragma once

#include "Vec3.h"
#include <math.h>
#include <limits>

class Quaternion
{
public:
    union {
        struct {
            double r;
            double i;
            double j;
            double k;
        };

        double data[4];
    };

    Quaternion() : r(1), i(0), j(0), k(0) {}

    Quaternion(const double r, const double i, const double j, const double k)
        : r(r), i(i), j(j), k(k)
    {
    }

    Quaternion(const vec3& vector)
        : r(0), i(vector._value[0]), j(vector._value[1]), k(vector._value[2])
    {
    }

    void normalise()
    {
        double d = r * r + i * i + j * j + k * k;

        if (d < std::numeric_limits<double>::epsilon()) {
            r = 1;
            return;
        }

        d = ((double)1.0) / sqrt(d);
        r *= d;
        i *= d;
        j *= d;
        k *= d;
    }

    void operator *=(const Quaternion& multiplier)
    {
        Quaternion q = *this;
        r = q.r * multiplier.r - q.i * multiplier.i -
            q.j * multiplier.j - q.k * multiplier.k;
        i = q.r * multiplier.i + q.i * multiplier.r +
            q.j * multiplier.k - q.k * multiplier.j;
        j = q.r * multiplier.j + q.j * multiplier.r +
            q.k * multiplier.i - q.i * multiplier.k;
        k = q.r * multiplier.k + q.k * multiplier.r +
            q.i * multiplier.j - q.j * multiplier.i;
    }

    Quaternion operator *(const Quaternion& multiplier)
    {
        Quaternion q = *this;
        Quaternion res;
        res.r = q.r * multiplier.r - q.i * multiplier.i -
            q.j * multiplier.j - q.k * multiplier.k;
        res.i = q.r * multiplier.i + q.i * multiplier.r +
            q.j * multiplier.k - q.k * multiplier.j;
        res.j = q.r * multiplier.j + q.j * multiplier.r +
            q.k * multiplier.i - q.i * multiplier.k;
        res.k = q.r * multiplier.k + q.k * multiplier.r +
            q.i * multiplier.j - q.j * multiplier.i;
        return res;
    }

    Quaternion operator *(double multiplier)
    {
        Quaternion q = *this;
        q.r *= multiplier;
        q.i *= multiplier;
        q.j *= multiplier;
        q.k *= multiplier;
        return q;
    }

    void operator +=(Quaternion& add)
    {
        r += add.r;
        i += add.i;
        j += add.j;
        k += add.k;
    }

    void operator /=(double div)
    {
        r /= div;
        i /= div;
        j /= div;
        k /= div;
    }

    void addScaledVector(const vec3& vector, double scale)
    {
        Quaternion q(0,
            vector._value[0] * scale,
            vector._value[1] * scale,
            vector._value[2] * scale);
        q *= *this;
        r += q.r * ((double)0.5);
        i += q.i * ((double)0.5);
        j += q.j * ((double)0.5);
        k += q.k * ((double)0.5);
    }

    void rotateByVector(const Vec3<double>& vector)
    {
        Quaternion q(0.0, vector._value[0], vector._value[1], vector._value[2]);
        (*this) *= q;
    }
};

#endif