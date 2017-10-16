#ifndef __CORE_H_INCLUDED__
#define __CORE_H_INCLUDED__


#include <iostream>
#include "precision.h"

class Vector2
{
public:
	real x;
	real y;

public:
	const static Vector2 X;
	const static Vector2 Y;
	const static Vector2 ORIGIN;

public:
	Vector2();
	Vector2(real x, real y);

	void print() const;

	void invert();
	real magnitude() const;
	real squareMagnitude() const;
	void normalize();
	Vector2 unit() const;

	Vector2 operator*(real a) const;
	void scale(real a);
	Vector2 operator+(const Vector2& v) const;
	void add(const Vector2& v);
	Vector2 operator-(const Vector2& v) const;
	Vector2 operator-() const;
	void minus(const Vector2& v);
	void addScaledVector(const Vector2& v, real a);
	
	Vector2 componentProduct(const Vector2 &v) const;
	void componentProductUpdate(const Vector2 &v);
	real dotProduct(const Vector2 &v) const;
	real operator*(const Vector2 &v) const; // dot product *
	real crossProduct(const Vector2 &v) const;
	Vector2 crossProduct(real z) const; // cross with vector(0, 0, z)

	void clear();
	void rotate(real angle);
	Vector2 normal() const; // rotate 90 degrees
};

// 2x2 matrix
class Matrix2
{
public:
	real data[4]; // ordered from left to right then up to down

public:
	Matrix2();
	Matrix2(real c0, real c1, real c2, real c3);
	
	void print() const;

	Matrix2 operator*(real s) const;
	Vector2 operator*(const Vector2 &v) const;
	Matrix2 operator*(const Matrix2 &m) const;
	Matrix2 operator+(const Matrix2 &m) const;

	real determinant() const;
	void setInverse(const Matrix2 &m);
	Matrix2 inverse() const;
	void invert();

	void setTranspose(const Matrix2 &m);
	Matrix2 transpose() const;
	
	void setOrientation(const Vector2 &v);
	void setComponents(const Vector2 &a, const Vector2 &b);
	//void setSkeySymmetric(const Vector2 &v);
};

// 2x3 matrix
// 3x3 if assuming last row is (0, 0, 1)
class Matrix3
{
public:
	real data[6];

public:
	Matrix3();
	Matrix3(real c0, real c1, real c2, real c3, real c4, real c5);

	void print() const;

	// homogeneous coordinates, extending vector by a 1
	Vector2 operator*(const Vector2 &v) const;
	Matrix3 operator*(const Matrix3 &m) const;

	real determinant() const;
	void setInverse(const Matrix3 &m);
	Matrix3 inverse() const;
	void invert();

	// assuming rotation matrix with translation
	void setOrientationAndPos(const Vector2 &ori, const Vector2 &pos);
	Vector2 transformInverse(const Vector2 &v) const;
	Vector2 transformDirection(const Vector2 &v) const;
	Vector2 transformInverseDirection(const Vector2 &v) const;

	Vector2 getAxis(int i) const;
	void fillGLMatrix(float m[16]) const;
};

#endif // __CORE_H_INCLUDED__