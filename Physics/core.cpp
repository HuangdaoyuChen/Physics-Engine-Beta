#include "core.h"

const Vector2 Vector2::X = Vector2(1, 0);
const Vector2 Vector2::Y = Vector2(0, 1);
const Vector2 Vector2::ORIGIN = Vector2(0, 0);

Vector2::Vector2()
{
	x = 0;
	y = 0;
}

Vector2::Vector2(real x, real y)
{
	Vector2::x = x;
	Vector2::y = y;
}

void Vector2::print() const
{
	printf("< %f , %f >", x, y);
}

void Vector2::invert()
{
	x = -x;
	y = -y;
}

real Vector2::magnitude() const
{
	return real_sqrt(x*x + y*y);
}

real Vector2::squareMagnitude() const
{
	return x*x + y*y;
}

void Vector2::normalize()
{
	real m = magnitude();
	if (m > 0)
		scale((real)1.0 / m);
}

Vector2 Vector2::unit() const
{
	real m = magnitude();
	if (m <= 0)
		return Vector2(0, 0);
	return Vector2(x / m, y / m);
}

Vector2 Vector2::operator*(real a) const
{
	return Vector2(x * a, y * a);
}

void Vector2::scale(real a)
{
	x = x * a;
	y = y * a;
}

Vector2 Vector2::operator+(const Vector2& v) const
{
	return Vector2(x + v.x, y + v.y);
}

void Vector2::add(const Vector2& v)
{
	x = x + v.x;
	y = y + v.y;
}

Vector2 Vector2::operator-(const Vector2& v) const
{
	return Vector2(x - v.x, y - v.y);
}

Vector2 Vector2::operator-() const
{
	return Vector2(-x, -y);
}

void Vector2::minus(const Vector2& v)
{
	x = x - v.x;
	y = y - v.y;
}

void Vector2::addScaledVector(const Vector2& v, real a)
{
	x = x + v.x * a;
	y = y + v.y * a;
}

Vector2 Vector2::componentProduct(const Vector2 &v) const
{
	return Vector2(x * v.x, y * v.y);
}

void Vector2::componentProductUpdate(const Vector2 &v)
{
	x = x * v.x;
	y = y * v.y;
}

real Vector2::dotProduct(const Vector2 &v) const
{
	return x * v.x + y * v.y;
}

real Vector2::operator*(const Vector2 &v) const
{
	return x * v.x + y * v.y;
}

real Vector2::crossProduct(const Vector2 &v) const
{
	return x * v.y - y * v.x;
}

Vector2 Vector2::crossProduct(real z) const
{
	return Vector2(y * z, -x * z);
}

void Vector2::rotate(real angle)
{
	real x2 = x * cos(angle) - y * sin(angle);
	real y2 = x * sin(angle) + y * cos(angle);
	x = x2;
	y = y2;
}

void Vector2::clear()
{
	x = 0;
	y = 0;
}

Vector2 Vector2::normal() const
{
	return Vector2(-y, x);
}

Matrix2::Matrix2()
{
	data[0] = 1;	data[1] = 0;
	data[2] = 0;	data[3] = 1;
}

Matrix2::Matrix2(real c0, real c1, real c2, real c3)
{
	data[0] = c0;	data[1] = c1;
	data[2] = c2;	data[3] = c3;
}

void Matrix2::print() const
{
	std::cout << "{ {" << data[0] << ", " << data[1] << "}, {" 
					   << data[2] << ", " << data[3] << "} }";
}

Matrix2 Matrix2::operator*(real s) const
{
	real c0 = data[0] * s;
	real c1 = data[1] * s;
	real c2 = data[2] * s;
	real c3 = data[3] * s;
	return Matrix2(c0, c1, c2, c3);
}

Vector2 Matrix2::operator*(const Vector2 &v) const
{
	real c0 = data[0] * v.x + data[1] * v.y;
	real c1 = data[2] * v.x + data[3] * v.y;
	return Vector2(c0, c1);
}

Matrix2 Matrix2::operator*(const Matrix2 &m) const
{
	real c0 = data[0] * m.data[0] + data[1] * m.data[2];
	real c1 = data[0] * m.data[1] + data[1] * m.data[3];
	real c2 = data[2] * m.data[0] + data[3] * m.data[2];
	real c3 = data[2] * m.data[1] + data[3] * m.data[3];
	return Matrix2(c0, c1, c2, c3);
}

Matrix2 Matrix2::operator+(const Matrix2 &m) const
{
	real c0 = data[0] + m.data[0];
	real c1 = data[1] + m.data[1];
	real c2 = data[2] + m.data[2];
	real c3 = data[3] + m.data[3];
	return Matrix2(c0, c1, c2, c3);
}

real Matrix2::determinant() const
{
	return (data[0] * data[3] - data[1] * data[2]);
}

void Matrix2::setInverse(const Matrix2 &m)
{
	real det = m.determinant();
	if (det == 0)
		return;

	real c0 = m.data[3] / det;
	real c1 = -m.data[1] / det;
	real c2 = -m.data[2] / det;
	real c3 = m.data[0] / det;

	data[0] = c0;
	data[1] = c1;
	data[2] = c2;
	data[3] = c3;
}

Matrix2 Matrix2::inverse() const
{
	Matrix2 result;
	result.setInverse(*this);
	return result;
}

void Matrix2::invert()
{
	setInverse(*this);
}

void Matrix2::setTranspose(const Matrix2 &m)
{
	real c0 = m.data[0];
	real c1 = m.data[2];
	real c2 = m.data[1];
	real c3 = m.data[3];

	data[0] = c0;
	data[1] = c1;
	data[2] = c2;
	data[3] = c3;
}

Matrix2 Matrix2::transpose() const
{
	Matrix2 result;
	result.setTranspose(*this);
	return result;
}

void Matrix2::setOrientation(const Vector2 &v)
{
	Vector2 o = v.unit();
	data[0] = o.x;	data[1] = -o.y;
	data[2] = o.y;	data[3] = o.x;
}

void Matrix2::setComponents(const Vector2 &a, const Vector2 &b)
{
	data[0] = a.x;	data[1] = b.x;
	data[2] = a.y;	data[3] = b.y;
}



Matrix3::Matrix3()
{
	data[0] = 1;	data[1] = 0;	data[2] = 0;
	data[3] = 0;	data[4] = 1;	data[5] = 0;
}

void Matrix3::print() const
{
	std::cout << "{ {" << data[0] << ", " << data[1] << ", " << data[2] << "}, {"
					   << data[3] << ", " << data[4] << ", " << data[5] << "} }";
}

Matrix3::Matrix3(real c0, real c1, real c2, real c3, real c4, real c5)
{
	data[0] = c0;	data[1] = c1;	data[2] = c2;
	data[3] = c3;	data[4] = c4;	data[5] = c5;
}

Vector2 Matrix3::operator*(const Vector2 &v) const
{
	real c0 = data[0] * v.x + data[1] * v.y + data[2];
	real c1 = data[3] * v.x + data[4] * v.y + data[5];
	return Vector2(c0, c1);
}

Matrix3 Matrix3::operator*(const Matrix3 &m) const
{
	real c0 = data[0] * m.data[0] + data[1] * m.data[3];
	real c1 = data[0] * m.data[1] + data[1] * m.data[4];
	real c2 = data[0] * m.data[2] + data[1] * m.data[5] + data[2];
	real c3 = data[3] * m.data[0] + data[4] * m.data[3];
	real c4 = data[3] * m.data[1] + data[4] * m.data[4];
	real c5 = data[3] * m.data[2] + data[4] * m.data[5] + data[5];
	return Matrix3(c0, c1, c2, c3, c4, c5);
}

real Matrix3::determinant() const
{
	return (data[0] * data[4] - data[1] * data[3]);
}

void Matrix3::setInverse(const Matrix3 &m)
{
	real det = m.determinant();
	if (det == 0)
		return;

	real c0 = m.data[4] / det;
	real c1 = -m.data[1] / det;
	real c2 = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) / det;
	real c3 = -m.data[3] / det;
	real c4 = m.data[0] / det;
	real c5 = (m.data[2] * m.data[3] - m.data[0] * m.data[5]) / det;

	data[0] = c0;
	data[1] = c1;
	data[2] = c2;
	data[3] = c3;
	data[4] = c4;
	data[5] = c5;
}

Matrix3 Matrix3::inverse() const
{
	Matrix3 result;
	result.setInverse(*this);
	return result;
}

void Matrix3::invert()
{
	setInverse(*this);
}

void Matrix3::setOrientationAndPos(const Vector2 &ori, const Vector2 &pos)
{
	Vector2 o = ori.unit();

	data[0] = o.x;
	data[1] = -o.y;
	data[2] = pos.x;

	data[3] = o.y;
	data[4] = o.x;
	data[5] = pos.y;
}

Vector2 Matrix3::transformInverse(const Vector2 &v) const
{
	Vector2 temp = v;
	temp.x -= data[2];
	temp.y -= data[5];

	real c0 = temp.x * data[0] + temp.y * data[3];
	real c1 = temp.x * data[1] + temp.y * data[4];

	return Vector2(c0, c1);
}

Vector2 Matrix3::transformDirection(const Vector2 &v) const
{
	real c0 = v.x * data[0] + v.y * data[1];
	real c1 = v.x * data[3] + v.y * data[4];
	return Vector2(c0, c1);
}

Vector2 Matrix3::transformInverseDirection(const Vector2 &v) const
{
	real c0 = v.x * data[0] + v.y * data[3];
	real c1 = v.x * data[1] + v.y * data[4];
	return Vector2(c0, c1);
}

Vector2 Matrix3::getAxis(int i) const
{
	return Vector2(data[i], data[i+3]);
}

void Matrix3::fillGLMatrix(float m[16]) const
{
	m[0] = data[0]; m[4] = data[1];	m[8]  = 0; m[12] = data[2];
	m[1] = data[3]; m[5] = data[4]; m[9]  = 0; m[13] = data[5];
	m[2] = 0;		m[6] = 0;		m[10] = 1; m[14] = 0;
	m[3] = 0;		m[7] = 0;		m[11] = 0; m[15] = 1;
}