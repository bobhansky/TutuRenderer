#pragma once

#include <math.h>


class Vector3i {
public:
	int x;
	int y;
	int z;

	Vector3i(int xval, int yval, int zval) : x(xval), y(yval),z(zval) {}

	Vector3i() {
		x = 0;
		y = 0;
		z = 0;
	}
};

class Vector4f {
public:
	float x;
	float y;
	float z;
	float w;

	Vector4f(float xval, float yval, float zval, float wval) : x(xval), y(yval), z(zval), w(wval) {}

	Vector4f() {
		x = 0.f;
		y = 0.f;
		z = 0.f;
		w = 0.f;
	}
};

class Vector2f {
public:
	float x;
	float y;
	Vector2f(float xval, float yval) : x(xval), y(yval) {}
	Vector2f() {
		x = -1.f;
		y = -1.f;
	}

	Vector2f operator*(const float& c) {
		return Vector2f(x * c, y * c);
	}
	Vector2f operator+(const Vector2f& v) const
	{
		return Vector2f(x + v.x, y + v.y);
	}
};



class Vector3f {
public:
	float x;
	float y;
	float z;

	Vector3f(float xval, float yval, float zval) : x(xval), y(yval), z(zval) {}

	Vector3f() {
		x = 0.f;
		y = 0.f;
		z = 0.f;
	}

	Vector3f(float i) {		// implicit 	Vector3f a = 1;
		x = i;
		y = i;
		z = i;
	}

	void print() {
		std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
	}

	// ************************* vector operations *************************
	
	// const float&		const lvalue reference, can point to both lvalue and rvalue
	Vector3f operator*(const float& c) {
		return Vector3f(x * c, y * c, z * c);
	}

	Vector3f operator/(const float& c) const{
		return Vector3f(x / c, y / c, z / c);
	}

	// element-wise product
	Vector3f operator*(const Vector3f& v) const
	{
		return Vector3f(x * v.x, y * v.y, z * v.z);
	}
	Vector3f operator-(const Vector3f& v) const
	{
		return Vector3f(x - v.x, y - v.y, z - v.z);
	}
	Vector3f operator+(const Vector3f& v) const
	{
		return Vector3f(x + v.x, y + v.y, z + v.z);
	}
	Vector3f operator - () const 
	{ 
		return Vector3f(-x, -y, -z); 
	}

	// copy assignment operator
	Vector3f& operator= (const Vector3f& other) {
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;

		return *this;
	}
	
	Vector3f& operator= (const float& scaler) {
		this->x = scaler;
		this->y = scaler;
		this->z = scaler;

		return *this;
	}

	// dot product
	float dot(const Vector3f& v) const {
		return x * v.x + y * v.y + z * v.z;
	}

	friend Vector3f operator*(float c, const Vector3f& v) {
		return Vector3f(v.x * c, v.y * c, v.z * c);
	}

	friend Vector3f operator-(float c, const Vector3f& v) {
		return Vector3f(c - v.x, c - v.y, c - v.z);
	}

	// ************************* vector operations ends ***********************

	// get norm of this vector
	float norm() {
		return sqrtf(x * x + y * y + z * z);
	}

	float norm2() {
		return x * x + y * y + z * z;
	}

};

// return the normalized version of vector v
inline Vector3f normalized(const Vector3f& v) {
	float mag = sqrtf((v.x * v.x + v.y * v.y + v.z * v.z));
	if (mag > 0) {
		float mag_inv = 1 / mag;		// for efficiency we times instead of divide 3 times
		return Vector3f(v.x * mag_inv, v.y * mag_inv, v.z * mag_inv);
	}
	return v;
}

// return the crossProduct of v1 and v2
Vector3f crossProduct(const Vector3f& v1, const Vector3f& v2) {
	return Vector3f(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

