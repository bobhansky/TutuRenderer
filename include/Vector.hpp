#pragma once

#include <math.h>
#include <iostream>




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

	void normalizeW() {

		x = x / w;
		y = y / w;
		z = z / w;
		w = w / w;
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
	float get(int i) const {
		switch (i)
		{
		case 0: {
			return x;
			break;
		}
		case 1: {
			return y;
			break;
		}
		case 2: {
			return z;
			break;
		}
		default:
			return -1;
			break;
		}
	}

	void print() {
		std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
	}

	std::string toString() const {
		return "(" + std::to_string(x)+ ", "+ std::to_string(y) +", "+ std::to_string(z) + ")";
	}

	// ************************* vector operations *************************

	operator std::string() const {
		return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z);
	}
	
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

	Vector3f operator/(const Vector3f& v) const {
		return Vector3f(x / v.x, y / v.y, z / v.z);
	}

	Vector3f operator-(const Vector3f& v) const
	{
		return Vector3f(x - v.x, y - v.y, z - v.z);
	}
	Vector3f operator+(const Vector3f& v) const
	{
		return Vector3f(x + v.x, y + v.y, z + v.z);
	}
	void operator+=(const Vector3f& v)
	{
		x = x + v.x;
		y = y + v.y;
		z = z + v.z;
	}
	void operator-=(const Vector3f& v)
	{
		x = x - v.x;
		y = y - v.y;
		z = z - v.z;
	}
	Vector3f operator -() const 
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


class Mat4f {
public:
	Mat4f() {
		for (int i = 0; i < 16; i++)
			ele[i] = 0;
	}

	Mat4f(float f1, float f2, float f3, float f4, float f5,
		float f6, float f7, float f8, float f9, float f10, 
		float f11, float f12, float f13, float f14, float f15, float f16 ) {
		ele[0] = f1; ele[1] = f2; ele[2] = f3; ele[3] = f4;
		ele[4] = f5; ele[5] = f6; ele[6] = f7; ele[7] = f8;
		ele[8] = f9; ele[9] = f10; ele[10] = f11; ele[11] = f12;
		ele[12] = f13; ele[13] = f14; ele[14] = f15; ele[15] = f16;
	}

	float get(int r, int c) const {
		return ele[c + r * 4];
	}

	void set(int r, int c, float val) {
		ele[c + r * 4] = val;
	}
	void setRow(int r, const Vector3f& vec, float val) {
		ele[0 + r * 4] = vec.x;
		ele[1 + r * 4] = vec.y;
		ele[2 + r * 4] = vec.z;
		ele[3 + r * 4] = val;
	}

	void print() {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
			std::cout << get(i, j) << " ";
			}
			std::cout << "\n";
		}
	}


	

	static Mat4f getTranslate(const Vector3f& v){
		Mat4f r;
		r.set(3, 3, 1);
		r.set(0, 3, v.x);
		r.set(1, 3, v.y);
		r.set(2, 3, v.z);

		r.set(0, 0, 1); r.set(1, 1, 1); r.set(2, 2, 1); r.set(3, 3, 1);
		return r;
	}

	static Mat4f getScale(const Vector3f& v){
		Mat4f r;
		r.set(3, 3, 1);
		r.set(0, 0, v.x);
		r.set(1, 1, v.y);
		r.set(2, 2, v.z);
		return r;
	}

	Vector4f operator*(const Vector4f& v) {
		Vector4f res;
		res.x = v.x * ele[0] + v.y * ele[1] + v.z * ele[2] + v.w * ele[3];
		res.y = v.x * ele[4] + v.y * ele[5] + v.z * ele[6] + v.w * ele[7];
		res.z = v.x * ele[8] + v.y * ele[9] + v.z * ele[10] + v.w * ele[11];
		res.w = v.x * ele[12] + v.y * ele[13] + v.z * ele[14] + v.w * ele[15];
		return res;
	}

	// copy from smallVCM
	Vector3f transformPoint(const Vector3f& p) {
		// get calculated W,vector.w, the forth dimension
		float w = get(3, 3);

		for (int c = 0; c < 3; c++)
			w += get(3, c) * p.get(c);

		// normalization factor
		const float invW = 1.f / w;

		Vector3f res(0);
		// translate value
		res.x = get(0, 3);
		// dot product
		for (int c = 0; c < 3; c++)
			res.x += p.get(c) * get(0, c);
		res.x *= invW;

		res.y = get(1, 3);
		for (int c = 0; c < 3; c++)
			res.y += p.get(c) * get(1, c);
		res.y *= invW;

		res.z = get(2, 3);
		for (int c = 0; c < 3; c++)
			res.z += p.get(c) * get(2, c);
		res.z *= invW;

		return res;
	}


public:
	float ele[16];
};

// copy from SmallVCM
Mat4f operator*(const Mat4f& left, const Mat4f& right)
{
	Mat4f res;
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			float r = 0;
			for (int i = 0; i < 4; i++)
				r += left.get(row, i) * right.get(i, col);
			res.set(row, col, r);
		}
	}
	return res;
}

// afov is horizon tal fov
Mat4f getPerspectiveMatrix(float aFov, float aNear, float aFar, float aspect_ratio) {
	Mat4f p2o;
	p2o.ele[0] = aNear;     p2o.ele[1] = 0.0f;  p2o.ele[2] = 0.0f;					p2o.ele[3] = 0.0f;
	p2o.ele[4] = 0.0f;  p2o.ele[5] = aNear;     p2o.ele[6] = 0.0f;					p2o.ele[7] = 0.0f;
	p2o.ele[8] = 0.0f;  p2o.ele[9] = 0.0f;		p2o.ele[10] = (aNear + aFar);			p2o.ele[11] = aNear * aFar;
	p2o.ele[12] = 0.0f; p2o.ele[13] = 0.0f;		p2o.ele[14] = -1.0f;					p2o.ele[15] = 0.0f;
	float r = std::tan((aFov / 2) * 3.1415926535897f / 180) * aNear;
	float l = -r;
	float t = r/aspect_ratio ;
	float b = -t;
	Mat4f orth_trans(1, 0, 0, -(r + l) / 2,
		0, 1, 0, -(t + b) / 2,
		0, 0, 1, -(aNear + aFar) / 2,
		0, 0, 0, 1);
	Mat4f orth_scale(2 / (r - l), 0, 0, 0,
		0, 2 / -(t - b), 0, 0,	// t - b is flipped so I add minus
		0, 0, 2 / (aNear - aFar), 0,
		0, 0, 0, 1);
	Mat4f orth = orth_scale * orth_trans;
	Mat4f proj = orth * p2o;
	return proj;
}