#pragma once

#include "Object.hpp"
#include "Intersection.hpp"



class Triangle : public Object{
public:
	// 3 vertices, from lower left counterclockwise 
	Vector3f v0;
	Vector3f v1;
	Vector3f v2;

	// normal direction of 3 vertices
	Vector3f n0, n1, n2;
	Vector2f uv0, uv1, uv2;	// texture coordinate u,v   (-1,-1) initially means no texture 

	// Using Moller Trumbore Algorithm to update the intersection between ray and triangle
	// solve with Cramer's rule
	// https://www.geeksforgeeks.org/system-linear-equations-three-variables-using-cramers-rule/#
	// i didnt read it carefully
	bool intersect(const Vector3f& orig, const Vector3f& dir, Intersection& inter) override {
		
		Vector3f E1 = v1 - v0;
		Vector3f E2 = v2 - v0;          // v2 - v1   get the strange res
		Vector3f S = orig - v0;
		Vector3f S1 = crossProduct(dir, E2);		// pvec
		Vector3f S2 = crossProduct(S, E1);			

		// check if ray parallel to the surface
		// and 
		// under surface ray					
		Vector3f normal = crossProduct(E1, E2);
		normal = normalized(normal);

		// if the ray is parallel to the surface, then no inter
		// I CONSIDER undersurface ray, so it's not >=
		if (dir.dot(normal) == 0.f)
			return false;

		Vector3f rightVec(S2.dot(E2), S1.dot(S),S2.dot(dir));	// right handside part in my note games101
		if (S1.dot(E1) == 0.f) return false;
		float left = 1.0f / S1.dot(E1);

		Vector3f res = left * rightVec;		// t u v are in res now

		if (res.x  > 0 && 1 - res.y - res.z  > 0 && res.y  > 0 && res.z  > 0) {
			// then update intersection
			inter.intersected = true;
			inter.obj = this;
			inter.t = res.x;
			inter.pos = orig + inter.t * dir;
			inter.mtlcolor = this->mtlcolor;
			// also get the interpolated normal direction
			// must normalize this interpolated direction!!!   2/19/2023 by bokitian
			inter.nDir = normalized((n0 * (1 - res.y - res.z)) + n1 * res.y + n2 * res.z);
			inter.nDir = normalized(crossProduct(E1, E2));

			// if this triangle's texture is activated, then do interpolation to 
			// calculate texture coordinates
			if (isTextureActivated)
			{
				inter.textPos = uv0 * (1 - res.y - res.z) + uv1 * res.y + uv2 * res.z;
				inter.diffuseIndex = this->textureIndex;
				inter.normalMapIndex = this->normalMapIndex;
			}
			return true;
		}
		return false;
		

		/*
		// OR use brutal force
		// find the plane first and then test of the point is inside the triangle
		Vector3f e1 = v1 - v0;
		Vector3f e2 = v2 - v0;
		Vector3f n = crossProduct(e1, e2);
		n = normalized(n);
		float A = n.x;
		float B = n.y;
		float C = n.z;
		float D = -A * v0.x - B * v0.y - C * v0.z;
		float denominator = A * dir.x + B * dir.y + C * dir.z;
		if (FLOAT_EQUAL(0.f, denominator)) return false;

		float t = -(A * orig.x + B * orig.y + C * orig.z + D) / denominator;
		Vector3f p = orig + t * dir ;

		float a, b, g;	// barycentric coefficient

		if (this->getBarycentric(p, a, b, g) && t > 0 && a > 0 && b > 0 && g > 0) {
			// then update intersection
			inter.intersected = true;
			inter.obj = this;
			inter.t = t;
			inter.pos = orig + inter.t * dir;
			inter.mtlcolor = this->mtlcolor;
			// must normalize this interpolated direction!!!   2/19/2023 by bokitian
			inter.nDir = normalized((n0 * a) + n1 *b + n2 * g);

			// if this triangle's texture is activated, then do interpolation to 
			// calculate texture coordinates
			return true;
		}
		return false;

		*/
	}


	// get the alpha beta gamma in barycentric corrdinate
	// update a b g in the parameter list
	// return true if we can find a b g
	// false otherwise
	bool getBarycentric(const Vector3f& point,
		float& alpha, float& beta, float& gamma) {
		// triangles.pdf   page 69

		Vector3f e1 = v1 - v0;
		Vector3f e2 = v2 - v0;
		Vector3f ep = point - v0;

		float d11 = e1.dot(e1);
		float d12 = e1.dot(e2);
		float d22 = e2.dot(e2);
		float d1p = e1.dot(ep);
		float d2p = e2.dot(ep);

		float det = d11 * d22 - d12 * d12;
		if (FLOAT_EQUAL(0.f, det)) return false;		// 3 vertices on a line: not a triangle

		beta = (d22 * d1p - d12 * d2p) / det;
		gamma = (d11 * d2p - d12 * d1p) / det;
		alpha = 1 - beta - gamma;
		return true;
	}

	void initializeBound() override {
		bound = BoundBox(v0, v1);
		bound = Union(bound, v2);
	}

	float getArea() override {
		Vector3f e1 = v1 - v0;
		Vector3f e2 = v2 - v0;

		e1 = v1 - v0;
		e2 = v2 - v0;
		return  crossProduct(e1, e2).norm() * 0.5f;
	}

	void samplePoint(Intersection& inter, float& pdf) override {
		
		float u = getRandomFloat();
		float v = getRandomFloat() * (1 - u);

		Vector3f pos = (1 - u - v) * v0 + u * v1 + v * v2;


		inter.pos = pos;
		inter.nDir = (1 - u - v) * n0 + u * n1 + v * n2;
		inter.nDir = normalized(inter.nDir);
		inter.intersected = true;
		inter.mtlcolor = mtlcolor;
		inter.obj = this;
		

		float area = getArea();
		pdf = 1.f / area ;

		
		if (isTextureActivated) {
			inter.normalMapIndex = normalMapIndex;
			inter.textPos = uv0 * (1 - u - v) + uv1 * u + uv2 * v;
			inter.diffuseIndex = textureIndex;
		}
	}
};