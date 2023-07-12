#pragma once

#include "Object.hpp"


class Sphere : public Object{
public:
	Vector3f centerPos;		// center position
	float radius;

	Sphere() {
		centerPos = Vector3f(0.f, 0.f, 0.f);
		radius = 1.f;
	}

	Sphere(float x, float y, float z, float r) {
		centerPos = Vector3f(x, y, z);
		radius = r;
	}


	virtual ~Sphere() {};

	// check if the ray will intersect with this sphere or not
	// if true, then set the nearest time of intersection as tNear, and mtlColor
	bool intersect(const Vector3f& orig, const Vector3f& dir, Intersection& inter) override {
		float A = 1.f;	// here is 1 since we are using normalized vector
		float B = 2 * (dir.x * (orig.x - centerPos.x) + dir.y * (orig.y - centerPos.y)
			+ dir.z * (orig.z - centerPos.z));
		float C = pow((orig.x - centerPos.x), 2) + pow((orig.y - centerPos.y), 2)
			+ pow((orig.z - centerPos.z), 2) - radius * radius;

		float t1 = 0;
		float t2 = 0;
		solveQuadratic(t1, t2, A, B, C);

		inter.intersected = false;


		// miss, no real solution
		if (FLOAT_EQUAL(t1, FLT_MAX) && FLOAT_EQUAL(t2, FLT_MAX)) {
			return false;
		}

		// ONE soluion
		else if (FLOAT_EQUAL(t1, t2)) {
			// intersection is behind the ray direction, then false
			if (t1 < 0 ) return false;
			else {
				inter.t = t1;
				// update intersection data
				inter.intersected = true;
				inter.obj = this;
				inter.mtlcolor = this->mtlcolor;
				inter.pos = orig + inter.t * dir;
				inter.nDir = normalized(inter.pos - centerPos);
				if (isTextureActivated)
				{
					// calculate the uv coordinate of this intersection
					float u, v;
					float phi = acos(inter.nDir.z);
					v = phi / M_PI;

					float theta = atan2(inter.nDir.y, inter.nDir.x);	// return [-pi, pi]
					// we need to map it to [0, 1]
					if (theta < 0) theta += 2 * M_PI;	// trigonometric functions are periodic
					u = (theta / (2.f * M_PI));			// 0 + [0, 1]    then if theta == 0, it is the left most point in width

					// or 
					// u = 0.5 + (theta / (2.f * M_PI));  // 0.5 + [-0.5, 0.5]	  then if theta == 0, it is the middle point in width

					inter.textPos = Vector2f(u, v);
					inter.textureIndex = this->textureIndex;
					inter.normalMapIndex = normalMapIndex;
				}
				return true;
			}

		}
		// TWO solution
		else {
			if (t1 > 0 && t2 > 0) {
				inter.t = t1;
			}
			else if (t1 > 0 && t2 < 0) {
				inter.t = t1;
			}
			else if (t1 < 0 && t2 > 0) {
				inter.t = t2;
			}
			else return false;

			// update intersection data
			inter.intersected = true;
			inter.obj = this;
			inter.mtlcolor = this->mtlcolor;
			inter.pos = orig + inter.t * dir;
			inter.nDir = normalized(inter.pos - centerPos);
			if (isTextureActivated)
			{
				// calculate the uv coordinate of this intersection
				float u, v;
				float phi = acos(inter.nDir.z);	// return [0, pi]
				v = phi / M_PI;

				float theta = atan2(inter.nDir.y, inter.nDir.x);	// return [-pi, pi]
				// we need to map it to [0, 1]
				if (theta < 0) theta += 2 * M_PI;	// trigonometric functions are periodic
				u = (theta / (2.f * M_PI));			// 0 + [0, 1]    then if theta == 0, it is the left most point in width

				// or 
				// u = 0.5 + (theta / (2.f * M_PI));  // 0.5 + [-0.5, 0.5]	  then if theta == 0, it is the middle point in width

				inter.textPos = Vector2f(u, v);
				inter.textureIndex = this->textureIndex;
				inter.normalMapIndex = normalMapIndex;
			}
			return true;
		}
		return false;
	}


	void initializeBound() override {
		Vector3f min = { centerPos.x - radius, centerPos.y - radius, centerPos.z - radius };
		Vector3f max = { centerPos.x + radius, centerPos.y + radius, centerPos.z + radius };
		bound = BoundBox(min, max);
	}

	float getArea() override {
		return radius * radius * M_PI;
	}

	void samplePoint(Intersection& inter, float& pdf) override {
		// theta: angle from +x to yz plane	
		// phi: angle from +z to xy plane
		// 7/6/2023: 

		float theta = getRandomFloat() * 2 * M_PI;
		float phi = getRandomFloat() * M_PI;

		inter.pos.x = centerPos.x + radius * cos(theta) * sin(phi);
		inter.pos.y = centerPos.y + radius * sin(theta) * sin(phi);
		inter.pos.z = centerPos.z + radius * cos(phi);

		inter.nDir = normalized(inter.pos - centerPos);

		inter.intersected = true;
		inter.mtlcolor = mtlcolor;
		inter.obj = this;

		// I don't update the texture corrdinate here	7/2/2023

		pdf = 1 / getArea();
	}
};



// 3/23/2023:
// Problem (solved somehow): doing raytracing with transparent object
// first calculating transmittance then reflection will causes self intersection (ray travel time < 0.001)
// and thus have a different intersection point and new reflection/refraction rays
// which is different from first calculating reflection and then transmittance (I wonder why 
// calculating in such order will not have self intersection)

// But why sphere?  inside the sphere, it has concave?
// 
// 
// solution is checking 
// if (t1 < 0.001f) return false;


// completely solved:
// change to current version of contribution calculation instead
// and we don't have to do sphere intersection edition

// reason for the phenomenon in previous version
// remains unknown


// 3/23/2023 20:43:
// truly solved:
// I use the same RayOrig for transmittanceand reflection
// so the order matters