#pragma once

#include <algorithm>

#include "Vector.hpp"


class BoundBox {
public:
	Vector3f pMin, pMax;		// two points specifiy the bound

	// construction, takes two point and specify the pmin pmax
	BoundBox(Vector3f& p1, Vector3f& p2) {


		pMin = Vector3f(
			fmin(p1.x, p2.x),
			fmin(p1.y, p2.y),
			fmin(p1.z, p2.z)
		);

		pMax = Vector3f(
			fmax(p1.x, p2.x),
			fmax(p1.y, p2.y),
			fmax(p1.z, p2.z)
		);
	}

	// default constructor
	BoundBox() {

	}

	// return the centroid of this bounding box
	Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

	// return the diagonal of this bounding box
	Vector3f Diagonal() const { return pMax - pMin; }

	// return the index of max elment in diagonal
	// which dimension is the longest one? x y z
	// helper function for dividing the bounding box
	int maxExtent() const
	{
		Vector3f d = Diagonal();
		if (d.x > d.y && d.x > d.z)
			return 0;
		else if (d.y > d.z)
			return 1;
		else
			return 2;
	}

	// return if a ray intersect with this bounding box
	bool IntersectRay(const Vector3f& rayOrig, const Vector3f& rayDir) {
		// for computation efficiency
		Vector3f invDir = { 1 / rayDir.x, 1 / rayDir.y, 1 / rayDir.z };

		// distance/speed 
		float tmin_x = (pMin.x - rayOrig.x) * invDir.x;      // first time ray gets into box in x direction
		float tmax_x = (pMax.x - rayOrig.x) * invDir.x;      // first time ray leaves box in x direction

		float tmin_y = (pMin.y - rayOrig.y) * invDir.y;      // first time ray gets into box in y direction
		float tmax_y = (pMax.y - rayOrig.y) * invDir.y;      // first time ray leaves box in y direction

		float tmin_z = (pMin.z - rayOrig.z) * invDir.z;      // first time ray gets into box in z direction
		float tmax_z = (pMax.z - rayOrig.z) * invDir.z;      // first time ray leaves box in z direction

		// check the sign and determine the corrected time
		if (rayDir.x < 0) std::swap(tmin_x, tmax_x);
		if (rayDir.y < 0) std::swap(tmin_y, tmax_y);
		if (rayDir.z < 0) std::swap(tmin_z, tmax_z);


		// tmin: for 3 dimensions, the latest time entering the box
		// tmax: for 3 dimensions, the earlest time leaving the box
		float t_enter, t_exit;
		// t_enter = fmax(tmin_x, fmax(tmin_y, tmin_z));
		float buffer = tmin_y > tmin_z ? tmin_y : tmin_z;
		t_enter = tmin_x > buffer ? tmin_x : buffer;

		// t_exit = fmin(tmax_x, fmin(tmax_y, tmax_z));
		buffer = tmax_y < tmax_z ? tmax_y : tmax_z;
		t_exit = tmax_x < buffer ? tmax_x : buffer;

		// use <= >= here.   when triangle is parallel to one dimension, 
		// t_enter == t_exit within that dimension
		if (t_enter <= t_exit && t_exit >= 0.f)	
			return true;

		return false;
	}
};


// union two bounds
BoundBox Union(BoundBox& b1, BoundBox& b2) {
	Vector3f min(fmin(b1.pMin.x, b2.pMin.x),
		fmin(b1.pMin.y, b2.pMin.y),
		fmin(b1.pMin.z, b2.pMin.z)
	);

	Vector3f max(fmax(b1.pMax.x, b2.pMax.x),
		fmax(b1.pMax.y, b2.pMax.y),
		fmax(b1.pMax.z, b2.pMax.z)
	);

	return BoundBox(min, max);
}

// union a bounding box and a point
inline BoundBox Union(BoundBox& b, Vector3f& v) {
	Vector3f min(fmin(b.pMin.x, v.x),
		fmin(b.pMin.y, v.y),
		fmin(b.pMin.z, v.z)
	);

	Vector3f max(fmax(b.pMax.x, v.x),
		fmax(b.pMax.y, v.y),
		fmax(b.pMax.z, v.z)
	);

	return  BoundBox(min, max);
}

