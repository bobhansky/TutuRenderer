#pragma once

#include "IIntersectStrategy.h"
#include "BVH.hpp"
#include "Vector.hpp"

// loop through all the objects to test intersection
// the elementary method
class BaseInterStrategy : public IIntersectStrategy {
	void UpdateInter(Intersection& inter, Scene& sce,
		const Vector3f& rayOrig, const Vector3f& rayDir) {

		for (const auto& obj : sce.objList) {
			Intersection interTemp;
			if (obj->intersect(rayOrig, rayDir, interTemp)) {	// intersect also update intersection
				// if the ray hits this object first, then we update intersection
				if (interTemp.t < inter.t) {
					inter = interTemp;
				}
			}
		}

	}

	virtual float getShadowCoeffi(Scene& sce, Intersection& p, Vector3f& lightPos) override{
		Vector3f orig = p.pos;
		orig = orig + 0.0005f * p.nDir;
		Vector3f raydir = normalized(lightPos - orig);
		float distance = (lightPos - orig).norm();

		// loop through all the objects in the scene 
		// if there's one valid intersection, thrn return 0
		float res = 1;
		for (auto& i : sce.objList) {
			if (i.get() == p.obj) continue;			// do not test intersection with itself
			if (i->isLight) continue;				// do not test with light avatar
			Intersection p_light_inter;

			if (i->intersect(orig, raydir, p_light_inter) && p_light_inter.t < distance) {
				res = res * (1 - p_light_inter.mtlcolor.alpha);
			}
		}
		return res;
	}


};