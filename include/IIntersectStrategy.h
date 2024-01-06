#pragma once

#include "Scene.hpp"
#include "Intersection.hpp"

// intersection test can be the basic way or the BVH way
class IIntersectStrategy {
public:
	// intersection with object
	virtual void UpdateInter(Intersection& inter, Scene& sce,
		const Vector3f& rayOrig, const Vector3f& rayDir) = 0;

	// calculate shadow coefficient, for hard shadow
	virtual float getShadowCoeffi(Scene& sce, Intersection& p, Vector3f& lightpos) = 0;


};