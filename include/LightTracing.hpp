#pragma once
// Integrator: light
#include "IIntegrator.hpp"

// light tracing / particle tracing
class LightTracing : public IIntegrator {
	LightTracing(PPMGenerator* g, IIntersectStrategy* inters) {
		this->g = g;
		this->interStrategy = inters;
	}

	virtual Vector3f integrate(const Vector3f& origin, const Vector3f& dir, PPMGenerator* g, int thdID = -1) {
		return 1;
	}
};