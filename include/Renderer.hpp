#pragma once

#include<fstream>
#include<iostream>
#include<stdexcept>
#include<string>
#include<vector>
#include<cmath>
#include<stack>
#include<thread>
#include<mutex>

#include "Vector.hpp"
#include "global.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Material.hpp"
#include "PPMGenerator.hpp"
#include "BVH.hpp"
#include "BoundBox.hpp"
#include "BVHStrategy.hpp"
#include "BaseInterStrategy.hpp"
#include "Texture.hpp"
#include "IIntegrator.hpp"
#include "PathTracing.hpp"
#include "LightTracing.hpp"
#include "NaivePT.hpp"
#include "BDPT.hpp"


class Renderer {
public:
	// constructor, takes a PPMGenerator for future commands
	Renderer(PPMGenerator* ppmg) {
		g = ppmg;

		if (EXPEDITE) interStrategy = new BVHStrategy();
		else interStrategy = new BaseInterStrategy();

		int inteType = g->integrateType;
		if (inteType == 0)
			integrator = new PathTracing(g, interStrategy);
		else if (inteType == 1)
			integrator = new LightTracing(g, interStrategy);
		else if (inteType == 2)
			integrator = new NaivePT(g, interStrategy);
		else if (inteType == 3)
			integrator = new BDPT(g, interStrategy);

		records = std::vector<std::string>(N_THREAD, std::string());

		g->scene.initializeBVH();
	}

	~Renderer() {
		delete interStrategy;
		delete integrator;
	}


	// takes a PPMGenerator and render its rgb array
	void render() {
		g->initializeLights();
		integrator->integrate(g);
	}

public:
	PPMGenerator* g;
	IIntersectStrategy* interStrategy;
	IIntegrator* integrator;
};
