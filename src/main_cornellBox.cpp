#include "../include/PPMGenerator.hpp"
#include "../include/Sphere.hpp"
#include "../include/Scene.hpp"
#include "../include/Object.hpp"
#include "../include/Renderer.hpp"
#include "../include/OBJ_Loader.h"
#include "../include/Postprocessor.hpp"

#include <string>
#include <chrono>


int main(int argc, char* argv[]) {
	if (argc < 2) {
		std::cout << "ERROR: lack of the input configuration file, please provide its path as the first argument.\n";
		return 0;
	}


	PPMGenerator g(argv[1]);


	Material floorMtl;
	floorMtl.mType = LAMBERTIAN;
	floorMtl.diffuse = { 0.725f, 0.71f, 0.68f };
	
	objl::Loader floor;
	if (floor.LoadFile("../model/cornellBox/floor.obj")) {
		g.loadObj(floor, floorMtl, -1, -1);
	}

	Material lightMtl;
	lightMtl.diffuse = { 0.725f, 0.71f, 0.68f };
	lightMtl.emission = { 47.8348007,38.5663986, 31.0807991 };

	objl::Loader light;
	if (light.LoadFile("../model/cornellBox/light.obj")) {
		g.loadObj(light, lightMtl, -1, -1);
	}

	Material green;
	green.mType = LAMBERTIAN;
	green.diffuse = { 0.14f, 0.45f, 0.091f };

	objl::Loader right;
	if (right.LoadFile("../model/cornellBox/right.obj")) {
		g.loadObj(right, green, -1, -1);
	}

	Material red;
	red.mType = LAMBERTIAN;
	red.diffuse = { 0.63f, 0.065f, 0.05f };

	objl::Loader left;
	if (left.LoadFile("../model/cornellBox/left.obj")) {
		g.loadObj(left, red, -1, -1);
	}

	Material white;
	white.mType = LAMBERTIAN;
	white.diffuse = { 0.725f, 0.71f, 0.68f };

	objl::Loader tall;
	if (tall.LoadFile("../model/cornellBox/tallbox.obj")) {
		g.loadObj(tall, white, -1, -1);
	}

	objl::Loader shortb;
	if (shortb.LoadFile("../model/cornellBox/shortbox.obj")) {
		g.loadObj(shortb, white, -1, -1);
	}

	
	Renderer r(&g);
	auto start = std::chrono::system_clock::now();		// #include <chrono>
	r.render();
	auto end = std::chrono::system_clock::now();
	std::cout << "\nRendering Time consumed: \n";
	std::cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds\n";


	Postprocessor p(&g.cam.FrameBuffer);
	start = std::chrono::system_clock::now();
	//g.output = p.performPostProcess();
	end = std::chrono::system_clock::now();
	std::cout << "\nPost Processing Time consumed: \n";
	std::cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds\n";

	std::cout << "output to img...\n";
	g.generate();


	return 0;
}