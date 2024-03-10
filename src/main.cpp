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

	Material lightmtl;
	lightmtl.emission = Vector3f(60.4f, 60.4f, 60.4f);

	Material orangelightmtl;
	orangelightmtl.diffuse = { 0.749, 0.074, 0.074 };
	orangelightmtl.emission = Vector3f(64.9f, 7.4f, 7.4f) * 1.6f;

	Material frontMtl;
	frontMtl.diffuse = { 0.725f, 0.71f, 0.68f };
	frontMtl.mType = MICROFACET;
	frontMtl.roughness = 0.2f;
	frontMtl.metallic = 0.15f;

	Material metalMtl;
	metalMtl.diffuse = { 0.725f, 0.71f, 0.68f };
	metalMtl.mType = MICROFACET;
	metalMtl.roughness = 0.25f;
	metalMtl.metallic = 0.4;

	Material leftMtl;
	leftMtl.diffuse = { 0.725f, 0.71f, 0.68f };
	leftMtl.mType = MICROFACET;
	leftMtl.roughness = 0.2;
	leftMtl.metallic = 0.67;

	Material floorMtl;
	floorMtl.diffuse = { 0.5, 0.5f, 0.5f };
	floorMtl.mType = LAMBERTIAN;
	floorMtl.roughness = 0.8f;
	floorMtl.metallic = 0;

	Material Unlit;
	Unlit.mType = UNLIT;

	Material shipmtl;
	shipmtl.diffuse = { 0.6, 0.6, 0.6 };
	shipmtl.mType = MICROFACET;
	shipmtl.roughness = 0.3f;
	shipmtl.metallic = 1;

	Material smoothmtl;
	smoothmtl.diffuse = { 0.725f, 0.71f, 0.68f };
	smoothmtl.mType = MICROFACET;
	smoothmtl.roughness = 0.07f;
	smoothmtl.metallic = 0;


	//objl::Loader light;
	//if (light.LoadFile("./model/space/light.obj")) {
	//	g.transObj(light, 10, -20, 0);
	//	g.loadObj(light, lightmtl, -1, -1);
	//}

	objl::Loader orange1;
	if (orange1.LoadFile("./model/space/lightOrange1.obj")) {
		g.transObj(orange1, 0, -5, 10);
		g.loadObj(orange1, lightmtl, -1, -1);

		g.transObj(orange1, 0, 15, 0);
		g.loadObj(orange1, lightmtl, -1, -1);

	}


	objl::Loader front;
	if (front.LoadFile("./model/space/front.obj"))
		g.loadObj(front, frontMtl, 1, -1);

	objl::Loader back;
	if (back.LoadFile("./model/space/back.obj"))
		g.loadObj(back, floorMtl, 1, -1);

	objl::Loader left;
	if (left.LoadFile("./model/space/left.obj"))
		g.loadObj(left, leftMtl, 1, -1);

	objl::Loader right;
	if (right.LoadFile("./model/space/right.obj"))
		g.loadObj(right, floorMtl, 2, -1);

	objl::Loader floor;
	if (floor.LoadFile("./model/space/floor.obj"))
		g.loadObj(floor, floorMtl, 2, -1);

	objl::Loader roof;
	if (roof.LoadFile("./model/space/roof.obj"))
		g.loadObj(roof, floorMtl, 2, -1);


	objl::Loader floor_r;
	if (floor_r.LoadFile("./model/space/floor_r.obj"))
		g.loadObj(floor_r, smoothmtl, -1, -1);

	objl::Loader sky;
	if (sky.LoadFile("./model/space/sky.obj")) {
		g.loadObj(sky, Unlit, 0, -1);
	}

	objl::Loader ship;
	if (ship.LoadFile("./model/space/sordfish_smooth.obj")) {
		g.rotateObj(ship, 0, -10);
		g.rotateObj(ship, 2, -15);
		g.scaleObj(ship, 0.1, 0.1, 0.1);
		g.transObj(ship, 32, -5, -10);
		g.loadObj(ship, shipmtl, -1, -1);
	}

	objl::Loader smallBox;
	if (smallBox.LoadFile("./model/space/smallBox.obj")) {
		g.rotateObj(smallBox, 1, 180);
		g.scaleObj(smallBox, 15, 15, 10);
		g.transObj(smallBox, -2.4, -44, -50);
		g.loadObj(smallBox, metalMtl, -1, -1);
	}

	Renderer r(&g);
	auto start = std::chrono::system_clock::now();		// #include <chrono>
	r.render();
	auto end = std::chrono::system_clock::now();
	std::cout << "\nRendering Time consumed: \n";
	std::cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds\n";


	Postprocessor p(&g.output);
	start = std::chrono::system_clock::now();
	//g.output = p.performPostProcess();
	end = std::chrono::system_clock::now();
	std::cout << "\nPost Processing Time consumed: \n";
	std::cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds\n";

	std::cout << "output to img...\n";
	g.generate();



	return 0;
}