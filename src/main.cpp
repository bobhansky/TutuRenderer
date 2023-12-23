#include "../include/PPMGenerator.hpp"
#include "../include/Sphere.hpp"
#include "../include/Scene.hpp"
#include "../include/Object.hpp"
#include "../include/Renderer.hpp"
#include "../include/OBJ_Loader.h"

#include <string>
#include <chrono>


int main(int argc, char* argv[]) {
	if (argc < 2) {
		std::cout << "ERROR: lack of the input configuration file, please provide its path as the first argument.\n";
		return 0;
	}


	PPMGenerator g(argv[1]);

	Material redmtl;
	redmtl.diffuse = Vector3f(0.63f, 0.065f, 0.05f);
	redmtl.ka = 0.2;
	redmtl.kd = 0.5;
	redmtl.ks = 0;
	redmtl.n = 64;
	redmtl.alpha = 1;
	redmtl.eta = 1.52;

	Material greenmtl;
	greenmtl.diffuse = Vector3f(0.14f, 0.45f, 0.091f);
	greenmtl.ka = 0.2;
	greenmtl.kd = 0.5;
	greenmtl.ks = 0;
	greenmtl.n = 64;
	greenmtl.alpha = 1;
	greenmtl.eta = 1.52;

	Material whitemtl;
	whitemtl.diffuse = Vector3f(0.725f, 0.71f, 0.68f);
	whitemtl.ka = 0.2;
	whitemtl.kd = 0.5;
	whitemtl.ks = 0;
	whitemtl.n = 64;
	whitemtl.alpha = 1;
	whitemtl.eta = 1.52;

	Material glass;
	glass.diffuse = Vector3f(0.725f, 0.71f, 0.68f);
	glass.ka = 0.1;
	glass.kd = 0.1;
	glass.ks = 0.2;
	glass.n = 64;
	glass.alpha = 0.2;
	glass.eta = 1.52;

	Material lightmtl;
	lightmtl.diffuse = Vector3f(0.65f, 0.65f, 0.65f);
	lightmtl.ka = 1;
	lightmtl.kd = 0.4;
	lightmtl.ks = 0;
	lightmtl.n = 64;
	lightmtl.alpha = 1;
	lightmtl.eta = 1.52;
	lightmtl.emission = Vector3f(47.8348f, 38.5664f, 31.0808f);


	objl::Loader floor;
	if (floor.LoadFile("./model/floor.obj"))
		g.loadObj(floor, whitemtl, -1, -1);

	objl::Loader shortBox;
	if (shortBox.LoadFile("./model/shortbox.obj"))
		g.loadObj(shortBox, whitemtl, -1, -1);

	objl::Loader tallbox;
	if (tallbox.LoadFile("./model/tallbox.obj"))
		g.loadObj(tallbox, whitemtl, -1, -1);

	objl::Loader left;
	if (left.LoadFile("./model/left.obj"))
		g.loadObj(left, redmtl, -1, -1);

	objl::Loader right;
	if (right.LoadFile("./model/right.obj"))
		g.loadObj(right, greenmtl, -1, -1);

	objl::Loader light;
	if (light.LoadFile("./model/light.obj"))
		g.loadObj(light, lightmtl, -1, -1);

	Renderer r(&g);
	auto start = std::chrono::system_clock::now();		// #include <chrono>


	r.render();
	g.generate();


	auto end = std::chrono::system_clock::now();
	std::cout << "\nRendering Time consumed: \n";
	std::cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds\n";
	return 0;


}