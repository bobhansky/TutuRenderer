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


	Material roomMtl;
	roomMtl.mType = LAMBERTIAN;
	roomMtl.diffuse = { 0.725f, 0.71f, 0.68f };
	
	objl::Loader room;
	if (room.LoadFile("../model/veach_bdpt/veach_room.obj")) {
		g.loadObj(room, roomMtl);
	}

	Material LlightMtl;
	LlightMtl.diffuse = { 0.725f, 0.71f, 0.68f };
	LlightMtl.emission = { 500.0, 500.0, 500.0 };

	objl::Loader Llight;
	if (Llight.LoadFile("../model/veach_bdpt/veach_Llight.obj")) {
		g.loadObj(Llight, LlightMtl);
	}

	Material tableMtl;
	tableMtl.mType = LAMBERTIAN;
	tableMtl.diffuse = { 0.32962962985, 0.257976263762, 0.150291711092 };

	objl::Loader table;
	if (table.LoadFile("../model/veach_bdpt/veach_table.obj")) {
		g.loadObj(table, tableMtl);
	}

	Material glassMtl;
	glassMtl.mType = PERFECT_REFRACTIVE;
	glassMtl.eta = 1.5f;

	objl::Loader glass;
	if (glass.LoadFile("../model/veach_bdpt/veach_glass.obj")) {
		g.loadObj(glass, glassMtl);
	}

	Material tallLampMtl;
	tallLampMtl.mType = MICROFACET_R;
	tallLampMtl.roughness = 0.2775146484375f;
	tallLampMtl.metallic = 0.5f;
	tallLampMtl.diffuse = { 0.32962962985, 0.257976263762, 0.150291711092 };

	objl::Loader tallLamp;
	if (tallLamp.LoadFile("../model/veach_bdpt/veach_tallLamp.obj")) {
		g.loadObj(tallLamp, tallLampMtl);
	}


	objl::Loader wallLamp;
	if (wallLamp.LoadFile("../model/veach_bdpt/veach_wallLamp.obj")) {
		g.loadObj(wallLamp, roomMtl);
	}


	Material sLlightMtl;
	sLlightMtl.diffuse = { 0.725f, 0.71f, 0.68f };
	sLlightMtl.emission = { 6999.999881f, 5450.000167f, 3630.000055f };

	objl::Loader slight;
	if (slight.LoadFile("../model/veach_bdpt/veach_slight.obj")) {
		g.loadObj(slight, sLlightMtl);
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