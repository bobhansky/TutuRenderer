#pragma once

#include <vector>
#include <memory>	// smart pointer

#include "Vector.hpp"
#include "Object.hpp"
#include "BVH.hpp"


class Scene {
public:
	// use pointer array instead of "object" array for 
	// unique_ptr<T>'s declared move constructor
	// move instead of copy for efficiency
	std::vector<std::unique_ptr<Object>> objList;
	
	BVHAccel* BVHaccelerator;

	// add object into object list
	void add(std::unique_ptr<Object> obj) {
		objList.emplace_back(std::move(obj));
	}



	// initalize bounding box
	void initializeBVH() {
		std::vector<Object*> objl;
		for (auto &i : objList) {
			objl.emplace_back(i.get());
		}

		BVHaccelerator = new BVHAccel(objl);
	}

	~Scene() {
		delete BVHaccelerator;
	}

};