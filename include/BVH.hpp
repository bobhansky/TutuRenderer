#pragma once

#include <vector>
#include <chrono>
#include <algorithm>
#include <cassert>

#include "BoundBox.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"
#include "Object.hpp"


// tree node, contains only ONE object
struct BVHNode {
	Object* obj;
	BoundBox bound;
	BVHNode* left = nullptr;
	BVHNode* right = nullptr;

	~BVHNode() {
	}
};


// BVH acceleration class
// contains a root BVHNode and algorithms to getIntersection with bounds
class BVHAccel {
public:
	// the scene passes in the objList
	BVHAccel(std::vector<Object*> objList): objects(objList) {
		auto start = std::chrono::system_clock::now();
		root = recursiveBuild(objects);
		auto end = std::chrono::system_clock::now();

		std::cout << "\nBVH Building Time consumed: \n";
		std::cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds\n";
	}
	
	~BVHAccel() {	
		deleteBVHtree(root);
	}

	// build the BVH tree based on the objList
	// the Obj in the list has its own BoundBox
	// we use them to initialize the tree
	BVHNode* recursiveBuild(std::vector<Object*> objList) {
		BVHNode* res = new BVHNode();

		// build BVH depending on the size of objList
		if (objList.size() == 0) return res;

		else if (objList.size() == 1) {
			res->bound = objList.at(0)->bound;
			res->left = nullptr;
			res->right = nullptr;
			res->obj = objList[0];
			return res;
		}

		else if (objList.size() == 2) {		// weird here
			res->left = recursiveBuild({ objList[0] });
			res->right = recursiveBuild({ objList[1] });

			res->bound = Union(res->left->bound, res->right->bound);
			return res;
		}

		else {	// multiple objects, then divide the box along the longest dimension
			// first union all the object
			// 5/6/2023: I loop and union objects here, which is very slow but the result 
			// seems to be correct.
			BoundBox unionBound = Union(objList[0]->bound, objList[1]->bound);
			for (int i = 2; i < objList.size(); i++) {
				unionBound = Union(unionBound, objList[i]->bound);
			}
			
			// first find the longest dimension
			// then sort objects by this dimension
			// find the middle object and divide the bound into two
			int longest = unionBound.maxExtent();
			switch (longest) 
			{
			case 0: {	// x dimension is the longest
				std::sort(objList.begin(), objList.end(),
					[](Object* o1, Object* o2) -> bool {
						return o1->bound.Centroid().x < o2->bound.Centroid().x;
					});
				break;
			}
			case 1: {  // y dimension is the longest
				std::sort(objList.begin(), objList.end(),
					[](Object* o1, Object* o2) -> bool {
						return o1->bound.Centroid().y < o2->bound.Centroid().y;
					});
				break;
			}
			case 2: {  // z dimension is the longest
				std::sort(objList.begin(), objList.end(),
					[](Object* o1, Object* o2) -> bool {
						return o1->bound.Centroid().z < o2->bound.Centroid().z;
					});
				break;
			}
			}
			/********* switch ends ***********/
			std::vector<Object*>::iterator begin = objList.begin();
			auto middle = begin + (objList.size() / 2);
			auto end = objList.end();

			std::vector<Object*> leftObjects(begin, middle);
			std::vector<Object*> rightObjects(middle, end);

			assert(objList.size() == rightObjects.size() + leftObjects.size());

			res->left = recursiveBuild(leftObjects);
			res->right = recursiveBuild(rightObjects);

			res->bound = Union(res->left->bound, res->right->bound);
		}

		return res;
	}

	BVHNode* getNode() { return root; }

private:
	std::vector<Object*> objects;	
	BVHNode* root;					// root of the tree


	void deleteBVHtree(BVHNode* node) {
		if (!node) return;

		if (node->left) deleteBVHtree(node->left);
		if (node->right) deleteBVHtree(node->right);

		delete node;
	}
};



// get the Intersection with ginven BVHnode, and ray info
Intersection getIntersection(BVHNode* node, const Vector3f& rayOrig, const Vector3f& rayDir) {
	Intersection inter;
	if (!node) return inter;
	// if ray miss this bound
	if (!node->bound.IntersectRay(rayOrig, rayDir))  
		return inter;

	// if the node is a leaf node
	// then test the intersection of ray and objects
	// in this project, leaf box always contain only 1 object
	if (!node->left && !node->right) {
		node->obj->intersect(rayOrig, rayDir, inter);
		return inter;
	}

	// if node is a internal node
	// then return the nearest intersection 
	Intersection linter = getIntersection(node->left, rayOrig, rayDir);
	Intersection rinter = getIntersection(node->right, rayOrig, rayDir);

	if (linter.t <= rinter.t) return linter;
	return rinter;
}

// test if there's a intesection with non-emissive obj, used for shadow ray
bool hasIntersection(BVHNode* node, const Vector3f& rayOrig, const Vector3f& rayDir, float dis) {
	if (!node) return false;

	if (!node->bound.IntersectRay(rayOrig, rayDir))
		return false;
	
	// if the node is a leaf node
	// then test the intersection of ray and objects
	// in this project, leaf box always contain only 1 object
	if (!node->left && !node->right) {
		Intersection inter;
		node->obj->intersect(rayOrig, rayDir, inter);
		if (inter.mtlcolor.hasEmission()) return false;	// do not test with light, 3/3/2024: not good but a hack

		if (inter.intersected && inter.t < dis)
			return true;
		return false;
	}

	// if node is a internal node
	if (hasIntersection(node->left, rayOrig, rayDir, dis))
		return true;

	return hasIntersection(node->right, rayOrig, rayDir, dis);
}

