#pragma once

#include <float.h>
#include "global.hpp"
#include "Vector.hpp"
#include "Material.hpp"
//#include "Object.hpp"


class Object;	// circular dependency issue
				// see https://stackoverflow.com/questions/23283080/compiler-error-c4430-missing-type-specifier-int-assumed

class Intersection {
public:

	bool intersected = false;
	float t = FLT_MAX;	// pos = rayPos + t * rayDir
	Vector3f pos;
	Vector3f Ng; // geometric normal
	Vector3f Ns;	// shading normal

	Vector2f textPos;	// texture coordinates if any	(-1, -1) means no texture
	int diffuseIndex = -1;
	int normalMapIndex = -1;
	int roughnessMapIndex = -1;
	int metallicMapIndex = -1;


	float beta, gamma;	// berrycentric coords


	Material mtlcolor;
	Object *obj = nullptr;		// this intersection is on which object	

};