#pragma once
#include "global.hpp"
#include "Vector.hpp"

#include <vector>


class Texture {
public:
	std::string name;
	int width = 0;
	int height = 0;
	std::vector<Vector3f> rgb;

	Vector3f getRGBat(float u, float v) {
		if (width == 0 && height == 0) {
			return Vector3f();
		}

		int x = u * width;
		int y = v * height;
		// float precision might lead to out of bounds 
		// so clamp it
		int index = y * width + x;
		if (index < 0) index = 0;
		if (index >= rgb.size()) index = rgb.size()-1;
		return getEleIn(rgb, index);
	}
};