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

	// by u v
	Vector3f getRGBat(float u, float v) {
		if (width == 0 && height == 0) {
			return Vector3f();
		}
		if (u > 0)
			u = u - (int)u;
		else u = 1 - (abs(u) - (int)abs(u));

		if (v > 0)
			v = v - (int)v;
		else v = 1 - (abs(v) - (int)abs(v));

		int x = u * width;
		int y = v * height;
		// float precision might lead to out of bounds 
		// so clamp it
		int index = y * width + x;
		if (index < 0) index = 0;
		if (index >= rgb.size()) index = rgb.size()-1;
		return getEleIn(rgb, index);
	}

	bool setRGB(int x, int y, const Vector3f& RGB) {
		if (x < 0 || x >= width || y < 0 || y >= height)
			return false;

		rgb[y + x * width] = RGB;
		return true;
	}

	bool setRGB(int index, const Vector3f& RGB) {
		if (index < 0 || index >= width * height)
			return false;

		rgb[index] = RGB;
		return true;
	}

	bool addRGB(int index, const Vector3f& RGB) {
		if (index < 0 || index >= width * height)
			return false;

		rgb[index] += RGB;
		return true;
	}
};