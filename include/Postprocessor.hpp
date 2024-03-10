#pragma once

#include<thread>
#include<mutex>
#include <math.h>

#include "PPMGenerator.hpp"
#include "Texture.hpp"



class Postprocessor {
public:
	Texture *source;	// Reference member have to be initialized on creation.
	// renderTextures[0]: original texture
	// renderTextures[1]: emmisive texture
	// renderTexture[2] & [3]: used for gaussian blur on emmisive texture
	std::vector<Texture> renderTextures;

	Postprocessor(Texture* source): source(source) {
		renderTextures = std::vector<Texture>(4);
		renderTextures[0] = *source;
	}

	Texture performPostProcess() {
		// 1. bloom: 
		//	1.1 extract emmisive texture first
		renderTextures[1] = getEmmisiveTexture(&renderTextures[0]);
		//	1.2 gaussian blur 
		int index = 2;		// 2 and 3 interchangablly
		renderTextures[index] = getGaussianBlurTexture(&renderTextures[1], 11, 30.f);
		index = 3;
		for (int i = 0; i < 2; i++) {
			renderTextures[index] = getGaussianBlurTexture(&renderTextures[index == 2 ? 3 : 2], 11, 30.f);
			index = index == 2 ? 3 : 2;
		}
		//	1.3 add raw img to blured
		index = index == 2 ? 3 : 2;
		// return renderTextures[index];		// return blured emmisive texture
		return add(&renderTextures[0], &renderTextures[index]);
		
	}

	

	Texture getGaussianBlurTexture(Texture *img, int kernelSize, float stddev ) {
		Texture src = *img;

		Texture res;
		res.height = src.height;
		res.width = src.width;
		int h = res.height; int w = res.width;
		res.rgb.resize(w * h);

// multithreading is slower than single threading
		static float E = 2.7182818f;
		auto gaussian = [](int inputX, float standardDev) -> float {
			return (1 / sqrt(2 * M_PI * standardDev)) * pow(E, -(inputX * inputX) / (2 * standardDev * standardDev));
			};

		// blur vertical
		for (int x = 0; x < w; x++) {
			for (int y = 0; y < h; y++) {
				Vector3f& color = res.rgb.at(y * w + x);

				int startY = -kernelSize * 0.5;
				Vector3f col;
				float kernelSum = 0;

				for (int i = 0; i < kernelSize; i++) {
					float U = (float)x / w;
					float V = (float)(y + i + startY) / h;
					float gauss = gaussian((startY + i), stddev);
					col = col + src.getRGBat(clamp(0, 0.999f, U), clamp(0, 0.999f, V)) * gauss;

					kernelSum += gauss;
				}
				color = col / kernelSum;
			}
		}
		// horizontally
		src = res;
		for (int x = 0; x < w; x++) {
			for (int y = 0; y < h; y++) {
				Vector3f& color = res.rgb.at(y * w + x);

				int startX = -kernelSize * 0.5;
				Vector3f col;
				float kernelSum = 0;

				for (int i = 0; i < kernelSize; i++) {
					float U = (float)(x + i + startX) / w;
					float V = (float)y / h;
					float gauss = gaussian((startX + i), stddev);
					col = col + src.getRGBat(clamp(0, 0.999f, U), clamp(0, 0.999f, V)) * gauss;
					kernelSum += gauss;
				}
				color = col / kernelSum;
			}
		}
		return res;
	}


	Texture getEmmisiveTexture(Texture* img) {
		Texture src = *img;
		Texture res;
		res.height = src.height;
		res.width = src.width;
		int h = res.height; int w = res.width;
		res.rgb.resize(w * h);

		for (int x = 0; x < w; x++) {
			for (int y = 0; y < h; y++) {
				Vector3f& color = res.rgb.at(y * res.width + x);

				float U = (float)x / w;
				float V = (float)y / h;
				Vector3f col = src.getRGBat(clamp(0, 1, U), clamp(0, 1, V));
				if (col.norm() > 3.f) {	// if emmisive pixel
					color = col;
				}
			}
		}
		return res;
	}

	Texture add(Texture*img1, Texture* img2){
		Texture src1 = *img1;
		Texture src2 = *img2;

		Texture res = src1;


		for (int x = 0; x < res.width; x++) {
			for (int y = 0; y < res.height; y++) {
				Vector3f& color = res.rgb.at(y * res.width + x);
				color.x += src2.rgb.at(y * res.width + x).x;
				color.y += src2.rgb.at(y * res.width + x).y;
				color.z += src2.rgb.at(y * res.width + x).z;
			}
		}
		return res;
	}
};
