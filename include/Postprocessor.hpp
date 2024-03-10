#pragma once

#include<thread>
#include<mutex>
#include <math.h>

#include "PPMGenerator.hpp"
#include "Texture.hpp"

#define STRENGTH 2
#define GAUSSIANLOOP 2
#define KERNELSIZE 20
#define STDDEV 60.f
#define EXPOSURE 1.f

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
		renderTextures[index] = getGaussianBlurTexture(&renderTextures[1], KERNELSIZE, STDDEV);
		index = 3;
		for (int i = 0; i < GAUSSIANLOOP; i++) {
			renderTextures[index] = getGaussianBlurTexture(&renderTextures[index == 2 ? 3 : 2], KERNELSIZE, STDDEV);
			index = index == 2 ? 3 : 2;
		}
		//	1.3 add raw img to blured
		index = index == 2 ? 3 : 2;
		// return renderTextures[index];		// return blured emmisive texture
		renderTextures[index] = add(&renderTextures[0], &renderTextures[index]);	// bloom 

#ifndef HDR
		return renderTextures[index];
#endif // !HDR
#ifdef HDR
		return getHDRtexture(&renderTextures[index]);
#endif
		
		
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
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
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
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
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

		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				Vector3f& color = res.rgb.at(y * res.width + x);

				float U = (float)x / w;
				float V = (float)y / h;
				Vector3f col = src.getRGBat(clamp(0, 0.999f, U), clamp(0, 0.999f, V));
				if (col.norm() > 3.f) {	// if emmisive pixel
					float mx = col.x > col.y ? col.x : col.y;
					mx = mx > col.z ? mx : col.z;
					color.x = rescale(col.x, mx, 0.f, STRENGTH, 0.f);
					color.y = rescale(col.y, mx, 0.f, STRENGTH, 0.f);
					color.z = rescale(col.z, mx, 0.f, STRENGTH, 0.f);
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


	// exposure tone mapping  https://learnopengl.com/Advanced-Lighting/HDR
	// Tone mapping is the process of transforming floating point color values 
	// to the expected [0.0, 1.0] range known as low dynamic range without losing too much detail, 
	// often accompanied with a specific stylistic color balance.
	Texture getHDRtexture(Texture* img) {
		float gamma = 2.2f;
		Texture	src = *img;
		Texture dst;
		dst.width = src.width;
		dst.height = src.height;
		dst.rgb.resize(dst.width * dst.height);

		for (int y = 0; y < dst.height; y++) {
			for (int x = 0; x < dst.width; x++) {
				Vector3f& color = dst.rgb.at(y * src.width + x);

				float U = (float)x / dst.width;
				float V = (float)y / dst.height;
				Vector3f hdrcolor = src.getRGBat(clamp(0, 0.999f, U), clamp(0, 0.999f, V));
				// exposure tone mapping
				Vector3f mapped;
				mapped.x = 1 - exp(-hdrcolor.x * EXPOSURE);
				mapped.y = 1 - exp(-hdrcolor.y * EXPOSURE);
				mapped.z = 1 - exp(-hdrcolor.z * EXPOSURE);

				color = mapped;
			}
		}
		return dst;
	}
};
