#pragma once

#include<mutex>
#include<omp.h>

#include "Vector.hpp"
#include "global.hpp"
#include "PPMGenerator.hpp"
#include "IIntersectStrategy.h"



std::mutex sampleLight_mutex;
omp_lock_t light_lock_omp;
std::vector<std::string> records;	// ray information, thread independent string

class IIntegrator {
public:
	virtual void integrate(PPMGenerator* g) = 0;

public:
	IIntersectStrategy* interStrategy;
	PPMGenerator* g;
};


void changeNormalDir(Intersection& inter, PPMGenerator* g) {
	Texture* nMap = g->normalMaps.at(inter.normalMapIndex);
	Vector3f color = nMap->getRGBat(inter.textPos.x, inter.textPos.y);

	switch (inter.obj->objectType)
	{
	case TRIANGLE: {
		Triangle* t = static_cast<Triangle*>(inter.obj);
		// our triangle start from lower left corner and go counterclockwise

		Vector3f e1 = t->v1 - t->v0;
		Vector3f e2 = t->v2 - t->v0;
		Vector3f nDir = crossProduct(e1, e2); // note the order!
		nDir = normalized(nDir);

		float deltaU1 = t->uv1.x - t->uv0.x;
		float deltaV1 = t->uv1.y - t->uv0.y;

		float deltaU2 = t->uv2.x - t->uv0.x;
		float deltaV2 = t->uv2.y - t->uv0.y;

		float coef = 1 / (-deltaU1 * deltaV2 + deltaV1 * deltaU2);

		Vector3f T = coef * (-deltaV2 * e1 + deltaV1 * e2);
		Vector3f B = coef * (-deltaU2 * e1 + deltaU1 * e2);
		T = normalized(T);
		B = normalized(B);

		Vector3f res;
		res.x = T.x * color.x + B.x * color.y + nDir.x * color.z;
		res.y = T.y * color.x + B.y * color.y + nDir.y * color.z;
		res.z = T.z * color.x + B.z * color.y + nDir.z * color.z;

		inter.nDir = normalized(res);
		break;
	}

	case SPEHRE: {
		Vector3f nDir = inter.nDir;
		Vector3f T = Vector3f(-nDir.y / sqrtf(nDir.x * nDir.x + nDir.y * nDir.y),
			nDir.x / sqrtf(nDir.x * nDir.x + nDir.y * nDir.y), 0);

		Vector3f B = crossProduct(nDir, T);

		Vector3f res;
		res.x = T.x * color.x + B.x * color.y + nDir.x * color.z;
		res.y = T.y * color.x + B.y * color.y + nDir.y * color.z;
		res.z = T.z * color.x + B.z * color.y + nDir.z * color.z;

		inter.nDir = normalized(res);
		break;
	}

	default:
		break;
	}

}

void textureModify(Intersection& inter, PPMGenerator* g) {
	// DIFFUSE
	if (!FLOAT_EQUAL(-1.f, inter.diffuseIndex)) {
		if (g->diffuseMaps.size() <= inter.diffuseIndex) {
			std::cout <<
				"\ninter.diffuseIndex is greater than diffuseTexuture.size()\nImport texture files in config.txt \n";
			exit(1);
		}
		inter.mtlcolor.diffuse = g->diffuseMaps.at(inter.diffuseIndex)
			->getRGBat(inter.textPos.x, inter.textPos.y);
	}
	// NORMAL
	if (inter.normalMapIndex != -1) {
		changeNormalDir(inter, g);
	}

	// ROUGHNESS
	if (inter.roughnessMapIndex != -1) {
		if (g->roughnessMaps.size() <= inter.diffuseIndex) {
			std::cout <<
				"\ninter.roughnessIndex is greater than roughness_texture.size()\nImport texture files in config.txt \n";
			exit(1);
		}
		inter.mtlcolor.roughness = g->roughnessMaps.at(inter.roughnessMapIndex)
			->getRGBat(inter.textPos.x, inter.textPos.y).x;

	}

	// METALLIC
	if (inter.metallicMapIndex != -1) {
		if (g->metallicMaps.size() <= inter.metallicMapIndex) {
			std::cout <<
				"\ninter.metallicIndex is greater than metallic_texture.size()\nImport texture files in config.txt \n";
			exit(1);
		}
		inter.mtlcolor.metallic = g->metallicMaps.at(inter.metallicMapIndex)
			->getRGBat(inter.textPos.x, inter.textPos.y).x;
	}
}

/// <summary>
/// shadow ray
/// </summary>
/// <param name="p">inter information</param>
/// <param name="lightPos">light position</param>
/// <returns>if the ray is blocked obj, return true</returns>
bool isShadowRayBlocked(Vector3f orig, Vector3f& lightPos, PPMGenerator* g) {
	Vector3f raydir = normalized(lightPos - orig);
	float distance = (lightPos - orig).norm();

	if (!EXPEDITE) {
		Intersection p_light_inter;
		for (auto& i : g->scene.objList) {
			//if (i->mtlcolor.hasEmission()) continue; // do not test with light avatar

			if (i->intersect(orig, raydir, p_light_inter) && p_light_inter.t < distance) {
				return true;
			}
		}
		return false;
	}
	else { // BVH intersection test
		return hasIntersection(g->scene.BVHaccelerator->getNode(), orig, raydir, distance);
	}
}

float getLightPdf(Intersection& inter, PPMGenerator* g) {
	if (!inter.intersected) return 0;

	static std::vector<Object*> lightList;
	static float totalArea = 0;
	static bool firstimeCall = true;
	int size = lightList.size();

	// if first time call it, put all the emissive object into lightList

	if (firstimeCall) {
		// 3/2/2024: need lock
#if MULTITHREAD==1
		sampleLight_mutex.lock();
#elif MULTITHREAD == 2
		omp_set_lock(&light_lock_omp);
#endif
		for (auto& i : g->scene.objList) {
			if (!firstimeCall)
				break;

			if (i->mtlcolor.hasEmission()) {
				lightList.emplace_back(i.get());
				// without lock, sometimes problem on i->getArea(), maybe due to unique_ptr
				totalArea += i->getArea();
			}
		}
		firstimeCall = false;
#if MULTITHREAD==1
		sampleLight_mutex.unlock();
#elif  MULTITHREAD == 2
		omp_unset_lock(&light_lock_omp);
#endif;
	}

	size = lightList.size();
	// if there's no light
	if (size == 0) {
		return 0;
	}
	if (!inter.obj->mtlcolor.hasEmission()) return 0;

	// independent event p(a&&b) == p(a) *  p(b)
	float area = inter.obj->getArea();
	return  1 / (size * area);
}

// sample all the emissive object to get one point on their surface,
// update the intersection, and the pdf to sample it
// pdf of that inter is, 1/area of THE object surface area
void sampleLight(Intersection& inter, float& pdf, PPMGenerator* g) {
	static std::vector<Object*> lightList;
	static float totalArea = 0;
	static bool firstimeCall = true;
	int size = lightList.size();

	// if first time call it, put all the emissive object into lightList

	if (firstimeCall) {
		// 3/2/2024: need lock
#if MULTITHREAD == 1
		sampleLight_mutex.lock();
#elif MULTITHREAD == 2
		omp_set_lock(&light_lock_omp);
#endif
		for (auto& i : g->scene.objList) {
			if (!firstimeCall)
				break;

			if (i->mtlcolor.hasEmission()) {
				lightList.emplace_back(i.get());
				totalArea += i->getArea();	// without lock, sometimes problem on i->getArea(), maybe due to unique_ptr
			}
		}
		firstimeCall = false;
#if MULTITHREAD == 1
		sampleLight_mutex.unlock();
#elif MULTITHREAD == 2
		omp_unset_lock(&light_lock_omp);
#endif
	}

	size = lightList.size();
	// if there's no light
	if (size == 0) {
		inter.intersected = false;
		pdf = 0;
		return;
	}

	// 3/3/2024 need a better sample method
	int index = (int)(getRandomFloat() * (size - 1) + 0.4999f);
	if (size == 1) index = 0;

	Object* lightObject = lightList.at(index);

	lightObject->samplePoint(inter, pdf);
	// independent event p(a&&b) == p(a) *  p(b)
	pdf = (1.f / (size * lightObject->getArea()));
}


bool sampleLightDir(Vector3f& N, float& dirPdf, Vector3f& sampledRes) {
	// cos-weighted
	float r1 = getRandomFloat();
	float r2 = getRandomFloat();
	float cosTheta = sqrt(r1);
	float phi = 2 * M_PI * r2;

	Vector3f dir;
	float sinTheta = sqrt(std::max(0.f, 1 - r1));
	dir.x = cos(phi) * sinTheta;
	dir.y = sin(phi) * sinTheta;
	dir.z = cosTheta;

	dir = normalized(dir);

	Vector3f res = SphereLocal2world(N, dir);
	if (normalized(res).dot(N) < 0)
		return false;

	dirPdf = 0.f;
	if (res.dot(N) > 0.0f)
		dirPdf = res.dot(N) / M_PI;

	sampledRes = res;
	return true;
}

// geometry term
float Geo(Vector3f& p1, const Vector3f& n1, Vector3f& p2, const Vector3f& n2) {
	Vector3f p12p2 = p2 - p1;
	float dis2 = p12p2.norm2();
	p12p2 = normalized(p12p2);
	float cos =abs(p12p2.dot(n1));
	float cosprime = abs((-p12p2).dot(n2));
	return cos * cosprime / dis2;
}

// importance function
float We(Intersection& inter, Camera& cam) {
	Vector3f camPos = cam.position;
	Vector3f inter2cam = camPos - inter.pos;
	inter2cam = normalized(inter2cam);

	// check if inter is in the frustum
	int index = cam.worldPos2PixelIndex(inter.pos);
	if (index < 0 || index >= cam.width * cam.height) {
		return 0.f;
	}

	float cosCamera = abs(cam.fwdDir.dot(-inter2cam));
	float distPixel2Cam = cam.imagePlaneDist / cosCamera;

	return distPixel2Cam * distPixel2Cam * cam.lensAreaInv * cam.filmPlaneAreaInv / (cosCamera * cosCamera);
}