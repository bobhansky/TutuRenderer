﻿#pragma once

#include "global.hpp"
#include "Vector.hpp"



enum MaterialType {
	LAMBERTIAN,	// cosine weighted
	PERFECT_REFLECTIVE,
	PERFECT_REFRACTIVE,
	MICROFACET,	// Cook Torrance Microfacet model  with GGX dist
	UNLIT
};


class Material {
public:
	Vector3f diffuse = Vector3f(0.9f, 0.9f, 0.9f);
	Vector3f specular = Vector3f(1.f);
	Vector3f emission = Vector3f(0.f);
	MaterialType mType = LAMBERTIAN;

	float alpha = 1;		// opacity		if alpha = 1. then no refraction 
	float eta = 1;			// index of refraction

	float roughness = 1;	// width parameter  alpha_g
	float metallic = 0;


	// copy assignment operator
	Material& operator= (const Material& other) {
		if (this != &other) {
			this->diffuse.x = other.diffuse.x;
			this->diffuse.y = other.diffuse.y;
			this->diffuse.z = other.diffuse.z;
			this->specular.x = other.specular.x;
			this->specular.y = other.specular.y;
			this->specular.z = other.specular.z;
			this->emission = other.emission;
			this->mType = other.mType;

			this->alpha = other.alpha;
			this->eta = other.eta;

			this->roughness = other.roughness;
			this->metallic = other.metallic;
		}
		return *this;
	}

	bool hasEmission() {
		return emission.x || emission.y || emission.z;
	}

	// BxDF, return vec3f of elements within [0,1] 
	// wi, wo: origin at inter.pos center, pointing outward
	// wi: incident ray
	// wo: view dir
	Vector3f BxDF(Vector3f& wi, Vector3f& wo, Vector3f& N, float eta_scene) {
		switch (mType) {
		case LAMBERTIAN: {
			float cos_theta = wo.dot(N);
			// account for reflection contribution only
			if (cos_theta > 0.f) {
				return diffuse / M_PI;
			}
			else return Vector3f(0.f);
			break;
		}

		case MICROFACET: {
			// Cook-Torrance Model

			Vector3f h = normalized(wi + wo);
			float costheta = h.dot(wo);

			Vector3f F0(0.04f);	// should be 0.04			
			F0 = lerp(F0, this->diffuse, this->metallic);
			Vector3f F = fresnelSchlick(costheta, F0);	// learnopgl https://learnopengl.com/PBR/Theory
			// float F = fresnel(-wi, h, eta_scene, this->eta);
			float D = D_ndf(h, N, roughness);
			//float G = GeometrySmith(N, wi, wo, (roughness + 1) * (roughness + 1) / 8);	// learnopgl
			float G = G_smf(wi, wo, N, roughness);
			Vector3f fr = (F * G * D) / (4 * wi.dot(N) * wo.dot(N));	// originaly float fr

			//fr = clamp(0, 1, fr);
			Vector3f diffuse_term = (1.f - F) * (diffuse / M_PI);
			Vector3f ref_term = fr;
			// return ref_term;	// used for MIS testing
			return diffuse_term + ref_term;
		}

		case PERFECT_REFLECTIVE:{
			if (FLOAT_EQUAL(normalized(wi + wo).dot(N), 1.f))
				// https://www.youtube.com/watch?v=sg2xdcB8M3c
				return 1 / N.dot(wi);
			return 0;
			break;
		}

		case PERFECT_REFRACTIVE: {	// todo list
			// https://www.youtube.com/watch?v=sg2xdcB8M3c
			Vector3f refDir = normalized(getReflectionDir(wo, N));
			float eta_i = eta_scene;
			float eta_t = this->eta;
			float F;
			if (wo.dot(N) < 0) {
				N = -N;
				std::swap(eta_i, eta_t);
				F = fresnel(wi, N, eta_i, eta_t);
			}
			else F = fresnel(wi, N, eta_i, eta_t);
			Vector3f transDir = normalized(getRefractionDir(wo, N, eta_i, eta_t));


			if (FLOAT_EQUAL(wi.dot(refDir), 1.f))
				return F * 1 / abs(N.dot(wi));
			else if(FLOAT_EQUAL(wi.dot(transDir), 1.f)) 
				return (std::pow(eta_t, 2)/ std::pow(eta_i, 2)) * (1 - F) * abs(1 / N.dot(wi));
			
			return Vector3f(0.f);
			break;			
		}

		default:
			return Vector3f(0.f);
		}
	}



	// sample a direction on the hemisphere
	// wi: incident dir, pointing outward
	// when passed in, eta_i is always eta_world
	bool sampleDirection(const Vector3f& wi, const Vector3f& N, Vector3f& sampledRes, float eta_i = 0.f) {
		switch (mType)
		{
		case MICROFACET:
		{
			if (wi.dot(N) <= 0.0f) 
				return false;		// crucial
			
			// https://zhuanlan.zhihu.com/p/78146875
			// https://agraphicsguynotes.com/posts/sample_microfacet_brdf/
			float r0 = getRandomFloat();
			float r1 = getRandomFloat();
			float a2 = roughness * roughness * roughness * roughness;
			float phi = 2 * M_PI * r1;
			float theta = std::acos(sqrt((1 - r0) / (r0 * (a2 - 1) + 1)));

			float r = std::sin(theta);
			Vector3f h = normalized(Vector3f(r * std::cos(phi), r * std::sin(phi), std::cos(theta)));
			Vector3f res = getReflectionDir(wi, SphereLocal2world(N, h));
			res = normalized(res);
			if (res.dot(N) <= 0)	// very crucial for white noise ??
				return false;

			sampledRes = res;
			return true;
			break;
		}

		case LAMBERTIAN: {
			if (wi.dot(N) <= 0.0f)
				return false;
			// cosine weighted
			// **** inverse transformation sampling
			// pbrt 13.6.1  *important
			// https://pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations
			// https://pbr-book.org/3ed-2018/Monte_Carlo_Integration/Transforming_between_Distributions
			// 
			// 3.6 Approximating Distributions
			// https://raytracing.github.io/books/RayTracingTheRestOfYourLife.html#generatingrandomdirections/uniformsamplingahemisphere
			// https://www.youtube.com/watch?v=rnBbYsysPaU&t=1s

			// 1. generate a random direction in sphere coordinate
			// 2. convert it to world corrdinate
			float r1 = getRandomFloat();
			float r2 = getRandomFloat();
			float cosTheta = sqrt(r1);
			float phi = 2 * M_PI * r2;

			Vector3f dir;
			float sinTheta = sqrtf(std::max(0.f, 1.f - powf(r1, 2)));
			dir.x = cos(phi) * sinTheta;
			dir.y = sin(phi) * sinTheta;
			dir.z = cosTheta;

			dir = normalized(dir);

			Vector3f res = SphereLocal2world(N, dir);
			if (normalized(res).dot(N) < 0)
				return false;

			sampledRes = res;
			return true;
			break;
		}

		case PERFECT_REFLECTIVE: {
			sampledRes = getReflectionDir(wi, N);
			return true;
			break;
		}
		case PERFECT_REFRACTIVE: {
			float eta_t = eta;
			Vector3f interN = N;
			if (wi.dot(N) < 0) {
				std::swap(eta_i, eta_t);
				interN = -interN;
			}

			float F = fresnel(wi, interN, eta_i, eta_t);
			if (getRandomFloat() < F) {
				sampledRes = getReflectionDir(wi, interN);
			}
			else sampledRes = getRefractionDir(wi, interN, eta_i, eta_t);

			return true;
			break;
		}
		default: {
			return false;
			break;
		}
		}

	}


	Vector3f SphereLocal2world(const Vector3f& n, const Vector3f& dir) {
		// https://raytracing.github.io/books/RayTracingTheRestOfYourLife.html#generatingrandomdirections/uniformsamplingahemisphere
		// 8. orthonormal basis
		// change of basis 

		// x y z local coordinates to s t n coordinates
		Vector3f a;
		Vector3f N = normalized(n);	//z
		// construct an Orthonalmal basis
		// randomly choose an a that is not parallel to N
		if (fabs(N.x) > 0.9f)
			a = { 0.f, 1.f, 0.f };
		else a = { 1.f, 0.f, 0.f };
		//Vector3f T = crossProduct(a, N); // y   X cross Y == Z      then S cross T should == N
		//Vector3f S = crossProduct(T, N); // x
		// ******** 
		// 2/21/2024 IMPORTANT
		// 2 unit vectors cross product doens't guarantee to produce unit vec, unless they are orthogonal
		// Vector3f S = crossProduct(N, a);		// reason for wrong result
		Vector3f S = normalized(crossProduct(N, a));
		Vector3f T = crossProduct(N, S);

		return normalized(dir.x * S + dir.y * T + dir.z * N);
	}

	// wo: -camera dir   wi: sampled dir
	// when passed in, eta_i is always eta_world, eta_t is always ior of inter.material
	float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float eta_i = 0.f, float eta_t = 0.f) const {
		switch (mType)
		{
		case LAMBERTIAN: {
			// uniform sample probability 1 / (2 * PI)
			if (wi.dot(N) > 0.0f)
				return wi.dot(N) / M_PI;  // cosine weighted pdf https://ameye.dev/notes/sampling-the-hemisphere/
			else
				return 0.0f;

			break;
		}
		case MICROFACET: {
			// corresponds to normal distribution function D
			// https://www.tobias-franke.eu/log/2014/03/30/notes_on_importance_sampling.html
			Vector3f h = normalized(wo + wi);
			float cosTheta = N.dot(h);
			cosTheta = std::max(cosTheta, 0.f);

			return D_ndf(h, N, roughness) * cosTheta / (4.f * wo.dot(h));
			break;
		}
		case PERFECT_REFLECTIVE: {
			if(FLOAT_EQUAL(normalized(wi+wo).dot(N), 1.f))
				return 1;
			 return 0;
			break;
		}

		case PERFECT_REFRACTIVE: {
			Vector3f refDir = normalized(getReflectionDir(wo, N));
			Vector3f nDir = N;
			if (wo.dot(nDir) < 0) {
				std::swap(eta_i, eta_t);
				nDir = -N;
			}
			Vector3f transDir = normalized(getRefractionDir(wo, nDir, eta_i, eta_t));

			float F = fresnel(wo, nDir, eta_i, eta_t);

			// check which direction is sampled 
			if (FLOAT_EQUAL(wi.dot(refDir), 1.f))
				return F;
			else if (FLOAT_EQUAL(wi.dot(transDir), 1.f))
				return 1 - F;

			return 0;
			break;
		}

		default:
			return 1;
			break;
		}
	}
};