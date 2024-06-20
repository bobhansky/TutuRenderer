#pragma once

#include "global.hpp"
#include "Vector.hpp"
#include <tuple>



enum MaterialType {
	LAMBERTIAN,	// cosine weighted
	PERFECT_REFLECTIVE,
	PERFECT_REFRACTIVE,
	MICROFACET_R,	// MICROFACET_REFLECTIVE, Cook Torrance Microfacet model  with GGX dist
	MICROFACET_T,	// MICROFACET_TRANSMISSIVE
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
	Vector3f BxDF(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float eta_scene, bool TIR = false) {
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

		case MICROFACET_R: {
			// Cook-Torrance Model

			Vector3f h = normalized(wi + wo);
			float costheta = h.dot(wo);

			Vector3f F0(0.04f);	// should be 0.04			
			F0 = lerp(F0, this->diffuse, this->metallic);
			Vector3f F = fresnelSchlick(costheta, F0);	// learnopgl https://learnopengl.com/PBR/Theory
			// float F = fresnel(wo, h, eta_scene, this->eta);
			float D = D_ndf(h, N, roughness);
			//float G = GeometrySmith(N, wi, wo, (roughness + 1) * (roughness + 1) / 8);	// learnopgl
			float G = G_smf(wi, wo, N, roughness, h);
			Vector3f fr = (F * G * D) / (4 * wi.dot(N) * wo.dot(N));	// originaly float fr

			//fr = clamp(0, 1, fr);
			Vector3f diffuse_term = (1.f - F) * (diffuse / M_PI);
			Vector3f ref_term = fr;
			// return ref_term;	// used for MIS testing
			return diffuse_term + ref_term;
		}
		case MICROFACET_T: {
			float eta_i = eta_scene;
			float eta_t = eta;
			Vector3f interN = N;
			if (wo.dot(N) < 0) {
				interN = -N;
				std::swap(eta_i, eta_t);
			}
			float F = fresnel(wo, interN, eta_i, eta_t);

			// if wi is reflection dir
			if (wi.dot(interN) >= 0) {
				Vector3f h = normalized(wo + wi);
				float cosTheta = h.dot(wo);
				cosTheta = abs(cosTheta);
				float F = fresnel(wo, h, eta_i, eta_t);
				if (TIR) 
					F = 1.f;
				float D = D_ndf(h, interN, roughness);
				float G = G_smf(wi, wo, interN, roughness, h);
				Vector3f fr = (F * G * D) / (4 * wi.dot(interN) * wo.dot(interN));	// originaly float fr
				return fr;
			}
			// wi is refraction dir
			else {	// need h and jacobian 
				Vector3f h = -normalized(eta_i * wo + eta_t * wi);
				if (h.dot(interN) < 0) h = -h;
				float cos_ih = wi.dot(h), cos_oh = wo.dot(h), 
					cos_in = wi.dot(interN), cos_on = wo.dot(interN);
				float F = fresnel(wo, h, eta_i, eta_t);
				float D = D_ndf(h, interN, roughness);
				float G = G_smf(wi, wo, interN, roughness, h);
				float numerator = abs(cos_ih) * abs(cos_oh) * eta_t * eta_t * (1 - F) * G * D;
				float denominator = abs(cos_in) * abs(cos_on) * std::powf(eta_i * cos_ih + eta_t * cos_oh, 2);
				return numerator / denominator;
			}
		}

		case PERFECT_REFLECTIVE:{
			if (FLOAT_EQUAL(normalized(wi + wo).dot(N), 1.f))
				// https://www.youtube.com/watch?v=sg2xdcB8M3c
				return 1 /abs(N.dot(wi));
			return 0;
			break;
		}

		case PERFECT_REFRACTIVE: {
			// https://www.youtube.com/watch?v=sg2xdcB8M3c
			// https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#fragment-BxDFDeclarations-7
			Vector3f refDir = normalized(getReflectionDir(wo, N));
			float eta_i = eta_scene;
			float eta_t = this->eta;
			float F;
			Vector3f interN = N;
			if (wo.dot(N) < 0) {
				interN = -N;
				std::swap(eta_i, eta_t);
			}
			F = fresnel(wo, interN, eta_i, eta_t);
			Vector3f transDir = normalized(getRefractionDir(wo, interN, eta_i, eta_t));

			interN = interN.dot(wi) < 0 ? -interN : interN;

			if (TIR) 
				return 1 / interN.dot(wi);
			if (FLOAT_EQUAL(wi.dot(refDir), 1.f))
				return F * 1 / interN.dot(wi);
			else if(FLOAT_EQUAL(wi.dot(transDir), 1.f)) 
				// 5/13/2024  this term make some faces bright? idk if it's correct
				return (eta_t * eta_t) / (eta_i * eta_i) * (1 - F) * 1 / interN.dot(wi);
			
			return Vector3f(0.f);
			break;			
		}

		default:
			return Vector3f(0.f);
		}
	}



	// sample a direction on the hemisphere, changes the value of "sampledRes"
	// wi: incident dir, pointing outward
	// when passed in, eta_i is always eta_world
	// returns: first bool for sample success, true for succeed
	//			second bool for special event happening, 1 for happened
	std::tuple<bool, bool> sampleDirection(const Vector3f& wi, const Vector3f& N, Vector3f& sampledRes, float eta_i = 1.f) {
		switch (mType)
		{
		case MICROFACET_R: {
			if (wi.dot(N) <= 0.0f) 
				return { false, false };		// crucial
			
			// https://zhuanlan.zhihu.com/p/78146875
			// https://agraphicsguynotes.com/posts/sample_microfacet_brdf/
			float r0 = getRandomFloat();
			float r1 = getRandomFloat();
			float alhpa = roughness * roughness;
			alpha = std::max(alpha, 1e-3f);
			float a2 = alhpa * alpha;
			
			float phi = 2 * M_PI * r1;
			float theta = std::acos(sqrt((1 - r0) / (r0 * (a2 - 1) + 1)));

			float r = std::sin(theta);
			Vector3f h = normalized(Vector3f(r * std::cos(phi), r * std::sin(phi), std::cos(theta)));
			Vector3f res = getReflectionDir(wi, SphereLocal2world(N, h));
			res = normalized(res);
			if (res.dot(N) <= 0)	// actually handled in shadow masking term, but only for specular term
				return {false, false};

			sampledRes = res;
			return { true, false };
			break;
		}
		case MICROFACET_T: {
			float r0 = getRandomFloat();
			float r1 = getRandomFloat();
			float a = roughness * roughness;
			a = std::max(a, 1e-3f);
			float a2 = a * a;

			float phi = 2 * M_PI * r1;
			float theta = std::acos(sqrt((1 - r0) / (r0 * (a2 - 1) + 1)));

			float r = std::sin(theta);
			Vector3f h = normalized(Vector3f(r * std::cos(phi), r * std::sin(phi), std::cos(theta)));
			float eta_t = eta;
			Vector3f interN = N;
			if (wi.dot(N) < 0) {
				std::swap(eta_i, eta_t);
				interN = -interN;
			}
			h = SphereLocal2world(interN, h);
			
			Vector3f res = getRefractionDir(wi, h, eta_i, eta_t);
			// if TIR
			if (res.norm2() == 0) {
				return { true, true };	// outside will handle sampleDir
			}

			float F = fresnel(wi, h, eta_i, eta_t);
			// choose refl or trans
			if (getRandomFloat() < F) {	
				sampledRes = getReflectionDir(wi, h);
			}
			else 
				sampledRes = res;

			return { true, false };
			break;
		}

		case LAMBERTIAN: {
			if (wi.dot(N) <= 0.0f)
				return {false, false};
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
			float sinTheta = sqrtf(std::max(0.f, 1.f - r1));
			dir.x = cos(phi) * sinTheta;
			dir.y = sin(phi) * sinTheta;
			dir.z = cosTheta;

			dir = normalized(dir);

			Vector3f res = SphereLocal2world(N, dir);
			if (normalized(res).dot(N) < 0)
				return { false, false };

			sampledRes = res;
			return { true,false };
			break;
		}

		case PERFECT_REFLECTIVE: {
			sampledRes = getReflectionDir(wi, N);
			return { true,false };
			break;
		}
		case PERFECT_REFRACTIVE: {
			float eta_t = eta;
			Vector3f interN = N;
			if (wi.dot(N) < 0) {
				std::swap(eta_i, eta_t);
				interN = -interN;
			}

			Vector3f res = getRefractionDir(wi, interN, eta_i, eta_t);
			// if TIR
			if (res.norm2() == 0) {
				return { true, true };	// outside will handle sampleDir
			}

			float F = fresnel(wi, interN, eta_i, eta_t);
			if (getRandomFloat() < F) {
				sampledRes = getReflectionDir(wi, interN);
			}
			else sampledRes = res;

			return { true,false };
			break;
		}
		default: {
			return { false,false };
			break;
		}
		}

	}


	

	// wo: -camera dir   wi: sampled dir
	// when passed in, eta_i is always eta_world, eta_t is always ior of inter.material
	float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float eta_i = 1.f, float eta_t = 1.f) const {
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
		case MICROFACET_R: {
			// corresponds to normal distribution function D
			// https://www.tobias-franke.eu/log/2014/03/30/notes_on_importance_sampling.html
			Vector3f h = normalized(wo + wi);
			float cosTheta = N.dot(h);
			cosTheta = std::max(cosTheta, 0.f);

			return D_ndf(h, N, roughness) * cosTheta / (4.f * wo.dot(h));
			break;
		}
		case MICROFACET_T: {
			Vector3f interN = N;
			if (wo.dot(N) < 0) {
				interN = -N;
				std::swap(eta_i, eta_t);
			}
			float F = fresnel(wo, interN, eta_i, eta_t);

			// if wi is reflection dir
			if (wi.dot(interN) >= 0) {
				Vector3f h = normalized(wo + wi);
				float cosTheta = interN.dot(h);
				cosTheta = abs(cosTheta);
				return F *  D_ndf(h, interN, roughness) * cosTheta / (4.f * wo.dot(h));
			}
			// wi is refraction dir
			else {	// need h and jacobian 
				Vector3f h = -normalized(eta_i * wo + eta_t * wi);
				float cosTheta = interN.dot(h);
				if (cosTheta < 0) {
					h = -h;
					cosTheta = abs(cosTheta);
				}
				float denominatorSqrt = eta_i * wi.dot(h) + eta_t * wo.dot(h);
				float jacobian = (eta_t * eta_t * abs(wo.dot(h)))/ (denominatorSqrt * denominatorSqrt);
				return (1 - F) *  D_ndf(h, interN, roughness) * cosTheta * jacobian;
			}
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