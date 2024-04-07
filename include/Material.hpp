#pragma once

#include "global.hpp"
#include "Vector.hpp"



enum MaterialType {
	LAMBERTIAN,	// cosine weighted
	SPECULAR_REFLECTIVE,
	PERFECT_REFRACTIVE,
	MICROFACET,	// Cook Torrance Microfacet model  with GGX dist
	UNLIT
};


class Material {
public:
	Vector3f diffuse =  Vector3f(0.9f, 0.9f, 0.9f);
	Vector3f specular = Vector3f(1.f);
	Vector3f emission = Vector3f(0.f);
	MaterialType mType = LAMBERTIAN;
	float ka = 0;
	float kd = 0;
	float ks = 0;			// if ks = 0, then the material is non-reflective
	float n = 0;			// highlights shininess power coefficient

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

			this->ka = other.ka;
			this->kd = other.kd;
			this->ks = other.ks;
			this->n = other.n;

			this->alpha = other.alpha;
			this->eta = other.eta;

			this->roughness = other.roughness;
			this->metallic = other.metallic;
		}
		return *this;
	}

	bool hasEmission() {
		return emission.x || emission.y || emission.z ;
	}

	// BxDF, return vec3f of elements within [0,1] 
	// wi, wo: origin at inter.pos center, pointing outward
	// wi: incident ray
	// wo: view dir
	Vector3f BxDF( Vector3f& wi,  Vector3f& wo, Vector3f& N, float eta_scene) {
		switch (mType) {
			case LAMBERTIAN: {
				float cos_theta =wo.dot(N);
				// account for reflection contribution only
				if (cos_theta > 0.f) {
					return diffuse / M_PI;
				}
				else return Vector3f(0.f);
				break;
			}
			
			case MICROFACET: {
				// Cook-Torrance Model
				
				// 1/11/2024
				// need to check N dot wi to see if the incident light is inside obj or outside
				// only for indirect illumination
				
				// ************** Reflection erm ********************
				Vector3f h = normalized(wi + wo);
				float costheta = h.dot(wo);

				Vector3f F0(0.04f);	// should be 0.04			
				F0 = lerp(F0, this->diffuse, this->metallic);
				Vector3f F = fresnelSchlick(costheta, F0);	// learnopgl https://learnopengl.com/PBR/Theory
				// float F = fresnel(-wi, h, eta_scene, this->eta);
				float D = D_ndf(h, N, roughness);
				//float G = GeometrySmith(N, wi, wo, (roughness + 1) * (roughness + 1) / 8);	// learnopgl
				float G = G_smf(wi, wo, N, roughness);
				Vector3f fr = (F * G * D) / ((4 * wi.dot(N)) * wo.dot(N));	// originaly float fr

				fr = clamp(0, 1, fr);	
				Vector3f diffuse_term = (1.f - F) * diffuse / M_PI;
				Vector3f ref_term =  fr * 1;		// or 1
				return diffuse_term + ref_term;
				
				
				//************** Reflection term ends ********************
			}

			case SPECULAR_REFLECTIVE:
			{
				return 1;
				break;
			}

			case PERFECT_REFRACTIVE:{
				return 1;
				break;
			}
			
			default:
				return Vector3f(0.f);
		}
	}



	// sample a direction on the hemisphere
	// wi: incident dir, pointing outward
	Vector3f sampleDirection(const Vector3f& wi, const Vector3f& N, 
		const float eta_i = 0.f, const float eta_t = 0.f) {

		switch (mType)
		{
		case MICROFACET: 
		{
			// https://zhuanlan.zhihu.com/p/78146875
			// https://agraphicsguynotes.com/posts/sample_microfacet_brdf/

			float r0 = getRandomFloat();
			float r1 = getRandomFloat();
			float a2 = roughness * roughness *roughness* roughness;
			float phi = 2 * M_PI * r1;
			float theta = std::acos(sqrt((1 - r0) / (r0 * (a2 - 1) + 1)));

			float r = std::sin(theta);
			return getReflectionDir(-wi, SphereLocal2world(N, Vector3f(r * std::cos(phi), r * std::sin(phi), std::cos(theta))));
			break;
		}

		case LAMBERTIAN: {
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
			float sinTheta = sqrtf(std::max(0.f, 1.f - powf(r1,2)));
			dir.x = cos(phi) * sinTheta;
			dir.y = sin(phi) * sinTheta;
			dir.z = cosTheta;

			dir = normalized(dir);
			
			return SphereLocal2world(N, dir);
			break;
		}

		case SPECULAR_REFLECTIVE: {
			return getReflectionDir(-wi, N);
			break;
		}
		case PERFECT_REFRACTIVE: {
			float F = fresnel(-wi, N, eta_i, eta);
			if (getRandomFloat() < F) {
				return getReflectionDir(-wi, N); 
			}
			else return getRefractionDir(-wi, N, eta_i, eta_t);

			break;
		}
		default: {
			return Vector3f(0.f);
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
		

		// tangent and binormal??  2/21/2024 still could not understand
		// https://tutorial.math.lamar.edu/classes/calcII/tangentnormalvectors.aspx
		// https://learnopengl.com/Advanced-Lighting/Normal-Mapping
		// transform local normal (0,0,1) to world normal (N.x, N.y, N.z)
		// nori: coordinateSystem
		// https://github.com/wjakob/nori/blob/master/src/common.cpp
		//Vector3f B, C;
		//if (std::fabs(n.x) > std::fabs(n.y)) {
		//	float invLen = 1.0f / std::sqrt(n.x * n.x + n.z * n.z);	
		//	C = Vector3f(n.z * invLen, 0.0f, -n.x * invLen);
		//	// n dot C:
		//	// n.x * n.z/sqrt(n.x * n.x + n.z * n.z) + 
		//	// 0 * n.y  +
		//	// n.z * -n.x /sqrt(n.x * n.x + n.z * n.z) == 0
		//	// C is on xz plane
		//}
		//else {
		//	float invLen = 1.0f / std::sqrt(n.y * n.y + n.z * n.z);
		//	C = Vector3f(0.0f, n.z * invLen, -n.y * invLen);
		//}
		//B = crossProduct(C, n);
		//return dir.x * B + dir.y * C + dir.z * n;
		
	}

	// wo: -camera dir   wi: sampled dir
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
		case SPECULAR_REFLECTIVE: {
			return 1;
			break;
		}

		case PERFECT_REFRACTIVE: {
			float F = fresnel(-wi, N, eta_i, this->eta);
			if (wi.dot(N) < 0)
				return 1 - F;
			
			return F;
			break;
		}

		default:
			return 1;
			break;
		}
	}
};