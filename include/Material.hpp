#pragma once

#include "global.hpp"
#include "Vector.hpp"



enum MaterialType {
	LAMBERTIAN,
	SPECULAR_REFLECTIVE
};


class Material {
public:
	Vector3f diffuse;
	Vector3f specular;
	Vector3f emission = Vector3f(0.f);
	MaterialType mType = LAMBERTIAN;
	float ka = 0;;
	float kd = 0;
	float ks = 0;			// if ks = 0, then the material is non-reflective
	float n = 0;			// highlights shininess power coefficient

	float alpha = 1;			// opacity		if alpha = 1. then no refraction 
	float eta = 1;				// index of refraction


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
		}
		return *this;
	}

	bool hasEmission() {
		return emission.x || emission.y || emission.z ;
	}

	// BxDF, return vec3f of elements within [0,1] 
	Vector3f BxDF(Vector3f& wi, Vector3f& wo, Vector3f& N) {
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
		}
	}



	// sample a direction on the hemisphere
	Vector3f sampleDirection(Vector3f& pos, Vector3f& N) {
		switch (mType)
		{
		case LAMBERTIAN: {
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
			float cosTheta = r1;
			float phi = 2 * M_PI * r2;

			Vector3f dir;
			float sinTheta = std::max(0.f, sqrtf(1- pow(r1,2)));
			dir.x = cos(phi) * sinTheta;
			dir.y = sin(phi) * sinTheta;
			dir.z = cosTheta;

			dir = normalized(dir);
			
			return SphereLocal2world(N, dir);

			
			break;
		}
		case SPECULAR_REFLECTIVE: {
			return 0;
			break;

		}
		default: {
			return Vector3f(0.f);
			break;
		}
		}

	}

	
	Vector3f SphereLocal2world(Vector3f& N, Vector3f& dir) {
		// https://raytracing.github.io/books/RayTracingTheRestOfYourLife.html#generatingrandomdirections/uniformsamplingahemisphere
		// 8. orthonormal basis
		// change of basis 

		// x y z local coordinates to s t n coordinates

		Vector3f a;
		N = normalized(N);	//z
		
		// construct an Orthonalmal basis
		// randomly choose an a that is not parallel to N
		if (fabs(N.x) > 0.9f)
			a = { 0.f, 1.f, 0.f };
		else a = { 1.f, 0.f, 0.f };

		Vector3f T = crossProduct(a, N); // y   X cross Y == Z      then S cross T should == N
		Vector3f S = crossProduct(T, N); // x
		
		return normalized(dir.x * S + dir.y * T + dir.z * N);
	}


	float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
		switch (mType)
		{
		case LAMBERTIAN: {
			// uniform sample probability 1 / (2 * PI)
			if (wo.dot(N) > 0.0f)
				return 0.5f / M_PI;
			else
			{
				return 0.0f;
			}
				
			break;
		}
		case SPECULAR_REFLECTIVE:
			break;
		default:
			break;
		}
	}


};