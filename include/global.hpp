#pragma once
#include <string>
#include <float.h>
#include <stdexcept>
#include <math.h>
#include <algorithm>
#include <random>
#include <iostream>
#include <stdio.h>

#include "Vector.hpp"


#define M_PI 3.1415926535897f
#define EPSILON 0.005f		// be picky about it, change it to accommodate object size

// lerp(x,v0,v1) = v0 + x(v1-v0);
// x is the portion
// v0 v1 are the values
float lerp(float v0, float v1, float x) {
	return v0 + x * (v1 - v0);
}

Vector3f lerp(Vector3f& v0, Vector3f& v1, float x) {
	Vector3f res;
	res.x = v0.x + x * (v1.x - v0.x);
	res.y = v0.y + x * (v1.y - v0.y);
	res.z = v0.z + x * (v1.z - v0.z);

	return res;
}

inline float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}

inline Vector3f clamp(const Vector3f& lo, const Vector3f& hi, Vector3f& v)
{
	Vector3f res;
	res.x = clamp(lo.x, hi.x, v.x);
	res.y = clamp(lo.y, hi.y, v.y);
	res.z = clamp(lo.z, hi.z, v.z);
	return res;
}

inline float rescale(float input, float originMax, float originMin, float targetMax, float targetMin) {
	return targetMin + ((targetMax - targetMin) * (input - originMin)/(originMax - originMin));
}


// check if str is convertable to a positive integer
// if there's a char other than 0 to 9
// then throw exeption
void checkPosInt(std::string& str) {
	for (auto i : str) {
		if (i < 48 || i > 57) {
			throw std::runtime_error(str + ": expect a positive number");
		}
	}
}

// check if str is convertable to a float
void checkFloat(std::string& str) {
	if (str.size() == 0) {
		throw std::runtime_error(str + ": not a valid float number");
	}

	bool dotAppeared = false;
	// for minus 
	if (str.at(0) == '-') {
		if(str.size()==1) throw std::runtime_error(str + ": not a valid float number");

		for (int i = 1; i < str.size(); i++) {
			// -abcdef   a can't be a char other than a number
			if (i == 1 && str[1] < 48 || str[i] > 57) throw std::runtime_error(str + ": not a valid float number");
			// for other index 
			else {
				// dot can only present once && can't be at the end of the str
				if (str[i] == '.' && !dotAppeared && i!=str.size()-1) {
					dotAppeared = true;		// 
				}
				else {	// expect numbers
					if(str[i] < 48 || str[i] > 57) throw std::runtime_error(str + ": not a valid float number");
				}
			}
		}
	}
	// for positive
	else {
		for (int i = 0; i < str.size(); i++) {
			// abcdef   a can't be a char other than a number
			if (i == 0 && str[0] < 48 || str[i] > 57) throw std::runtime_error(str + ": not a valid float number");
			// for other index 
			else {
				// dot can only present once && can't be at the end of the str
				if (str[i] == '.' && !dotAppeared && i != str.size() - 1) {
					dotAppeared = true;		// 
				}
				else {	// expect numbers
					if (str[i] < 48 || str[i] > 57) throw std::runtime_error(str + ": not a valid float number");
				}
			}
		}
	}

}

// convert degree to radians
float degree2Radians(const float& d) {
	return d * M_PI / 180.f;
}

// check if two float numbers are equal
inline bool FLOAT_EQUAL(const float& x, const float& y) {
	return (fabs(x - y) < 0.00001f);
}

/// <summary		// ctrl + / 
/// solve
/// A*t^2 + B*t + C = 0
/// </summary>
/// <param name="t1"> first solution  </param>
/// <param name="t2"> second solution	</param>
/// 
/// if there's only one solution, then either t1 or t2 == FLT_MAX
/// if there's no real solution, t1 == t2 == FLT_MAX
void solveQuadratic(float& t1, float& t2, float& A, float& B, float& C) {
	float discriminant = B * B - 4 * A * C;
	// no real solution
	if (discriminant < 0) {
		t1 = FLT_MAX;
		t2 = FLT_MAX;
	}
	// one real solution
	else if (discriminant == 0) {
		t1 = (-B + sqrtf(discriminant)) / (2 * A);
		t2 = t1;
	}
	// two real solution
	else {
		t1 = (-B + sqrtf(discriminant)) / (2 * A);
		t2 = (-B - sqrtf(discriminant)) / (2 * A);
	}
	
	// always make t1 the smaller result
	if (t1 > t2) std::swap(t1, t2);
}



// check if a ele existing within a string vector
bool existIn(std::string& ele, std::vector<std::string>& v) {
	for (auto i : v) {
		if (ele.compare(i) == 0) return true;
	}
	return false;
}



// get a uniformly distributed number in range [0,1)
float getRandomFloat() {
	// see 
	// https://stackoverflow.com/questions/38367976/do-stdrandom-device-and-stdmt19937-follow-an-uniform-distribution

	// an uniformly - distributed random number generator, use it to seed a pseudo-random generator
	static std::random_device dev;
	// a fast pseudo-random number generator, use this to seed a particular distribution
	static std::mt19937 rng(dev());		
	static std::uniform_real_distribution<float> dist(0,1); // distribution in range [0.0, 1.0)

	return dist(rng);	
}

// cout to terminal the progress
void showProgress(float prog) {
	int barWidth = 60;

	int pos = barWidth * prog;
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos) std::cout << "=";
		else if (i == pos) std::cout << ">";
		else std::cout << " ";
	}

	std::cout << int(prog * 100.0) << " %\r";
}


// safely get vec3f element in an vec3f array
Vector3f getEleIn(std::vector<Vector3f>& arr, int index) {
	if (index >= arr.size() || index < 0) {
		throw std::runtime_error(index + " is out of bound: array has size: " + arr.size());
	}
	return arr.at(index);
}

// for uv, texture mapping
Vector2f getEleIn(std::vector<Vector2f>& arr, int index) {
	if (index >= arr.size() || index < 0) {
		throw std::runtime_error(index + " is out of bound: array has size: " + arr.size());
	}
	return arr.at(index);
}

// Schlick approximation used for microfacet model
// https://learnopengl.com/PBR/Theory:
// For conductor surfaces (metals), calculating the base reflectivity with indices of refraction doesn't properly hold 
// and we need to use a different Fresnel equation for conductors altogether. 
Vector3f fresnelSchlick(float cosTheta, const Vector3f& F0)
{
	return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

// fresnel, get the specular reflection fraction 
float fresnel(const Vector3f& Incident, const Vector3f& normal, const float eta_i, const float eta_t) {
	Vector3f I = normalized(-Incident);
	Vector3f N = normalized(normal);
	// there are two possible cases:
	// 1. ray is bouncing at outer surface
	// 2. ray os bouncing at innner surface
	// if ray is bouncing at inner surface (I dot N < 0)
	// reverse N direction
	float cosI_N = I.dot(N);
	if (cosI_N < 0) N = -N;
	
	// The Schlick approximation defines the Fresnel
	// reflectance coefficient using the function :
	// Fr = F0 + (1–F0 )(1–cos(theta_i))^5

	// Schlick approximation: a faster approach to define F0
	float F0 = powf(((eta_t - eta_i) / (eta_t + eta_i)), 2.f);	
	float Fr = F0 + (1 - F0) * (powf(1 - (I.dot(N)), 5.f));

	return Fr;
}

// get the reflection direction (un-normalized) by given incident ray and inter's normal direction 
Vector3f getReflectionDir(const Vector3f& incident, const Vector3f& normal) {
	Vector3f I = -normalized(incident);
	Vector3f N = normalized(normal);
	
	return 2 * (N.dot(I)) * N - I;
}

// get the transmittance ray direction  (un-normalized)
// incident: ray dir from source to this intersection
// normal: the normal direction of the point
// eta: indices of refraction
Vector3f getRefractionDir(const Vector3f& incident, const Vector3f& normal, float eta_i, float eta_t) {
	// see 03-15.raytracing.pdf page 68
	// additional notes:
	// there are two possible cases:
	// 1. ray is traveling from outside to inside of the obj
	// 2. ray is traveling from inside of the obj to outside
	// since we define object's normal pointing toward the outside,
	// we can check the sign of I dot N to tell which case it is
	Vector3f I = normalized(-incident);
	Vector3f N = normalized(normal);
	float cos_theta_i = N.dot(I);
	cos_theta_i = clamp(-1, 1, cos_theta_i);

	if (cos_theta_i < 0) {
		N = -N;
		cos_theta_i = -cos_theta_i;
	}

	float sin_theta_i = sqrtf(1 - powf(cos_theta_i, 2));
	float sin_theta_t = (eta_i / eta_t) * sin_theta_i;

	// check total internal reflection case:
	// if it is the case, return 0 0 0, meanning no refraction dir
	if (sin_theta_i > (eta_t / eta_i)) 
		return Vector3f(0);

	float cos_theta_t = sqrtf(1 - powf(sin_theta_t, 2));

	return cos_theta_t * (-N) + eta_i / eta_t * (cos_theta_i * N - I);
}

/// <summary>
/// Normal Distribution Function
/// isotropic GGX
/// </summary>
/// <param name="h">: half vector, also the micro normal m </param>
/// <param name="n">: macrosurface normal  </param>
/// <param name="roughness">: width parameter alpha_g </param>
/// <returns>return the area of microfacet with micro normal h in dA</returns>
float D_ndf(const Vector3f& h, const Vector3f& n, float roughness) {
	float alpha = roughness * roughness;
	float cos_nh_2 = (n.dot(h)) * (n.dot(h));
	cos_nh_2 = std::max(cos_nh_2, 0.f);
	float sin_nh_2 = 1 - cos_nh_2;
	float sum = alpha * alpha * cos_nh_2 + sin_nh_2;
	float res = (alpha * alpha) / (M_PI * (sum * sum));

	return res;
}

/// <summary>
/// shadow masking function
/// </summary>
/// <param name="wi">: incident solid angle</param>
/// <param name="wo">: observing/out solid angle</param>
/// <param name="n">: normal</param>
/// <param name="roughness">: width parameter alpha_g </param>
/// <returns> the fraction of unblocked part, [0,1]</returns>
float G_smf(Vector3f& wi, Vector3f& wo, Vector3f& n, float roughness) {
	float alpha = roughness * roughness;
	float angle_wi_n = acosf(wi.dot(n));
	float angle_wo_n = acosf(wo.dot(n));
	// in paper   Microfacet Models for Refraction through Rough Surface
	float G1_wi = 2 / (1 + sqrtf(1 + alpha * alpha * powf(tanf(angle_wi_n), 2)));
	float G1_wo = 2 / (1 + sqrtf(1 + alpha * alpha * powf(tanf(angle_wo_n), 2)));

	return G1_wi * G1_wo;
	// return clamp(0.f, 1.f, G1_wi * G1_wo);

	
	// a better G  according to https://zhuanlan.zhihu.com/p/434964126
	//float Ai = (-1 + sqrtf(1 + roughness * roughness * powf(tanf(angle_wi_n), 2))) * 0.5;
	//float Ao = (-1 + sqrtf(1 + roughness * roughness * powf(tanf(angle_wo_n), 2))) * 0.5;
	//return 1 / (1 + Ai + Ao);
}

// from https://learnopengl.com/PBR/Theory
float GeometrySchlickGGX(float NdotV, float k)
{
    float nom   = NdotV;
    float denom = NdotV * (1.0 - k) + k;
	
    return nom / denom;
}

// V 
float GeometrySmith(Vector3f& N, Vector3f& wi, Vector3f& wo, float k)
{
	float NdotV = std::max(N.dot(wo), 0.0f);
	float NdotL = std::max(N.dot(wi), 0.0f);
	float ggx1 = GeometrySchlickGGX(NdotV, k);
	float ggx2 = GeometrySchlickGGX(NdotL, k);

	return ggx1 * ggx2;
}