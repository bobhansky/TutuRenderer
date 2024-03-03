#pragma once

#include<fstream>
#include<iostream>
#include<stdexcept>
#include<string>
#include<vector>
#include<cmath>
#include<stack>
#include<thread>
#include<mutex>

#include "Vector.hpp"
#include "global.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Material.hpp"
#include "PPMGenerator.hpp"
#include "BVH.hpp"
#include "BoundBox.hpp"
#include "BVHStrategy.hpp"
#include "BaseInterStrategy.hpp"


bool PRINT = false;			// debug helper
int SPP = 1;
float SPP_inv = 1.f / SPP;
float Russian_Roulette = 0.78f;

#define EXPEDITE true			// BVH to expedite intersection
#define MULTITHREAD				// multi threads to expedite, comment it out for better ebug
// 12/23/2023  when it's 3 sometimes error happen in debug mode
// exited with code -1073741819. (first time compiling will most likely result in this)
#define N_THREAD 20 
#define GAMMA_COORECTION 


// thread argument
// 12/23/2023: unknown reason, if I put start and end inside this arg struct
// instead of passing them as thread function parameters like Im doing now
// some of upper img is not rendered 
struct Thread_arg {
	// ul, delta_v  delta_h  g->rgb  eyePos
	const Vector3f* ul;
	const Vector3f* delta_v;
	const Vector3f* delta_h;
	const Vector3f* c_off_h;
	const Vector3f* c_off_v;
	std::vector<Vector3i>* rgb_array;
	const Vector3f* eyePos;
	int Ncol;
	Renderer* r;
	
};

void sub_render(Thread_arg*, int, int, int);


/// <summary>
/// this is the class perform computer graphics algorithms
/// It takes a ppmgenerator and then do algorithm based on ppmGenerator's data
/// </summary>
class Renderer {
public:
	// constructor, takes a PPMGenerator for future commands
	Renderer(PPMGenerator* ppmg) {
		g = ppmg;

		if (EXPEDITE) interStrategy = new BVHStrategy();
		else interStrategy = new BaseInterStrategy();
		
		g->scene.initializeBVH();
	}

	~Renderer() {
		delete interStrategy;
	}



	// takes a PPMGenerator and render its rgb array
	void render() {
		// we shoot a ray from eyePos, trough each pixel, 
		// to the scene. update the rgb info if we hit objects
		// v: vertical unit vector of the view plane
		// u: horizontal unit vector of the view plane

		Vector3f u = crossProduct(g->viewdir, g->updir);
		u = normalized(u);
		Vector3f v = crossProduct(u, g->viewdir);
		v = normalized(v);

		// calculate the near plane parameters
		// we set the distance from eye to near plane to 1, namely the nearplane.z = eyePos.z - 1;
		float d = 1.f;
		// distance from eye to nearplane MATTERS when we are doing orthographic ray
		// cuz we need a bigger viewplane   I set it to 4 in respond to my scene set
		if (g->parallel_projection) d = 4.f;

		// tan(hfov/2) =  nearplane.width/2 : d
		// h/2 = tan(hfov/2) * d 
		// here height_half and width_half are the h and w of the nearplane in the world space
		float width_half = fabs(tan(degree2Radians(g->hfov / 2.f)) * d);
		float aspect_ratio = g->width / (float)g->height;
		float height_half = width_half / aspect_ratio;


		// we sample the center of the pixel 
		// so we need to add offset to the center later
		Vector3f n = normalized(g->viewdir);
		Vector3f eyePos = g->eyePos;
		Vector3f ul = eyePos + d * n - width_half * u + height_half * v;
		Vector3f ur = eyePos + d * n + width_half * u + height_half * v;
		Vector3f ll = eyePos + d * n - width_half * u - height_half * v;
		Vector3f lr = eyePos + d * n + width_half * u - height_half * v;


		Vector3f delta_h = Vector3f(0, 0, 0);	// delta horizontal
		if (g->width != 1) delta_h = (ur - ul) / (g->width - 1);
		Vector3f delta_v = Vector3f(0, 0, 0);	// delta vertical
		if (g->height != 1) delta_v = (ll - ul) / (g->height - 1);
		Vector3f c_off_h = (ur - ul) / (float)(g->width * 2);	// center horizontal offset
		Vector3f c_off_v = (ll - ul) / (float)(g->height * 2); // vertical

		// trace ray: cast a ray from eyePos through each pixel
		// try to get the tNear and its intersection properties

		#ifdef MULTITHREAD
		int rowPerthd = g->height / N_THREAD;
		std::thread thds[N_THREAD];
	
		for (int i = 0; i < N_THREAD - 1; i++) {
			Thread_arg arg{
				&ul,
				&delta_v,
				&delta_h,
				&c_off_h,
				&c_off_v,
				&g->rgb,
				&eyePos,
				g->width,
				this
			};
			thds[i] = std::thread(sub_render, &arg, i, (i * rowPerthd), ((i + 1) * rowPerthd));
		}
		// last thread
		Thread_arg arg_end{
				&ul,
				&delta_v,
				&delta_h,
				&c_off_h,
				&c_off_v,
				&g->rgb,
				&eyePos,
				g->width,
				this
		};
		thds[N_THREAD-1] = std::thread(sub_render, &arg_end, N_THREAD-1, (N_THREAD - 1) * rowPerthd, g->height);

		for (int i = N_THREAD-1; i >= 0; i--) {
			thds[i].join();
		}

		#endif


		// single thread
		#ifndef MULTITHREAD
		for (int y = 0; y < g->height; y++) {
			Vector3f v_off = y * delta_v;
			//PRINT = false;
			for (int x = 0; x < g->width; x++) {
				if (x == 1599 && y == 115)
					PRINT = true;

				Vector3i& color = g->rgb.at(g->getIndex(x, y));		// update this color to change the rgb array
				Vector3f h_off = x * delta_h;
				Vector3f pixelPos = ul + h_off + v_off + c_off_h + c_off_v;		// pixel center position in world space
				Vector3f rayDir;
				Vector3f eyeLocation;
				// calculate the rayDir and eyeLocation base on different projection method
				if (!g->parallel_projection) {	// perspective
					rayDir = normalized(pixelPos - eyePos);
					eyeLocation = eyePos;
				}
				else {	// orthographic 
					// set ray direction orthogonal to the view plane
					rayDir = n;
					eyeLocation = pixelPos - d * n;		// - d * n  only to set a distance between the eye location and near plane(enlarged)
				}

				// trace ray into each pixel
				Vector3f res;
				for (int i = 0; i < SPP; i++) {
					res = res + traceRay(eyeLocation, rayDir, 0);
				}
				res = res * SPP_inv;

#ifdef GAMMA_COORECTION
				// gamma correction
				color.x = 255 * pow(clamp(0, 1, res.x), 0.6f);
				color.y = 255 * pow(clamp(0, 1, res.y), 0.6f);
				color.z = 255 * pow(clamp(0, 1, res.z), 0.6f);
#endif

#ifndef GAMMA_COORECTION
				color.x = 255 * clamp(0, 1, res.x);
				color.y = 255 * clamp(0, 1, res.y);
				color.z = 255 * clamp(0, 1, res.z);
#endif // !GAMMA_COORECTION
			}

			showProgress((float)y / g->height);
		}
		showProgress(1.f);
		std::cout << std::endl;
		#endif // !

	}

public:
	PPMGenerator* g;
	IIntersectStrategy* interStrategy;
	std::mutex sampleLight_mutex;
	

	// -dir is the wo, trace a ray into the scene and update intersection
	// use Russian Roulette to terminate
	Vector3f traceRay(const Vector3f& origin, const Vector3f& dir, int depth) {
		Intersection inter;
		// loop through all the objects in the scene and find the nearest intersection
		// const Class &: const lvalue reference
		// test intersection
		interStrategy->UpdateInter(inter, this->g->scene, origin, dir);

		// if ray has no intersection, return bkgcolor
		if (!inter.intersected)		return g->bkgcolor;
		// if ray hit emissive object, return the L_o
		if (inter.mtlcolor.hasEmission()) {
			if(depth == 0)
				return inter.mtlcolor.diffuse; //inter.mtlcolor.emission;
			return inter.mtlcolor.emission;
		}
		
		// **************** TEXUTRE ********************
		// if diffuse texture is activated, change mtlcolor.diffuse to texture data
		if (! FLOAT_EQUAL(-1.f, inter.textureIndex) && !FLOAT_EQUAL(-1.f, inter.textPos.x) 
			&& !FLOAT_EQUAL(-1.f, inter.textPos.y)) {
			if (g->textures.size() <= inter.textureIndex) {
				std::cout << 
					"\ninter.textureIndex is greater than texuture.size()\nImport texture files in config.txt \n";
				exit(0);
				}
				inter.mtlcolor.diffuse = g->textures.at(inter.textureIndex)
					.getRGBat(inter.textPos.x, inter.textPos.y);
		}
		// if do shading with normal map
		if (inter.normalMapIndex != -1) {
			changeNormalDir(inter);
		}
		// **************** TEXUTRE ENDS ****************


		// if UNLIT material, return diffuse
		if (inter.mtlcolor.mType == UNLIT) return inter.mtlcolor.diffuse;

		Vector3f dir_illu(0.f);
		Vector3f indir_illu(0.f);

		if (inter.mtlcolor.mType == SPECULAR_REFLECTIVE)
			return calcForMirror(origin, dir, inter, depth+1);
		
		// ****** Direct illumination
		float light_pdf;
		Intersection light_inter;
		sampleLight(light_inter, light_pdf);
		if (!light_inter.intersected) {

		}
		else {
			// test if the ray is blocked in the middle 
			if (isShadowRayBlocked(inter, light_inter.pos)) {}
			else {	// ray is not blocked, then calculate the direct illumination
				Vector3f L_i = light_inter.mtlcolor.emission;
				Vector3f light_N = normalized(light_inter.nDir);
				Vector3f p_to_light = normalized(light_inter.pos - inter.pos);
				float cos_theta_prime = light_N.dot(-p_to_light);
				// if light does not illuminate this direction (p_to_light is on the back side of the light)
				if (cos_theta_prime < 0) {
				}
				else {
					float dis2 = (light_inter.pos - inter.pos).dot(light_inter.pos - inter.pos);
					float cos_theta = p_to_light.dot(inter.nDir);
					Vector3f wo = -dir;
					Vector3f f_r = inter.mtlcolor.BxDF(p_to_light, wo, inter.nDir, g->eta);

					dir_illu = L_i * f_r * cos_theta * cos_theta_prime / dis2 / light_pdf;
				}
			}
			// clamp in case that dir_illu is a verysmall negative number
			dir_illu.x = clamp(0, 1, dir_illu.x);
			dir_illu.y = clamp(0, 1, dir_illu.y);
			dir_illu.z = clamp(0, 1, dir_illu.z);
		}

		// ****** Indirect Illumination
		
		// test RussianRoulette first
		if (getRandomFloat() > Russian_Roulette)
			return dir_illu;

		// inter point p to another point x
		Vector3f p_to_x_dir = inter.mtlcolor.sampleDirection(normalized(-dir), inter.nDir);

		p_to_x_dir = normalized(p_to_x_dir);
		
		Intersection x_inter;
		Vector3f rayOrig = inter.pos + inter.nDir * EPSILON;
		interStrategy->UpdateInter(x_inter, g->scene,rayOrig , p_to_x_dir);
		// calculate only when inter is on a non-emissive object
		if (x_inter.intersected && !x_inter.mtlcolor.hasEmission()) {
			float cos_pnormal_xdir = std::max(0.f, inter.nDir.dot(p_to_x_dir));

			Vector3f wo = -dir;
			float pdf = inter.mtlcolor.pdf(p_to_x_dir, wo, inter.nDir);
			// BRDF has reciprocity
			Vector3f f_r = inter.mtlcolor.BxDF(p_to_x_dir, wo, inter.nDir, g->eta);

			// in case that pdf is too small and generate white img
			if (pdf == 0.f) return 0;
			if (pdf > 1) pdf = 1;

			Vector3f res = traceRay(rayOrig, p_to_x_dir, depth + 1);
			indir_illu = res * f_r * cos_pnormal_xdir / pdf / Russian_Roulette;
		}
		Vector3f ret = dir_illu + indir_illu;
		return clamp(Vector3f(0), Vector3f(1), ret);
		
		/*
		// ****** calculate reflection and transmittance contribution
		Vector3f refRayOrig = inter.pos;		// reflection ray origin
		Vector3f traRayOrig = inter.pos;		// tranmittance ray origin
		Vector3f N = normalized(inter.nDir);
		float fr = 0;
		float eta_i = 0, eta_t = 0;

		
		// 3/18/2023 23:59:  deal with reflection and refrection individually 
		// when reflection, we are not entering or leaving the object 
		// check if we are entering a object or escaping from it

		// get ior depending on the location of incident ray
		// N.dot(-dir)
		float cosN_Dir = N.dot(-dir);

		if (cosN_Dir > 0) {	// if incident ray is outside
			eta_i = g->eta;
			eta_t = inter.mtlcolor.eta;

			fr = fresnel(dir, N, eta_i, eta_t);
		}
		else { // incident ray is inside
			eta_i = inter.mtlcolor.eta;
			eta_t = g->eta;	// get previous medium's ior

			fr = fresnel(dir, N, eta_i, eta_t);
		}

		Vector3f refractDir = normalized(getRefractionDir(dir, N, eta_i, eta_t));
		Vector3f reflectDir = normalized(getReflectionDir(dir, inter.nDir));
		float cos_refle_N = reflectDir.dot(N);
		float cos_refra_N = refractDir.dot(N);
		// surface normal always points outward direction
		// if angle between surface normal and reflection dir > 90 degree
		// then we are inside of the object
		// other wise we are outside 
		if (cos_refle_N < 0) {	// have to be picky about this offset: try to edit EPSILON
			refRayOrig = refRayOrig - EPSILON * N;
		}
		else {
			refRayOrig = refRayOrig + EPSILON * N;
		}
		if (cos_refra_N < 0) {	// refraction ray is on the opposite side of N
			// then we enter the obj
			// do rayOrig shifting
			traRayOrig = traRayOrig - EPSILON * N;
		}
		else {	// refraction ray is on the same side of N
			// then we leave the obj
			traRayOrig = traRayOrig + EPSILON * N;
		}

		// total internal reflection
		if (FLOAT_EQUAL(0, refractDir.norm()))	fr = 1.f;
		
		Vector3f R_lambda;
		Vector3f T_lambda;

		
		if (!FLOAT_EQUAL(1.f, inter.mtlcolor.alpha) && !FLOAT_EQUAL(fr, 1.f))	// if object is transparent && exclude TIR
			T_lambda = traceRay(traRayOrig, refractDir, depth + 1);
		if (inter.mtlcolor.ks != 0)		// if object is reflective 
			R_lambda = traceRay(refRayOrig, reflectDir, depth + 1);
		
	
		return blinnPhongRes + fr * R_lambda + (1 - fr) * (1 - inter.mtlcolor.alpha) * T_lambda;

		*/
	}


	

	/// <summary>
	/// shadow ray
	/// </summary>
	/// <param name="p">inter information</param>
	/// <param name="lightPos">light position</param>
	/// <returns>if the ray is blocked by non-emissive obj, return true</returns>
	bool isShadowRayBlocked(Intersection& p, Vector3f& lightPos) {
		Vector3f orig = p.pos;
		orig = orig + EPSILON * p.nDir;
		Vector3f raydir = normalized(lightPos - orig);
		float distance = (lightPos - orig).norm();

		if (!EXPEDITE) {
			Intersection p_light_inter;
			for (auto& i : g->scene.objList) {
				if (i->mtlcolor.hasEmission()) continue; // do not test with light avatar
				
				if (i->intersect(orig, raydir, p_light_inter) && p_light_inter.t < distance ) {
					return true;
				}
			}
			return false;
		}
		else { // BVH intersection test
			return hasIntersection(g->scene.BVHaccelerator->getNode(), orig, raydir, distance);
		}
	}



	// TBN transformation matrix to change the dir of normal
	void changeNormalDir(Intersection& inter) {
		Texture& nMap = g->normalMaps.at(inter.normalMapIndex);
		Vector3f color = nMap.getRGBat(inter.textPos.x, inter.textPos.y);

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
			float deltaV1 = t->uv1.y - t->uv1.y;

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

	// sample all the emissive object to get one point on their surface,
	// update the intersection, and the pdf to sample it
	// pdf of that inter is, 1/area of THE object surface area
	void sampleLight(Intersection& inter, float& pdf) {
		static std::vector<Object*> lightList;
		static float totalArea = 0;
		static bool firstimeCall = true;
		int size = lightList.size();
		
		// if first time call it, put all the emissive object into lightList

		if (firstimeCall) {
			// 3/2/2024: need lock
			sampleLight_mutex.lock();
			for (auto& i : g->scene.objList) {
				if (!firstimeCall)
					break;

				if (i->mtlcolor.hasEmission()) {
					lightList.emplace_back(i.get());
					totalArea += i->getArea();	// without lock, problem on i->getArea(), maybe due to unique_ptr
				}
			}
			firstimeCall = false;
			sampleLight_mutex.unlock();
		}

		size = lightList.size();
		// if there's no light
		if (size == 0) {
			inter.intersected = false;
			return;
		}
		
		int index = (int)(getRandomFloat() * (size - 1) + 0.4999f);
		if (size == 1) index = 0;
		
		Object* lightObject = lightList.at(index);

		lightObject->samplePoint(inter, pdf);
		// independent event p(a&&b) == p(a) *  p(b)
		pdf = pdf * lightObject->getArea() / totalArea;
	}

	/// <summary>
	/// special case for perfect specular reflection
	/// calculate color if hit on lights
	/// </summary>
	/// <param name="origin"> ray origin </param>
	/// <param name="dir"> ray direction </param>
	/// <param name="inter"> ray object intersection</param>
	/// <param name="depth"> ray bouncing depth</param>
	/// <returns></returns>
	Vector3f calcForMirror(const Vector3f& origin, const Vector3f& dir,  Intersection& inter, int depth) {
		if (depth > 9) return 0;	// 2 mirror reflect forever causing stack overflow
		Vector3f p_to_x_dir = inter.mtlcolor.sampleDirection(normalized(-dir), inter.nDir);

		p_to_x_dir = normalized(p_to_x_dir);

		Intersection x_inter;
		Vector3f rayOrig = inter.pos + inter.nDir * EPSILON;
		interStrategy->UpdateInter(x_inter, g->scene, rayOrig, p_to_x_dir);
		if (x_inter.intersected ) {
			float cos_pnormal_xdir = inter.nDir.dot(p_to_x_dir);

			Vector3f wo = -dir;
			float pdf = inter.mtlcolor.pdf(wo, p_to_x_dir, inter.nDir);
			// BRDF has reciprocity
			Vector3f f_r = inter.mtlcolor.BxDF(p_to_x_dir, wo, inter.nDir, g->eta);

			// in case that pdf is too small and generate white img
			if (pdf == 0.f) return { 0,0,0 };
			if (pdf > 1.f) pdf = 1.f;

			Vector3f res = traceRay(rayOrig, p_to_x_dir, depth+1);
			return res * f_r * cos_pnormal_xdir / pdf ;
		}

		return 0;
	}
};



/// <summary>
/// used in multithreads rendering. each thread call this funciton
/// </summary>
/// <param name="arg"></param>
/// <param name="threadID"></param>
/// <param name="s">start row</param>
/// <param name="e">end row exclusive</param>
void sub_render(Thread_arg* arg, int threadID, int s, int e) {

	const Vector3f ul = *arg->ul;
	const Vector3f delta_v = *arg->delta_v;
	const Vector3f delta_h = *arg->delta_h;
	const Vector3f c_off_h = *arg->c_off_h;
	const Vector3f c_off_v = *arg->c_off_v;
	std::vector<Vector3i>* rgb_array = arg->rgb_array;
	const Vector3f eyePos = *arg->eyePos;
	int Ncol = arg->Ncol;
	Renderer *r = arg->r;
	PPMGenerator* g = r->g;

	for (int y = s; y < e; y++) {
		for (int x = 0; x < Ncol; x++) {
			Vector3i& color = rgb_array->at(g->getIndex(x, y));
			Vector3f rayDir = { 0,0,0 };
			Vector3f pixelPos = ul + x * delta_h + y * delta_v + c_off_v + c_off_v;
			rayDir = normalized((pixelPos - eyePos));

			// trace ray into each pixel
			Vector3f res;
			for (int i = 0; i < SPP; i++) {
				res = res + r->traceRay(eyePos, rayDir, 0);
			}
			res = res * SPP_inv;
#ifdef GAMMA_COORECTION
			// gamma correction
			color.x = 255 * pow(clamp(0, 1, res.x), 0.6f);
			color.y = 255 * pow(clamp(0, 1, res.y), 0.6f);
			color.z = 255 * pow(clamp(0, 1, res.z), 0.6f);
#endif

#ifndef GAMMA_COORECTION
			color.x = 255 * clamp(0, 1, res.x);
			color.y = 255 * clamp(0, 1, res.y);
			color.z = 255 * clamp(0, 1, res.z);
#endif // !GAMMA_COORECTION


		}
		//showProgress((float)y / e);		// comment it out for a clean terminal
	}
	
}





