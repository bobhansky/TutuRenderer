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
#include "Texture.hpp"


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
	Texture* output;
	const Vector3f* eyePos;
	int Ncol;
	Renderer* r;
};

std::vector<std::string> records;	// ray information, thread independent string

void sub_render(Thread_arg*, int, int, int);


float getMisWeight(float pdf, float otherPdf);


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

		records = std::vector<std::string>(N_THREAD, std::string());

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
				&g->output,
				&eyePos,
				g->width,
				this
			};
#if RECORD
			thds[i] = std::thread(sub_render_record, &arg, i, (i * rowPerthd), ((i + 1) * rowPerthd));
#else
			thds[i] = std::thread(sub_render, &arg, i, (i * rowPerthd), ((i + 1) * rowPerthd));
#endif
		}
		// last thread
		Thread_arg arg_end{
				&ul,
				&delta_v,
				&delta_h,
				&c_off_h,
				&c_off_v,
				&g->output,
				&eyePos,
				g->width,
				this
		};
#if RECORD
		thds[N_THREAD - 1] = std::thread(sub_render_record, &arg_end, N_THREAD - 1, (N_THREAD - 1) * rowPerthd, g->height);
#else
		thds[N_THREAD - 1] = std::thread(sub_render, &arg_end, N_THREAD - 1, (N_THREAD - 1) * rowPerthd, g->height);
#endif

		for (int i = N_THREAD - 1; i >= 0; i--) {
			thds[i].join();
		}


		// single thread
#else
		for (int y = 0; y < g->height; y++) {
			Vector3f v_off = y * delta_v;
			//PRINT = false;
			for (int x = 0; x < g->width; x++) {
				if (x == 1026 && y == 618) {
					PRINT = true;
				}

				Vector3f& color = g->output.rgb.at(g->getIndex(x, y));		// update this color to change the rgb array
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
				Vector3f estimate;
				for (int i = 0; i < SPP; i++) {
					estimate = estimate + traceRay(eyeLocation, rayDir, 0);
				}
				estimate = estimate * SPP_inv;

				PRINT = false;
				color = estimate;
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
	Vector3f traceRay(const Vector3f& origin, const Vector3f& dir, int depth, Vector3f tp, 
		Intersection* nxtInter = nullptr, int thdID = -1, bool recording = false) {

#if RECORD
		if (recording) {
			records[thdID].append("\n***Depth=" + std::to_string(depth) +": Orig" + origin.toString() 
				+ ", "+ "Dir" + dir.toString());
		}
#endif
		if (depth > MAX_DEPTH) return 0;

		Vector3f sampleValue = 0;
		Intersection inter;
		// const Class &: const lvalue reference
		// get intersection
		if (nxtInter)	inter = *nxtInter;
		else interStrategy->UpdateInter(inter, this->g->scene, origin, dir);

		// if UNLIT material, return diffuse
		if (inter.mtlcolor.mType == UNLIT) return inter.mtlcolor.diffuse;

		// if ray has no intersection, return bkgcolor
		if (!inter.intersected)		return g->bkgcolor;
		// if ray hit emissive object, return the L_o
		// depth > 0: indirect light, excluded
		if (inter.mtlcolor.hasEmission()) {
			return inter.mtlcolor.emission;
		}
		Vector3f wo = -dir;

		// TEXTURE
		if(inter.obj->isTextureActivated)
			textureModify(inter);

#if MIS
		// ******************* direct illumination ********************
		// *********************** Sample Light ********************
		// https://www.youtube.com/watch?v=xrsHo8kcCX0
		float light_pdf;
		float mis_weight_l = 0.f;
		float mat_pdf;
		float mis_weight_m = 0.f;
		Intersection light_inter;
		sampleLight(light_inter, light_pdf);
		if (!light_inter.intersected || isShadowRayBlocked(inter, light_inter.pos)) {}
		else {
			Vector3f wi = light_inter.pos - inter.pos;
			float r2 = wi.norm2();
			wi = normalized(wi);
			if (wi.dot(light_inter.nDir) > 0) {}
			else {
				mat_pdf = inter.mtlcolor.pdf(wi, wo, inter.nDir);	// w.r.t solid angle
				Vector3f light_N = normalized(light_inter.nDir);
				float cos_theta_prime = light_N.dot(-wi);
				if (cos_theta_prime <= 0) goto jmp;
				float dot = inter.nDir.dot(wi);
				float cos_theta = dot < 0 ? 0 : dot;
				// transform the pdfs to the same space: solid angle space
				// pdf_m / pdf_l = dA / dw
				// dw = dA * cos_theta_prime / r2
				// dA/dw = r2 / cos_theta_prime
				// pdf_l = pdf_m * cos_theta_prime / r2
				float pdfl = light_pdf;
				light_pdf = light_pdf * r2 / cos_theta_prime;
				mis_weight_l = getMisWeight(light_pdf, mat_pdf);
				Vector3f f_r = inter.mtlcolor.BxDF(wi, wo, inter.nDir, g->eta);
				Vector3f L_i = light_inter.mtlcolor.emission;
				if (r2 * pdfl < MIN_DIVISOR) return sampleValue;
				sampleValue = sampleValue +
					(mis_weight_l * L_i * f_r * cos_theta * cos_theta_prime / (r2 * pdfl));
			}
		}

		// *********************** Sample BSDF ********************
jmp:
		Vector3f wi; 
		if (!inter.mtlcolor.sampleDirection(wo, inter.nDir, wi))
			return sampleValue;

		mat_pdf = inter.mtlcolor.pdf(wi, wo, inter.nDir);
		Intersection x_inter;
		Vector3f rayOrig = inter.pos + inter.nDir * EPSILON;
		interStrategy->UpdateInter(x_inter, g->scene, rayOrig, wi);
		if (!x_inter.intersected) {}
		else {
			float dot = inter.nDir.dot(wi);
			float cos_theta = dot < 0 ? 0 : dot;

			light_pdf = getLightPdf(x_inter);
			if (light_pdf) {
				Vector3f light_N = normalized(x_inter.nDir);
				float cos_theta_prime = light_N.dot(-wi);
				if(cos_theta_prime <= 0)
					goto jmp2;

				// transform the pdfs to the same space
				// pdf_l = pdf_m * cos_theta_prime / r2
				float r2 = (x_inter.pos - inter.pos).norm2();
				float l_pdf_transformed = light_pdf * r2 / cos_theta_prime;
				mis_weight_m = getMisWeight(mat_pdf, l_pdf_transformed);
				Vector3f f_r = inter.mtlcolor.BxDF(wi, wo, inter.nDir, g->eta);
				Vector3f L_i = x_inter.mtlcolor.emission;
				
				if (mat_pdf < MIN_DIVISOR) return sampleValue;
				sampleValue = sampleValue +
					(mis_weight_m * L_i * f_r * cos_theta / mat_pdf);
				return sampleValue;
			}
			// ******************* direct illumination ENDS ********************
			else {	// indirect illumination
jmp2:
				float rr_prob = std::max(tp.x, std::max(tp.y, tp.z));
				tp = depth > MIN_DEPTH ? tp : 1;
				if (getRandomFloat() > rr_prob)
					return sampleValue;

				Vector3f f_r = inter.mtlcolor.BxDF(wi, wo, inter.nDir, g->eta);
				Vector3f coe =  f_r * cos_theta / (mat_pdf * rr_prob);
				if (mat_pdf * rr_prob < MIN_DIVISOR) return sampleValue;
				tp = tp * coe;
				Vector3f Li = traceRay(rayOrig, wi, depth + 1, tp , &x_inter, thdID, recording );

#if RECORD
				if (recording) 
					records[thdID].append("\nDepth " + std::to_string(depth+1) +"[ret]" + Li.toString() );
#endif
				sampleValue = sampleValue + (Li * coe);
			}
			// 4/30/2024
			// by experiment it doesnt always equal 1, need to verify in the future
			// only W1(x_1) + W2(x_1) == 1
			// std::cout << mis_weight_m << " ";std::cout<<  mis_weight_l << "\n";		
		}




#else // Next Event Estimation Only
		if (inter.mtlcolor.mType == SPECULAR_REFLECTIVE)
			return calcForMirror(origin, dir, inter, depth);
		if (inter.mtlcolor.mType == PERFECT_REFRACTIVE)
			return calcForRefractive(origin, dir, inter, depth);

		Vector3f dir_illu(0.f);
		Vector3f indir_illu(0.f);
		// ****** Direct illumination
		float light_pdf;
		Intersection light_inter;
		sampleLight(light_inter, light_pdf);
		if (!light_inter.intersected) {}
		else {
			// test if the ray is blocked in the middle 
			if (isShadowRayBlocked(inter, light_inter.pos)) {}
			else {	// ray is not blocked, then calculate the direct illumination
				Vector3f L_i = light_inter.mtlcolor.emission;
				Vector3f light_N = normalized(light_inter.nDir);
				Vector3f p_to_light = normalized(light_inter.pos - inter.pos);
				float cos_theta_prime = light_N.dot(-p_to_light);
				// if light does not illuminate this direction (p_to_light is on the back side of the light)
				if (cos_theta_prime < 0) {}
				else {
					float dis2 = (light_inter.pos - inter.pos).norm2();
					float cos_theta = p_to_light.dot(inter.nDir);
					Vector3f f_r = inter.mtlcolor.BxDF(p_to_light, wo, inter.nDir, g->eta);

					dir_illu = L_i * f_r * cos_theta * cos_theta_prime / (dis2 * light_pdf);
				}
			}
		}

		// ****** Indirect Illumination
		float rr_prob = std::max(tp.x, std::max(tp.y, tp.z));
		tp = depth > MIN_DEPTH ? tp : 1;
		if (getRandomFloat() > rr_prob)
			return sampleValue;

		// inter point p to another point x
		Vector3f p_to_x_dir;
		if (!inter.mtlcolor.sampleDirection(wo, inter.nDir, p_to_x_dir))
			return sampleValue;

		Intersection x_inter;
		Vector3f rayOrig = inter.pos + inter.nDir * EPSILON;
		interStrategy->UpdateInter(x_inter, g->scene, rayOrig, p_to_x_dir);
		// calculate only when inter is on a non-emissive object
		if (x_inter.intersected && !x_inter.mtlcolor.hasEmission()) {

			float pdf = inter.mtlcolor.pdf(p_to_x_dir, wo, inter.nDir);
			float cos_theta = std::max(0.f, inter.nDir.dot(p_to_x_dir));

			Vector3f f_r = inter.mtlcolor.BxDF(p_to_x_dir, wo, inter.nDir, g->eta);
			Vector3f coe = f_r * cos_theta / (pdf * rr_prob);
			tp = tp * coe;
			if (pdf * rr_prob < MIN_DIVISOR) return sampleValue;
			Vector3f Li = traceRay(rayOrig, p_to_x_dir, depth + 1, tp, &x_inter);

			indir_illu = indir_illu + (Li * coe);
		}

		sampleValue = sampleValue + dir_illu + indir_illu;
#endif
		return sampleValue;
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



	// TBN transformation matrix to change the dir of normal
	void changeNormalDir(Intersection& inter) {
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

	void textureModify(Intersection& inter) {
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
			changeNormalDir(inter);
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
					totalArea += i->getArea();	// without lock, sometimes problem on i->getArea(), maybe due to unique_ptr
				}
			}
			firstimeCall = false;
			sampleLight_mutex.unlock();
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

	/// <summary>
	/// special case for perfect specular reflection if res is calculated by dir illu + indir illu
	/// calculate color if hit on lights
	/// </summary>
	/// <param name="origin"> ray origin </param>
	/// <param name="dir"> ray direction </param>
	/// <param name="inter"> ray object intersection</param>
	/// <param name="depth"> ray bouncing depth</param>
	/// <returns></returns>
	Vector3f calcForMirror(const Vector3f& origin, const Vector3f& dir, Intersection& inter, int depth) {
		if (depth > MAX_DEPTH) return 0;	// 2 mirror reflect forever causing stack overflow
		Vector3f p_to_x_dir;
		inter.mtlcolor.sampleDirection(normalized(-dir), inter.nDir, p_to_x_dir);

		p_to_x_dir = normalized(p_to_x_dir);

		Intersection x_inter;
		Vector3f rayOrig = inter.pos + inter.nDir * EPSILON;
		interStrategy->UpdateInter(x_inter, g->scene, rayOrig, p_to_x_dir);
		if (x_inter.intersected) {
			// float cos_pnormal_xdir = inter.nDir.dot(p_to_x_dir);
			// Vector3f wo = -dir;
			// float pdf = inter.mtlcolor.pdf(wo, p_to_x_dir, inter.nDir);
			// BRDF has reciprocity
			// Vector3f f_r = inter.mtlcolor.BxDF(p_to_x_dir, wo, inter.nDir, g->eta);

			Vector3f res = traceRay(rayOrig, p_to_x_dir, depth + 1, Vector3f(1));
			return res; // *f_r* cos_pnormal_xdir / pdf;
		}
		return 0;
	}

	/// <summary>
	/// special case for perfect refraction
	/// </summary>
	/// <param name="origin"></param>
	/// <param name="dir"></param>
	/// <param name="inter"></param>
	/// <param name="depth"></param>
	/// <returns></returns>
	Vector3f calcForRefractive(const Vector3f& origin, const Vector3f& dir, Intersection& inter, int depth) {
		if (depth > MAX_DEPTH) return 0;

		Vector3f N = inter.nDir;

		Vector3f wo = -dir;
		float eta_i = g->eta;
		float eta_t = inter.mtlcolor.eta;
		if (wo.dot(N) > 0) {
		}
		else {	// ray is inside the obj, then swap
			eta_i += eta_t;
			eta_t = eta_i - eta_t;
			eta_i = eta_i - eta_t;
			N = -N;
		}

		Vector3f p_to_x_dir;
		inter.mtlcolor.sampleDirection(wo, N, p_to_x_dir, eta_i, eta_t);
		p_to_x_dir = normalized(p_to_x_dir);
		float pdf = inter.mtlcolor.pdf(p_to_x_dir, wo, N, eta_i, eta_t);
		Vector3f f_r = inter.mtlcolor.BxDF(p_to_x_dir, wo, N, eta_i);


		// total internal reflection
		if (p_to_x_dir.norm() == 0.f) {
			p_to_x_dir = getReflectionDir(dir, N);
			pdf = 1;
		}

		Vector3f rayOrig = inter.pos;
		float cos_pnormal_xdir = 0;
		if (p_to_x_dir.dot(N) > 0) {	// if p_to_x is reflection ray
			rayOrig = rayOrig + N * EPSILON;
			//cos_pnormal_xdir = N.dot(p_to_x_dir);
		}
		else {	// if p_to_x is refraction ray
			rayOrig = rayOrig - N * EPSILON;
			//cos_pnormal_xdir = (-N).dot(p_to_x_dir);	
		}
		Vector3f res = traceRay(rayOrig, p_to_x_dir, depth + 1, Vector3f(1));

		return res; //* cos_pnormal_xdir * f_r / pdf;
	}

	float getLightPdf(Intersection& inter) {
		if (!inter.intersected) return 0;

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
					// without lock, sometimes problem on i->getArea(), maybe due to unique_ptr
					totalArea += i->getArea();
				}
			}
			firstimeCall = false;
			sampleLight_mutex.unlock();
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
};



/// <summary>
/// used in multithreads rendering. each thread call this funciton
/// </summary>
/// <param name="arg"></param>
/// <param name="threadID"></param>
/// <param name="s">start row</param>
/// <param name="e">end row exclusive</param>
void sub_render(Thread_arg* a, int threadID, int s, int e) {
	Thread_arg arg = *a;

	const Vector3f ul = *arg.ul;
	const Vector3f delta_v = *arg.delta_v;
	const Vector3f delta_h = *arg.delta_h;
	const Vector3f c_off_h = *arg.c_off_h;
	const Vector3f c_off_v = *arg.c_off_v;
	std::vector<Vector3f>* rgb_array = &(arg.output->rgb);
	const Vector3f eyePos = *arg.eyePos;
	int Ncol = arg.Ncol;
	Renderer* r = arg.r;
	PPMGenerator* g = r->g;

	for (int y = s; y < e; y++) {
		for (int x = 0; x < Ncol; x++) {
			Vector3f& color = rgb_array->at(g->getIndex(x, y));
			Vector3f rayDir = { 0,0,0 };
			Vector3f pixelPos = ul + x * delta_h + y * delta_v + c_off_v + c_off_v;
			rayDir = normalized((pixelPos - eyePos));

			// trace ray into each pixel
			Vector3f estimate;
			for (int i = 0; i < SPP; i++) {
				Vector3f res = r->traceRay(eyePos, rayDir, 0, Vector3f(1),nullptr, threadID, true);
				if(!isnan(res.x) && !isnan(res.y) && !isnan(res.z))	// wipe out the white noise
					estimate = estimate + res;	
			}
			color = estimate * SPP_inv;
		}
	}
}

// for recording
#if RECORD
void sub_render_record(Thread_arg* a, int threadID, int s, int e) {
	Thread_arg arg = *a;

	const Vector3f ul = *arg.ul;
	const Vector3f delta_v = *arg.delta_v;
	const Vector3f delta_h = *arg.delta_h;
	const Vector3f c_off_h = *arg.c_off_h;
	const Vector3f c_off_v = *arg.c_off_v;
	std::vector<Vector3f>* rgb_array = &(arg.output->rgb);
	const Vector3f eyePos = *arg.eyePos;
	int Ncol = arg.Ncol;
	Renderer* r = arg.r;
	PPMGenerator* g = r->g;

	std::ofstream fout;
	std::string outfileName;
	outfileName.append("./Records/" + std::to_string(s) + "to" + std::to_string(e) + ".txt");
	fout.open(outfileName);


	for (int y = s; y < e; y++) {
		for (int x = 0; x < Ncol; x++) {
			Vector3f& color = rgb_array->at(g->getIndex(x, y));
			Vector3f rayDir = { 0,0,0 };
			Vector3f pixelPos = ul + x * delta_h + y * delta_v + c_off_v + c_off_v;
			rayDir = normalized((pixelPos - eyePos));

			if (y >= RECORD_MIN_Y && x >= RECORD_MIN_X && y < RECORD_MAX_Y && x < RECORD_MAX_X) {
				records[threadID].append("\n------------\n(" + std::to_string(x) + ", " + std::to_string(y) + "): \n");
			}

			// trace ray into each pixel
			Vector3f estimate;
			for (int i = 0; i < SPP; i++) {

				if (y >= RECORD_MIN_Y && x >= RECORD_MIN_X && y < RECORD_MAX_Y && x < RECORD_MAX_X)
					records[threadID].append("\n  SPP = " + std::to_string(i)) + "\n";


				Vector3f res = r->traceRay(eyePos, rayDir, 0, Vector3f(1), nullptr, threadID, true);
				if (!isnan(res.x) && !isnan(res.y) && !isnan(res.z))
					estimate = estimate + res;


				if (y >= RECORD_MIN_Y && x >= RECORD_MIN_X && y < RECORD_MAX_Y && x < RECORD_MAX_X)
					records[threadID].append("  SPP = " + std::to_string(i) + " val: " + res.toString() + "\n");

			}
			estimate = estimate * SPP_inv;
			color = estimate;



			if (y >= RECORD_MIN_Y && x >= RECORD_MIN_X && y < RECORD_MAX_Y && x < RECORD_MAX_X) {
				fout << records[threadID] << "\n[Pixel(" + std::to_string(x) + ", " + std::to_string(y) + ") val]: "
					+ estimate.toString() + "\n";
			}
			records[threadID].clear();
		}
	}
	fout.close();
}
#endif

float getMisWeight(float pdf, float otherPdf) {
	// balance heuristic
	//return pdf / (pdf + otherPdf);

	// power heuristic
	return (pdf * pdf) / ((pdf + otherPdf) * (pdf + otherPdf));
}



