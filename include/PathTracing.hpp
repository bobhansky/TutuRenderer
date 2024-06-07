#pragma once
// Integrator: path
#include "IIntegrator.hpp"

class PathTracing;
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
	const Vector3f* eyePos;
	PPMGenerator* g;
	PathTracing* pt;
};

void sub_render(Thread_arg* a, int threadID, int s, int e);

// for recording
#if RECORD
void sub_render_record(Thread_arg* a, int threadID, int s, int e);
#endif


// undirectional path tracing
class PathTracing : public IIntegrator {
public:
	PathTracing(PPMGenerator* g, IIntersectStrategy* inters) {
		this->g = g;
		this->interStrategy = inters;
	}

	/// <summary>
	/// special case for NEE perfect specular reflection if res is calculated by dir illu + indir illu
	/// calculate color if hit on lights
	/// </summary>
	/// <param name="origin"> ray origin </param>
	/// <param name="dir"> ray direction </param>
	/// <param name="inter"> ray object intersection</param>
	/// <param name="depth"> ray bouncing depth</param>
	/// <returns></returns>
	Vector3f calcForMirror(const Vector3f& origin, const Vector3f& dir, Intersection& inter, int depth) {
		if (depth > MAX_DEPTH) return 0;	// 2 mirror reflect forever causing stack overflow
		Vector3f wi;
		inter.mtlcolor.sampleDirection(normalized(-dir), inter.nDir, wi);

		wi = normalized(wi);

		Intersection x_inter;
		Vector3f rayOrig = inter.pos + inter.nDir * EPSILON;
		interStrategy->UpdateInter(x_inter, g->scene, rayOrig, wi);
		if (x_inter.intersected) {
			float cos = inter.nDir.dot(wi);
			Vector3f wo = -dir;
			float pdf = inter.mtlcolor.pdf(wo, wi, inter.nDir);
			Vector3f f_r = inter.mtlcolor.BxDF(wi, wo, inter.nDir, g->eta);

			Vector3f res = traceRay(rayOrig, wi, depth + 1, Vector3f(1));
			return res * f_r * cos / pdf;
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
	Vector3f calcForRefractive(const Vector3f& origin, const Vector3f& dir, Intersection& inter, int depth,
		int thdID = -1, bool recording = false) {
		if (depth > MAX_DEPTH) return 0;
		Vector3f N = inter.nDir;

		Vector3f wo = -dir;
		float eta_i = g->eta;
		float eta_t = inter.mtlcolor.eta;

		Vector3f wi;
		auto [sampleSuccess, TIR] = inter.mtlcolor.sampleDirection(wo, N, wi, eta_i);
		wi = normalized(wi);
		float pdf = inter.mtlcolor.pdf(wi, wo, N, eta_i, eta_t);

		// total internal reflection
		if (TIR) {
			wi = normalized(getReflectionDir(wo, N));
			pdf = 1;
			if (inter.mtlcolor.mType == MICROFACET_T) {
				Vector3f interN = N;
				if (wo.dot(N) < 0) {
					interN = -N;
					std::swap(eta_i, eta_t);
				}
				Vector3f h = normalized(wo + wi);
				float F = fresnel(wo, interN, eta_i, eta_t);
				float cosTheta = interN.dot(h);
				cosTheta = abs(cosTheta);
				wi = normalized(getReflectionDir(wo, h));
				pdf = 1 * D_ndf(h, interN, inter.mtlcolor.roughness) * cosTheta / (4.f * wo.dot(h));
			}
		}
		Vector3f f_r = inter.mtlcolor.BxDF(wi, wo, N, eta_i, TIR);

		Vector3f rayOrig = inter.pos;
		float cos = 0;
		if (wi.dot(N) > 0) {	// if wi is reflection ray
			rayOrig = rayOrig + N * EPSILON;
			cos = N.dot(wi);
		}
		else {	// if wi is refraction ray
			rayOrig = rayOrig - N * EPSILON;
			cos = (-N).dot(wi);
		}
		Vector3f Li = traceRay(rayOrig, wi, depth + 1, Vector3f(1));

#if RECORD
		if (recording)
			records[thdID].append("\nDepth " + std::to_string(depth + 1) + "[ret]" + Li.toString());
#endif
		if (pdf < MIN_DIVISOR)
			return 0;
		return Li * cos * f_r / pdf;
	}

	Vector3f traceRay(const Vector3f& origin, const Vector3f& dir, int depth, Vector3f tp,
		Intersection* nxtInter = nullptr, int thdID = -1, bool recording = false) {

#if RECORD
		if (recording) {
			records[thdID].append("\n***Depth=" + std::to_string(depth) + ": Orig" + origin.toString()
				+ ", " + "Dir" + dir.toString());
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
		if (inter.mtlcolor.hasEmission())  return inter.mtlcolor.emission;

		Vector3f wo = -dir;

		// TEXTURE
		if (inter.obj->isTextureActivated)
			textureModify(inter, g);

		if (inter.mtlcolor.mType == PERFECT_REFRACTIVE
			|| inter.mtlcolor.mType == MICROFACET_T)
			return calcForRefractive(origin, dir, inter, depth, thdID, recording);

#if MIS
		// ******************* direct illumination ********************
		// *********************** Sample Light ********************
		// https://www.youtube.com/watch?v=xrsHo8kcCX0
		float light_pdf;
		float mis_weight_l = 0.f;
		float mat_pdf;
		float mis_weight_m = 0.f;
		Intersection light_inter;
		sampleLight(light_inter, light_pdf, g);

		bool rayInside = inter.nDir.dot(wo) < 0;
		Vector3f shadowRayOrig = inter.pos;
		offsetRayOrig(shadowRayOrig, inter.nDir, rayInside);
		if (!light_inter.intersected || isShadowRayBlocked(shadowRayOrig, light_inter.pos, g)) {}
		else {
			Vector3f wi = light_inter.pos - inter.pos;
			float r2 = wi.norm2();
			wi = normalized(wi);
			if (wi.dot(light_inter.nDir) > 0) {}
			else {
				mat_pdf = inter.mtlcolor.pdf(wi, wo, inter.nDir, g->eta, inter.mtlcolor.eta);	// w.r.t solid angle
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
		auto [sampleSucess, specialEvent] = inter.mtlcolor.sampleDirection(wo, inter.nDir, wi, g->eta);
		if (!sampleSucess)
			return sampleValue;

		mat_pdf = inter.mtlcolor.pdf(wi, wo, inter.nDir, g->eta, inter.mtlcolor.eta);
		Intersection x_inter;
		Vector3f rayOrig = inter.pos;
		offsetRayOrig(rayOrig, inter.nDir, wi.dot(inter.nDir) < 0);

		interStrategy->UpdateInter(x_inter, g->scene, rayOrig, wi);
		if (!x_inter.intersected) {}
		else {
			float dot = abs(inter.nDir.dot(wi));
			float cos_theta = dot;

			light_pdf = getLightPdf(x_inter, g);
			if (light_pdf) {
				Vector3f light_N = normalized(x_inter.nDir);
				float cos_theta_prime = light_N.dot(-wi);
				if (cos_theta_prime <= 0)
					goto jmp2;

				// transform the pdfs to the same space
				// pdf_l = pdf_m * cos_theta_prime / r2
				float r2 = (x_inter.pos - inter.pos).norm2();
				float l_pdf_transformed = light_pdf * r2 / cos_theta_prime;

				mis_weight_m = getMisWeight(mat_pdf, l_pdf_transformed);
				if (inter.mtlcolor.mType == PERFECT_REFLECTIVE && mat_pdf == 1.f)
					mis_weight_m = 1.f;
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
				tp = depth > MIN_DEPTH ? tp : 1;
				float rr_prob = std::max(tp.x, std::max(tp.y, tp.z));
				if (getRandomFloat() > rr_prob)
					return sampleValue;

				Vector3f f_r = inter.mtlcolor.BxDF(wi, wo, inter.nDir, g->eta);
				Vector3f coe = f_r * cos_theta / (mat_pdf * rr_prob);
				if (mat_pdf * rr_prob < MIN_DIVISOR) return sampleValue;
				tp = tp * coe;

				Vector3f Li = traceRay(rayOrig, wi, depth + 1, tp, &x_inter, thdID, recording);

#if RECORD
				if (recording)
					records[thdID].append("\nDepth " + std::to_string(depth + 1) + "[ret]" + Li.toString());
#endif
				sampleValue = sampleValue + (Li * coe);
			}
			// 4/30/2024
			// by experiment it doesnt always equal 1, need to verify in the future
			// only W1(x_1) + W2(x_1) == 1
			// std::cout << mis_weight_m << " ";std::cout<<  mis_weight_l << "\n";		
		}




#else // Next Event Estimation Only
		if (inter.mtlcolor.mType == PERFECT_REFLECTIVE)
			return calcForMirror(origin, dir, inter, depth);

		Vector3f dir_illu(0.f);
		Vector3f indir_illu(0.f);
		// ****** Direct illumination
		float light_pdf;
		Intersection light_inter;
		sampleLight(light_inter, light_pdf, g);
		if (!light_inter.intersected) {}
		else {
			// test if the ray is blocked in the middle 
			bool rayInside = inter.nDir.dot(wo) < 0;
			Vector3f shadowRayOrig = inter.pos;
			offsetRayOrig(shadowRayOrig, inter.nDir, rayInside);
			if (isShadowRayBlocked(shadowRayOrig, light_inter.pos, g)) {}
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
		tp = depth > MIN_DEPTH ? tp : 1;
		float rr_prob = std::max(tp.x, std::max(tp.y, tp.z));
		if (getRandomFloat() > rr_prob)
			return sampleValue;

		// inter point p to another point x
		Vector3f wi;
		auto [sampleSucess, TIR] = inter.mtlcolor.sampleDirection(wo, inter.nDir, wi, g->eta);
		if (!sampleSucess)
			return sampleValue;

		Intersection x_inter;
		bool rayInside = inter.nDir.dot(wi) < 0;
		Vector3f rayOrig = inter.pos;
		offsetRayOrig(rayOrig, inter.nDir, rayInside);
		interStrategy->UpdateInter(x_inter, g->scene, rayOrig, wi);
		// calculate only when inter is on a non-emissive object
		if (x_inter.intersected && !x_inter.mtlcolor.hasEmission()) {
			float pdf = inter.mtlcolor.pdf(wi, wo, inter.nDir, g->eta, inter.mtlcolor.eta);
			float cos_theta = abs(inter.nDir.dot(wi));

			Vector3f f_r = inter.mtlcolor.BxDF(wi, wo, inter.nDir, g->eta);
			Vector3f coe = f_r * cos_theta / (pdf * rr_prob);
			tp = tp * coe;
			if (pdf * rr_prob < MIN_DIVISOR) return sampleValue + dir_illu;
			Vector3f Li = traceRay(rayOrig, wi, depth + 1, tp, &x_inter);

			indir_illu = indir_illu + (Li * coe);
		}

		sampleValue = sampleValue + dir_illu + indir_illu;
#endif
		return sampleValue;
	}


	virtual Vector3f integrate(PPMGenerator* g) {
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

#if MULTITHREAD
		int rowPerthd = g->height / N_THREAD;
		std::thread thds[N_THREAD];

		for (int i = 0; i < N_THREAD - 1; i++) {
			Thread_arg arg{
				&ul,
				&delta_v,
				&delta_h,
				&c_off_h,
				&c_off_v,
				&eyePos,
				g,
				this
			};
#if RECORD
			thds[i] = std::thread(&sub_render_record, &arg, i, (i * rowPerthd), ((i + 1) * rowPerthd));
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
				&eyePos,
				g,
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


		// Single thread
#else
		for (int y = 0; y < g->height; y++) {
			Vector3f v_off = y * delta_v;
			//PRINT = false;
			for (int x = 0; x < g->width; x++) {
				if (x == 695 && y == 658) {
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
					estimate = estimate + traceRay(eyePos, rayDir, 0, Vector3f(1), nullptr, -1, RECORD);
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
};

/// <summary>
/// used in multithreads Path Tracing integrator. each thread call this funciton
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
	PPMGenerator* g = arg.g;
	PathTracing* pt = arg.pt;
	std::vector<Vector3f>* rgb_array = &(g->output.rgb);
	const Vector3f eyePos = *arg.eyePos;
	int Ncol = g->width;

	for (int y = s; y < e; y++) {
		for (int x = 0; x < Ncol; x++) {
			Vector3f& color = rgb_array->at(g->getIndex(x, y));
			Vector3f rayDir = { 0,0,0 };
			Vector3f pixelPos = ul + x * delta_h + y * delta_v + c_off_v + c_off_v;
			rayDir = normalized((pixelPos - eyePos));

			// trace ray into each pixel
			Vector3f estimate;
			for (int i = 0; i < SPP; i++) {
				Vector3f res = pt->traceRay(eyePos, rayDir, 0, Vector3f(1), nullptr, threadID, RECORD);
				if (!isnan(res.x) && !isnan(res.y) && !isnan(res.z))	// wipe out the white noise
					estimate = estimate + res;
			}
			color = estimate * SPP_inv;
		}
	}
}


#if RECORD
void sub_render_record(Thread_arg* a, int threadID, int s, int e) {
	Thread_arg arg = *a;

	const Vector3f ul = *arg.ul;
	const Vector3f delta_v = *arg.delta_v;
	const Vector3f delta_h = *arg.delta_h;
	const Vector3f c_off_h = *arg.c_off_h;
	const Vector3f c_off_v = *arg.c_off_v;
	PPMGenerator* g = arg.g;
	PathTracing* pt = arg.pt;
	std::vector<Vector3f>* rgb_array = &(g->output.rgb);
	const Vector3f eyePos = *arg.eyePos;
	int Ncol = g->width;

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


				Vector3f res = pt->traceRay(eyePos, rayDir, 0, Vector3f(1), nullptr, threadID, RECORD);
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