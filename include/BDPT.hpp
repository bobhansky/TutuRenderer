#pragma once
// Integrator: bdpt
// only consider t >= 1 case 
#include "IIntegrator.hpp"
#include <omp.h>
#include <thread>;

#define MAX_PATHLENGTH 7		// MAX_PATHLENGTH + 1 vertices in total
#define CHECK 0 			    // check unweighted contribution of one single strategy
#define S_CHECK 2				// when checking, set MAX_PATHLENGTH to S_CHECK + T_CHECK + 1 for performance
#define T_CHECK 1
#define CHECK_MIS 1			// when checking, set it 1 if want MIS res

std::mutex mutex_color;
omp_lock_t omp_lock_color;

class BDPT;

struct Thread_arg_bdpt {
	const Vector3f* ul;
	const Vector3f* delta_v;
	const Vector3f* delta_h;
	const Vector3f* c_off_h;
	const Vector3f* c_off_v;
	const Vector3f* eyePos;

	PPMGenerator* g;
	const BDPT* bdpt;
};

void sub_render_bdpt(Thread_arg_bdpt* a, int threadID, int s, int e);

namespace bdpt {
	struct eyePathVert {
		Vector3f throughput;
		Intersection inter;
		float fwdPdf;
		float revPdf;
		float G; // Geo term with previous vertex
		bool isDelta;
	};

	struct lightPathVert {
		Vector3f throughput;
		Intersection inter;
		float fwdPdf;
		float revPdf;
		float G; // Geo term with previous vertex
		bool isDelta;
	};

	struct misNode {
		float carry_towardLight;	// projected solid angle * G: actually the pdf w.r.t area
		float carry_towardEye;
		bool isDelta;
	};
}

class BDPT : public IIntegrator{
public:

	BDPT(PPMGenerator* g, IIntersectStrategy* inters) {
		this->g = g;
		this->interStrategy = inters;
		omp_init_lock(&omp_lock_color);
		omp_init_lock(&light_lock_omp);
	}

	// given the actual path strategy s = s, t = t, compute the mis weight of this strategy
	float MISweight(std::vector<bdpt::eyePathVert>& epverts, std::vector<bdpt::lightPathVert>& lpverts,
		int s, int t, Camera &cam) {
		// refer to https://pbr-book.org/3ed-2018/Light_Transport_III_Bidirectional_Methods/Bidirectional_Path_Tracing 16.3.4
		// refer "A LOT" to https://rendering-memo.blogspot.com/2016/03/bidirectional-path-tracing-8-combine.html
		// direct visible light
		if (s + t == 2) return 1;
		// protect epverts and lpverts
		float pdf_tEndFwd, pdf_tEndRev, pdf_sEndFwd, pdf_sEndRev, G_connect;

		// ************************* connected vertex update *******************************
		// the  (lpverts[s-1], epverts[t-1]) need to update new pdf fwd and pdf rev
		// assume epverts[t-1] is light: before calling this function, it is already checked
		if (s == 0) {
			// epverts[t-1] is on the light
			bdpt::eyePathVert lightPrev = epverts[t - 2];
			bdpt::eyePathVert lightvert = epverts[t - 1];
			Vector3f wo = normalized(lightPrev.inter.pos - lightvert.inter.pos);
			float cos = abs(lightvert.inter.Ng.dot(wo));
			float dirpdf = cos / M_PI;
			dirpdf = dirpdf / cos;	// projected solid angle
			float pickpdf = getLightPdf(lightvert.inter, g);
			pdf_tEndFwd = pickpdf;
			pdf_tEndRev = dirpdf;
			// No sEnd pdfs
		}
		else {
			bdpt::lightPathVert sEndvert = lpverts[s - 1];
			bdpt::eyePathVert tEndvert = epverts[t - 1];
			G_connect = Geo(sEndvert.inter.pos, sEndvert.inter.Ng, tEndvert.inter.pos, tEndvert.inter.Ng);
			Vector3f tEnd2sEnd = normalized(sEndvert.inter.pos - tEndvert.inter.pos);
			// reevaluate direction pdf
			if (t == 1) {
				Vector3f cam2sEnd = normalized(sEndvert.inter.pos - tEndvert.inter.pos);
				float camcos = tEndvert.inter.Ng.dot(cam2sEnd);
				float d = cam.imagePlaneDist / camcos;
				pdf_tEndFwd = (cam.filmPlaneAreaInv * d * d / camcos) / camcos;	// projected solid angle
				pdf_tEndRev = cam.lensAreaInv;	// no actual reverse pdf

				Vector3f s2prev = normalized(lpverts[s - 2].inter.pos - sEndvert.inter.pos);
				pdf_sEndFwd = sEndvert.inter.mtlcolor.pdf(-cam2sEnd, s2prev, sEndvert.inter.Ns, g->eta, sEndvert.inter.mtlcolor.eta)
					/ abs((-cam2sEnd).dot(sEndvert.inter.Ng));
				pdf_sEndRev = sEndvert.inter.mtlcolor.pdf(s2prev, -cam2sEnd, sEndvert.inter.Ns, g->eta, sEndvert.inter.mtlcolor.eta)
					/ abs(s2prev.dot(sEndvert.inter.Ng));
			}
			else if (s == 1) {
				Vector3f light2tEnd = normalized(tEndvert.inter.pos - sEndvert.inter.pos);
				float cos = sEndvert.inter.Ng.dot(light2tEnd);
				pdf_sEndFwd = cos / M_PI / cos;	// projected solid angle
				pdf_sEndRev = sEndvert.revPdf;	// no actual reverse pdf

				Vector3f t2prev = normalized(epverts[t - 2].inter.pos - tEndvert.inter.pos);
				pdf_tEndFwd = tEndvert.inter.mtlcolor.pdf(-light2tEnd, t2prev, tEndvert.inter.Ns, g->eta, tEndvert.inter.mtlcolor.eta)
					/ abs((-light2tEnd).dot(tEndvert.inter.Ng));
				pdf_tEndRev = tEndvert.inter.mtlcolor.pdf(t2prev, -light2tEnd, tEndvert.inter.Ns, g->eta, tEndvert.inter.mtlcolor.eta)
					/ abs(t2prev.dot(tEndvert.inter.Ng));
			}
			// most cases
			else {
				Vector3f s2t = normalized(tEndvert.inter.pos - sEndvert.inter.pos);
				Vector3f s2prev = normalized(lpverts[s - 2].inter.pos - sEndvert.inter.pos);
				Vector3f t2prev = normalized(epverts[t - 2].inter.pos - tEndvert.inter.pos);

				pdf_sEndFwd = sEndvert.inter.mtlcolor.pdf(s2t, s2prev, sEndvert.inter.Ns, g->eta, sEndvert.inter.mtlcolor.eta)
					/ abs(s2t.dot((sEndvert.inter.Ng)));
				pdf_sEndRev = sEndvert.inter.mtlcolor.pdf(s2prev, s2t, sEndvert.inter.Ns, g->eta, sEndvert.inter.mtlcolor.eta)
					/ abs(s2prev.dot((sEndvert.inter.Ng)));

				pdf_tEndFwd = tEndvert.inter.mtlcolor.pdf(-s2t, t2prev, tEndvert.inter.Ns, g->eta, tEndvert.inter.mtlcolor.eta)
					/ abs((-s2t).dot(tEndvert.inter.Ng));
				pdf_tEndRev = tEndvert.inter.mtlcolor.pdf(t2prev, -s2t, tEndvert.inter.Ns, g->eta, tEndvert.inter.mtlcolor.eta)
					/ abs(t2prev.dot(tEndvert.inter.Ng));
			}
		}
		// ************************* connected vertex update ends *******************************

		// ************************* initializing mis nodes *******************************
		// if s=2,t=2: x0 x1 x2 x3 x4,		x0 on the light, x4 on the eye
		std::vector<bdpt::misNode> misnodes(s + t, bdpt::misNode());
		// initialize mis nodes k = s + t - 1
		// light path part: i < s  <= k - 1 		k - 1 because I exclude the case t = 0
		// p0 to p(k-1), 0 is toward the dir of light, k is toward the dir of camera
		int k = s + t - 1;
		// stop before the connnecting vertex lpverts[s-1]
		for (int i = 0; i < s - 1; ++i) {
			misnodes[i].carry_towardLight = (i == 0) ? 
				lpverts[0].revPdf : lpverts[i].revPdf * lpverts[i].G;
			misnodes[i].carry_towardEye = lpverts[i].fwdPdf * lpverts[i + 1].G;
			misnodes[i].isDelta = lpverts[i].isDelta;
		}
		// i = s - 1, the connecting vertex
		if (s > 0) {
			// if connecting vertex is on the light, the carry_towardEye = the light point pick pdf
			misnodes[s - 1].carry_towardLight = (s == 1) ?
				pdf_sEndRev : pdf_sEndRev * lpverts[s - 1].G;
			// no s == k + 1 case cuz no t == 0 case
			misnodes[s - 1].carry_towardEye = pdf_sEndFwd * G_connect;
			misnodes[s - 1].isDelta = lpverts[s - 1].isDelta;
		}

		// eye path parts: s < i <= k - 1,  ti the index of eye path
		for (int ti = 0; ti < t - 1; ++ti) {
			misnodes[k - ti].carry_towardEye = (ti == 0) ?
				epverts[ti].revPdf : epverts[ti].revPdf * epverts[ti].G;
			misnodes[k - ti].carry_towardLight = epverts[ti].fwdPdf * epverts[ti + 1].G;
			misnodes[k - ti].isDelta = epverts[ti].isDelta;
		}
		// for connecting vertex on eye path: t -1
		// t will always > 0
		// if connecting vertex is on the camera, the carry_towardEye = the camera point pick pdf
		misnodes[k - (t - 1)].carry_towardEye = (t == 1) ?
			pdf_tEndRev : pdf_tEndRev * epverts[t - 1].G;
		// t == s + t: s == 0 case, t end is on the light
		misnodes[k - (t - 1)].carry_towardLight = (s == 0) ?	// if s==0, misnodes[0].towardLight = pickpdf
			pdf_tEndFwd : pdf_tEndFwd * G_connect;
		misnodes[k - (t - 1)].isDelta = epverts[t - 1].isDelta;
		// ************************* initializing mis nodes ends *******************************

		float p_i_plus_1 = 1.0f;
		float denominator = 1.0f;	// include  p_s / p_s  == 1
		for (int i = s; i < k; ++i) {	// i < k cuz no t = 0 case, p_i+1 is up to k
			if (i == 0) {	// s == 0 case
				//					  Pa(x0)
				p_i_plus_1 *= misnodes[0].carry_towardLight / misnodes[1].carry_towardLight;
				if (misnodes[1].isDelta) continue;	// veach thesis 10.3.5, set pi to 0
			}
			else {
				p_i_plus_1 *= misnodes[i - 1].carry_towardEye / misnodes[i + 1].carry_towardLight;
				// i is the previous node of i_plus_1 (if i is delta, set pi and p_i+1 to 0)
				if (misnodes[i].isDelta || misnodes[i + 1].isDelta) continue;
			}
			denominator += p_i_plus_1 * p_i_plus_1;	// power heuristic
		}
		float p_i_minus_1 = 1.0f;
		for (int i = s; i > 0; --i) {
			if (i == (k + 1)) {	// k = s + t -1,  t = 0 case
			}
			else if (i == 1) {
				p_i_minus_1 *= misnodes[1].carry_towardLight / misnodes[0].carry_towardLight;
				if (misnodes[0].isDelta) continue;
			}
			else {
				p_i_minus_1 *= misnodes[i].carry_towardLight / misnodes[i - 2].carry_towardEye;
				// i-2 is the previous node of i_minus_1 (if i is delta, set pi and p_i+1 to 0)
				if (misnodes[i - 1].isDelta || misnodes[i - 2].isDelta) continue;
			}
			denominator += p_i_minus_1 * p_i_minus_1;
		}
		float res = 1 / denominator;
		if (res < MIN_DIVISOR || isnan(res) || isinf(res))
			return 0;

		return 1 / denominator;
	}

	// build eye path vertices
	// end if ( !intersected || intersect light ||  vertice number >= MAX_PATHLENGTH + 1)
	void buildEyePath(std::vector<bdpt::eyePathVert>& epverts) {
		Vector3f tp = epverts[1].throughput;
		Intersection nxtInter = epverts[1].inter;
		Vector3f wi = normalized(epverts[1].inter.pos - epverts[0].inter.pos);

		epverts.pop_back();
		float dirPdf = 0;	// pdf_w
		Vector3f orig;
		int size = epverts.size();

		while (size < MAX_PATHLENGTH + 1) {
			bdpt::eyePathVert ev;
			ev.inter = nxtInter;
			ev.throughput = tp;

			// TEXTURE
			if (ev.inter.obj->isTextureActivated) textureModify(ev.inter, g);

			// sample next inter
			Vector3f wo = -wi;
			auto [success, TIR] = ev.inter.mtlcolor.sampleDirection(wo, ev.inter.Ns, wi, g->eta);
			if (!success) break;

			wi = normalized(wi);
			dirPdf = ev.inter.mtlcolor.pdf(wi, wo, ev.inter.Ns, g->eta, ev.inter.mtlcolor.eta);
			if (TIR) {
				wi = normalized(getReflectionDir(wo, ev.inter.Ns));
				dirPdf = 1;
			}
			if (dirPdf == 0) break;;
			float cos = abs(wi.dot(ev.inter.Ng));
			ev.fwdPdf = dirPdf / cos;

			if (ev.inter.mtlcolor.mType == PERFECT_REFLECTIVE || ev.inter.mtlcolor.mType == PERFECT_REFRACTIVE) {
				ev.revPdf = ev.fwdPdf;
				ev.isDelta = true;
			}
			else {
				ev.revPdf = ev.inter.mtlcolor.pdf(wo, wi, ev.inter.Ns, g->eta, ev.inter.mtlcolor.eta);
				ev.revPdf = ev.revPdf / abs(wo.dot(ev.inter.Ng));
				ev.isDelta = false;
			}
			bdpt::eyePathVert pre = epverts[size - 1];
			ev.G = Geo(pre.inter.pos, pre.inter.Ng, ev.inter.pos, ev.inter.Ng);
			epverts.emplace_back(ev);

			// if eye vertex == light, forming a C(t=n,s=0) case
			if (ev.inter.mtlcolor.hasEmission())
				return;

			// for next vertex
			Vector3f bsdf = ev.inter.mtlcolor.BxDF(wi, wo, ev.inter.Ng, ev.inter.Ns, g->eta, false, TIR);
			if (dirPdf < MIN_DIVISOR)
				break;
			tp = tp * bsdf * cos / dirPdf;

			// find next inter
			orig = ev.inter.pos;
			bool rayInside = ev.inter.Ns.dot(wi) < 0;
			offsetRayOrig(orig, ev.inter.Ns, rayInside);
			interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);

			if (!nxtInter.intersected)
				return;

			size = epverts.size();
		}
	}

	// end if ( !intersected || intersect light ||  vertice number >= MAX_PATHLENGTH + 1)
	void buildLightPath(std::vector<bdpt::lightPathVert>& lpverts) {
		// sample light position
		Intersection lightInter;
		float pickpdf;
		sampleLight(lightInter, pickpdf, g);
		float pdf;

		// s = 1
		Vector3f tp = 1 / pickpdf;
		bdpt::lightPathVert lpv;
		lpv.inter = lightInter;
		lpv.throughput = tp;
		pdf = pickpdf;
		lpv.revPdf = pickpdf;	// no actual reverse pdf for s = 0, here just to store pickpdf
		lpv.isDelta = false;

		float dirPdf;
		Vector3f wi;
		if (!sampleLightDir(lightInter.Ng, dirPdf, wi))
			return;
		wi = normalized(wi);
		float wi_n_cos = abs(wi.dot(lightInter.Ng));
		lpv.fwdPdf = dirPdf / wi_n_cos;
		lpverts.emplace_back(lpv);
		// for s = 2
		tp = lpverts[0].throughput * wi_n_cos / dirPdf;

		Vector3f orig = lightInter.pos;
		offsetRayOrig(orig, lightInter.Ns, false);
		Intersection nxtInter;
		interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);
		if (!nxtInter.intersected)
			return;
		if (nxtInter.mtlcolor.hasEmission())
			return;

		// random walk
		int size = lpverts.size();
		while (size < MAX_PATHLENGTH) {	// no t = 0 case
			bdpt::lightPathVert lv;
			lv.inter = nxtInter;
			lv.throughput = tp;

			// TEXTURE
			if (lv.inter.obj->isTextureActivated) textureModify(lv.inter, g);

			// sample next inter
			Vector3f wo = -wi;
			auto [success, TIR] = lv.inter.mtlcolor.sampleDirection(wo, lv.inter.Ns, wi, g->eta);
			if (!success) break;

			wi = normalized(wi);
			dirPdf = lv.inter.mtlcolor.pdf(wi, wo, lv.inter.Ns, g->eta, lv.inter.mtlcolor.eta);

			if (TIR) {
				wi = normalized(getReflectionDir(wo, lv.inter.Ns));
				dirPdf = 1;
			}
			if (dirPdf == 0) break;
			float cos = abs(wi.dot(lv.inter.Ng));
			lv.fwdPdf = dirPdf / cos;

			if (lv.inter.mtlcolor.mType == PERFECT_REFLECTIVE || lv.inter.mtlcolor.mType == PERFECT_REFRACTIVE) {
				lv.revPdf = lv.fwdPdf;
				lv.isDelta = true;
			}
			else {
				lv.revPdf = lv.inter.mtlcolor.pdf(wo, wi, lv.inter.Ns, g->eta, lv.inter.mtlcolor.eta);
				lv.revPdf = lv.revPdf / abs(wo.dot(lv.inter.Ng));
				lv.isDelta = false;
			}
			bdpt::lightPathVert pre = lpverts[size - 1];
			lv.G = Geo(pre.inter.pos, pre.inter.Ng, lv.inter.pos, lv.inter.Ng);
			lpverts.emplace_back(lv);

			if (lv.inter.mtlcolor.hasEmission())
				return;

			// for next vertex
			Vector3f bsdf = lv.inter.mtlcolor.BxDF(wi, wo, lv.inter.Ng, lv.inter.Ns, g->eta, true, TIR);
			if (dirPdf < MIN_DIVISOR)
				break;
			tp = tp * bsdf * cos / dirPdf;

			// find next inter
			orig = lv.inter.pos;
			bool rayInside = lv.inter.Ns.dot(wi) < 0;
			offsetRayOrig(orig, lv.inter.Ns, rayInside);
			interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);
			if (!nxtInter.intersected)
				return;

			size = lpverts.size();
		}
	}
	

	// only consider the Contribution(s = n1, 2 <= t <= n2) situation:
	// 0 light path vertex and n eye path vertex (the camera)
	virtual void integrate(PPMGenerator* g) {
		Camera& cam = g->cam;
		Vector3f u = crossProduct(cam.fwdDir, cam.upDir);
		u = normalized(u);
		Vector3f v = crossProduct(u, cam.fwdDir);
		v = normalized(v);
		float d = cam.imagePlaneDist;
		float width_half = fabs(tan(degree2Radians(cam.hfov / 2.f)) * d);
		float aspect_ratio = cam.width / (float)cam.height;
		float height_half = width_half / aspect_ratio;

		Vector3f n = normalized(g->viewdir);
		Vector3f eyePos = cam.position;
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

#if MULTITHREAD == 0
		// through each pixel
		for (int y = 0; y < g->height; y++) {
			Vector3f v_off = y * delta_v;
			for (int x = 0; x < g->width; x++) {
				//if (x == 623 && y == 745) {
				//	PRINT = true;
				//}
				Vector3f& color = g->cam.FrameBuffer.rgb.at(g->getIndex(x, y));		// update this color to change the rgb array
				Vector3f h_off = x * delta_h;
				Vector3f pixelPos = ul + h_off + v_off + c_off_h + c_off_v;		// pixel center position in world space
				Vector3f rayDir;

				rayDir = normalized(pixelPos - eyePos);
				Vector3f estimate;
				for (int i = 0; i < SPP; i++) {
					std::vector<bdpt::eyePathVert> epverts;
					std::vector<bdpt::lightPathVert> lpverts;


					Vector3f wi = rayDir;
					// build eye path vertices
					// add camera point and first intersection vertex
					bdpt::eyePathVert ev;
					ev.inter.pos = eyePos;
					ev.inter.intersected = true;
					ev.inter.Ng = cam.fwdDir;
					ev.throughput = Vector3f(1.f);
					ev.revPdf = cam.lensAreaInv;	// no reverse pdf for t = 1 case, here just to store the pdf of sampling camera point
					
					float wi_n_cos = abs(wi.dot(cam.fwdDir));
					float d2 = (pixelPos - cam.position).norm2();
					ev.fwdPdf = d2 * cam.filmPlaneAreaInv / wi_n_cos;
					ev.fwdPdf = ev.fwdPdf / wi_n_cos;	// projected solid angle pdf
					ev.isDelta = false;

					epverts.emplace_back(ev);

					float pdfCam_w = d2 * cam.lensAreaInv * cam.filmPlaneAreaInv / wi_n_cos;
					Vector3f tp = epverts[0].throughput * wi_n_cos / pdfCam_w;
					Intersection eVert2;
					interStrategy->UpdateInter(eVert2, g->scene, eyePos, wi);
					if (!eVert2.intersected)
						break;

					ev.inter = eVert2;
					ev.throughput = tp;

					epverts.emplace_back(ev);
					buildEyePath(epverts);
					buildLightPath(lpverts);


					Intersection pixelInter;
					pixelInter.pos = pixelPos;
					Vector3f we = We(pixelInter, cam);
					Vector3f contrib;

					// compute contribution
					// only t >= 1 case contribute
					if (epverts.size() < 1) 
						continue;

					for (int pathLength = 1; pathLength <= MAX_PATHLENGTH; pathLength++) {
						// for path with pathLength, list all possible strategies 
						// no s = n, t = 0 case, so s < pathLength + 1 instead of <=
						for (int s = 0; s < pathLength + 1; s++) {
							int t = pathLength + 1 - s;
							// can't form path with such length
							if (t <= 0 || t > epverts.size() || s > lpverts.size()) continue;
#if CHECK
							// Debug purpose, only check 1 unweighted contribution
							if (t != T_CHECK || s != S_CHECK) continue;
#endif 

							// s == 0 case: naive path tracing
							// no connection, so no G term
							if (s == 0) {
								bdpt::eyePathVert ev = epverts[t-1];
								if (!ev.inter.mtlcolor.hasEmission()) continue;
								Vector3f l = ev.inter.mtlcolor.emission;
								contrib = we * ev.throughput * l * SPP_inv;
								if (contrib.norm2() == 0) continue;
								//if (isnan(contrib.x) || isinf(contrib.x)) continue;
								float misw = MISweight(epverts, lpverts, s, t, g->cam);
#if CHECK
								misw = CHECK_MIS ? misw : 1;
#endif 
								cam.FrameBuffer.addRGB(x + y * cam.width, misw * contrib);
								continue;
							}
							// light tracing and connect to camera case
							// the pixel getting contribution need to be reevaluated
							if (t == 1) {
								bdpt::lightPathVert lv = lpverts[s - 1];
								if (lv.inter.mtlcolor.hasEmission()) continue;
								Vector3f l = lpverts[0].inter.mtlcolor.emission;
								Vector3f orig = lv.inter.pos;
								Vector3f wi = normalized(cam.position - orig);
								Vector3f wo;
								bool rayInside; 
								Vector3f bsdf;
								if (s == 1) {
									bsdf = 1;
									rayInside = false;
								}
								else {
									wo = normalized(lpverts[s - 2].inter.pos - lv.inter.pos);
									rayInside =  wi.dot(lv.inter.Ns) < 0;
									bsdf = lv.inter.mtlcolor.BxDF(wi, wo, lv.inter.Ng, lv.inter.Ns, g->eta, true);
								}
								float G = Geo(cam.position, cam.fwdDir, lv.inter.pos, lv.inter.Ng);
								Vector3f we = We(lv.inter, cam);
								contrib = l * bsdf * lv.throughput * G * we * SPP_inv;
								if (contrib.norm2() == 0) continue;
								//if (isnan(contrib.x) || isinf(contrib.x)) continue;
								float misw = MISweight(epverts, lpverts, s, t, g->cam);

								// connect to camera
								offsetRayOrig(orig, lv.inter.Ns, rayInside);
								if (!isShadowRayBlocked(orig, cam.position, g) && wi.dot(cam.fwdDir) < 0) {
									int index = cam.worldPos2PixelIndex(lv.inter.pos);

#if CHECK
									misw = CHECK_MIS ? misw : 1;
#endif
									cam.FrameBuffer.addRGB(index, misw * contrib);
								}
								continue;
							}

							bdpt::lightPathVert lv = lpverts[s - 1];
							Vector3f l = lpverts[0].inter.mtlcolor.emission;
							bdpt::eyePathVert ev = epverts[t - 1];
							if (ev.inter.mtlcolor.hasEmission())
								continue;

							Vector3f connectDir = normalized(ev.inter.pos - lv.inter.pos);
							Vector3f e_wo = normalized(epverts[t - 2].inter.pos - ev.inter.pos);
							Vector3f evBSDF = ev.inter.mtlcolor.BxDF(-connectDir, e_wo, ev.inter.Ng, ev.inter.Ns, g->eta);

							Vector3f lvBSDF;
							Vector3f l_wo;
							if (s == 1) {
								if (connectDir.dot(lv.inter.Ns) >= 0)
									lvBSDF = Vector3f(1.f);
								else lvBSDF = Vector3f(0);
							}
							else {
								l_wo = normalized(lpverts[s - 2].inter.pos - lv.inter.pos);
								lvBSDF = lv.inter.mtlcolor.BxDF(connectDir, l_wo, lv.inter.Ng, lv.inter.Ns, g->eta);
							}
							// connecting 
							// check if two points are not blocked
							Vector3f eOrig = ev.inter.pos;
							bool rayInside = e_wo.dot(ev.inter.Ns) < 0;
							offsetRayOrig(eOrig, ev.inter.Ns, rayInside);
							
							Vector3f lorig = lv.inter.pos;
							if (s == 1) {
								lorig = lv.inter.pos;
								offsetRayOrig(lorig, lv.inter.Ns, false);
							}
							else {
								rayInside = l_wo.dot(lv.inter.Ns) < 0;
								offsetRayOrig(lorig, lv.inter.Ns, rayInside);
							}
							
							if (isShadowRayBlocked(eOrig, lorig, g))
								continue;

							float G = Geo(ev.inter.pos, ev.inter.Ng, lv.inter.pos, lv.inter.Ng);
							// unweighted contribution
							contrib = we * ev.throughput * evBSDF * G * lv.throughput * lvBSDF * l * SPP_inv;
							if (contrib.norm2() == 0) continue;
							//if (isnan(contrib.x) || isinf(contrib.x)) continue;
							float misw = MISweight(epverts, lpverts, s, t, g->cam);
#if CHECK
							misw = CHECK_MIS ? misw : 1;
#endif
							cam.FrameBuffer.addRGB(x + y * cam.width, misw * contrib);
						}
					}
				}
			}
		}

#elif MULTITHREAD == 1
		int rowPerthd = g->height / N_THREAD;
		std::thread thds[N_THREAD];

		for (int i = 0; i < N_THREAD - 1; i++) {
			Thread_arg_bdpt arg{
				&ul,
				&delta_v,
				&delta_h,
				&c_off_h,
				&c_off_v,
				&eyePos,
				g,
				this
			};
			// if((i * rowPerthd) <= 147 && 147 < ((i + 1) * rowPerthd))
			thds[i] = std::thread(sub_render_bdpt, &arg, i, (i * rowPerthd), ((i + 1) * rowPerthd));
		}

		// last thread
		Thread_arg_bdpt arg_end{
				&ul,
				&delta_v,
				&delta_h,
				&c_off_h,
				&c_off_v,
				&eyePos,
				g,
				this
		};
		thds[N_THREAD - 1] = std::thread(sub_render_bdpt, &arg_end, N_THREAD - 1, (N_THREAD - 1) * rowPerthd, g->height);

		for (int i = 0; i < N_THREAD; i++) {
			thds[i].join();
		}

#elif MULTITHREAD == 2
		int rowPerthd = g->height / N_THREAD;
		Thread_arg_bdpt arg{
				&ul,
				&delta_v,
				&delta_h,
				&c_off_h,
				&c_off_v,
				&eyePos,
				g,
				this
		};

		#pragma omp parallel for
		for (int i = 0; i < N_THREAD - 1; i++)
			sub_render_bdpt(&arg, i, (i * rowPerthd), ((i + 1) * rowPerthd));

		Thread_arg_bdpt arg_end{
			&ul,
			&delta_v,
			&delta_h,
			&c_off_h,
			&c_off_v,
			&eyePos,
			g,
			this
		};
		#pragma omp parallel for
		for (int i = 0; i < 1; i++)
			sub_render_bdpt(&arg_end, N_THREAD - 1, (N_THREAD - 1) * rowPerthd, g->height);
#endif
	}
};



void sub_render_bdpt(Thread_arg_bdpt* a, int threadID, int s, int e) {
	Thread_arg_bdpt arg = *a;

	const Vector3f ul = *arg.ul;
	const Vector3f delta_v = *arg.delta_v;
	const Vector3f delta_h = *arg.delta_h;
	const Vector3f c_off_h = *arg.c_off_h;
	const Vector3f c_off_v = *arg.c_off_v;
	PPMGenerator* g = arg.g;
	std::vector<Vector3f>* rgb_array = &(g->cam.FrameBuffer.rgb);
	const Vector3f eyePos = *arg.eyePos;
	int Ncol = g->width;
	Camera& cam = g->cam;
	BDPT bdpt = *a->bdpt;

	for (int y = s; y < e; y++) {
		Vector3f v_off = y * delta_v;
		for (int x = 0; x < Ncol; x++) {

			Vector3f& color = cam.FrameBuffer.rgb.at(g->getIndex(x, y));		// update this color to change the rgb array
			Vector3f h_off = x * delta_h;
			Vector3f pixelPos = ul + h_off + v_off + c_off_h + c_off_v;		// pixel center position in world space
			Vector3f rayDir;
			rayDir = normalized(pixelPos - eyePos);
			Vector3f estimate;
			//if (x == 725 && y == 709)
			//	int a = 0;

			for (int i = 0; i < SPP; i++) {
				std::vector<bdpt::eyePathVert> epverts;
				std::vector<bdpt::lightPathVert> lpverts;

				Vector3f wi = rayDir;
				// build eye path vertices
				// add camera point and first intersection vertex
				bdpt::eyePathVert ev;
				ev.inter.pos = eyePos;
				ev.inter.intersected = true;
				ev.inter.Ng = cam.fwdDir;
				ev.throughput = Vector3f(1.f);
				ev.revPdf = cam.lensAreaInv;	// no reverse pdf for t = 1 case, here just to store the pdf of sampling camera point

				float wi_n_cos = abs(wi.dot(cam.fwdDir));
				float d2 = (pixelPos - cam.position).norm2();
				ev.fwdPdf = d2 * cam.filmPlaneAreaInv / wi_n_cos;
				ev.fwdPdf = ev.fwdPdf / wi_n_cos;	// projected solid angle pdf
				ev.isDelta = false;

				epverts.emplace_back(ev);
				
				float pdfCam_w = d2 * cam.lensAreaInv * cam.filmPlaneAreaInv / wi_n_cos;
				Vector3f tp = epverts[0].throughput * wi_n_cos / pdfCam_w;
				Intersection eVert2;
				bdpt.interStrategy->UpdateInter(eVert2, g->scene, eyePos, wi);
				if (!eVert2.intersected)
					break;

				ev.inter = eVert2;
				ev.throughput = tp;

				epverts.emplace_back(ev);
				bdpt.buildEyePath(epverts);
				bdpt.buildLightPath(lpverts);

				Intersection pixelInter;
				pixelInter.pos = pixelPos;
				Vector3f we = We(pixelInter, cam);
				Vector3f contrib;

				// compute contribution
				// only t >= 1 case contribute
				if (epverts.size() < 2)
					continue;
				for (int pathLength = 1; pathLength <= MAX_PATHLENGTH; pathLength++) {
					// for path with pathLength, list all possible strategies 
					// no s = n, t = 0 case, so s < pathLength + 1 instead of <=
					for (int s = 0; s < pathLength + 1; s++) {
						int t = pathLength + 1 - s;
						// can't form path with such length
						if (t <= 0 || t > epverts.size() || s > lpverts.size()) continue;

#if CHECK
						// Debug purpose, only check 1 unweighted contribution
						if (t != T_CHECK || s != S_CHECK) continue;
#endif 
						// s == 0 case: naive path tracing
						// no connection, so no G term
						if (s == 0) {
							if (ev.inter.mtlcolor.mType == UNLIT) {
								estimate += ev.inter.mtlcolor.diffuse;
								continue;
							}
							bdpt::eyePathVert ev = epverts[t - 1];
							if (!ev.inter.mtlcolor.hasEmission()) continue;
							Vector3f l = ev.inter.mtlcolor.emission;
							contrib = we * ev.throughput * l;
							if (contrib.norm2() == 0) continue;
							if (isnan(contrib.x)) continue;

							float misw = bdpt.MISweight(epverts, lpverts, s, t, g->cam);
#if CHECK
							misw = CHECK_MIS ? misw : 1;
#endif

							estimate += misw * contrib;
							continue;
						}
						// light tracing and connect to camera case
						// the pixel getting contribution need to be reevaluated
						if (t == 1) {
							bdpt::lightPathVert lv = lpverts[s - 1];
							if (lv.inter.mtlcolor.hasEmission()) continue;
							Vector3f l = lpverts[0].inter.mtlcolor.emission;
							Vector3f orig = lv.inter.pos;
							Vector3f wi = normalized(cam.position - orig);
							Vector3f wo;
							bool rayInside;
							Vector3f bsdf;
							if (s == 1) {
								bsdf = 1;
								rayInside = false;
							}
							else {
								wo = normalized(lpverts[s - 2].inter.pos - lv.inter.pos);
								rayInside = wi.dot(lv.inter.Ng) < 0;
								bsdf = lv.inter.mtlcolor.BxDF(wi, wo, lv.inter.Ng, lv.inter.Ns, g->eta, true);
							}
							float G = Geo(cam.position, cam.fwdDir, lv.inter.pos, lv.inter.Ng);
							Vector3f we = We(lv.inter, cam);
							contrib = l * bsdf * lv.throughput * G * we * SPP_inv;
							if (contrib.norm2() == 0) continue;
							if (isnan(contrib.x)) continue;

							float misw = bdpt.MISweight(epverts, lpverts, s, t, g->cam);
#if CHECK
							misw = CHECK_MIS ? misw : 1;
#endif

							// connect to camera
							offsetRayOrig(orig, lv.inter.Ns, rayInside);
							if (!isShadowRayBlocked(orig, cam.position, g) && wi.dot(cam.fwdDir) < 0) {
								int index = cam.worldPos2PixelIndex(lv.inter.pos);
#if MULTITHREAD==1
								mutex_color.lock();
								cam.FrameBuffer.addRGB(index, misw * contrib);
								mutex_color.unlock();
#elif MULTITHREAD == 2
								omp_set_lock(&omp_lock_color);
								cam.FrameBuffer.addRGB(index, misw * contrib);
								omp_unset_lock(&omp_lock_color);
#else
								cam.FrameBuffer.addRGB(index, misw * contrib);
#endif
							}
							continue;
						}

						bdpt::lightPathVert lv = lpverts[s - 1];
						Vector3f l = lpverts[0].inter.mtlcolor.emission;
						bdpt::eyePathVert ev = epverts[t - 1];
						if (ev.inter.mtlcolor.hasEmission())
							continue;

						Vector3f connectDir = normalized(ev.inter.pos - lv.inter.pos);
						Vector3f e_wo = normalized(epverts[t - 2].inter.pos - ev.inter.pos);
						Vector3f evBSDF = ev.inter.mtlcolor.BxDF(-connectDir, e_wo, ev.inter.Ng, ev.inter.Ns, g->eta, false);

						Vector3f lvBSDF;
						Vector3f l_wo;
						if (s == 1) {
							if (connectDir.dot(lv.inter.Ns) >= 0)
								lvBSDF = Vector3f(1.f);
							else lvBSDF = Vector3f(0);
						}
						else {
							l_wo = normalized(lpverts[s - 2].inter.pos - lv.inter.pos);
							lvBSDF = lv.inter.mtlcolor.BxDF(connectDir, l_wo, lv.inter.Ng, lv.inter.Ns, g->eta, true);
						}
						// connecting 
						// check if two points are not blocked
						Vector3f eOrig = ev.inter.pos;
						bool rayInside = e_wo.dot(ev.inter.Ns) < 0;
						offsetRayOrig(eOrig, ev.inter.Ns, rayInside);

						Vector3f lorig = lv.inter.pos;
						if (s == 1) {
							lorig = lv.inter.pos;
							offsetRayOrig(lorig, lv.inter.Ns, false);
						}
						else {
							rayInside = l_wo.dot(lv.inter.Ns) < 0;
							offsetRayOrig(lorig, lv.inter.Ns, rayInside);
						}
						if (isShadowRayBlocked(eOrig, lorig, g))
							continue;

						float G = Geo(ev.inter.pos, ev.inter.Ng, lv.inter.pos, lv.inter.Ng);
						// unweighted contribution
						contrib = we * ev.throughput * evBSDF * G * lv.throughput * lvBSDF * l;
						if (contrib.norm2() == 0) continue;
						if (isnan(contrib.x)) continue;

						float misw = bdpt.MISweight(epverts, lpverts, s, t, g->cam);
#if CHECK
						misw = CHECK_MIS ? misw : 1;
#endif
						estimate += misw * contrib;
					}
				}
			}
#if MULTITHREAD == 1
			mutex_color.lock();
			cam.FrameBuffer.addRGB(x + y * cam.width, estimate * SPP_inv);
			mutex_color.unlock();
#elif MULTITHREAD == 2
			omp_set_lock(&omp_lock_color);
			cam.FrameBuffer.addRGB(x + y * cam.width, estimate* SPP_inv);
			omp_unset_lock(&omp_lock_color);
#endif
		}
	}
}
