#pragma once
// Integrator: bdpt
// only consider t >=2 case 
#include "IIntegrator.hpp"

#define MAX_PATHLENGTH 3		// MAX_PATHLENGTH + 1 vertices in total
#define CHECK 0
#define S_CHECK 1				// when checking, set MAX_PATHLENGTH to S_CHECK + T_CHECK + 1
#define T_CHECK 3

class BDPT : public IIntegrator{
public:
	struct eyePathVert {
		Vector3f throughput;
		Intersection inter;
		float pdf;
	};

	struct lightPathVert {
		Vector3f throughput;
		Intersection inter;
		float pdf;
	};

	struct misNode {
		Vector3f contrib;
		float pdf;
		int contributingPixelIndex;
		// path info, only for debugging
		int s;
		int t;
	};

	BDPT(PPMGenerator* g, IIntersectStrategy* inters) {
		this->g = g;
		this->interStrategy = inters;
	}

	// build eye path vertices
	// end if ( !intersected || intersect light ||  vertice number >= MAX_PATHLENGTH + 1)
	void buildEyePath(std::vector<eyePathVert>& epverts) {
		Vector3f tp = epverts[1].throughput;
		Intersection nxtInter = epverts[1].inter;
		Vector3f wi = normalized(epverts[1].inter.pos - epverts[0].inter.pos);
		float pdf = epverts[1].pdf;
		epverts.pop_back();
		float dirPdf = 0;	// pdf_w
		Vector3f orig;
		int size = epverts.size();

		while (size < MAX_PATHLENGTH + 1) {
			eyePathVert ev;
			ev.inter = nxtInter;
			ev.throughput = tp;
			ev.pdf = pdf;

			// TEXTURE
			if (ev.inter.obj->isTextureActivated) textureModify(ev.inter, g);

			epverts.emplace_back(ev);
			// if eye vertex == light, forming a C(t=n,s=0) case
			if (ev.inter.mtlcolor.hasEmission())
				return;

			// sample next inter
			Vector3f wo = -wi;
			auto [success, TIR] = ev.inter.mtlcolor.sampleDirection(wo, ev.inter.nDir, wi, g->eta);
			if (!success) break;

			wi = normalized(wi);
			dirPdf = ev.inter.mtlcolor.pdf(wi, wo, ev.inter.nDir, g->eta, ev.inter.mtlcolor.eta);
			if (dirPdf == 0) break;;
			if (TIR) {
				wi = normalized(getReflectionDir(wo, ev.inter.nDir));
				dirPdf = 1;
			}

			// for next vertex
			float cos = abs(wi.dot(ev.inter.nDir));
			Vector3f bsdf = ev.inter.mtlcolor.BxDF(wi, wo, ev.inter.nDir, g->eta, TIR);
			if (dirPdf < MIN_DIVISOR)
				break;
			tp = tp * bsdf * cos / dirPdf;

			// find next inter
			orig = ev.inter.pos;
			bool rayInside = ev.inter.nDir.dot(wi) < 0;
			offsetRayOrig(orig, ev.inter.nDir, rayInside);
			interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);
			if (!nxtInter.intersected)
				return;

			float G = Geo(ev.inter.pos, ev.inter.nDir, nxtInter.pos, nxtInter.nDir);
			pdf *= G * dirPdf / cos;
			size = epverts.size();
		}
	}

	// end if ( !intersected || intersect light ||  vertice number >= MAX_PATHLENGTH + 1)
	void buildLightPath(std::vector<lightPathVert>& lpverts) {
		// sample light position
		Intersection lightInter;
		float pickpdf;
		sampleLight(lightInter, pickpdf, g);
		float pdf;

		// s = 0
		Vector3f tp = 1 / pickpdf;
		lightPathVert lpv;
		lpv.inter = lightInter;
		lpv.throughput = tp;
		pdf = pickpdf;
		lpv.pdf = pdf;
		lpverts.emplace_back(lpv);

		float dirPdf;
		Vector3f wi;
		if (!sampleLightDir(lightInter.nDir, dirPdf, wi))
			return;
		wi = normalized(wi);

		// for s = 1
		float wi_n_cos = abs(wi.dot(lpverts[0].inter.nDir));
		pdf = pdf * dirPdf / wi_n_cos;
		tp = lpverts[0].throughput * wi_n_cos / dirPdf;

		Vector3f orig = lightInter.pos;
		offsetRayOrig(orig, lightInter.nDir, false);
		Intersection nxtInter;
		interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);
		if (!nxtInter.intersected)
			return;
		if (nxtInter.mtlcolor.hasEmission())
			return;
		float G = Geo(lpv.inter.pos, lpv.inter.nDir, nxtInter.pos, nxtInter.nDir);
		pdf *= G;

		// random walk
		int size = lpverts.size();
		while (size < MAX_PATHLENGTH + 1) {
			lightPathVert lv;
			lv.inter = nxtInter;
			lv.throughput = tp;
			lv.pdf = pdf;

			if (lv.inter.mtlcolor.hasEmission())
				return;

			// TEXTURE
			if (lv.inter.obj->isTextureActivated) textureModify(lv.inter, g);
			lpverts.emplace_back(lv);

			// sample next inter
			Vector3f wo = -wi;
			auto [success, TIR] = lv.inter.mtlcolor.sampleDirection(wo, lv.inter.nDir, wi, g->eta);
			if (!success) break;

			wi = normalized(wi);
			dirPdf = lv.inter.mtlcolor.pdf(wi, wo, lv.inter.nDir, g->eta, lv.inter.mtlcolor.eta);
			if (dirPdf == 0) break;;
			if (TIR) {
				wi = normalized(getReflectionDir(wo, lv.inter.nDir));
				dirPdf = 1;
			}

			// for next vertex
			float cos = abs(wi.dot(lv.inter.nDir));
			Vector3f bsdf = lv.inter.mtlcolor.BxDF(wi, wo, lv.inter.nDir, g->eta, TIR);
			if (dirPdf < MIN_DIVISOR)
				break;
			tp = tp * bsdf * cos / dirPdf;

			// find next inter
			orig = lv.inter.pos;
			bool rayInside = lv.inter.nDir.dot(wi) < 0;
			offsetRayOrig(orig, lv.inter.nDir, rayInside);
			interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);
			if (!nxtInter.intersected)
				return;

			G = Geo(lv.inter.pos, lv.inter.nDir, nxtInter.pos, nxtInter.nDir);
			pdf = pdf * G * dirPdf / cos;

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

		// through each pixel
		for (int y = 0; y < g->height; y++) {
			Vector3f v_off = y * delta_v;
			for (int x = 0; x < g->width; x++) {
				if (x == 136 && y == 316) {
					PRINT = true;
				}

				Vector3f& color = g->cam.FrameBuffer.rgb.at(g->getIndex(x, y));		// update this color to change the rgb array
				Vector3f h_off = x * delta_h;
				Vector3f pixelPos = ul + h_off + v_off + c_off_h + c_off_v;		// pixel center position in world space
				Vector3f rayDir;

				rayDir = normalized(pixelPos - eyePos);
				Vector3f estimate;
				for (int i = 0; i < SPP; i++) {
					std::vector<eyePathVert> epverts;
					std::vector<lightPathVert> lpverts;

					Vector3f wi = rayDir;
					// build eye path vertices
					// add camera point and first intersection vertex
					eyePathVert ev;
					ev.inter.pos = eyePos;
					ev.inter.intersected = true;
					ev.inter.nDir = cam.fwdDir;
					ev.throughput = Vector3f(1.f);
					ev.pdf = 1;
					epverts.emplace_back(ev);

					float wi_n_cos = abs(wi.dot(epverts[0].inter.nDir));
					float d2 = (pixelPos - cam.position).norm2();
					float pdfCam_w = d2 * cam.lensAreaInv * cam.filmPlaneAreaInv / wi_n_cos;
					float G = Geo(cam.position, cam.fwdDir, pixelPos, -cam.fwdDir);
					float pdf = G * pdfCam_w /wi_n_cos;
					Vector3f tp = epverts[0].throughput * wi_n_cos / pdfCam_w;
					Intersection eVert2;
					interStrategy->UpdateInter(eVert2, g->scene, eyePos, wi);
					if (!eVert2.intersected)
						break;

					ev.inter = eVert2;
					ev.throughput = tp;
					ev.pdf = pdf;
					epverts.emplace_back(ev);
					buildEyePath(epverts);
					buildLightPath(lpverts);

					Intersection pixelInter;
					pixelInter.pos = pixelPos;
					Vector3f we = We(pixelInter, cam);

					// compute contribution
					// only t>=2 case contribute
					if (epverts.size() < 2) 
						continue;

					for (int pathLength = 1; pathLength <= MAX_PATHLENGTH; pathLength++) {
						// for path with pathLength, list all possible strategies 
						// no s = n, t = 0 case, so s < pathLength + 1 instead of <=
						std::vector<misNode> misnodes;
						misnodes.resize(0);
						for (int s = 0; s < pathLength + 1; s++) {
							int t = pathLength + 1 - s;
#if CHECK
							// Debug purpose, only check 1 unweighted contribution
							if (t != T_CHECK || s != S_CHECK) continue;
#endif 

							// can't form path with such length
							if (t <= 0 || t > epverts.size() || s > lpverts.size()) continue;
							// s == 0 case: naive path tracing
							// no connection, so no G term
							if (s == 0) {
								eyePathVert ev = epverts[t-1];
								Vector3f l = ev.inter.mtlcolor.emission;
#if CHECK
								cam.FrameBuffer.addRGB(x + y * cam.width, we * ev.throughput * l * SPP_inv);
#else
								misNode mnode;
								mnode.contrib = we * ev.throughput * l;
								mnode.pdf = ev.pdf;
								mnode.t = t;
								mnode.s = s;
								mnode.contributingPixelIndex = x + y * cam.width;
								misnodes.emplace_back(mnode);
#endif 
								continue;
							}
							// light tracing and connect to camera case
							// the pixel getting contribution need to be reevaluated
							if (t == 1) {
								lightPathVert lv = lpverts[s - 1];
								if (lv.inter.mtlcolor.hasEmission()) continue;
								Vector3f l = lpverts[0].inter.mtlcolor.emission;
								Vector3f orig = lv.inter.pos;
								Vector3f wo = normalized(cam.position - orig);
								bool rayInside = lv.inter.nDir.dot(wo) < 0;
								offsetRayOrig(orig, lv.inter.nDir, rayInside);
								Vector3f bsdf = lv.inter.mtlcolor.BxDF(wi, wo, lv.inter.nDir, g->eta);
								float G = Geo(cam.position, cam.fwdDir, lv.inter.pos, lv.inter.nDir);
								Vector3f we = We(lv.inter, cam);
								Vector3f res = l * bsdf * lv.throughput * G * we;
								misNode mnode;
								mnode.contrib = res;
								mnode.pdf = lv.pdf;
								mnode.t = t;
								mnode.s = s;

								// connect to camera
								if (!isShadowRayBlocked(orig, cam.position, g)) {
									int index = cam.worldPos2PixelIndex(lv.inter.pos);
#if CHECK
									cam.FrameBuffer.addRGB(index, mnode.contrib/SPP);
#else
									mnode.contributingPixelIndex = index;
									misnodes.emplace_back(mnode);
#endif
								}
								continue;
							}

							lightPathVert lv = lpverts[s - 1];
							Vector3f l = lpverts[0].inter.mtlcolor.emission;
							eyePathVert ev = epverts[t - 1];
							if (ev.inter.mtlcolor.hasEmission())
								continue;

							Vector3f connectDir = normalized(ev.inter.pos - lv.inter.pos);
							Vector3f e_wo = normalized(epverts[t - 2].inter.pos - ev.inter.pos);
							Vector3f evBSDF = ev.inter.mtlcolor.BxDF(-connectDir, e_wo, ev.inter.nDir, g->eta);

							Vector3f lvBSDF;
							Vector3f l_wo;
							if (s == 1) lvBSDF = Vector3f(1.f);
							else {
								l_wo = normalized(lpverts[s - 2].inter.pos - lv.inter.pos);
								lvBSDF = lv.inter.mtlcolor.BxDF(connectDir, l_wo, lv.inter.nDir, g->eta);
							}
							// connecting 
							// check if two points are not blocked
							Vector3f eOrig = ev.inter.pos;
							bool rayInside = e_wo.dot(ev.inter.nDir) < 0;
							offsetRayOrig(eOrig, ev.inter.nDir, rayInside);
							
							Vector3f lorig = lv.inter.pos;
							if(s == 1) lorig = lv.inter.pos;
							else {
								rayInside = l_wo.dot(lv.inter.nDir) < 0;
								offsetRayOrig(lorig, lv.inter.nDir, rayInside);
							}
							
							if (isShadowRayBlocked(eOrig, lorig, g))
								continue;

							float G = Geo(ev.inter.pos, ev.inter.nDir, lv.inter.pos, lv.inter.nDir);
							// unweighted contribution
							Vector3f contrib = we * ev.throughput * evBSDF * G * lv.throughput * lvBSDF * l;
#if CHECK
							cam.FrameBuffer.addRGB(x + y * cam.width, contrib * SPP_inv);
#else
							misNode mnode;
							mnode.contrib = contrib;
							mnode.pdf = ev.pdf * lv.pdf;
							mnode.t = t;
							mnode.s = s;
							mnode.contributingPixelIndex = x + y * cam.width;
							misnodes.emplace_back(mnode);
#endif
						}
#if !CHECK
						float pdfAll = 0.f;
						for (int i = 0; i < misnodes.size(); ++i) {
							if(misnodes[i].contrib.norm2() != 0)
								pdfAll += misnodes[i].pdf;
						}
						for (int i = 0; i < misnodes.size(); ++i) {
							if (misnodes[i].contrib.norm2() != 0 && pdfAll / misnodes[i].pdf >= MIN_DIVISOR)
							{
								Vector3f c = misnodes[i].contrib * ((misnodes[i].pdf ) / ( pdfAll));
								cam.FrameBuffer.addRGB(misnodes[i].contributingPixelIndex, c* SPP_inv); // when checking,comment it out

							}
						}
#endif
					}
				}
			}
		}
	}
};