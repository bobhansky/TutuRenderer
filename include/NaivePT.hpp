#pragma once
// Integrator: bdpt
#include "IIntegrator.hpp"

#define MAXDEPTH 6

struct eyePathVert {
	Vector3f throughput;
	Intersection inter;
};

// 6/21/2024   now test a pathtracing 
class NaivePT : public IIntegrator {
public:
	NaivePT(PPMGenerator* g, IIntersectStrategy* inters) {
		this->g = g;
		this->interStrategy = inters;
	}

	// geometry term
	float Geo(Vector3f& p1, const Vector3f& n1, Vector3f& p2, const Vector3f& n2) {
		Vector3f p12p2 = p2 - p1;
		float dis2 = p12p2.norm2();
		p12p2 = normalized(p12p2);
		float cos = p12p2.dot(n1);
		float cosprime = (-p12p2).dot(n2);
		return cos * cosprime / dis2;
	}

	float We(Intersection& inter, Camera& cam) {
		Vector3f camPos = cam.position;
		Vector3f inter2cam = camPos - inter.pos;
		inter2cam = normalized(inter2cam);

		// check if inter is in the frustum
		int index = cam.worldPos2PixelIndex(inter.pos);
		if (index < 0 || index >= cam.width * cam.height) {
			return 0.f;
		}

		float cosCamera = cam.fwdDir.dot(-inter2cam);
		float distPixel2Cam = cam.imagePlaneDist / cosCamera;

		return distPixel2Cam * distPixel2Cam * cam.lensAreaInv  * cam.filmPlaneAreaInv  / (cosCamera * cosCamera);
	}

	// path tracing only consider the Contribution(s = 0, t = n) situation:
	// 0 light path vertex and n eye path vertex (the camera)
	virtual void integrate(PPMGenerator* g) {
		// refer https://rendering-memo.blogspot.com/2016/03/bidirectional-path-tracing-5-more-than.html
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

		for (int y = 0; y < g->height; y++) {
			Vector3f v_off = y * delta_v;
			//PRINT = false;
			for (int x = 0; x < g->width; x++) {
				if (x == 1345 && y == 60) {
					PRINT = true;
				}

				Vector3f& color = g->cam.FrameBuffer.rgb.at(g->getIndex(x, y));		// update this color to change the rgb array
				Vector3f h_off = x * delta_h;
				Vector3f pixelPos = ul + h_off + v_off + c_off_h + c_off_v;		// pixel center position in world space
				Vector3f rayDir;
				Vector3f eyeLocation;
				// calculate the rayDir and eyeLocation base on different projection method
				rayDir = normalized(pixelPos - eyePos);
				eyeLocation = eyePos;

				// trace ray into each pixel
				Vector3f estimate;
				for (int i = 0; i < SPP; i++) {
					//estimate = estimate + traceRay(eyePos, rayDir, 0, Vector3f(1), nullptr, -1, RECORD);
					// all light path vertices
					std::vector<eyePathVert> epverts;
					float pdfCam = 1.f;
					Vector3f wi = rayDir;

					// camera vertex, t = 0 
					Vector3f tp = 1;
					eyePathVert epv;
					epv.inter = Intersection();
					epv.inter.pos = eyePos;
					epv.inter.intersected = true;
					epv.inter.nDir = cam.fwdDir;
					epv.throughput = tp;
					epverts.emplace_back(epv);
					float dirPdf = 0;
					Vector3f orig = eyePos;

					// for next vertex
					float wi_n_cos = abs(wi.dot(epverts[0].inter.nDir));
					tp = epverts[0].throughput * wi_n_cos / 1;

					Intersection nxtInter;
					interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);
					if (!nxtInter.intersected)
						continue;

					// random walk to build light path
					for (int t = 1; t < MAXDEPTH; ++t) {
						eyePathVert lv;
						lv.throughput = tp;
						lv.inter = nxtInter;
						// TEXTURE
						if (lv.inter.obj->isTextureActivated) textureModify(lv.inter, g);

						//  t = 0
						epverts.emplace_back(lv);
						if (lv.inter.mtlcolor.hasEmission())
							break;
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
						float cos = abs(wi.dot(lv.inter.nDir));
						// for next vertex
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
							break;
					}
					// evaluate path contribution Contribution(s, t = 1)
					// vertices can't form a path
					int size = epverts.size();
					if (size < 2)
						continue;

					eyePathVert ev = epverts[size - 1];
					// only consider the case camera ray hits light:
					// s = 0, t = n
					if (!ev.inter.mtlcolor.hasEmission())
						continue;
					if (size == 2) {
						// only consider emmisive obj when there are only 2 vertices
						if (ev.inter.mtlcolor.hasEmission())
							estimate += ev.inter.mtlcolor.emission * We(ev.inter, cam);
						continue;
					}

					// evaluate pixel contribution
					// https://agraphicsguynotes.com/posts/the_missing_primary_ray_pdf_in_path_tracing/
					float G = Geo(cam.position, cam.fwdDir, pixelPos, -cam.fwdDir);
					Vector3f l = ev.inter.mtlcolor.emission;
					Intersection pixelInter;
					pixelInter.pos = pixelPos;
					Vector3f we = We(pixelInter, cam);
					// pixel area == 1, pdf point = 1/1
					float p = cam.lensAreaInv  * cam.filmPlaneAreaInv * 1;
					// can just let res = l * tp,
					// G, We, 1/p cancel each others.
					Vector3f res = (1/p) * l * ev.throughput * G * we;
					estimate += res;
				}
				estimate = estimate * SPP_inv;

				PRINT = false;
				color = estimate;
			}
		}
	}
};