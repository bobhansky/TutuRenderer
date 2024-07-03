#pragma once
// Integrator: naivept
#include "IIntegrator.hpp"



// when evaluating pixel value, "way n" parts need to correspond to each others
class NaivePT : public IIntegrator {
public:

	struct eyePathVert {
		Vector3f throughput;
		Intersection inter;
	};

	NaivePT(PPMGenerator* g, IIntersectStrategy* inters) {
		this->g = g;
		this->interStrategy = inters;
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

				rayDir = normalized(pixelPos - eyePos);
				eyeLocation = eyePos;

				Vector3f estimate;
				for (int i = 0; i < SPP; i++) {
					std::vector<eyePathVert> epverts;
					float pdfCam = 1.f;
					Vector3f wi = rayDir;

					// camera vertex, t = 0 
					Vector3f tp = 1;
					eyePathVert epv;
					epv.inter = Intersection();
					epv.inter.pos = eyePos;
					epv.inter.intersected = true;
					epv.inter.Ng = cam.fwdDir;
					epv.throughput = tp;
					epverts.emplace_back(epv);
					float dirPdf = 0;
					Vector3f orig = eyePos;

					// for next vertex
					// way 1: stick to original mesurement function
					// tp = 1;	

					// ***** way 2: 
					// 6/22/2024:
					// the pdf of first point w.r.t area == choose the pixel, the pdf_A = 1/FilmArea,
					// pdf_w = 1/FilmArea/lensArea * d^2/ camCos
					// == d^2 / (FilmArea * lensArea* camCos)
					float wi_n_cos = abs(wi.dot(epverts[0].inter.Ng)); // camCos
					float d2 = (pixelPos - cam.position).norm2();
					float pdfCam_w = d2 * cam.lensAreaInv * cam.filmPlaneAreaInv / wi_n_cos;
					// projected solid angle pdf
					tp = epverts[0].throughput * wi_n_cos / pdfCam_w;
					// ***** way 2 ends

					Intersection nxtInter;
					interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);
					if (!nxtInter.intersected)
						continue;

					// random walk to build light path
					for (int t = 1; t < MAXDEPTH; ++t) {
						eyePathVert ev;
						ev.throughput = tp;
						ev.inter = nxtInter;
						// TEXTURE
						if (ev.inter.obj->isTextureActivated) textureModify(ev.inter, g);

						//  t = 1
						epverts.emplace_back(ev);
						if (ev.inter.mtlcolor.hasEmission())
							break;
						// sample next inter
						Vector3f wo = -wi;
						auto [success, TIR] = ev.inter.mtlcolor.sampleDirection(wo, ev.inter.Ng, wi, g->eta);
						if (!success) break;

						wi = normalized(wi);
						dirPdf = ev.inter.mtlcolor.pdf(wi, wo, ev.inter.Ng, g->eta, ev.inter.mtlcolor.eta);
						if (dirPdf == 0) break;;
						if (TIR) {
							wi = normalized(getReflectionDir(wo, ev.inter.Ng));
							dirPdf = 1;
						}
						float cos = abs(wi.dot(ev.inter.Ng));
						// for next vertex
						Vector3f bsdf = ev.inter.mtlcolor.BxDF(wi, wo, ev.inter.Ng, g->eta, TIR);
						if (dirPdf < MIN_DIVISOR)
							break;
						tp = tp * bsdf * cos / dirPdf;

						// find next inter
						orig = ev.inter.pos;
						bool rayInside = ev.inter.Ng.dot(wi) < 0;
						offsetRayOrig(orig, ev.inter.Ng, rayInside);
						interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);
						if (!nxtInter.intersected)
							break;
					}
					// evaluate path contribution Contribution(s, t = 1)
					int size = epverts.size();

					eyePathVert ev = epverts[size - 1];

					// evaluate pixel contribution
					// https://agraphicsguynotes.com/posts/the_missing_primary_ray_pdf_in_path_tracing/
					float G = Geo(cam.position, cam.fwdDir, pixelPos, -cam.fwdDir);
					Vector3f l = ev.inter.mtlcolor.emission;
					Intersection pixelInter;
					pixelInter.pos = pixelPos;
					Vector3f we = We(pixelInter, cam);
					// pixel area == 1, pdf point = 1/1
					float p = cam.lensAreaInv  * cam.filmPlaneAreaInv * 1;
					// can just let res = l * tp: G, We, 1/p cancel each others.
					// Vector3f res = (1/p) * l * ev.throughput * G * we;	// way 1, original mesureament function
					Vector3f res = l * ev.throughput * we; // way 2, no connection vertices, no G term
					estimate += res;
				}
				estimate = estimate * SPP_inv;
				color = estimate;
			}
		}
	}
};