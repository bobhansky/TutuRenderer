#pragma once
// Integrator: light
#include "IIntegrator.hpp"

#define CHECK_LT 0		// checking world to raster
#define MAXDEPTH 2

// light tracing / particle tracing
class LightTracing : public IIntegrator {
public:
	struct lightPathVert {
		Vector3f throughput;
		Intersection inter;
	};

	LightTracing(PPMGenerator* g, IIntersectStrategy* inters) {
		this->g = g;
		this->interStrategy = inters;
	}



	// light tracing only consider the Contribution(s = n, t = 1) situation:
	// n light path vertex and 1 eye path vertex (the camera)
	virtual void integrate(PPMGenerator* g) {
		// refer https://rendering-memo.blogspot.com/2016/03/bidirectional-path-tracing-5-more-than.html
		Camera& cam = g->cam;
#if CHECK_LT
		Vector3f u = crossProduct(cam.fwdDir, cam.upDir);
		u = normalized(u);
		Vector3f v = crossProduct(u, cam.fwdDir);
		v = normalized(v);

		// calculate the near plane parameters
		// we set the distance from eye to near plane to 1, namely the nearplane.z = eyePos.z - 1;
		float d = 1.f;

		// tan(hfov/2) =  nearplane.width/2 : d
		// h/2 = tan(hfov/2) * d 
		// here height_half and width_half are the h and w of the nearplane in the world space
		float width_half = fabs(tan(degree2Radians(cam.hfov / 2.f)) * d);
		float aspect_ratio = cam.width / (float)cam.height;
		float height_half = width_half / aspect_ratio;


		// we sample the center of the pixel 
		// so we need to add offset to the center later
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
				if (x == 477 && y == 420) {
					PRINT = true;
				}

				Vector3f h_off = x * delta_h;
				Vector3f pixelPos = ul + h_off + v_off + c_off_h + c_off_v;		// pixel center position in world space
				Vector3f rayDir;
				Vector3f eyeLocation;
				// calculate the rayDir and eyeLocation base on different projection method
				if (!g->parallel_projection) {	// perspective
					rayDir = normalized(pixelPos - eyePos);
					eyeLocation = eyePos;
				}

				Intersection inter = getIntersection(g->scene.BVHaccelerator->getNode(), eyePos, rayDir);
				if (!inter.intersected)
					continue;
				Vector3f ret = inter.mtlcolor.diffuse;
				Vector3f interPos = inter.pos;
				int i = cam.worldPos2PixelIndex(interPos);
				if (!cam.FrameBuffer.setRGB(i, ret)) {
					// std::cout<<"cam set rgb out of bound\n";
				}

			}
			showProgress((float)y / g->height);
		}
#else
		int PathCnt = cam.width * cam.height;
		for (int y = 0; y < cam.height; y++) {
			for (int x = 0; x < cam.width; x++) {
				for (int i = 0; i < SPP; i++) {

					// all light path vertices
					std::vector<lightPathVert> lpverts;
					float pdfCam = 1.f;

					// sample light position
					Intersection lightInter;
					float pickpdf;
					sampleLight(lightInter, pickpdf, g);
					// sample direction
					float dirPdf;
					Vector3f wi;	// ray direciton
					if (!sampleLightDir(lightInter.Ng, dirPdf, wi))
						continue;
					wi = normalized(wi);

					// visible light connect eye
					Vector3f orig = lightInter.pos;
					offsetRayOrig(orig, lightInter.Ns, false);
					if (!isShadowRayBlocked(orig, cam.position, g)) {
						int index = cam.worldPos2PixelIndex(lightInter.pos);

						cam.FrameBuffer.setRGB(index, lightInter.mtlcolor.emission * We(lightInter, cam) * SPP_inv);
					}

					// 1 / light pdf
					// tp at s = 1;
					Vector3f tp = 1 / pickpdf;
					lightPathVert lpv;
					lpv.inter = lightInter;
					lpv.throughput = tp;
					lpverts.emplace_back(lpv);

					// for next vertex
					float wi_n_cos = abs(wi.dot(lpverts[0].inter.Ng));
					tp = lpverts[0].throughput * wi_n_cos / dirPdf;

					Intersection nxtInter;
					interStrategy->UpdateInter(nxtInter, g->scene, orig, wi);
					if (!nxtInter.intersected)
						continue;


					// random walk to build light path
					for (int s = 1; s < MAXDEPTH; ++s) {
						lightPathVert lv;
						lv.throughput = tp;
						lv.inter = nxtInter;
						// TEXTURE
						if (lv.inter.obj->isTextureActivated)
							textureModify(lv.inter, g);

						lpverts.emplace_back(lv);
						// sample next inter
						Vector3f wo = -wi;
						auto [success, TIR] = lv.inter.mtlcolor.sampleDirection(wo, lv.inter.Ns, wi, g->eta);
						if (!success) break;

						wi = normalized(wi);
						dirPdf = lv.inter.mtlcolor.pdf(wi, wo, lv.inter.Ns, g->eta, lv.inter.mtlcolor.eta);
						if (dirPdf == 0) break;;
						if (TIR) {
							wi = normalized(getReflectionDir(wo, lv.inter.Ns));
							dirPdf = 1;
						}
						float cos = abs(wi.dot(lv.inter.Ng));
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
							break;
					}

					// evaluate path contribution Contribution(s, t = 1)
					int size = lpverts.size();
					for (int s = 1; s < size; ++s) {
						lightPathVert lv = lpverts[s];
						float G = Geo(cam.position, cam.fwdDir, lv.inter.pos, lv.inter.Ng);
						Vector3f l = lightInter.mtlcolor.emission;
						Vector3f wo = normalized(lpverts[s - 1].inter.pos - lpverts[s].inter.pos);
						Vector3f wi = normalized(cam.position - lpverts[s].inter.pos);
						Vector3f bsdf = lv.inter.mtlcolor.BxDF(wi, wo, lv.inter.Ng, lv.inter.Ns, 1.f, true);
						Vector3f we = We(lv.inter, cam);
						Vector3f res = pdfCam * l * bsdf * lv.throughput * G * we;

						// connect to camera
						Vector3f orig = lv.inter.pos;
						bool rayInside = lv.inter.Ns.dot(wo) < 0;
						offsetRayOrig(orig, lv.inter.Ns, rayInside);
						if (!isShadowRayBlocked(orig, cam.position, g)) {
							int index = cam.worldPos2PixelIndex(lv.inter.pos);
							Vector3f col = cam.FrameBuffer.addRGB(index, res * SPP_inv);
						}
					}
				}
			}
			
		}
#endif
	}

public:


};