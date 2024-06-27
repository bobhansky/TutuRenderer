#pragma once
// Integrator: light
#include "IIntegrator.hpp"

#define CHECK_LT 0		// checking world to raster
#define MAXDEPTH 2
#define SAMPLE 1

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

	// probablly need to delete
	void connect2Cam(Vector3f& lighttp, Intersection& inter, Camera& cam, Vector3f& wi, int pathCnt) {
		Vector3f camPos = cam.position;
		Vector3f inter2cam = camPos - inter.pos;
		float inter2camDist2 = inter2cam.norm2();
		inter2cam = normalized(inter2cam);
		float cosInter2Cam = inter.nDir.dot(inter2cam);

		// check if inter is infront of cam
		if (cam.fwdDir.dot(-inter2cam) < 0)
			return;

		Vector3f bsdf = inter.mtlcolor.BxDF(wi, inter2cam, inter.nDir, 1.f);

		// compute pdf conversion, referrence to smallVCM
		// my note:	integrating the image/film plane, we want the pdf w.r.t image plane
		// dw/dA = pA/pw, dw = dA * cos / dist^2,  dw/dA = cos / dist^2
		// pA = pw * cos / dist^2,  pw = pA * dist^2/cos
		float cosCamera = cam.fwdDir.dot(-inter2cam);
		float distPixel2Cam = cam.imagePlaneDist / cosCamera;
		// convert pdf w.r.t img plane Area to pdf w.r.t lens/camera solid angle
		float pImgA2camW = distPixel2Cam * distPixel2Cam / cosCamera;
		// convert pdf w.r.t lens/camera solid angle to pdf w.r.t interesction surface area
		float pImgA2surA = pImgA2camW * cosInter2Cam / inter2camDist2;

		// "We put the virtual image plane at such a distance from the camera origin
		// that the pixel area is one and thus the image plane sampling pdf is 1.
		// The area pdf of aHitpoint as sampled from the camera is then equal to
		// the conversion factor from image plane area density to surface area density"
		// my note: the pdf of the pixel this intersection point would be mapped to is decided, pdf_whichPixel = 1;
		// then which point inside this pixel is calculated by 1/PixelArea = 1/1;
		// joint pdf w.r.t image area: 1 * 1/1 == 1
		float pSurface2Img = 1 / pImgA2surA;

		// pathCnt here is the pixels numbers: we integrate the pixels
		Vector3f contribution = lighttp * bsdf / (pathCnt * pSurface2Img);

		Vector3f orig = inter.pos;
		offsetRayOrig(orig, inter.nDir);
		if (!isShadowRayBlocked(orig, cam.position, g)) {
			int index = cam.worldPos2PixelIndex(inter.pos);
			Vector3f col = cam.FrameBuffer.addRGB(index, contribution);
		}

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
		for (int i = 0; i < PathCnt*SAMPLE; i++) {
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
			if (!sampleLightDir(lightInter.nDir, dirPdf, wi))
				continue;
			wi = normalized(wi);

			// visible light connect eye
			Vector3f orig = lightInter.pos;
			offsetRayOrig(orig, lightInter.nDir, false);
			if (!isShadowRayBlocked(orig, cam.position, g)) {
				int index = cam.worldPos2PixelIndex(lightInter.pos);
				cam.FrameBuffer.setRGB(index, lightInter.mtlcolor.emission * We(lightInter, cam));
			}

			// 1 / light pdf
			// tp at s = 1;
			Vector3f tp = 1 / pickpdf;
			lightPathVert lpv;
			lpv.inter = lightInter;
			lpv.throughput = tp;
			lpverts.emplace_back(lpv);

			// for next vertex
			float wi_n_cos = abs(wi.dot(lpverts[0].inter.nDir));
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
			int size = lpverts.size();
			for (int s = 1; s < size; ++s) {
				lightPathVert lv = lpverts[s];
				float G = Geo(cam.position, cam.fwdDir, lv.inter.pos, lv.inter.nDir);
				Vector3f l = lightInter.mtlcolor.emission;
				Vector3f wi = normalized(lpverts[s - 1].inter.pos - lpverts[s].inter.pos);
				Vector3f wo = normalized(cam.position - lpverts[s].inter.pos);
				Vector3f bsdf = lv.inter.mtlcolor.BxDF(wi, wo, lv.inter.nDir, 1.f);
				Vector3f we = We(lv.inter, cam);
				Vector3f res = pdfCam * l * bsdf * lv.throughput * G * we;

				// connect to camera
				Vector3f orig = lv.inter.pos;
				bool rayInside = lv.inter.nDir.dot(wo) < 0;
				offsetRayOrig(orig, lv.inter.nDir, rayInside);
				if (!isShadowRayBlocked(orig, cam.position, g)) {
					int index = cam.worldPos2PixelIndex(lv.inter.pos);
					if (index == 581 + cam.width * 642)
						int a = 1;
					Vector3f col = cam.FrameBuffer.addRGB(index, res / SAMPLE);
				}
			}
		}
#endif
	}

public:


};