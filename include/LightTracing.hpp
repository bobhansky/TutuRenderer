#pragma once
// Integrator: light
#include "IIntegrator.hpp"

#define CHECK 0		// checking world to raster

// light tracing / particle tracing
class LightTracing : public IIntegrator {
public:
	LightTracing(PPMGenerator* g, IIntersectStrategy* inters) {
		this->g = g;
		this->interStrategy = inters;
	}

	bool sampleLightDir(Vector3f& N, float& dirPdf, Vector3f& sampledRes) {
		// cos-weighted
		float r1 = getRandomFloat();
		float r2 = getRandomFloat();
		float cosTheta = sqrt(r1);
		float phi = 2 * M_PI * r2;

		Vector3f dir;
		float sinTheta = sqrt(std::max(0.f, 1 - r1));
		dir.x = cos(phi) * sinTheta;
		dir.y = sin(phi) * sinTheta;
		dir.z = cosTheta;

		dir = normalized(dir);

		Vector3f res = SphereLocal2world(N, dir);
		if (normalized(res).dot(N) < 0)
			false;

		sampledRes = res;
		return true;
	}


	virtual void integrate(PPMGenerator* g) {
		Camera& cam = g->cam;
#if CHECK
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
		int iteMax = cam.width * cam.height;
		for (int i = 0; i < iteMax; i++) {
			Vector3f throughput(1);
			int pathLength = 0;
			Vector3f Li;	// carried Li
			int s = 0;		// depth, s from the light path

			// sample light
			Intersection vertexInter;
			float pickpdf;
			sampleLight(vertexInter, pickpdf, g);
			// sample direction
			float dirPdf;
			Vector3f wi;
			if (!sampleLightDir(vertexInter.nDir, dirPdf, wi))
				continue;

			Li = vertexInter.mtlcolor.emission;
			// conect eye
			Vector3f orig = vertexInter.pos;
			offsetRayOrig(orig, vertexInter.nDir, false);
			if (!isShadowRayBlocked(orig, cam.position, g)) {
				int index = cam.worldPos2PixelIndex(vertexInter.pos);
				cam.FrameBuffer.setRGB(index, Li);
			}

			// direct illumi
			Intersection nxtIner;
			interStrategy->UpdateInter(nxtIner, g->scene, orig, wi);
			if (!nxtIner.intersected)
				continue;

			//throughput *= 
		}

		

#endif

	}
};