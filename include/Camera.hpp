#include "Vector.hpp"
#include "Texture.hpp"

// matrix is copied from SmallVCM

class Camera {
public:
	Camera() {
	}

	// initialize all the data
	void initialize(Vector3f bkgcolor) {
		fwdDir = normalized(fwdDir);
		rightDir = crossProduct(fwdDir, upDir);
		rightDir = normalized(rightDir);
		upDir = crossProduct(rightDir, fwdDir);
		upDir = normalized(upDir);


		// projection, copy from smallVCM
		Vector3f pos(
			rightDir.dot(position),
			upDir.dot(position),
			(-fwdDir).dot(position)
		);

		FrameBuffer.rgb.resize(width * height);
		FrameBuffer.rgb.assign(width * height, Vector3f(bkgcolor.x, bkgcolor.y, bkgcolor.z));	//  set to bkgcolor
		FrameBuffer.width = width;
		FrameBuffer.height = height;

		world2Cam.setRow(0, rightDir, -pos.x);
		world2Cam.setRow(1, upDir, -pos.y);
		world2Cam.setRow(2, -fwdDir, -pos.z);
		world2Cam.setRow(3, Vector3f(0.f), 1.f);

		perspective = getPerspectiveMatrix(hfov, 0.1f, 10000.f, (float)width/height);
		world2ndc = perspective * world2Cam;
		world2Raster = Mat4f::getTranslate(Vector3f(1.f, 1.f, 0)) * world2ndc;
		world2Raster = Mat4f::getScale(Vector3f(width * 0.5f, height * 0.5f, 0)) * world2Raster;

		// an img plane dist which makes each pixel area exactly 1
		float tanHalfHfov = tan(degree2Radians(hfov * 0.5f));
		imagePlaneDist = width / (2.f * tanHalfHfov);

		filmPlaneAreaInv = 1.f / (width * height);
		lensAreaInv = 1.f;
	}

	// return -1 if can't
	int raster2pxlIndex(Vector4f& raster) {
		int x = (int)raster.x;
		int y = (int)raster.y;
		if (x < 0 || x >= width || y < 0 || y >= height)
			return -1;

		return x + width * y;
	}

	int worldPos2PixelIndex(const Vector3f& pos) {
		// Vector3f raster1 = world2Raster.transformPoint(pos); 
		// float precision may skip pixels
		Vector4f raster  = world2Raster * Vector4f(pos.x , pos.y, pos.z, 1);
		raster.normalizeW();
		raster.x -= 0.5f;
		raster.y -= 0.5f;
		
		// debug
		//Vector4f pCam = world2Cam * Vector4f(pos.x, pos.y, pos.z, 1);
		//pCam.normalizeW();
		//Vector4f pNDC = perspective * pCam;
		//pNDC.normalizeW();
		//Vector4f pR = Mat4f::getTranslate(Vector3f(1.f, 1.f, 0)) * pNDC;
		//pR.normalizeW();
		//pR = Mat4f::getScale(Vector3f(width * 0.5f, height * 0.5f, 0)) * pR;

		return raster2pxlIndex(raster);
	}

public:
	Vector3f fwdDir;
	Vector3f upDir;
	Vector3f rightDir;
	Vector3f position;
	Texture FrameBuffer;

	int width;
	int height;
	int hfov;
	float imagePlaneDist;
	float filmPlaneAreaInv;
	float lensAreaInv;

	Mat4f world2Cam;
	Mat4f perspective;
	Mat4f world2ndc;
	Mat4f world2Raster;
};