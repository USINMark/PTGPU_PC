/*
Copyright (c) 2009 David Bucciarelli (davibu@interfree.it)

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
 * Based on smallpt, a Path Tracer by Kevin Beason, 2008
 * Modified by David Bucciarelli to show the output via OpenGL/GLUT, ported
 * to C, work with float, fixed RR, ported to OpenCL, etc.
 */

#include "stdafx.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <math.h>

#ifdef WIN32
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#endif

#include "CL/cl.h"
#include "CLBVH.h"
#include "include/camera.h"
#include "include/scene.h"
#include "include/displayfunc.h"
#include "include/geom.h"

#ifndef __ANDROID__
#include <GL/glut.h>
#define M_PI       3.14159265358979323846 

bool Read(char *fileName);
#endif

/* Options */
int useGPU = 0;
int forceWorkSize = 0;

/* OpenCL variables */
static cl_context context;
static cl_command_queue commandQueue;
static cl_program program;
static cl_kernel kernelRad, kernelBvh, kernelOpt;
unsigned int workGroupSize = 1;

char bvhFileName[MAX_FN] = "include\\BVH.cl";

static Vec *colors;
static unsigned int *seeds;
Camera camera;
static int currentSample = 0;
Shape *shapes;
Poi *pois;
unsigned int shapeCnt = 0, poiCnt = 0;
TreeNode *tn, *tl;

void FreeBuffers() {
	free(seeds);
	free(colors);
	free(pixels);
}

void AllocateBuffers() {
	const int pixelCount = height * width ;
	int i;
	colors = (Vec *)malloc(sizeof(Vec)*pixelCount);

	seeds = (unsigned int *)malloc(sizeof(unsigned int)*pixelCount*2);
	for (i = 0; i < pixelCount * 2; i++) {
		seeds[i] = rand();
		if (seeds[i] < 2)
			seeds[i] = 2;
	}

	pixels = (unsigned int *)malloc(sizeof(unsigned int)*pixelCount);
}

char *ReadSources(const char *fileName) {
	FILE *file = fopen(fileName, "r");
	if (!file) {
		LOGE("Failed to open file '%s'\n", fileName);
		return NULL;
	}

	if (fseek(file, 0, SEEK_END)) {
		LOGE("Failed to seek file '%s'\n", fileName);
		return NULL;
	}

	long size = ftell(file);
	if (size == 0) {
		LOGE("Failed to check position on file '%s'\n", fileName);
		return NULL;
	}

	rewind(file);

	char *src = (char *)malloc(sizeof(char) * size + 1);
	if (!src) {
		LOGE("Failed to allocate memory for file '%s'\n", fileName);
		return NULL;
	}

	LOGI("Reading file '%s' (size %ld bytes)\n", fileName, size);
	size_t res = fread(src, 1, sizeof(char) * size, file);
	if (res != sizeof(char) * size) {
		//LOGE("Failed to read file '%s' (read %lu)\n Content: %s\n", fileName, res, src);
		//return NULL;
	}
	src[res] = '\0'; /* NULL terminated */

	fclose(file);

	return src;

}

#define MAX_STYPE 255

void SetUpOpenCL() {
	cl_device_type dType;

	if (useGPU)
		dType = CL_DEVICE_TYPE_GPU;
	else
		dType = CL_DEVICE_TYPE_CPU;

	// Select the platform
	cl_uint numPlatforms;
	cl_platform_id platform = NULL;
	cl_int status = clGetPlatformIDs(0, NULL, &numPlatforms);
	if (status != CL_SUCCESS) {
		LOGE("Failed to get OpenCL platforms\n");
		return;
	}

	if (numPlatforms > 0) {
		cl_platform_id *platforms = (cl_platform_id *)malloc(sizeof(cl_platform_id) * numPlatforms);
		status = clGetPlatformIDs(numPlatforms, platforms, NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL platform IDs\n");
			return;
		}

		unsigned int i;
		for (i = 0; i < numPlatforms; ++i) {
			char pbuf[100];
			status = clGetPlatformInfo(platforms[i],
				CL_PLATFORM_VENDOR,
				sizeof(pbuf),
				pbuf,
				NULL);

			status = clGetPlatformIDs(numPlatforms, platforms, NULL);
			if (status != CL_SUCCESS) {
				LOGE("Failed to get OpenCL platform IDs\n");
				return;
			}

			LOGI("OpenCL Platform %d: %s\n", i, pbuf);
		}

		platform = platforms[2];
		free(platforms);
	}

	// Select the device

	cl_device_id devices[32];
	cl_uint deviceCount;
	status = clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 32, devices, &deviceCount);
	if (status != CL_SUCCESS) {
		LOGE("Failed to get OpenCL device IDs\n");
		return;
	}

	int deviceFound = 0;
	cl_device_id selectedDevice;
	unsigned int i;
	for (i = 0; i < deviceCount; ++i) {
		cl_device_type type = 0;
		status = clGetDeviceInfo(devices[i],
			CL_DEVICE_TYPE,
			sizeof(cl_device_type),
			&type,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL device info: %d\n", status);
			return;
		}

		char stype[MAX_STYPE];
		switch (type) {
		case CL_DEVICE_TYPE_ALL:
			strcpy(stype, "TYPE_ALL");
			break;
		case CL_DEVICE_TYPE_DEFAULT:
			strcpy(stype, "TYPE_DEFAULT");
			break;
		case CL_DEVICE_TYPE_CPU:
			strcpy(stype, "TYPE_CPU");
			if (!useGPU && !deviceFound) {
				selectedDevice = devices[i];
				deviceFound = 1;
			}
			break;
		case CL_DEVICE_TYPE_GPU:
			strcpy(stype, "TYPE_GPU");
			if (useGPU && !deviceFound) {
				selectedDevice = devices[i];
				deviceFound = 1;
			}
			break;
		default:
			strcpy(stype, "TYPE_UNKNOWN");
			break;
		}
		LOGI("OpenCL Device %d: Type = %s\n", i, stype);

		char buf[256];
		status = clGetDeviceInfo(devices[i],
			CL_DEVICE_NAME,
			sizeof(char[256]),
			&buf,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL device info: %d\n", status);
			return;
		}

		LOGI("OpenCL Device %d: Name = %s\n", i, buf);

		cl_uint units = 0;
		status = clGetDeviceInfo(devices[i],
			CL_DEVICE_MAX_COMPUTE_UNITS,
			sizeof(cl_uint),
			&units,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL device info: %d\n", status);
			return;
		}

		LOGI("OpenCL Device %d: Compute units = %u\n", i, units);

		size_t gsize = 0;
		status = clGetDeviceInfo(devices[i],
			CL_DEVICE_MAX_WORK_GROUP_SIZE,
			sizeof(size_t),
			&gsize,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL device info: %d\n", status);
			return;
		}

		LOGI("OpenCL Device %d: Max. work group size = %d\n", i, (unsigned int)gsize);
	}

	if (!deviceFound) {
		LOGE("Unable to select an appropriate device\n");
		return;
	}

	// Create the context

	cl_context_properties cps[3] = {
		CL_CONTEXT_PLATFORM,
		(cl_context_properties)platform,
		0
	};

	cl_context_properties *cprops = (NULL == platform) ? NULL : cps;
	context = clCreateContext(
		cprops,
		1,
		&selectedDevice,
		NULL,
		NULL,
		&status);
	if (status != CL_SUCCESS) {
		LOGE("Failed to open OpenCL context\n");
		return;
	}

	/* Get the device list data */
	size_t deviceListSize;
	status = clGetContextInfo(
		context,
		CL_CONTEXT_DEVICES,
		32,
		devices,
		&deviceListSize);
	if (status != CL_SUCCESS) {
		LOGE("Failed to get OpenCL context info: %d\n", status);
		return;
	}

	/* Print devices list */
	for (i = 0; i < deviceListSize / sizeof(cl_device_id); ++i) {
		cl_device_type type = 0;
		status = clGetDeviceInfo(devices[i],
			CL_DEVICE_TYPE,
			sizeof(cl_device_type),
			&type,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL device info: %d\n", status);
			return;
		}

		char stype[MAX_STYPE];
		switch (type) {
		case CL_DEVICE_TYPE_ALL:
			strcpy(stype, "TYPE_ALL");
			break;
		case CL_DEVICE_TYPE_DEFAULT:
			strcpy(stype, "TYPE_DEFAULT");
			break;
		case CL_DEVICE_TYPE_CPU:
			strcpy(stype, "TYPE_CPU");
			break;
		case CL_DEVICE_TYPE_GPU:
			strcpy(stype, "TYPE_GPU");
			break;
		default:
			strcpy(stype, "TYPE_UNKNOWN");
			break;
		}
		LOGI("[SELECTED] OpenCL Device %d: Type = %s\n", i, stype);

		char buf[256];
		status = clGetDeviceInfo(devices[i],
			CL_DEVICE_NAME,
			sizeof(char[256]),
			&buf,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL device info: %d\n", status);
			return;
		}

		LOGI("[SELECTED] OpenCL Device %d: Name = %s\n", i, buf);

		cl_uint units = 0;
		status = clGetDeviceInfo(devices[i],
			CL_DEVICE_MAX_COMPUTE_UNITS,
			sizeof(cl_uint),
			&units,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL device info: %d\n", status);
			return;
		}

		LOGI("[SELECTED] OpenCL Device %d: Compute units = %u\n", i, units);

		size_t gsize = 0;
		status = clGetDeviceInfo(devices[i],
			CL_DEVICE_MAX_WORK_GROUP_SIZE,
			sizeof(size_t),
			&gsize,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL device info: %d\n", status);
			return;
		}

		LOGI("[SELECTED] OpenCL Device %d: Max. work group size = %d\n", i, (unsigned int)gsize);

		char strExt[1024];

		status = clGetDeviceInfo(devices[i],
			CL_DEVICE_EXTENSIONS,
			sizeof(strExt),
			&strExt,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL device info: %d\n", status);
			return;
		}

		LOGI("[SELECTED] Extensions: %s\n", strExt);
	}

	cl_command_queue_properties prop = 0;
	commandQueue = clCreateCommandQueue(
		context,
		devices[0],
		prop,
		&status);
	if (status != CL_SUCCESS) {
		LOGE("Failed to create OpenCL command queue: %d\n", status);
		return;
	}

	/*------------------------------------------------------------------------*/
	AllocateBuffers();

	/*------------------------------------------------------------------------*/
	/* Create the kernel program */
	const char *sourcesBvh = ReadSources(bvhFileName);
	program = clCreateProgramWithSource(
		context,
		1,
		&sourcesBvh,
		NULL,
		&status);
	if (status != CL_SUCCESS) {
		LOGE("Failed to open OpenCL kernel sources (BVH): %d\n", status);
		return;
	}

	status = clBuildProgram(program, 1, devices, "-I. -I/storage/emulated/0/Download/kernels/", NULL, NULL);
	if (status != CL_SUCCESS) {
		LOGE("Failed to build OpenCL kernel (BVH): %d\n", status);

		size_t retValSize;
		status = clGetProgramBuildInfo(
			program,
			devices[0],
			CL_PROGRAM_BUILD_LOG,
			0,
			NULL,
			&retValSize);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL kernel info size: %d\n", status);
			return;
		}

		char *buildLog = (char *)malloc(retValSize + 1);
		status = clGetProgramBuildInfo(
			program,
			devices[0],
			CL_PROGRAM_BUILD_LOG,
			retValSize,
			buildLog,
			NULL);
		if (status != CL_SUCCESS) {
			LOGE("Failed to get OpenCL kernel info: %d\n", status);
			return;
		}
		buildLog[retValSize] = '\0';
#if 0
		// Query binary (PTX file) size
		size_t bin_sz;
		status = clGetProgramInfo(program, CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &bin_sz, NULL);

		// Read binary (PTX file) to memory bufferf
		unsigned char *bin = (unsigned char *)malloc(bin_sz);
		status = clGetProgramInfo(program, CL_PROGRAM_BINARIES, sizeof(unsigned char *), &bin, NULL);

		char ptxFileName[255];
		strcpy(ptxFileName, bvhFileName);
		strcat(ptxFileName, ".ptx");

		// Save PTX to add_vectors_ocl.ptx
		FILE *fp = fopen(ptxFileName, "wb");
		fwrite(bin, sizeof(char), bin_sz, fp);
		fclose(fp);

		free(bin);
#endif
		LOGE("OpenCL Programm Build Log: %s\n", buildLog);

		FILE *fp = fopen("error_BVH.txt", "wt");
		fwrite(buildLog, sizeof(char), retValSize + 1, fp);
		fclose(fp);

		free(buildLog);
		return;
	}

	kernelRad = clCreateKernel(program, "kernelConstructRadixTree", &status);
	if (status != CL_SUCCESS) {
		LOGE("Failed to create OpenCL kernelConstructRadixTree kernel: %d\n", status);
		return;
	}

	kernelBvh = clCreateKernel(program, "kernelConstructBVHTree", &status);
	if (status != CL_SUCCESS) {
		LOGE("Failed to create OpenCL kernelConstructBVHTree kernel: %d\n", status);
		return;
	}

	kernelOpt = clCreateKernel(program, "kernelOptimize", &status);
	if (status != CL_SUCCESS) {
		LOGE("Failed to create OpenCL kernelOptimize kernel: %d\n", status);
		return;
	}

	// LordCRC's patch for better workGroupSize
	size_t gsize = 0;
	status = clGetKernelWorkGroupInfo(kernelRad,
		devices[0],
		CL_KERNEL_WORK_GROUP_SIZE,
		sizeof(size_t),
		&gsize,
		NULL);
	if (status != CL_SUCCESS) {
		LOGE("Failed to get OpenCL kernel work group size info: %d\n", status);
		return;
	}

	workGroupSize = (unsigned int)gsize;
	LOGI("OpenCL Device 0: kernel work group size = %d\n", workGroupSize);

	if (forceWorkSize > 0) {
		LOGI("OpenCL Device 0: forced kernel work group size = %d\n", forceWorkSize);
		workGroupSize = forceWorkSize;
	}
}

void UpdateRendering(void) {
	double startTime = WallClockTime();

	const float invWidth = 1.f / width;
	const float invHeight = 1.f / height;

	int x, y;
	for (y = 0; y < height; y++) { /* Loop over image rows */
		for (x = 0; x < width; x++) { /* Loop cols */
			const int i = (height - y - 1) * width + x;
			const int i2 = 2 * i;

			const float r1 = GetRandom(&seeds[i2], &seeds[i2 + 1]) - .5f;
			const float r2 = GetRandom(&seeds[i2], &seeds[i2 + 1]) - .5f;
			const float kcx = (x + r1) * invWidth - .5f;
			const float kcy = (y + r2) * invHeight - .5f;

			Vec rdir;
			vinit(rdir,
				camera.x.x * kcx + camera.y.x * kcy + camera.dir.x,
				camera.x.y * kcx + camera.y.y * kcy + camera.dir.y,
				camera.x.z * kcx + camera.y.z * kcy + camera.dir.z);

			Vec rorig;
			vsmul(rorig, 0.1f, rdir);
			vadd(rorig, rorig, camera.orig)

			vnorm(rdir);
			const Ray ray = {rorig, rdir};
			
			Vec r;
			r.x = r.y = r.z = 1.0f;

			RadiancePathTracing(shapes, shapeCnt, pois, poiCnt, tn, tl, 
				//vnbtn, vnbtl, nbtnCnt, 
				&ray, &seeds[i2], &seeds[i2 + 1], &r);

			if (currentSample == 0)
				colors[i] = r;
			else {
				const float k1 = currentSample;
				const float k2 = 1.f / (k1 + 1.f);
				colors[i].x = (colors[i].x * k1 + r.x) * k2;
				colors[i].y = (colors[i].y * k1 + r.y) * k2;
				colors[i].z = (colors[i].z * k1 + r.z) * k2;
			}

			pixels[y * width + x] = toInt(colors[i].x) |
					(toInt(colors[i].y) << 8) |
					(toInt(colors[i].z) << 16);
		}
	}

	const float elapsedTime = WallClockTime() - startTime;
	const float sampleSec = height * width / elapsedTime;
	printf("Rendering time %.3f sec (pass %d)  Sample/sec  %.1fK\n",
		elapsedTime, currentSample, sampleSec / 1000.f);

	currentSample++;
}

void ReInitScene() {
	currentSample = 0;
}

void ReInit(const int reallocBuffers) {
	// Check if I have to reallocate buffers
	if (reallocBuffers) {
		FreeBuffers();
		AllocateBuffers();
	}

	UpdateCamera();
	currentSample = 0;
	//glutPostRedisplay();
}

void idleFunc(void) {
	glutPostRedisplay();
}

void displayFunc(void) {
	glClear(GL_COLOR_BUFFER_BIT);
	glRasterPos2i(0, 0);

	UpdateRendering();
	glDrawPixels(width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

	glutSwapBuffers();
}

void reshapeFunc(int newWidth, int newHeight) {
	width = newWidth;
	height = newHeight;

	glViewport(0, 0, width, height);
	glLoadIdentity();
	glOrtho(0.f, width - 1.f, 0.f, height - 1.f, -1.f, 1.f);

	ReInit(1);

	glutPostRedisplay();
}

#define MOVE_STEP 10.0f
#define ROTATE_STEP (2.f * M_PI / 180.f)

void keyFunc(unsigned char key, int x, int y) {
	switch (key) {
	case 'p': {
		FILE *f = fopen("image.ppm", "w"); // Write image to PPM file.
		if (!f) {
			fprintf(stderr, "Failed to open image file: image.ppm\n");
		}
		else {
			fprintf(f, "P3\n%d %d\n%d\n", width, height, 255);

			int x, y;
			for (y = height - 1; y >= 0; --y) {
				unsigned char *p = (unsigned char *)(&pixels[y * width]);
				for (x = 0; x < width; ++x, p += 4)
					fprintf(f, "%d %d %d ", p[0], p[1], p[2]);
			}

			fclose(f);
		}
		break;
	}
	case 27: /* Escape key */
		fprintf(stderr, "Done.\n");
		exit(0);
		break;
	case ' ': /* Refresh display */
		ReInit(1);
		break;
	case 'a': {
		Vec dir = camera.x;
		vnorm(dir);
		vsmul(dir, -MOVE_STEP, dir);
		vadd(camera.orig, camera.orig, dir);
		vadd(camera.target, camera.target, dir);
		ReInit(0);
		break;
	}
	case 'd': {
		Vec dir = camera.x;
		vnorm(dir);
		vsmul(dir, MOVE_STEP, dir);
		vadd(camera.orig, camera.orig, dir);
		vadd(camera.target, camera.target, dir);
		ReInit(0);
		break;
	}
	case 'w': {
		Vec dir = camera.dir;
		vsmul(dir, MOVE_STEP, dir);
		vadd(camera.orig, camera.orig, dir);
		vadd(camera.target, camera.target, dir);
		ReInit(0);
		break;
	}
	case 's': {
		Vec dir = camera.dir;
		vsmul(dir, -MOVE_STEP, dir);
		vadd(camera.orig, camera.orig, dir);
		vadd(camera.target, camera.target, dir);
		ReInit(0);
		break;
	}
	case 'r':
		camera.orig.y += MOVE_STEP;
		camera.target.y += MOVE_STEP;
		ReInit(0);
		break;
	case 'f':
		camera.orig.y -= MOVE_STEP;
		camera.target.y -= MOVE_STEP;
		ReInit(0);
		break;
	default:
		break;
	}
}

void specialFunc(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_UP: {
		Vec t = camera.target;
		vsub(t, t, camera.orig);
		t.y = t.y * cos(-ROTATE_STEP) + t.z * sin(-ROTATE_STEP);
		t.z = -t.y * sin(-ROTATE_STEP) + t.z * cos(-ROTATE_STEP);
		vadd(t, t, camera.orig);
		camera.target = t;
		ReInit(0);
		break;
	}
	case GLUT_KEY_DOWN: {
		Vec t = camera.target;
		vsub(t, t, camera.orig);
		t.y = t.y * cos(ROTATE_STEP) + t.z * sin(ROTATE_STEP);
		t.z = -t.y * sin(ROTATE_STEP) + t.z * cos(ROTATE_STEP);
		vadd(t, t, camera.orig);
		camera.target = t;
		ReInit(0);
		break;
	}
	case GLUT_KEY_LEFT: {
		Vec t = camera.target;
		vsub(t, t, camera.orig);
		t.x = t.x * cos(-ROTATE_STEP) - t.z * sin(-ROTATE_STEP);
		t.z = t.x * sin(-ROTATE_STEP) + t.z * cos(-ROTATE_STEP);
		vadd(t, t, camera.orig);
		camera.target = t;
		ReInit(0);
		break;
	}
	case GLUT_KEY_RIGHT: {
		Vec t = camera.target;
		vsub(t, t, camera.orig);
		t.x = t.x * cos(ROTATE_STEP) - t.z * sin(ROTATE_STEP);
		t.z = t.x * sin(ROTATE_STEP) + t.z * cos(ROTATE_STEP);
		vadd(t, t, camera.orig);
		camera.target = t;
		ReInit(0);
		break;
	}
	case GLUT_KEY_PAGE_UP:
		camera.target.y += MOVE_STEP;
		ReInit(0);
		break;
	case GLUT_KEY_PAGE_DOWN:
		camera.target.y -= MOVE_STEP;
		ReInit(0);
		break;
	default:
		break;
	}
}

void InitGlut(int argc, char *argv[], char *windowTittle) {
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInit(&argc, argv);

	glutCreateWindow(windowTittle);

	glutReshapeFunc(reshapeFunc);
	glutKeyboardFunc(keyFunc);
	glutSpecialFunc(specialFunc);
	glutDisplayFunc(displayFunc);
	glutIdleFunc(idleFunc);
}

int main(int argc, char *argv[]) {
	amiSmallptCPU = 1;

	printf("Usage: %s\n", argv[0]);
	printf("Usage: %s <window width> <window height> <scene file>\n", argv[0]);

	if (argc == 4) {
		srand(time(NULL));

		width = atoi(argv[1]);
		height = atoi(argv[2]);

		Read(argv[3]);
		AddWallLight();
		
		//UpdateCamera();
		SetUpOpenCL();

		BuildBVH();
		ReInit(0);
	} else if (argc == 1) {
		//spheres = CornellSpheres;
		shapeCnt = sizeof(CornellSpheres) / sizeof(Sphere);
		shapes = (Shape *)malloc(sizeof(Shape) * shapeCnt);
		for (int i = 0; i < shapeCnt; i++)
		{
			shapes[i].type = SPHERE;
			shapes[i].s = CornellSpheres[i];
		}
		
		vinit(camera.orig, 50.f, 45.f, 205.6f);
		vinit(camera.target, 50.f, 45 - 0.042612f, 204.6);
	} else
		exit(-1);

	UpdateCamera();

	/*------------------------------------------------------------------------*/

	AllocateBuffers();

	/*------------------------------------------------------------------------*/

	InitGlut(argc, argv, (char *)"SmallPT CPU V1.6 (Written by David Bucciarelli)");

    glutMainLoop( );

	return 0;
}

void AddWallLight()
{
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt++].s = { WALL_RAD,{ WALL_RAD + 25.0f, 0.0f, 0.0f },{ 0.f, 0.f, 0.f },{ .75f, .25f, .25f }, DIFF }; /* Left */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt++].s = { WALL_RAD,{ -WALL_RAD - 25.0f, 0.0f, 0.0f },{ 0.f, 0.f, 0.f },{ .25f, .25f, .75f }, DIFF }; /* Rght */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt++].s = { WALL_RAD,{ 0.0f, 0.0f, WALL_RAD - 25.0f },{ 0.f, 0.f, 0.f },{ .75f, .75f, .75f }, DIFF }; /* Back */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt++].s = { WALL_RAD,{ 0.0f, 0.0f, -WALL_RAD + 100.0f },{ 0.f, 0.f, 0.f },{ 0.f, 0.f, 0.f }, DIFF }; /* Frnt */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt++].s = { WALL_RAD,{ 0.0f, WALL_RAD + 25.0f, 0.0f },{ 0.f, 0.f, 0.f },{ .75f, .75f, .75f }, DIFF }; /* Botm */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt++].s = { WALL_RAD,{ 0.0f, -WALL_RAD - 25.0f, 0.0f },{ 0.f, 0.f, 0.f },{ .75f, .75f, .75f }, DIFF }; /* Top */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt++].s = { 5.0f,{ 10.0f, -10.0f, 0.0f },{ 0.f, 0.f, 0.f },{ .9f, .9f, .9f }, SPEC }, /* Mirr */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt++].s = { 5.0f,{ -5.0f, -10.0f, 0.0f },{ 0.f, 0.f, 0.f },{ .9f, .9f, .9f }, REFR }, /* Glas */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt++].s = { 2.f,{ 10.0f, 15.0f, 0.0f },{ 12.f, 12.f, 12.f },{ 0.f, 0.f, 0.f }, DIFF }; /* Lite */
}

void BuildBVH()
{
	CLBVH *pCB = new CLBVH(shapes, shapeCnt, pois, poiCnt, commandQueue, context, kernelRad, kernelBvh, kernelOpt);

	pCB->buildRadixTree();
	pCB->buildBVHTree();
	//pCB->optimize();

	pCB->getTrees(&tn, &tl);
	/*
	pCB->makeNaiveBVHTree();
	pCB->getTree(&nbtn, &nbtnCnt);
	*/
}