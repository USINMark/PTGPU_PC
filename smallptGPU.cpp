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

// Jens's patch for MacOS
#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include "CL/cl.h"
#endif

#include "include/CLBVH.h"
#include "include/KDTree.h"
#include "include/camera.h"
#include "include/scene.h"
#include "include/displayfunc.h"
#include "include/geom.h"
#include "include/geomfunc.h"
#include "include/native-lib.h"

#ifndef __ANDROID__
#include <GL/glut.h>
#define M_PI       3.14159265358979323846 

bool Read(char *fileName, bool *walllight);
#else 
#include <android/asset_manager.h>
#endif

#ifdef EXP_KERNEL
Ray *ray;
Vec *throughput, *result;
int *specularBounce, *terminated;

static cl_mem rayBuffer, throughputBuffer, specularBounceBuffer, terminatedBuffer, resultBuffer;
static cl_kernel kernelGen, kernelRadiance, kernelFill;
char kernelFileName[MAX_FN] = "include\\rendering_kernel_exp.cl";
#else
static cl_kernel kernel;
char kernelFileName[MAX_FN] = "include\\rendering_kernel.cl";
#endif

#ifdef CPU_PARTRENDERING
static cl_kernel kernelBox;
#endif

#if (ACCELSTR == 1)
static cl_mem btnBuffer;
static cl_mem btlBuffer;
BVHNodeGPU *btn, *btl;
static cl_kernel kernelRad, kernelBvh, kernelOpt;
char bvhFileName[MAX_FN] = "include\\BVH.cl";
#elif (ACCELSTR == 2)
static cl_mem kngBuffer;
static cl_mem knBuffer;
KDNodeGPU *pkngbuf;
int *pknbuf; 
int kngCnt;
int knCnt;
#endif

#define MAX_STYPE 255
#define MAX_INCLUDE 255
#define MAX_ERROR 255
#define MAX_LOG 255
//#define PTX_ERROR

/* OpenCL variables */
static cl_context context;
static cl_mem colorBuffer, pixelBuffer, seedBuffer, shapeBuffer, cameraBuffer;
static cl_command_queue commandQueue;
static cl_program program;

static Vec *colors;
static unsigned int *seeds;
static int currentSample = 0;

/* Options */
int useGPU = 1;
int forceWorkSize = 0;

unsigned int workGroupSize = 1, shapeCnt = 0, lightCnt = 0;
int pixelCount;
Camera camera;
Shape *shapes;

#if (ACCELSTR == 1)
void BuildBVH();
#elif (ACCELSTR == 2)
void BuildKDtree();
#endif

#define clErrchk(ans) { clAssert((ans), __FILE__, __LINE__); }

inline void clAssert(cl_int code, const char *file, int line)
{
	if (code != CL_SUCCESS)
	{
		LOGI("Error: %d in %s (%d)\n", code, file, line);
	}
}

void FreeBuffers() {
#ifdef EXP_KERNEL
	if (resultBuffer)
	{
		clErrchk(clReleaseMemObject(resultBuffer));
	}
	if (terminatedBuffer)
	{
		clErrchk(clReleaseMemObject(terminatedBuffer));
	}
	if (specularBounceBuffer)
	{
		clErrchk(clReleaseMemObject(specularBounceBuffer));
	}
	if (throughputBuffer)
	{
		clErrchk(clReleaseMemObject(throughputBuffer));
	}
	if (rayBuffer)
	{
		clErrchk(clReleaseMemObject(rayBuffer));
	}
#endif
	if (seedBuffer)
	{
		clErrchk(clReleaseMemObject(seedBuffer));
	}	
	if (pixelBuffer)
	{
		clErrchk(clReleaseMemObject(pixelBuffer));
	}
	if (colorBuffer)
	{
		clErrchk(clReleaseMemObject(colorBuffer));
	}

#ifdef EXP_KERNEL
	if (result) free(result);
	if (terminated) free(terminated);
	if (specularBounce) free(specularBounce);
	if (throughput) free(throughput);
	if (ray) free(ray);
#endif
	if (seeds) free(seeds);
	if (colors) free(colors);
	if (pixels) free(pixels);
}

void AllocateBuffers() {
	pixelCount = width * height;
	int i;
    
	colors = (Vec *)malloc(sizeof(Vec) * pixelCount);
	seeds = (unsigned int *)malloc(sizeof(unsigned int) * pixelCount * 2);

	for (i = 0; i < pixelCount * 2; i++) {
		seeds[i] = rand();
		if (seeds[i] < 2) seeds[i] = 2;
	}

	pixels = (unsigned int *)malloc(sizeof(unsigned int) * pixelCount);

	cl_int status;
	cl_uint sizeBytes = sizeof(Vec) * width * height;

    colorBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeBytes, NULL, &status);
	clErrchk(status);
	
	sizeBytes = sizeof(unsigned char[4]) * width * height;
    pixelBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeBytes, NULL, &status);
	clErrchk(status);

	sizeBytes = sizeof(unsigned int) * width * height * 2;
	seedBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeBytes, NULL, &status);
	clErrchk(status);

	status = clEnqueueWriteBuffer(commandQueue, colorBuffer, CL_TRUE, 0, sizeBytes, colors, 0, NULL, NULL);
	clErrchk(status);

	status = clEnqueueWriteBuffer(commandQueue, seedBuffer, CL_TRUE, 0, sizeBytes, seeds, 0, NULL, NULL);
	clErrchk(status);

#ifdef EXP_KERNEL
	ray = (Ray *)malloc(sizeof(Ray) * width * height);

	rayBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Ray) *  width * height, NULL, &status);
	clErrchk(status);

	throughput = (Vec *)malloc(sizeof(Vec) * width * height);

	throughputBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Ray) *  width * height, NULL, &status);
	clErrchk(status);

	specularBounce = (int *)malloc(sizeof(int) * width * height);

	specularBounceBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Ray) *  width * height, NULL, &status);
	clErrchk(status);

	terminated = (int *)malloc(sizeof(int) * width * height);

	terminatedBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Ray) *  width * height, NULL, &status);
	clErrchk(status);

	result = (Vec *)malloc(sizeof(Vec) * width * height);

	resultBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Vec) *  width * height, NULL, &status);
	clErrchk(status);	
#endif
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

void SetUpOpenCL() {
	cl_device_type dType;

	if (useGPU) dType = CL_DEVICE_TYPE_GPU;
	else dType = CL_DEVICE_TYPE_CPU;

	// Select the platform
    cl_uint numPlatforms;
	cl_platform_id platform = NULL;

	clErrchk(clGetPlatformIDs(0, NULL, &numPlatforms));

	if (numPlatforms > 0) {
		cl_platform_id *platforms = (cl_platform_id *)malloc(sizeof(cl_platform_id) * numPlatforms);

		clErrchk(clGetPlatformIDs(numPlatforms, platforms, NULL));

		unsigned int i;
		for (i = 0; i < numPlatforms; ++i) {
			char pbuf[100];

			clErrchk(clGetPlatformInfo(platforms[i], CL_PLATFORM_VENDOR, sizeof(pbuf), pbuf, NULL));			
			clErrchk(clGetPlatformIDs(numPlatforms, platforms, NULL));
			
			LOGI("OpenCL Platform %d: %s\n", i, pbuf);
		}

		platform = platforms[0];
		free(platforms);
	}

	// Select the device
	cl_device_id devices[32];
	cl_uint deviceCount;

	clErrchk(clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 32, devices, &deviceCount));
	
	int deviceFound = 0;
	cl_device_id selectedDevice;
	unsigned int i;

	for (i = 0; i < deviceCount; ++i) {
		cl_device_type type = 0;

		clErrchk(clGetDeviceInfo(devices[i], CL_DEVICE_TYPE, sizeof(cl_device_type), &type, NULL));
		
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
		clErrchk(clGetDeviceInfo(devices[i], CL_DEVICE_NAME, sizeof(char[256]), &buf, NULL));		

		LOGI("OpenCL Device %d: Name = %s\n", i, buf);

		cl_uint units = 0;
		clErrchk(clGetDeviceInfo(devices[i], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(cl_uint), &units, NULL));		

		LOGI("OpenCL Device %d: Compute units = %u\n", i, units);

		size_t gsize = 0;
		clErrchk(clGetDeviceInfo(devices[i], CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(size_t), &gsize, NULL));
		
		LOGI("OpenCL Device %d: Max. work group size = %d\n", i, (unsigned int)gsize);
	}

	if (!deviceFound) {
		LOGE("Unable to select an appropriate device\n");
		exit(0);
		return ;
	}

	// Create the context
	cl_context_properties cps[3] = { CL_CONTEXT_PLATFORM, (cl_context_properties) platform, 0 };
	cl_context_properties *cprops = (NULL == platform) ? NULL : cps;
	cl_int status;

	context = clCreateContext(cprops, 1, &selectedDevice, NULL, NULL, &status);
	clErrchk(status);

    /* Get the device list data */
	size_t deviceListSize;
	clErrchk(clGetContextInfo(context, CL_CONTEXT_DEVICES, 32, devices, &deviceListSize));
	
	/* Print devices list */
	for (i = 0; i < deviceListSize / sizeof(cl_device_id); ++i) {
		cl_device_type type = 0;
		char stype[MAX_STYPE];

		clErrchk(clGetDeviceInfo(devices[i], CL_DEVICE_TYPE, sizeof(cl_device_type), &type, NULL));
		
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
		clErrchk(clGetDeviceInfo(devices[i], CL_DEVICE_NAME, sizeof(char[256]), &buf, NULL));
		
		LOGI("[SELECTED] OpenCL Device %d: Name = %s\n", i, buf);

		cl_uint units = 0;
		clErrchk(clGetDeviceInfo(devices[i], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(cl_uint), &units, NULL));
		
		LOGI("[SELECTED] OpenCL Device %d: Compute units = %u\n", i, units);

		size_t gsize = 0;
		clErrchk(clGetDeviceInfo(devices[i], CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(size_t), &gsize, NULL));
		
		LOGI("[SELECTED] OpenCL Device %d: Max. work group size = %d\n", i, (unsigned int)gsize);
	}

	cl_command_queue_properties prop = 0;

	commandQueue = clCreateCommandQueue(context, devices[0], prop, &status);
	clErrchk(status);	

	/*------------------------------------------------------------------------*/
#ifdef __ANDROID__
	shapeBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Shape) * shapeCnt, NULL, &status);
#else
    shapeBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(Shape) * shapeCnt, NULL, &status);
#endif
	clErrchk(status);
	
	clErrchk(clEnqueueWriteBuffer(commandQueue, shapeBuffer, CL_TRUE, 0, sizeof(Shape) * shapeCnt, shapes, 0, NULL, NULL));
	
#if (ACCELSTR == 1)
	/*------------------------------------------------------------------------*/
	btnBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(BVHNodeGPU) * (shapeCnt-1), NULL, &status);
	clErrchk(status);

	btlBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(BVHNodeGPU) * shapeCnt, NULL, &status);
	clErrchk(status);
#elif (ACCELSTR == 2)
	/*------------------------------------------------------------------------*/
	kngBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(KDNodeGPU) * (kngCnt), NULL, &status);
	clErrchk(status);

	knBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(int) * knCnt, NULL, &status);
	clErrchk(status);
#endif
	cameraBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(Camera), NULL, &status);
	clErrchk(status);

	clErrchk(clEnqueueWriteBuffer(commandQueue, cameraBuffer, CL_TRUE, 0, sizeof(Camera), &camera, 0, NULL, NULL));
	
	AllocateBuffers();

	/*------------------------------------------------------------------------*/
	/* Create the kernel program */
	const char *sources = ReadSources(kernelFileName);
	program = clCreateProgramWithSource(context, 1, &sources, NULL, &status);
	clErrchk(status);

	char strInclude[MAX_INCLUDE];
	strcpy(strInclude, "-DGPU_KERNEL -I. ");
	//strcat(strInclude, strResPath);
	//strcat(strInclude, "/include");

	status = clBuildProgram(program, 1, devices, strInclude, NULL, NULL);
	clErrchk(status);

	if (status != CL_SUCCESS) {
		LOGE("Failed to build OpenCL kernel: %d\n", status);

        size_t retValSize;
		clErrchk(clGetProgramBuildInfo(program, devices[0], CL_PROGRAM_BUILD_LOG, 0, NULL, &retValSize));
        
        char *buildLog = (char *)malloc(retValSize + 1);
		clErrchk(clGetProgramBuildInfo(program, devices[0], CL_PROGRAM_BUILD_LOG, retValSize, buildLog, NULL));
		
        buildLog[retValSize] = '\0';		
#ifdef PTX_ERROR
		// Query binary (PTX file) size
		size_t bin_sz;
		status = clGetProgramInfo(program, CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &bin_sz, NULL);

		// Read binary (PTX file) to memory bufferf
		unsigned char *bin = (unsigned char *)malloc(bin_sz);
		status = clGetProgramInfo(program, CL_PROGRAM_BINARIES, sizeof(unsigned char *), &bin, NULL);

		char ptxFileName[255];
		strcpy(ptxFileName, kernelFileName);
		strcat(ptxFileName, ".ptx");

		// Save PTX to add_vectors_ocl.ptx
		FILE *fp = fopen(ptxFileName, "wb");
		fwrite(bin, sizeof(char), bin_sz, fp);
		fclose(fp);

		free(bin);
#endif
		LOGE("OpenCL Programm Build Log: %s\n", buildLog);

		char strError[MAX_ERROR];
		//strcpy(strError, strResPath);
		strcat(strError, "error_renderingkernel.txt");

		FILE *fp = fopen(strError, "wt");
		fwrite(buildLog, sizeof(char), retValSize + 1, fp);
		fclose(fp);

		free(buildLog);
		return ;
    }
#ifdef EXP_KERNEL
	kernelGen = clCreateKernel(program, "GenerateCameraRay_exp", &status);
	clErrchk(status);

	kernelRadiance = clCreateKernel(program, "RadiancePathTracing_exp", &status);
	clErrchk(status);

	kernelFill = clCreateKernel(program, "FillPixel_exp", &status);
	clErrchk(status);
#else
	kernel = clCreateKernel(program, "RadianceGPU", &status);
	clErrchk(status);
#endif

#ifdef CPU_PARTRENDERING
	kernelBox = clCreateKernel(program, "RadianceBoxGPU", &status);
	clErrchk(status);
#endif
#if (ACCELSTR == 1)
	/* Create the kernel program */
	const char *sourcesBvh = ReadSources(bvhFileName);
	program = clCreateProgramWithSource(context, 1, &sourcesBvh, NULL, &status);
	clErrchk(status);

    //char strInclude[MAX_INCLUDE];
    strcpy(strInclude, "-DGPU_KERNEL -I. ");
    //strcat(strInclude, strResPath);
    //strcat(strInclude, "/include");

	status = clBuildProgram(program, 1, devices, strInclude, NULL, NULL);
	clErrchk(status);

	if (status != CL_SUCCESS) {
		LOGE("Failed to build OpenCL kernel (BVH): %d\n", status);

		size_t retValSize;
		clErrchk(clGetProgramBuildInfo(program, devices[0], CL_PROGRAM_BUILD_LOG, 0, NULL, &retValSize));
		
		char *buildLog = (char *)malloc(retValSize + 1);
		clErrchk(clGetProgramBuildInfo(program, devices[0], CL_PROGRAM_BUILD_LOG, retValSize, buildLog, NULL));
		
		buildLog[retValSize] = '\0';
#if 0
		// Query binary (PTX file) size
		size_t bin_sz;
		clErrchk(clGetProgramInfo(program, CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &bin_sz, NULL));

		// Read binary (PTX file) to memory bufferf
		unsigned char *bin = (unsigned char *)malloc(bin_sz);
		clErrchk(clGetProgramInfo(program, CL_PROGRAM_BINARIES, sizeof(unsigned char *), &bin, NULL));

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

		char strError[MAX_ERROR];
		//strcpy(strError, strResPath);
		strcat(strError, "error_BVH.txt");

		FILE *fp = fopen(strError, "wt");
		fwrite(buildLog, sizeof(char), retValSize + 1, fp);
		fclose(fp);

		free(buildLog);
		return;
	}

	kernelRad = clCreateKernel(program, "kernelConstructRadixTree", &status);
	clErrchk(status);

	kernelBvh = clCreateKernel(program, "kernelConstructBVHTree", &status);
	clErrchk(status);

	kernelOpt = clCreateKernel(program, "kernelOptimize", &status);
	clErrchk(status);
#endif

	// LordCRC's patch for better workGroupSize
	size_t gsize = 0;
#ifdef EXP_KERNEL
	clErrchk(clGetKernelWorkGroupInfo(kernelGen, devices[0], CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &gsize, NULL));
#else
	clErrchk(clGetKernelWorkGroupInfo(kernel, devices[0], CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &gsize, NULL));
#endif

	workGroupSize = (unsigned int) gsize;
	LOGI("OpenCL Device 0: kernel work group size = %d\n", workGroupSize);

	if (forceWorkSize > 0) {
		LOGI("OpenCL Device 0: forced kernel work group size = %d\n", forceWorkSize);
		workGroupSize = forceWorkSize;
	}
}

#ifdef CPU_PARTRENDERING
void ExecuteBoxKernel(int x, int y, int bwidth, int bheight) {
	/* Enqueue a kernel run call */
	size_t globalThreads[1];

	globalThreads[0] = bwidth * bheight;

	if (globalThreads[0] % workGroupSize != 0)
		globalThreads[0] = (globalThreads[0] / workGroupSize + 1) * workGroupSize;

	size_t localThreads[1];

	localThreads[0] = workGroupSize;

	clErrchk(clEnqueueNDRangeKernel(commandQueue, kernelBox, 1, NULL, globalThreads, localThreads, 0, NULL, NULL));
}

void DrawBox(int xstart, int ystart, int bwidth, int bheight, int twidth, int theight, double &cpuTotalTime, double &rwTotalTime) {
	const float invWidth = 1.f / twidth;
	const float invHeight = 1.f / theight;

	double rwStartTime = WallClockTime();

	clErrchk(clEnqueueReadBuffer(commandQueue, colorBuffer, CL_TRUE, 0, sizeof(Vec) * twidth * theight, colors, 0, NULL, NULL));
	clErrchk(clEnqueueReadBuffer(commandQueue, pixelBuffer, CL_TRUE, 0, sizeof(unsigned int) * twidth * theight, pixels, 0, NULL, NULL));
	
	rwTotalTime += (WallClockTime() - rwStartTime);

	double cpuStartTime = WallClockTime();

#pragma omp parallel for
	for (int y = ystart; y < ystart + bheight; y++) { /* Loop over image rows */
		for (int x = xstart; x < xstart + bwidth; x++) { /* Loop cols */
			const int i = (theight - y - 1) * twidth + x;
			const int i2 = i << 1;

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
			const Ray ray = { rorig, rdir };

			Vec r;
			r.x = r.y = r.z = 1.0f;

			RadiancePathTracing(shapes, shapeCnt, lightCnt, 
#if (ACCELSTR == 1)
				btn, btl,
#elif (ACCELSTR == 2)
				pkngbuf, kngCnt, pknbuf, knCnt,
#endif
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

			pixels[y * twidth + x] = toInt(colors[i].x) |
				(toInt(colors[i].y) << 8) |
				(toInt(colors[i].z) << 16);
		}
	}
	cpuTotalTime += (WallClockTime() - cpuStartTime);

	rwStartTime = WallClockTime();

	clErrchk(clEnqueueWriteBuffer(commandQueue, pixelBuffer, CL_TRUE, 0, sizeof(unsigned int) * twidth * theight, pixels, 0, NULL, NULL));
	clErrchk(clEnqueueWriteBuffer(commandQueue, colorBuffer, CL_TRUE, 0, sizeof(Vec) * twidth * theight, colors, 0, NULL, NULL));
	
	rwTotalTime += (WallClockTime() - rwStartTime);
}

unsigned int *DrawAllBoxes(int bwidth, int bheight, float *rCPU, bool bFirst) {
	int startSampleCount = currentSample, nGPU = 0, nCPU = 1, index = 0;
	bool cpuTurn = false;
	double startTime = WallClockTime(), setStartTime, kernelStartTime;
	double setTotalTime = 0.0, kernelTotalTime = 0.0, rwTotalTime = 0.0;
	double cpuTotalTime = 0.0;

	cl_int status;

	for (int y = 0; y < height; y += bheight) {
		for (int x = 0; x < width; x += bwidth) {
			if (cpuTurn) {
				DrawBox(x, y, bwidth, bheight, width, height, cpuTotalTime, rwTotalTime);

				nCPU++;

				if ((float)nGPU / nCPU >= *rCPU) cpuTurn = true;
				else cpuTurn = false;
			}
			else
			{
				index = 0;
				setStartTime = WallClockTime();

				/* Set kernel arguments */
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&colorBuffer));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&seedBuffer));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&shapeBuffer));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&cameraBuffer));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(unsigned int), (void *)&shapeCnt));				
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(unsigned int), (void *)&lightCnt));
#if (ACCELSTR == 1)
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&btnBuffer));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&btlBuffer));
#elif (ACCELSTR == 2)
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&kngBuffer));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(int), (void *)&kngCnt));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&knBuffer));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(int), (void *)&knCnt));
#endif
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(int), (void *)&x));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(int), (void *)&y));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(int), (void *)&bwidth));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(int), (void *)&bheight));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(int), (void *)&width));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(int), (void *)&height));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(int), (void *)&currentSample));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&pixelBuffer));
#ifdef DEBUG_INTERSECTIONS
				int *debug1 = (int *)malloc(sizeof(int) * shapeCnt);
				memset(debug1, 0, sizeof(int) * shapeCnt);

				cl_mem debugBuffer1 = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int) * shapeCnt, NULL, &status);
				clErrchk(status);

				clErrchk(clEnqueueWriteBuffer(commandQueue, debugBuffer1, CL_TRUE, 0, sizeof(int) * shapeCnt, debug1, 0, NULL, NULL));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&debugBuffer1));

				float *debug2 = (float *)malloc(6 * sizeof(float) * shapeCnt);
				memset(debug2, 0, sizeof(float) * shapeCnt);

				cl_mem debugBuffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, 6 * sizeof(float) * shapeCnt, NULL, &status);
				clErrchk(status);

				clErrchk(clEnqueueWriteBuffer(commandQueue, debugBuffer2, CL_TRUE, 0, 6 * sizeof(float) * shapeCnt, debug2, 0, NULL, NULL));
				clErrchk(clSetKernelArg(kernelBox, index++, sizeof(cl_mem), (void *)&debugBuffer2));
#endif
				setTotalTime += (WallClockTime() - setStartTime);

				//--------------------------------------------------------------------------
#ifdef CURRENT_SAMPLE
				if (currentSample < 20) {
#endif
					kernelStartTime = WallClockTime();
					ExecuteBoxKernel(x, y, bwidth, bheight);
					//clFinish(commandQueue);
					kernelTotalTime += (WallClockTime() - kernelStartTime);
#ifdef CURRENT_SAMPLE
				}
				else {
					/* After first 20 samples, continue to execute kernels for more and more time */
					const float k = min(currentSample - 20, 100) / 100.f;
					const float tresholdTime = 0.5f * k * 1000.0f;
					for (;;) {
						kernelStartTime = WallClockTime();
						ExecuteBoxKernel();
						//clFinish(commandQueue);
						kernelTotalTime += (WallClockTime() - kernelStartTime);

						currentSample++;
						const float elapsedTime = WallClockTime() - startTime;
						if (elapsedTime > tresholdTime)
							break;
					}
				}
#endif
				//readStartTime = WallClockTime();

				//clFinish(commandQueue);
				//readTotalTime += (WallClockTime() - readStartTime);
#ifdef DEBUG_INTERSECTIONS
				status = clEnqueueReadBuffer(commandQueue, debugBuffer1, CL_TRUE, 0, sizeof(int) * shapeCnt, debug1, 0, NULL, NULL);
				clErrchk(status);

				clErrchk(clEnqueueReadBuffer(commandQueue, debugBuffer2, CL_TRUE, 0, 6 * sizeof(float) * shapeCnt, debug2, 0, NULL, NULL));

				FILE *f = fopen("images\\intersections.txt", "wt"); // Write image to PPM file.
				for (int i = 0; i < shapeCnt; i++) {
					fprintf(f, "%d, ", debug1[i]);
				}
				fprintf(f, "\n");
				for (int i = 0; i < 6 * shapeCnt; i++) {
					fprintf(f, "%f, ", debug2[i]);
				}
				fclose(f);

				clErrchk(clReleaseMemObject(debugBuffer2));
				free(debug2);

				clErrchk(clReleaseMemObject(debugBuffer1));
				free(debug1);
#endif				
				nGPU++;

				if ((float)nGPU / nCPU >= *rCPU) cpuTurn = true;
				else cpuTurn = false;
			}
		}
	}

	double rwStartTime = WallClockTime();
	clErrchk(clEnqueueReadBuffer(commandQueue, pixelBuffer, CL_TRUE, 0, sizeof(unsigned int) * width * height, pixels, 0, NULL, NULL));
	rwTotalTime += (WallClockTime() - rwStartTime);

	//if (bFirst) 
	*rCPU = (cpuTotalTime / (nCPU - 1)) / ((setTotalTime + kernelTotalTime + rwTotalTime) / nGPU);

	currentSample++;

	/*------------------------------------------------------------------------*/
	const double elapsedTime = WallClockTime() - startTime;
	const int samples = currentSample - startSampleCount;
	const double sampleSec = samples * height * width / elapsedTime;

	LOGI("Set time %.5f msec, Kernel time %.5f msec, CPU time %.5f msec, RW time %.5f msec, Total time %.5f msec (pass %d)  Sample/sec  %.1fK\n",
		setTotalTime, kernelTotalTime, cpuTotalTime, rwTotalTime, elapsedTime, currentSample, sampleSec / 1000.f);

	return pixels;
}

unsigned int *DrawFrame()
{
    static float rCPU = 1.0f;
    static bool first = true;

    unsigned int *pPixels = DrawAllBoxes(160, 120, &rCPU, first);
    first = false;

    return pPixels;
}
#else
#ifdef EXP_KERNEL
void ExecuteKernel(cl_kernel p_kernel) {
	/* Enqueue a kernel run call */
	size_t globalThreads[1];

	globalThreads[0] = width * height;

	if (globalThreads[0] % workGroupSize != 0) globalThreads[0] = (globalThreads[0] / workGroupSize + 1) * workGroupSize;

	size_t localThreads[1];

	localThreads[0] = workGroupSize;

	clErrchk(clEnqueueNDRangeKernel(commandQueue, p_kernel, 1, NULL, globalThreads, localThreads, 0, NULL, NULL));
}
#else
void ExecuteKernel() {
	/* Enqueue a kernel run call */
	size_t globalThreads[1];

	globalThreads[0] = width * height;

	if (globalThreads[0] % workGroupSize != 0) globalThreads[0] = (globalThreads[0] / workGroupSize + 1) * workGroupSize;

	size_t localThreads[1];

	localThreads[0] = workGroupSize;
	
	clErrchk(clEnqueueNDRangeKernel(commandQueue, kernel, 1, NULL, globalThreads, localThreads, 0, NULL, NULL));
}
#endif

unsigned int *DrawFrame() {
	int len = pixelCount * sizeof(unsigned int), index = 0;
	double startTime = WallClockTime(), setStartTime, kernelStartTime, readStartTime;
	double setTotalTime = 0.0, kernelTotalTime = 0.0, readTotalTime = 0.0;
	int startSampleCount = currentSample;

#ifdef EXP_KERNEL
	for (int j = 0; j < MAX_SPP; j++)
	{
		index = 0;

		/* Set kernel arguments */
		setStartTime = WallClockTime();
		clErrchk(clSetKernelArg(kernelGen, index++, sizeof(cl_mem), (void *)&cameraBuffer));
        clErrchk(clSetKernelArg(kernelGen, index++, sizeof(cl_mem), (void *)&seedBuffer));
		clErrchk(clSetKernelArg(kernelGen, index++, sizeof(int), (void *)&width));
		clErrchk(clSetKernelArg(kernelGen, index++, sizeof(int), (void *)&height));
        clErrchk(clSetKernelArg(kernelGen, index++, sizeof(cl_mem), (void *)&rayBuffer));
		clErrchk(clSetKernelArg(kernelGen, index++, sizeof(cl_mem), (void *)&throughputBuffer));
		clErrchk(clSetKernelArg(kernelGen, index++, sizeof(cl_mem), (void *)&specularBounceBuffer));
		clErrchk(clSetKernelArg(kernelGen, index++, sizeof(cl_mem), (void *)&terminatedBuffer));
		clErrchk(clSetKernelArg(kernelGen, index++, sizeof(cl_mem), (void *)&resultBuffer));
		setTotalTime += (WallClockTime() - setStartTime);

		kernelStartTime = WallClockTime();
		ExecuteKernel(kernelGen);
		clFinish(commandQueue);
		kernelTotalTime += (WallClockTime() - kernelStartTime);

		for (int i = 0; i < MAX_DEPTH; i++)
		{
			index = 0;

			setStartTime = WallClockTime();
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&shapeBuffer));
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(unsigned int), (void *)&shapeCnt));
            clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(unsigned int), (void *)&lightCnt));
            clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(int), (void *)&width));
            clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(int), (void *)&height));
#if (ACCELSTR == 1)
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&btnBuffer));
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&btlBuffer));
#elif (ACCELSTR == 2)
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&kngBuffer));
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(int), (void *)&kngCnt));
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&knBuffer));
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(int), (void *)&knCnt));
#endif
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&rayBuffer));
            clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&seedBuffer));
            clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&throughputBuffer));
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&specularBounceBuffer));
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&terminatedBuffer));
			clErrchk(clSetKernelArg(kernelRadiance, index++, sizeof(cl_mem), (void *)&resultBuffer));
			setTotalTime += (WallClockTime() - setStartTime);

			kernelStartTime = WallClockTime();
			ExecuteKernel(kernelRadiance);
			//clFinish(commandQueue);
			kernelTotalTime += (WallClockTime() - kernelStartTime);
		}

		index = 0;

		setStartTime = WallClockTime();

		clErrchk(clSetKernelArg(kernelFill, index++, sizeof(int), (void *)&width));
		clErrchk(clSetKernelArg(kernelFill, index++, sizeof(int), (void *)&height));
		clErrchk(clSetKernelArg(kernelFill, index++, sizeof(int), (void *)&currentSample));
        clErrchk(clSetKernelArg(kernelFill, index++, sizeof(cl_mem), (void *)&colorBuffer));
        clErrchk(clSetKernelArg(kernelFill, index++, sizeof(cl_mem), (void *)&resultBuffer));
		clErrchk(clSetKernelArg(kernelFill, index++, sizeof(cl_mem), (void *)&pixelBuffer));
		setTotalTime += (WallClockTime() - setStartTime);

		kernelStartTime = WallClockTime();
		ExecuteKernel(kernelFill);
		clFinish(commandQueue);
		kernelTotalTime += (WallClockTime() - kernelStartTime);
	}
	readStartTime = WallClockTime();
	//--------------------------------------------------------------------------
	/* Enqueue readBuffer */
	clErrchk(clEnqueueReadBuffer(commandQueue, pixelBuffer, CL_TRUE, 0, len, pixels, 0, NULL, NULL));
	//clFinish(commandQueue);
	readTotalTime += (WallClockTime() - readStartTime);
#else
	setStartTime = WallClockTime();

	/* Set kernel arguments */
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *)&colorBuffer));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *)&seedBuffer));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *) &cameraBuffer));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *)&shapeBuffer));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(unsigned int), (void *) &shapeCnt));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(unsigned int), (void *)&lightCnt));
#if (ACCELSTR == 1)	
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *)&btnBuffer));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *)&btlBuffer));
#elif (ACCELSTR == 2)
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *)&kngBuffer));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(int), (void *)&kngCnt));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *)&knBuffer));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(int), (void *)&knCnt));
#endif
	clErrchk(clSetKernelArg(kernel, index++, sizeof(int), (void *) &width));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(int), (void *) &height));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(int), (void *)&currentSample));
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *) &pixelBuffer));
#ifdef DEBUG_INTERSECTIONS
    cl_int status;
    int *debug1 = (int *)malloc(sizeof(int) * shapeCnt);
    memset(debug1, 0, sizeof(int) * shapeCnt);

    cl_mem debugBuffer1 = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int) * shapeCnt, NULL, &status);
	clErrchk(status);

	clErrchk(clEnqueueWriteBuffer(commandQueue, debugBuffer1, CL_TRUE, 0, sizeof(int) * shapeCnt, debug1, 0, NULL, NULL));
	
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *) &debugBuffer1));
    
	float *debug2 = (float *)malloc(6 * sizeof(float) * shapeCnt);
	memset(debug2, 0, sizeof(float) * shapeCnt);

	cl_mem debugBuffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, 6 * sizeof(float) * shapeCnt, NULL, &status);
	clErrchk(status);
	
	clErrchk(clEnqueueWriteBuffer(commandQueue, debugBuffer2, CL_TRUE, 0, 6 * sizeof(float) * shapeCnt, debug2, 0, NULL, NULL));
	
	clErrchk(clSetKernelArg(kernel, index++, sizeof(cl_mem), (void *) &debugBuffer2));	
#endif
	setTotalTime += (WallClockTime() - setStartTime);

	//--------------------------------------------------------------------------
#ifdef CURRENT_SAMPLE
	if (currentSample < 20) {
#endif
	kernelStartTime = WallClockTime();
	ExecuteKernel();
	//clFinish(commandQueue);
	kernelTotalTime += (WallClockTime() - kernelStartTime);

#ifdef CURRENT_SAMPLE
    }
    else {
        /* After first 20 samples, continue to execute kernels for more and more time */
        const float k = min(currentSample - 20, 100) / 100.f;
        const float tresholdTime = 0.5f * k * 1000.0f;
        for (;;) {
            kernelStartTime = WallClockTime();
            ExecuteKernel();
            //clFinish(commandQueue);
            kernelTotalTime += (WallClockTime() - kernelStartTime);

            currentSample++;
            const float elapsedTime = WallClockTime() - startTime;
            if (elapsedTime > tresholdTime)
                break;
        }
    }
#endif
    readStartTime = WallClockTime();
    //--------------------------------------------------------------------------
    /* Enqueue readBuffer */
	clErrchk(clEnqueueReadBuffer(commandQueue, pixelBuffer, CL_TRUE, 0, len, pixels, 0, NULL, NULL));
	//clFinish(commandQueue);
	readTotalTime += (WallClockTime() - readStartTime);
#ifdef DEBUG_INTERSECTIONS
	clErrchk(clEnqueueReadBuffer(commandQueue, debugBuffer1, CL_TRUE, 0, sizeof(int) * shapeCnt, debug1, 0, NULL, NULL));
	clErrchk(clEnqueueReadBuffer(commandQueue, debugBuffer2, CL_TRUE, 0, 6 * sizeof(float) * shapeCnt, debug2, 0, NULL, NULL));

	char strLog[MAX_LOG];
	strcpy(strLog, strResPath);
	strcat(strLog, "/intersections.txt");

    FILE *f = fopen(strLog, "wt"); // Write image to PPM file.
	for(int i = 0; i < shapeCnt; i++) {
		fprintf(f, "%d, ", debug1[i]);
	}
	fprintf(f, "\n");
	for(int i = 0; i < 6 * shapeCnt; i++) {
		fprintf(f, "%f, ", debug2[i]);
	}
	fclose(f);

	clErrchk(clReleaseMemObject(debugBuffer2));
    free(debug2);

	clErrchk(clReleaseMemObject(debugBuffer1));
	free(debug1);
#endif    
#endif
	currentSample++;

	/*------------------------------------------------------------------------*/
	const double elapsedTime = WallClockTime() - startTime;
	const int samples = currentSample - startSampleCount;
	const double sampleSec = samples * height * width / elapsedTime;
	LOGI("Set time %.5f msec, Kernel time %.5f msec, Read time %.5f msec, Total time %.5f msec (pass %d)  Sample/sec  %.1fK\n",
		setTotalTime, kernelTotalTime, readTotalTime, elapsedTime, currentSample, sampleSec / 1000.f);

	return pixels;
}
#endif

void ReInitScene() {
    currentSample = 0;

    // Redownload the scene
	clErrchk(clEnqueueWriteBuffer(commandQueue, shapeBuffer, CL_TRUE, 0, sizeof(Shape) * shapeCnt, shapes, 0, NULL, NULL));
}

void ReInit(const int reallocBuffers) {
    // Check if I have to reallocate buffers
    if (reallocBuffers) {
        FreeBuffers();
        UpdateCamera();
        AllocateBuffers();
    } else {
        UpdateCamera();
	}

	clErrchk(clEnqueueWriteBuffer(commandQueue, cameraBuffer, CL_TRUE, 0, sizeof(Camera), &camera, 0, NULL, NULL));
#if (ACCELSTR == 1)
	clErrchk(clEnqueueWriteBuffer(commandQueue, btnBuffer, CL_TRUE, 0, sizeof(BVHNodeGPU) * (shapeCnt - 1), btn, 0, NULL, NULL));
	clErrchk(clEnqueueWriteBuffer(commandQueue, btlBuffer, CL_TRUE, 0, sizeof(BVHNodeGPU) * (shapeCnt), btl, 0, NULL, NULL));
#elif (ACCELSTR == 2)
	clErrchk(clEnqueueWriteBuffer(commandQueue, kngBuffer, CL_TRUE, 0, sizeof(KDNodeGPU) * (kngCnt), pkngbuf, 0, NULL, NULL));
	clErrchk(clEnqueueWriteBuffer(commandQueue, knBuffer, CL_TRUE, 0, sizeof(int) * (knCnt), pknbuf, 0, NULL, NULL));
#endif

    currentSample = 0;
}

#ifdef WIN32
static int mouseX = 0, mouseY = 0;
static int mouseButton = 0;

#define TWO_PI 6.28318530717958647693f
#define PI_OVER_TWO 1.57079632679489661923f

#define MOVE_STEP 10.0f
#define ROTATE_STEP (1.f * M_PI / 180.f)

void idleFunc(void) {
	glutPostRedisplay();
}

void displayFunc(void) {
	glClear(GL_COLOR_BUFFER_BIT);
	glRasterPos2i(0, 0);
#ifdef CPU_PARTRENDERING
	static float rCPU = 1.0f;
	static bool first = true;

	DrawAllBoxes(160, 120, &rCPU, first);
	first = false;
#else
	DrawFrame();
#endif
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

/// gets the current mouse position and compares to the last one to check if
/// we need to change the pitch/yaw of the camera
/// it only changes the camera if the mouse button is pressed
void motionFunc(int x, int y) {
	int deltaX = mouseX - x;
	int deltaY = mouseY - y;

	if (deltaX != 0 || deltaY != 0) {
		// rotate the camera using pitch (nodding movement) and yaw (nonono movement)
		if (mouseButton == GLUT_LEFT_BUTTON) {
			camera.yaw += deltaX * 0.01;
			camera.yaw = camera.yaw - TWO_PI * floor(camera.yaw / TWO_PI);
			camera.pitch += -deltaY * 0.01;
			camera.pitch = clamp(camera.pitch, -PI_OVER_TWO, PI_OVER_TWO);
		}

		glutSetCursor(GLUT_CURSOR_CROSSHAIR);

		mouseX = x;
		mouseY = y;
		ReInit(0);
	}
	else {
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}

/// simply records the mouse state
void mouseFunc(int button, int state, int x, int y) {
	mouseButton = button;
	mouseX = x;
	mouseY = y;

	motionFunc(x, y);
}

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
		camera.pitch += -0.01;
		camera.pitch = clamp(camera.pitch, -PI_OVER_TWO, PI_OVER_TWO);
		ReInit(0);
		break;
	}
	case GLUT_KEY_DOWN: {
		camera.pitch += 0.01;
		camera.pitch = clamp(camera.pitch, -PI_OVER_TWO, PI_OVER_TWO);
		ReInit(0);
		break;
	}
	case GLUT_KEY_LEFT: {
		camera.yaw += 0.01;
		camera.yaw = camera.yaw - TWO_PI * floor(camera.yaw / TWO_PI);
		ReInit(0);
		break;
	}
	case GLUT_KEY_RIGHT: {
		camera.yaw += -0.01;
		camera.yaw = camera.yaw - TWO_PI * floor(camera.yaw / TWO_PI);
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
	glutMouseFunc(mouseFunc);
	glutMotionFunc(motionFunc);
	glutKeyboardFunc(keyFunc);
	glutSpecialFunc(specialFunc);
	glutDisplayFunc(displayFunc);
	glutIdleFunc(idleFunc);

	glViewport(0, 0, width, height);
	glLoadIdentity();
	glOrtho(0.f, width - 1.f, 0.f, height - 1.f, -1.f, 1.f);
}

int main(int argc, char *argv[]) {
    amiSmallptCPU = 0;

    if (argc == 7) {
		bool walllight = true;
		srand(time(NULL));

        useGPU = atoi(argv[1]);
        forceWorkSize = atoi(argv[2]);
		//strcpy(forceWorkSize, argv[2]);
#ifndef EXP_KERNEL
		strcpy(kernelFileName, argv[3]);
#endif        
        width = atoi(argv[4]);
        height = atoi(argv[5]);
		
		Read(argv[6], &walllight);
		if (walllight) AddWallLight();

#if (ACCELSTR == 0)
		SetUpOpenCL();
#elif (ACCELSTR == 1)
		SetUpOpenCL();
		BuildBVH();
#elif (ACCELSTR == 2)
		BuildKDtree();
		SetUpOpenCL();
#endif
		UpdateCamera();		
    } else if (argc == 1) {
		srand(time(NULL));

		shapeCnt = sizeof(CornellSpheres) / sizeof(Sphere);
		shapes = (Shape *)malloc(sizeof(Shape) * shapeCnt);

		for(int i = 0; i < shapeCnt; i++)
		{
			shapes[i].type = SPHERE;
			shapes[i].s = CornellSpheres[i];
			shapes[i].e = t[i].e;
			shapes[i].c = t[i].c;
			shapes[i].refl = t[i].refl;
		}

        vinit(camera.orig, 50.f, 45.f, 205.6f);
        vinit(camera.target, 50.f, 45 - 0.042612f, 204.6);
	}
	else
	{
		LOGE("Usage: %s\n", argv[0]);
		LOGE("Usage: %s <use CPU/GPU device (0=CPU or 1=GPU)> <workgroup size (0=default value or anything > 0 and power of 2)> <kernel file name> <window width> <window height> <scene file>\n", argv[0]);
	
		exit(-1);
	}
	
    /*------------------------------------------------------------------------*/
	InitGlut(argc, argv, (char *)"SmallPTGPU (Added the BVH and KDTree for the intersection tests)");
	
	LOGI("Acceleration for intersection test: ");
#if (ACCELSTR == 0)
	LOGI("No Accel\n");
#elif (ACCELSTR == 1)
	LOGI("BVH\n");
#elif (ACCELSTR == 2)
	LOGI("KDTree\n");
#endif	

	glutMainLoop();

    return 0;
}
#endif

void AddWallLight()
{
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt].s = { WALL_RAD,{ WALL_RAD + 25.0f, 0.0f, 0.0f } };  shapes[shapeCnt].e = { 0.f, 0.f, 0.f }; shapes[shapeCnt].c = { .75f, .25f, .25f }; shapes[shapeCnt++].refl = DIFF; /* Left */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt].s = { WALL_RAD,{ -WALL_RAD - 25.0f, 0.0f, 0.0f } };  shapes[shapeCnt].e = { 0.f, 0.f, 0.f }; shapes[shapeCnt].c = { .25f, .25f, .75f }; shapes[shapeCnt++].refl = DIFF; /* Rght */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt].s = { WALL_RAD,{ 0.0f, 0.0f, WALL_RAD - 25.0f } };  shapes[shapeCnt].e = { 0.f, 0.f, 0.f }; shapes[shapeCnt].c = { .75f, .75f, .75f }; shapes[shapeCnt++].refl = DIFF; /* Back */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt].s = { WALL_RAD,{ 0.0f, 0.0f, -WALL_RAD + 100.0f } };  shapes[shapeCnt].e = { 0.f, 0.f, 0.f }; shapes[shapeCnt].c = { 0.f, 0.f, 0.f }; shapes[shapeCnt++].refl = DIFF; /* Frnt */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt].s = { WALL_RAD,{ 0.0f, WALL_RAD + 25.0f, 0.0f } };  shapes[shapeCnt].e = { 0.f, 0.f, 0.f }; shapes[shapeCnt].c = { .75f, .75f, .75f }; shapes[shapeCnt++].refl = DIFF; /* Botm */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt].s = { WALL_RAD,{ 0.0f, -WALL_RAD - 25.0f, 0.0f } };  shapes[shapeCnt].e = { 0.f, 0.f, 0.f }; shapes[shapeCnt].c = { .75f, .75f, .75f }; shapes[shapeCnt++].refl = DIFF; /* Top */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt].s = { 5.0f,{ 10.0f, -17.0f, 0.0f } };  shapes[shapeCnt].e = { 0.f, 0.f, 0.f }; shapes[shapeCnt].c = { .9f, .9f, .9f }; shapes[shapeCnt++].refl = SPEC; /* Mirr */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt].s = { 5.0f,{ -10.0f, -17.0f, 0.0f } };  shapes[shapeCnt].e = { 0.f, 0.f, 0.f }; shapes[shapeCnt].c = { .9f, .9f, .9f }; shapes[shapeCnt++].refl = REFR; /* Glas */
	shapes[shapeCnt].type = SPHERE; shapes[shapeCnt].s = { 2.f,{ 10.0f, 15.0f, 0.0f } };  shapes[shapeCnt].e = { 12.f, 12.f, 12.f }; shapes[shapeCnt].c = { 0.f, 0.f, 0.f }; shapes[shapeCnt++].refl = DIFF; /* Lite */
}

#if (ACCELSTR == 1)
void BuildBVH()
{
	CLBVH *pCB = new CLBVH(shapes, shapeCnt, commandQueue, context, kernelRad, kernelBvh, kernelOpt);

	pCB->buildRadixTree();
	pCB->buildBVHTree();
	//pCB->optimize();

	pCB->getTrees(&btn, &btl);
	/*
    pCB->makeNaiveBVHTree();
    pCB->getTree(&nbtn, &nbtnCnt);
    */
}
#elif (ACCELSTR == 2)
void BuildKDtree()
{
	std::vector<Shape *> vs;

	for (int i = 0; i < shapeCnt; i++) {
		shapes[i].index = i;
		shapes[i].morton_code = 0;

		vs.push_back(&shapes[i]);
	}

	KDTree *kdTree = new KDTree();
	KDTreeNode *rootNode = kdTree->build(vs, 0);
	//kdTree->printNode(rootNode, 0);

	kdTree->getTrees(rootNode, &pkngbuf, &kngCnt, &pknbuf, &knCnt);
}
#endif
