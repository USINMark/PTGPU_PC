#ifndef ACCELCOMMON_H
#define ACCELCOMMON_H

inline int min2(int a, int b)
{
	return (a < b) ? a : b;
}

inline int max2(int a, int b)
{
	return (a > b) ? a : b;
}

inline float min2(float a, float b)
{
	return (a < b) ? a : b;
}

inline float max2(float a, float b)
{
	return (a > b) ? a : b;
}

inline float min3(float a, float b, float c)
{
	return min2(min2(a, b), c);
}

inline float max3(float a, float b, float c)
{
	return max2(max2(a, b), c);
}
#endif