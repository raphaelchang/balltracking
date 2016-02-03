#ifndef RANSAC_H_
#define RANSAC_H_

#include <OpenNI.h>

using namespace std;

class RANSAC
{
public:
	RANSAC(int iterations, float inlierDist);

	void MarkPointsInBestFitPlane(float **pointCloud, int numPts, bool *result, float *normalX, float *normalY, float *normalZ, float *d, float *intersect);

private:
	int m_iterations;
	float m_inlierDist;
};

#endif
