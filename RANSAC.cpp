#include "RANSAC.h"
#include "math.h"

RANSAC::RANSAC(int iterations, float inlierDist)
{
	m_iterations = iterations;
	m_inlierDist = inlierDist;
}

void RANSAC::MarkPointsInBestFitPlane(float **pointCloud, int numPts, bool *result, float *normalX, float *normalY, float *normalZ, float *d, float *intersect)
{
	if (numPts < 3)
		return;

	int maxInliers = 0, inlier = 0;

	int pos1, pos2, pos3;
	float* p1;
	float* p2;
	float* p3;
	float* p;
	int best[3];
	float A, B, C, D;
	float dis;
	for (int i = 0; i < m_iterations; i++) {
		// Get random points, ensure all points are different and that they have not been used before
		do
		{
			pos1 = rand() % numPts;
			pos2 = rand() % numPts;
			pos3 = rand() % numPts;
		}
		while (pos1 == pos2 || pos2 == pos3 || pos1 == pos3);

		p1 = pointCloud[pos1];
		p2 = pointCloud[pos2];
		p3 = pointCloud[pos3];

		// Calculate A, B, C and D, for the plane spanned by points p1, p2, and p3
		A = p1[1] * (p2[2] - p3[2]) + p2[1] * (p3[2] - p1[2]) + p3[1] * (p1[2] - p2[2]);
		B = p1[2] * (p2[0] - p3[0]) + p2[2] * (p3[0] - p1[0]) + p3[2] * (p1[0] - p2[0]);
		C = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]);
		D = -(p1[0] * (p2[1] * p3[2] - p3[1] * p2[2]) + p2[0] * (p3[1] * p1[2] - p1[1] * p3[2]) + p3[0] * (p1[1] * p2[2] - p2[1] * p1[2]));

		inlier = 0;

		for (int j = 0; j < numPts; j += 16) {
			// Calculate distance between point and plane
			p = pointCloud[j];
			// Calculate the distance between the point and the plane
			dis = abs(A * p[0] + B * p[1] + C * p[2] + D) / sqrt(A * A + B * B + C * C);

			if (dis < m_inlierDist){
				inlier++;
			}
		}
		// If the inlier number is better than the best plane found so far, then we take these points to be considered the new best plane
		if (inlier > maxInliers) {
			maxInliers = inlier;
			best[0] = pos1;
			best[1] = pos2;
			best[2] = pos3;
			if (inlier >= numPts)
				break;
		}
	}

	if (maxInliers == 0)
		return;

	// Use the best points, recreate the plane
	p1 = pointCloud[best[0]];
	p2 = pointCloud[best[1]];
	p3 = pointCloud[best[2]];
	A = p1[1] * (p2[2] - p3[2]) + p2[1] * (p3[2] - p1[2]) + p3[1] * (p1[2] - p2[2]);
	B = p1[2] * (p2[0] - p3[0]) + p2[2] * (p3[0] - p1[0]) + p3[2] * (p1[0] - p2[0]);
	C = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]);
	D = -(p1[0] * (p2[1] * p3[2] - p3[1] * p2[2]) + p2[0] * (p3[1] * p1[2] - p1[1] * p3[2]) + p3[0] * (p1[1] * p2[2] - p2[1] * p1[2]));

	for (int j = 0; j < numPts; j++) {
		// Calculate distance between point and plane
		p = pointCloud[j];
		dis = abs(A * p[0] + B * p[1] + C * p[2] + D) / sqrt(A * A + B * B + C * C);

		if (dis < m_inlierDist){
			result[j] = true;
		}
	}

	*intersect = -D / C;

	*normalX = A / sqrt(A * A + B * B + C * C) * (D < 0 ? 1 : -1);
	*normalY = B / sqrt(A * A + B * B + C * C) * (D < 0 ? 1 : -1);
	*normalZ = C / sqrt(A * A + B * B + C * C) * (D < 0 ? 1 : -1);
	*d = D / sqrt(A * A + B * B + C * C) * (D < 0 ? 1 : -1);
}
