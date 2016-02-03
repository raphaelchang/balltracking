/*
 * BlobExtractor.h
 *
 *  Created on: Jan 2, 2014
 *      Author: raphael
 */

#ifndef BLOBEXTRACTOR_H_
#define BLOBEXTRACTOR_H_

#include "DisjointSet.h"

class BlobExtractor {
public:

	BlobExtractor(int width, int height, double minArea);
	~BlobExtractor();

	int ExtractBlobs(bool **binaryImage, int **result);

private:
	double m_minArea;
	int width, height;
	DisjointSet ds;
	node **labels;
};

#endif /* BLOBEXTRACTOR_H_ */
