/*
 * BlobExtractor.cpp
 *
 *  Created on: Jan 2, 2014
 *      Author: raphael
 */

#include "BlobExtractor.h"
#include <cstdio>
#include <map>

BlobExtractor::BlobExtractor(int width, int height, double minArea) {
	this->width = width;
	this->height = height;
	m_minArea = minArea;
	labels = new node*[width * height];
}

BlobExtractor::~BlobExtractor() {
	// TODO Auto-generated destructor stub
}

int BlobExtractor::ExtractBlobs(bool **binaryImage, int **result) {
	for (int i = 0; i < width * height; i++)
	{
		labels[i] = 0;
	}
	ds.Reset();

	if (binaryImage[0][0])
		labels[0] = ds.MakeSet(0);
	for (int j = 1; j < width; j++) // First row
	{
		if (binaryImage[0][j])
			labels[j] = binaryImage[0][j] != binaryImage[0][j - 1] ? ds.MakeSet(0) : labels[j - 1];
	}
	for (int i = 1; i < height; i++) {
		for (int j = 0; j < width; j++)
		{
			if (j > 0 && binaryImage[i][j] == binaryImage[i][j - 1]) {
				labels[i * width + j] = labels[i * width + j - 1];
				if (binaryImage[i][j - 1] == binaryImage[i - 1][j] && binaryImage[i][j - 1] && binaryImage[i - 1][j])
					ds.Union(labels[i * width + j - 1], labels[i * width + j - width]);
			}
			else if (binaryImage[i][j] == binaryImage[i - 1][j])
				labels[i * width + j] = labels[i * width + j - width];
			else if (binaryImage[i][j])
				labels[i * width + j] = ds.MakeSet(0);
		}
	}

	int count = ds.Reduce();
	int counts[count];
	for (int i = 0; i < count; i++)
	{
		counts[i] = 0;
	}
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			if (labels[j * width + i] != 0)
				counts[ds.Find(labels[j * width + i])->i]++;
		}
	}
	int filteredCount = 0;
	std::map<int, int> indexToCount;
	for (int i = 0; i < count; i++)
	{
		if (counts[i] >= m_minArea)
		{
			indexToCount[i] = ++filteredCount;
		}
	}

	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (labels[j * width + i] == 0 || indexToCount.find(ds.Find(labels[j * width + i])->i) == indexToCount.end())
			{
				result[j][i] = 0;
				continue;
			}
			int index = indexToCount[ds.Find(labels[j * width + i])->i];
			result[j][i] = index;
		}
	}

	return filteredCount;
}
