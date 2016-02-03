/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#ifndef _ONI_SAMPLE_VIEWER_H_
#define _ONI_SAMPLE_VIEWER_H_

#include <OpenNI.h>
#include "RANSAC.h"
#include "BlobExtractor.h"

class Viewer
{
public:
	Viewer(const char* strSampleName, openni::VideoStream& depth, openni::VideoStream& color);
	virtual ~Viewer();

	virtual openni::Status init(int argc, char **argv);
	virtual openni::Status run();	//Does not return

protected:
	virtual void display();
	virtual void displayPostDraw(){};	// Overload to draw over the screen image
	virtual void handleResize(int w, int h);
	virtual void onKey(unsigned char key, int x, int y);

	virtual openni::Status initOpenGL(int argc, char **argv);
	void initOpenGLHooks();

private:
	Viewer(const Viewer&);
	Viewer& operator=(Viewer&);

	static Viewer* ms_self;
	static void glutIdle();
	static void glutDisplay();
	static void glutKeyboard(unsigned char key, int x, int y);
	static void glutResize(int w, int h);
	static void glutSpecialKeys(int key, int x, int y);

	void pointCloudCenter(vector<float*> &cloud, vector<openni::RGB888Pixel> &rgb, float *x, float *y, float *z);
	float raw_depth_to_meters(int depth_value);
	void depthToWorld(float cgx, float cgy, float cgz, float *x, float *y, float *z);

	char			m_strSampleName[ONI_MAX_STR];
	int					m_width;
	int					m_height;
	const openni::RGB888Pixel* m_colorData;
	bool*				m_inPlane;
	float**				m_pointCloud;
	int*				m_pixelToCloud;
	bool**				m_binaryImage;
	int**				m_blobs;
	RANSAC*				m_ransac;
	BlobExtractor*		m_blobExtractor;

	openni::VideoStream&		m_depth;
	openni::VideoStream&		m_color;
	openni::VideoStream*		m_stream[2];

	openni::VideoFrameRef	m_depthFrame;
	openni::VideoFrameRef	m_colorFrame;

	float angle;
	float lx, lz;
	float cx, cz;

	bool first;
};


#endif // _ONI_SAMPLE_VIEWER_H_
