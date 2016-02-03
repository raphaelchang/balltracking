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
// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE 
#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "Viewer.h"
#include "Matrix.h"
#include "math.h"

#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#define GL_WIN_SIZE_X	1280
#define GL_WIN_SIZE_Y	1024
#define TEXTURE_SIZE	512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH1

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

Viewer* Viewer::ms_self = NULL;

void Viewer::glutIdle()
{
	glutPostRedisplay();
}
void Viewer::glutDisplay()
{
	Viewer::ms_self->display();
}
void Viewer::glutKeyboard(unsigned char key, int x, int y)
{
	Viewer::ms_self->onKey(key, x, y);
}
void Viewer::glutSpecialKeys(int key, int x, int y)
{
	Viewer::ms_self->onKey(key, x, y);
}
void Viewer::glutResize(int w, int h)
{
	Viewer::ms_self->handleResize(w, h);
}

Viewer::Viewer(const char* strSampleName, openni::VideoStream& depth, openni::VideoStream& color) :
	m_depth(depth), m_color(color)
{
	ms_self = this;
	strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);

	openni::VideoMode videoMode = m_depth.getVideoMode();
	m_width = videoMode.getResolutionX();
	m_height = videoMode.getResolutionY();

	m_stream[0] = &depth;
	m_stream[1] = &color;
	m_inPlane = new bool[m_width * m_height];
	m_pointCloud = new float*[m_width * m_height];
	for (int i = 0; i < m_width * m_height; i++)
		m_pointCloud[i] = new float[3];
	m_binaryImage = new bool*[m_height / 2];
	for (int i = 0; i < m_height / 2; i++)
		m_binaryImage[i] = new bool[m_width / 2];
	m_blobs = new int*[m_height / 2];
	for (int i = 0; i < m_height / 2; i++)
		m_blobs[i] = new int[m_width / 2];
	m_pixelToCloud = new int[m_width * m_height];
	m_ransac = new RANSAC(50, 20);
	m_blobExtractor = new BlobExtractor(m_width / 2, m_height / 2, 150);
	angle = 0.0;
	lx = 0.0f, lz = -1.0f;
	cx = 0.0f, cz = 5.0f;
	first = true;
}

Viewer::~Viewer()
{
	ms_self = NULL;
	delete[] m_inPlane;
	for (int i = 0; i < m_width * m_height; i++)
		delete[] m_pointCloud[i];
	delete[] m_pointCloud;
	for (int i = 0; i < m_height / 2; i++)
		delete[] m_binaryImage[i];
	delete[] m_binaryImage;
	for (int i = 0; i < m_height / 2; i++)
		delete[] m_blobs[i];
	delete[] m_blobs;
	delete[] m_pixelToCloud;
	delete m_ransac;
	delete m_blobExtractor;
}

openni::Status Viewer::init(int argc, char **argv)
{
	return initOpenGL(argc, argv);
}

openni::Status Viewer::run()
{
	glutMainLoop();

	return openni::STATUS_OK;
}

void Viewer::handleResize(int w, int h) 
{
	//Tell OpenGL how to convert from coordinates to pixel values
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION); //Switch to setting the camera perspective

	//Set the camera perspective
	glLoadIdentity(); //Reset the camera
	gluPerspective(60.0,				  //The camera angle
		(double)w / (double)h, //The width-to-height ratio
		100.0,				   //The near z clipping coordinate
		20000.0);				//The far z clipping coordinate
}

void Viewer::display()
{
	int changedIndex;
	openni::Status rc = openni::OpenNI::waitForAnyStream(m_stream, 2, &changedIndex);
	if (rc != openni::STATUS_OK)
	{
		printf("Wait failed\n");
		return;
	}

	if (changedIndex == 0 && !first)
	{
		m_depth.readFrame(&m_depthFrame);

		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		gluLookAt(cx, 1.0f, cz,
			cx + lx, 1.0f, cz + lz,
			0.0f, 1.0f,  0.0f);

		memset(m_inPlane, false, m_width * m_height * sizeof(bool));

		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
		int rowSize = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
		int cnt = 0;
		for (int y = 0; y < m_depthFrame.getHeight(); y += 2)
		{
			const openni::DepthPixel* pDepth = pDepthRow;

			for (int x = 0; x < m_depthFrame.getWidth(); x += 2, pDepth += 2)
			{
				if (*pDepth != 0)
				{
					float wx, wy, wz;
					openni::CoordinateConverter::convertDepthToWorld(m_depth, x, y, *pDepth, &wx, &wy, &wz);
					//depthToWorld(x, y, *pDepth, &wx, &wy, &wz);
					m_pointCloud[cnt][0] = wx;
					m_pointCloud[cnt][1] = wy;
					m_pointCloud[cnt][2] = wz;
					m_pixelToCloud[y * m_width + x] = cnt;
					cnt++;
				}
				else
					m_pixelToCloud[y * m_width + x] = -1;
			}

			pDepthRow += rowSize * 2;
		}

		float intersect;
		float normalX, normalY, normalZ, d;
		m_ransac->MarkPointsInBestFitPlane(m_pointCloud, cnt, m_inPlane, &normalX, &normalY, &normalZ, &d, &intersect);

		pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
		rowSize = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

		for (int y = 0; y < m_depthFrame.getHeight(); y += 2)
		{
			for (int x = 0; x < m_depthFrame.getWidth(); x += 2)
			{
				m_binaryImage[y / 2][x / 2] = m_pixelToCloud[y * m_width + x] != -1 ? !m_inPlane[m_pixelToCloud[y * m_width + x]] : false;
			}
		}

		int blobs = m_blobExtractor->ExtractBlobs(m_binaryImage, m_blobs);
		vector<float*> blobPoints[blobs];
		vector<openni::RGB888Pixel> blobColors[blobs];
		for (int y = 0; y < m_depthFrame.getHeight(); y += 2)
		{
			for (int x = 0; x < m_depthFrame.getWidth(); x += 2)
			{
				if (m_pixelToCloud[y * m_width + x] != -1)
				{
					int i = m_pixelToCloud[y * m_width + x];
					if (m_blobs[y / 2][x / 2] == 0)
					{
					}
					else
					{
						openni::RGB888Pixel pixel = m_colorData[y * m_width + x];
						blobColors[m_blobs[y / 2][x / 2] - 1].push_back(pixel);
						glColor3f(pixel.r / 255.0, pixel.g / 255.0, pixel.b / 255.0);
						glBegin(GL_POINTS);
						glVertex3f(m_pointCloud[i][0], m_pointCloud[i][1], -m_pointCloud[i][2]);
						glEnd();
						blobPoints[m_blobs[y / 2][x / 2] - 1].push_back(m_pointCloud[i]);
					}
				}
			}
		}
		float maxPercentRed = -1;
		float maxPercentBlue = -1;
		int blueBlob;
		int redBlob;
		for (int i = 0; i < blobs; i++)
		{
			int numBlue = 0, numRed = 0;
			for (vector<openni::RGB888Pixel>::iterator it = blobColors[i].begin(); it < blobColors[i].end(); it++)
			{
				openni::RGB888Pixel pixel = *it;
				if (pixel.b > 100 && pixel.b - pixel.r > 0 && pixel.b - pixel.g > 0)
				{
					numBlue++;
				}
				if (pixel.r > 100 && pixel.r - pixel.g > 50 && pixel.r - pixel.b > 50)
				{
					numRed++;
				}
			}
			if ((float)numBlue / blobColors[i].size() > maxPercentBlue)
			{
				maxPercentBlue = (float)numBlue / blobColors[i].size();
				blueBlob = i;
			}
			if ((float)numRed / blobColors[i].size() > maxPercentRed)
			{
				maxPercentRed = (float)numRed / blobColors[i].size();
				redBlob = i;
			}
		}
		float redX, redY, redZ;
		float blueX, blueY, blueZ;
		if (blobs > 0)
		{
			pointCloudCenter(blobPoints[blueBlob], blobColors[blueBlob], &blueX, &blueY, &blueZ);
			glPointSize( 20.0 );
			glColor3f(0, 0, 1);
			glBegin(GL_POINTS);
			glVertex3f(blueX, blueY, -blueZ);
			glEnd();

			pointCloudCenter(blobPoints[redBlob], blobColors[redBlob], &redX, &redY, &redZ);
			glColor3f(1, 0, 0);
			glBegin(GL_POINTS);
			glVertex3f(redX, redY, -redZ);
			glEnd();
			glPointSize( 2.0 );
		}
		float originX = -d * normalX;
		float originY = -d * normalY;
		float originZ = -d * normalZ;
		glColor3f(1, 1, 0);
//		glBegin(GL_LINES);
//		glVertex3f(0,0,-intersect);
//		glVertex3f(normalX * 100, normalY * 100, -(intersect + normalZ * 100));
//		glEnd();
//		glBegin(GL_LINES);
//		glVertex3f(0,0,0);
//		glVertex3f(0,0,-intersect);
//		glEnd();
//		glBegin(GL_LINES);
//		glVertex3f(originX, originY, -originZ);
//		glVertex3f(0, 0, -intersect);
//		glEnd();
		float yx = -normalX;
		float yy = -normalY;
		float yz = -normalZ;
		float mag = sqrt(originX * originX + originY * originY + (originZ - intersect) * (originZ - intersect));
		float zx = -originX / mag;
		float zy = -originY / mag;
		float zz = (intersect - originZ) / mag;
		float xx = yy * zz - yz * zy;
		float xy = yz * zx - yx * zz;
		float xz = yx * zy - yy * zx;
//		originX = originY = 0;
//		originZ = intersect;
		glColor3f(0, 1, 1);
		glBegin(GL_LINES);
		glVertex3f(originX, originY, -originZ);
		glVertex3f(originX + zx * 100, originY + zy * 100, -(originZ + zz * 100));
		glEnd();
		glColor3f(1, 0, 1);
		glBegin(GL_LINES);
		glVertex3f(originX, originY, -originZ);
		glVertex3f(originX + yx * 100, originY + yy * 100, -(originZ + yz * 100));
		glEnd();
		glColor3f(1, 1, 0);
		glBegin(GL_LINES);
		glVertex3f(originX, originY, -originZ);
		glVertex3f(originX + xx * 100, originY + xy * 100, -(originZ + xz * 100));
		glEnd();
		Matrix<float> transform(4, 4);
		Matrix<float> blue(4, 1);
		Matrix<float> red(4, 1);
		transform.put(0, 0, xx);
		transform.put(1, 0, xy);
		transform.put(2, 0, xz);
		transform.put(3, 0, 0);
		transform.put(0, 1, yx);
		transform.put(1, 1, yy);
		transform.put(2, 1, yz);
		transform.put(3, 1, 0);
		transform.put(0, 2, zx);
		transform.put(1, 2, zy);
		transform.put(2, 2, zz);
		transform.put(3, 2, 0);
		transform.put(0, 3, originX);
		transform.put(1, 3, originY);
		transform.put(2, 3, originZ);
		transform.put(3, 3, 1);
		blue.put(0, 0, blueX);
		blue.put(1, 0, blueY);
		blue.put(2, 0, blueZ);
		blue.put(3, 0, 1);
		red.put(0, 0, redX);
		red.put(1, 0, redY);
		red.put(2, 0, redZ);
		red.put(3, 0, 1);
		Matrix<float> localBlue = transform.getInverse() * blue;
		Matrix<float> localRed = transform.getInverse() * red;

		printf("Blue: %f, %f, %f\n", localBlue.get(0, 0), localBlue.get(1, 0), localBlue.get(2, 0));
		printf("Red: %f, %f, %f\n", localRed.get(0, 0), localRed.get(1, 0), localRed.get(2, 0));
		fflush(stdout);

		// Swap the OpenGL display buffers
		glutSwapBuffers();
		first = true;
	}
	else
	{
		m_color.readFrame(&m_colorFrame);
		m_colorData = (const openni::RGB888Pixel*)m_colorFrame.getData();
		first = false;
	}
}

void Viewer::pointCloudCenter(vector<float*> &cloud, vector<openni::RGB888Pixel> &rgb, float *x, float *y, float *z)
{
	float sumX = 0, sumY = 0, sumZ = 0;
	int count = 0;
	for (unsigned int i = 0; i < cloud.size(); i++)
	{
		openni::RGB888Pixel pixel = rgb[i];
		if ((pixel.b > 100 && pixel.b - pixel.r > 0 && pixel.b - pixel.g > 0) || (pixel.r > 100 && pixel.r - pixel.g > 50 && pixel.r - pixel.b > 50))
		{
			sumX += (cloud[i])[0];
			sumY += (cloud[i])[1];
			sumZ += (cloud[i])[2];
			count++;
		}
	}
	*x = sumX / (float)count;
	*y = sumY / (float)count;
	*z = sumZ / (float)count;
}

float Viewer::raw_depth_to_meters(int depth_value)
{ 
	float depth_value_f = (float) depth_value; 
	if (depth_value < 2047){ 
		float depth = 1.0 / (depth_value_f  * -0.0030711016 + 3.3309495161);
		return depth; 
	}
	return 0.0f; 
} 

void Viewer::depthToWorld(float cgx, float cgy, float cgz, float *x, float *y, float *z)
{
	double fx_d = 1.0 / 5.9421434211923247e+02;
	double fy_d = 1.0 / 5.9104053696870778e+02;
	double cx_d = 3.3930780975300314e+02;
	double cy_d = 2.4273913761751615e+02;

	float depth = raw_depth_to_meters(cgz);

	*x = (float) (cgx - cx_d) * depth * fx_d;
	*y = (float) (cgy - cy_d) * depth * fy_d;
	*z = (float) depth;
}

void Viewer::onKey(unsigned char key, int /*x*/, int /*y*/)
{
	float fraction = 10.0f;

	switch (key) {
	case GLUT_KEY_LEFT :
		angle -= 0.01f;
		lx = sin(angle);
		lz = -cos(angle);
		break;
	case GLUT_KEY_RIGHT :
		angle += 0.01f;
		lx = sin(angle);
		lz = -cos(angle);
		break;
	case GLUT_KEY_UP :
		cx += lx * fraction;
		cz += lz * fraction;
		break;
	case GLUT_KEY_DOWN :
		cx -= lx * fraction;
		cz -= lz * fraction;
		break;
	case 27:
		m_depth.stop();
		m_depth.destroy();
		openni::OpenNI::shutdown();
		exit (1);
		break;
	}
}

openni::Status Viewer::initOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow (m_strSampleName);
	// 	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	initOpenGLHooks();

	glEnable(GL_DEPTH_TEST);
	glPointSize( 2.0 );

	return openni::STATUS_OK;

}
void Viewer::initOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	glutReshapeFunc(glutResize);
	glutSpecialFunc(glutSpecialKeys);
}
