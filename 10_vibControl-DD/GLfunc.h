#pragma once

#include <gl/gl_screenshot.h> //bitmapèoóÕóp
#include <gl/glut.h>
#include <gl/freeglut.h>	//to leave glutMainLoop


#define WIDTH 400
#define HEIGHT 600
const float aspectRatio = (float)HEIGHT / WIDTH;
//const float extra_to_1 = 0.1 - 0.85 + length*DoF;	//égÇÌÇ»Ç¢Ç©Ç‡
const float extra_to_1 = 0.1 - 0.85 + 0.35*10;

void Circle2D(float radius, int x, int y);

void Circle2DFill(float radius, int x, int y);

void Oval2D(float radius, int x, int y, float ovalx, float ovaly);

void Oval2DFill(float radius, int x, int y, float ovalx, float ovaly);

void drawCircle(float radius, int partition, float x, float y);


