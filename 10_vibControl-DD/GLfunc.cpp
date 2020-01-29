#include "GLfunc.h"
#include "Calculation.h"


// https://w.atwiki.jp/opengl/pages/23.html
void Circle2D(float radius, int x, int y)
{
	for (float th1 = 0.0; th1 <= 360.0; th1 = th1 + 1.0)
	{
		float th2 = th1 + 10.0;
		float th1_rad = th1 / 180.0 * M_PI;
		float th2_rad = th2 / 180.0 * M_PI;

		float x1 = radius * cos(th1_rad);
		float y1 = radius * sin(th1_rad);
		float x2 = radius * cos(th2_rad);
		float y2 = radius * sin(th2_rad);

		glBegin(GL_LINES);
		glVertex2f(x1 + x, y1 + y);
		glVertex2f(x2 + x, y2 + y);
		glEnd();
	}
}
void Circle2DFill(float radius, int x, int y)
{
	for (float th1 = 0.0; th1 <= 360.0; th1 = th1 + 1.0)
	{
		float th2 = th1 + 10.0;
		float th1_rad = th1 / 180.0 * M_PI;
		float th2_rad = th2 / 180.0 * M_PI;

		float x1 = radius * cos(th1_rad);
		float y1 = radius * sin(th1_rad);
		float x2 = radius * cos(th2_rad);
		float y2 = radius * sin(th2_rad);

		glBegin(GL_TRIANGLES);
		glVertex2f(x, y);
		glVertex2f(x1 + x, y1 + y);
		glVertex2f(x2 + x, y2 + y);
		glEnd();
	}
}
void Oval2D(float radius, int x, int y, float ovalx, float ovaly)
{
	for (float th1 = 0.0; th1 <= 360.0; th1 = th1 + 1.0)
	{
		float th2 = th1 + 10.0;
		float th1_rad = th1 / 180.0 * M_PI;
		float th2_rad = th2 / 180.0 * M_PI;

		float x1 = radius * cos(th1_rad)*(ovalx / 100.0f);
		float y1 = radius * sin(th1_rad)*(ovaly / 100.0f);
		float x2 = radius * cos(th2_rad)*(ovalx / 100.0f);
		float y2 = radius * sin(th2_rad)*(ovaly / 100.0f);

		glBegin(GL_LINES);
		glVertex2f(x1 + x, y1 + y);
		glVertex2f(x2 + x, y2 + y);
		glEnd();
	}
}
void Oval2DFill(float radius, int x, int y, float ovalx, float ovaly)
{
	for (float th1 = 0.0; th1 <= 360.0; th1 = th1 + 1.0)
	{
		float th2 = th1 + 10.0;
		float th1_rad = th1 / 180.0 * M_PI;
		float th2_rad = th2 / 180.0 * M_PI;

		float x1 = radius * cos(th1_rad)*(ovalx / 100.0f);
		float y1 = radius * sin(th1_rad)*(ovaly / 100.0f);
		float x2 = radius * cos(th2_rad)*(ovalx / 100.0f);
		float y2 = radius * sin(th2_rad)*(ovaly / 100.0f);

		glBegin(GL_TRIANGLES);
		glVertex2f(x, y);
		glVertex2f(x1 + x, y1 + y);
		glVertex2f(x2 + x, y2 + y);
		glEnd();
	}
}


// https://yukun.info/opengl-circle/
void drawCircle(float radius, int partition, float x, float y)
{
	float ver_x, ver_y = 0.0;
	float rate;

	glBegin(GL_POLYGON); // ƒ|ƒŠƒSƒ“‚Ì•`‰æ
	// ‰~‚ð•`‰æ
	for (int i = 0; i < partition; i++) {
		// À•W‚ðŒvŽZ
		rate = (double)i / partition;
		ver_x = radius * cos(2.0 * M_PI * rate);
		ver_y = radius * sin(2.0 * M_PI * rate);
		ver_x *= aspectRatio;	//windowsize‚É‚æ‚é‚ä‚ª‚Ý•â³
		ver_x += x;
		ver_y += y;
		glVertex2f(ver_x, ver_y); // ’¸“_À•W‚ðŽw’è
	}
	glEnd(); // ƒ|ƒŠƒSƒ“‚Ì•`‰æI—¹

}

