#include <GL/glut.h>
#include "isometry.hpp"

#define TIMER_ID 		    0
#define TIMER_INTERVAL      25
#define ANIMATION_INTERVAL  0.02

//Declare callback functions
void on_keyboard(unsigned char key, int x, int y);
void on_reshape(int width, int height);
void on_display(void);
void on_timer(int id);

//Coordinate system
void draw_coosys();

//Animation time
double t = 0;

bool animation_ongoing = false;

//Overall time of animation
double overall_time = 1.5;

//Translation parameters
double x_t = 0, y_t = 0, z_t = 0;

//Rotation parameters
double phi = 0, theta = 0, psi = 0;

//Start and end coordinates of object
double startCentar[] = {0.0, 0.0, 0.0};
double endCentar[] = {10.0, 4.0, 2.0};

//Start and end angles of object
double startEuler[] = {10.0, 10.0, 45.0};
double endEuler[]   = {45.0, -45.0, -30.0};



int main(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

	glutInitWindowSize(800, 800);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Slerp Animation");

	glutKeyboardFunc(on_keyboard);
	glutReshapeFunc(on_reshape);
	glutDisplayFunc(on_display);

	//OpenGL initialization
	glClearColor(0, 0, .2, 0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);

	glutMainLoop();

	return 0;
}


void on_keyboard(unsigned char key, int x, int y) {
	(void)x; (void)y;

	switch (key) {
	//On 'ESC' or 'q' exit program
	case 27:
	case 'q':
	case 'Q':
		exit(EXIT_SUCCESS);

	//Start animation
	case ' ':
		if (!animation_ongoing) {
			glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
			animation_ongoing = true;
		}
		break;

	//Stop animation
	case 's':
	case 'S':
		animation_ongoing = false;
		break;

	//Reset animation
	case 'r':
	case 'R':
		t = 0;
		if (!animation_ongoing) {
			glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
			animation_ongoing = true;
		}
		break;
	}
}


void on_reshape(int width, int height) {
	glViewport(0, 0, width, height);
	//Set projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (float) width / height, 1, 100);
}


void on_timer(int id) {
	if (id != TIMER_ID) return;

	//End of animation
	if(t >= overall_time) {
		animation_ongoing = false;
	}
	else {
		//Time interval
		t += ANIMATION_INTERVAL;
	}

	glutPostRedisplay();

	//Set timer again if need so
	if (animation_ongoing) {
		glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
	}
}


void draw_coosys() {
	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(1, 0, 0);
	glVertex3f(0, 0, 0);

	glColor3f(0, 1, 0);
	glVertex3f(0, 1, 0);
	glVertex3f(0, 0, 0);

	glColor3f(0, 0, 1);
	glVertex3f(0, 0, 1);
	glVertex3f(0, 0, 0);
	glEnd();
}


static void calcTranslationParameters(double &x_t, double &y_t, double &z_t, double t) {
	x_t = (1 - t / overall_time) * startCentar[0] + t * endCentar[0] / overall_time;
	y_t = (1 - t / overall_time) * startCentar[1] + t * endCentar[1] / overall_time;
	z_t = (1 - t / overall_time) * startCentar[2] + t * endCentar[2] / overall_time;
}


static void calcRotationParameters(double &phi, double &theta, double &psi, double t) {
	auto axisAngle1 = A2AxisAngle( Euler2A( startEuler[0] * M_PI / 180.0,
											startEuler[1] * M_PI / 180.0,
											startEuler[2] * M_PI / 180.0));
	auto q1 = AxisAngle2Q(axisAngle1.first, axisAngle1.second);

	auto axisAngle2 = A2AxisAngle( Euler2A( endEuler[0] * M_PI / 180.0,
											endEuler[1] * M_PI / 180.0,
											endEuler[2] * M_PI / 180.0));
	auto q2 = AxisAngle2Q(axisAngle2.first, axisAngle2.second);

	auto resAxisAngle = Q2AxisAngle(slerp(q1, q2, overall_time, t));
	auto euler_t = A2Euler(Rodrigues(resAxisAngle.first, resAxisAngle.second));

	phi     = euler_t[0] * 180 / M_PI;
	theta   = euler_t[1] * 180 / M_PI;
	psi     = euler_t[2] * 180 / M_PI;
}


static void animate() {
	//Ending coordinate system
	glPushMatrix();
		glTranslatef(endCentar[0], endCentar[1], endCentar[2]);
		glRotatef(endEuler[0], 1, 0, 0);
		glRotatef(endEuler[1], 0, 1, 0);
		glRotatef(endEuler[2], 0, 0, 1);
		draw_coosys();
	glPopMatrix();


	//Set transformation parameters
	calcTranslationParameters(x_t, y_t, z_t, t);
	calcRotationParameters(phi, theta, psi, t);

	//Create object and move it
    glPushMatrix();
		glColor3f(1, 1, 0);

		glTranslatef(x_t, y_t, z_t);

		glRotatef(phi, 1, 0, 0);
		glRotatef(theta, 0, 1, 0);
		glRotatef(psi, 0, 0, 1);

		glutWireTeapot(1);
	glPopMatrix();

	//Starting coordinate system
	glPushMatrix();
		glTranslatef(startCentar[0], startCentar[1], startCentar[2]);
		glRotatef(startEuler[0], 1, 0, 0);
		glRotatef(startEuler[1], 0, 1, 0);
		glRotatef(startEuler[2], 0, 0, 1);
		draw_coosys();
	glPopMatrix();
}

void on_display(void) {
	//Clear previous content of window
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Set view point
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(
		12, 2, 12,
		6, 2, 0,
		0, 1, 0);

	animate();

	glutSwapBuffers();
}
