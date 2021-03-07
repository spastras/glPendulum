/* =============== */
/* Header Includes */
/* =============== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

/* ===================== */
/* Constants Declaration */
/* ===================== */

/* Integration Method */

//#define EULER
#define VERLET

/* Physical Constants */

#define mass 1.0
#define g 10.0
#define dt 0.001

/* Initial Conditions */

#define r0 1.0
#define theta0 M_PI_2
#define fi0 0.0

#define pr0 0.0
#define ptheta0 0.0
#define pfi0 1.5

/*
#define pr0 0.0
#define ptheta0 1.0
#define pfi0 sqrt(r0/g)
*/
/*
#define pr0 0.0
#define ptheta0 5.0
#define pfi0 10*sqrt(r0/g)
*/

#define epsilon 0.0
//#define epsilon 0.01

/* Graphical Configuration */

#define WINDOWWIDTH 750
#define WINDOWHEIGHT 750
#define SUBWINDOWBORDER 6

#define FPS 60.0
#define PATHSIZE 100000
#define PHASESPACESIZE 100000

#define RADIANSPERDEGREE 0.0174533
#define SPHEREDENSITY 2000.0
#define ARROWSCALE 0.2
#define ARROWDIAMETER 0.01

#define NGRID 10
#define BOXLEN 2.0
#define BOXEDGE (float)BOXLEN/2.0

#define NPHASESPACEGRID 10
#define PHASESPACEOFFSET 0.30
#define PHASESPACEAXISMARKERSIZE 0.02
#define PHASESPACEAXISMARKEROFFSET 0.20
#define PHASESPACEAXISNAMEOFFSET 0.03
#define PHASESPACEXAXISVALUEOFFSET 0.09
#define PHASESPACEYAXISVALUEOFFSET 0.05
#define PHASESPACECURRENTMARKERSIZE 3.0

#define TOGGLEROTATION 1
#define TOGGLETRACERS 2
#define TOGGLE2DPATH 3
#define TOGGLE3DPATH 4
#define TOGGLEMOMENTUM 5
#define TOGGLEPHASESPACE 6

#define TOGGLEPHASESPACEINTERVAL 10

#define SHOWFPS 1
#define SHOWTIME 1
#define SHOWENERGY 1

/* ========== */
/* Structures */
/* ========== */

struct pendulum {
    float m;
    float r, theta, fi;
    float pr, ptheta, pfi;
};

/* ================ */
/* Global Variables */
/* ================ */

/* pthread Variables */

pthread_t integrateThreadID;

/* GLUT Variables */

int mainWindow, subWindow[3];
int menuInUse=0;

/* Camera Configuration */

float rCam=1.0;
float thetaCam=M_PI_2*3.0/4.0;
float fiCam=-M_PI_4/2.0;

float rStepCam=0.2;
float angleStepCamActive=0.02f;
float angleStepCamPassive=0.01f;

/* Control Variables */

int rotationEnabled=0;
int tracersEnabled=0;
int path2DEnabled=0;
int path3DEnabled=1;
int momentumEnabled=0;
int phaseSpaceEnabled=0;
int integrate=1;

float deltaRCam=0.0;
float deltaThetaCam=0.0;
float deltaFiCam=0.0;

int pathCurrent=0;
int phaseSpaceCurrent=0;

float stepsPerSecond=1000.0;

float deltaStepsPerSecondStep=1.0;

float deltaStepsPerSecond=0.0;

/* Data Variables */

float t;

struct pendulum pendulum;

float x[PATHSIZE], y[PATHSIZE], z[PATHSIZE];

float r[PHASESPACESIZE], theta[PHASESPACESIZE], fi[PHASESPACESIZE];
float pr[PHASESPACESIZE], ptheta[PHASESPACESIZE], pfi[PHASESPACESIZE];
float rlim[2], thetalim[2], filim[2];
float prlim[2], pthetalim[2], pfilim[2];

float initialEnergy;

/* Graphical Variables */

GLfloat light_position[] = {1.0,1.0,1.0,0.0};
GLfloat mat_specular[] = {1.0,1.0,1.0,1.0};
GLfloat mat_shininess[] = {50.0};
GLfloat qaWhite[] = {1.0, 1.0, 1.0, 1.0};
GLfloat qaBrown[] = {0.82, 0.70, 0.55, 1.0};
GLfloat qaBlack[] = {0.0, 0.0, 0.0, 1.0};
GLfloat qaYellow[] = {1.0, 1.0, 0.0, 1.0};
GLfloat qaOffWhite[] = {0.25, 0.25, 0.25, 1.0};
GLfloat qaOffRed[] = {0.25, 0.0, 0.0, 1.0};
GLfloat qaOffGreen[] = {0.0, 0.25, 0.0, 1.0};
GLfloat qaOffBlue[] = {0.0, 0.0, 0.25, 1.0};
GLfloat qaOffYellow[] = {0.25, 0.25, 0.0, 1.0};

/* =================== */
/* Function Prototypes */
/* =================== */

float computeEnergy(void);

void printUsage(void);
void printStatus(int status);

void setInitialConditions(void);
void *integrateThread(void *arg);
void takeStep(void);
void redisplayTimer(int value);
void togglePhaseSpaceTimer(int value);

void normalKey(unsigned char key, int x, int y);
void normalKeyRelease(unsigned char key, int x, int y);
void specialKey(int key, int x, int y);
void specialKeyRelease(int key, int x, int y);
void processMenuEvent(int option);
void processMenuStatus(int status, int x, int y);

void glMarkPoint(float x, float y);
void drawPhaseSpaceAxes(float *xlim, float *ylim, char *xname, char *yname);
void glPrintString(char *string, float x, float y, float z);
void glArrow(GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2, GLfloat D, GLfloat *qaColor);

void configureWindow(void);
void setProjection(int w, int h);
void resize(int w, int h);

void display(void);
void display0(void);
void display1(void);
void display2(void);

void createPhaseSpaceSubWindows(void);

int main(int argc, char **argv);