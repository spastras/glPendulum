#include "glPendulum.h"

/* computeEnergy */

float computeEnergy(void) {
    return (1.0/(2.0*pendulum.m))*(pendulum.pr*pendulum.pr+pendulum.ptheta*pendulum.ptheta/(pendulum.r*pendulum.r)+pendulum.pfi*pendulum.pfi/(pendulum.r*pendulum.r*sin(pendulum.theta)*sin(pendulum.theta)))+pendulum.m*g*pendulum.r*cos(pendulum.theta);
}

/* printUsage */

void printUsage(void) {
    printf("\n");
    printf("[i] -<================>-\n");
    printf("[i] -<=[ glPendulum ]=>-\n");
    printf("[i] -<================>-\n");
    printf("\n");
    printf("[i] Controls:\n");
    printf("\n");
    printf("[i] (arrows)\t->\trotate\n");
    printf("[i] r\t\t->\tenable\\disable rotation\n");
    printf("[i] ALT + r\t->\tswitch direction of rotation\n");
    printf("[i] p\t\t->\tplay\\pause\n");
    printf("[i] ALT + p\t->\treset\n");
    printf("[i] + \\ -\t->\tincrease\\decrease steps per second\n");
    printf("[i] q\t\t->\tquit\n");
    printf("\n");
    return;
}

/* printStatus */

void printStatus(int status) {
    printf("\e[2K\r[i] Status: ");
    switch (status) {
        case 0 :
            printf("paused");
            break;
		case 1 :
            printf("running");
            break;
		case -1 :
            printf("initializing");
            break;
		case -2 :
            printf("exiting\n");
            break;
	}
    fflush(stdout);
    return;
}

/* setInitialConditions */

void setInitialConditions(void) {
    pendulum.m=mass;
    pendulum.r=r0;
    pendulum.theta=theta0;
    pendulum.fi=fi0;
    pendulum.pr=pr0;
    pendulum.ptheta=ptheta0;
    pendulum.pfi=pfi0;
    t=0.0;
    return;
}

/* integrateThread */

void *integrateThread(void *arg) {
    printStatus(1);
    while((integrate)&&(pendulum.r>0.0)) {
        stepsPerSecond+=deltaStepsPerSecond;
        usleep((useconds_t)1000000.0/stepsPerSecond);
        takeStep();
    }
    if(integrate) {
        integrate=!integrate;
    }
    printStatus(0);
    return NULL;
}

/* takeStep */

void takeStep(void) {

    int i;

    float r1, pr1;
    float theta1, ptheta1;
    float fi1, pfi1;
    float costheta,sintheta;

    if(pendulum.r<=0.0) {
        return;
    }
    
    sintheta=sin(pendulum.theta);
    costheta=cos(pendulum.theta);

    #ifdef VERLET

    //pr1=pendulum.pr-dt*(-(1.0/(pendulum.m*pendulum.r*pendulum.r*pendulum.r))*(pendulum.ptheta*pendulum.ptheta+pendulum.pfi*pendulum.pfi/(sintheta*sintheta))+pendulum.m*g*costheta);
    //r1=pendulum.r+dt*pr1/pendulum.m;
    pr1=pendulum.pr;
    r1=pendulum.r-epsilon*dt;
    ptheta1=pendulum.ptheta-dt*(-(pendulum.pfi*pendulum.pfi*costheta)/(pendulum.m*pendulum.r*pendulum.r*sintheta*sintheta*sintheta)-pendulum.m*g*pendulum.r*sintheta);
    theta1=pendulum.theta+dt*ptheta1/(pendulum.m*r1*r1);
    pfi1=pendulum.pfi;
    fi1=pendulum.fi+dt*pfi1/(pendulum.m*r1*r1*sin(theta1)*sin(theta1));

    #endif

    #ifdef EULER

    //r1=pendulum.r+dt*pendulum.pr/pendulum.m;
    //pr1=pendulum.pr-dt*(-(1.0/(pendulum.m*pendulum.r*pendulum.r*pendulum.r))*(pendulum.ptheta*pendulum.ptheta+pendulum.pfi*pendulum.pfi/(sintheta*sintheta))+pendulum.m*g*costheta);
    r1=pendulum.r-epsilon*dt;
    pr1=pendulum.pr;
    theta1=pendulum.theta+dt*pendulum.ptheta/(pendulum.m*pendulum.r*pendulum.r);
    ptheta1=pendulum.ptheta-dt*(-(pendulum.pfi*pendulum.pfi*costheta)/(pendulum.m*pendulum.r*pendulum.r*sintheta*sintheta*sintheta)-pendulum.m*g*pendulum.r*sintheta);
    fi1=pendulum.fi+dt*pendulum.pfi/(pendulum.m*pendulum.r*pendulum.r*sintheta*sintheta);
    pfi1=pendulum.pfi;
    
    #endif

    if(fi1>=2.0*M_PI) {
        fi1=fi1-2.0*M_PI;
    }
    if(fi1<0.0) {
        fi1=2.0*M_PI-fi1;
    }

    pendulum.r=r1;
    pendulum.pr=pr1;
    pendulum.theta=theta1;
    pendulum.ptheta=ptheta1;
    pendulum.fi=fi1;
    pendulum.pfi=pfi1;

    t=t+dt;

    if(path2DEnabled||path3DEnabled) {

        /* Save path point */

        if(pathCurrent==PATHSIZE) {
            pathCurrent=0;
        }

        x[pathCurrent]=pendulum.r*sin(pendulum.theta)*cos(pendulum.fi);
        y[pathCurrent]=pendulum.r*sin(pendulum.theta)*sin(pendulum.fi);
        z[pathCurrent]=pendulum.r*cos(pendulum.theta);
        pathCurrent++;

    }

    if(phaseSpaceEnabled) {

        /* Save phase space point */

        if(phaseSpaceCurrent==PHASESPACESIZE) {
            phaseSpaceCurrent=0;
        }

        if(phaseSpaceCurrent==0) {
            for(i=0;i<2;i++) {
                rlim[i]=pendulum.r;
                thetalim[i]=pendulum.theta;
                filim[i]=pendulum.fi;
                prlim[i]=pendulum.pr;
                pthetalim[i]=pendulum.ptheta;
                pfilim[i]=pendulum.pfi;
            }
            rlim[0]=0.0;
            if(pendulum.pfi>=0.0) {
                pfilim[0]=0.0;
                pfilim[1]=pendulum.pfi;
            }
            else {
                pfilim[0]=pendulum.pfi;
                pfilim[1]=0.0;
            }
        }
        else {
            /*
            if(pendulum.r<rlim[0]) {
                rlim[0]=pendulum.r;
            }
            */
            if(pendulum.r>rlim[1]) {
                rlim[1]=pendulum.r;
            }
            if(pendulum.theta<thetalim[0]) {
                thetalim[0]=pendulum.theta;
            }
            if(pendulum.theta>thetalim[1]) {
                thetalim[1]=pendulum.theta;
            }
            if(pendulum.fi<filim[0]) {
                filim[0]=pendulum.fi;
            }
            if(pendulum.fi>filim[1]) {
                filim[1]=pendulum.fi;
            }
            /*
            if(pendulum.pr<prlim[0]) {
                prlim[0]=pendulum.pr;
            }
            if(pendulum.pr>prlim[1]) {
                prlim[1]=pendulum.pr;
            }
            */
            if(pendulum.ptheta<pthetalim[0]) {
                pthetalim[0]=pendulum.ptheta;
            }
            if(pendulum.ptheta>pthetalim[1]) {
                pthetalim[1]=pendulum.ptheta;
            }
            /*
            if(pendulum.pfi<pfilim[0]) {
                pfilim[0]=pendulum.pfi;
            }
            if(pendulum.pfi>pfilim[1]) {
                pfilim[1]=pendulum.pfi;
            }
            */
        }

        r[phaseSpaceCurrent]=pendulum.r;
        theta[phaseSpaceCurrent]=pendulum.theta;
        fi[phaseSpaceCurrent]=pendulum.fi;
        pr[phaseSpaceCurrent]=pendulum.pr;
        ptheta[phaseSpaceCurrent]=pendulum.ptheta;
        pfi[phaseSpaceCurrent]=pendulum.pfi;
        phaseSpaceCurrent++;

    }

    //printf("%lf %lf %lf %lf %lf %lf\n",r1,pr1,theta1,ptheta1,fi1,pfi1);

    return;

}

/* redisplayTimer */

void redisplayTimer(int value) {
    glutSetWindow(mainWindow);
    glutPostRedisplay();
    glutTimerFunc(1000.0/FPS, redisplayTimer, 0);
    return;
}

/* togglePhaseSpaceTimer */

void togglePhaseSpaceTimer(int value) {
    if(!menuInUse) {
        phaseSpaceEnabled=!phaseSpaceEnabled;
        phaseSpaceCurrent=0;
        if(!phaseSpaceEnabled) {
            glutDestroyWindow(subWindow[1]);
            glutDestroyWindow(subWindow[2]);
        }
        else {
            createPhaseSpaceSubWindows();
        }
        glutSetWindow(mainWindow);
        resize(glutGet(GLUT_WINDOW_WIDTH),glutGet(GLUT_WINDOW_HEIGHT));
    }
    else {
        glutTimerFunc(TOGGLEPHASESPACEINTERVAL, togglePhaseSpaceTimer, 0);
    }
    return;
}

/* normalKey */

void normalKey(unsigned char key, int x, int y) {
    int modifiers;
    switch (key) {
		case 27 :
		case 113 :
            printStatus(-2);
            exit(0);
            break;
		case 114 :
            modifiers=glutGetModifiers();
            if(modifiers==GLUT_ACTIVE_ALT) {
                angleStepCamPassive=-angleStepCamPassive;
            }
            else {
                rotationEnabled=!rotationEnabled;
            }
            break;
		case 112 :
            modifiers=glutGetModifiers();
            if(modifiers==GLUT_ACTIVE_ALT) {
                pathCurrent=0;
                phaseSpaceCurrent=0;
                setInitialConditions();
            }
            else {
                integrate=!integrate;
                if(integrate) {
                    pthread_create(&integrateThreadID, NULL, integrateThread, NULL);
                }
                else {
                    pthread_join(integrateThreadID, NULL);
                }
            }
            break;
        case 43 :
            deltaStepsPerSecond=deltaStepsPerSecondStep;
            break;
        case 45 :
            deltaStepsPerSecond=-deltaStepsPerSecondStep;
            break;
        case 119 :
            //deltaRCam+=rStepCam;
            break;
        case 115 :
            //deltaRCam-=rStepCam;
            break;
	}
    return;
}

/* normalKeyRelease */

void normalKeyRelease(unsigned char key, int x, int y) {
	switch (key) {
        case 43 :
		case 45 :
            deltaStepsPerSecond=0.0;
            break;
		case 119 :
		case 115 :
            //deltaRCam = 0.0;
            break;
	}
    glutSetWindow(mainWindow);
    glutPostRedisplay();
    return;
}

/* specialKey */

void specialKey(int key, int x, int y) {
	switch (key) {
		case GLUT_KEY_LEFT :
			deltaFiCam-=angleStepCamActive;
			break;
		case GLUT_KEY_RIGHT :
			deltaFiCam+=angleStepCamActive;
			break;
		case GLUT_KEY_UP :
			deltaThetaCam-=angleStepCamActive;
			break;
		case GLUT_KEY_DOWN :
			deltaThetaCam+=angleStepCamActive;
			break;
	}
    glutSetWindow(mainWindow);
    glutPostRedisplay();
    return;
}

/* specialKeyRelease */

void specialKeyRelease(int key, int x, int y) {
	switch (key) {
		case GLUT_KEY_LEFT :
		case GLUT_KEY_RIGHT :
            deltaFiCam = 0.0;
            break;
		case GLUT_KEY_UP :
		case GLUT_KEY_DOWN :
            deltaThetaCam = 0.0;
            break;
	}
    glutSetWindow(mainWindow);
    glutPostRedisplay();
    return;
}

/* processMenuEvent */

void processMenuEvent(int option) {
	switch (option) {
        case TOGGLEROTATION :
			rotationEnabled=!rotationEnabled;
            break;
		case TOGGLETRACERS :
			tracersEnabled=!tracersEnabled;
            break;
        case TOGGLE2DPATH :
			path2DEnabled=!path2DEnabled;
            pathCurrent=0;
            break;
        case TOGGLE3DPATH :
			path3DEnabled=!path3DEnabled;
            pathCurrent=0;
            break;
        case TOGGLEMOMENTUM :
			momentumEnabled=!momentumEnabled;
            break;
        case TOGGLEPHASESPACE :
            glutTimerFunc(TOGGLEPHASESPACEINTERVAL, togglePhaseSpaceTimer, 0);
            break;
	}
    return;
}

/* processMenuStatus */

void processMenuStatus(int status, int x, int y) {
	if(status==GLUT_MENU_IN_USE) {
		menuInUse=1;
    }
	else {
		menuInUse=0;
    }
    return;
}

/* glMarkPoint */

void glMarkPoint(float x, float y) {
    glColor3f(1.0,0.0,0.0);
    glEnable(GL_POINT_SMOOTH);
    glPointSize(PHASESPACECURRENTMARKERSIZE);
    glBegin(GL_POINTS);
        glVertex2f(x,y);
    glEnd();
    return;
}

/* drawPhaseSpaceAxes */

void drawPhaseSpaceAxes(float *xlim, float *ylim, char *xname, char *yname) {

    int i, j;

    float dx, dy;
    float xmid, ymid;

    char valueString[64];

    /* Set helper variables */

    dx=xlim[1]-xlim[0];
    dy=ylim[1]-ylim[0];
    xmid=(xlim[1]+xlim[0])/2.0;
    ymid=(ylim[1]+ylim[0])/2.0;

    /* Draw grid */

    glColor3f(0.3,0.3,0.3);
    glBegin(GL_LINES);
        glVertex2f(xlim[0]-PHASESPACEOFFSET*dx,ymid);
        glVertex2f(xlim[1]+PHASESPACEOFFSET*dx,ymid);
        glVertex2f(xmid,ylim[0]-PHASESPACEOFFSET*dy);
        glVertex2f(xmid,ylim[1]+PHASESPACEOFFSET*dy);
        for(i=0;i<=NPHASESPACEGRID;i++) {
            glVertex2f(xmid-dx*PHASESPACEAXISMARKERSIZE,ylim[0]-PHASESPACEAXISMARKEROFFSET*dy+i*dy*(1.0+2.0*PHASESPACEAXISMARKEROFFSET)/NGRID);
            glVertex2f(xmid+dx*PHASESPACEAXISMARKERSIZE,ylim[0]-PHASESPACEAXISMARKEROFFSET*dy+i*dy*(1.0+2.0*PHASESPACEAXISMARKEROFFSET)/NGRID);
            glVertex2f(xlim[0]-PHASESPACEAXISMARKEROFFSET*dx+i*dx*(1.0+2.0*PHASESPACEAXISMARKEROFFSET)/NGRID,ymid-dy*PHASESPACEAXISMARKERSIZE);
            glVertex2f(xlim[0]-PHASESPACEAXISMARKEROFFSET*dx+i*dx*(1.0+2.0*PHASESPACEAXISMARKEROFFSET)/NGRID,ymid+dy*PHASESPACEAXISMARKERSIZE);
        }
    glEnd();

    /* Draw axes names */

    glRasterPos2f(xlim[1]+dx*(PHASESPACEOFFSET+PHASESPACEAXISNAMEOFFSET),ymid);
    i=0;
    while(xname[i]!=(char)NULL) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, xname[i++]);
    }
    glRasterPos2f(xmid,ylim[1]+dy*(PHASESPACEOFFSET+PHASESPACEAXISNAMEOFFSET));
    i=0;
    while(yname[i]!=(char)NULL) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, yname[i++]);
    }

    /* Draw markers positions */

    for(i=0;i<=NPHASESPACEGRID;i=i+2) {
        glRasterPos2f(xmid+dx*PHASESPACEYAXISVALUEOFFSET,ylim[0]-PHASESPACEAXISMARKEROFFSET*dy+i*dy*(1.0+2.0*PHASESPACEAXISMARKEROFFSET)/NGRID);
        sprintf(valueString,"%.1f",ylim[0]-PHASESPACEAXISMARKEROFFSET*dy+i*dy*(1.0+2.0*PHASESPACEAXISMARKEROFFSET)/NGRID);
        j=0;
        while(valueString[j]!=(char)NULL) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, valueString[j++]);
        }
        glRasterPos2f(xlim[0]-PHASESPACEAXISMARKEROFFSET*dx+i*dx*(1.0+2.0*PHASESPACEAXISMARKEROFFSET)/NGRID,ymid-dy*PHASESPACEXAXISVALUEOFFSET);
        sprintf(valueString,"%.1f",xlim[0]-PHASESPACEAXISMARKEROFFSET*dx+i*dx*(1.0+2.0*PHASESPACEAXISMARKEROFFSET)/NGRID);
        j=0;
        while(valueString[j]!=(char)NULL) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, valueString[j++]);
        }
    }
    
    return;

}

/* glPrintString */

void glPrintString(char *string, float x, float y, float z) {

    int i;

    glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 0);
	glMatrixMode(GL_MODELVIEW);

	glPushMatrix();
	glLoadIdentity();
    glRasterPos3f(x, y, z);
	for(i=0;string[i]!='\0';i++) {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, string[i]);
	}
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);

    return;

}

/* glArrow */

void glArrow(GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2, GLfloat D, GLfloat *qaColor)
{

    float x=x2-x1;
    float y=y2-y1;
    float z=z2-z1;
    float L=sqrt(x*x+y*y+z*z);
    
    GLUquadricObj *quadObj;
    
    glPushMatrix();
    
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, qaColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaColor);
    
    glTranslated(x1,y1,z1);
    
    if((x!=0.0)||(y!=0.0)) {
        glRotatef(atan2(y,x)/RADIANSPERDEGREE,0.0,0.0,1.0);
        glRotatef(atan2(sqrt(x*x+y*y),z)/RADIANSPERDEGREE,0.0,1.0,0.0);
    } else if (z<0) {
        glRotatef(180.0,1.0,0.0,0.0);
    }

    glTranslatef(0.0,0.0,L-4.0*D);

    quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);
    gluCylinder(quadObj, 2.0*D, 0.0, 4.0*D, 32, 1);
    gluDeleteQuadric(quadObj);

    quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);
    gluDisk(quadObj, 0.0, 2.0*D, 32, 1);
    gluDeleteQuadric(quadObj);
    
    glTranslatef(0.0,0.0,-L+4.0*D);

    quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);
    gluCylinder(quadObj, D, D, L-4.0*D, 32, 1);
    gluDeleteQuadric(quadObj);

    quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);
    gluDisk(quadObj, 0.0, D, 32, 1);
    gluDeleteQuadric(quadObj);

    glPopMatrix();

    return;

}

/* configureWindow */

void configureWindow(void) {

    int menu;

    /* Configure window */

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

    /* Register callbacks */

    glutIgnoreKeyRepeat(1);
    glutKeyboardFunc(normalKey);
    glutKeyboardUpFunc(normalKeyRelease);
    glutSpecialFunc(specialKey);
	glutSpecialUpFunc(specialKeyRelease);

    /* Create menu */

	menu=glutCreateMenu(processMenuEvent);
	glutAddMenuEntry("Toggle Rotation",TOGGLEROTATION);
    glutAddMenuEntry("Toggle Tracers",TOGGLETRACERS);
    glutAddMenuEntry("Toggle 2D Path",TOGGLE2DPATH);
    glutAddMenuEntry("Toggle 3D Path",TOGGLE3DPATH);
    glutAddMenuEntry("Toggle Momentum",TOGGLEMOMENTUM);
    glutAddMenuEntry("Toggle Phase Space",TOGGLEPHASESPACE);
    glutMenuStatusFunc(&processMenuStatus);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

    return;
    
}

/* setProjection */

void setProjection(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0,0,w,h);
    if(w<=h)
        glOrtho(-2.0,2.0,-2.0*h/w,2.0*h/w,-10.0,100.0);
    else
        glOrtho(-2.0*w/h,2.0*w/h,-2.0,2.0,-10.0,100.0);
    //gluPerspective(120,(float)h/(float)w,1,100);
    glMatrixMode(GL_MODELVIEW);
    return;
}

/* resize */

void resize(int w, int h)
{

    /* Resize mainWindow */

    glutSetWindow(mainWindow);
    glutReshapeWindow(w, h);

    /* Resize subWindow0 */
    
	glutSetWindow(subWindow[0]);
    glutPositionWindow(SUBWINDOWBORDER, SUBWINDOWBORDER);
    if(!phaseSpaceEnabled) {
        glutReshapeWindow(w-2.0*SUBWINDOWBORDER, h-2.0*SUBWINDOWBORDER);
        setProjection(w-2.0*SUBWINDOWBORDER, h-2.0*SUBWINDOWBORDER);
    }
    else {
        glutReshapeWindow(w-2.0*SUBWINDOWBORDER, h/2.0-3.0*SUBWINDOWBORDER/2.0);
	    setProjection(w-2.0*SUBWINDOWBORDER, h/2.0-3.0*SUBWINDOWBORDER/2.0);
    }

    if(phaseSpaceEnabled) {

        /* Resize subWindow1 */

        glutSetWindow(subWindow[1]);
        glutPositionWindow(SUBWINDOWBORDER, h/2.0+SUBWINDOWBORDER/2.0);
        glutReshapeWindow(w/2.0-3.0*SUBWINDOWBORDER/2.0, h/2.0-3.0*SUBWINDOWBORDER/2.0);
        setProjection(w/2.0-3.0*SUBWINDOWBORDER/2.0, h/2.0-3.0*SUBWINDOWBORDER/2.0);

        /* Resize subWindow2 */

        glutSetWindow(subWindow[2]);
        glutPositionWindow(w/2.0+SUBWINDOWBORDER/2.0, h/2.0+SUBWINDOWBORDER/2.0);
        glutReshapeWindow(w/2.0-3.0*SUBWINDOWBORDER/2.0, h/2.0-3.0*SUBWINDOWBORDER/2.0);
        setProjection(w/2.0-3.0*SUBWINDOWBORDER/2.0, h/2.0-3.0*SUBWINDOWBORDER/2.0);

    }

    return;

}

/* display */

void display(void)
{
    glutSetWindow(mainWindow);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0.11, 0.12, 0.13, 0.0);
    glutSwapBuffers();
    display0();
    if(phaseSpaceEnabled) {
        display1();
        display2();
    }
    return;
}

/* display0 */

void display0(void)
{

    int i;
    float xPen, yPen, zPen;
    float pxPen, pyPen, pzPen;
    #if SHOWFPS||SHOWTIME||SHOWENERGY
    char status[128]="";
    #endif
    #if SHOWFPS
    static int frameNumber;
    static long time, timebase;
    static float currentFPS;
    #endif
    #if SHOWENERGY
    float energy;
    #endif

    /* Set window */

    glutSetWindow(subWindow[0]);

    /* Compute pendulum position */

    xPen=pendulum.r*sin(pendulum.theta)*cos(pendulum.fi);
    yPen=pendulum.r*sin(pendulum.theta)*sin(pendulum.fi);
    zPen=pendulum.r*cos(pendulum.theta);

    /* Rotate the camera */

    if(rotationEnabled) {
        fiCam+=angleStepCamPassive;
    }
    /*
    if(deltaRCam) {
        rCam+=deltaRCam;
        glutSetWindow(mainWindow);
        glutPostRedisplay();
    }
    */
    if(deltaThetaCam) {
        thetaCam+=deltaThetaCam;
        glutSetWindow(mainWindow);
        glutPostRedisplay();
        glutSetWindow(subWindow[0]);
    }
    if(deltaFiCam) {
        fiCam+=deltaFiCam;
        glutSetWindow(mainWindow);
        glutPostRedisplay();
        glutSetWindow(subWindow[0]);
    }

    /* Clear buffer */

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

    /* Set the camera */

	gluLookAt(rCam*sin(thetaCam)*cos(fiCam),rCam*sin(thetaCam)*sin(fiCam),rCam*cos(thetaCam),0.0,0.0,0.0,0.0,0.0,1.0);

    /* Set shade model */

    glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);

    /* Set the lighting */

    glLightfv(GL_LIGHT0,GL_POSITION,light_position);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

    /* Set material specular, shininess, ambient and diffuse */

    glMaterialfv(GL_FRONT,GL_SPECULAR,mat_specular);
	glMaterialfv(GL_FRONT,GL_SHININESS,mat_shininess);
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,qaWhite);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaOffWhite);

    #if SHOWFPS

    /* Compute FPS */

	frameNumber++;
	time=glutGet(GLUT_ELAPSED_TIME);
	if(time-timebase>1000.0) {
        currentFPS=frameNumber*1000.0/(time-timebase);
		timebase=time;
		frameNumber=0;
	}

    #endif

    #if SHOWENERGY

    /* Compute energy */

    energy=computeEnergy();

    #endif

    /* Create status string */

    #if SHOWFPS
        sprintf(status+strlen(status), "FPS:%4.2f ", currentFPS);
    #endif
    #if SHOWTIME
        sprintf(status+strlen(status), "[ t=%f ] ", t);
    #endif
    #if SHOWENERGY
        sprintf(status+strlen(status), "[ E=%f dE/E0=%1.4e ]", energy, (energy-initialEnergy)/initialEnergy);
    #endif

    /* Print status */
    
    #if SHOWFPS||SHOWTIME||SHOWENERGY
        glPrintString(status, 5.0, 15.0, 0.0);
    #endif

    /* Draw pendulum */

    glPushMatrix();
	glTranslatef(xPen,yPen,zPen);
	glutSolidSphere(0.10,50,50);
    glPopMatrix();

    /* Draw wire */

    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaBrown);
    glBegin(GL_LINE_STRIP);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(xPen,yPen,zPen);
    glEnd();

    /* Draw grid */
    
    glBegin(GL_LINES);
        for(i=0;i<=NGRID;i++) {
            if(i==0) {
                glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaOffWhite);
            }
            else {
                glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaBlack);
            }
            glVertex3f((i-(float)NGRID/2.0)*BOXLEN/(float)NGRID,-BOXEDGE,-BOXEDGE);
            glVertex3f((i-(float)NGRID/2.0)*BOXLEN/(float)NGRID,BOXEDGE,-BOXEDGE);
            glVertex3f(-BOXEDGE,(i-(float)NGRID/2.0)*BOXLEN/(float)NGRID,-BOXEDGE);
            glVertex3f(BOXEDGE,(i-(float)NGRID/2.0)*BOXLEN/(float)NGRID,-BOXEDGE);
        };
    glEnd();

    /* Draw cube */

    glBegin(GL_LINES);

        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaOffWhite);
        glVertex3f(-BOXEDGE,-BOXEDGE,BOXEDGE);
        glVertex3f(-BOXEDGE,-BOXEDGE,-BOXEDGE);
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaBlack);

        glVertex3f(-BOXEDGE,BOXEDGE,BOXEDGE);
        glVertex3f(-BOXEDGE,BOXEDGE,-BOXEDGE);
        glVertex3f(BOXEDGE,BOXEDGE,BOXEDGE);
        glVertex3f(BOXEDGE,BOXEDGE,-BOXEDGE);
        glVertex3f(BOXEDGE,-BOXEDGE,BOXEDGE);
        glVertex3f(BOXEDGE,-BOXEDGE,-BOXEDGE);

        glVertex3f(-BOXEDGE,BOXEDGE,BOXEDGE);
        glVertex3f(-BOXEDGE,-BOXEDGE,BOXEDGE);
        glVertex3f(BOXEDGE,BOXEDGE,BOXEDGE);
        glVertex3f(-BOXEDGE,BOXEDGE,BOXEDGE);
        glVertex3f(BOXEDGE,BOXEDGE,BOXEDGE);
        glVertex3f(BOXEDGE,-BOXEDGE,BOXEDGE);
        glVertex3f(BOXEDGE,-BOXEDGE,BOXEDGE);
        glVertex3f(-BOXEDGE,-BOXEDGE,BOXEDGE);

    glEnd();

    /* Draw axes names */

    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaOffWhite);
    glRasterPos3f((BOXEDGE+0.2*BOXEDGE), -BOXEDGE, -BOXEDGE);
    if(!phaseSpaceEnabled) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'x');
    }
    else {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, 'x');
    }
    glRasterPos3f(-BOXEDGE, (BOXEDGE+0.2*BOXEDGE), -BOXEDGE);
    if(!phaseSpaceEnabled) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'y');
    }
    else {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, 'y');
    }
    glRasterPos3f(-BOXEDGE, -BOXEDGE, (BOXEDGE+0.2*BOXEDGE));
    if(!phaseSpaceEnabled) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'z');
    }
    else {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, 'z');
    }
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaBlack);

    if(tracersEnabled) {

        glBegin(GL_LINES);

            glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaOffRed);

            /* Draw xy-tracer */

            glVertex3f(xPen,yPen,zPen);
            glVertex3f(xPen,yPen,-BOXEDGE);
            
            /* Draw xz-tracer */

            glVertex3f(xPen,yPen,zPen);
            glVertex3f(xPen,-BOXEDGE,zPen);

            /* Draw yz-tracer */

            glVertex3f(xPen,yPen,zPen);
            glVertex3f(-BOXEDGE,yPen,zPen);

        glEnd();

    }

    if(path2DEnabled) {

        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaOffBlue);

        /* Draw xy-plane path */

        glBegin(GL_LINE_STRIP);
            for(i=0;i<pathCurrent;i++) {
                glVertex3f(x[i],y[i],-BOXEDGE);
            }
        glEnd();

        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaOffGreen);

        /* Draw xz-plane path */

        glBegin(GL_LINE_STRIP);
            for(i=0;i<pathCurrent;i++) {
                glVertex3f(x[i],-BOXEDGE,z[i]);
            }
        glEnd();

        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaOffYellow);

        /* Draw yz-plane path */

        glBegin(GL_LINE_STRIP);
            for(i=0;i<pathCurrent;i++) {
                glVertex3f(-BOXEDGE,y[i],z[i]);
            }
        glEnd();

    }

    if(path3DEnabled) {

        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, qaOffWhite);

        /* Draw 3D path */

        glBegin(GL_LINE_STRIP);
            for(i=0;i<pathCurrent;i++) {
                glVertex3f(x[i],y[i],z[i]);
            }
        glEnd();

    }

    if(momentumEnabled) {

        /* Compute pendulum momentum */

        pxPen=pendulum.pr*sin(pendulum.theta)*cos(pendulum.fi)+pendulum.ptheta*cos(pendulum.theta)*cos(pendulum.fi)-pendulum.pfi*sin(pendulum.fi);
        pyPen=pendulum.pr*sin(pendulum.theta)*sin(pendulum.fi)+pendulum.ptheta*cos(pendulum.theta)*sin(pendulum.fi)+pendulum.pfi*cos(pendulum.fi);
        pzPen=pendulum.pr*cos(pendulum.theta)-pendulum.ptheta*sin(pendulum.theta);

        /* Draw momentum arrows */

        glPushMatrix();
        glTranslatef(xPen,yPen,zPen);
        glArrow(0.0,0.0,0.0,ARROWSCALE*pxPen,ARROWSCALE*pyPen,ARROWSCALE*pzPen,ARROWDIAMETER,qaOffWhite);
        glArrow(0.0,0.0,0.0,ARROWSCALE*pxPen,0.0,0.0,ARROWDIAMETER,qaOffRed);
        glArrow(0.0,0.0,0.0,0.0,ARROWSCALE*pyPen,0.0,ARROWDIAMETER,qaOffGreen);
        glArrow(0.0,0.0,0.0,0.0,0.0,ARROWSCALE*pzPen,ARROWDIAMETER,qaOffBlue);
        glPopMatrix();
        
    }

    /* Swap buffers */

	glFlush();
	glutSwapBuffers();

    return;
    
}

/* display1 */

void display1(void)
{

    int i;

    /* Set window */

    glutSetWindow(subWindow[1]);

    /* Clear buffer */

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

    /* Set plot dimensions */

	gluOrtho2D(thetalim[0],thetalim[1],pthetalim[0],pthetalim[1]);

    /* Draw Axes */

    drawPhaseSpaceAxes(thetalim,pthetalim,"theta","ptheta");

    /* Mark the latest point */

    glMarkPoint(theta[phaseSpaceCurrent-1],ptheta[phaseSpaceCurrent-1]);

    /* Draw ptheta(theta) */

    glColor3f(0.6,0.6,0.6);
    glBegin(GL_LINE_STRIP);
        for(i=0;i<phaseSpaceCurrent;i++) {
            glVertex2f(theta[i],ptheta[i]);
        }
    glEnd();

    /* Swap buffers */

	glFlush();
	glutSwapBuffers();

    return;
    
}

/* display2 */

void display2(void)
{

    int i;

    /* Set window */

    glutSetWindow(subWindow[2]);

    /* Clear buffer */

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

    /* Set plot dimensions */

	gluOrtho2D(filim[0],filim[1],pfilim[0],pfilim[1]);

    /* Draw Axes */

    drawPhaseSpaceAxes(filim,pfilim,"fi","pfi");

    /* Mark the latest point */

    glMarkPoint(fi[phaseSpaceCurrent-1],pfi[phaseSpaceCurrent-1]);

    /* Draw pfi(fi) */

    glColor3f(0.6,0.6,0.6);
    glBegin(GL_LINE_STRIP);
        for(i=0;i<phaseSpaceCurrent;i++) {
            glVertex2f(fi[i],pfi[i]);
        }
    glEnd();

    /* Swap buffers */

	glFlush();
	glutSwapBuffers();

    return;
    
}

/* createPhaseSpaceSubWindows */

void createPhaseSpaceSubWindows(void) {

    /* Create subWindow1 */

    subWindow[1]=glutCreateSubWindow(mainWindow, SUBWINDOWBORDER, WINDOWHEIGHT/2.0+SUBWINDOWBORDER/2.0, WINDOWWIDTH/2.0-3.0*SUBWINDOWBORDER/2.0, WINDOWHEIGHT/2.0-3.0*SUBWINDOWBORDER/2.0);
    glutDisplayFunc(display1);
    configureWindow();

    /* Create subWindow2 */

    subWindow[2]=glutCreateSubWindow(mainWindow, WINDOWWIDTH/2.0+SUBWINDOWBORDER/2.0, WINDOWHEIGHT/2.0+SUBWINDOWBORDER/2.0, WINDOWWIDTH/2.0-3.0*SUBWINDOWBORDER/2.0, WINDOWHEIGHT/2.0-3.0*SUBWINDOWBORDER/2.0);
    glutDisplayFunc(display2);
    configureWindow();

    return;

}

/* main */

int main(int argc, char **argv)
{

    /* Initialize */

    printUsage();
    printStatus(-1);
    setInitialConditions();
    initialEnergy=computeEnergy();

    /* Initialize GLUT */

    glutInit(&argc, argv);
    glutInitWindowPosition(-1,-1);
    glutInitWindowSize(WINDOWWIDTH, WINDOWHEIGHT);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    /* Create main window */

    mainWindow=glutCreateWindow("glPendulum");
    glutDisplayFunc(display);
	glutReshapeFunc(resize);
    configureWindow();

    /* Create subWindow0 */

    if(!phaseSpaceEnabled) {
        subWindow[0]=glutCreateSubWindow(mainWindow, SUBWINDOWBORDER, SUBWINDOWBORDER, WINDOWWIDTH-2.0*SUBWINDOWBORDER, WINDOWHEIGHT-2.0*SUBWINDOWBORDER);
    }
	else {
        subWindow[0]=glutCreateSubWindow(mainWindow, SUBWINDOWBORDER, SUBWINDOWBORDER, WINDOWWIDTH-2.0*SUBWINDOWBORDER, WINDOWHEIGHT/2.0-3.0*SUBWINDOWBORDER/2.0);
    }
	glutDisplayFunc(display0);
    configureWindow();

    /* Create secondary subWindows */

    if(phaseSpaceEnabled) {
        createPhaseSpaceSubWindows();
    }

    /* Set display timer */

    glutTimerFunc(1000.0/FPS, redisplayTimer, 0);

    /* Integrate */

    printStatus(0);
    if(integrate) {
        pthread_create(&integrateThreadID, NULL, integrateThread, NULL);
    }

    /* Main GLUT loop */

    glutMainLoop();

    return 1;

}