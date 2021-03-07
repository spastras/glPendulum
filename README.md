# glPendulum

![Alt Text](https://raw.githubusercontent.com/spastras/glPendulum/main/glPendulumDemo.gif)

## Description

glPendulum is a simple application I developed in the context of a postgraduate course in Mechanics. It integrates Hamilton's equations of a spherical pendulum, the length of which may change at a constant rate, and presents the results graphically using OpenGL.

It can also draw the 2D & 3D path of the motion of the pendulum as well as p_pheta(theta) and p_phi(phi) planes of phase space.

The main purpose of this application is to provide the user with a visual representation of the motion of the pendulum using one of two simple integrators.

If you have any suggestions or bug reports please don't hesitate to contact me!

## Integrators

* Euler
* Verlet

## Dependencies

* libpthread
* libm
* libGL
* libGLU
* libglut

On **Linux**, installing an implementation of OpenGL and GLUT should suffice, e.g. on the latest version of Ubuntu this can be done by executing:
```console
$ sudo apt-get install freeglut3-dev
```

On **Mac OS**, these dependecies can be satisfied by installing Xcode and Apple's Command Line Developer Tools.

## Compilation

You can compile and run glPendulum on Linux or Mac OS by executing:
```console
$ make && ./glPendulum
```

## Known issues

* Resizing or moving the GUI window might not work as expected depending on your window manager
* Initial conditions and options can only be set by editing and recompiling the sources
* The precision of this application is limited due to the use of single precision floating point variables