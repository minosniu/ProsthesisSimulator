#ifndef MAIN_H
#define MAIN_H

//#include <windows.h>		// Archivo de cabecera para Windows
#include <math.h>			// Archivo de cabecera para Funciones Matem?ticas
#include <stdio.h>			// Header File For Standard Input/Output
#include <stdlib.h>			// Header File For Standard Library
#include <fstream>
#include <vector>
#ifdef __linux__
    #include <GL/gl.h>
    #include <GL/glu.h>
    #include <GL/glut.h>
#elif __APPLE__
    #include <OpenGL/gl.h>
    #include <OpenGL/glu.h>
    #include <GLUT/glut.h>
#endif

//#include <crtdbg.h>
//#include <gl/gl.h>				// Archivo de cabecera para la libreria OpenGL32
//#include <gl/glu.h>			// Archivo de cabecera para la libreria GLu32
//#include <gl/glaux.h>			// Archivo de cabecera para la libreria Glaux
//#include <gl/glext.h>			// Archivo de cabecera para la libreria Glext
#include "Vector.h"
#include "cargadorTGA.h"

void DibujaGrupoBloques();
void DibujaGrupoArboles();
void DibujaGrupoPlantas();
void DibujaArbol();
int  ManejaTeclado();

struct paramObjCam
{
	Vector PosicionObj;	//La posici?n (x,y,z) del objeto
	Vector PosicionObjAnt;	//La posici?n anterior (x,y,z) del objeto
	Vector Direccion;		//La direcci?n en que se dirige el objeto en forma de vector=(x,y,z)
	Vector DireccionCam;//La direcci?n que define la posici?n de la c?mara respecto al personaje IZQUIERDA
	Vector PosicionCam; //La posici?n de la c?mara que sigue al objeto con origen en el ojo izquierdo
	Vector ObjetivoCam;	//El punto (x,y,z) que est? viendo la c?mara izquierda

	float VelocidadObj;		//La velocidad a la que se mueve el objeto
	float DistanciaCam;		//La distancia que la c?mara est? separada del objeto
	float AngDir;			//Se usa para llevar control del angulo para el c?lculo del vector de direcci?n
	float AngDirCam;		//Se usa para llevar control del angulo para el c?lculo de posici?n de la c?mara
	float AngObj;			//Se usa para llevar control del ?ngulo de rotaci?n para el modelo del objeto

	float CamaraPosAlt;		//Se usa para definir y cambiar si es necesario la altura de la c?mara
	float CamaraObjAlt;		//Se usa para definir y cambiar si es necesario la altura del objetivo de la c?mara

	float escalaX;
	float escalaY;
	float escalaZ;

	bool visible;
	bool caminando;
	bool saltando;
		
};

struct FRAME
{
	float Angt1;
	float Angt2;
	float Angc1;
	float Angc2;
	float Angbi1;
	float Angbi2;
	float Angbib;
	float Angbd1;
	float Angbd2;
	float Angbdb;
	float Angpizq;
	float Angpder;
	float Angpi;
	float Angpd;

	float Xtor;
	float Ytor;
	float Ztor;

	float incAngt1;
	float incAngt2;
	float incAngc1;
	float incAngc2;
	float incAngbi1;
	float incAngbi2;
	float incAngbib;
	float incAngbd1;
	float incAngbd2;
	float incAngbdb;
	float incAngpizq;
	float incAngpder;
	float incAngpi;
	float incAngpd;

	float incXtor;
	float incYtor;
	float incZtor;

};
#endif 
