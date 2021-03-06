//DEL VIERNES QUE VIENE AL OTRO ESTAMOS A 26 VIERNES FEBRERO

//Cargador de modelos usando trianglestrip

#define USING_SERIAL 0 //0 means we aren't using serial, lol XD


#define mensajeError(cadena) MessageBoxA(NULL,cadena,"Mensaje",MB_OK)

#include "Main.h"
#include "3ds.h"
#include <math.h>

GLuint	base;				// Base Display List For The Font Set

bool bShowData=false;

bool bMuscleTension=false;

int iRenderMode=1;	//1	Normal
					//2	Wireframe

unsigned char dato1=1,dato2=0,identificador=0;
char mandacaracter='8';

//Serial Port Handle
//HANDLE hSerial;

bool bBanderaDireccion=false;

//Variables para vista
float anguloOjo;

//Variables de C?maraRotacional
float distancia=10.0f, alturaCamara=5, alturaObjetivo, angulo;

//Variables auxiliares de la pr?tesis
float posicionadorX=0,posicionadorY=0,posicionadorZ=0;
float rotadorX=0,rotadorY=0,rotadorZ=0;
float rotadorX2=0,rotadorY2=0,rotadorZ2=0;
float rotadorX3=0,rotadorY3=0,rotadorZ3=0;
float rotadorX4=0,rotadorY4=0,rotadorZ4=0;

//Variables de la pr?tesis

float AngDistalIndice;
float AngDistalAnular;
float AngDistalMedio;
float AngDistalPulgar;
float AngDistalMenique;

float AngMedialIndice;
float AngMedialAnular;
float AngMedialMedio;
float AngMetacarpianoPulgar;
float AngMedialMenique;

float AngProximalIndice;
float AngProximalAnular;
float AngProximalMedio;
float AngProximalPulgar;
float AngProximalMenique;

float dimension1=15;
float dimension2=dimension1*2/4;

bool	keys[256];			// Arreglo para el manejo de teclado
bool	active=true;		// Bandera de ventana activa

int glWidth;
int glHeight;

//Apuntador para primitivas de cuadricas
GLUquadricObj	*e;

#define GL_PI 3.141593


//Contenedores de texturas de cada modelo
CTga textureModel1[10];
CTga textureModel2[10];
CTga textureModel3[10];
CTga textureModel4[10];
CTga textureModel5[10];
CTga textureModel6[10];
CTga textureModel7[10];

//Contenedor de texturas para el escenario
CTga textura[16];

//Objeto que da acceso a las funciones del cargador 3ds
CLoad3DS g_Load3ds;

//Instancias de la estructura que almacenaran los datos de cada modelo
t3DModel g_3DModelDistal[5];
t3DModel g_3DModelMedial[5];
t3DModel g_3DModelProximal[5];
t3DModel g_3DModelDorso;
t3DModel g_3DModelPalma;

//Objeto para acceder a las variables de control del personaje
paramObjCam player1;

//Objeto para acceder a la selecci?n de materiales

GLuint grupos;

//Variables para iluminacion
GLfloat LightPos[] = {-100.0f, 40.0f, 50.0f, 1.0f};		// Posici?n de la luz
GLfloat LightAmb[] = { 0.9f,  0.9f, 0.9f, 1.0f};		// Valores de la componente ambiente
GLfloat LightDif[] = { 0.9f,  0.9f, 0.9f, 1.0f};		// Valores de la componente difusa
GLfloat LightSpc[] = { 0.6f,  0.6f, 0.6f, 1.0f};		// Valores de la componente especular
CVector lightPosition;

//Variables para animaci?n de texturas
float aText1;

GLvoid glPrint(const char *fmt, ...);					// Custom GL "Print" Routine

void DibujaEjes()
{
	glLineWidth(1.0);
	glBegin(GL_LINES);
		//Eje X
		glColor3f(1.0f,1.0f,0.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f( 90.0f, 0.0f, 0.0f);

		//Eje Y
		glColor3f(0.0f,1.0f,1.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f,  90.0f, 0.0f);

		//Eje Z
		glColor3f(1.0f,0.0f,1.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, 90.0f);


	glEnd();
	glLineWidth(1.0);
	glPointSize(5.0f);

	glBegin(GL_POINTS);
		//"Flecha" eje X
		glColor3f(1.0f,0.0f,0.0f);
		glVertex3f( 90.0f, 0.0f, 0.0f);

		//"Flecha" eje Y
		glColor3f(0.0f,1.0f,0.0f);
		glVertex3f(0.0f,  90.0f, 0.0f);

		//"Flecha" eje Z
		glColor3f(0.0f,0.0f,1.0f);
		glVertex3f(0.0f, 0.0f,  90.0f);
	glEnd();

	glPointSize(1.0f);

	glColor3f(1.0f,1.0f,1.0f);		//Para que no afecte a algo posterior
}

char str0[30]="Nickel";

//Primero al origen
//Luego a la pieza

float limiteinferior,limitesuperior;

float CambiaRango(unsigned char data,float liminf,float limsup)
{
	return liminf+(limsup-liminf)*(float)data/255;
}


void DibujaProtesis(short dat1,short dat2,short iden)
{
	float angData1;
	float angData2;

#if(USING_SERIAL==1)
	angData1=CambiaRango(dat1,-15,70);
	angData2=CambiaRango(dat2,-7,35);
	
	rotadorX=angData1;
	//
#endif
	rotadorX2=rotadorX/2.0f;
	rotadorX3=rotadorX2/2.0f;
	rotadorX4=rotadorX3/2.0f;
	
	glPushMatrix();

	g_Load3ds.Render3DSFile(&g_3DModelDorso, textureModel4, iRenderMode);
	g_Load3ds.Render3DSFile(&g_3DModelPalma, textureModel5, iRenderMode);

	// Indice
	glPushMatrix();
		//glTranslatef(posicionadorX,posicionadorY,posicionadorZ);
		//glRotatef(rotadorY,0,1,0);
		glTranslatef(2.0f,0.26f,-0.52f);
		glRotatef(91.5f,0,1,0);
		glRotatef(rotadorX,1,0,0);
		g_Load3ds.Render3DSFile(&g_3DModelProximal[2], textureModel1, iRenderMode);
		glPushMatrix();
			glTranslatef(0.0f,0.0f,1.81f);
			glRotatef(rotadorX2,1,0,0);
			g_Load3ds.Render3DSFile(&g_3DModelMedial[2], textureModel1, iRenderMode);
			glPushMatrix();
				glTranslatef(0.0f,0.0f,1.02f);
				glRotatef(rotadorX3,1,0,0);
				g_Load3ds.Render3DSFile(&g_3DModelDistal[2], textureModel1, iRenderMode);
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();
	
	// Medial
	glPushMatrix();
		glTranslatef(1.82f,0.25f,-1.46f);
		glRotatef(97.5f,0,1,0);
		glRotatef(rotadorX,1,0,0);
		g_Load3ds.Render3DSFile(&g_3DModelProximal[1], textureModel1, iRenderMode);
		glPushMatrix();
			glTranslatef(0.0f,0.0f,1.61f);
			glRotatef(rotadorX2,1,0,0);
			g_Load3ds.Render3DSFile(&g_3DModelMedial[1], textureModel1, iRenderMode);
			glPushMatrix();
				glTranslatef(0.0f,0.0f,0.96f);
				glRotatef(rotadorX3,1,0,0);
				g_Load3ds.Render3DSFile(&g_3DModelDistal[1], textureModel1, iRenderMode);
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();

	// Anular
	glPushMatrix();
		glTranslatef(1.89f,0.25f,0.52f);
		glRotatef(85.5f,0,1,0);
		glRotatef(rotadorX,1,0,0);
		g_Load3ds.Render3DSFile(&g_3DModelProximal[3], textureModel1, iRenderMode);
		glPushMatrix();
			glTranslatef(0.0f,0.0f,1.7f);
			glRotatef(rotadorX2,1,0,0);
			g_Load3ds.Render3DSFile(&g_3DModelMedial[3], textureModel1, iRenderMode);
			glPushMatrix();
				glTranslatef(0.0f,0.0f,0.99f);
				glRotatef(rotadorX3,1,0,0);
				g_Load3ds.Render3DSFile(&g_3DModelDistal[1], textureModel1, iRenderMode);
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();

	// Me?ique
	glPushMatrix();
		glTranslatef(1.64f,0.24f,1.32f);
		glRotatef(84.0f,0,1,0);
		glRotatef(rotadorX,1,0,0);
		g_Load3ds.Render3DSFile(&g_3DModelProximal[4], textureModel1, iRenderMode);
		glPushMatrix();
			glTranslatef(0.0f,0.0f,1.34f);
			glRotatef(rotadorX2,1,0,0);
			g_Load3ds.Render3DSFile(&g_3DModelMedial[4], textureModel1, iRenderMode);
			glPushMatrix();
				glTranslatef(0.0f,0.0f,0.79f);
				glRotatef(rotadorX3,1,0,0);
				g_Load3ds.Render3DSFile(&g_3DModelDistal[4], textureModel1, iRenderMode);
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();

	// Pulgar
	glPushMatrix();
		glTranslatef(-1.53f,-0.13f,-1.03f);
		glRotatef(-rotadorX3-26,1,0,0);
		g_Load3ds.Render3DSFile(&g_3DModelProximal[0], textureModel1, iRenderMode);
		glPushMatrix();
			glTranslatef(1.28f,0.0f,-2.78f);
			glRotatef(-rotadorX3,0,1,0);
			g_Load3ds.Render3DSFile(&g_3DModelMedial[0], textureModel1, iRenderMode);
			glPushMatrix();
				glTranslatef(1.28f+0.06f,0.0f,0.0f);
				glRotatef(-rotadorX3,0,1,0);
				g_Load3ds.Render3DSFile(&g_3DModelDistal[0], textureModel1, iRenderMode);
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();


	//glPushMatrix();
		//glTranslatef(3.89f,-0.60f,0.58f);
		//glRotatef(98.7,0,1,0);
		//glTranslatef(0.41f,0.88f,-2.14f);
		//glRotatef(rotadorX,1,0,0);
		//glTranslatef(-0.41f,-0.88f,2.14f);
		//g_Load3ds.Render3DSFile(&g_3DModelProximal[1], textureModel1, 1);
		//glPushMatrix();
		//	glTranslatef(0.0f,-0.56f,0.70);
		//	glTranslatef(0.4f,1.42f,-1.23f);
		//	//glRotatef(rotadorX/2,1,0,0);
		//	glRotatef(rotadorX2,1,0,0);
		//	glTranslatef(-0.4f,-1.42f,1.23f);
		//	g_Load3ds.Render3DSFile(&g_3DModelMedial[1], textureModel1, 1);
		//	glPushMatrix();
		//		glTranslatef(0.0f,0.12f,1.0f);
		//		glTranslatef(0.38f,1.3f,-1.31f);
		//		glRotatef(rotadorX2/2,1,0,0);
		//		//glRotatef(rotadorX3,1,0,0);
		//		glTranslatef(-0.38f,-1.3f,1.31f);
		//		g_Load3ds.Render3DSFile(&g_3DModelDistal[1], textureModel1, 1);
		//	glPopMatrix();
		//glPopMatrix();
	//glPopMatrix();
	
	glPopMatrix();
}

//LRESULT	CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);	// Declaracion de WndProc (Procedimiento de ventana)

GLvoid ReDimensionaEscenaGL(GLsizei width, GLsizei height)	// Redimensiona e inicializa la ventana
{
	if (height==0)							// Para que no se presente una division por cero
	{
		height=1;							// la altura se iguala a 1
	}

	glViewport(0,0,width,height);					// Resetea el puerto de vista

	glMatrixMode(GL_PROJECTION);					// Selecciona la Matriz de Proyeccion
	glLoadIdentity();								// Resetea la Matriz de Proyeccion

	// Calcula el radio de aspecto o proporcion de medidas de la ventana
	gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,2000.0f);
	
	glMatrixMode(GL_MODELVIEW);							// Selecciona la Matriz de Vista de Modelo
	glLoadIdentity();									// Resetea la Matriz de Vista de Modelo

	glWidth=width;
	glHeight=height;
}

int CargaModelos()
{
	if(!g_Load3ds.Load3DSFile("Modelos/Distal_Pulgar.3ds", &g_3DModelDistal[0], textureModel1))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Distal_Indice.3ds", &g_3DModelDistal[1], textureModel2))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Distal_Medio.3ds", &g_3DModelDistal[2], textureModel3))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Distal_Anular.3ds", &g_3DModelDistal[3], textureModel4))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Distal_Menique.3ds", &g_3DModelDistal[4], textureModel5))
		return 0;

	if(!g_Load3ds.Load3DSFile("Modelos/Proximal_Pulgar.3ds", &g_3DModelMedial[0], textureModel1))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Medial_Indice.3ds", &g_3DModelMedial[1], textureModel2))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Medial_Medio.3ds", &g_3DModelMedial[2], textureModel3))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Medial_Anular.3ds", &g_3DModelMedial[3], textureModel4))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Medial_Menique.3ds", &g_3DModelMedial[4], textureModel5))
		return 0;

	if(!g_Load3ds.Load3DSFile("Modelos/Metacarpiano_Pulgar.3ds", &g_3DModelProximal[0], textureModel1))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Proximal_Indice.3ds", &g_3DModelProximal[1], textureModel2))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Proximal_Medio.3ds", &g_3DModelProximal[2], textureModel3))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Proximal_Anular.3ds", &g_3DModelProximal[3], textureModel4))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Proximal_Menique.3ds", &g_3DModelProximal[4], textureModel5))
		return 0;
	
	if(!g_Load3ds.Load3DSFile("Modelos/Dorso.3ds", &g_3DModelDorso, textureModel4))
		return 0;
	if(!g_Load3ds.Load3DSFile("Modelos/Palma.3ds", &g_3DModelPalma, textureModel5))
		return 0;

	return true;
}

void DescargaModelos()
{
	g_Load3ds.UnLoad3DSFile(&g_3DModelDistal[0], textureModel1);
	g_Load3ds.UnLoad3DSFile(&g_3DModelDistal[1], textureModel2);
	g_Load3ds.UnLoad3DSFile(&g_3DModelDistal[2], textureModel3);
	g_Load3ds.UnLoad3DSFile(&g_3DModelDistal[3], textureModel4);
	g_Load3ds.UnLoad3DSFile(&g_3DModelDistal[4], textureModel5);

	g_Load3ds.UnLoad3DSFile(&g_3DModelMedial[0], textureModel1);
	g_Load3ds.UnLoad3DSFile(&g_3DModelMedial[1], textureModel2);
	g_Load3ds.UnLoad3DSFile(&g_3DModelMedial[2], textureModel3);
	g_Load3ds.UnLoad3DSFile(&g_3DModelMedial[3], textureModel4);
	g_Load3ds.UnLoad3DSFile(&g_3DModelMedial[4], textureModel5);

	g_Load3ds.UnLoad3DSFile(&g_3DModelProximal[0], textureModel1);
	g_Load3ds.UnLoad3DSFile(&g_3DModelProximal[1], textureModel2);
	g_Load3ds.UnLoad3DSFile(&g_3DModelProximal[2], textureModel3);
	g_Load3ds.UnLoad3DSFile(&g_3DModelProximal[3], textureModel4);
	g_Load3ds.UnLoad3DSFile(&g_3DModelProximal[4], textureModel5);

	g_Load3ds.UnLoad3DSFile(&g_3DModelDorso, textureModel4);
	g_Load3ds.UnLoad3DSFile(&g_3DModelPalma, textureModel5);	
}

void CargaTexturas()
{
}

void DescargaTexturas()
{
}

void InicializaParametrosdeControl()
{
	//Vamos a definir un valor de un ?ngulo de apertura entre dos
	anguloOjo=1.0f*GL_PI/180.0f;	//4 grados entre ojos.
}




void InicializaVariablesProtesis()
{
	
}

float fCRed=0;

int IniGL(GLvoid)										// Aqui se configuran los parametros iniciales de OpenGL
{
	glShadeModel(GL_SMOOTH);							// Activa Smooth Shading
	glClearColor(1.0f, 0.0f, 0.0f, 0.5f);				// Fondo negro
	glClearDepth(1.0f);									// Valor para el Depth Buffer
	glEnable(GL_DEPTH_TEST);							// Activa Depth Testing
	glDepthFunc(GL_LEQUAL);								// Tipo de Depth Testing a usar
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Correccion de c?lculos de perspectiva

	glCullFace(GL_BACK);								// Configurado para eliminar caras traseras
	glEnable(GL_CULL_FACE);								// Activa eliminacion de caras ocultas

	glLightfv(GL_LIGHT0, GL_POSITION, LightPos);		// Posicion de la luz0
	glLightfv(GL_LIGHT0, GL_AMBIENT,  LightAmb);		// Componente ambiente
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  LightDif);		// Componente difusa
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpc);		// Componente especular

	glEnable(GL_LIGHT0);					// Activa luz0
	glEnable(GL_LIGHTING);					// Habilita la iluminaci?n

	CargaModelos();
	CargaTexturas();
//    initSerialPort();


	e=gluNewQuadric();
	InicializaParametrosdeControl();

	
	return true;										
}
float AngPos=PI;

void CambiaAnguloCamara(int funcion)
{
	if(funcion == 1) //Incrementa ?ngulo de la c?mara
	{
		AngPos+=1;
	}
	else if(funcion == 2) //Decrementa ?ngulo de la c?mara
	{
		AngPos-=1;
	}
}


float angu1=0,angu2=0;

void DibujaLuz(CVector l)
{
	//Dibuja una esfera que representa la fuente luminosa
	glDisable(GL_LIGHTING);				// Deshabilita iluminaci?n
	
	glPushMatrix();
		glTranslatef(l.x, l.y, l.z);		// Traslada a la posicion de la luz

		glColor3f(1.0f, 1.0f, 0.0f);		// Color amarillo
		gluSphere(e, 3.5f, 16, 8);		// Dibuja la esfera
	glPopMatrix();


	//glEnable(GL_LIGHTING);				// Habilita Iluminaci?n
	glColor3f(1.0f, 1.0f, 1.0f);
}


void RenderizaEscena(GLvoid)								// Aqui se dibuja todo lo que aparecera en la ventana
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	
	glLoadIdentity();

	if(!bBanderaDireccion)
		gluLookAt(	distancia*cos(angulo-anguloOjo), alturaCamara, distancia*sin(angulo-anguloOjo), 
					0.0f, alturaObjetivo, 0.0f, 
					0.0f, 1.0f, 0.0f);
	else
		gluLookAt(	0.0f, distancia, 0.0f, 
					0.0f, 0.0f, 0.0f, 
					0.0f, 0.0f, -1.0f);


	glDisable(GL_LIGHTING);
	DibujaEjes();
	glEnable(GL_LIGHTING);
	
	DibujaProtesis(dato1,dato2,identificador);
    glutSwapBuffers();
    glutPostRedisplay();
}

int contador=0;

void on_exit() {
	// TODO Add me back
    DescargaModelos();
    DescargaTexturas();
}
void processSpecialKeys(int key, int x, int y)
{
	switch (key) {
		case GLUT_KEY_UP : distancia -= 0.05f; break;
		case GLUT_KEY_DOWN : distancia += 0.05f; break;
		case GLUT_KEY_LEFT : angulo += 0.008f; break;
		case GLUT_KEY_RIGHT : angulo -= 0.008f; break;
		default: break;
	}
}
void processNormalKeys(unsigned char key, int x, int y)
{
	switch (key) {
		case 27: exit(0);
		case 32: bBanderaDireccion = true; break;
		case 13: bBanderaDireccion = false; break;
		//Illumination controls
		case 'z': alturaObjetivo += 0.05f; break;
		case 'x': alturaObjetivo -= 0.05f; break;
		case 'q': posicionadorX += 0.01f; break;
		case 'w': posicionadorY += 0.01f; break;
		case 'e': posicionadorZ += 0.01f; break;
		case 'a': posicionadorX -= 0.01f; break;
		case 's': posicionadorY -= 0.01f; break;
		case 'd': posicionadorZ -= 0.01f; break;
		case 'r': if(rotadorX<80.0f) {rotadorX += 1.5f; break;}
		case 't': rotadorY += 1.5f; break;
		case 'y': rotadorZ += 1.5f; break;
		case 'f': if(rotadorX>-5.0f) {rotadorX -= 1.50f; break;}
		case 'g': rotadorY -= 1.5f; break;
		case 'h': rotadorZ -= 1.5f; break;
		case 'u': rotadorX2 += 0.5f; break;
		case 'j': rotadorX2 -= 0.5f; break;
		case 'i': rotadorX3 += 0.5f; break;
		case 'k': rotadorX3 -= 0.5f; break;
		case 'c': LightPos[1]  +=  1.0f; break; //Hacia arriba
		case 'v': LightPos[1]  -=  1.0f; break; //Hacia abajo
		case 'b': LightPos[2]  +=  1.0f; break; //Hacia adelante
		case 'n': LightPos[2]  -=  1.0f; break; //Hacia atr?s
		case '1': iRenderMode=1; break;
		case '2': iRenderMode=2; break;
		case '9': bShowData=false; break;
		case '0': bShowData=true; break;
		case 'o': alturaCamara += 0.05f; break;
		case 'p': alturaCamara -= 0.05f; break;
		default: break;
	}
}

int main(int argc, char** argv) {
	// TODO Add me back
	//CargaModelos();
	//CargaTexturas();
	atexit(on_exit);

	glutInit(&argc, argv);
	//glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);
	int mainWindow = glutCreateWindow ("Prosthesis");
	IniGL();
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glutKeyboardFunc(processNormalKeys);
    glutSpecialFunc(processSpecialKeys);
	glutDisplayFunc(RenderizaEscena);
    glutIdleFunc(RenderizaEscena);
    glutReshapeFunc(ReDimensionaEscenaGL);
	// This goes at GLUT exit

	glutMainLoop();
}


//int ManejaTeclado()
//{
	//if(keys[VK_SPACE])
	//{
		//bBanderaDireccion=true;
	//}

	//if(keys[VK_RETURN])
	//{
		//bBanderaDireccion=false;
	//}

	//if(keys[VK_UP])
	//{
		//distancia-=0.05f;
		
		////ControlPersonaje(3);
	//}
	//if(keys[VK_DOWN])
	//{
		//distancia+=0.05f;

		////ControlPersonaje(4);
	//}
	//if(keys[VK_LEFT])
	//{
		//angulo+=0.008f;
		////ControlPersonaje(2);
	//}
	//if(keys[VK_RIGHT])
	//{
		//angulo-=0.008f;

		////ControlPersonaje(1);
	//}
	//if(keys[VK_SHIFT])
	//{
		//alturaCamara+=0.05f;
		////ControlPersonaje(5);
	//}
	//if(keys[VK_CONTROL])
	//{
		//alturaCamara-=0.05f;
		////ControlPersonaje(6);
	//}
	////Controles de la iluminaci?n
	//if (keys['Z'])
		//alturaObjetivo+=0.05f;
		////LightPos[0] += 1.0f; //Hacia la derecha

	//if (keys['X'])
		//alturaObjetivo-=0.05f;
		////LightPos[0] -= 1.0f; //Hacia la izquierda

	//if (keys['Q'])
		//posicionadorX+=0.01f;

	//if (keys['W'])
		//posicionadorY+=0.01f;

	//if (keys['E'])
		//posicionadorZ+=0.01f;

	
	//if (keys['A'])
		//posicionadorX-=0.01f;

	//if (keys['S'])
		//posicionadorY-=0.01f;

	//if (keys['D'])
		//posicionadorZ-=0.01f;


	//if ((keys['R']&&rotadorX<80.0f))//||(bBanderaDireccion&&rotadorX<70.0f))
		//rotadorX+=1.5f;

	//if (keys['T'])
		//rotadorY+=1.5f;

	//if (keys['Y'])
		//rotadorZ+=1.5f;




	//if (keys['F']&&rotadorX>-5.0f)//||(!bBanderaDireccion&&rotadorX>-5.0f))
		//rotadorX-=1.50f;

	//if (keys['G'])
		//rotadorY-=1.5f;

	//if (keys['H'])
		//rotadorZ-=1.5f;


	//if (keys['U'])
		//rotadorX2+=0.5f;

	//if (keys['J'])
		//rotadorX2-=0.5f;


	//if (keys['I'])
		//rotadorX3+=0.5f;

	//if (keys['K'])
		//rotadorX3-=0.5f;


	//if (keys['C'])
		//LightPos[1] += 1.0f; //Hacia arriba

	//if (keys['V'])
		//LightPos[1] -= 1.0f; //Hacia abajo

	//if (keys['B'])
		//LightPos[2] += 1.0f; //Hacia adelante

	//if (keys['N'])
		//LightPos[2] -= 1.0f; //Hacia atr?s


	//if (keys['O']&&mandacaracter<255)
		//mandacaracter+=1;

	//if (keys['L']&&mandacaracter>0)
		//mandacaracter-=1;

	
	//if (keys['1'])
		//iRenderMode=1;
	//else if (keys['2'])
		//iRenderMode=2;

	
	
	//if (keys['T'])		//Title
		//bShowData=false;
	//else if (keys['D'])		//Reset
		//bShowData=true;


	//return true;
//}


GLvoid glPrint(const char *fmt, ...)					// Custom GL "Print" Routine
{
	char		text[256];								// Holds Our String
	va_list		ap;										// Pointer To List Of Arguments

	if (fmt == NULL)									// If There's No Text
		return;											// Do Nothing

	va_start(ap, fmt);									// Parses The String For Variables
	    vsprintf(text, fmt, ap);						// And Converts Symbols To Actual Numbers
	va_end(ap);											// Results Are Stored In Text

	glPushAttrib(GL_LIST_BIT);							// Pushes The Display List Bits
	glListBase(base - 32);								// Sets The Base Character to 32
	glCallLists(strlen(text), GL_UNSIGNED_BYTE, text);	// Draws The Display List Text
	glPopAttrib();										// Pops The Display List Bits
}

