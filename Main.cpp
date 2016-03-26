#define USING_SERIAL 0 //0 means we aren't using serial
#define USING_TCP_SERVER 0 //0 means we aren't the TCP Server

#define DRAW_FULL		//Draws full prosthesis
//#define DRAW_SINGLE	//Draws single joint
//#define DRAW_DOUBLE	//Draws two joints
#define DRAW_TRIPLE	//Draws three joints

#define DRAW_MODELS
//#define DRAW_CYLINDERS

#define DYNAMICS_TYPEA_ON 1

#define errorMsg(cadena) print("%s", cadena)//MessageBoxA(NULL,cadena,"Mensaje",MB_OK)

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
TGALoader textureModel1[10];
TGALoader textureModel2[10];
TGALoader textureModel3[10];
TGALoader textureModel4[10];
TGALoader textureModel5[10];
TGALoader textureModel6[10];
TGALoader textureModel7[10];

//Contenedor de texturas para el escenario
TGALoader textura[16];

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
Vector lightPosition;

//Variables para animaci?n de texturas
float aText1;

GLvoid glPrint(const char *fmt, ...);					// Custom GL "Print" Routine


// Biological Properties - Muscle	
float gMusB = 50.0f; 
float gMusKse = 136.0f;//136.0f;
float gMusKpe = 75.0f;//120.0;

float d_force(float T_0, float x1, float x2, float A)
{
    // Take state variables
    // Return derivatives
    
    float x0 = 1.0f;
    
    float Kse_x_Kpe_o_b;
    float Kse_o_b_m_one_p_Kpe_o_Kse;
    float Kse_o_b;
    float dT_0; 

    float rate_change_x = x2; // slope;
    
    
    x0 = 1.0;
    Kse_x_Kpe_o_b = gMusKse / gMusB * gMusKpe; // 204
    Kse_o_b_m_one_p_Kpe_o_Kse = gMusKse / gMusB * (1 + gMusKpe / gMusKse); // 4.22
    Kse_o_b = gMusKse / gMusB; // 2.72
        
    dT_0 = Kse_x_Kpe_o_b *(x1 - x0) + gMusKse * x2 - Kse_o_b_m_one_p_Kpe_o_Kse * T_0 + Kse_x_Kpe_o_b * A;
        
    return dT_0;
}




void Izhikevich(double *neuron_state, double *neuron_input)
{
  double const A = 0.02; // a: time scale of the recovery variable u
  double const B = 0.2; // b:sensitivity of the recovery variable u to the subthreshold fluctuations of v.
  double const C = -65.0; // c: reset value of v caused by fast high threshold (K+)
  double const D = 6.0; // d: reset of u caused by slow high threshold Na+ K+ conductances
  double const X = 5.0; // x
  double const Y = 140.0; // y

  double v = neuron_state[0];
  double u = neuron_state[1];
  double vv, uu;
  double I = neuron_input[0];
  double vol_spike = neuron_input[1];
  double DT = neuron_input[2]*100.0;

  vv = v + DT * (0.04 * v*v + X * v + Y - u + I); // neuron[0] = v;
  uu = u + DT * A * (B * v - u); // neuron[1] = u; See iZhikevich model
  
  // 
  // SetRandomSeed(0);  
  // Firing threshold randomly distributed 25.0 ~ 35.0
  double TH_RANGE = 10.0;
  double TH = 30.0 - TH_RANGE + ( 2.0 * TH_RANGE * rand() / ( RAND_MAX + 1.0 ) );
  //*exportTH = TH;

  
  
  if (vv >= TH) // if spikes    
    {
      neuron_state[0] = C;
      neuron_state[1] = uu + D;
      neuron_state[2] = vol_spike; // a flag in double that tells if the neuron is spiking
      //printf("%.3f\t\n", neuron_state[2]);

    }
  else
    {
      neuron_state[0] = vv;
      neuron_state[1] = uu;
      neuron_state[2] = 0.0;
    };
	
}

void Spindle(double *spindle_state, double *spindle_input)
{
	// ##############
	// ## KSR             [10.4649 10.4649 10.4649]
	// ## KPR             [0.1127 0.1623 0.1623]
	// ## B0DAMP          [0.0605 0.0822 0.0822]
	// ## BDAMP           [0.2356 -0.046 -0.069]
	// ## F0ACT           [0 0 0]
	// ## FACT            [0.0289 0.0636 0.0954]
	// ## XII             [1 0.7 0.7]
	// ## LPRN            [1 0.92 0.92]
	// ## GI              [20000 10000 10000]
	// ## GII             [20000 7250 7250]
	// ## ANONLINEAR      [0.3 0.3 0.3]
	// ## RLDFV           [0.46 0.46 0.46]
	// ## LSR0            [0.04 0.04 0.04]
	// ## LPR0            [0.76 0.76 0.76]
	// ## L2ND            [1 0.04 0.04]
	// ## TAO             [0.192 0.185 0.0001]
	// ## MASS            [0.0002 0.0002 0.0002]
	// ## FSAT            [1 0.5 1]

    //Declarations for Bag I
    
	double KSR = 10.4649;
	double KPR = 0.1127;
	double B0DAMP=0.0605;
	double BDAMP=0.2356;
	double F0ACT=0.0;
	double FACT=0.0289;
	double XII=1.0;
	double LPRN=1.0;
	double GI = 20000.0;
	double GII = 20000.0;
	double ANONLINEAR = 0.25;	//0.3 in original
	double RLDFV=0.46;
	double LSR0 = 0.04;
	double LPR0 = 0.76;
	double L2ND = 1.00;
	double TAO = 0.192;
	double MASS = 0.0002;
	double FSAT = 1.00;

	const double freq = 60.0;
    		
	double gd = spindle_input[0];
	double lce = spindle_input[1];
	double DT = spindle_input[2];
  
	double alpha = 0.0;
	double beta = 1.0;
	
	double dx0;
	double dx1;
	double dx2;
	//double fib;

    double mingd;
	double CSS;
    double sig;

	double x0_prev = spindle_state[0];
	double x1_prev = spindle_state[1];
	double x2_prev = spindle_state[2];
        
	double fib = spindle_state[3];

        
	double dx0_prev = spindle_state[4];
	double dx1_prev = spindle_state[5];
	double dx2_prev = spindle_state[6];

    double xx0, xx1, xx2, x0, x1, x2;
    
 
    
    //*** BAG I
    
	x0 = x0_prev + dx0_prev * DT;
	x1 = x1_prev + dx1_prev * DT;
	x2 = x2_prev + dx2_prev * DT;

	mingd = gd*gd / (gd*gd + freq*freq);
	dx0 = (mingd - x0) / 0.149; // Tao: 0.149
	dx1 = x2;
	if (x2 < 0.0)
	CSS = -1.0;
	else 
	CSS = 1.0;
    
    CSS = (fabs(x2) == 0.0) ? 0.0 : CSS;

    sig=((x1 - RLDFV) > 0.0) ? (x1 - RLDFV) : 0.0;

	//printf("%.6f\n", lce);
	// dx2 = (1 / MASS) * (KSR * lce - (KSR + KPR) * x1 - CSS * (BDAMP * x0) * (fabs(x2)) - 0.4);
	// dx2 = (1 / MASS) * (KSR * lce - (KSR + KPR) * x1 - CSS * (BDAMP * x0) * sig * sqrt(sqrt(fabs(x2))) - 0.4);
    dx2 = (1.0 / MASS) * 
        (KSR * lce - 
            (KSR + KPR) * x1 - 
            (B0DAMP + BDAMP * x0) * sig * CSS * pow(fabs(x2), ANONLINEAR) - 
            (FACT * x0) - KSR*LSR0 + KPR*LPR0 );

    xx0 = x0 + DT * (dx0 + dx0_prev)/2.0;
    xx1 = x1 + DT * (dx1 + dx1_prev)/2.0;
    xx2 = x2 + DT * (dx2 + dx2_prev)/2.0;

	fib = GI * (lce - xx1 - LSR0);
	fib = (fib >= 0.0 && fib <= 100000.0) ? fib : (fib >100000.0 ? 100000.0: 0.0);

	spindle_state[0] = xx0;
	spindle_state[1] = xx1;
	spindle_state[2] = xx2;
	spindle_state[3] = fib;
	spindle_state[4] = dx0;
	spindle_state[5] = dx1;
	spindle_state[6] = dx2;
    
    
    //Declarations for Bag II
    
    double x3_prev = spindle_state[7];
	double x4_prev = spindle_state[8];
	double x5_prev = spindle_state[9];
    
    double fib_bag2 = spindle_state[10];
    
    double dx3_prev = spindle_state[11];
	double dx4_prev = spindle_state[12];
	double dx5_prev = spindle_state[13];
    
    double gs = spindle_input[3];   //Change this in Update
	
    double dx3;
	double dx4;
	double dx5;
    
    double mings;
    double CSS_bag2;
    double sig_bag2;
        
    double xx3, xx4, xx5, x3, x4, x5;    
    //*** BAG II
    
    KSR = 10.4649;
	KPR = 0.1623;
	B0DAMP =0.0822;
	BDAMP =-0.046;
	F0ACT =0.0;
	FACT =0.0636;
	XII =0.7;
	LPRN =0.92;
	GI = 10000.0;
	GII = 7250.0;
	ANONLINEAR = 0.3;	//0.3 in original
	RLDFV=0.46;
	LSR0 = 0.04;
	LPR0 = 0.76;
	L2ND = 0.04;
	TAO = 0.185;
	MASS = 0.0002;
	FSAT = 0.50;
    
    
	x3 = x3_prev + dx3_prev * DT;
	x4 = x4_prev + dx4_prev * DT;
	x5 = x5_prev + dx5_prev * DT;
    
    mings = gs*gs / (gs*gs + freq*freq);
	
    dx3 = (mings - x3)/ 0.205;
    
    dx4 = x5;
    
    if (x5 < 0.0)
	CSS_bag2 = -1.0;
	else 
	CSS_bag2 = 1.0;
    
    sig_bag2 = x4 - RLDFV;
    sig_bag2 = (sig_bag2 > 0) ? sig_bag2 : 0.0 ;
    
    dx5 = (1.0 / MASS) * 
        (KSR * lce - 
            (KSR + KPR) * x4 -
            (B0DAMP + BDAMP * x3) * sig_bag2 * CSS_bag2 * pow(fabs(x5), ANONLINEAR) -
            (FACT * x3) -
            KSR*LSR0 + KPR*LPR0 );
    
    xx3 = x3 + DT * (dx3 + dx3_prev)/2.0;
    xx4 = x4 + DT * (dx4 + dx4_prev)/2.0;
    xx5 = x5 + DT * (dx5 + dx5_prev)/2.0;

    fib_bag2 = GI * (lce - xx4 - LSR0);
	fib_bag2 = (fib_bag2 >= 0.0 && fib_bag2 <= 100000.0) ? fib_bag2 : (fib_bag2 >100000.0 ? 100000.0: 0.0);

    spindle_state[7] = xx3;
	spindle_state[8] = xx4;
	spindle_state[9] = xx5;
	spindle_state[10] = fib_bag2;
	spindle_state[11] = dx3;
	spindle_state[12] = dx4;
	spindle_state[13] = dx5;   
    
}




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


void DrawSimulatedFingers(short dat1,short dat2,short iden)
{
	float angData1;


	glPushMatrix();
		//glTranslatef(2.0f,0.26f,-0.52f);
		//glRotatef(91.5f,0,1,0);
		glRotatef(GL_PI/4,0,0,1);
		glRotatef(rotadorX,1,0,0);
#ifdef DRAW_MODELS
		g_Load3ds.Render3DSFile(&g_3DModelProximal[2], textureModel1, iRenderMode);
#endif
#ifdef DRAW_CYLINDERS
		glScalef(0.03f,0.03f,0.03f);
		g_Load3ds.Render3DSFile(&g_3DModelDorso, textureModel1, iRenderMode);
#endif
		glPopMatrix();
	
	
}

void DrawSimulatedFingers2(short dat1,short dat2,short iden)
{
	float angData1;

	//+++ FOR NOW
	rotadorX2=rotadorX/2.0f;
	rotadorX3=rotadorX2/2.0f;
	rotadorX4=rotadorX3/2.0f;


	glPushMatrix();
		//glTranslatef(2.0f,0.26f,-0.52f);
		//glRotatef(91.5f,0,1,0);
		glRotatef(rotadorX,1,0,0);

		g_Load3ds.Render3DSFile(&g_3DModelProximal[2], textureModel1, iRenderMode);
		glPushMatrix();
			glTranslatef(0.0f,0.0f,1.81f);
			glRotatef(rotadorX2,1,0,0);
			g_Load3ds.Render3DSFile(&g_3DModelMedial[2], textureModel1, iRenderMode);
		glPopMatrix();
	glPopMatrix();
}


void DrawSimulatedFingers3(short dat1,short dat2,short iden)
{
	float angData1;

	//+++ FOR NOW
	rotadorX2=rotadorX/2.0f;
	rotadorX3=rotadorX2/2.0f;
	rotadorX4=rotadorX3/2.0f;


	glPushMatrix();
		//glTranslatef(2.0f,0.26f,-0.52f);
		//glRotatef(91.5f,0,1,0);
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

GLvoid resizeGLScene(GLsizei width, GLsizei height)	// Redimensiona e inicializa la ventana
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
	
	if(!g_Load3ds.Load3DSFile("Modelos/FingerA3.3ds", &g_3DModelDorso, textureModel4))
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

void CargaTexturas() {
}

void DescargaTexturas() {
}

void InicializaParametrosdeControl() {
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
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);				// Fondo negro
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

void DibujaLuz(Vector l)
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


void renderScene(GLvoid)								// Aqui se dibuja todo lo que aparecera en la ventana
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
	
	//DibujaProtesis(dato1,dato2,identificador);
#ifdef DRAW_SINGLE
	DrawSimulatedFingers(dato1,dato2,identificador);
#endif
#ifdef DRAW_DOUBLE
	DrawSimulatedFingers2(dato1,dato2,identificador);
#endif
#ifdef DRAW_TRIPLE
	DrawSimulatedFingers3(dato1,dato2,identificador);
#endif
}

int contador = 0;

void on_exit() {
	DescargaModelos();
	DescargaTexturas();
}

int main(int argc, char** argv) {
	CargaModelos();
	CargaTexturas();

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);
	int mainWindow = glutCreateWindow ("Prosthesis");
	IniGL();
	//glutKeyboardFunc(processNormalKeys);
	//glutSpecialFunc(processSpecialKeys);
	glutDisplayFunc(renderScene);
	glutIdleFunc(renderScene);
	glutReshapeFunc(resizeGLScene);
	// This goes at GLUT exit
	atexit(on_exit);

	glutMainLoop();
}

// TODO: Implement using GLUT
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

