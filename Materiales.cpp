#include "Materiales.h"

//Valores default del material en OpenGL
GLfloat gDefaultDiffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
GLfloat gDefaultAmbient[] = {0.2f, 0.2f, 0.2f, 1.0f};
GLfloat gDefaultEmission[]= {0.0f, 0.0f, 0.0f, 1.0f};
GLfloat gDefaultSpecular[]= {0.0f, 0.0f, 0.0f, 1.0f};
GLfloat gDefaultShininess = 0.0f;

//Total
GLfloat totalAmb[4] = {1.0f,1.0f,1.0f,1.0f};
GLfloat totalDif[4] = {1.0f,1.0f,1.0f,1.0f};
GLfloat totalSpe[4] = {1.0f,1.0f,1.0f,1.0f};
GLfloat totalShi = 128.0f;

//Nada
GLfloat nadaAmb[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat nadaDif[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat nadaSpe[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat nadaShi = 1.0f;

//Solo difuso
GLfloat difusionAmb[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat difusionDif[4] = {1.0f,1.0f,1.0f,1.0f};
GLfloat difusionSpe[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat difusionShi = 1.0f;

//Solo ambiente
GLfloat ambienteAmb[4] = {1.0f,1.0f,1.0f,1.0f};
GLfloat ambienteDif[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat ambienteSpe[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat ambienteShi = 1.0f;

//Solo especular
GLfloat especularAmb[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat especularDif[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat especularSpe[4] = {1.0f,1.0f,1.0f,1.0f};
GLfloat especularShi = 40.0f;

//Azul
GLfloat azulAmb[4] = {0.4f,0.6f,1.0f,1.0f};
GLfloat azulDif[4] = {0.4f,0.6f,1.0f,1.0f};
GLfloat azulSpe[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat azulShi = 0.0f;

//Verde
GLfloat verdeAmb[4] = {0.4f,0.6f,0.1f,1.0f};
GLfloat verdeDif[4] = {0.4f,0.6f,0.1f,1.0f};
GLfloat verdeSpe[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat verdeShi = 0.0f;

//Naranja
GLfloat naranjaAmb[4] = {1.0f,0.7f,0.2f,1.0f};
GLfloat naranjaDif[4] = {1.0f,0.7f,0.2f,1.0f};
GLfloat naranjaSpe[4] = {0.0f,0.0f,0.0f,1.0f};
GLfloat naranjaShi = 0.0f;

//Bronce
GLfloat bronceAmb[4] = {0.2125f,0.1275f,0.054f,1.0f};
GLfloat bronceDif[4] = {0.714f,0.4284f,0.18144f,1.0f};
GLfloat bronceSpe[4] = {0.393548f,0.271906f,0.166721f,1.0f};
GLfloat bronceShi = 25.6f;

//Oro
GLfloat oroAmb[4] = {0.24725f,0.2245f,0.0645f,1.0f};
GLfloat oroDif[4] = {1.0f,1.0f,1.0f,1.0f};
GLfloat oroSpe[4] = {0.797357f,0.723991f,0.208006f,1.0f};
GLfloat oroShi = 128.0f;

//Aluminio
GLfloat aluminioAmb[4] = {0.30f,0.30f,0.35f,1.0f};
GLfloat aluminioDif[4] = {0.30f,0.30f,0.50f,1.0f};
GLfloat aluminioSpe[4] = {0.70f,0.70f,0.80f,1.0f};
GLfloat aluminioShi = 9.0f;

//Cobre
GLfloat cobreAmb[4] = {0.33f,0.26f,0.23f,1.0f};
GLfloat cobreDif[4] = {0.50f,0.11f,0.0f,1.0f};
GLfloat cobreSpe[4] = {0.95f,0.73f,0.0f,1.0f};
GLfloat cobreShi = 93.0f;

//Plata
GLfloat plataAmb[4] = {0.19225f,0.19225f,0.19225f,1.0f};
GLfloat plataDif[4] = {0.50754f,0.50754f,0.50754f,1.0f};
GLfloat plataSpe[4] = {0.508273f,0.508273f,0.508273f,1.0f};
GLfloat plataShi = 51.2f;

//Esmeralda
GLfloat esmeraldaAmb[4] = {0.0215f,0.1745f,0.0215f,0.55f};
GLfloat esmeraldaDif[4] = {0.07568f,0.61424f,0.07568f,0.55f};
GLfloat esmeraldaSpe[4] = {0.633f,0.727811f,0.633f,0.55};
GLfloat esmeraldaShi = 76.8f;

CMateriales::CMateriales()
{
	
}

CMateriales::~CMateriales()
{
	
}

void CMateriales::SeleccionaMaterial(int tipo)
{
	if(tipo == 0)
	{
		//Material default de OpenGL
		glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, gDefaultDiffuse);
		glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, gDefaultAmbient);
		glMaterialfv( GL_FRONT_AND_BACK, GL_EMISSION,gDefaultEmission); 
		glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR,gDefaultSpecular);
		glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS,gDefaultShininess);
	}
	else if(tipo == 1)
	{
		//Total
		glMaterialfv(GL_FRONT, GL_AMBIENT,   totalAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE,   totalDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR,  totalSpe);
		glMaterialf (GL_FRONT, GL_SHININESS, totalShi);
	}
	else if(tipo == 2)
	{
		//Nada
		glMaterialfv(GL_FRONT, GL_AMBIENT,   nadaAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE,   nadaDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR,  nadaSpe);
		glMaterialf (GL_FRONT, GL_SHININESS, nadaShi);
	}
	else if(tipo == 3)
	{
		//Solo Difusion
		glMaterialfv(GL_FRONT, GL_AMBIENT,   difusionAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE,   difusionDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR,  difusionSpe);
		glMaterialf (GL_FRONT, GL_SHININESS, difusionShi);
	}
	else if(tipo == 4)
	{
		//Solo Ambiente
		glMaterialfv(GL_FRONT, GL_AMBIENT,   ambienteAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE,   ambienteDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR,  ambienteSpe);
		glMaterialf (GL_FRONT, GL_SHININESS, ambienteShi);
	}
	else if(tipo == 5)
	{
		//Solo Especular
		glMaterialfv(GL_FRONT, GL_AMBIENT, especularAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, especularDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR, especularSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, especularShi);
	}
	else if(tipo == 6)
	{
		//Azul
		glMaterialfv(GL_FRONT, GL_AMBIENT, azulAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, azulDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR, azulSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, azulShi);
	}
	else if(tipo == 7)
	{
		//Verde
	    glMaterialfv(GL_FRONT, GL_AMBIENT, verdeAmb);
	    glMaterialfv(GL_FRONT, GL_DIFFUSE, verdeDif);
	    glMaterialfv(GL_FRONT, GL_SPECULAR, verdeSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, verdeShi);
	}
	else if(tipo == 8)
	{
		//Naranja
		glMaterialfv(GL_FRONT, GL_AMBIENT, naranjaAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, naranjaDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR, naranjaSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, naranjaShi);
	}
	else if(tipo == 9)
	{
		//Bronce
		glMaterialfv(GL_FRONT, GL_AMBIENT, bronceAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, bronceDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR, bronceSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, bronceShi);
	}
	else if(tipo == 10)
	{
		//Oro
		glMaterialfv(GL_FRONT, GL_AMBIENT, oroAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, oroDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR, oroSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, oroShi);
	}
	else if(tipo == 11)
	{
		//Aluminio
		glMaterialfv(GL_FRONT, GL_AMBIENT, aluminioAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, aluminioDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR, aluminioSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, aluminioShi);
	}
	else if(tipo == 12)
	{
		//Cobre
		glMaterialfv(GL_FRONT, GL_AMBIENT, cobreAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, cobreDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR, cobreSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, cobreShi);
	}
	else if(tipo == 13)
	{
		//Plata
		glMaterialfv(GL_FRONT, GL_AMBIENT, plataAmb);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, plataDif);
		glMaterialfv(GL_FRONT, GL_SPECULAR, plataSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, plataShi);
	}
	else if(tipo == 14)
	{
		//Esmeralda
	    glMaterialfv(GL_FRONT, GL_AMBIENT, esmeraldaAmb);
	    glMaterialfv(GL_FRONT, GL_DIFFUSE, esmeraldaDif);
	    glMaterialfv(GL_FRONT, GL_SPECULAR, esmeraldaSpe);
		glMaterialf(GL_FRONT, GL_SHININESS, esmeraldaShi);
	}
	
}