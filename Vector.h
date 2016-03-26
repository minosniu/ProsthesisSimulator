#ifndef VECTOR_H
#define VECTOR_H

#define PI 3.1415926535897932					// This is our famous PI

class Vector
{
	public:

		//Constructor
		Vector() {}

		// Este constructor permite inicializar los datos
		Vector(float X, float Y, float Z) 
		{ 
			x = X; y = Y; z = Z;
		}

		Vector operator+(Vector vVector)
		{
			//Regresa el vector resultante de la suma
			return Vector(vVector.x + x, vVector.y + y, vVector.z + z);
		}

		//Aqui se sobrecarga el operador - de modo que se puedan restar vectores
		Vector operator-(Vector vVector)
		{
			//Regresa el vector resultante de la resta
			return Vector(x - vVector.x, y - vVector.y, z - vVector.z);
		}
		
		//Aqui se sobrecarga el operador * de modo que se pueda multiplicar un vector por un escalar
		Vector operator*(float num)
		{
			//Regresa el vector escalado
			return Vector(x * num, y * num, z * num);
		}

		//Aqui se sobrecarga el operador / de modo que se pueda dividir un vector entre un escalar
		Vector operator/(float num)
		{
			//Regresa el vector escalado (reducido)
			return Vector(x / num, y / num, z / num);
		}

		float x, y, z;
		
};

//Esto regresa el valor absoluto de "num"
float Absoluto(float num);

//Esto regresa un vector perpendicular a 2 vectors dados, por medio del producto cruz.
Vector Cruz(Vector vVector1, Vector vVector2);

//Esto regresa la magnitud de una normal (o cualquier otro vector)
float Magnitud(Vector vNormal);

//Esto regresa un vector normalizado (Un vector de longitud 1)
Vector Normaliza(Vector vNormal);

#endif // VECTOR_H
