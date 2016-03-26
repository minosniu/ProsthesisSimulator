#ifndef TGALOADER_H
#define TGALOADER_H

#include "Main.h"	

class TGALoader
{
	public:
		TGALoader();
		~TGALoader(); 
		
		int LoadTGA(char *filename);
		int freeData();
		void Elimina();
		
		unsigned char *imageData;
		int bpp,width,height;

		GLuint texID;
	private:
		
};

#endif // TGALOADER_H
