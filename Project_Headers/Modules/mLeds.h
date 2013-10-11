/* 
------------------------------------------------------------ 
Copyright 2003-2007 Haute Ecole ARC Ingénierie,  
Switzerland. All rights reserved 
------------------------------------------------------------ 
Nom du fichier :  mLeds.h   
Auteur et Date :  Mueller Michael 11.10.2013
 
But : Pilotage de LEDs d'éclairage
 
Modifications 
Date    Faite  Ctrl    Description 
------------------------------------------------------------ 
*/ 

#ifndef __MLEDS__
#define __MLEDS__


void mLeds_Setup();
void mLeds_Open();
void mLeds_Close();

void mLeds_writeDyC(float aDyC);


#endif
